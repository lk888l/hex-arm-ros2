#!/usr/bin/env python3
"""Measure transport traces or run an isolated ROS/Zenoh MOCK benchmark.

Inside the sourced ROS container:
  python3 scripts/benchmark-transport.py run --output /tmp/transport --duration 5 --warmup 1 --repetitions 1
  python3 scripts/benchmark-transport.py run --output /tmp/fault --scenario delay-recovery --duration 3 --warmup 1 --repetitions 1
  python3 scripts/benchmark-transport.py analyze /tmp/transport

Defaults are 30 s warmup, 300 s sampling and three independent runs. The run
fixture always passes --mock and the impossible-hardware mock profile. It loads
the installed C++ plugin with direct Zenoh, a 100 Hz control loop and Rust mock.
No automatic fixture opens CAN. Analyze also supports archived bridge traces.
"""
from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time
from collections import defaultdict


PAIRS = (
    ("generator_publish", "python_receive", "source"),
    ("cxx_write", "cxx_publish", "source"),
    ("cxx_write", "python_receive", "source"),
    ("python_receive", "zenoh_put", "seq"),
    ("python_receive", "rust_decode", "seq"),
    ("python_receive", "rust_accept", "seq"),
    ("rust_accept", "control_consume", "seq"),
    ("control_consume", "mailbox_write", "seq"),
    ("mailbox_write", "can_send_begin", "seq"),
    ("can_send_begin", "can_send", "seq"),
    ("python_receive", "can_send", "seq"),
    ("rust_state_publish", "state_receive", "seq"),
    ("state_receive", "python_state_publish", "seq"),
    ("python_state_publish", "cxx_state_receive", "source"),
    ("python_state_publish", "cxx_read", "source"),
    ("cxx_state_receive", "cxx_read", "source"),
    ("gate_wait_begin", "gate_wait_end", "span"),
    ("cxx_write", "direct_put", "seq"),
    ("direct_put", "rust_accept", "seq"),
    ("rust_state_publish", "direct_state_receive", "seq"),
    ("rust_state_publish", "direct_read", "seq"),
    ("direct_state_receive", "direct_read", "seq"),
    ("control_cycle", "control_cycle_end", "seq"),
)


def percentile(values, fraction):
    ordered = sorted(values)
    if not ordered:
        return None
    point = (len(ordered) - 1) * fraction
    lower = math.floor(point)
    return ordered[lower] + (ordered[math.ceil(point)] - ordered[lower]) * (point - lower)


def distribution(values):
    return {"count": len(values), **{name: percentile(values, q)
        for name, q in (("p50_ms", .5), ("p95_ms", .95), ("p99_ms", .99), ("max_ms", 1.))}}


def analyze_run(directory: Path, begin_ns=0, end_ns=2**63 - 1):
    rows = []
    dropped = skipped = 0
    files = sorted(directory.glob("*.csv"))
    for path in files:
        file_dropped = 0
        with path.open(newline="", encoding="utf-8") as source:
            for item in csv.DictReader(source):
                row = {name: int(item.get(name) or 0) for name in
                       ("timestamp_ns", "pid", "seq", "generation", "source_stamp_ns", "span_id")}
                row["stage"] = item["stage"]
                if row["stage"] == "trace_dropped":
                    file_dropped = max(file_dropped, row["seq"])
                elif begin_ns <= row["timestamp_ns"] <= end_ns:
                    skipped += row["stage"] == "cxx_skip"
                    rows.append(row)
        dropped += file_dropped
    rows.sort(key=lambda item: item["timestamp_ns"])
    stages = defaultdict(list)
    indices = defaultdict(dict)
    for row in rows:
        stages[row["stage"]].append(row)
        # Match one single run (one driver/bridge) by sequence. For repeated
        # control/CAN output, measure the first consumption/send of each command.
        for kind, key in (("source", row["source_stamp_ns"]), ("seq", row["seq"]),
                          ("span", (row["pid"], row["span_id"]) if row["span_id"] else 0)):
            if key:
                indices[(row["stage"], kind)].setdefault(key, row["timestamp_ns"])
    latencies = {}
    negative_pairs = {}
    for start, finish, kind in PAIRS:
        left, right = indices[(start, kind)], indices[(finish, kind)]
        common = left.keys() & right.keys()
        negative_pairs[start + "->" + finish] = sum(right[key] < left[key] for key in common)
        latencies[start + "->" + finish] = {**distribution([
            (right[key] - left[key]) / 1e6 for key in common if right[key] >= left[key]]),
            "unmatched_start_count": len(left.keys() - right.keys())}
    # C++ sequence and Python sequence are independent. Join them through the
    # preserved ROS source stamp, then match the first CAN send for that command.
    can_by_seq = indices[("can_send", "seq")]
    cxx_by_source = indices[("cxx_write", "source")]
    full_path = []
    matched_cxx_sources = set()
    for row in stages["python_receive"]:
        source_stamp = row["source_stamp_ns"]
        if source_stamp in matched_cxx_sources:
            continue
        start, finish = cxx_by_source.get(source_stamp), can_by_seq.get(row["seq"])
        if start is not None and finish is not None and finish >= start:
            full_path.append((finish-start)/1e6)
            matched_cxx_sources.add(source_stamp)
    latencies["cxx_write->can_send"] = {**distribution(full_path),
        "unmatched_start_count": len(cxx_by_source) - len(matched_cxx_sources)}
    direct = bool(stages["direct_put"])
    # A direct command preserves its write sequence through Protobuf.
    for finish in ("rust_accept", "can_send"):
        samples = []
        if direct:
            starts, ends = indices[("cxx_write", "seq")], indices[(finish, "seq")]
            samples = [(ends[key] - starts[key]) / 1e6 for key in starts.keys() & ends.keys()
                       if ends[key] >= starts[key]]
        else:
            ends = indices[(finish, "seq")]
            for row in stages["python_receive"]:
                start, end = cxx_by_source.get(row["source_stamp_ns"]), ends.get(row["seq"])
                if start is not None and end is not None and end >= start:
                    samples.append((end - start) / 1e6)
        latencies["cxx_write->" + finish] = {**distribution(samples),
            "unmatched_start_count": max(0, len(cxx_by_source) - len(samples))}
    # Feedback has independent Rust and ROS identities. A received DDS frame
    # may be replaced before read(), so keep delivery and consumption separate.
    feedback_matches = {}
    for start_stage in ("rust_state_publish", "state_receive"):
        for finish_stage in ("cxx_state_receive", "cxx_read"):
            feedback = []
            feedback_seen = set()
            starts = indices[(start_stage, "seq")]
            finishes = indices[(finish_stage, "source")]
            for row in stages["python_state_publish"]:
                if row["seq"] in feedback_seen:
                    continue
                start = starts.get(row["seq"])
                finish = finishes.get(row["source_stamp_ns"])
                if start is not None and finish is not None and finish >= start:
                    feedback.append((finish-start)/1e6)
                    feedback_seen.add(row["seq"])
            key = start_stage + "->" + finish_stage
            latencies[key] = {**distribution(feedback),
                              "unmatched_start_count": len(starts) - len(feedback_seen)}
            feedback_matches[key] = bool(feedback)
    # Include *every* read, including repeated consumption of the same sample.
    # First-consumption latency alone conceals feedback aging during a stall.
    published = indices[("rust_state_publish", "seq")]
    ros_to_seq = {row["source_stamp_ns"]: row["seq"] for row in stages["python_state_publish"]}
    feedback_ages = []
    for row in stages["direct_read"] if direct else stages["cxx_read"]:
        seq = row["seq"] if direct else ros_to_seq.get(row["source_stamp_ns"])
        origin = published.get(seq)
        if origin is not None and row["timestamp_ns"] >= origin:
            feedback_ages.append((row["timestamp_ns"] - origin) / 1e6)
    latencies["feedback_age_at_read"] = distribution(feedback_ages)
    spacing = {}
    for stage, items in sorted(stages.items()):
        # Per-process spacing avoids a merged stream hiding a stall in one owner.
        process_rows = defaultdict(list)
        for row in items:
            process_rows[row["pid"]].append(row["timestamp_ns"])
        gaps = [(b-a)/1e6 for timestamps in process_rows.values()
                for a, b in zip(timestamps, timestamps[1:])]
        boundary_gaps = []
        if begin_ns and end_ns < 2**63 - 1:
            boundary_gaps = [(timestamps[0] - begin_ns)/1e6
                             for timestamps in process_rows.values()]
            boundary_gaps += [(end_ns - timestamps[-1])/1e6
                              for timestamps in process_rows.values()]
        spacing[stage] = {**distribution(gaps), "samples": len(items),
                         "max_silent_interval_ms": max(gaps + boundary_gaps, default=None)}
    cxx_present = bool(stages["cxx_write"])
    can_present = bool(stages["can_send"])
    cycles = [row["timestamp_ns"] for row in stages["control_cycle"]]
    cycle_jitter = distribution([abs((b - a) / 1e6 - 10.) for a, b in zip(cycles, cycles[1:])])
    return {"files": len(files), "clock": "Linux CLOCK_MONOTONIC; ROS stamps are keys only",
            "coverage": {"cxx": cxx_present, "python": bool(stages["python_receive"]),
                         "rust": bool(stages["rust_accept"]), "can": can_present,
                         "end_to_end": bool(latencies["cxx_write->can_send"]["count"]),
                         "direct_zenoh": direct,
                         "feedback_end_to_end": (bool(stages["direct_state_receive"]) if direct else
                                                 feedback_matches["rust_state_publish->cxx_state_receive"]),
                         "feedback_received": (bool(stages["direct_state_receive"]) if direct else
                                               feedback_matches["rust_state_publish->cxx_state_receive"]),
                         "feedback_consumed": bool(feedback_ages)},
            "latencies": latencies, "stage_spacing": spacing,
            "control_cycle_absolute_jitter": cycle_jitter,
            "command_publish_skipped": skipped, "trace_records_dropped": dropped,
            "negative_pairs": {key: value for key, value in negative_pairs.items() if value}}


def analyze(directory: Path):
    manifests = sorted(directory.rglob("manifest.json"))
    if manifests:
        runs = []
        for path in manifests:
            metadata = json.loads(path.read_text())
            runs.append({"directory": str(path.parent), "metadata": metadata,
                         "metrics": analyze_run(path.parent, metadata.get("sample_begin_ns", 0),
                                                metadata.get("sample_end_ns", 2**63-1))})
        return {"runs": runs}
    return {"runs": [{"directory": str(directory), "metrics": analyze_run(directory)}]}


def stop_process(process):
    if process is None:
        return
    for stop_signal, timeout in ((signal.SIGINT, 4.), (signal.SIGTERM, 2.), (signal.SIGKILL, 2.)):
        if process.poll() is not None:
            return
        os.killpg(process.pid, stop_signal)
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            pass


def load_fixture(workspace):
    spec = importlib.util.spec_from_file_location("transport_smoke",
        workspace / "src/hex_arm_bringup/test/test_protocol_smoke.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class TcpFaultProxy:
    """Local test-only TCP cut; the Rust process and control loop keep running."""
    def __init__(self, endpoint):
        if not endpoint.startswith("tcp/127.0.0.1:"):
            raise ValueError("disconnect fixture requires a loopback TCP endpoint")
        self.target = ("127.0.0.1", int(endpoint.rsplit(":", 1)[1]))
        self.lock = threading.Lock()
        self.connections = set()
        self.paused = threading.Event()
        self.closed = threading.Event()
        self.listener = socket.socket()
        self.listener.bind(("127.0.0.1", 0))
        self.listener.listen()
        self.listener.settimeout(.1)
        self.endpoint = f"tcp/127.0.0.1:{self.listener.getsockname()[1]}"
        self.worker = threading.Thread(target=self.accept, daemon=True)
        self.worker.start()

    def accept(self):
        while not self.closed.is_set():
            try:
                client, _ = self.listener.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            try:
                if self.paused.is_set():
                    client.close()
                    continue
                server = socket.create_connection(self.target, timeout=.2)
                server.settimeout(None)
            except OSError:
                client.close()
                continue
            with self.lock:
                self.connections.update((client, server))
            for source, destination in ((client, server), (server, client)):
                threading.Thread(target=self.forward, args=(source, destination), daemon=True).start()

    def forward(self, source, destination):
        try:
            while not self.closed.is_set() and not self.paused.is_set():
                data = source.recv(65536)
                if not data:
                    break
                destination.sendall(data)
        except OSError:
            pass
        finally:
            for connection in (source, destination):
                try:
                    connection.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                connection.close()
                with self.lock:
                    self.connections.discard(connection)

    def cut(self):
        self.paused.set()
        with self.lock:
            for connection in self.connections:
                try:
                    connection.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass

    def close(self):
        self.closed.set()
        self.cut()
        self.listener.close()
        self.worker.join(timeout=.5)


def run_plugin_once(args, directory):
    """Actual plugin -> direct Zenoh -> Rust mock, with a 100 Hz C++ loop."""
    import rclpy
    from hex_arm_msgs.srv import DiscoverMotors
    fixture = load_fixture(args.workspace)
    directory.mkdir(parents=True, exist_ok=False)
    environment = dict(os.environ, HEX_ARM_TRACE_DIR=str(directory.resolve()),
                       ROS_DOMAIN_ID=str(args.domain_id))
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    processes, logs, workers = [], [], []
    probe = plugin = controller = proxy = None
    endpoint = args.endpoint
    metadata = {"backend": "zenoh", "scenario": args.scenario,
                "fixture": "installed C++ plugin + Rust mock; CAN absent",
                "warmup_sec": args.warmup, "duration_sec": args.duration, "passed": False}
    driver_command = [str(args.workspace / "install/hex_arm_controller/lib/hex_arm_controller/hex_arm_controller"),
                      "--profile", str(args.workspace / "src/hex_arm_controller/test/firefly_y6.mock.yaml"),
                      "--mock", "--zenoh-listen", args.endpoint]

    def launch(name, command):
        log = (directory / (name + ".log")).open("w")
        logs.append(log)
        process = subprocess.Popen(command, env=environment, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        processes.append(process)
        return process

    try:
        controller = launch("controller", driver_command)
        if args.scenario == "disconnect":
            proxy = TcpFaultProxy(args.endpoint)
            endpoint = proxy.endpoint
        rclpy.init()
        probe = fixture.TransportProbe()
        plugin = launch("plugin", [str(args.workspace / "install/hex_arm_hardware/lib/hex_arm_hardware/transport_probe"),
            endpoint, fixture.PREFIX, str(args.duration), str(args.warmup), args.scenario])
        deadline = time.monotonic() + 25.
        while time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=.01)
            output = (directory / "plugin.log").read_text()
            ready = [line for line in output.splitlines() if line.startswith("PLUGIN_ACTIVE_NS=")]
            if ready:
                active_ns = int(ready[0].split("=")[1])
                break
            if plugin.poll() is not None:
                raise RuntimeError("plugin failed to activate: " + output[-3000:])
        else:
            raise RuntimeError("plugin activation deadline exceeded")
        metadata["sample_begin_ns"] = active_ns + int(args.warmup * 1e9)
        metadata["sample_end_ns"] = metadata["sample_begin_ns"] + int(args.duration * 1e9)
        inject_ns = metadata["sample_begin_ns"] + int(args.duration * .5e9)
        if args.scenario == "cpu":
            for _ in range(args.cpu_workers):
                workers.append(launch("cpu-" + str(len(workers)), [sys.executable, "-c", "while True: pass"]))
        if args.scenario == "diagnostic":
            from diagnostic_msgs.msg import DiagnosticArray
            for _ in range(8):
                probe.create_subscription(DiagnosticArray, "/diagnostics", lambda _: None, 10)
        manager = (probe.create_client(DiscoverMotors, "/hex_arm/discover_motors")
                   if args.scenario == "management" else None)
        next_request, pending, injected = time.monotonic(), None, False
        deadline = time.monotonic() + args.warmup + args.duration + 15.
        while plugin.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=.005)
            if manager is not None and time.monotonic() >= next_request and (pending is None or pending.done()):
                if pending is not None and not pending.result().success:
                    raise RuntimeError("cached discovery failed")
                pending = manager.call_async(DiscoverMotors.Request())
                next_request = time.monotonic() + .1
            if not injected and time.monotonic_ns() >= inject_ns:
                injected = True
                if args.scenario in ("stopped-command", "delay-recovery", "disconnect", "restart"):
                    metadata["fault_injected_ns"] = inject_ns
                if args.scenario == "disconnect":
                    proxy.cut()
                    try:
                        until = time.monotonic() + .3
                        while time.monotonic() < until:
                            rclpy.spin_once(probe, timeout_sec=.005)
                    finally:
                        proxy.paused.clear()
                elif args.scenario == "restart":
                    stop_process(controller)
                    controller = launch("controller-restarted", driver_command)
        if plugin.poll() is None:
            raise RuntimeError("plugin did not exit within bounded shutdown window")
        if plugin.returncode != 0:
            raise RuntimeError("plugin failed: " + (directory / "plugin.log").read_text()[-4000:])
        metadata["publications_received"] = dict(probe.received)
        if not all(probe.received.values()):
            raise RuntimeError("direct diagnostic publications missing: " + str(probe.received))
        metadata["final_fault_latched"] = bool(probe.driver and probe.driver.fault_latched)
        metadata["final_fault_code"] = probe.driver.fault_code if probe.driver else None
        if args.scenario in ("stopped-command", "delay-recovery", "reactivate"):
            if not probe.driver or not probe.driver.fault_latched or probe.driver.fault_code != 0x1003:
                raise RuntimeError("stopped writer did not trigger Rust watchdog")
        metadata["passed"] = True
    except BaseException as error:
        metadata["error"] = str(error)
        raise
    finally:
        if controller is not None and controller.poll() is None:
            os.killpg(controller.pid, signal.SIGCONT)
        for process in reversed(processes):
            stop_process(process)
        if proxy is not None:
            proxy.close()
        if probe is not None:
            probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for log in logs:
            log.close()
        (directory / "manifest.json").write_text(json.dumps(metadata, indent=2) + "\n")
    if args.scenario == "reactivate":
        marker = next(line for line in (directory / "plugin.log").read_text().splitlines()
                      if line.startswith("PLUGIN_REACTIVATED_NS="))
        metadata["fault_injected_ns"] = int(marker.split("=")[1])
        metadata["reactivated_ns"] = metadata["fault_injected_ns"]
    if "fault_injected_ns" in metadata:
        grace_ns = 0 if args.scenario == "reactivate" else 150_000_000
        after = analyze_run(directory, metadata["fault_injected_ns"] + grace_ns,
                            metadata["sample_end_ns"])
        accepted = after["stage_spacing"].get("rust_accept", {}).get("samples", 0)
        metadata["accepted_commands_after_fault"] = accepted
        metadata["passed"] = accepted == 0
        (directory / "manifest.json").write_text(json.dumps(metadata, indent=2) + "\n")
        if accepted:
            raise RuntimeError("commands resumed after fault without explicit reactivation")


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be finite and positive")
    return number


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="operation", required=True)
    inspect = subparsers.add_parser("analyze", help="analyze CSVs without ROS dependencies")
    inspect.add_argument("directory", type=Path)
    inspect.add_argument("--output", type=Path)
    run = subparsers.add_parser("run", help="start isolated MOCK processes and workload")
    run.add_argument("--workspace", type=Path, default=Path(__file__).resolve().parents[1])
    run.add_argument("--output", type=Path, required=True)
    run.add_argument("--scenario", choices=("baseline", "trajectory", "diagnostic", "management", "cpu", "stopped-command", "delay-recovery", "disconnect", "restart", "reactivate", "reactivate-stream"), default="baseline")
    run.add_argument("--warmup", type=float, default=30.)
    run.add_argument("--duration", type=positive, default=300.)
    run.add_argument("--repetitions", type=int, default=3)
    run.add_argument("--cpu-workers", type=int, default=2)
    run.add_argument("--domain-id", type=int, default=73)
    run.add_argument("--endpoint", default="tcp/127.0.0.1:7459")
    args = parser.parse_args()
    if args.operation == "run":
        if not math.isfinite(args.warmup) or args.warmup < 0 or args.repetitions < 1 or args.cpu_workers < 1:
            parser.error("warmup must be nonnegative; repetitions and CPU workers must be positive")
        if args.scenario in ("stopped-command", "delay-recovery") and args.duration < 1.0:
            parser.error("fault scenarios require duration >= 1 second")
        for repetition in range(1, args.repetitions + 1):
            run_plugin_once(args, args.output / f"{args.scenario}-{repetition}")
        report = analyze(args.output)
        destination = args.output / "report.json"
    else:
        report = analyze(args.directory)
        destination = args.output
    if destination:
        destination.write_text(json.dumps(report, indent=2) + "\n")
        print(destination)
    else:
        print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
