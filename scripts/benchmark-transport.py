#!/usr/bin/env python3
"""Measure transport traces or run an isolated ROS/Zenoh MOCK benchmark.

Inside the sourced ROS container:
  python3 scripts/benchmark-transport.py run --output /tmp/transport --duration 5 --warmup 1 --repetitions 1
  python3 scripts/benchmark-transport.py run --output /tmp/fault --scenario delay-recovery --duration 3 --warmup 1 --repetitions 1
  python3 scripts/benchmark-transport.py analyze /tmp/transport

Defaults are 30 s warmup, 300 s sampling and three independent runs. The run
fixture always passes --mock and the impossible-hardware mock profile. It
exercises the Python bridge and Rust controller; it does not claim C++ or CAN
coverage. Analyze can also consume traces captured from a complete stack.
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
import subprocess
import sys
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
    return {"files": len(files), "clock": "Linux CLOCK_MONOTONIC; ROS stamps are keys only",
            "coverage": {"cxx": cxx_present, "python": bool(stages["python_receive"]),
                         "rust": bool(stages["rust_accept"]), "can": can_present,
                         "end_to_end": bool(full_path),
                         "feedback_end_to_end": feedback_matches["rust_state_publish->cxx_state_receive"],
                         "feedback_received": feedback_matches["rust_state_publish->cxx_state_receive"],
                         "feedback_consumed": feedback_matches["rust_state_publish->cxx_read"]},
            "latencies": latencies, "stage_spacing": spacing,
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


def run_once(args, directory):
    import rclpy
    from hex_arm_msgs.srv import DiscoverMotors
    from lifecycle_msgs.msg import Transition
    from std_srvs.srv import Trigger
    fixture = load_fixture(args.workspace)
    directory.mkdir(parents=True, exist_ok=False)
    environment = dict(os.environ, HEX_ARM_TRACE_DIR=str(directory.resolve()))
    # A unique ROS domain prevents accidental association with another stack.
    environment["ROS_DOMAIN_ID"] = str(args.domain_id)
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    controller = bridge = probe = None
    workers = []
    logs = []
    # Use the same bounded asynchronous trace writer as the bridge so the
    # workload generator does not add periodic filesystem I/O to the baseline.
    trace_spec = importlib.util.spec_from_file_location("benchmark_trace",
        args.workspace / "src/hex_arm_bridge/hex_arm_bridge/trace.py")
    trace_module = importlib.util.module_from_spec(trace_spec)
    trace_spec.loader.exec_module(trace_module)
    os.environ["HEX_ARM_TRACE_DIR"] = str(directory.resolve())
    source_trace = trace_module.TransportTrace()
    metadata = {"scenario": args.scenario, "repetition": directory.name,
                "fixture": "mock driver + Python bridge; C++ plugin/CAN absent",
                "warmup_sec": args.warmup, "duration_sec": args.duration,
                "fault_gap_sec": args.fault_gap, "passed": False}
    try:
        for name, command in (
            ("controller", [str(args.workspace / "install/hex_arm_controller/lib/hex_arm_controller/hex_arm_controller"),
                            "--profile", str(args.workspace / "src/hex_arm_controller/test/firefly_y6.mock.yaml"),
                            "--mock", "--zenoh-listen", args.endpoint]),
            ("bridge", [str(args.workspace / "install/hex_arm_bridge/lib/hex_arm_bridge/hex_arm_bridge"),
                        "--ros-args", "-p", f"robot_prefix:={fixture.PREFIX}",
                        "-p", f"zenoh_connect:={args.endpoint}"])):
            log = (directory / (name + ".log")).open("w")
            logs.append(log)
            process = subprocess.Popen(command, env=environment, stdout=log,
                                       stderr=subprocess.STDOUT, start_new_session=True)
            if name == "controller":
                controller = process
            else:
                bridge = process
        rclpy.init()
        probe = fixture.BridgeProbe()
        if not probe.change_state.wait_for_service(timeout_sec=15.):
            raise RuntimeError("mock lifecycle bridge unavailable")
        probe.transition(Transition.TRANSITION_CONFIGURE)
        probe.transition(Transition.TRANSITION_ACTIVATE)
        deadline = time.monotonic() + 5.
        while (probe.hold is None or probe.driver is None or
               probe.publisher.get_subscription_count() == 0) and time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=.01)
        if probe.hold is None or probe.driver is None:
            raise RuntimeError("mock six-joint state unavailable")
        # Replace only the fixture's source timer; the actual bridge/Rust paths run unchanged.
        probe.command_timer.cancel()
        reference = list(probe.hold.position)
        moving = args.scenario == "trajectory"
        started_at = time.monotonic()
        def publish():
            if moving:
                phase = (time.monotonic() - started_at) * math.pi / 4
                probe.hold.position[0] = reference[0] + .03 * math.sin(phase)
                probe.hold.velocity[0] = .03 * math.pi / 4 * math.cos(phase)
            probe.hold.header.stamp = probe.get_clock().now().to_msg()
            source_stamp = probe.hold.header.stamp.sec * 1_000_000_000 + probe.hold.header.stamp.nanosec
            source_trace.emit("generator_publish", source_stamp_ns=source_stamp)
            probe.publisher.publish(probe.hold)
        probe.command_timer = probe.create_timer(.01, publish)
        if not probe.activate.wait_for_service(timeout_sec=5.):
            raise RuntimeError("mock activate service unavailable")
        result = fixture._spin_future(probe, probe.activate.call_async(Trigger.Request()), 10.)
        if not result.success:
            raise RuntimeError("mock enable failed: " + result.message)
        if args.scenario == "diagnostic":
            from diagnostic_msgs.msg import DiagnosticArray
            from sensor_msgs.msg import JointState
            for _ in range(8):
                probe.create_subscription(DiagnosticArray, "/diagnostics", lambda _: None, 10)
                probe.create_subscription(JointState, "/hex_arm/internal/state", lambda _: None,
                    fixture.QoSProfile(depth=1, reliability=fixture.ReliabilityPolicy.BEST_EFFORT))
        if args.scenario == "cpu":
            for _ in range(args.cpu_workers):
                workers.append(subprocess.Popen([sys.executable, "-c", "while True: pass"],
                                               start_new_session=True,
                                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        manager = None
        pending = None
        next_request = time.monotonic()
        if args.scenario == "management":
            manager = probe.create_client(DiscoverMotors, "/hex_arm/discover_motors")
        warmup_end = time.monotonic() + args.warmup
        while time.monotonic() < warmup_end:
            rclpy.spin_once(probe, timeout_sec=.005)
            if probe.driver.fault_latched:
                raise RuntimeError("mock watchdog/fault during warmup: " + probe.driver.fault_reason)
        metadata["sample_begin_ns"] = time.monotonic_ns()
        sample_end = time.monotonic() + args.duration
        inject_at = time.monotonic() + args.duration / 2.
        injected = False
        while time.monotonic() < sample_end:
            rclpy.spin_once(probe, timeout_sec=.005)
            if controller.poll() is not None or bridge.poll() is not None:
                raise RuntimeError("mock process exited during measurement")
            if manager is not None and time.monotonic() >= next_request and (pending is None or pending.done()):
                if pending is not None:
                    response = pending.result()
                    if not response.success:
                        raise RuntimeError("cached discovery rejected: " + response.message)
                request = DiscoverMotors.Request()
                request.refresh = False
                pending = manager.call_async(request)
                next_request = time.monotonic() + .1
            if args.scenario in ("stopped-command", "delay-recovery") and not injected and time.monotonic() >= inject_at:
                injected = True
                old_stamp = probe.hold.header.stamp
                probe.command_timer.cancel()
                metadata["fault_injected_ns"] = time.monotonic_ns()
                if args.scenario == "delay-recovery":
                    wait_until = time.monotonic() + args.fault_gap
                    while time.monotonic() < wait_until:
                        rclpy.spin_once(probe, timeout_sec=.005)
                    probe.hold.header.stamp = old_stamp
                    probe.publisher.publish(probe.hold)
                    metadata["old_command_replayed_ns"] = time.monotonic_ns()
                    probe.command_timer.reset()
            fault_scenario = args.scenario in ("stopped-command", "delay-recovery")
            if not fault_scenario and probe.driver.fault_latched:
                raise RuntimeError("mock watchdog/fault during sampling: " + probe.driver.fault_reason)
        metadata["sample_end_ns"] = time.monotonic_ns()
        metadata["final_fault_latched"] = probe.driver.fault_latched
        metadata["final_fault_code"] = probe.driver.fault_code
        if args.scenario == "stopped-command" or (args.scenario == "delay-recovery" and args.fault_gap >= .15):
            if not probe.driver.fault_latched or probe.driver.fault_code != 0x1003:
                raise RuntimeError("command outage did not retain the fail-closed watchdog fault")
        if args.scenario == "delay-recovery" and args.fault_gap < .1:
            metadata["interpretation"] = "Short-gap stale source stamps are observational only; no source-stamp freshness contract exists."
        result = fixture._spin_future(probe, probe.deactivate.call_async(Trigger.Request()), 10.)
        if not result.success:
            raise RuntimeError("mock disable failed: " + result.message)
        metadata["passed"] = True
    except BaseException as error:
        metadata["error"] = str(error)
        raise
    finally:
        metadata.setdefault("sample_end_ns", time.monotonic_ns())
        if probe is not None:
            probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for worker in workers:
            stop_process(worker)
        stop_process(bridge)
        stop_process(controller)
        for log in logs:
            log.close()
        source_trace.close()
        unsafe_replay = False
        if (metadata["passed"] and args.scenario == "delay-recovery" and args.fault_gap >= .15):
            after_replay = analyze_run(directory, begin_ns=metadata["old_command_replayed_ns"],
                                       end_ns=metadata["sample_end_ns"])
            metadata["accepted_commands_after_replay"] = after_replay["stage_spacing"].get(
                "rust_accept", {}).get("samples", 0)
            unsafe_replay = metadata["accepted_commands_after_replay"] != 0
            if unsafe_replay:
                metadata["passed"] = False
                metadata["error"] = "commands were accepted after the timed-out source resumed"
        (directory / "manifest.json").write_text(json.dumps(metadata, indent=2) + "\n")
        if unsafe_replay:
            raise RuntimeError(metadata["error"])


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
    run.add_argument("--scenario", choices=("baseline", "trajectory", "diagnostic", "management", "cpu", "stopped-command", "delay-recovery"), default="baseline")
    run.add_argument("--warmup", type=float, default=30.)
    run.add_argument("--duration", type=positive, default=300.)
    run.add_argument("--repetitions", type=int, default=3)
    run.add_argument("--cpu-workers", type=int, default=2)
    run.add_argument("--fault-gap", type=positive, default=.25)
    run.add_argument("--domain-id", type=int, default=73)
    run.add_argument("--endpoint", default="tcp/127.0.0.1:7459")
    args = parser.parse_args()
    if args.operation == "run":
        if not math.isfinite(args.warmup) or args.warmup < 0 or args.repetitions < 1 or args.cpu_workers < 1:
            parser.error("warmup must be nonnegative; repetitions and CPU workers must be positive")
        if args.scenario in ("stopped-command", "delay-recovery") and args.duration < max(1., 2*args.fault_gap + .5):
            parser.error("fault scenarios require duration >= max(1, 2*fault-gap + 0.5) seconds")
        for repetition in range(1, args.repetitions + 1):
            run_once(args, args.output / f"{args.scenario}-{repetition}")
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
