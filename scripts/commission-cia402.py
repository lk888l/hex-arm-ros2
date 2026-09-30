#!/usr/bin/env python3
"""Run a staged single-axis CiA402 campaign and retain independent shutdown evidence.

Run inside the ROS workspace environment, with no other CAN controller running.
The first displacement is at most 0.03 rad; later magnitudes grow by at most 2x.
Each test returns to its measured starting position. This does not calibrate a
profile or certify mechanical clearance, full travel, or multi-axis operation.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import signal
import shutil
import subprocess
import sys

import yaml


ROOT = Path(__file__).resolve().parents[1]
ANSI = re.compile(r"\x1b\[[0-9;]*m")


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate_steps(deltas, duration, allow_hold=False):
    if not deltas or not math.isfinite(duration) or not 0 < duration <= 30:
        raise ValueError("nonempty steps and finite duration in (0,30] required")
    if allow_hold and deltas == [0.0]:
        return
    previous = 0.015
    for delta in deltas:
        if not math.isfinite(delta) or not 0.0001 <= abs(delta) <= 0.25:
            raise ValueError("each delta magnitude must be in [0.0001,0.25] rad")
        if abs(delta) > previous * 2 + 1e-9:
            raise ValueError("start at <=0.03 rad and grow magnitudes by at most 2x")
        previous = abs(delta)


def verify_disabled(report):
    nodes = report.get("nodes", [])
    if not report.get("passed") or len(nodes) != 6 or report.get("errors"):
        raise RuntimeError("independent six-axis state/identity read failed")
    if len({node.get("node") for node in nodes}) != 6:
        raise RuntimeError("independent read contains duplicate/missing node IDs")
    for node in nodes:
        if (not node.get("disabled") or node.get("error_code") != 0
                or node.get("heartbeat_consumer") != 0):
            raise RuntimeError(f"node {node.get('node')}: disable/heartbeat release unconfirmed")
        span = node.get("span_rad", float("nan"))
        if not math.isfinite(span) or not 0 <= span <= 0.001:
            raise RuntimeError(f"node {node.get('node')}: arm is not stationary")


def run_process(command, log, timeout):
    """Forward interruption/timeout to the controller's confirmed cleanup path."""
    with log.open("x") as output:
        child = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT)
        interrupted = None
        def stop(signum, _frame):
            nonlocal interrupted
            interrupted = signum
            child.send_signal(signum)
        old = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM)}
        try:
            try:
                rc = child.wait(timeout=timeout)
                return 128 + interrupted if interrupted is not None else rc
            except subprocess.TimeoutExpired:
                child.terminate()
                # Never start another test while this controller still owns CAN.
                child.wait(timeout=20)
                return 124
        finally:
            for sig, handler in old.items():
                signal.signal(sig, handler)


def metrics_from_log(log):
    text = ANSI.sub("", log.read_text())
    metrics = {}
    for key in ("measured_peak_delta_rad", "return_error_rad", "max_tracking_error_rad",
                "max_velocity_rad_s", "max_torque_nm", "samples", "start_position_rad",
                "final_position_rad", "most_positive_delta_rad", "most_negative_delta_rad"):
        values = re.findall(rf"\b{key}=([-+0-9.eE]+)", text)
        if values:
            value = float(values[-1])
            if math.isfinite(value):
                metrics[key] = value
    return metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", required=True, type=Path)
    parser.add_argument("--axis", required=True, choices=[f"joint_{i}" for i in range(1, 7)])
    parser.add_argument("--support-axes", nargs="+", default=[], choices=[f"joint_{i}" for i in range(1, 7)])
    parser.add_argument("--prepare-supports", nargs="+", default=[], metavar="joint_N:delta_rad")
    parser.add_argument("--max-temperature-rise-c", type=float, default=2.0,
                        help="Explicit supported-test temperature rise limit, 2 to 10 C")
    parser.add_argument("--deltas-rad", required=True, type=float, nargs="+")
    parser.add_argument("--duration-sec", type=float, default=20)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--binary", type=Path, default=ROOT / "install/hex_arm_controller/lib/hex_arm_controller/hex_arm_commission")
    parser.add_argument("--allow-motion", action="store_true")
    args = parser.parse_args()
    try:
        validate_steps(args.deltas_rad, args.duration_sec, allow_hold=bool(args.support_axes))
        if (not math.isfinite(args.max_temperature_rise_c) or not 2 <= args.max_temperature_rise_c <= 10
                or (not args.support_axes and args.max_temperature_rise_c != 2)):
            raise ValueError("temperature rise authority requires named supports and a limit in [2,10] C")
        if args.axis in args.support_axes or len(set(args.support_axes)) != len(args.support_axes):
            raise ValueError("support axes must be distinct from each other and the moving axis")
        if args.prepare_supports and (not args.support_axes or len(args.prepare_supports) > 2):
            raise ValueError("preparation requires named supports and at most two steps")
        profile = yaml.safe_load(args.profile.read_text())
        if (profile.get("schema_version") != 3 or profile.get("joint_coordinate_version") != 2
                or profile["bus"].get("protocol") != "cia402"
                or profile["bus"].get("transport") != "socket_can"):
            raise ValueError("requires schema v3 / coordinates v2 / SocketCAN CiA402")
        if not args.allow_motion:
            raise ValueError("--allow-motion is required for physical commissioning")
        binary = args.binary.resolve(strict=True)
        args.output_dir.mkdir(parents=True, exist_ok=False)
    except (OSError, ValueError, KeyError) as error:
        parser.error(str(error))
    report = {"started_utc": datetime.now(timezone.utc).isoformat(),
              "profile": str(args.profile), "profile_sha256": sha256(args.profile),
              "binary_sha256": sha256(binary), "axis": args.axis, "support_axes": args.support_axes,
              "prepare_supports": args.prepare_supports,
              "max_temperature_rise_c": args.max_temperature_rise_c,
              "trials": [], "passed": False, "error": None}
    shutil.copyfile(args.profile, args.output_dir / "profile.local.yaml")
    report["runner_sha256"] = sha256(__file__)
    report["state_reader_sha256"] = sha256(ROOT / "scripts/read-cia402-state.py")
    def save():
        (args.output_dir / "result.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    def read_state(stem, owner, key):
        state_path = args.output_dir / f"{stem}.json"
        rc = run_process([sys.executable, str(ROOT / "scripts/read-cia402-state.py"),
                          "--interface", profile["bus"]["interface"], "--profile", str(args.profile),
                          "--samples", "3", "--output", str(state_path)],
                         args.output_dir / f"{stem}.log", 45)
        state = json.loads(state_path.read_text())
        # Preserve the failed observation too: loss of stationarity after
        # disabling is evidence, not grounds to drop the final capture.
        owner[key] = state
        try:
            verify_disabled(state)
            if rc != 0:
                raise RuntimeError("independent read exited unsuccessfully")
        except RuntimeError as error:
            owner[f"{key}_error"] = str(error)
            raise
        return state
    save()
    try:
        read_state("initial-state", report, "initial_state")
        for index, delta in enumerate(args.deltas_rad):
            if sha256(args.profile) != report["profile_sha256"] or sha256(binary) != report["binary_sha256"]:
                raise RuntimeError("profile or executable changed during campaign")
            stem = f"{index:02d}"
            log = args.output_dir / f"{stem}-motion.log"
            command = [str(binary), "--profile", str(args.profile), "--commission-axis", args.axis,
                       "--delta-rad", str(delta), "--duration-sec", str(args.duration_sec),
                       "--allow-motion", "--allow-expanded-motion"]
            if args.support_axes:
                command.extend(["--max-temperature-rise-c", str(args.max_temperature_rise_c)])
                command.extend(["--support-axes", *args.support_axes])
            if args.prepare_supports:
                command.extend(["--prepare-supports", *args.prepare_supports])
            trial = {"delta_rad": delta, "duration_sec": args.duration_sec,
                     "command": command, "passed": False}
            report["trials"].append(trial)
            save()
            trial["exit_code"] = run_process(command, log, 90 + 50 * len(args.prepare_supports))
            trial["log_sha256"] = sha256(log)
            trial["metrics"] = metrics_from_log(log)
            read_state(f"{stem}-disabled", trial, "final_state")
            trial["passed"] = trial["exit_code"] == 0
            save()
            print(f"{args.axis} delta={delta:+.3f}: {'PASS' if trial['passed'] else 'FAIL'}", flush=True)
            if not trial["passed"]:
                raise RuntimeError("motion test failed; all later steps withheld")
        report["passed"] = True
    except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as error:
        report["error"] = str(error)
        print(str(error), file=sys.stderr)
    finally:
        report["finished_utc"] = datetime.now(timezone.utc).isoformat()
        save()
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
