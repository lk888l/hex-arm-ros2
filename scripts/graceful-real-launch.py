#!/usr/bin/env python3
"""Own ROS launch until return-to-ready and driver damping have completed."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import yaml


def eligible(profile, startup_report):
    try:
        report = json.loads(Path(startup_report).read_text())
        qualified_stop = (profile.get("controller", {}).get("shutdown_damping") is not None
                          or (profile.get("bus", {}).get("protocol") == "meow"
                              and bool(report.get("steps")))
                          or (profile.get("bus", {}).get("protocol") == "cia402"
                              and "sequence" in report and "measured_hold" in report))
        return (qualified_stop
                and report.get("passed") is True and "ready_hold" in report
                and not report.get("deactivated", False))
    except (OSError, ValueError):
        return False


def stop_command(profile, profile_path, startup_report, output):
    folded_meow = (profile.get("bus", {}).get("protocol") == "meow"
                   and profile.get("controller", {}).get("shutdown_damping") is None)
    script = "commission-meow-ros.py" if folded_meow else "commission-shutdown-ros.py"
    command = [sys.executable, str(Path(__file__).with_name(script)),
               "--profile", str(profile_path), "--startup-report", str(startup_report),
               "--output", str(output)]
    if folded_meow:
        command.append("--allow-motion")
    return command


def signal_group(process, sig):
    if process is not None:
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            pass


def stop_child(process, signal_initial=True):
    if process is None:
        return
    if signal_initial and process.poll() is None:
        signal_group(process, signal.SIGINT)
    try:
        process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        signal_group(process, signal.SIGTERM)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            signal_group(process, signal.SIGKILL)
            process.wait(timeout=5)
    # A dead launch leader must not leave its group running.
    signal_group(process, signal.SIGTERM)


def record_run_manifest(profile, run_dir, scope, command):
    # Synthetic non-ROS test processes do not have an installed robot model.
    if Path(command[0]).name != "ros2" or command[1:2] != ["launch"]:
        return
    from ament_index_python.packages import get_package_prefix
    prefix = get_package_prefix("hex_arm_controller")
    recorder = Path(__file__).with_name("runtime-manifest.py")
    if not recorder.is_file():
        recorder = Path("/usr/local/bin/hex-arm-runtime-manifest.py")
    argv = [sys.executable, str(recorder), "run", "--prefix", prefix,
            "--profile", str(profile), "--allow-development", "--scope", scope,
            "--output", str(run_dir / "run-manifest.json")]
    argv.extend("--launch-argument=" + argument for argument in command)
    if os.environ.get("HEX_ARM_RUNTIME_IMAGE"):
        argv += ["--image-identity", os.environ["HEX_ARM_RUNTIME_IMAGE"]]
    subprocess.run(argv, check=True, timeout=10)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", required=True, type=Path)
    parser.add_argument("--scope", choices=["moveit", "bringup", "startup"], required=True)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command
    if command and command[0] == "--":
        command = command[1:]
    if not command:
        parser.error("ROS launch command required")
    profile = yaml.safe_load(args.profile.read_text())
    signals = []
    def requested(sig, _frame):
        signals.append(sig)
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, requested)
    # Keep startup and soft-stop evidence beside the scoped final-disable receipt.
    receipt = os.environ.get("HEX_ARM_SHUTDOWN_REPORT")
    audit_dir = Path(receipt).parent if receipt else None
    run_dir = Path(tempfile.mkdtemp(prefix="hex-arm-exit-", dir=audit_dir))
    startup_report = run_dir / "startup-ready.json"
    env = {**os.environ, "HEX_ARM_STARTUP_REPORT": str(startup_report)}
    launch = None
    helper = None
    soft_failed = False
    try:
        if signals:
            return 1
        record_run_manifest(args.profile, run_dir, args.scope, command)
        launch = subprocess.Popen(command, env=env, start_new_session=True)
        while launch.poll() is None and not signals:
            time.sleep(0.05)
        if launch.poll() is not None:
            return launch.returncode  # Unexpected ROS exit: never initiate more motion.
        # TERM/HUP and the second interrupt retain the immediate disable path.
        if (len(signals) == 1 and signals[0] == signal.SIGINT
                and args.scope == "moveit" and eligible(profile, startup_report)):
            print("graceful exit: returning through the verified startup pose and stop sequence; second Ctrl+C disables immediately", flush=True)
            helper = subprocess.Popen(stop_command(
                profile, args.profile, startup_report, run_dir / "soft-stop.json"),
                env=env, start_new_session=True)
            deadline = time.monotonic() + 120
            while (helper.poll() is None and launch.poll() is None and len(signals) == 1
                   and time.monotonic() < deadline):
                time.sleep(0.05)
            soft_failed = helper.poll() != 0 or launch.poll() is not None
            print(f"graceful exit: {'FAILED/interrupted' if soft_failed else 'complete'}; report {run_dir / 'soft-stop.json'}", flush=True)
        else:
            print("graceful exit: no qualified normal-stop request; disabling immediately", flush=True)
    finally:
        signal_group(launch, signal.SIGINT)
        stop_child(helper)
        stop_child(launch, signal_initial=False)
    # ROS launch failures must remain visible even after a successful soft stop.
    return 1 if soft_failed or launch.returncode != 0 else 0


if __name__ == "__main__":
    sys.exit(main())
