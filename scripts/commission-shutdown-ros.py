#!/usr/bin/env python3
"""Return through strict MoveIt planning, verify ready, ask Rust to damp and disable."""
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import time

import rclpy
from action_msgs.srv import CancelGoal
from hex_arm_msgs.msg import DriverState
from std_srvs.srv import Trigger
import yaml

spec = importlib.util.spec_from_file_location("startup_client", Path(__file__).with_name("commission-startup-ros.py"))
startup = importlib.util.module_from_spec(spec)
spec.loader.exec_module(startup)


class StopProbe(startup.Probe):
    def __init__(self, profile, record_commands=False):
        super().__init__(profile, record_commands=record_commands)
        self.driver = None
        self.driver_at = 0.0
        self.monitor_motion = False
        self.driver_sub = self.create_subscription(DriverState, "/hex_arm/driver_state", self.driver_state, 10)

    def driver_state(self, message):
        self.driver = message
        self.driver_at = time.monotonic()

    def check_live(self):
        now = time.monotonic()
        if now - self.received_at > 0.2 or now - self.driver_at > 0.2:
            raise RuntimeError("shutdown feedback stale")
        if len(self.positions) != 6 or len(self.velocity) != 6:
            raise RuntimeError("shutdown requires six-axis feedback")
        if not all(math.isfinite(v) for v in [*self.q(), *self.velocity.values()]):
            raise RuntimeError("shutdown feedback nonfinite")
        d = self.driver
        if not (d and d.mode == 2 and d.session_owned and d.profile_valid and d.calibrated
                and d.all_motors_online and d.feedback_fresh and not d.fault_latched):
            raise RuntimeError("shutdown requires healthy active driver")
        self.check_point(self.q())

    def wait(self, future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.monitor_motion:
                self.check_live()
        if not future.done():
            raise TimeoutError("shutdown ROS operation timed out")
        return future.result()

    def cancel_motion(self):
        # Cancel both MoveIt ownership and any direct trajectory goal before planning.
        for action in ("/move_action", "/execute_trajectory", "/firefly_arm_controller/follow_joint_trajectory"):
            client = self.create_client(CancelGoal, action + "/_action/cancel_goal")
            try:
                if not client.wait_for_service(timeout_sec=2.0):
                    raise RuntimeError(f"shutdown cannot cancel {action}")
                result = self.wait(client.call_async(CancelGoal.Request()), 3.0)
                if result.return_code not in (0, 3):
                    raise RuntimeError(f"shutdown cancel rejected by {action}")
            finally:
                self.destroy_client(client)

    def wait_stationary(self, target=None):
        deadline = time.monotonic() + 5.0
        stable = None
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            self.check_live()
            good = all(abs(v) <= 0.02 for v in self.velocity.values())
            if target is not None:
                good = good and all(abs(q - t) <= startup.HOLD_POSITION_TOLERANCE_RAD for q, t in zip(self.q(), target))
            stable = (stable or time.monotonic()) if good else None
            if stable is not None and time.monotonic() - stable >= 0.5:
                return
        raise RuntimeError("shutdown pose did not settle")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--startup-report", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    prior = json.loads(args.startup_report.read_text())
    if prior.get("passed") is not True or "ready_hold" not in prior or prior.get("deactivated"):
        raise RuntimeError("this launch has not completed verified startup")
    profile = yaml.safe_load(args.profile.read_text())
    sequence = None
    ready = startup.READY
    if profile.get("bus", {}).get("protocol") == "cia402":
        digest = hashlib.sha256(args.profile.read_bytes()).hexdigest()
        if prior.get("profile_sha256") != digest or "sequence" not in prior:
            raise RuntimeError("CiA402 normal stop requires this launch's arm-bound startup evidence")
        reference = prior["measured_hold"]["reference_q"]
        sequence = startup.cia402_steps(prior["sequence"], profile, reference, digest)
        ready = sequence[-1][1]
    elif not profile.get("controller", {}).get("shutdown_damping"):
        raise RuntimeError("damping is not configured")
    rclpy.init()
    node = StopProbe(profile)
    report = {"passed": False, "phase": "preflight",
              "shutdown_damping": profile.get("controller", {}).get("shutdown_damping")}
    try:
        deadline = time.monotonic() + 5
        while (len(node.positions) != 6 or node.driver is None) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        node.check_live()
        node.monitor_motion = True
        node.cancel_motion()
        node.wait_stationary()
        report["phase"] = "return_to_ready"
        report["return_started_at"] = time.monotonic()
        if not all(abs(q - t) <= startup.HOLD_POSITION_TOLERANCE_RAD for q, t in zip(node.q(), ready)):
            if not (node.validity.wait_for_service(timeout_sec=2.0)
                    and node.move_group.wait_for_server(timeout_sec=2.0)
                    and node.trajectory_executor.wait_for_server(timeout_sec=2.0)):
                raise RuntimeError("strict MoveIt unavailable; return-to-ready refused")
            report["return"] = node.plan_and_execute(ready)
        node.wait_stationary(ready)
        report["ready_q"] = node.q()
        if sequence is not None:
            report["phase"] = "return_to_folded"
            node.check_sequence_path(reference, sequence)
            previous = [reference, *(step[1] for step in sequence[:-1])]
            for index in reversed(range(len(sequence))):
                label, _, duration = sequence[index]
                node.direct_step(previous[index], duration, f"return_{label}")
                node.is_valid(node.q(), startup.FOLDED_CONTACTS if index == 0 else frozenset())
            node.wait_stationary(reference)
            report["folded_q"] = node.q()
            node.monitor_motion = False
            node.deactivate()
            report.update(passed=True, phase="disabled_confirmed")
            return
        report["phase"] = "damping"
        report["damping_started_at"] = time.monotonic()
        client = node.create_client(Trigger, "/hex_arm_bridge/damped_stop")
        if not client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("Rust damping service unavailable")
        node.monitor_motion = False  # Rust now owns monitoring through to disable.
        result = node.wait(client.call_async(Trigger.Request()), 38.0)
        if not result.success:
            raise RuntimeError(result.message)
        report.update(passed=True, phase="disabled_confirmed")
    except BaseException as error:
        report["error"] = str(error)
        if node.active_goal is not None and rclpy.ok():
            node.monitor_motion = False
            try:
                node.wait(node.active_goal.cancel_goal_async(), 2.0)
            except Exception:
                pass
        raise
    finally:
        # ROS feedback remains live during Rust damping; retain evidence for tuning.
        report["samples"] = getattr(node, "samples", [])
        report["steps"] = getattr(node, "steps", [])
        report["sequence_path_checks"] = getattr(node, "sequence_path_checks", [])
        args.output.write_text(json.dumps(report, indent=2) + "\n")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
