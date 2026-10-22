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
from control_msgs.msg import JointTrajectoryControllerState
from hex_arm_msgs.msg import DriverState, InterpolationState
from std_srvs.srv import Trigger
import yaml
from hex_arm_bringup.trajectory_diagnostics import feedback_record, feedback_summary, point_record

spec = importlib.util.spec_from_file_location("startup_client", Path(__file__).with_name("commission-startup-ros.py"))
startup = importlib.util.module_from_spec(spec)
spec.loader.exec_module(startup)

SHUTDOWN_PATH_TOLERANCE_RAD = 0.11
SHUTDOWN_RETURN_VELOCITY_SCALING = 0.25
SHUTDOWN_RETURN_ACCELERATION_SCALING = 0.25


def shutdown_scaling(value):
    value = float(value)
    if not math.isfinite(value) or not 0 < value <= 1:
        raise argparse.ArgumentTypeError("shutdown scaling must be finite and within (0, 1]")
    return value


def add_shutdown_arguments(parser):
    parser.add_argument("--shutdown-velocity-scaling", type=shutdown_scaling,
                        default=SHUTDOWN_RETURN_VELOCITY_SCALING)
    parser.add_argument("--shutdown-acceleration-scaling", type=shutdown_scaling,
                        default=SHUTDOWN_RETURN_ACCELERATION_SCALING)


def collect_diagnostics(node, report):
    for name in ("samples", "steps", "action_feedback", "controller_states", "interpolation_states", "driver_states",
                 "commands", "sequence_path_checks", "handoff"):
        report[name] = getattr(node, name, [])
    report["last_plan"] = getattr(node, "last_plan_diagnostics", None)
    observations = report["interpolation_states"]
    segments = {sample["command_generation"]: sample for sample in observations}
    report["interpolation_summary"] = {
        "available": bool(observations), "samples": len(observations),
        "observed_segments": len(segments),
        "retimed_observed_segments": sum(sample["effective_duration_ns"] > sample["requested_duration_ns"]
                                         for sample in segments.values()),
        "max_effective_duration_ns": max((sample["effective_duration_ns"] for sample in observations), default=0),
    }
    report.setdefault("motion_completed", False)
    report["disable_result"] = {
        "confirmed": not report.get("already_inactive", False) and bool(
            report.get("deactivated") or report.get("phase") == "disabled_confirmed"),
        "error": report.get("deactivation_error"),
    }
    report["diagnostic_clocks"] = {
        "t": "client monotonic seconds", "header_stamp_ns": "ROS time",
        "interpolation_source_stamp_ns": "driver process monotonic; not synchronized with client",
    }


def already_folded_reference(profile, positions):
    """Recognize folded structure without moving a placed base or free wrist."""
    if (profile.get("bus", {}).get("protocol", "meow") != "meow"
            or len(positions) != 6 or any(not math.isfinite(q) for q in positions)
            or any(abs(positions[axis]-startup.FOLDED[axis]) > startup.HOLD_POSITION_TOLERANCE_RAD
                   for axis in (1, 2, 3))):
        return None
    # J2/J3/J4 determine the folded support structure. J1/J5/J6 retain their
    # measured placement, subject to the same hardware bounds and state-validity
    # check as every stop. No shutdown endpoint tolerance is widened.
    return startup.folded_entry_command(profile, positions)


def measured_folded_reentry(node, report):
    """Return a stationary startup-window contact pose through the original axis order."""
    if node.profile.get("bus", {}).get("protocol", "meow") != "meow":
        return False
    try:
        reference = startup.validate_measured_startup(
            node.profile, node.q(), [node.velocity.get(name) for name in startup.JOINTS], legacy_report=True)
    except RuntimeError:
        return False
    if not node.validity.wait_for_service(timeout_sec=2.0):
        raise RuntimeError("strict MoveIt unavailable; measured folded return refused")
    # A valid entry uses the existing MoveIt-to-READY stop. Only the documented
    # folded contacts enter this bounded re-entry, with no planning-scene ACM edit.
    if node.is_valid(node.q(), startup.FOLDED_CONTACTS):
        return False
    steps, previous = [], reference.copy()
    for axis in (2, 3, 1):
        target = previous.copy()
        target[axis] = startup.FOLDED[axis]
        duration = startup.rest_to_rest_duration(previous, target, node.motion_limits)
        steps.append((f"return_startup_j{axis + 1}", target, duration))
        for k in range(81):
            q = [a+(b-a)*k/80 for a, b in zip(previous, target)]
            node.check_point(q)
            valid = node.is_valid(q, startup.FOLDED_CONTACTS)
            node.sequence_path_checks.append({
                "step": steps[-1][0], "q": q, "strict_valid": valid,
                "measured_folded_reentry": True,
            })
        previous = target.copy()
    # Complete the endpoint, position, collision and dynamics preflight before
    # sending the first goal. Free axes retain the bounded measured hold target.
    report["phase"] = "return_measured_to_folded"
    report["folded_reentry_reference_q"] = reference.copy()
    node.folded_entry_checked = True
    for label, target, duration in steps:
        node.direct_step(target, duration, label, retime=True)
        node.wait_stationary(target)
        node.is_valid(node.q(), startup.FOLDED_CONTACTS)
    if already_folded_reference(node.profile, node.q()) is None:
        raise RuntimeError("measured folded return did not reach the original folded structure tolerance")
    report.update(measured_folded_reentry=True, folded_q=node.q(), motion_completed=True)
    node.monitor_motion = False
    node.deactivate()
    report.update(passed=True, deactivated=True, phase="disabled_confirmed")
    return True


class StopProbe(startup.Probe):
    def __init__(self, profile, record_commands=False, motion_limits=None, instance=None):
        super().__init__(profile, record_commands=True, motion_limits=motion_limits, instance=instance)
        self.driver = None
        self.driver_at = 0.0
        self._driver_stamp = None
        self.driver_states = []
        self.monitor_motion = False
        self.shutdown_path_tolerance_rad = None
        self.controller_states = []
        self.interpolation_states = []
        self.handoff = []
        self._controller_at = 0.0
        self._controller_stamp = None
        self._controller_reference = None
        self._handoff_complete = False
        self.driver_sub = self.create_subscription(DriverState, self.instance.ros_name("hex_arm/driver_state"), self.driver_state, 10)
        self.create_subscription(JointTrajectoryControllerState,
                                 self.instance.controller_ros_name() + "/controller_state",
                                 self.controller_state, startup.qos_profile_sensor_data)
        self.create_subscription(InterpolationState, self.instance.ros_name("hex_arm/interpolation_state"),
                                 self.interpolation_state, startup.qos_profile_sensor_data)

    def controller_state(self, message):
        # Jazzy uses reference/feedback; older control_msgs used desired/actual.
        reference = getattr(message, "reference", None)
        actual = getattr(message, "feedback", None)
        if reference is None:
            reference, actual = message.desired, message.actual
        stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
        names = list(message.joint_names)
        record = {"t": time.monotonic(), "joint_names": names,
                  "header_stamp_ns": stamp[0] * 1_000_000_000 + stamp[1],
                  "reference": point_record(reference), "actual": point_record(actual),
                  "error": point_record(message.error)}
        self.controller_states.append(record)
        if ((self._controller_stamp is None or stamp > self._controller_stamp) and stamp != (0, 0)
                and len(names) == len(reference.positions) == len(reference.velocities) == 6
                and set(names) == set(self.instance.joint_names)
                and all(math.isfinite(v) for v in (*reference.positions, *reference.velocities))):
            self._controller_stamp, self._controller_at = stamp, record["t"]
            self._controller_reference = (
                [reference.positions[names.index(name)] for name in self.instance.joint_names],
                [reference.velocities[names.index(name)] for name in self.instance.joint_names])

    def interpolation_state(self, message):
        self.interpolation_states.append({
            "t": time.monotonic(), "joint_names": list(message.joint_names),
            **{key: getattr(message, key) for key in (
                "source_stamp_ns", "command_sequence", "command_generation", "segment_start_ns",
                "requested_duration_ns", "effective_duration_ns", "sample_stamp_ns")},
            **{key: list(getattr(message, key)) for key in (
                "command_position", "command_velocity", "interpolated_position", "interpolated_velocity")},
        })

    def settled_reference(self):
        self.check_live()
        if self._controller_reference is None or time.monotonic() - self._controller_at > 0.2:
            raise RuntimeError("shutdown requires fresh controller command reference")
        q, dq = self._controller_reference
        if (any(abs(v) > startup.HOLD_VELOCITY_PEAK_RAD_S for v in dq)
                or any(abs(v) > startup.HOLD_VELOCITY_PEAK_RAD_S for v in self.velocity.values())
                or any(abs(a-b) > startup.HOLD_POSITION_TOLERANCE_RAD for a, b in zip(q, self.q()))):
            raise RuntimeError("shutdown requires a settled continuous command reference")
        self.check_point(q)
        return list(q)

    def prepare_handoff(self):
        self.cancel_motion()
        self.wait_stationary()
        reference = self.settled_reference()
        self.handoff.append({"phase": "stationary", "t": time.monotonic(),
                             "actual": self.q(), "command_reference": reference})
        self.last_commanded_target = reference
        self._handoff_complete = True

    def planning_reference(self, initial):
        if not self._handoff_complete:
            raise RuntimeError("shutdown motion requires completed goal handoff")
        return self.settled_reference()

    def check_execution_start(self, trajectory):
        reference = self.settled_reference()
        first = list(trajectory.points[0].positions)
        self.last_plan_diagnostics["execution_start"] = {
            "t": time.monotonic(), "actual": self.q(), "command_reference": reference,
            "first_point": first,
        }
        if any(abs(a-b) > 1e-4 for a, b in zip(first, reference)):
            raise RuntimeError("shutdown plan first point no longer matches the settled command reference")

    def execute_planned_trajectory(self, trajectory, timeout):
        if self.shutdown_path_tolerance_rad is None:
            return super().execute_planned_trajectory(trajectory, timeout)
        # ExecuteTrajectory cannot carry per-goal tolerances. Send the fully
        # checked MoveIt plan to the same controller with the shutdown override.
        goal = startup.FollowJointTrajectory.Goal()
        goal.trajectory = trajectory.joint_trajectory
        startup.set_path_tolerance(goal, self.shutdown_path_tolerance_rad)
        self.last_plan_diagnostics.update(
            execution_action="follow_joint_trajectory",
            path_tolerance_rad=self.shutdown_path_tolerance_rad)
        if not self.fjt.wait_for_server(timeout_sec=2.0):
            raise RuntimeError("shutdown FJT action unavailable")
        feedback_start = len(self.action_feedback)
        def record_feedback(message):
            self.action_feedback.append(feedback_record(message.feedback, "return_to_ready"))
        self.last_plan_diagnostics["execution_started_at"] = time.monotonic()
        handle = self.wait(self.fjt.send_goal_async(goal, feedback_callback=record_feedback), 5.0)
        if not handle.accepted:
            raise RuntimeError("shutdown FJT execution rejected")
        self.active_goal = handle
        try:
            wrapped = self.wait(handle.get_result_async(), timeout)
        finally:
            self.last_plan_diagnostics.update(feedback_summary(
                self.action_feedback[feedback_start:], goal.trajectory.joint_names))
            self.last_plan_diagnostics["execution_finished_at"] = time.monotonic()
        self.active_goal = None
        self.last_plan_diagnostics.update(status=wrapped.status, fjt_error_code=wrapped.result.error_code,
                                          error_string=wrapped.result.error_string)
        if wrapped.status != startup.GoalStatus.STATUS_SUCCEEDED or wrapped.result.error_code != 0:
            raise RuntimeError(
                f"shutdown FJT execution failed: {wrapped.result.error_code}; {wrapped.result.error_string}")
        return {"fjt_error_code": wrapped.result.error_code}

    def driver_state(self, message):
        self.driver = message
        stamp = (message.stamp.sec, message.stamp.nanosec)
        if stamp != (0, 0) and (self._driver_stamp is None or stamp > self._driver_stamp):
            self.driver_at = time.monotonic()
            self._driver_stamp = stamp
        self.driver_states.append({"t": time.monotonic(), "stamp": list(stamp),
                                   **{name: getattr(message, name) for name in (
                                       "mode", "session_owned", "feedback_fresh", "fault_latched",
                                       "fault_code", "fault_reason", "command_age_s", "feedback_age_s")}})

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
            detail = f"; mode={d.mode}, fault={d.fault_code}: {d.fault_reason}" if d else ""
            raise RuntimeError("shutdown requires healthy active driver" + detail)
        if self.profile.get("bus", {}).get("protocol", "meow") == "meow":
            self.check_point(startup.folded_entry_command(self.profile, self.q()))
        else:
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
        instance = startup.instance_of(self)
        actions = [(instance.ros_name("move_action"), startup.MoveGroup),
                   (instance.ros_name("execute_trajectory"), startup.ExecuteTrajectory),
                   (instance.controller_ros_name() + "/follow_joint_trajectory", startup.FollowJointTrajectory)]
        for action, action_type in actions:
            client = self.create_client(CancelGoal, action + "/_action/cancel_goal")
            results = self.create_client(action_type.Impl.GetResultService, action + "/_action/get_result")
            try:
                if not client.wait_for_service(timeout_sec=2.0):
                    raise RuntimeError(f"shutdown cannot cancel {action}")
                result = self.wait(client.call_async(CancelGoal.Request()), 3.0)
                if result.return_code not in (0, 3):
                    raise RuntimeError(f"shutdown cancel rejected by {action}")
                record = {"action": action, "return_code": result.return_code, "goals": []}
                self.handoff.append(record)
                for info in result.goals_canceling:
                    if not results.wait_for_service(timeout_sec=2.0):
                        raise RuntimeError(f"shutdown cannot confirm canceled goal from {action}")
                    request = action_type.Impl.GetResultService.Request()
                    request.goal_id = info.goal_id
                    terminal = self.wait(results.call_async(request), 5.0)
                    record["goals"].append({"id": list(info.goal_id.uuid), "status": terminal.status})
                    if terminal.status not in (startup.GoalStatus.STATUS_SUCCEEDED,
                                               startup.GoalStatus.STATUS_CANCELED,
                                               startup.GoalStatus.STATUS_ABORTED):
                        raise RuntimeError(f"shutdown old goal has no terminal result from {action}")
            finally:
                self.destroy_client(client)
                self.destroy_client(results)

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
    startup.add_instance_arguments(parser)
    add_shutdown_arguments(parser)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--startup-report", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    instance = startup.instance_from_args(args)
    prior = json.loads(args.startup_report.read_text())
    from hex_arm_bringup.integration import verify_report_instance
    verify_report_instance(prior, instance)
    if prior.get("passed") is not True or "ready_hold" not in prior or prior.get("deactivated"):
        raise RuntimeError("this launch has not completed verified startup")
    profile = yaml.safe_load(args.profile.read_text())
    digest = hashlib.sha256(args.profile.read_bytes()).hexdigest()
    if prior.get("profile_sha256") != digest:
        raise RuntimeError("normal stop requires this launch's exact hardware profile SHA256")
    if prior.get("startup_mode") == "measured_hold":
        startup.verified_measured_hold(
            profile, prior, hashlib.sha256(args.profile.read_bytes()).hexdigest())
    sequence = None
    ready = startup.READY
    if profile.get("bus", {}).get("protocol", "meow") == "cia402":
        digest = hashlib.sha256(args.profile.read_bytes()).hexdigest()
        if prior.get("profile_sha256") != digest or "sequence" not in prior:
            raise RuntimeError("CiA402 normal stop requires this launch's arm-bound startup evidence")
        reference = prior["measured_hold"]["reference_q"]
        sequence = startup.cia402_steps(prior["sequence"], profile, reference, digest)
        ready = sequence[-1][1]
    elif not profile.get("controller", {}).get("shutdown_damping"):
        raise RuntimeError("damping is not configured")
    rclpy.init()
    node = StopProbe(profile, motion_limits=prior.get("motion_limits"), instance=instance)
    report = {"passed": False, "phase": "preflight", "instance": instance.as_dict(),
              "profile_sha256": hashlib.sha256(args.profile.read_bytes()).hexdigest(),
              "startup_report": str(args.startup_report),
              "shutdown_path_tolerance_rad": SHUTDOWN_PATH_TOLERANCE_RAD,
              "shutdown_velocity_scaling": args.shutdown_velocity_scaling,
              "shutdown_acceleration_scaling": args.shutdown_acceleration_scaling,
              "shutdown_damping": profile.get("controller", {}).get("shutdown_damping")}
    try:
        deadline = time.monotonic() + 5
        while (len(node.positions) != 6 or node.driver is None) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        node.check_live()
        node.shutdown_path_tolerance_rad = SHUTDOWN_PATH_TOLERANCE_RAD
        node.monitor_motion = True
        node.prepare_handoff()
        folded_reference = already_folded_reference(profile, node.q())
        if folded_reference is not None:
            if not node.validity.wait_for_service(timeout_sec=2.0):
                raise RuntimeError("strict MoveIt unavailable; folded stop refused")
            node.is_valid(node.q(), startup.FOLDED_CONTACTS)
            report.update(already_folded=True, folded_q=node.q(), motion_completed=True)
            node.monitor_motion = False
            node.deactivate()
            report.update(passed=True, phase="disabled_confirmed")
            return
        if prior.get("startup_mode") == "measured_hold" and measured_folded_reentry(node, report):
            return
        report["phase"] = "return_to_ready"
        report["return_started_at"] = time.monotonic()
        if not all(abs(q - t) <= startup.HOLD_POSITION_TOLERANCE_RAD for q, t in zip(node.q(), ready)):
            if not (node.validity.wait_for_service(timeout_sec=2.0)
                    and node.move_group.wait_for_server(timeout_sec=2.0)
                    and node.trajectory_executor.wait_for_server(timeout_sec=2.0)):
                raise RuntimeError("strict MoveIt unavailable; return-to-ready refused")
            report["return"] = node.plan_and_execute(
                ready, args.shutdown_velocity_scaling, args.shutdown_acceleration_scaling)
        node.wait_stationary(ready)
        report["ready_q"] = node.q()
        report["ready_completed"] = True
        if sequence is not None:
            report["phase"] = "return_to_folded"
            node.check_sequence_path(reference, sequence)
            previous = [reference, *(step[1] for step in sequence[:-1])]
            for index in reversed(range(len(sequence))):
                label, _, duration = sequence[index]
                node.direct_step(previous[index], duration, f"return_{label}")
                node.is_valid(node.q(), startup.FOLDED_CONTACTS if index == 0 else frozenset())
            node.wait_stationary(reference)
            report.update(folded_q=node.q(), motion_completed=True)
            node.monitor_motion = False
            node.deactivate()
            report.update(passed=True, phase="disabled_confirmed")
            return
        report["phase"] = "damping"
        report["damping_started_at"] = time.monotonic()
        client = node.create_client(Trigger, instance.ros_name("hex_arm/damped_stop"))
        if not client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("Rust damping service unavailable")
        node.monitor_motion = False  # Rust now owns monitoring through to disable.
        result = node.wait(client.call_async(Trigger.Request()), 38.0)
        if not result.success:
            raise RuntimeError(result.message)
        report.update(passed=True, motion_completed=True, phase="disabled_confirmed")
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
        collect_diagnostics(node, report)
        args.output.write_text(json.dumps(report, indent=2) + "\n")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    from hex_arm_bringup.operation_lock import run_stop_script
    run_stop_script(main)
