#!/usr/bin/env python3
"""Activate a prepared ROS stack and run the operator-verified fold exit.

Run only with a supervised real launch and its exact reduced hardware profile.
J6 is smoothly aligned to zero when needed, without changing its calibration.
The three fixed startup moves then use FollowJointTrajectory. The optional final
15 mrad J2 move is planned by MoveIt and checked against the profile and strict
planning scene before ExecuteTrajectory. This client never changes calibration.
"""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import (
    ConfigureController, ListHardwareComponents, LoadController,
    SetHardwareComponentState, SwitchController,
)
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetStateValidity
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
import yaml
from hex_arm_bringup.startup_recipe import load_recipe, ready_position

JOINTS = [f"joint_{i}" for i in range(1, 7)]
STARTUP_RECIPE = load_recipe()
FOLDED = STARTUP_RECIPE["folded_position_rad"]
READY = ready_position(STARTUP_RECIPE)
# Small free-joint placement differences do not redefine encoder calibration.
FOLDED_TOLERANCE = STARTUP_RECIPE["ros_folded_tolerance_rad"]


def startup_steps(profile, positions, velocities):
    """Return bounded FJT goals; only J1–J5 define the folded entry posture."""
    if (len(positions) != 6 or len(velocities) != 6
            or not all(math.isfinite(q) for q in positions)
            or any(not math.isfinite(v) or abs(v) > STARTUP_RECIPE["stopped_velocity_rad_s"]
                   for v in velocities)):
        raise RuntimeError("startup requires finite, stationary six-axis feedback")
    if any(abs(a - b) > tolerance for a, b, tolerance
           in zip(positions[:5], FOLDED[:5], FOLDED_TOLERANCE)):
        raise RuntimeError(f"fixed startup requires J1–J5 folded reference: {positions}")
    j6 = profile["joints"][5]["limits"]
    if not j6["position_lower_rad"] <= positions[5] <= j6["position_upper_rad"]:
        raise RuntimeError("measured J6 is outside the selected profile")
    steps = []
    # Preserve the measured folded J2/J3/J4 values until each axis's turn.
    # The nominal fold is a posture check, not a goal that must be reached first.
    target = list(positions)
    for axis in (0, 4, 5):
        target[axis] = 0.0
    if abs(positions[5]) > 0.01:
        # Quintic rest-to-rest peaks are 1.875*d/T and 5.774*d/T².
        # Use at most half the profile speed/acceleration, capped at 0.05 SI.
        duration = 2.0
        for actual, desired, joint in zip(positions, target, profile["joints"]):
            distance = abs(desired - actual)
            velocity = min(0.05, joint["limits"]["velocity_rad_s"] / 2.0)
            acceleration = min(0.05, joint["limits"]["acceleration_rad_s2"] / 2.0)
            duration = max(duration, 1.875 * distance / velocity,
                           math.sqrt(5.774 * distance / acceleration))
        steps.append(("align_j6", target.copy(), math.ceil(duration) + 1))
    for step in STARTUP_RECIPE["steps"]:
        axis, value, duration = step["joint_index"], step["target_rad"], step["duration_sec"]
        target[axis] = value
        steps.append((f"startup_j{axis + 1}", target.copy(), duration))
    return steps


class Probe(Node):
    def __init__(self, profile, record_commands=False):
        super().__init__("hex_arm_real_commissioning")
        self.profile = profile
        self.positions = {}
        self.velocity = {}
        self.received_at = 0.0
        self.samples = []
        self.commands = []
        self.action_feedback = []
        self.service_timeouts = []
        self.steps = []
        self.fjt = ActionClient(self, FollowJointTrajectory,
                                "/firefly_arm_controller/follow_joint_trajectory")
        self.move_group = ActionClient(self, MoveGroup, "/move_action")
        self.trajectory_executor = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.validity = self.create_client(GetStateValidity, "/check_state_validity")
        self.hardware = self.create_client(ListHardwareComponents,
                                          "/controller_manager/list_hardware_components")
        self.create_subscription(
            JointState, "/hex_arm/internal/state", self.state, qos_profile_sensor_data)
        self.switch = self.create_client(SwitchController, "/controller_manager/switch_controller")
        self.set_hardware = self.create_client(
            SetHardwareComponentState, "/controller_manager/set_hardware_component_state")
        # A diagnostic subscriber must not add reliable-reader backpressure
        # to the command publisher, especially during cold DDS discovery.
        if record_commands:
            self.create_subscription(
                JointState, "/hex_arm/internal/command", self.command, qos_profile_sensor_data)
        self.load_controller = self.create_client(LoadController, "/controller_manager/load_controller")
        self.configure_controller = self.create_client(ConfigureController, "/controller_manager/configure_controller")
        self.hardware_activation_requested = False
        self.active_goal = None

    def activate_controllers(self, *, startup_ready=False):
        # Resolve all DDS services and configure plugins while motors are disabled.
        for endpoint in (self.hardware, self.load_controller, self.configure_controller,
                         self.set_hardware, self.switch):
            if not endpoint.wait_for_service(timeout_sec=10.0):
                raise RuntimeError("controller startup service unavailable")
        components = self.list_hardware_components(attempts=3)
        if not any(c.name == "FireflyY6System" and "hex_arm_hardware" in c.plugin_name
                   and c.state.id == 2 for c in components):
            raise RuntimeError("controller startup requires the real hardware plugin in INACTIVE")
        names = ["joint_state_broadcaster", "firefly_arm_controller"]
        for name in names:
            request = LoadController.Request(); request.name = name
            if not self.wait(self.load_controller.call_async(request), 10.0).ok:
                raise RuntimeError(f"failed to load {name} before enable")
            request = ConfigureController.Request(); request.name = name
            if not self.wait(self.configure_controller.call_async(request), 10.0).ok:
                raise RuntimeError(f"failed to configure {name} before enable")
        if not self.fjt.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("configured trajectory action unavailable before enable")
        deadline = time.monotonic() + 5.0
        while len(self.positions) < 6 and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
        self.spin(1.0)
        if startup_ready:
            # Reject a wrong posture/path while motors are still disabled.
            self.check_point(self.q())
            for _, target, _ in startup_steps(
                    self.profile, self.q(), [self.velocity[name] for name in JOINTS]):
                self.check_point(target)
        print("Controllers loaded and configured; DDS ready while hardware INACTIVE", flush=True)
        request = SetHardwareComponentState.Request()
        request.name = "FireflyY6System"; request.target_state.id = 3
        self.hardware_activation_requested = True
        result = self.wait(self.set_hardware.call_async(request), 10.0)
        if not result.ok or result.state.id != 3:
            raise RuntimeError("hardware activation failed")
        request = SwitchController.Request()
        request.activate_controllers = names
        request.strictness = SwitchController.Request.STRICT
        request.timeout.sec = 3
        if not self.wait(self.switch.call_async(request), 5.0).ok:
            raise RuntimeError("failed to activate the prepared controller group")
        print("Prepared controllers active; measured-pose stream continues", flush=True)

    def list_hardware_components(self, attempts=1):
        # Fast DDS can discover a new service before its response reader matches.
        # Retry this read-only query before enable; never retry a motor command.
        for attempt in range(1, attempts + 1):
            future = self.hardware.call_async(ListHardwareComponents.Request())
            try:
                return self.wait(future, 5.0).component
            except TimeoutError as error:
                self.hardware.remove_pending_request(future)
                future.cancel()
                self.service_timeouts.append({
                    "service": "list_hardware_components", "attempt": attempt,
                    "t": time.monotonic(),
                })
                if attempt == attempts:
                    raise TimeoutError(
                        f"list_hardware_components timed out after {attempts} attempts"
                    ) from error
                self.get_logger().warning(
                    "Hardware status response timed out before enable; retrying read-only query"
                )

    def command(self, message):
        self.commands.append({"t": time.monotonic(), "q": list(message.position),
                              "dq": list(message.velocity)})

    def deactivate(self):
        if not self.switch.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("controller manager unavailable for orderly deactivation")
        request = SwitchController.Request()
        request.deactivate_controllers = ["firefly_arm_controller", "joint_state_broadcaster"]
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.timeout.sec = 3
        result = self.wait(self.switch.call_async(request), 5.0)
        if not result.ok:
            raise RuntimeError(f"controller deactivation failed: {result.message}")
        request = SetHardwareComponentState.Request()
        request.name = "FireflyY6System"
        request.target_state.id = 2
        result = self.wait(self.set_hardware.call_async(request), 10.0)
        if not result.ok or result.state.id != 2:
            raise RuntimeError("hardware did not confirm INACTIVE")
        print("Controllers stopped; real hardware confirmed INACTIVE", flush=True)

    def state(self, message):
        self.positions.update(zip(message.name, message.position))
        self.velocity.update(zip(message.name, message.velocity))
        self.received_at = time.monotonic()
        if all(name in self.positions for name in JOINTS):
            self.samples.append({"t": self.received_at, "q": self.q(),
                                 "dq": [self.velocity.get(name, 0.0) for name in JOINTS]})

    def q(self):
        return [self.positions[name] for name in JOINTS]

    def wait(self, future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not future.done():
            raise TimeoutError("ROS operation timed out")
        return future.result()

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if time.monotonic() - self.received_at > 0.2:
                raise RuntimeError("real driver feedback is stale")

    def check_point(self, positions):
        if len(positions) != 6:
            raise RuntimeError("trajectory must contain six joints")
        for q, joint in zip(positions, self.profile["joints"]):
            limits = joint["limits"]
            if not math.isfinite(q) or not (
                limits["position_lower_rad"] - 1e-6 <= q <= limits["position_upper_rad"] + 1e-6
            ):
                raise RuntimeError(f"trajectory exceeds {joint['name']} profile authority: {q}")

    def direct_step(self, target, duration, label):
        self.check_point(target)
        started_at = time.monotonic()
        initial = self.q()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        point = JointTrajectoryPoint()
        point.positions = list(target)
        point.velocities = [0.0] * 6
        point.accelerations = [0.0] * 6
        point.time_from_start.sec = duration
        goal.trajectory.points = [point]
        def record_feedback(message):
            feedback = message.feedback
            self.action_feedback.append({
                "t": time.monotonic(), "step": label,
                "desired": list(feedback.desired.positions),
                "desired_velocity": list(feedback.desired.velocities),
                "actual": list(feedback.actual.positions),
                "error": list(feedback.error.positions),
            })
        feedback_start = len(self.action_feedback)
        handle = self.wait(self.fjt.send_goal_async(goal, feedback_callback=record_feedback), 5.0)
        if not handle.accepted:
            raise RuntimeError("fixed startup FJT goal rejected")
        self.active_goal = handle
        result = self.wait(handle.get_result_async(), duration + 5.0)
        self.active_goal = None
        self.spin(0.3)
        record = {"step": label, "started_at": started_at, "duration_sec": duration, "initial": initial,
                  "target": list(target), "actual": self.q(),
                  "status": result.status, "error_code": result.result.error_code,
                  "message": result.result.error_string}
        feedback = self.action_feedback[feedback_start:]
        if feedback:
            record["max_tracking_error_rad"] = [
                max(abs(sample["error"][i]) for sample in feedback) for i in range(6)]
        self.steps.append(record)
        print(json.dumps(record), flush=True)
        if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code != 0:
            raise RuntimeError(f"fixed startup FJT failed: {record}")
        return record

    def is_valid(self, positions):
        req = GetStateValidity.Request()
        req.group_name = "arm"
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = list(positions)
        result = self.wait(self.validity.call_async(req), 5.0)
        if not result.valid:
            pairs = [(c.contact_body_1, c.contact_body_2) for c in result.contacts]
            raise RuntimeError(f"strict MoveIt state rejected: {positions}, contacts={pairs}")

    def plan_and_execute(self, target):
        initial = self.q()
        self.check_point(target)
        self.is_valid(self.q())
        self.is_valid(target)
        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.pipeline_id = "ompl"
        goal.request.num_planning_attempts = 2
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.2
        goal.request.max_acceleration_scaling_factor = 0.1
        goal.request.start_state.is_diff = True
        desired = Constraints()
        desired.joint_constraints = [
            JointConstraint(joint_name=name, position=q, tolerance_above=0.0001,
                            tolerance_below=0.0001, weight=1.0)
            for name, q in zip(JOINTS, target)
        ]
        goal.request.goal_constraints = [desired]
        # Tell the planner the independently enforced reduced hardware bounds.
        envelope = Constraints()
        for joint in self.profile["joints"]:
            lower = joint["limits"]["position_lower_rad"]
            upper = joint["limits"]["position_upper_rad"]
            envelope.joint_constraints.append(JointConstraint(
                joint_name=joint["name"], position=(lower + upper) / 2.0,
                tolerance_above=(upper - lower) / 2.0,
                tolerance_below=(upper - lower) / 2.0, weight=1.0))
        goal.request.path_constraints = envelope
        goal.planning_options.plan_only = True
        goal.planning_options.planning_scene_diff.is_diff = True
        handle = self.wait(self.move_group.send_goal_async(goal), 5.0)
        if not handle.accepted:
            raise RuntimeError("MoveGroup planning goal rejected")
        self.active_goal = handle
        wrapped = self.wait(handle.get_result_async(), 15.0)
        self.active_goal = None
        plan = wrapped.result
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or plan.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"MoveIt planning failed: {plan.error_code.val}")
        trajectory = plan.planned_trajectory
        if list(trajectory.joint_trajectory.joint_names) != JOINTS:
            raise RuntimeError("unexpected planned joint order")
        if not trajectory.joint_trajectory.points:
            raise RuntimeError("empty MoveIt plan")
        for point in trajectory.joint_trajectory.points:
            self.check_point(point.positions)
            for i, value in enumerate(point.velocities):
                if abs(value) > self.profile["joints"][i]["limits"]["velocity_rad_s"] + 1e-5:
                    raise RuntimeError("planned velocity exceeds hardware profile")
            for i, value in enumerate(point.accelerations):
                if abs(value) > self.profile["joints"][i]["limits"]["acceleration_rad_s2"] + 1e-5:
                    raise RuntimeError("planned acceleration exceeds hardware profile")
            self.is_valid(point.positions)
        print(f"Strict MoveIt plan checked: {len(trajectory.joint_trajectory.points)} points", flush=True)
        execution = ExecuteTrajectory.Goal()
        execution.trajectory = trajectory
        handle = self.wait(self.trajectory_executor.send_goal_async(execution), 5.0)
        if not handle.accepted:
            raise RuntimeError("MoveIt execution rejected")
        self.active_goal = handle
        wrapped = self.wait(handle.get_result_async(), 30.0)
        self.active_goal = None
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or wrapped.result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"MoveIt execution failed: {wrapped.result.error_code.val}")
        self.spin(1.0)
        return {"target": target, "initial": initial, "actual": self.q(), "moveit_error_code": wrapped.result.error_code.val,
                "plan_points": len(trajectory.joint_trajectory.points)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--allow-motion", action="store_true", required=True)
    parser.add_argument("--moveit", action="store_true")
    parser.add_argument("--activate-controllers", action="store_true",
                        help="Load/configure controllers while inactive, then enable and activate as a group")
    parser.add_argument("--hold-current", action="store_true",
                        help="Keep measured-pose activation without the fixed startup moves")
    parser.add_argument("--record-commands", action="store_true",
                        help="Optionally sample commands with best-effort QoS for diagnostics")
    parser.add_argument("--deactivate-after", action="store_true",
                        help="Stop controllers and disable hardware after this trial")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    profile = yaml.safe_load(args.profile.read_text())
    if not profile["calibrated"] or (not args.hold_current and profile["bus"]["protocol"] != "meow"):
        raise RuntimeError("requires a calibrated profile; fixed startup requires Meow")
    if args.hold_current and args.moveit:
        raise RuntimeError("--moveit requires the fixed ready sequence")
    rclpy.init()
    node = Probe(profile, record_commands=args.record_commands)
    report = {"time": datetime.now(timezone.utc).isoformat(),
              "profile": str(args.profile), "steps": node.steps}
    hardware_verified = False
    try:
        if args.activate_controllers:
            node.activate_controllers(startup_ready=not args.hold_current)
        if not node.fjt.wait_for_server(timeout_sec=40.0) or not node.hardware.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("active real ROS controllers are unavailable")
        components = node.list_hardware_components()
        if not any(c.name == "FireflyY6System" and "hex_arm_hardware" in c.plugin_name
                   and c.state.label == "active" for c in components):
            raise RuntimeError("expected active real FireflyY6System, refusing other hardware")
        hardware_verified = True
        deadline = time.monotonic() + 5.0
        while len(node.positions) < 6 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        node.spin(3.0)
        if args.hold_current:
            report["measured_hold"] = {"q": node.q()}
        else:
            steps = startup_steps(profile, node.q(), [node.velocity[name] for name in JOINTS])
            # Check every target before sending the first motion command.
            for _, target, _ in steps:
                node.check_point(target)
            for label, target, duration in steps:
                node.direct_step(target, duration, label)
            start = len(node.samples)
            node.spin(10.0)
            held = node.samples[start:]
            report["ready_hold"] = {
                "seconds": 10.0, "samples": len(held), "last_q": node.q(),
                "max_error_rad": [max(abs(s["q"][i] - READY[i]) for s in held) for i in range(6)],
                "max_velocity_rad_s": max(abs(v) for s in held for v in s["dq"]),
            }
            print(json.dumps(report["ready_hold"]), flush=True)
            if max(report["ready_hold"]["max_error_rad"]) > 0.005:
                raise RuntimeError("ready hold exceeds the existing 0.005 rad ROS goal requirement")
            if args.moveit:
                if not (node.move_group.wait_for_server(timeout_sec=5.0)
                        and node.trajectory_executor.wait_for_server(timeout_sec=5.0)
                        and node.validity.wait_for_service(timeout_sec=5.0)):
                    raise RuntimeError("MoveIt services unavailable")
                target = READY.copy()
                target[1] += 0.015
                report["moveit"] = node.plan_and_execute(target)
                print(json.dumps(report["moveit"]), flush=True)
        report["passed"] = True
        if not args.deactivate_after:
            mode = "Measured pose held" if args.hold_current else "startup_ready reached and verified"
            print(f"{mode}; controller continues holding. Report: {args.output}", flush=True)
    except BaseException as error:
        report["passed"] = False
        report["error"] = str(error)
        if node.active_goal is not None:
            node.wait(node.active_goal.cancel_goal_async(), 3.0)
        raise
    finally:
        trial_motion_passed = report.get("passed", False)
        cleanup_error = None
        stop_after = args.deactivate_after or (args.activate_controllers and not trial_motion_passed)
        if stop_after and (hardware_verified or node.hardware_activation_requested):
            try:
                node.deactivate()
                report["deactivated"] = True
            except Exception as error:
                cleanup_error = error
                report["passed"] = False
                report["deactivated"] = False
                report["cleanup_error"] = str(error)
        report["samples"] = node.samples
        report["commands"] = node.commands
        report["action_feedback"] = node.action_feedback
        report["service_timeouts"] = node.service_timeouts
        args.output.write_text(json.dumps(report, indent=2) + "\n")
        node.destroy_node()
        rclpy.shutdown()
        if cleanup_error is not None and trial_motion_passed:
            raise RuntimeError(f"trial motion passed, but deactivation failed: {cleanup_error}")


if __name__ == "__main__":
    main()
