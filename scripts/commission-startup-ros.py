#!/usr/bin/env python3
"""Activate a prepared ROS stack and verify the selected startup path.

Run only with a supervised real launch and its exact reduced hardware profile.
Meow aligns J6 when needed, then follows the verified J2, J4, J3 sequence.
CiA402 holds its measured pose and verifies the hold before enabling the
trajectory controller. This client never changes calibration.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import re
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from controller_manager_msgs.srv import (
    ConfigureController, ListHardwareComponents, LoadController,
    SetHardwareComponentState, SwitchController,
)
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetStateValidity
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint
import yaml
from hex_arm_bringup.startup_recipe import (
    load_recipe, ready_position, resolve_motion_limits, rest_to_rest_duration,
)

JOINTS = [f"joint_{i}" for i in range(1, 7)]
STARTUP_RECIPE = load_recipe()
FOLDED = STARTUP_RECIPE["folded_position_rad"]
READY = ready_position(STARTUP_RECIPE)
# Small free-joint placement differences do not redefine encoder calibration.
FOLDED_TOLERANCE = STARTUP_RECIPE["ros_folded_tolerance_rad"]
HOLD_POSITION_TOLERANCE_RAD = 0.015
HOLD_VELOCITY_PEAK_RAD_S = 0.02
READY_HOLD_SECONDS = 3.0
POSITION_EPSILON_RAD = 1e-6
# Existing strict-model contacts at the documented powered-off folded pose.
# Only a bounded, independently commissioned J3 exit/re-entry may traverse them.
FOLDED_CONTACTS = {tuple(sorted(pair)) for pair in (("link_1", "link_5"), ("link_2", "link_4"))}



def trajectory_duration(seconds):
    """Convert YAML numeric seconds into ROS integer seconds/nanoseconds."""
    if isinstance(seconds, bool) or not math.isfinite(seconds) or not 0 < seconds <= 120:
        raise RuntimeError("trajectory duration must be finite and within (0, 120] seconds")
    whole, fractional = divmod(round(seconds * 1_000_000_000), 1_000_000_000)
    return Duration(sec=whole, nanosec=fractional)


def rest_to_rest_points(initial, target, duration):
    """Explicit derivatives prevent a prior MoveIt endpoint accelerating held axes."""
    start = JointTrajectoryPoint()
    start.positions = list(initial)
    start.velocities = [0.0] * 6
    start.accelerations = [0.0] * 6
    end = JointTrajectoryPoint()
    end.positions = list(target)
    end.velocities = [0.0] * 6
    end.accelerations = [0.0] * 6
    end.time_from_start = trajectory_duration(duration)
    return [start, end]


def set_path_tolerance(goal, position_rad):
    """Override only this trajectory's position tracking tolerance."""
    if position_rad is None:
        return
    if isinstance(position_rad, bool) or not math.isfinite(position_rad) or position_rad <= 0:
        raise RuntimeError("trajectory path tolerance must be finite and positive")
    goal.path_tolerance = [
        JointTolerance(name=name, position=position_rad)
        for name in goal.trajectory.joint_names
    ]


def planned_execution_timeout(points):
    """Allow slow expanded travel while retaining a bounded execution deadline."""
    times = [p.time_from_start.sec + p.time_from_start.nanosec / 1e9 for p in points]
    if (not times or any(not math.isfinite(t) or t < 0 for t in times)
            or any(b <= a for a, b in zip(times, times[1:]))
            or not 0 < times[-1] <= 120):
        raise RuntimeError('planned trajectory requires increasing timestamps and duration <=120 s')
    return max(30., times[-1] + 10.)

def measured_hold_duration(profile):
    """Bound the startup gravity ramp before handing commands to ros2_control."""
    rate = profile.get("controller", {}).get("gravity_startup_slew_rate_nm_s")
    if rate is None:
        return 3.0
    limits = [j["limits"]["torque_nm"] for j in profile["joints"]]
    if (not math.isfinite(rate) or rate <= 0 or len(limits) != 6
            or any(not math.isfinite(t) or t <= 0 for t in limits)):
        raise RuntimeError("measured-pose hold requires finite positive gravity slew and torque limits")
    duration = max(3.0, max(limits) / rate + 1.0)
    if duration > 30.0:
        raise RuntimeError("gravity startup ramp exceeds the 30 second commissioning hold budget")
    return duration


def measured_hold_metrics(reference, samples, enable_started_at=None):
    """Reject malformed feedback before computing hold acceptance metrics."""
    if len(reference) != 6 or not all(math.isfinite(q) for q in reference) or not samples:
        raise RuntimeError("measured-pose hold requires a finite reference and fresh samples")
    for sample in samples:
        if any(len(sample[key]) != 6 or not all(math.isfinite(v) for v in sample[key])
               for key in ("q", "dq")):
            raise RuntimeError("measured-pose hold requires finite six-axis feedback")
    errors = [max(abs(s["q"][i] - reference[i]) for s in samples) for i in range(6)]
    velocity = max(abs(v) for s in samples for v in s["dq"])
    if max(errors) > HOLD_POSITION_TOLERANCE_RAD:
        raise RuntimeError(f"measured-pose hold exceeded {HOLD_POSITION_TOLERANCE_RAD} rad")
    for sample in samples:
        transient = (enable_started_at is not None
                     and math.isfinite(sample.get("t", math.nan))
                     and 0 <= sample["t"] - enable_started_at <= .25)
        speed_limit = (.15 if transient else .05) if enable_started_at is not None else HOLD_VELOCITY_PEAK_RAD_S
        if max(abs(v) for v in sample["dq"]) > speed_limit:
            raise RuntimeError("measured-pose hold exceeded its bounded startup/stationary velocity limit")
    return errors, velocity


def cia402_steps(recipe, profile, positions, profile_sha256):
    """Validate an arm-bound, short sequence without changing any calibration."""
    if (recipe.get("schema_version") != 1 or recipe.get("profile_sha256") != profile_sha256
            or profile["bus"]["protocol"] != "cia402"):
        raise RuntimeError("CiA402 sequence requires its exact qualified hardware profile")
    reference = recipe.get("entry_position_rad", [])
    if (len(positions) != 6 or len(reference) != 6
            or any(not math.isfinite(v) for v in (*positions, *reference))
            or any(abs(a - b) > .01 for a, b in zip(positions, reference))):
        raise RuntimeError("CiA402 sequence requires its stationary entry pose within 0.01 rad")
    if any(abs(a - b) > tolerance for a, b, tolerance
           in zip(reference[:5], FOLDED[:5], FOLDED_TOLERANCE)):
        raise RuntimeError("CiA402 sequence entry must be the documented folded pose")
    raw = recipe.get("steps", [])
    # This first deployed recipe is deliberately limited to the tested order.
    if [s.get("joint") for s in raw] != ["joint_3", "joint_4", "joint_2"]:
        raise RuntimeError("CiA402 sequence requires qualified J3 -> J4 -> J2 order")
    target = list(positions)
    result = []
    for step, sign, bound in zip(raw, (-1, -1, 1), (.06, .06, .12)):
        axis = JOINTS.index(step["joint"])
        delta, duration = step["delta_rad"], step["duration_sec"]
        if (not math.isfinite(delta) or not .02 <= sign * delta <= bound
                or not isinstance(duration, int) or isinstance(duration, bool) or not 20 <= duration <= 30):
            raise RuntimeError("CiA402 sequence exceeds bounded travel/duration")
        target[axis] += delta
        for q, joint in zip(target, profile["joints"]):
            limits = joint["limits"]
            if not limits["position_lower_rad"] <= q <= limits["position_upper_rad"]:
                raise RuntimeError("CiA402 sequence target exceeds hardware profile")
        limits = profile["joints"][axis]["limits"]
        if (1.875 * abs(delta) / duration > limits["velocity_rad_s"]
                or 5.774 * abs(delta) / duration**2 > limits["acceleration_rad_s2"]):
            raise RuntimeError("CiA402 sequence exceeds quintic motion rates")
        result.append((f"cia402_{step['joint']}", target.copy(), duration))
    return result


def folded_entry_command(profile, positions):
    """Bound a Meow entry hold using the profile's feedback-only allowance."""
    if profile["bus"]["protocol"] != "meow" or len(positions) != 6:
        raise RuntimeError("folded entry margin requires six-axis Meow feedback")
    reference = []
    for q, joint in zip(positions, profile["joints"]):
        limits = joint["limits"]
        margin = limits.get("measured_position_margin_rad", 0.0)
        if (isinstance(margin, bool) or not isinstance(margin, (int, float))
                or not math.isfinite(margin) or not 0 <= margin <= .01):
            raise RuntimeError(f"invalid measured position margin for {joint['name']}")
        lower, upper = limits["position_lower_rad"], limits["position_upper_rad"]
        if (not math.isfinite(q) or not
                lower - margin - POSITION_EPSILON_RAD <= q <= upper + margin + POSITION_EPSILON_RAD):
            raise RuntimeError(f"folded feedback exceeds {joint['name']} profile allowance: {q}")
        reference.append(min(upper, max(lower, q)))
    return reference


def startup_steps(profile, positions, velocities, *, align_folded=False, motion_limits=None):
    """Return bounded FJT goals; only J1–J5 define the folded entry posture."""
    if (len(positions) != 6 or len(velocities) != 6
            or not all(math.isfinite(q) for q in positions)
            or any(not math.isfinite(v) or abs(v) > STARTUP_RECIPE["stopped_velocity_rad_s"]
                   for v in velocities)):
        raise RuntimeError("startup requires finite, stationary six-axis feedback")
    tolerances = FOLDED_TOLERANCE
    if align_folded:
        if profile["bus"]["protocol"] != "meow":
            raise RuntimeError("explicit folded alignment requires Meow")
        # J2/J3 still define the documented folded structure. Only explicitly
        # acknowledged placement of the base/wrist is aligned before unfolding.
        tolerances = [.15, .01, .01, .03, .05]
    if any(abs(a - b) > tolerance + POSITION_EPSILON_RAD for a, b, tolerance
           in zip(positions[:5], FOLDED[:5], tolerances)):
        raise RuntimeError(f"fixed startup requires J1–J5 folded reference: {positions}")
    j6 = profile["joints"][5]["limits"]
    if not j6["position_lower_rad"] <= positions[5] <= j6["position_upper_rad"]:
        raise RuntimeError("measured J6 is outside the selected profile")
    rates = resolve_motion_limits(profile, motion_limits)
    steps = []
    # Preserve the folded J2/J3/J4 values until each axis's turn, bounding
    # feedback within its allowed margin to the strict command endpoints.
    # The nominal fold is a posture check, not a goal that must be reached first.
    previous = folded_entry_command(profile, positions)
    target = previous.copy()
    for axis in (0, 4, 5):
        target[axis] = 0.0
    if align_folded:
        target[3] = 0.0
    if abs(positions[5]) > 0.01 or align_folded:
        duration = rest_to_rest_duration(previous, target, rates)
        steps.append(("align_folded" if align_folded else "align_j6", target.copy(), duration))
        previous = target.copy()
    for step in STARTUP_RECIPE["steps"]:
        axis, value = step["joint_index"], step["target_rad"]
        target[axis] = value
        duration = rest_to_rest_duration(previous, target, rates)
        steps.append((f"startup_j{axis + 1}", target.copy(), duration))
        previous = target.copy()
    return steps


class Probe(Node):
    def __init__(self, profile, record_commands=False, sequence=None, profile_sha256=None,
                 align_folded=False, motion_limits=None):
        super().__init__("hex_arm_real_commissioning")
        self.profile = profile
        self.motion_limits = resolve_motion_limits(profile, motion_limits)
        self.folded_entry_checked = False
        self.positions = {}
        self.velocity = {}
        self.received_at = 0.0
        self.samples = []
        self.commands = []
        self.action_feedback = []
        self.service_timeouts = []
        self.steps = []
        self.last_commanded_target = None
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
        self.hold_reference = None
        self.hold_samples_start = None
        self.sequence = sequence
        self.profile_sha256 = profile_sha256
        self.sequence_path_checks = []
        self.align_folded = align_folded

    def unlock_moveit_after_hold(self, readiness_token):
        if not readiness_token:
            return False
        self.moveit_ready_publisher = self.create_publisher(
            String, "/hex_arm/internal/moveit_startup_ready",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.moveit_ready_publisher.publish(String(data=readiness_token))
        if not (self.move_group.wait_for_server(timeout_sec=5.0)
                and self.trajectory_executor.wait_for_server(timeout_sec=5.0)):
            raise RuntimeError("MoveIt did not acknowledge the verified startup execution handoff")
        return True

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
            steps = startup_steps(
                    self.profile, self.q(), [self.velocity[name] for name in JOINTS],
                    align_folded=getattr(self, "align_folded", False),
                    motion_limits=getattr(self, "motion_limits", None))
            self.check_point(folded_entry_command(self.profile, self.q()))
            for _, target, _ in steps:
                self.check_point(target)
            self.folded_entry_checked = True
        else:
            # CiA402 starts by holding the measured pose. Check the position
            # window and stationary feedback before requesting motor torque.
            self.check_point(self.q())
            velocities = [self.velocity.get(name) for name in JOINTS]
            if any(v is None or not math.isfinite(v)
                   or abs(v) > STARTUP_RECIPE["stopped_velocity_rad_s"]
                   for v in velocities):
                raise RuntimeError("measured-pose activation requires stationary six-axis feedback")
            self.hold_reference = self.q()
            self.hold_samples_start = len(self.samples)
            if getattr(self, "sequence", None) is not None:
                steps = cia402_steps(self.sequence, self.profile, self.hold_reference, self.profile_sha256)
                self.check_sequence_path(self.hold_reference, steps)
        self.hold_reference = self.q()
        self.hold_samples_start = len(self.samples)
        print("Controllers loaded and configured; DDS ready while hardware INACTIVE", flush=True)
        request = SetHardwareComponentState.Request()
        request.name = "FireflyY6System"; request.target_state.id = 3
        self.hardware_activation_requested = True
        self.enable_started_at = time.monotonic()
        result = self.wait(self.set_hardware.call_async(request), 10.0)
        if not result.ok or result.state.id != 3:
            raise RuntimeError("hardware activation failed")
        request = SwitchController.Request()
        # Both protocols prove gravity-supported holding before trajectories.
        request.activate_controllers = ["joint_state_broadcaster"]
        request.strictness = SwitchController.Request.STRICT
        request.timeout.sec = 3
        if not self.wait(self.switch.call_async(request), 5.0).ok:
            raise RuntimeError("failed to activate the prepared controller group")
        message = "State broadcaster active; verifying measured-pose hold before trajectories"
        print(message, flush=True)

    def activate_trajectory_controller(self):
        request = SwitchController.Request()
        request.activate_controllers = ["firefly_arm_controller"]
        request.strictness = SwitchController.Request.STRICT
        request.timeout.sec = 3
        if not self.wait(self.switch.call_async(request), 5.0).ok:
            raise RuntimeError("failed to activate trajectory controller after measured-pose hold")

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
        # Never merge partial messages with cached axes or accept NaN through
        # max(): either case could certify a hold from invalid feedback.
        if (len(message.name) != 6 or set(message.name) != set(JOINTS)
                or len(message.position) != 6 or len(message.velocity) != 6
                or not all(math.isfinite(v) for v in (*message.position, *message.velocity))):
            raise RuntimeError("startup requires complete finite six-axis joint feedback")
        self.positions = dict(zip(message.name, message.position))
        self.velocity = dict(zip(message.name, message.velocity))
        self.received_at = time.monotonic()
        if all(name in self.positions for name in JOINTS):
            self.samples.append({"t": self.received_at, "q": self.q(),
                                 "dq": [self.velocity[name] for name in JOINTS]})

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

    def spin_hold(self, reference, seconds, start, enable_started_at=None):
        # Check every new batch during the hold, including activation feedback.
        if len(self.samples) > start:
            measured_hold_metrics(reference, self.samples[start:], enable_started_at)
        cursor = len(self.samples)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.spin(min(.05, max(0.0, end - time.monotonic())))
            if len(self.samples) > cursor:
                measured_hold_metrics(reference, self.samples[cursor:], enable_started_at)
                cursor = len(self.samples)
        return measured_hold_metrics(reference, self.samples[start:], enable_started_at)

    def check_point(self, positions):
        if len(positions) != 6:
            raise RuntimeError("trajectory must contain six joints")
        for q, joint in zip(positions, self.profile["joints"]):
            limits = joint["limits"]
            if not math.isfinite(q) or not (
                limits["position_lower_rad"] - POSITION_EPSILON_RAD <= q
                <= limits["position_upper_rad"] + POSITION_EPSILON_RAD
            ):
                raise RuntimeError(f"trajectory exceeds {joint['name']} profile authority: {q}")

    def direct_step(self, target, duration, label, *, retime=False):
        self.check_point(target)
        started_at = time.monotonic()
        initial = self.q()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        # Preserve the prior settled position reference while clearing its
        # derivatives. Rebasing to compliant feedback would drop PD support.
        reference = self.last_commanded_target or initial
        if self.last_commanded_target is None and getattr(self, "folded_entry_checked", False):
            reference = folded_entry_command(self.profile, reference)
        self.check_point(reference)
        if any(abs(a-b) > HOLD_POSITION_TOLERANCE_RAD for a, b in zip(reference, initial)):
            raise RuntimeError("direct trajectory requires a settled prior reference")
        if retime:
            # The first goal uses fresh measured feedback; later goals retain
            # the settled command reference rather than rebasing PD support.
            duration = rest_to_rest_duration(reference, target, self.motion_limits)
        for a, b, joint in zip(reference, target, self.profile["joints"]):
            limits = self.motion_limits[joint["name"]]
            if (1.875 * abs(b-a) / duration > limits["velocity_rad_s"]
                    or 5.774 * abs(b-a) / duration**2 > limits["acceleration_rad_s2"]):
                raise RuntimeError("fixed trajectory exceeds quintic motion rates")
        goal.trajectory.points = rest_to_rest_points(reference, target, duration)
        path_tolerance = getattr(self, "shutdown_path_tolerance_rad", None)
        set_path_tolerance(goal, path_tolerance)
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
                  "command_reference": list(reference),
                  "target": list(target), "actual": self.q(),
                  "status": result.status, "error_code": result.result.error_code,
                  "message": result.result.error_string}
        if path_tolerance is not None:
            record["path_tolerance_rad"] = path_tolerance
        feedback = self.action_feedback[feedback_start:]
        if feedback:
            record["max_tracking_error_rad"] = [
                max(abs(sample["error"][i]) for sample in feedback) for i in range(6)]
        self.steps.append(record)
        print(json.dumps(record), flush=True)
        if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code != 0:
            raise RuntimeError(f"fixed startup FJT failed: {record}")
        self.last_commanded_target = list(target)
        return record

    def is_valid(self, positions, allowed_contacts=frozenset()):
        req = GetStateValidity.Request()
        req.group_name = "arm"
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = list(positions)
        result = self.wait(self.validity.call_async(req), 5.0)
        if not result.valid:
            pairs = {tuple(sorted((c.contact_body_1, c.contact_body_2))) for c in result.contacts}
            if not pairs or not pairs.issubset(allowed_contacts):
                raise RuntimeError(f"strict MoveIt state rejected: {positions}, contacts={pairs}")
        return result.valid

    def check_sequence_path(self, initial, steps):
        if not self.validity.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("strict MoveIt collision service required before sequence activation")
        previous = list(initial)
        for index, (label, target, _) in enumerate(steps):
            for k in range(41):
                q = [a + (b-a) * k/40 for a, b in zip(previous, target)]
                self.check_point(q)
                strict = self.is_valid(q, FOLDED_CONTACTS if index == 0 and k < 40 else frozenset())
                self.sequence_path_checks.append({"step": label, "q": q, "strict_valid": strict})
            previous = target

    def plan_and_execute(self, target, velocity_scaling=1.0, acceleration_scaling=1.0):
        self.last_plan_diagnostics = None
        for value in (velocity_scaling, acceleration_scaling):
            if isinstance(value, bool) or not math.isfinite(value) or not 0 < value <= 1:
                raise RuntimeError("planning scaling factors must be finite and within (0, 1]")
        initial = self.q()
        self.check_point(target)
        self.is_valid(self.q())
        self.is_valid(target)
        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.pipeline_id = "ompl"
        goal.request.num_planning_attempts = 2
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = velocity_scaling
        goal.request.max_acceleration_scaling_factor = acceleration_scaling
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
        execution_timeout = planned_execution_timeout(trajectory.joint_trajectory.points)
        for point in trajectory.joint_trajectory.points:
            self.check_point(point.positions)
            if (len(point.velocities) != 6 or len(point.accelerations) != 6
                    or not all(math.isfinite(value) for value in [*point.velocities, *point.accelerations])):
                raise RuntimeError("planned dynamics must contain six finite velocities and accelerations")
            for i, value in enumerate(point.velocities):
                if abs(value) > self.profile["joints"][i]["limits"]["velocity_rad_s"] + 1e-5:
                    raise RuntimeError("planned velocity exceeds hardware profile")
            for i, value in enumerate(point.accelerations):
                if abs(value) > self.profile["joints"][i]["limits"]["acceleration_rad_s2"] + 1e-5:
                    raise RuntimeError("planned acceleration exceeds hardware profile")
            self.is_valid(point.positions)
        print(f"Strict MoveIt plan checked: {len(trajectory.joint_trajectory.points)} points", flush=True)
        self.last_plan_diagnostics = {
            "target": target, "initial": initial, "plan_points": len(trajectory.joint_trajectory.points),
            "execution_timeout_sec": execution_timeout,
            "velocity_scaling": velocity_scaling, "acceleration_scaling": acceleration_scaling,
            "planned_duration_sec": (trajectory.joint_trajectory.points[-1].time_from_start.sec
                                     + trajectory.joint_trajectory.points[-1].time_from_start.nanosec * 1e-9),
            "planned_peak_velocity_rad_s": [max(abs(p.velocities[i]) for p in trajectory.joint_trajectory.points)
                                            for i in range(6)],
            "planned_peak_acceleration_rad_s2": [max(abs(p.accelerations[i]) for p in trajectory.joint_trajectory.points)
                                                for i in range(6)],
        }
        execution = self.execute_planned_trajectory(trajectory, execution_timeout)
        self.last_commanded_target = list(target)
        self.spin(1.0)
        return {**self.last_plan_diagnostics, "actual": self.q(), **execution}

    def execute_planned_trajectory(self, trajectory, timeout):
        execution = ExecuteTrajectory.Goal()
        execution.trajectory = trajectory
        handle = self.wait(self.trajectory_executor.send_goal_async(execution), 5.0)
        if not handle.accepted:
            raise RuntimeError("MoveIt execution rejected")
        self.active_goal = handle
        wrapped = self.wait(handle.get_result_async(), timeout)
        self.active_goal = None
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or wrapped.result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"MoveIt execution failed: {wrapped.result.error_code.val}")
        return {"moveit_error_code": wrapped.result.error_code.val}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--motion-limits", default="",
                        help="Internal JSON of this launch's effective MoveIt velocity/acceleration caps")
    parser.add_argument("--allow-motion", action="store_true", required=True)
    parser.add_argument("--moveit", action="store_true")
    parser.add_argument("--activate-controllers", action="store_true",
                        help="Prepare controllers, then enable hardware and run the protocol startup path")
    parser.add_argument("--hold-current", action="store_true",
                        help="Verify a stationary measured-pose hold before activating trajectories")
    parser.add_argument("--cia402-sequence", type=Path,
                        help="Arm-bound, commissioned J3/J4/J2 sequence; requires strict MoveIt")
    parser.add_argument("--return-to-start", action="store_true",
                        help="Reverse the CiA402 sequence after successful validation")
    parser.add_argument("--allow-enable-transient", action="store_true",
                        help="Bound Meow enable/ramp speeds to 0.15 for 0.25 s then 0.05 rad/s; require settled hold before trajectories")
    parser.add_argument("--align-folded", action="store_true",
                        help="Explicitly align bounded Meow base/wrist placement before the fixed startup")
    parser.add_argument("--record-commands", action="store_true",
                        help="Optionally sample commands with best-effort QoS for diagnostics")
    parser.add_argument("--moveit-ready-token", default="",
                        help="Internal per-launch execution handoff, published only after the stationary hold")
    parser.add_argument("--deactivate-after", action="store_true",
                        help="Stop controllers and disable hardware after this trial")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    profile = yaml.safe_load(args.profile.read_text())
    motion_limits = resolve_motion_limits(
        profile, json.loads(args.motion_limits) if args.motion_limits else None)
    if args.moveit_ready_token and (not args.activate_controllers
                                   or not re.fullmatch(r"[0-9a-f]{32}", args.moveit_ready_token)):
        raise RuntimeError("MoveIt handoff requires controller activation and a per-launch token")
    if args.allow_enable_transient and (not args.activate_controllers or profile["bus"]["protocol"] != "meow"):
        raise RuntimeError("enable transient allowance requires explicit Meow activation")
    if args.align_folded and (args.hold_current or profile["bus"]["protocol"] != "meow"):
        raise RuntimeError("--align-folded requires the fixed Meow startup")
    if not profile["calibrated"] or (not args.hold_current and profile["bus"]["protocol"] != "meow"):
        raise RuntimeError("requires a calibrated profile; fixed startup requires Meow")
    if args.hold_current and args.moveit and not args.cia402_sequence:
        raise RuntimeError("--moveit requires the fixed ready sequence")
    if args.cia402_sequence and (not args.hold_current or not args.activate_controllers):
        raise RuntimeError("CiA402 sequence requires measured-pose controller activation")
    if args.return_to_start and not args.cia402_sequence:
        raise RuntimeError("--return-to-start requires a CiA402 sequence")
    profile_sha256 = hashlib.sha256(args.profile.read_bytes()).hexdigest()
    sequence = yaml.safe_load(args.cia402_sequence.read_text()) if args.cia402_sequence else None
    if sequence is not None:
        cia402_steps(sequence, profile, sequence.get("entry_position_rad", []), profile_sha256)
    hold_duration = measured_hold_duration(profile)
    rclpy.init()
    node = Probe(profile, record_commands=args.record_commands, sequence=sequence,
                 profile_sha256=profile_sha256, align_folded=args.align_folded,
                 motion_limits=motion_limits)
    report = {"time": datetime.now(timezone.utc).isoformat(),
              "profile": str(args.profile), "profile_sha256": profile_sha256, "steps": node.steps,
              "motion_limits": motion_limits,
              "hold_position_tolerance_rad": HOLD_POSITION_TOLERANCE_RAD,
              "hold_velocity_peak_rad_s": HOLD_VELOCITY_PEAK_RAD_S}
    if sequence is not None:
        report["sequence"] = sequence
        report["sequence_sha256"] = hashlib.sha256(args.cia402_sequence.read_bytes()).hexdigest()
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
        hold_reference = node.hold_reference
        if hold_reference is None:
            hold_reference = node.q()
        hold_samples_start = (
            node.hold_samples_start if node.hold_samples_start is not None else len(node.samples)
        )
        max_error_rad, max_velocity_rad_s = node.spin_hold(
            hold_reference, hold_duration, hold_samples_start,
            node.enable_started_at if args.allow_enable_transient else None)
        held = node.samples[hold_samples_start:]
        report["measured_hold"] = {
            "enable_transient_allowed": args.allow_enable_transient,
            "enable_started_at": getattr(node, "enable_started_at", None),
            "seconds": hold_duration,
            "reference_q": hold_reference, "q": node.q(), "samples": len(held),
            "max_error_rad": max_error_rad, "max_velocity_rad_s": max_velocity_rad_s,
        }
        if args.allow_enable_transient:
            settle_start = len(node.samples)
            errors, velocity = node.spin_hold(hold_reference, .5, settle_start)
            report["measured_hold"]["settled_max_error_rad"] = errors
            report["measured_hold"]["settled_max_velocity_rad_s"] = velocity
        if args.activate_controllers:
            post_activation_start = len(node.samples)
            node.activate_trajectory_controller()
            node.spin_hold(hold_reference, 1.0, post_activation_start)
            after_activation = node.samples[post_activation_start:]
            errors_after, max_velocity_after_activation = measured_hold_metrics(
                hold_reference, after_activation)
            max_error_after_activation = max(errors_after)
            report["measured_hold"]["post_controller_max_error_rad"] = max_error_after_activation
            report["measured_hold"]["post_controller_max_velocity_rad_s"] = max_velocity_after_activation
        if args.hold_current:
            if sequence is not None:
                steps = cia402_steps(sequence, profile, hold_reference, profile_sha256)
                for label, target, duration in steps:
                    node.direct_step(target, duration, label)
                    node.is_valid(node.q())
                ready = steps[-1][1]
                start = len(node.samples)
                errors, velocity = node.spin_hold(ready, READY_HOLD_SECONDS, start)
                report["ready_hold"] = {"seconds": READY_HOLD_SECONDS, "target": ready, "q": node.q(),
                                        "max_error_rad": errors, "max_velocity_rad_s": velocity}
                report["moveit_execution_unlocked"] = node.unlock_moveit_after_hold(args.moveit_ready_token)
                if args.moveit:
                    if not (node.move_group.wait_for_server(timeout_sec=5.0)
                            and node.trajectory_executor.wait_for_server(timeout_sec=5.0)):
                        raise RuntimeError("MoveIt action servers unavailable")
                    target = ready.copy(); target[1] += .015
                    report["moveit"] = node.plan_and_execute(target)
                    report["moveit_return"] = node.plan_and_execute(ready)
                if args.return_to_start:
                    previous = [hold_reference, *(step[1] for step in steps[:-1])]
                    for index in reversed(range(len(steps))):
                        label, _, duration = steps[index]
                        node.direct_step(previous[index], duration, f"return_{label}")
                        node.is_valid(node.q(), FOLDED_CONTACTS if index == 0 else frozenset())
                    start = len(node.samples)
                    errors, velocity = node.spin_hold(hold_reference, 1.0, start)
                    report["returned_hold"] = {"max_error_rad": errors, "max_velocity_rad_s": velocity}
            else:
                report["moveit_execution_unlocked"] = node.unlock_moveit_after_hold(args.moveit_ready_token)
        else:
            steps = startup_steps(profile, node.q(), [node.velocity[name] for name in JOINTS],
                                  align_folded=args.align_folded, motion_limits=node.motion_limits)
            node.folded_entry_checked = True
            # Check every target before sending the first motion command.
            for _, target, _ in steps:
                node.check_point(target)
            for label, target, duration in steps:
                node.direct_step(target, duration, label, retime=True)
            start = len(node.samples)
            node.spin(READY_HOLD_SECONDS)
            held = node.samples[start:]
            report["ready_hold"] = {
                "seconds": READY_HOLD_SECONDS, "samples": len(held), "last_q": node.q(),
                "max_error_rad": [max(abs(s["q"][i] - READY[i]) for s in held) for i in range(6)],
                "max_velocity_rad_s": max(abs(v) for s in held for v in s["dq"]),
            }
            print(json.dumps(report["ready_hold"]), flush=True)
            try:
                measured_hold_metrics(READY, held)
            except RuntimeError as error:
                raise RuntimeError(f"ready hold verification failed: {error}") from error
            report["moveit_execution_unlocked"] = node.unlock_moveit_after_hold(args.moveit_ready_token)
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
        report["sequence_path_checks"] = node.sequence_path_checks
        args.output.write_text(json.dumps(report, indent=2) + "\n")
        node.destroy_node()
        rclpy.shutdown()
        if cleanup_error is not None and trial_motion_passed:
            raise RuntimeError(f"trial motion passed, but deactivation failed: {cleanup_error}")


if __name__ == "__main__":
    main()
