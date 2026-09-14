#!/usr/bin/env python3
from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from action_msgs.msg import GoalStatus
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetStateValidity
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


JOINTS = [f"joint_{index}" for index in range(1, 7)]
SURVEYED_START = [0.0, -1.570, 1.570, 0.0, 0.0, 0.0]
SURVEYED_TARGET = [0.0, -1.560, 3.120, 0.0, 0.0, 0.05]
ACTIVE_COLLISION_GUARD = [0.0, 1.570, 1.570, 0.0, 0.0, 0.0]
LAUNCH_FIXTURE = Path(__file__).with_name("plan_only_move_group.launch.py")


def _spin_until(node: Node, future, timeout: float):
    deadline = time.monotonic() + timeout
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not future.done():
        raise TimeoutError("offline MoveIt operation timed out")
    return future.result()


class PlanOnlyProbe(Node):
    def __init__(self) -> None:
        super().__init__("firefly_y6_plan_only_probe")
        self.move_group = ActionClient(self, MoveGroup, "/move_action")
        self.validity = self.create_client(GetStateValidity, "/check_state_validity")

    def state_validity(self, positions: list[float]):
        request = GetStateValidity.Request()
        request.group_name = "arm"
        request.robot_state.is_diff = False
        request.robot_state.joint_state.name = JOINTS
        request.robot_state.joint_state.position = positions
        return _spin_until(self, self.validity.call_async(request), 10.0)

    def plan(self, start: list[float], target: list[float]):
        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.pipeline_id = "ompl"
        goal.request.num_planning_attempts = 3
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.1
        goal.request.start_state.is_diff = False
        goal.request.start_state.joint_state.name = JOINTS
        goal.request.start_state.joint_state.position = start
        constraints = Constraints()
        constraints.name = "surveyed_fold_plan_only_target"
        constraints.joint_constraints = [
            JointConstraint(
                joint_name=name,
                position=position,
                tolerance_above=0.001,
                tolerance_below=0.001,
                weight=1.0,
            )
            for name, position in zip(JOINTS, target, strict=True)
        ]
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = True
        goal.planning_options.planning_scene_diff.is_diff = True

        handle = _spin_until(self, self.move_group.send_goal_async(goal), 10.0)
        if not handle.accepted:
            raise RuntimeError("offline plan-only MoveGroup goal was rejected")
        wrapped = _spin_until(self, handle.get_result_async(), 30.0)
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(
                f"offline plan-only action finished with status {wrapped.status}"
            )
        result = wrapped.result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                "offline plan-only MoveIt failed with code "
                f"{result.error_code.val}: {result.error_code.message}"
            )
        if not result.planned_trajectory.joint_trajectory.points:
            raise RuntimeError("offline plan-only MoveIt returned an empty trajectory")
        return result


def _contact_pairs(response) -> set[tuple[str, str]]:
    return {
        tuple(sorted((contact.contact_body_1, contact.contact_body_2)))
        for contact in response.contacts
    }


def test_plan_only_overlay_accepts_only_the_surveyed_mesh_contacts() -> None:
    # A unique domain prevents this fixture from discovering a developer's
    # running MoveIt graph. The fixture itself contains no hardware processes.
    test_domain = os.environ.get(
        "HEX_ARM_TEST_ROS_DOMAIN_ID", str(120 + os.getpid() % 80)
    )
    os.environ["ROS_DOMAIN_ID"] = test_domain
    child_environment = os.environ.copy()
    process = subprocess.Popen(
        ["ros2", "launch", str(LAUNCH_FIXTURE)],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
        env=child_environment,
    )
    rclpy.init()
    node = PlanOnlyProbe()
    failed = False
    try:
        deadline = time.monotonic() + 30.0
        while not (
            node.move_group.wait_for_server(timeout_sec=0.25)
            and node.validity.wait_for_service(timeout_sec=0.25)
        ):
            if process.poll() is not None:
                output = process.stdout.read() if process.stdout else ""
                raise RuntimeError(
                    f"offline MoveIt launch exited early ({process.returncode})\n{output}"
                )
            if time.monotonic() >= deadline:
                raise TimeoutError("offline MoveGroup action or validity service did not appear")

        surveyed = node.state_validity(SURVEYED_START)
        if not surveyed.valid or surveyed.contacts:
            raise RuntimeError(
                "plan-only overlay did not admit the surveyed fold: "
                f"contacts={_contact_pairs(surveyed)}"
            )

        guard = node.state_validity(ACTIVE_COLLISION_GUARD)
        guard_pairs = _contact_pairs(guard)
        if guard.valid or guard_pairs != {
            ("base_link", "link_3"),
            ("base_link", "link_4"),
            ("base_link", "link_5"),
            ("link_1", "link_3"),
            ("link_1", "link_4"),
        }:
            raise RuntimeError(
                "plan-only overlay disabled an unrelated collision pair: "
                f"valid={guard.valid}, contacts={guard_pairs}"
            )

        node.plan(SURVEYED_START, SURVEYED_TARGET)
    except BaseException:
        failed = True
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        output = process.stdout.read() if process.stdout else ""
        if failed or process.returncode not in (0, -signal.SIGINT, 130):
            print(output, file=sys.stderr)
        if "Added FollowJointTrajectory controller" in output:
            raise RuntimeError(
                "plan-only move_group unexpectedly configured a trajectory "
                "execution controller client"
            )
        crash_markers = ("Segmentation fault", "exit code -11", "process has died")
        if not failed and any(marker in output for marker in crash_markers):
            print(output, file=sys.stderr)
            raise RuntimeError("ordered-shutdown move_group exited uncleanly")
        if not failed and "process has finished cleanly" not in output:
            print(output, file=sys.stderr)
            raise RuntimeError("ordered-shutdown move_group clean exit was not observed")
        if not failed and "[rclcpp]: signal_handler" in output:
            print(output, file=sys.stderr)
            raise RuntimeError("move_group unexpectedly installed the rclcpp signal handler")
