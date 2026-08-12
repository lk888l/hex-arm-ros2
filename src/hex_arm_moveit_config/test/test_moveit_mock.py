#!/usr/bin/env python3
from __future__ import annotations

import math
import os
import signal
import subprocess
import sys
import time

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINTS = [f"joint_{index}" for index in range(1, 7)]
TARGET = [0.20, 0.20, 1.30, -0.20, 0.15, 0.10]


def spin_until(node: Node, future, timeout: float):
    deadline = time.monotonic() + timeout
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not future.done():
        raise TimeoutError("MoveIt operation timed out")
    return future.result()


class MoveItProbe(Node):
    def __init__(self) -> None:
        super().__init__("firefly_y6_moveit_probe")
        self.client = ActionClient(self, MoveGroup, "/move_action")
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/firefly_arm_controller/follow_joint_trajectory",
        )
        self.positions: dict[str, float] = {}
        self.create_subscription(JointState, "/joint_states", self._state, 10)

    def _state(self, message: JointState) -> None:
        self.positions.update(zip(message.name, message.position, strict=False))

    def request(self, target: list[float], *, plan_only: bool):
        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.pipeline_id = "ompl"
        goal.request.num_planning_attempts = 3
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.1
        goal.request.start_state.is_diff = True
        constraints = Constraints()
        constraints.name = "mock_joint_target"
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
        goal.planning_options.plan_only = plan_only
        goal.planning_options.planning_scene_diff.is_diff = True

        handle = spin_until(self, self.client.send_goal_async(goal), 10.0)
        if not handle.accepted:
            raise RuntimeError("MoveGroup goal was rejected")
        wrapped = spin_until(self, handle.get_result_async(), 30.0)
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"MoveGroup action finished with status {wrapped.status}")
        result = wrapped.result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f"MoveIt failed with code {result.error_code.val}: {result.error_code.message}"
            )
        if not result.planned_trajectory.joint_trajectory.points:
            raise RuntimeError("MoveIt returned an empty joint trajectory")
        return result


def main() -> None:
    # Keep the integration test isolated from a developer's concurrently
    # running RViz/mock session on the default ROS domain.
    test_domain = os.environ.get(
        "HEX_ARM_TEST_ROS_DOMAIN_ID", str(100 + os.getpid() % 100)
    )
    os.environ["ROS_DOMAIN_ID"] = test_domain
    child_environment = os.environ.copy()
    process = subprocess.Popen(
        [
            "ros2",
            "launch",
            "hex_arm_moveit_config",
            "moveit_mock.launch.py",
            "use_rviz:=false",
            "limits_profile:=sim",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
        env=child_environment,
    )
    rclpy.init()
    node = MoveItProbe()
    failed = False
    try:
        deadline = time.monotonic() + 30.0
        while not (
            node.client.wait_for_server(timeout_sec=0.25)
            and node.trajectory_client.wait_for_server(timeout_sec=0.25)
        ):
            if process.poll() is not None:
                output = process.stdout.read() if process.stdout else ""
                raise RuntimeError(f"MoveIt launch exited early ({process.returncode})\n{output}")
            if time.monotonic() >= deadline:
                raise TimeoutError("MoveGroup or trajectory-controller action did not appear")

        # The probe may discover the controller before move_group's newly
        # constructed controller handle has completed DDS action discovery.
        # Let that independent client settle before the first execution.
        time.sleep(1.0)

        node.request(TARGET, plan_only=True)
        node.request(TARGET, plan_only=False)
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.05)
        if any(name not in node.positions for name in JOINTS):
            raise RuntimeError("/joint_states did not contain all six joints")
        errors = [
            abs(node.positions[name] - expected)
            for name, expected in zip(JOINTS, TARGET, strict=True)
        ]
        if not all(math.isfinite(error) and error <= 0.03 for error in errors):
            raise RuntimeError(f"MoveIt mock execution joint error exceeds tolerance: {errors}")
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


def test_moveit_mock() -> None:
    main()


if __name__ == "__main__":
    main()
