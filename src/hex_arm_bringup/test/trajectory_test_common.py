from __future__ import annotations

import math
import os
import signal
import subprocess
import sys
import time

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


JOINTS = [f"joint_{index}" for index in range(1, 7)]


def spin_until(node: Node, future, timeout: float):
    deadline = time.monotonic() + timeout
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not future.done():
        raise TimeoutError("ROS operation timed out")
    return future.result()


class TrajectoryProbe(Node):
    def __init__(self) -> None:
        super().__init__("firefly_y6_trajectory_probe")
        self.client = ActionClient(
            self, FollowJointTrajectory, "/firefly_arm_controller/follow_joint_trajectory")
        self.positions: dict[str, float] = {}
        self.create_subscription(JointState, "/joint_states", self._state, 10)

    def _state(self, message: JointState) -> None:
        self.positions.update(zip(message.name, message.position, strict=False))

    def send(self, position: list[float], duration: float):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        point = JointTrajectoryPoint()
        point.positions = position
        point.velocities = [0.0] * 6
        seconds = int(duration)
        point.time_from_start.sec = seconds
        point.time_from_start.nanosec = int((duration - seconds) * 1e9)
        goal.trajectory.points = [point]
        handle = spin_until(self, self.client.send_goal_async(goal), 10.0)
        if not handle.accepted:
            raise RuntimeError("trajectory goal was rejected")
        return handle


def run(backend: str, launch_timeout: float, goal_timeout: float) -> None:
    command = [
        "ros2", "launch", "hex_arm_bringup", f"{backend}.launch.py",
        "use_rviz:=false",
    ]
    if backend == "gz":
        command.append("headless:=true")
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    rclpy.init()
    node = TrajectoryProbe()
    failed = False
    try:
        deadline = time.monotonic() + launch_timeout
        while not node.client.wait_for_server(timeout_sec=0.25):
            if process.poll() is not None:
                output = process.stdout.read() if process.stdout else ""
                raise RuntimeError(f"bringup exited early ({process.returncode})\n{output}")
            if time.monotonic() >= deadline:
                raise TimeoutError("FollowJointTrajectory action did not appear")

        target = [0.15, 0.25, 1.25, -0.20, 0.15, -0.10]
        handle = node.send(target, 2.0)
        result = spin_until(node, handle.get_result_async(), goal_timeout)
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"trajectory failed with status {result.status}: {result.result.error_string}")
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.05)
        if any(name not in node.positions for name in JOINTS):
            raise RuntimeError("/joint_states did not contain all six joints")
        tolerance = 0.03 if backend == "mock" else 0.10
        errors = [abs(node.positions[name] - expected) for name, expected in zip(JOINTS, target, strict=True)]
        if not all(math.isfinite(error) and error <= tolerance for error in errors):
            raise RuntimeError(f"final joint error exceeds tolerance: {errors}")

        second_target = [-0.20, 0.05, 0.90, 0.25, -0.10, 0.20]
        handle = node.send(second_target, 2.0)
        result = spin_until(node, handle.get_result_async(), goal_timeout)
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(
                f"second trajectory failed with status {result.status}: {result.result.error_string}")
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.05)
        errors = [
            abs(node.positions[name] - expected)
            for name, expected in zip(JOINTS, second_target, strict=True)
        ]
        if not all(math.isfinite(error) and error <= tolerance for error in errors):
            raise RuntimeError(f"second final joint error exceeds tolerance: {errors}")

        cancel_handle = node.send([-0.5, -0.3, 0.8, 0.4, -0.2, 0.5], 8.0)
        cancel_deadline = time.monotonic() + 0.5
        while time.monotonic() < cancel_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        cancel_response = spin_until(node, cancel_handle.cancel_goal_async(), 5.0)
        if not cancel_response.goals_canceling:
            raise RuntimeError("trajectory cancellation was not accepted")
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

