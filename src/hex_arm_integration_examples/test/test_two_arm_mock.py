"""Exercise actual controllers: isolation, wrong-arm rejection and fault scope."""
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest
import rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


def complete(node, future, timeout=15.0):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    assert future.done(), "ROS operation did not complete"
    return future.result()


def wait_for(node, predicate, timeout=15.0):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    assert predicate(), "expected joint state did not arrive"


@pytest.mark.parametrize("launch_name", ["isolated_arms", "shared_manager"])
def test_two_arms_are_independent(tmp_path, monkeypatch, launch_name):
    monkeypatch.setenv("ROS_DOMAIN_ID", str(50 + os.getpid() % 100))
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    log_path = tmp_path / f"{launch_name}.log"
    with log_path.open("w") as log:
        process = subprocess.Popen(
            ["ros2", "launch", "hex_arm_integration_examples", launch_name + ".launch.py"],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node("dual_arm_probe")
    positions = {}
    states = node.create_subscription(JointState, "/joint_states",
        lambda message: positions.update(zip(message.name, message.position)), 10)
    clients = {}
    for side in ("left", "right"):
        controller = f"/{side}/firefly_arm_controller" if launch_name == "isolated_arms" else f"/{side}_arm_controller"
        clients[side] = ActionClient(node, FollowJointTrajectory, controller + "/follow_joint_trajectory")

    def send(side, target, prefix=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [f"{prefix if prefix is not None else side + '_'}joint_{index}" for index in range(1, 7)]
        goal.trajectory.points = [JointTrajectoryPoint(positions=target, velocities=[0.0] * 6)]
        goal.trajectory.points[0].time_from_start.sec = 1
        return complete(node, clients[side].send_goal_async(goal))

    def execute(side, target):
        handle = send(side, target)
        assert handle.accepted
        result = complete(node, handle.get_result_async())
        assert result.status == GoalStatus.STATUS_SUCCEEDED, result.result.error_string
        wait_for(node, lambda: all(abs(positions.get(f"{side}_joint_{index}", 100.0) - target[index - 1]) < 0.01 for index in range(1, 7)))

    try:
        for client in clients.values():
            assert client.wait_for_server(timeout_sec=40.0), log_path.read_text()
        wait_for(node, lambda: len(positions) == 12)
        assert not send("left", [0.0] * 6, prefix="right_").accepted
        left_target = [0.1, 0.2, 0.3, -0.1, 0.05, -0.05]
        execute("left", left_target)
        assert all(abs(positions[f"right_joint_{index}"]) < 0.001 for index in range(1, 7))
        execute("right", [-0.1, 0.1, 0.2, 0.1, -0.05, 0.05])
        assert all(abs(positions[f"left_joint_{index}"] - left_target[index - 1]) < 0.01 for index in range(1, 7))
        if launch_name == "isolated_arms":
            # Stop the actual left manager; the enclosing launch and right arm
            # must remain operational. Never signal a process outside this launch.
            found = False
            for entry in Path("/proc").iterdir():
                if not entry.name.isdigit():
                    continue
                try:
                    arguments = (entry / "cmdline").read_bytes().split(b"\0")
                    if (arguments and arguments[0].endswith(b"/ros2_control_node")
                            and b"__ns:=/left" in arguments and os.getpgid(int(entry.name)) == process.pid):
                        os.kill(int(entry.name), signal.SIGINT)
                        found = True
                        break
                except (OSError, ProcessLookupError):
                    continue
            assert found, "could not identify this launch's left controller manager"
            execute("right", [0.15, -0.1, 0.15, -0.1, 0.1, 0.0])
            assert process.poll() is None, log_path.read_text()
    finally:
        for client in clients.values():
            client.destroy()
        node.destroy_subscription(states)
        node.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=10)
        print(log_path.read_text())
