"""Plan and execute through renamed, nested-namespace MoveIt and controllers."""
import os
import signal
import subprocess
import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml

from action_msgs.msg import GoalStatus
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from hex_arm_bringup.model import write_rviz_config


def test_namespaced_moveit_rviz_targets_the_planning_group():
    source = Path(get_package_share_directory("hex_arm_moveit_config")) / "config/moveit.rviz"
    output = write_rviz_config(source, namespace="/robot/left", joint_prefix="left_",
                               planning_group="left_arm", use_sim_time=True)
    config = yaml.safe_load(Path(output).read_text())
    display = next(item for item in config["Visualization Manager"]["Displays"]
                   if item["Class"] == "moveit_rviz_plugin/MotionPlanning")
    assert display["Move Group Namespace"] == "/robot/left"
    assert display["Planning Request"]["Planning Group"] == "left_arm"
    assert display["Planning Scene Topic"] == "/robot/left/monitored_planning_scene"
    assert display["Planned Path"]["Trajectory Topic"] == "/robot/left/display_planned_path"
    assert display["Planned Path"]["Use Sim Time"] is True


def test_namespaced_moveit_executes_prefixed_arm(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "175")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    log_path = tmp_path / "namespaced_moveit.log"
    with log_path.open("w") as log:
        process = subprocess.Popen([
            "ros2", "launch", "hex_arm_moveit_config", "moveit_mock.launch.py",
            "use_rviz:=false", "namespace:=robot/left", "joint_prefix:=left_",
            "hardware_name:=LeftArmSystem", "controller_name:=left_trajectory",
            "controller_manager:=robot_control", "planning_group:=left_arm",
            "move_group_name:=left_move_group"],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node("prefixed_moveit_probe")
    client = ActionClient(node, MoveGroup, "/robot/left/move_action")
    positions = {}
    subscription = node.create_subscription(JointState, "/robot/left/joint_states",
        lambda message: positions.update(zip(message.name, message.position)), 10)

    def wait(future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert future.done(), log_path.read_text()
        return future.result()

    try:
        assert client.wait_for_server(timeout_sec=45.0), log_path.read_text()
        target = [0.20, 0.20, 1.30, -0.20, 0.15, 0.10]
        goal = MoveGroup.Goal()
        goal.request.group_name = "left_arm"
        goal.request.pipeline_id = "ompl"
        goal.request.num_planning_attempts = 3
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.1
        goal.request.start_state.is_diff = True
        constraints = Constraints(name="prefixed_joint_target")
        constraints.joint_constraints = [JointConstraint(
            joint_name=f"left_joint_{index}", position=position,
            tolerance_above=0.001, tolerance_below=0.001, weight=1.0)
            for index, position in enumerate(target, 1)]
        goal.request.goal_constraints = [constraints]
        goal.planning_options.planning_scene_diff.is_diff = True
        handle = wait(client.send_goal_async(goal), 10.0)
        assert handle.accepted, log_path.read_text()
        result = wait(handle.get_result_async(), 45.0)
        assert result.status == GoalStatus.STATUS_SUCCEEDED, log_path.read_text()
        assert result.result.error_code.val == MoveItErrorCodes.SUCCESS
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if all(abs(positions.get(f"left_joint_{index}", 100.0) - value) < 0.02 for index, value in enumerate(target, 1)):
                break
        assert set(positions) == {f"left_joint_{index}" for index in range(1, 7)}
        assert all(abs(positions[f"left_joint_{index}"] - value) < 0.02 for index, value in enumerate(target, 1))
    finally:
        client.destroy()
        node.destroy_subscription(subscription)
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
