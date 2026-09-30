"""Verify the real action gate with MoveIt, without any hardware process."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from moveit_msgs.action import ExecuteTrajectory
from std_msgs.msg import String
import yaml

from test_moveit_plan_only import PlanOnlyProbe
from test_deployment_planning import PACKAGE_ROOT, RATE, START, TARGET


TOKEN = "a" * 32


@pytest.mark.parametrize("unlock", [False, True])
def test_execution_waits_for_this_launchs_readiness_but_collision_checks_remain_available(tmp_path, unlock):
    profile = yaml.safe_load((PACKAGE_ROOT.parents[1] / "config/hardware/firefly_y6.example.yaml").read_text())
    for joint in profile["joints"]:
        joint["limits"].update(position_lower_rad=-2.8, position_upper_rad=2.8,
                               velocity_rad_s=RATE, acceleration_rad_s2=RATE)
    path = tmp_path / "offline-profile.yaml"
    path.write_text(yaml.safe_dump(profile))
    environment = {**os.environ, "ROS_DOMAIN_ID": str(120 + os.getpid() % 80)}
    os.environ["ROS_DOMAIN_ID"] = environment["ROS_DOMAIN_ID"]
    with tempfile.TemporaryFile(mode="w+") as output:
        child = subprocess.Popen(
            ["ros2", "launch", str(Path(__file__).with_name("deployment_move_group.launch.py")),
             f"hardware_profile:={path}", f"startup_readiness_token:={TOKEN}"],
            stdout=output, stderr=subprocess.STDOUT, start_new_session=True, env=environment)
        rclpy.init()
        node = PlanOnlyProbe()
        executor = ActionClient(node, ExecuteTrajectory, "/execute_trajectory")
        try:
            deadline = time.monotonic() + 30.0
            while not node.validity.wait_for_service(timeout_sec=0.2):
                if child.poll() is not None or time.monotonic() >= deadline:
                    output.seek(0)
                    raise RuntimeError(f"gated planner did not start: {output.read()}")
            assert node.state_validity(START).valid
            assert not node.move_group.wait_for_server(timeout_sec=0.5)
            assert not executor.wait_for_server(timeout_sec=0.5)
            publisher = node.create_publisher(
                String, "/hex_arm/internal/moveit_startup_ready",
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))
            publisher.publish(String(data="b" * 32))
            assert not node.move_group.wait_for_server(timeout_sec=0.5)
            assert not executor.wait_for_server(timeout_sec=0.5)
            if unlock:
                publisher.publish(String(data=TOKEN))
                assert node.move_group.wait_for_server(timeout_sec=5.0)
                assert executor.wait_for_server(timeout_sec=5.0)
                assert node.plan(START, TARGET, 1.0, 1.0).error_code.val == 1
        finally:
            executor.destroy()
            node.destroy_node()
            rclpy.shutdown()
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try:
                    child.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGTERM)
                    child.wait(timeout=5)
        output.seek(0)
        log = output.read()
        assert child.returncode == 0, log
        assert "MoveIt execution locked" in log
        assert ("MoveIt execution unlocked" in log) is unlock
        assert "process has died" not in log
