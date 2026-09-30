"""Exercise SI hardware dynamics in OMPL/TOTG without opening a motor bus."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
import yaml

from test_moveit_plan_only import PlanOnlyProbe


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
START = [0.0, -1.35, 1.43, -0.3, 0.0, 0.0]
TARGET = [0.0, -0.75, 0.83, -0.3, 0.0, 0.0]
RATE = 1.2566370614359172  # GUI 0.2 Rev/s and 0.2 Rev/s^2 times 2*pi.


def test_strict_full_range_plan_exceeds_commissioning_speed_without_exceeding_si_limits(tmp_path):
    profile = yaml.safe_load((PACKAGE_ROOT.parents[1] / "config/hardware/firefly_y6.example.yaml").read_text())
    for joint in profile["joints"]:
        joint["limits"].update(position_lower_rad=-2.8, position_upper_rad=2.8,
                               velocity_rad_s=RATE, acceleration_rad_s2=RATE)
    path = tmp_path / "offline-profile.yaml"
    path.write_text(yaml.safe_dump(profile))
    environment = {**os.environ, "ROS_DOMAIN_ID": str(120 + os.getpid() % 80)}
    # Initialize this client in the same isolated domain as the offline child.
    os.environ["ROS_DOMAIN_ID"] = environment["ROS_DOMAIN_ID"]
    with tempfile.TemporaryFile(mode="w+") as output:
        child = subprocess.Popen(["ros2", "launch", str(Path(__file__).with_name("deployment_move_group.launch.py")),
                                  f"hardware_profile:={path}"], stdout=output, stderr=subprocess.STDOUT,
                                 start_new_session=True, env=environment)
        rclpy.init()
        node = PlanOnlyProbe()
        try:
            deadline = time.monotonic() + 30.0
            while not (node.move_group.wait_for_server(timeout_sec=0.2)
                       and node.validity.wait_for_service(timeout_sec=0.2)):
                if child.poll() is not None or time.monotonic() >= deadline:
                    output.seek(0)
                    raise RuntimeError(f"offline strict planner did not start: {output.read()}")
            assert node.state_validity(START).valid
            assert node.state_validity(TARGET).valid
            plan = node.plan(START, TARGET, 1.0, 1.0).planned_trajectory.joint_trajectory
            assert max(abs(v) for point in plan.points for v in point.velocities) > 0.1
            for point in plan.points:
                assert len(point.velocities) == len(point.accelerations) == 6
                assert max(abs(v) for v in point.velocities) <= RATE + 1e-5
                assert max(abs(a) for a in point.accelerations) <= RATE + 1e-5
                assert node.state_validity(list(point.positions)).valid
        finally:
            node.destroy_node()
            rclpy.shutdown()
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try:
                    child.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGTERM)
                    child.wait(timeout=5)
