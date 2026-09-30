"""Full launch smoke test in an isolated ROS domain; --mock only."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from hex_arm_msgs.msg import DriverState
import pytest
import rclpy
from std_srvs.srv import Trigger


@pytest.mark.parametrize("activate", [False, True])
def test_complete_mock_launch_observation_and_confirmed_stop(tmp_path, activate):
    root = Path(__file__).resolve().parents[3]
    env = dict(os.environ, ROS_DOMAIN_ID="88", ROS_LOG_DIR=str(tmp_path / "logs"))
    context = rclpy.context.Context()
    rclpy.init(context=context, domain_id=88)
    node = rclpy.create_node("hand_guiding_launch_probe", context=context)
    executor = rclpy.executors.SingleThreadedExecutor(context=context)
    executor.add_node(node)
    latest = []
    subscription = node.create_subscription(DriverState, "/hex_arm/driver_state", latest.append, 10)
    process = None
    with (tmp_path / "launch.log").open("w+") as log:
        try:
            process = subprocess.Popen([
                "ros2", "launch", "hex_arm_bringup", "gravity_comp.launch.py",
                f"hardware_profile:={root}/src/hex_arm_controller/test/firefly_y6.mock.yaml",
                "mock:=true", f"activate_hardware:={str(activate).lower()}",
                "zenoh_endpoint:=tcp/127.0.0.1:17488", "use_rviz:=false",
            ], env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            expected = 4 if activate else 1
            deadline = time.monotonic() + 20
            while not latest or latest[-1].mode != expected:
                assert process.poll() is None, "launch exited before the expected state"
                assert time.monotonic() < deadline, "launch did not reach the expected state"
                executor.spin_once(timeout_sec=0.05)
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
                assert latest[-1].mode == expected
                assert not latest[-1].fault_latched
            if activate:
                client = node.create_client(Trigger, "/hex_arm_gravity_comp/stop")
                assert client.wait_for_service(timeout_sec=3)
                future = client.call_async(Trigger.Request())
                executor.spin_until_future_complete(future, timeout_sec=5)
                assert future.done() and future.result().success
            else:
                assert not latest[-1].session_owned
                process.send_signal(signal.SIGINT)
            assert process.wait(timeout=12) == 0
            reports = list((tmp_path / "logs").rglob("hand-guiding-shutdown.json"))
            assert len(reports) == 1
            assert json.loads(reports[0].read_text())["state"] == "disabled_confirmed"
        except BaseException:
            log.seek(0)
            print(log.read())
            raise
        finally:
            if process is not None and process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
            node.destroy_subscription(subscription)
            executor.shutdown()
            node.destroy_node()
            context.try_shutdown()
