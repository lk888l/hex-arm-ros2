"""Actual ROS owner + Zenoh + Rust process; always --mock, never opens CAN."""
import json
import os
from pathlib import Path
import queue
import signal
import socket
import subprocess
import time

from ament_index_python.packages import get_package_prefix
import pytest
import zenoh

from hex_arm_bridge.gravity_comp import GravityCompClient
from hex_arm_bridge.pb import robot_api_pb2 as pb


ROOT = Path(__file__).resolve().parents[3]
PREFIX = "hexmeow/wsl/firefly_y6_test"


def stop_process(process):
    if process is None:
        return
    if process.poll() is None:
        process.send_signal(signal.SIGTERM)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            process.kill()
    process.wait(timeout=5)


@pytest.mark.parametrize("owner_signal", [signal.SIGTERM, signal.SIGKILL, signal.SIGSTOP])
def test_actual_ros_owner_shutdown_crash_and_freeze(tmp_path, owner_signal):
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    endpoint = f"tcp/127.0.0.1:{port}"
    driver_bin = Path(get_package_prefix("hex_arm_controller")) / "lib/hex_arm_controller/hex_arm_controller"
    owner_bin = Path(get_package_prefix("hex_arm_bridge")) / "lib/hex_arm_bridge/hex_arm_gravity_comp"
    report = tmp_path / "shutdown.json"
    driver = owner = session = subscriber = None
    states = queue.Queue()
    config = zenoh.Config()
    config.insert_json5("connect/endpoints", json.dumps([endpoint]))
    config.insert_json5("scouting/multicast/enabled", "false")

    def wait_state(predicate, timeout=10):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                state = states.get(timeout=min(0.1, max(0.001, deadline - time.monotonic())))
            except queue.Empty:
                continue
            if predicate(state):
                return state
        raise AssertionError("expected driver state was not published")

    with (tmp_path / "driver.log").open("w+") as driver_log, (tmp_path / "owner.log").open("w+") as owner_log:
        try:
            driver = subprocess.Popen([
                str(driver_bin), "--mock", "--profile",
                str(ROOT / "src/hex_arm_controller/test/firefly_y6.mock.yaml"),
                "--urdf", str(ROOT / "src/xpkg_urdf_firefly_y6/urdf/xpkg_urdf_firefly_y6.urdf"),
                "--zenoh-listen", endpoint, "--shutdown-report", str(report),
            ], stdout=driver_log, stderr=subprocess.STDOUT)
            session = zenoh.open(config)
            subscriber = session.declare_subscriber(f"{PREFIX}/driver_state",
                lambda sample: states.put(pb.DriverState.FromString(bytes(sample.payload.to_bytes()))))
            wait_state(lambda state: state.mode == pb.OPERATING_MODE_DISABLED)
            env = dict(os.environ, ROS_DOMAIN_ID="87")
            owner = subprocess.Popen([
                str(owner_bin), "--ros-args", "-p", f"robot_prefix:={PREFIX}",
                "-p", f"zenoh_connect:={endpoint}", "-p", "damping:=[1.0,1.0,1.0,1.0,1.0,1.0]",
            ], env=env, stdout=owner_log, stderr=subprocess.STDOUT)
            wait_state(lambda state: state.mode == pb.OPERATING_MODE_GRAVITY_COMP)
            deadline = time.monotonic() + 0.75
            while time.monotonic() < deadline:
                state = wait_state(lambda _: True)
                assert not state.fault_latched, state.fault_reason
                assert state.mode == pb.OPERATING_MODE_GRAVITY_COMP
            assert owner.poll() is None
            # Another process can observe but cannot steal the owner's session.
            probe = GravityCompClient(session, PREFIX, [1.0] * 6)
            acquired = probe.query("rpc/acquire_session", pb.AcquireSessionRequest(
                client_name="integration-competitor"), pb.AcquireSessionResponse)
            assert not acquired.ok and acquired.current_holder
            before = time.monotonic()
            owner.send_signal(owner_signal)
            if owner_signal == signal.SIGTERM:
                state = wait_state(lambda s: s.mode == pb.OPERATING_MODE_DISABLED and not s.session_owned)
                assert owner.wait(timeout=5) == 0
            else:
                state = wait_state(lambda s: s.mode == pb.OPERATING_MODE_FAULT, timeout=2)
                assert state.fault_code == 0x1003
                assert "heartbeat timeout" in state.fault_reason
                assert time.monotonic() - before < 1.5
                log = probe.query("events/recent", None, pb.EventLog)
                # Allow state publication to precede confirmed disable briefly.
                deadline = time.monotonic() + 1
                while not any(event.code == "fault_disable_confirmed" for event in log.events):
                    assert time.monotonic() < deadline
                    time.sleep(0.02)
                    log = probe.query("events/recent", None, pb.EventLog)
                if owner_signal == signal.SIGSTOP:
                    owner.kill()
            stop_process(driver)
            assert driver.returncode == 0
            assert json.loads(report.read_text())["state"] == "disabled_confirmed"
        except BaseException:
            for name, stream in (("driver", driver_log), ("owner", owner_log)):
                stream.seek(0)
                print(f"{name} log:\n{stream.read()}")
            raise
        finally:
            if owner is not None and owner_signal == signal.SIGSTOP and owner.poll() is None:
                owner.kill()
            stop_process(owner)
            stop_process(driver)
            if subscriber is not None:
                subscriber.undeclare()
            if session is not None:
                session.close()
