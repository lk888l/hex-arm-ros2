"""Actual Qt + Zenoh + Rust --mock. These tests never open a CAN interface."""
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from python_qt_binding.QtWidgets import QApplication
import pytest
import zenoh

from hex_arm_tools.pb import robot_api_pb2 as pb
from hex_arm_visualization.motor_monitor import MonitorWindow
from hex_arm_visualization.motor_monitor_client import MonitorCache, MonitorClient


ROOT = Path(__file__).resolve().parents[3]
PREFIX = "hexmeow/wsl/firefly_y6_test"


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


def endpoint():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return f"tcp/127.0.0.1:{sock.getsockname()[1]}"


def wait_for(app, predicate, tick=lambda: None, timeout=8):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        tick()
        app.processEvents()
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError("monitor condition timed out")


def test_live_client_only_reads_description_and_receives_its_own_robot(app):
    address = endpoint()
    config = zenoh.Config()
    config.insert_json5("mode", '"peer"')
    config.insert_json5("listen/endpoints", f'["{address}"]')
    config.insert_json5("scouting/multicast/enabled", "false")
    session = zenoh.open(config)
    queries, commands = [], []

    def query(request):
        queries.append(str(request.key_expr))
        if str(request.key_expr) == PREFIX + "/arm/description":
            request.reply(request.key_expr, pb.ArmDescription(
                dof=6, joint_names=[f"joint_{i + 1}" for i in range(6)]).SerializeToString())

    queryable = session.declare_queryable(PREFIX + "/**", query)
    command_subscriber = session.declare_subscriber(PREFIX + "/arm/command", commands.append)
    cache = MonitorCache(0.15)
    client = MonitorClient(cache, PREFIX, address)
    window = MonitorWindow(cache, PREFIX, address)
    joint = pb.JointState(header=pb.Header(stamp_ns=10), q=[0.] * 6,
                          dq=[0.] * 6, tau_est=[1.25] * 6,
                          motor_diagnostics=[pb.MotorDiagnostics(
                              node_id=i + 1, motor_temperature_c=35., driver_temperature_c=40.,
                              online=True, feedback_fresh=True) for i in range(6)])
    driver = pb.DriverState(header=pb.Header(stamp_ns=10), feedback_fresh=True,
                            mode=pb.OPERATING_MODE_DISABLED)

    def publish():
        session.put(PREFIX + "/arm/joint_state", joint.SerializeToString())
        session.put(PREFIX + "/driver_state", driver.SerializeToString())

    try:
        window.show()
        client.start()
        wait_for(app, lambda: cache.snapshot().names and cache.snapshot().joint is not None,
                 tick=publish)
        window.refresh()
        assert window.table.item(0, 4).text() == "1.250"
        assert window.table.item(0, 5).text() == "35.0"
        assert window.table.item(0, 6).text() == "40.0"
        assert not cache.snapshot().driver.session_owned

        # Unrelated robots cannot keep this arm's display alive.
        wait_for(app, lambda: "状态流中断" in window.status.text(), tick=lambda: session.put(
            "another/robot/arm/joint_state", joint.SerializeToString()))
        # Invalid motion numbers still deliver online/fault diagnostics.
        joint.q[0] = float("nan")
        joint.motor_diagnostics[0].fault_code = 0xa001
        wait_for(app, lambda: window.table.item(0, 9).text() == "0xA001", tick=publish)
        assert window.table.item(0, 2).text() == "—"
        assert window.table.item(0, 7).text() == "在线"
    finally:
        window.close()
        assert client.close()
        queryable.undeclare()
        command_subscriber.undeclare()
        session.close()
    assert queries and set(queries) == {PREFIX + "/arm/description"}
    assert commands == []


def test_rust_mock_and_driver_restart_remain_unowned(app, tmp_path):
    address = endpoint()
    binary = Path(get_package_prefix("hex_arm_controller")) / "lib/hex_arm_controller/hex_arm_controller"
    urdf = Path(get_package_share_directory("xpkg_urdf_firefly_y6")) / "urdf/xpkg_urdf_firefly_y6.urdf"
    cache = MonitorCache(0.2)
    client = MonitorClient(cache, PREFIX, address)
    window = MonitorWindow(cache, PREFIX, address)
    process = None

    def ready():
        state = cache.snapshot()
        return (state.joint is not None and len(state.joint.motor_diagnostics) == 6
                and state.driver is not None and state.names
                and time.monotonic() - state.joint_at < 0.2)

    def launch(log):
        return subprocess.Popen([
            str(binary), "--mock", "--profile",
            str(ROOT / "src/hex_arm_controller/test/firefly_y6.mock.yaml"),
            "--urdf", str(urdf), "--zenoh-listen", address,
            "--shutdown-report", str(tmp_path / "shutdown.json"),
        ], stdout=log, stderr=subprocess.STDOUT)

    with (tmp_path / "driver.log").open("w+") as log:
        try:
            process = launch(log)
            window.show()
            client.start()
            wait_for(app, ready)
            assert cache.snapshot().driver.mode == pb.OPERATING_MODE_DISABLED
            assert not cache.snapshot().driver.session_owned
            assert [m.node_id for m in cache.snapshot().joint.motor_diagnostics] == list(range(1, 7))

            # Close/restart the actual daemon while keeping the same window open.
            process.send_signal(signal.SIGTERM)
            assert process.wait(timeout=8) == 0
            wait_for(app, lambda: "状态流中断" in window.status.text())
            process = launch(log)
            wait_for(app, ready, timeout=12)
            assert "故障" not in window.status.text()
            assert cache.snapshot().driver.mode == pb.OPERATING_MODE_DISABLED
            assert not cache.snapshot().driver.session_owned
            window.close()
            assert client.close()
            assert process.poll() is None
        except BaseException:
            log.seek(0)
            print(log.read())
            raise
        finally:
            client.close()
            window.close()
            if process is not None and process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3)


@pytest.mark.parametrize("address", [None, "invalid-endpoint"])
def test_cli_can_close_while_endpoint_is_unavailable(address):
    process = subprocess.Popen([
        sys.executable, "-m", "hex_arm_visualization.motor_monitor",
        "--robot-prefix", PREFIX, "--zenoh-connect", address or endpoint(),
    ], env=dict(os.environ, QT_QPA_PLATFORM="offscreen"),
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        time.sleep(0.6)
        assert process.poll() is None
        process.terminate()
        output, _ = process.communicate(timeout=5)
        assert process.returncode == 0, output
        assert "Traceback" not in output
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate(timeout=3)
