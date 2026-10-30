"""Table behavior, data validity and source-clock restarts without hardware."""
import argparse
import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from python_qt_binding.QtWidgets import QApplication
import pytest

from hex_arm_tools.pb import robot_api_pb2 as pb
from hex_arm_visualization.motor_monitor import MonitorWindow, positive_seconds, robot_prefix
from hex_arm_visualization.motor_monitor_client import MonitorCache


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


def joint_message(stamp=100, diagnostics=True):
    return pb.JointState(
        header=pb.Header(stamp_ns=stamp), q=[0.1 * i for i in range(6)],
        dq=[-0.2] * 6, tau_est=[-1.234] * 6,
        motor_diagnostics=[pb.MotorDiagnostics(
            node_id=i + 11, motor_temperature_c=31.5 + i,
            driver_temperature_c=40.5 + i, online=True, feedback_fresh=True,
        ) for i in range(6)] if diagnostics else [])


def describe(cache):
    cache.description(pb.ArmDescription(dof=6, joint_names=[f"axis_{i}" for i in range(6)]))


def texts(window, row):
    return [window.table.item(row, column).text() for column in range(10)]


def test_table_keeps_diagnostics_when_one_motion_value_is_invalid(app):
    cache = MonitorCache()
    message = joint_message()
    message.q[0] = float("nan")
    message.motor_diagnostics[0].fault_code = 0xa123
    message.motor_diagnostics[1].online = False
    message.motor_diagnostics[2].feedback_fresh = False
    message.motor_diagnostics[3].driver_temperature_c = float("inf")
    cache.joint(message, now=10)
    describe(cache)
    cache.driver(pb.DriverState(header=pb.Header(stamp_ns=100), mode=pb.OPERATING_MODE_FAULT,
                                fault_latched=True, fault_code=0x1004,
                                fault_reason="motor fault"), now=10)
    window = MonitorWindow(cache, "test/arm", "tcp/localhost:7448")
    try:
        window.refresh(now=10.1)
        assert texts(window, 0) == ["axis_0", "11", "—", "-0.200", "-1.234",
                                    "31.5", "40.5", "在线", "正常", "0xA123"]
        assert texts(window, 1)[7] == "离线"
        assert texts(window, 2)[8] == "过期"
        assert texts(window, 3)[6] == "—"
        assert "0x1004" in window.status.text() and "motor fault" in window.status.text()
        assert window.table.item(0, 0).foreground().color().name() == "#b00020"
        assert window.table.item(1, 0).foreground().color().name() == "#707070"

        window.refresh(now=11)
        assert "状态流中断" in window.status.text()
        assert "驱动状态已过期" in window.status.text()
        assert texts(window, 0)[7:9] == ["未知", "状态流中断"]
        assert texts(window, 1)[4] == "-1.234"  # Retain, visibly stale, the last value.
    finally:
        window.close()


def test_older_driver_and_incomplete_extensions_never_invent_zero_temperatures(app):
    cache = MonitorCache()
    window = MonitorWindow(cache, "test/arm", "tcp/localhost:7448")
    try:
        assert "等待关节状态" in window.status.text()
        assert texts(window, 0)[1:] == ["—"] * 9
        for count in (0, 5):
            message = joint_message()
            del message.motor_diagnostics[count:]
            cache.joint(message, now=10)
            window.refresh(now=10.1)
            assert texts(window, 0)[2:5] == ["0.000", "-0.200", "-1.234"]
            assert texts(window, 0)[5:] == ["—"] * 5
            assert "API 0.5+" in window.status.text()
    finally:
        window.close()


def test_source_restart_refreshes_description_and_drops_previous_mode():
    cache = MonitorCache()
    assert cache.joint(joint_message(100), now=1)
    describe(cache)
    cache.driver(pb.DriverState(header=pb.Header(stamp_ns=100),
                                mode=pb.OPERATING_MODE_ACTIVE), now=1)
    assert not cache.joint(joint_message(101), now=1.1)
    assert cache.joint(joint_message(1), now=1.2)
    assert cache.snapshot().names == ()
    assert cache.snapshot().driver is None
    assert cache.joint(joint_message(2), now=3)  # Reconnect after a reception gap.


def test_bad_shapes_and_descriptions_do_not_replace_valid_data():
    cache = MonitorCache()
    valid = joint_message()
    cache.joint(valid, now=1)
    for message in (pb.JointState(), pb.JointState(header=pb.Header(stamp_ns=2))):
        with pytest.raises(ValueError):
            cache.joint(message, now=2)
    assert cache.snapshot().joint is valid
    assert cache.snapshot().joint_at == 1
    with pytest.raises(ValueError):
        cache.description(pb.ArmDescription(dof=6, joint_names=["joint"] * 6))


@pytest.mark.parametrize("value", ["0", "-1", "nan", "inf"])
def test_invalid_stale_threshold(value):
    with pytest.raises(argparse.ArgumentTypeError):
        positive_seconds(value)


@pytest.mark.parametrize("value", ["", "/", "robot/**", "robot name"])
def test_prefix_must_select_one_robot(value):
    with pytest.raises(argparse.ArgumentTypeError):
        robot_prefix(value)
