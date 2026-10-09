from __future__ import annotations

import math
from typing import Any, Optional
from diagnostic_msgs.msg import KeyValue
from hex_arm_msgs.msg import DriverState as RosDriverState, MotorIdentity as RosMotorIdentity
from hex_arm_bridge.pb import robot_api_pb2 as pb
from hex_arm_bridge.protocol import JOINT_NAMES


def ros_motor(motor: pb.MotorIdentity) -> RosMotorIdentity:
    result = RosMotorIdentity()
    result.node_id = motor.node_id
    result.vendor_id = motor.vendor_id
    result.product_code = motor.product_code
    result.revision = motor.revision
    result.serial_number = motor.serial_number
    result.model = motor.model
    result.identity_verified = motor.identity_verified
    return result


def ros_driver(driver: pb.DriverState, stamp: Any) -> RosDriverState:
    result = RosDriverState()
    result.stamp = stamp
    result.mode = driver.mode
    result.session_owned = driver.session_owned
    result.profile_valid = driver.profile_valid
    result.calibrated = driver.calibrated
    result.all_motors_online = driver.all_motors_online
    result.feedback_fresh = driver.feedback_fresh
    result.fault_latched = driver.fault_latched
    result.fault_code = driver.fault_code
    result.fault_reason = driver.fault_reason
    result.command_age_s = driver.command_age_s
    result.feedback_age_s = driver.feedback_age_s
    result.motors = [ros_motor(motor) for motor in driver.motors]
    return result


def _temperature_vector_error(joint: Any | None) -> Optional[str]:
    """Validate optional per-axis temperature telemetry without raising."""
    if joint is None:
        return None
    temperatures = getattr(joint, "temp", ())
    try:
        count = len(temperatures)
    except TypeError:
        return "joint temperature vector is not a sequence"
    if count == 0:
        return None
    if count != len(JOINT_NAMES):
        return "joint temperature vector does not contain six values"
    try:
        finite = all(math.isfinite(float(value)) for value in temperatures)
    except (TypeError, ValueError, OverflowError):
        finite = False
    if not finite:
        return "joint temperature vector contains a non-finite value"
    return None


def _operating_mode_text(mode: Any) -> str:
    try:
        value = int(mode)
        return pb.OperatingMode.Name(value)
    except (TypeError, ValueError, OverflowError):
        return f"UNKNOWN({mode})"


def _diagnostic_values(
    joint: Any | None,
    driver: Any | None,
    joint_age: float,
    driver_age: float,
) -> list[KeyValue]:
    """Build stable driver diagnostics plus optional six-axis temperatures."""
    values = [
        KeyValue(key="joint_state_age_s", value=f"{joint_age:.6f}"),
        KeyValue(key="driver_state_age_s", value=f"{driver_age:.6f}"),
    ]
    if driver is not None:
        values.extend(
            [
                KeyValue(
                    key="mode",
                    value=_operating_mode_text(getattr(driver, "mode", -1)),
                ),
                KeyValue(
                    key="session",
                    value="owned"
                    if bool(getattr(driver, "session_owned", False))
                    else "unowned",
                ),
                KeyValue(
                    key="profile",
                    value="valid"
                    if bool(getattr(driver, "profile_valid", False))
                    else "invalid",
                ),
                KeyValue(
                    key="calibrated",
                    value=str(bool(getattr(driver, "calibrated", False))).lower(),
                ),
                KeyValue(
                    key="all_online",
                    value=str(bool(getattr(driver, "all_motors_online", False))).lower(),
                ),
                KeyValue(
                    key="feedback_fresh",
                    value=str(bool(getattr(driver, "feedback_fresh", False))).lower(),
                ),
                KeyValue(
                    key="fault_code",
                    value=f"0x{int(getattr(driver, 'fault_code', 0)):08x}",
                ),
            ]
        )

    temperatures = getattr(joint, "temp", ()) if joint is not None else ()
    temperature_error = _temperature_vector_error(joint)
    if temperature_error is None and len(temperatures) == len(JOINT_NAMES):
        values.extend(
            KeyValue(key=f"{name}_temperature_c", value=f"{float(value):.3f}")
            for name, value in zip(JOINT_NAMES, temperatures)
        )
    return values
