import math
import threading
import time
from types import SimpleNamespace

import pytest

from hex_arm_bridge.node import (
    HexArmBridge,
    _Snapshot,
    _active_ownership_error,
    _generic_response_error,
    _positive_timeout,
    _readiness_error,
)
from hex_arm_bridge.protocol import JOINT_NAMES, optional_vector, reorder, require_api_major


def _ready_snapshot(now: float = 10.0) -> _Snapshot:
    driver = SimpleNamespace(
        profile_valid=True,
        calibrated=True,
        all_motors_online=True,
        feedback_fresh=True,
        fault_latched=False,
        fault_reason="",
        session_owned=True,
        mode=3,
    )
    return _Snapshot(
        joint_state=SimpleNamespace(q=[0.0] * 6),
        driver_state=driver,
        joint_received_at=now - 0.01,
        driver_received_at=now - 0.02,
    )


def test_reorder_accepts_permutation() -> None:
    names = ["joint_6", "joint_1", "joint_4", "joint_2", "joint_5", "joint_3"]
    values = [6.0, 1.0, 4.0, 2.0, 5.0, 3.0]
    assert reorder(values, names) == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]


@pytest.mark.parametrize(
    ("values", "names"),
    [
        ([0.0] * 5, JOINT_NAMES[:5]),
        ([0.0] * 6, ["joint_1"] * 6),
        ([0.0] * 5 + [math.nan], JOINT_NAMES),
    ],
)
def test_reorder_rejects_incomplete_ambiguous_or_nonfinite(values, names) -> None:
    with pytest.raises(ValueError):
        reorder(values, names)


def test_optional_vector_contract() -> None:
    assert optional_vector([], 3, 2.0) == [2.0, 2.0, 2.0]
    assert optional_vector([1.0, 2.0, 3.0], 3, 0.0) == [1.0, 2.0, 3.0]
    with pytest.raises(ValueError):
        optional_vector([1.0], 3, 0.0)


def test_api_major_is_strict() -> None:
    require_api_major(0)
    with pytest.raises(RuntimeError):
        require_api_major(1)


def test_positive_timeout_rejects_nonpositive_or_nonfinite_values() -> None:
    assert _positive_timeout(0.5, "timeout") == 0.5
    for value in (0.0, -1.0, math.inf, math.nan):
        with pytest.raises(ValueError):
            _positive_timeout(value, "timeout")


def test_generic_response_error_checks_transport_and_business_results() -> None:
    assert _generic_response_error(SimpleNamespace(ok=True, error=""), "disable") is None
    assert _generic_response_error(None, "disable") == "disable returned no response"
    assert _generic_response_error(
        SimpleNamespace(ok=False, error="session mismatch"), "release"
    ) == "release rejected: session mismatch"
    assert _generic_response_error(SimpleNamespace(ok=False, error=""), "release") == (
        "release rejected"
    )


def test_release_session_checks_both_responses_even_after_disable_rejection() -> None:
    calls = []
    responses = [
        SimpleNamespace(ok=False, error="disable rejected"),
        SimpleNamespace(ok=True, error=""),
    ]

    def query(key, payload, response_type):
        del payload, response_type
        calls.append(key)
        return responses.pop(0)

    bridge = SimpleNamespace(prefix="robot", _query=query)
    error = HexArmBridge._release_session(bridge, 7)
    assert error == "DISABLED mode request rejected: disable rejected"
    assert calls == ["robot/rpc/set_mode", "robot/rpc/release_session"]


def test_safe_release_stops_commands_and_retains_failed_session_for_retry() -> None:
    bridge = SimpleNamespace(
        _hardware_transition_lock=threading.RLock(),
        _lock=threading.RLock(),
        _session_id=7,
        _hardware_active=True,
        _session=object(),
        _runtime_gate_latched=False,
        _runtime_gate_reason="",
        _release_session=lambda session_id: "release rejected",
    )
    assert HexArmBridge._safe_release(bridge) == "release rejected"
    assert bridge._session_id == 7
    assert not bridge._hardware_active
    assert bridge._runtime_gate_latched

    bridge._release_session = lambda session_id: None
    assert HexArmBridge._safe_release(bridge) is None
    assert bridge._session_id == 0
    assert not bridge._runtime_gate_latched


def test_runtime_gate_latches_without_automatic_recovery() -> None:
    logs = []
    snapshot = _ready_snapshot(now=time.monotonic())
    snapshot.driver_state.mode = 1
    bridge = SimpleNamespace(
        _hardware_transition_lock=threading.RLock(),
        _lock=threading.RLock(),
        _runtime_gate_latched=False,
        _runtime_gate_reason="",
        _session_id=7,
        _hardware_active=True,
        _snapshot=snapshot,
        get_parameter=lambda name: SimpleNamespace(value=0.1),
        get_logger=lambda: SimpleNamespace(error=logs.append),
    )
    assert HexArmBridge._latch_runtime_gate_if_needed(bridge)
    assert not bridge._hardware_active
    assert bridge._runtime_gate_latched

    snapshot.driver_state.mode = 2
    assert HexArmBridge._latch_runtime_gate_if_needed(bridge)
    assert bridge._runtime_gate_latched


def test_destroy_ros_entities_is_idempotent_without_real_ros_entities() -> None:
    destroyed = []
    bridge = SimpleNamespace(
        _lock=threading.RLock(),
        _diag_timer="timer",
        _command_sub="subscription",
        _services=["service_a", "service_b"],
        _state_pub="state_pub",
        _driver_pub="driver_pub",
        _diag_pub="diag_pub",
        destroy_timer=lambda entity: destroyed.append(("timer", entity)),
        destroy_subscription=lambda entity: destroyed.append(("subscription", entity)),
        destroy_service=lambda entity: destroyed.append(("service", entity)),
        destroy_lifecycle_publisher=lambda entity: destroyed.append(("publisher", entity)),
        get_logger=lambda: SimpleNamespace(warning=lambda message: None),
    )
    HexArmBridge._destroy_ros_entities(bridge)
    first_destroy_count = len(destroyed)
    HexArmBridge._destroy_ros_entities(bridge)
    assert first_destroy_count == 7
    assert len(destroyed) == first_destroy_count


def test_readiness_accepts_fresh_joint_and_safe_driver_states() -> None:
    assert _readiness_error(_ready_snapshot(), now=10.0, state_timeout_sec=0.1) is None


def test_readiness_tracks_joint_and_driver_freshness_separately() -> None:
    snapshot = _ready_snapshot()
    snapshot.joint_received_at = 9.0
    assert _readiness_error(snapshot, 10.0, 0.1) == "joint state is stale"

    snapshot = _ready_snapshot()
    snapshot.driver_received_at = 9.0
    assert _readiness_error(snapshot, 10.0, 0.1) == "driver state is stale"


@pytest.mark.parametrize(
    ("field", "expected"),
    [
        ("profile_valid", "hardware profile is not valid"),
        ("calibrated", "hardware is not calibrated"),
        ("all_motors_online", "not all motors are online"),
        ("feedback_fresh", "motor feedback is not fresh"),
    ],
)
def test_readiness_rejects_each_driver_safety_gate(field: str, expected: str) -> None:
    snapshot = _ready_snapshot()
    setattr(snapshot.driver_state, field, False)
    assert _readiness_error(snapshot, 10.0, 0.1) == expected


def test_readiness_rejects_faults_and_invalid_joint_vectors() -> None:
    snapshot = _ready_snapshot()
    snapshot.driver_state.fault_latched = True
    snapshot.driver_state.fault_reason = "over temperature"
    assert _readiness_error(snapshot, 10.0, 0.1) == (
        "controller fault is latched: over temperature"
    )

    snapshot = _ready_snapshot()
    snapshot.joint_state.q = [0.0] * 5 + [math.nan]
    assert _readiness_error(snapshot, 10.0, 0.1) == (
        "joint state does not contain six finite positions"
    )


def test_active_ownership_requires_new_owned_active_driver_state() -> None:
    snapshot = _ready_snapshot()
    assert _active_ownership_error(snapshot, 10.0, 0.1, active_mode=3) is None
    assert _active_ownership_error(
        snapshot,
        10.0,
        0.1,
        active_mode=3,
        driver_received_after=snapshot.driver_received_at,
    ) == "no new driver state received after ACTIVE request"

    snapshot = _ready_snapshot()
    snapshot.driver_state.session_owned = False
    assert _active_ownership_error(snapshot, 10.0, 0.1, 3) == (
        "driver does not confirm session ownership"
    )

    snapshot = _ready_snapshot()
    snapshot.driver_state.mode = 2
    assert _active_ownership_error(snapshot, 10.0, 0.1, 3) == (
        "driver does not confirm ACTIVE mode"
    )
