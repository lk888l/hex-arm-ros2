import gc
import math
import os
import signal
import sys
import threading
import time
from types import SimpleNamespace

import pytest
import rclpy
from builtin_interfaces.msg import Time as RosTime
from diagnostic_msgs.msg import DiagnosticStatus

import hex_arm_bridge.node as bridge_node
from hex_arm_bridge.pb import robot_api_pb2 as pb
from hex_arm_bridge.node import (
    _BridgeShutdownRequested,
    HexArmBridge,
    _RepeatedErrorLog,
    _Snapshot,
    _active_ownership_error,
    _configure_zenoh_endpoint,
    _diagnostic_values,
    _generic_response_error,
    _observation_error,
    _positive_timeout,
    _quiesce_multithreaded_executor,
    _require_fault_timeout_support,
    _readiness_error,
    _temperature_vector_error,
)
from hex_arm_bridge.protocol import JOINT_NAMES, optional_vector, reorder, require_api_major


def test_bridge_preserves_rclpy_lifecycle_service_registry() -> None:
    initialized_here = not rclpy.ok()
    if initialized_here:
        rclpy.init()
    bridge = HexArmBridge()
    try:
        service_names = {service.srv_name for service in bridge.services}
        assert "/hex_arm_bridge/get_state" in service_names
        assert "/hex_arm_bridge/change_state" in service_names
        assert bridge._bridge_services == []

        bridge._destroy_ros_entities()
        service_names_after_cleanup = {
            service.srv_name for service in bridge.services
        }
        assert "/hex_arm_bridge/get_state" in service_names_after_cleanup
        assert "/hex_arm_bridge/change_state" in service_names_after_cleanup
    finally:
        bridge.destroy_node()
        if initialized_here and rclpy.ok():
            rclpy.shutdown()


@pytest.mark.parametrize("outcome", ["ok", "rejected", "timeout"])
def test_damped_stop_hands_off_once_and_never_resumes_ros_commands(outcome):
    response = SimpleNamespace(success=False, message="")
    bridge = SimpleNamespace(_hardware_transition_lock=threading.RLock(),
        _lock=threading.RLock(), _hardware_active=True, _session_id=42, prefix="arm")
    calls = []
    def query(key, payload, response_type, timeout):
        assert not bridge._hardware_active
        request = pb.DampedStopRequest.FromString(payload)
        assert request.session_id == 42
        calls.append(key)
        if outcome == "timeout":
            raise TimeoutError("no acknowledgement")
        return pb.GenericResponse(ok=outcome == "ok", error="rejected" if outcome != "ok" else "")
    bridge._query_with_timeout = query
    HexArmBridge._damped_stop(bridge, None, response)
    assert response.success == (outcome == "ok")
    assert not bridge._hardware_active
    assert bridge._session_id == (0 if outcome == "ok" else 42)
    HexArmBridge._damped_stop(bridge, None, response)
    assert not response.success
    assert len(calls) == 1


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


def _publish_diagnostic(temperatures):
    published = []
    now = time.monotonic()
    joint = SimpleNamespace(
        q=[0.0] * 6,
        dq=[0.0] * 6,
        tau_est=[0.0] * 6,
        temp=temperatures,
    )
    driver = SimpleNamespace(
        mode=pb.OPERATING_MODE_DISABLED,
        session_owned=True,
        profile_valid=True,
        calibrated=True,
        all_motors_online=True,
        feedback_fresh=True,
        fault_latched=False,
        fault_code=0x1234,
        fault_reason="",
    )
    bridge = SimpleNamespace(
        _latch_runtime_gate_if_needed=lambda: False,
        _lock=threading.RLock(),
        _snapshot=_Snapshot(joint, driver, now - 0.01, now - 0.02),
        _runtime_gate_latched=False,
        _runtime_gate_reason="",
        _state_pub=None,
        _driver_pub=None,
        _diag_pub=SimpleNamespace(publish=published.append),
        prefix="firefly_y6",
        get_parameter=lambda name: SimpleNamespace(value=0.1),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=RosTime)
        ),
    )
    HexArmBridge._publish_diagnostics(bridge)
    assert len(published) == 1
    return published[0].status[0]


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


def test_repeated_error_log_reports_first_change_periodic_and_recovery() -> None:
    error_log = _RepeatedErrorLog(interval_sec=5.0)

    assert error_log.report("invalid vector", now=10.0) == "invalid vector"
    assert error_log.report("invalid vector", now=10.01) is None
    assert error_log.report("invalid vector", now=14.99) is None
    assert error_log.report("invalid vector", now=15.0) == (
        "invalid vector (2 repeated samples suppressed)"
    )
    assert error_log.report("protobuf decode failed", now=15.01) == (
        "protobuf decode failed"
    )
    assert error_log.report("protobuf decode failed", now=15.02) is None
    assert error_log.recover() == (
        "protobuf decode failed (1 additional repeated samples suppressed)"
    )

    # Recovery resets the state, so a later failure is immediately visible.
    assert error_log.report("invalid vector", now=15.03) == "invalid vector"


def test_invalid_zenoh_joint_state_callback_is_immediate_but_rate_limited(
    monkeypatch,
) -> None:
    errors = []
    infos = []
    now = [10.0]
    monkeypatch.setattr(bridge_node.time, "monotonic", lambda: now[0])
    bridge = SimpleNamespace(
        _lock=threading.RLock(),
        _accept_zenoh_state=True,
        _snapshot=_Snapshot(),
        _joint_state_error_log=_RepeatedErrorLog(interval_sec=5.0),
        _latch_runtime_gate_if_needed=lambda: False,
        get_logger=lambda: SimpleNamespace(error=errors.append, info=infos.append),
    )
    invalid_sample = SimpleNamespace(
        payload=pb.JointState(q=[0.0] * 5).SerializeToString()
    )

    for timestamp in (10.0, 10.01, 14.99, 15.0):
        now[0] = timestamp
        HexArmBridge._on_zenoh_joint_state(bridge, invalid_sample)

    # The first bad startup sample is never hidden, while a 100 Hz stream of
    # the same failure cannot emit a 100 Hz ERROR stream.
    assert errors == [
        "invalid Zenoh joint state: joint state does not contain six finite positions",
        "invalid Zenoh joint state: joint state does not contain six finite positions "
        "(2 repeated samples suppressed)",
    ]

    now[0] = 15.01
    valid_sample = SimpleNamespace(
        payload=pb.JointState(q=[0.0] * 6).SerializeToString()
    )
    HexArmBridge._on_zenoh_joint_state(bridge, valid_sample)
    assert len(bridge._snapshot.joint_state.q) == 6
    assert infos == [
        "valid Zenoh joint state resumed after invalid input: "
        "joint state does not contain six finite positions"
    ]

    now[0] = 15.02
    HexArmBridge._on_zenoh_joint_state(bridge, invalid_sample)
    assert len(errors) == 3

    bridge._accept_zenoh_state = False
    now[0] = 20.02
    HexArmBridge._on_zenoh_joint_state(bridge, invalid_sample)
    assert len(errors) == 3


def test_explicit_zenoh_endpoint_is_direct_peer_without_multicast() -> None:
    calls = []
    config = SimpleNamespace(
        insert_json5=lambda key, value: calls.append((key, value))
    )

    _configure_zenoh_endpoint(config, " tcp/127.0.0.1:7448 ")

    assert calls == [
        ("mode", '"peer"'),
        ("connect/endpoints", '["tcp/127.0.0.1:7448"]'),
        ("scouting/multicast/enabled", "false"),
    ]


def test_empty_zenoh_endpoint_preserves_standalone_discovery_defaults() -> None:
    calls = []
    config = SimpleNamespace(
        insert_json5=lambda key, value: calls.append((key, value))
    )

    _configure_zenoh_endpoint(config, "  ")

    assert calls == []


def test_generic_response_error_checks_transport_and_business_results() -> None:
    assert _generic_response_error(SimpleNamespace(ok=True, error=""), "disable") is None
    assert _generic_response_error(None, "disable") == "disable returned no response"
    assert _generic_response_error(
        SimpleNamespace(ok=False, error="session mismatch"), "release"
    ) == "release rejected: session mismatch"
    assert _generic_response_error(SimpleNamespace(ok=False, error=""), "release") == (
        "release rejected"
    )


def test_bridge_requires_the_fail_closed_timeout_capability() -> None:
    _require_fault_timeout_support([pb.TIMEOUT_BEHAVIOR_FAULT])
    _require_fault_timeout_support(
        [pb.TIMEOUT_BEHAVIOR_HOLD, pb.TIMEOUT_BEHAVIOR_FAULT]
    )
    with pytest.raises(RuntimeError, match="does not advertise TIMEOUT_BEHAVIOR_FAULT"):
        _require_fault_timeout_support(
            [pb.TIMEOUT_BEHAVIOR_HOLD, pb.TIMEOUT_BEHAVIOR_RAMP_STOP]
        )


def test_diagnostics_expose_driver_gates_and_six_axis_temperatures() -> None:
    status = _publish_diagnostic([30.0, 31.25, 32.5, 33.75, 34.0, 35.125])
    values = {value.key: value.value for value in status.values}

    assert status.level == DiagnosticStatus.OK
    assert values["mode"] == "OPERATING_MODE_DISABLED"
    assert values["session"] == "owned"
    assert values["profile"] == "valid"
    assert values["calibrated"] == "true"
    assert values["all_online"] == "true"
    assert values["feedback_fresh"] == "true"
    assert values["fault_code"] == "0x00001234"
    assert [values[f"joint_{axis}_temperature_c"] for axis in range(1, 7)] == [
        "30.000",
        "31.250",
        "32.500",
        "33.750",
        "34.000",
        "35.125",
    ]


def test_empty_temperature_vector_is_an_allowed_optional_field() -> None:
    status = _publish_diagnostic([])
    values = {value.key: value.value for value in status.values}
    assert status.level == DiagnosticStatus.OK
    assert not any(key.endswith("_temperature_c") for key in values)
    assert _temperature_vector_error(SimpleNamespace(temp=[])) is None


@pytest.mark.parametrize(
    ("temperatures", "expected"),
    [
        ([20.0] * 5, "does not contain six values"),
        ([20.0] * 5 + [math.nan], "contains a non-finite value"),
        (None, "is not a sequence"),
    ],
)
def test_malformed_temperature_vectors_warn_instead_of_raising(
    temperatures, expected: str
) -> None:
    status = _publish_diagnostic(temperatures)
    values = {value.key: value.value for value in status.values}
    assert status.level == DiagnosticStatus.WARN
    assert expected in status.message
    assert not any(key.endswith("_temperature_c") for key in values)


def test_diagnostic_values_are_safe_for_an_unknown_mode() -> None:
    values = _diagnostic_values(
        SimpleNamespace(temp=[]),
        SimpleNamespace(
            mode=999,
            session_owned=False,
            profile_valid=False,
            calibrated=False,
            all_motors_online=False,
            feedback_fresh=False,
            fault_code=0,
        ),
        0.01,
        0.02,
    )
    assert {value.key: value.value for value in values}["mode"] == "UNKNOWN(999)"


def test_release_session_checks_both_responses_even_after_disable_rejection() -> None:
    calls = []
    responses = [
        SimpleNamespace(ok=False, error="disable rejected"),
        SimpleNamespace(ok=True, error=""),
    ]

    def query(key, payload, response_type, timeout, *, cancel_on_shutdown):
        del payload, response_type, timeout
        assert not cancel_on_shutdown
        calls.append(key)
        return responses.pop(0)

    bridge = SimpleNamespace(
        prefix="robot",
        _query_with_timeout=query,
        get_parameter=lambda name: SimpleNamespace(
            value=3.5 if name == "mode_transition_timeout_sec" else 0.5
        ),
    )
    error = HexArmBridge._release_session(bridge, 7)
    assert error == "DISABLED mode request rejected: disable rejected"
    assert calls == ["robot/rpc/set_mode", "robot/rpc/release_session"]


def test_shutdown_cancels_a_blocking_zenoh_query(monkeypatch) -> None:
    started = threading.Event()

    class CancellationToken:
        def __init__(self) -> None:
            self.cancelled = threading.Event()

        def cancel(self) -> None:
            self.cancelled.set()

    class Session:
        def get(self, key, **kwargs):
            assert key == "robot/description"
            token = kwargs["cancellation_token"]
            started.set()
            token.cancelled.wait(5.0)
            return []

    monkeypatch.setitem(
        sys.modules,
        "zenoh",
        SimpleNamespace(CancellationToken=CancellationToken),
    )
    bridge = SimpleNamespace(
        _lock=threading.RLock(),
        _session=Session(),
        _shutdown_requested=threading.Event(),
        _active_query_tokens=set(),
    )
    bridge._raise_if_stop_requested = (
        HexArmBridge._raise_if_stop_requested.__get__(bridge, HexArmBridge)
    )

    errors = []

    def query() -> None:
        try:
            HexArmBridge._query_with_timeout(
                bridge, "robot/description", b"", object, 30.0
            )
        except Exception as error:
            errors.append(error)

    worker = threading.Thread(target=query)
    worker.start()
    assert started.wait(1.0)
    HexArmBridge.request_stop(bridge)
    worker.join(1.0)

    assert not worker.is_alive()
    assert len(errors) == 1
    assert isinstance(errors[0], _BridgeShutdownRequested)
    assert bridge._active_query_tokens == set()


def test_configuration_idle_is_restored_when_configuration_is_cancelled() -> None:
    configuration_idle = threading.Event()

    def configure(state):
        del state
        raise _BridgeShutdownRequested("test shutdown")

    bridge = SimpleNamespace(
        _configuration_idle=configuration_idle,
        _configure=configure,
    )
    with pytest.raises(_BridgeShutdownRequested):
        HexArmBridge.on_configure(bridge, None)
    assert configuration_idle.is_set()


def test_sigint_shutdown_cancels_queued_callbacks_before_node_destruction(
    capsys,
) -> None:
    """Regress the exact Jazzy SIGINT teardown race without hardware or CAN.

    Saturating the Python worker pool leaves an rclpy callback submitted but
    not yet counted by ``_WorkTracker``.  The former shutdown sequence would
    report idle, destroy the node, and later print:
    ``The following exception was never retrieved: cannot use Destroyable ...``.
    """
    from rclpy.executors import MultiThreadedExecutor

    initialized_here = not rclpy.ok()
    if initialized_here:
        rclpy.init()
    bridge = HexArmBridge()
    executor = MultiThreadedExecutor(num_threads=2)
    release_workers = threading.Event()
    timer = bridge.create_timer(0.001, lambda: None)
    executor.add_node(bridge)
    try:
        # Occupy both Python workers with non-rclpy work, then submit one ready
        # timer handler.  This deterministically creates the queue window that
        # SIGINT hit on the real observe launch.
        blockers = [
            executor._executor.submit(release_workers.wait, 2.0)
            for _ in range(2)
        ]
        time.sleep(0.02)
        executor.spin_once(timeout_sec=0.5)
        queued = list(executor._futures)
        assert len(queued) == 1
        assert not queued[0].done()
        assert executor._work_tracker.wait(timeout_sec=0.1)

        # Exercise the same signal-to-request_stop handoff as the executable,
        # then release the artificial blockers while quiesce joins the pool.
        previous_sigint = signal.getsignal(signal.SIGINT)
        signal.signal(
            signal.SIGINT,
            lambda signum, frame: bridge.request_stop(),
        )
        try:
            os.kill(os.getpid(), signal.SIGINT)
        finally:
            signal.signal(signal.SIGINT, previous_sigint)
        assert bridge._shutdown_requested.is_set()
        release_timer = threading.Timer(0.02, release_workers.set)
        release_timer.start()
        errors = _quiesce_multithreaded_executor(executor)
        release_timer.join(1.0)

        assert errors == []
        assert executor._futures == []
        assert all(blocker.done() for blocker in blockers)

        executor.remove_node(bridge)
        assert executor.shutdown(timeout_sec=1.0)
        bridge.destroy_timer(timer)
        bridge.destroy_node()

        # Force rclpy Task finalizers now so an unretrieved exception cannot be
        # hidden until interpreter shutdown.
        queued.clear()
        gc.collect()
        assert "exception was never retrieved" not in capsys.readouterr().err
    finally:
        release_workers.set()
        if not bridge._destroyed:
            executor.remove_node(bridge)
            executor.shutdown(timeout_sec=1.0)
            bridge.destroy_node()
        if initialized_here and rclpy.ok():
            rclpy.shutdown()


def test_mode_queries_use_the_management_plane_timeout() -> None:
    calls = []
    bridge = SimpleNamespace(
        get_parameter=lambda name: SimpleNamespace(
            value=3.5 if name == "mode_transition_timeout_sec" else 0.5
        ),
        _query_with_timeout=lambda key, payload, response_type, timeout: calls.append(
            (key, payload, response_type, timeout)
        ),
    )
    HexArmBridge._query_mode(bridge, "robot/rpc/set_mode", b"request", object)
    assert calls == [("robot/rpc/set_mode", b"request", object, 3.5)]


def test_ros_commands_delegate_gains_and_gravity_to_the_hardware_controller() -> None:
    sent = []

    class Session:
        def put(self, key, payload):
            sent.append((key, payload))

    bridge = SimpleNamespace(
        _hardware_transition_lock=threading.RLock(),
        _lock=threading.RLock(),
        _latch_runtime_gate_if_needed=lambda: False,
        _session_id=7,
        _hardware_active=True,
        _session=Session(),
        _joint_names=list(JOINT_NAMES),
        prefix="robot",
        get_parameter=lambda name: SimpleNamespace(value=0.01),
        get_logger=lambda: SimpleNamespace(error=lambda message: None),
    )
    message = SimpleNamespace(
        name=list(JOINT_NAMES),
        position=[0.0] * 6,
        velocity=[],
    )
    HexArmBridge._on_ros_command(bridge, message)
    assert len(sent) == 1
    command = pb.JointTrajectory.FromString(sent[0][1])
    assert list(command.points[0].kp) == []
    assert list(command.points[0].kd) == []
    assert list(command.points[0].tau_ff) == []
    assert command.on_timeout == pb.TIMEOUT_BEHAVIOR_FAULT


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


def test_clear_fault_uses_scoped_recovery_session_when_control_session_is_absent() -> None:
    calls = []
    responses = [
        SimpleNamespace(ok=True, session_id=41),
        SimpleNamespace(ok=True, error=""),
        SimpleNamespace(ok=True, error=""),
    ]

    def query(key, payload, response_type):
        del payload, response_type
        calls.append(key)
        return responses.pop(0)

    bridge = SimpleNamespace(
        prefix="robot",
        _hardware_transition_lock=threading.RLock(),
        _lock=threading.RLock(),
        _session_id=0,
        _hardware_active=False,
        _runtime_gate_latched=False,
        _runtime_gate_reason="",
        _query=query,
        _query_mode=query,
    )
    response = SimpleNamespace(success=False, message="")

    HexArmBridge._clear_fault(bridge, None, response)

    assert response.success
    assert response.message == "fault cleared; recovery session released"
    assert calls == [
        "robot/rpc/acquire_session",
        "robot/rpc/clear_fault",
        "robot/rpc/release_session",
    ]
    assert bridge._session_id == 0
    assert not bridge._runtime_gate_latched


def test_clear_fault_retains_unreleased_recovery_session_without_activating() -> None:
    responses = [
        SimpleNamespace(ok=True, session_id=41),
        SimpleNamespace(ok=False, error="motor still faulted"),
        SimpleNamespace(ok=False, error="release rejected"),
    ]

    def query(key, payload, response_type):
        del key, payload, response_type
        return responses.pop(0)

    bridge = SimpleNamespace(
        prefix="robot",
        _hardware_transition_lock=threading.RLock(),
        _lock=threading.RLock(),
        _session_id=0,
        _hardware_active=False,
        _runtime_gate_latched=False,
        _runtime_gate_reason="",
        _query=query,
        _query_mode=query,
    )
    response = SimpleNamespace(success=False, message="")

    HexArmBridge._clear_fault(bridge, None, response)

    assert not response.success
    assert "motor still faulted" in response.message
    assert "release rejected" in response.message
    assert bridge._session_id == 41
    assert not bridge._hardware_active
    assert bridge._runtime_gate_latched


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
        _stream_node=None,
        _stop_streaming=lambda: None,
        _diag_timer="timer",
        _command_sub="subscription",
        _bridge_services=["service_a", "service_b"],
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


def test_observation_allows_uncalibrated_or_faulted_disabled_hardware() -> None:
    snapshot = _ready_snapshot()
    snapshot.driver_state.calibrated = False
    snapshot.driver_state.fault_latched = True
    snapshot.driver_state.fault_reason = "commissioning fault"
    assert _observation_error(snapshot, now=10.0, state_timeout_sec=0.1) is None
    assert _readiness_error(snapshot, now=10.0, state_timeout_sec=0.1) == (
        "hardware is not calibrated"
    )


@pytest.mark.parametrize(
    ("field", "expected"),
    [
        ("profile_valid", "hardware profile is not valid"),
        ("all_motors_online", "not all motors are online"),
        ("feedback_fresh", "motor feedback is not fresh"),
    ],
)
def test_observation_still_requires_trustworthy_live_feedback(
    field: str, expected: str
) -> None:
    snapshot = _ready_snapshot()
    setattr(snapshot.driver_state, field, False)
    assert _observation_error(snapshot, 10.0, 0.1) == expected


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


def test_zenoh_state_callbacks_complete_while_mode_transaction_waits() -> None:
    bridge = SimpleNamespace(
        _hardware_transition_lock=threading.RLock(),
        _lock=threading.RLock(),
        _accept_zenoh_state=True,
        _snapshot=_Snapshot(),
        _joint_state_error_log=_RepeatedErrorLog(),
        _runtime_gate_latched=False,
        _session_id=0,
        _hardware_active=False,
    )
    bridge._latch_runtime_gate_if_needed = (
        lambda: HexArmBridge._latch_runtime_gate_if_needed(bridge)
    )
    completed = threading.Event()
    failures = []

    def receive():
        try:
            HexArmBridge._on_zenoh_joint_state(
                bridge, SimpleNamespace(payload=pb.JointState(q=[0.0] * 6).SerializeToString())
            )
            HexArmBridge._on_zenoh_driver_state(
                bridge, SimpleNamespace(payload=pb.DriverState(
                    mode=pb.OPERATING_MODE_ACTIVE, session_owned=True
                ).SerializeToString())
            )
            completed.set()
        except BaseException as error:
            failures.append(error)

    # The activation service holds this lock while awaiting a delayed mode RPC.
    # Receive callbacks must return so Zenoh can continue delivering the reply.
    with bridge._hardware_transition_lock:
        worker = threading.Thread(target=receive, daemon=True)
        worker.start()
        returned_during_transition = completed.wait(0.3)
    worker.join(timeout=1.0)
    assert not failures
    assert returned_during_transition
    assert bridge._snapshot.driver_state.mode == pb.OPERATING_MODE_ACTIVE
    assert bridge._snapshot.joint_received_at > 0.0


def test_stream_executor_progresses_and_joins_independently_of_management() -> None:
    from rclpy.node import Node

    initialized_here = not rclpy.ok()
    if initialized_here:
        rclpy.init()
    bridge = HexArmBridge()
    ticks = []
    try:
        bridge._stream_node = Node("stream_executor_test", context=bridge.context)
        bridge._diag_timer = bridge._stream_node.create_timer(
            0.005, lambda: ticks.append(time.monotonic()))
        with bridge._hardware_transition_lock:
            bridge._start_streaming()
            time.sleep(0.04)
            assert len(ticks) >= 3
        bridge._destroy_ros_entities()
        count = len(ticks)
        time.sleep(0.02)
        assert len(ticks) == count
        assert bridge._stream_thread is None
        assert bridge._stream_executor is None
        bridge._destroy_ros_entities()
    finally:
        bridge.destroy_node()
        if initialized_here and rclpy.ok():
            rclpy.shutdown()


def test_inactive_ros_commands_do_not_block_the_stream_during_mode_rpc() -> None:
    bridge = SimpleNamespace(
        _lock=threading.RLock(), _hardware_transition_lock=threading.RLock(),
        _hardware_active=False,
    )
    finished = threading.Event()

    def callback():
        HexArmBridge._on_ros_command(bridge, None)
        finished.set()

    with bridge._hardware_transition_lock:
        worker = threading.Thread(target=callback)
        worker.start()
        progressed = finished.wait(0.2)
    worker.join(1.0)
    assert progressed


def test_slow_diagnostic_formatting_does_not_block_stream_and_worker_is_joined():
    from rclpy.node import Node

    initialized_here = not rclpy.ok()
    if initialized_here:
        rclpy.init()
    bridge = HexArmBridge()
    entered = threading.Event()
    release = threading.Event()
    ticks = []
    def slow_diagnostic():
        entered.set()
        release.wait(1.0)
    bridge._publish_diagnostics = slow_diagnostic
    try:
        bridge._stream_node = Node("diagnostic_isolation_test", context=bridge.context)
        bridge._diag_timer = bridge._stream_node.create_timer(.005, lambda: ticks.append(1))
        bridge._start_streaming()
        assert entered.wait(.5)
        count = len(ticks)
        time.sleep(.04)
        assert len(ticks) >= count + 3
        release.set()
        bridge._destroy_ros_entities()
        assert bridge._diagnostic_thread is None
        assert bridge._stream_thread is None
    finally:
        release.set()
        bridge.destroy_node()
        if initialized_here and rclpy.ok():
            rclpy.shutdown()


def test_high_rate_snapshot_does_not_format_diagnostics():
    now = time.monotonic()
    snapshot = _ready_snapshot(now)
    forbidden = SimpleNamespace(publish=lambda _: pytest.fail("stream published diagnostics"))
    bridge = SimpleNamespace(_latch_runtime_gate_if_needed=lambda: False,
        _lock=threading.RLock(), _snapshot=snapshot, _runtime_gate_latched=False,
        _state_pub=None, _driver_pub=None, _diag_pub=forbidden)
    HexArmBridge._publish_snapshot(bridge)


def test_transport_trace_is_bounded_and_drains_after_overflow(tmp_path):
    from hex_arm_bridge.trace import TransportTrace
    import csv

    release = threading.Event()
    class PausedTrace(TransportTrace):
        def _drain(self):
            release.wait(1.)
            super()._drain()
    trace = PausedTrace(capacity=2, directory=str(tmp_path))
    try:
        for seq in range(20):
            trace.emit("python_receive", seq, 42, 123)
        assert trace._queue.qsize() == 2
        assert trace.dropped == 18
    finally:
        release.set()
        assert trace.close()
    with next(tmp_path.glob("*.csv")).open() as source:
        rows = list(csv.DictReader(source))
    assert [row["stage"] for row in rows] == ["python_receive", "python_receive", "trace_dropped"]
    assert int(rows[-1]["seq"]) == 18
    assert all(int(row["timestamp_ns"]) > 0 for row in rows)


def test_transport_trace_shutdown_wait_is_bounded(tmp_path):
    from hex_arm_bridge.trace import TransportTrace

    release = threading.Event()
    class BlockedTrace(TransportTrace):
        def _drain(self):
            release.wait(1.)
            super()._drain()
    trace = BlockedTrace(directory=str(tmp_path))
    worker = trace._thread
    try:
        assert worker.daemon
        assert not trace.close(timeout_sec=.001)
    finally:
        release.set()
        worker.join(1.)
        assert not worker.is_alive()


@pytest.mark.parametrize("trace_enabled", [False, True])
def test_command_header_correlation_is_opt_in(trace_enabled):
    from sensor_msgs.msg import JointState

    commands = []
    events = []
    trace = SimpleNamespace(enabled=trace_enabled,
        emit=lambda *args: events.append(args))
    bridge = SimpleNamespace(_lock=threading.RLock(),
        _hardware_transition_lock=threading.RLock(), _hardware_active=True,
        _session_id=42, _session=SimpleNamespace(put=lambda key, payload: commands.append(payload)),
        _latch_runtime_gate_if_needed=lambda: False, _joint_names=list(JOINT_NAMES),
        _trace=trace, _trace_seq=0, prefix="arm",
        get_parameter=lambda name: SimpleNamespace(value=.01),
        get_logger=lambda: SimpleNamespace(error=lambda error: pytest.fail(error)))
    message = JointState()
    message.name = list(JOINT_NAMES)
    message.position = [0.] * 6
    message.velocity = [0.] * 6
    message.header.stamp = RosTime(sec=123, nanosec=456)
    HexArmBridge._on_ros_command(bridge, message)
    assert len(commands) == 1
    command = pb.JointTrajectory.FromString(commands[0])
    assert command.session_id == 42
    assert command.t_from_start_ns == [10_000_000]
    assert command.on_timeout == pb.TIMEOUT_BEHAVIOR_FAULT
    assert command.HasField("header") == trace_enabled
    if trace_enabled:
        assert command.header.seq == 1
        assert command.header.stamp_ns > 0
        assert events == [("python_receive", 1, 42, 123_000_000_456),
                          ("zenoh_put", 1, 42, 123_000_000_456)]
    else:
        assert events == []
