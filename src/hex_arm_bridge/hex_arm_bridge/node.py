from __future__ import annotations

import json
import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Optional

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from hex_arm_msgs.msg import DriverState as RosDriverState, MotorIdentity as RosMotorIdentity
from hex_arm_msgs.srv import DiscoverMotors, SetGravity, SetOperatingMode
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState as RosJointState
from std_srvs.srv import Trigger

from hex_arm_bridge.pb import robot_api_pb2 as pb
from hex_arm_bridge.protocol import JOINT_NAMES, reorder, require_api_major


@dataclass
class _Snapshot:
    joint_state: pb.JointState | None = None
    driver_state: pb.DriverState | None = None
    joint_received_at: float = 0.0
    driver_received_at: float = 0.0


def _positive_timeout(value: Any, name: str) -> float:
    timeout = float(value)
    if not math.isfinite(timeout) or timeout <= 0.0:
        raise ValueError(f"{name} must be finite and greater than zero")
    return timeout


class _BridgeShutdownRequested(RuntimeError):
    """Internal control-flow exception used to abort blocking lifecycle work."""


def _quiesce_multithreaded_executor(executor: Any) -> list[BaseException]:
    """Stop queued callbacks before any node entity is destroyed.

    ROS 2 Jazzy's ``MultiThreadedExecutor.shutdown()`` can destroy its guard
    condition while callbacks submitted to its Python thread pool are still
    queued.  A queued callback which starts after ``destroy_node()`` then
    raises ``InvalidHandle`` ("cannot use Destroyable ..."), and the executor
    never retrieves that task's exception because spinning has already
    stopped.  Quiesce the worker pool first, cancel work which has not started,
    and explicitly retrieve results from every callback which did run.

    This deliberately uses the Jazzy executor's private worker/future lists;
    callers feature-check them and fail closed if the executor implementation
    no longer exposes the required shutdown contract.
    """
    worker_pool = getattr(executor, "_executor", None)
    callback_futures = getattr(executor, "_futures", None)
    if worker_pool is None or callback_futures is None:
        raise RuntimeError(
            "MultiThreadedExecutor does not expose the callback-drain API "
            "required for safe bridge shutdown"
        )

    # No more spin_once() calls are allowed once this starts.  Running bridge
    # callbacks have already received request_stop(), so their Zenoh waits are
    # cooperative; pending callbacks are cancelled without touching ROS
    # entities which are about to be destroyed.
    worker_pool.shutdown(wait=True, cancel_futures=True)

    errors: list[BaseException] = []
    for future in list(callback_futures):
        if not future.done():
            # The corresponding ThreadPoolExecutor item was cancelled before
            # invoking the rclpy Task.  Cancel the rclpy Task as well so its
            # never-started handler coroutine is closed instead of producing
            # a separate "coroutine ... was never awaited" warning.
            future.cancel()
        try:
            future.result()
        except BaseException as error:  # Future.__del__ warns for any BaseException.
            errors.append(error)
    callback_futures.clear()
    return errors


@dataclass
class _RepeatedErrorLog:
    """Keep a high-rate repeated input error visible without flooding logs."""

    interval_sec: float = 5.0
    _current_error: str | None = None
    _last_logged_at: float = 0.0
    _suppressed_count: int = 0

    def __post_init__(self) -> None:
        self.interval_sec = _positive_timeout(
            self.interval_sec, "error log interval"
        )

    def report(self, error: Any, now: float) -> Optional[str]:
        detail = str(error).strip() or type(error).__name__
        error_changed = detail != self._current_error
        interval_elapsed = (
            self._current_error is not None
            and (
                not math.isfinite(now)
                or now < self._last_logged_at
                or now - self._last_logged_at >= self.interval_sec
            )
        )
        if error_changed or interval_elapsed:
            suffix = (
                f" ({self._suppressed_count} repeated samples suppressed)"
                if not error_changed and self._suppressed_count
                else ""
            )
            self._current_error = detail
            self._last_logged_at = now
            self._suppressed_count = 0
            return f"{detail}{suffix}"
        self._suppressed_count += 1
        return None

    def recover(self) -> Optional[str]:
        if self._current_error is None:
            return None
        suffix = (
            f" ({self._suppressed_count} additional repeated samples suppressed)"
            if self._suppressed_count
            else ""
        )
        detail = f"{self._current_error}{suffix}"
        self.reset()
        return detail

    def reset(self) -> None:
        self._current_error = None
        self._last_logged_at = 0.0
        self._suppressed_count = 0


def _configure_zenoh_endpoint(config: Any, connect: Any) -> None:
    """Use a deterministic direct peer route when an endpoint is provided."""
    endpoint = str(connect).strip()
    if not endpoint:
        return
    # A peer can connect directly to the controller's peer listener and still
    # declare/query the bridge API. Zenoh client mode expects a router endpoint
    # and therefore cannot establish this controller-to-bridge peer link.
    config.insert_json5("mode", '"peer"')
    config.insert_json5("connect/endpoints", json.dumps([endpoint]))
    config.insert_json5("scouting/multicast/enabled", "false")


def _generic_response_error(response: Any, operation: str) -> Optional[str]:
    if response is None:
        return f"{operation} returned no response"
    if bool(getattr(response, "ok", False)):
        return None
    detail = str(getattr(response, "error", "")).strip()
    return f"{operation} rejected: {detail}" if detail else f"{operation} rejected"


def _require_fault_timeout_support(supported_timeouts: Any) -> None:
    supported = {int(value) for value in supported_timeouts}
    if pb.TIMEOUT_BEHAVIOR_FAULT not in supported:
        raise RuntimeError(
            "controller does not advertise TIMEOUT_BEHAVIOR_FAULT; "
            "refusing to stream real-hardware commands"
        )


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


def _observation_error(
    snapshot: _Snapshot, now: float, state_timeout_sec: float
) -> Optional[str]:
    """Return errors that make even read-only live-state observation unreliable."""
    timeout = _positive_timeout(state_timeout_sec, "state_timeout_sec")
    if not math.isfinite(now):
        return "monotonic clock is invalid"

    joint = snapshot.joint_state
    if joint is None:
        return "joint state has not been received"
    positions = getattr(joint, "q", ())
    if len(positions) != 6 or not all(math.isfinite(float(value)) for value in positions):
        return "joint state does not contain six finite positions"
    joint_age = now - snapshot.joint_received_at
    if (
        not math.isfinite(snapshot.joint_received_at)
        or snapshot.joint_received_at <= 0.0
        or joint_age < 0.0
        or joint_age > timeout
    ):
        return "joint state is stale"

    driver = snapshot.driver_state
    if driver is None:
        return "driver state has not been received"
    driver_age = now - snapshot.driver_received_at
    if (
        not math.isfinite(snapshot.driver_received_at)
        or snapshot.driver_received_at <= 0.0
        or driver_age < 0.0
        or driver_age > timeout
    ):
        return "driver state is stale"
    if not bool(getattr(driver, "profile_valid", False)):
        return "hardware profile is not valid"
    if not bool(getattr(driver, "all_motors_online", False)):
        return "not all motors are online"
    if not bool(getattr(driver, "feedback_fresh", False)):
        return "motor feedback is not fresh"
    return None


def _readiness_error(
    snapshot: _Snapshot, now: float, state_timeout_sec: float
) -> Optional[str]:
    """Return the first fail-closed error that blocks torque-producing modes."""
    observation_error = _observation_error(snapshot, now, state_timeout_sec)
    if observation_error is not None:
        return observation_error
    driver = snapshot.driver_state
    assert driver is not None
    if not bool(getattr(driver, "calibrated", False)):
        return "hardware is not calibrated"
    if bool(getattr(driver, "fault_latched", True)):
        reason = str(getattr(driver, "fault_reason", "")).strip()
        return f"controller fault is latched: {reason}" if reason else "controller fault is latched"
    return None


def _active_ownership_error(
    snapshot: _Snapshot,
    now: float,
    state_timeout_sec: float,
    active_mode: int,
    driver_received_after: Optional[float] = None,
) -> Optional[str]:
    readiness_error = _readiness_error(snapshot, now, state_timeout_sec)
    if readiness_error is not None:
        return readiness_error
    if (
        driver_received_after is not None
        and snapshot.driver_received_at <= driver_received_after
    ):
        return "no new driver state received after ACTIVE request"
    driver = snapshot.driver_state
    if not bool(getattr(driver, "session_owned", False)):
        return "driver does not confirm session ownership"
    if int(getattr(driver, "mode", -1)) != int(active_mode):
        return "driver does not confirm ACTIVE mode"
    return None


class HexArmBridge(LifecycleNode):
    """Protocol adapter only; trajectory generation and motor safety live elsewhere."""

    def __init__(self) -> None:
        super().__init__("hex_arm_bridge")
        self.declare_parameter("robot_prefix", "hexmeow/wsl/firefly_y6_0")
        self.declare_parameter("zenoh_connect", "")
        self.declare_parameter("required_api_major", 0)
        self.declare_parameter("state_timeout_sec", 0.100)
        self.declare_parameter("startup_timeout_sec", 15.0)
        self.declare_parameter("query_timeout_sec", 0.500)
        self.declare_parameter("mode_transition_timeout_sec", 5.0)
        self.declare_parameter("command_period_sec", 0.010)
        self.declare_parameter(
            "urdf_path", "", ParameterDescriptor(description="Controller publishes this URDF to existing GUI clients"))

        self._session: Any = None
        # Keep long management RPCs from starving the command and state paths.
        # Each group is internally serialized, but the executor may run the
        # three groups concurrently.
        self._management_callback_group = MutuallyExclusiveCallbackGroup()
        self._command_callback_group = MutuallyExclusiveCallbackGroup()
        self._state_callback_group = MutuallyExclusiveCallbackGroup()
        self._subscribers: list[Any] = []
        self._session_id = 0
        self._hardware_active = False
        self._lock = threading.RLock()
        self._hardware_transition_lock = threading.RLock()
        self._destroy_lock = threading.Lock()
        self._destroyed = False
        self._shutdown_requested = threading.Event()
        self._configuration_idle = threading.Event()
        self._configuration_idle.set()
        self._active_query_tokens: set[Any] = set()
        # rclpy's signal handler shuts the Context down before the executor's
        # worker callback necessarily returns.  Wake configuration/query waits
        # immediately so process teardown never has to wait for the full
        # startup timeout.
        self.context.on_shutdown(self.request_stop)
        self._accept_zenoh_state = False
        self._runtime_gate_latched = False
        self._runtime_gate_reason = ""
        self._snapshot = _Snapshot()
        self._joint_state_error_log = _RepeatedErrorLog()
        self._joint_names = list(JOINT_NAMES)
        self._state_pub = None
        self._driver_pub = None
        self._diag_pub = None
        self._diag_timer = None
        self._command_sub = None
        # Do not use ``_services`` here: rclpy.node.Node owns that attribute
        # and its executor uses it to discover parameter and lifecycle service
        # entities.  Shadowing it leaves the services visible in the ROS graph
        # but prevents their callbacks from ever being scheduled.
        self._bridge_services: list[Any] = []

    @property
    def prefix(self) -> str:
        return str(self.get_parameter("robot_prefix").value).rstrip("/")

    def request_stop(self) -> None:
        """Request cooperative cancellation; safe from rclpy's signal thread."""
        self._shutdown_requested.set()
        with self._lock:
            tokens = list(self._active_query_tokens)
        for token in tokens:
            try:
                token.cancel()
            except Exception:
                pass

    def wait_for_configuration_idle(self, timeout_sec: float) -> bool:
        return self._configuration_idle.wait(
            _positive_timeout(timeout_sec, "configuration shutdown timeout")
        )

    def _raise_if_stop_requested(self) -> None:
        if self._shutdown_requested.is_set():
            raise _BridgeShutdownRequested("bridge shutdown requested")

    def _wait_interruptibly(self, timeout_sec: float) -> None:
        if self._shutdown_requested.wait(max(0.0, timeout_sec)):
            self._raise_if_stop_requested()

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self._configuration_idle.clear()
        try:
            return self._configure(state)
        finally:
            self._configuration_idle.set()

    def _configure(self, state: State) -> TransitionCallbackReturn:
        del state
        try:
            self._raise_if_stop_requested()
            self.get_logger().info("bridge configuration started")
            previous_release_error = self._safe_release()
            self._close_zenoh()
            self._destroy_ros_entities()
            if previous_release_error is not None:
                raise RuntimeError(f"previous session was not released: {previous_release_error}")
            startup_timeout = _positive_timeout(
                self.get_parameter("startup_timeout_sec").value, "startup_timeout_sec"
            )
            _positive_timeout(self.get_parameter("query_timeout_sec").value, "query_timeout_sec")
            _positive_timeout(
                self.get_parameter("mode_transition_timeout_sec").value,
                "mode_transition_timeout_sec",
            )
            _positive_timeout(self.get_parameter("state_timeout_sec").value, "state_timeout_sec")
            deadline = time.monotonic() + startup_timeout
            with self._lock:
                self._snapshot = _Snapshot()
            import zenoh

            config = zenoh.Config()
            connect = str(self.get_parameter("zenoh_connect").value)
            _configure_zenoh_endpoint(config, connect)
            self.get_logger().info(
                f"opening Zenoh session via {connect.strip() or 'default discovery'}"
            )
            self._raise_if_stop_requested()
            self._session = zenoh.open(config)
            self._raise_if_stop_requested()
            self.get_logger().info("Zenoh session opened; waiting for controller API")
            description = self._query_with_retry(
                f"{self.prefix}/description", b"", pb.RobotDescription, deadline
            )
            if description is None or description.api_version is None:
                raise RuntimeError("controller description query returned no API version")
            require_api_major(description.api_version.major, int(self.get_parameter("required_api_major").value))
            arm = self._query_with_retry(
                f"{self.prefix}/arm/description", b"", pb.ArmDescription, deadline
            )
            if arm is None or arm.dof != 6 or set(arm.joint_names) != set(JOINT_NAMES):
                raise RuntimeError("controller arm description is not the expected six-joint Firefly Y6")
            _require_fault_timeout_support(arm.supported_timeouts)
            self._joint_names = list(arm.joint_names)

            with self._lock:
                self._accept_zenoh_state = True
            self._subscribers = [
                self._session.declare_subscriber(f"{self.prefix}/arm/joint_state", self._on_zenoh_joint_state),
                self._session.declare_subscriber(f"{self.prefix}/driver_state", self._on_zenoh_driver_state),
            ]
            self.get_logger().info("controller API verified; waiting for fresh six-axis state")
            self._wait_for_observation(deadline)
            state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self._state_pub = self.create_lifecycle_publisher(RosJointState, "/hex_arm/internal/state", state_qos)
            self._driver_pub = self.create_lifecycle_publisher(RosDriverState, "/hex_arm/driver_state", 10)
            self._diag_pub = self.create_lifecycle_publisher(DiagnosticArray, "/diagnostics", 10)
            self._command_sub = self.create_subscription(
                RosJointState,
                "/hex_arm/internal/command",
                self._on_ros_command,
                1,
                callback_group=self._command_callback_group,
            )
            self._bridge_services.append(
                self.create_service(
                    Trigger,
                    "/hex_arm_bridge/activate_hardware",
                    self._activate_hardware,
                    callback_group=self._management_callback_group,
                )
            )
            self._bridge_services.append(
                self.create_service(
                    Trigger,
                    "/hex_arm_bridge/deactivate_hardware",
                    self._deactivate_hardware,
                    callback_group=self._management_callback_group,
                )
            )
            self._bridge_services.append(
                self.create_service(
                    Trigger,
                    "/hex_arm/clear_fault",
                    self._clear_fault,
                    callback_group=self._management_callback_group,
                )
            )
            self._bridge_services.append(
                self.create_service(
                    DiscoverMotors,
                    "/hex_arm/discover_motors",
                    self._discover_motors,
                    callback_group=self._management_callback_group,
                )
            )
            self._bridge_services.append(
                self.create_service(
                    SetOperatingMode,
                    "/hex_arm/set_mode",
                    self._set_mode,
                    callback_group=self._management_callback_group,
                )
            )
            self._bridge_services.append(
                self.create_service(
                    SetGravity,
                    "/hex_arm/set_gravity",
                    self._set_gravity,
                    callback_group=self._management_callback_group,
                )
            )
            self._diag_timer = self.create_timer(
                0.01,
                self._publish_snapshot,
                callback_group=self._state_callback_group,
            )
            self.get_logger().info("bridge configured with fresh DISABLED-state observation")
            return TransitionCallbackReturn.SUCCESS
        except _BridgeShutdownRequested:
            self.get_logger().info("bridge configuration cancelled by shutdown request")
            release_error = self._safe_release()
            self._close_zenoh()
            self._destroy_ros_entities()
            if release_error is not None:
                self.get_logger().error(
                    f"configuration cancellation release failed: {release_error}"
                )
            return TransitionCallbackReturn.ERROR
        except Exception as error:  # lifecycle boundary must fail closed
            self.get_logger().error(f"configuration rejected: {error}")
            release_error = self._safe_release()
            self._close_zenoh()
            self._destroy_ros_entities()
            if release_error is not None:
                self.get_logger().error(f"configuration rollback release failed: {release_error}")
            return TransitionCallbackReturn.ERROR

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        # Lifecycle activation publishes observed state; it does not acquire a
        # hardware session or request torque. Commissioning therefore remains
        # observable while `calibrated=false` or a drive fault is latched.
        error = self._state_observation_error()
        if error is not None:
            release_error = self._safe_release()
            self.get_logger().error(f"activation rejected: {error}")
            if release_error is not None:
                self.get_logger().error(f"activation rollback release failed: {release_error}")
            return TransitionCallbackReturn.ERROR
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        release_error = self._safe_release()
        result = super().on_deactivate(state)
        if release_error is not None:
            self.get_logger().error(f"deactivation release failed: {release_error}")
            return TransitionCallbackReturn.ERROR
        return result

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        del state
        release_error = self._safe_release()
        self._close_zenoh()
        self._destroy_ros_entities()
        if release_error is not None:
            self.get_logger().error(f"cleanup release failed: {release_error}")
            return TransitionCallbackReturn.ERROR
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        del state
        release_error = self._safe_release()
        self._close_zenoh()
        self._destroy_ros_entities()
        if release_error is not None:
            self.get_logger().error(f"shutdown release failed: {release_error}")
            return TransitionCallbackReturn.ERROR
        return TransitionCallbackReturn.SUCCESS

    def destroy_node(self) -> None:
        self.request_stop()
        with self._destroy_lock:
            if self._destroyed:
                return
            self._destroyed = True
        release_error = self._safe_release()
        self._close_zenoh()
        self._destroy_ros_entities()
        if release_error is not None:
            self.get_logger().error(f"destroy release failed: {release_error}")
        super().destroy_node()

    def _destroy_ros_entities(self) -> None:
        with self._lock:
            timer = self._diag_timer
            self._diag_timer = None
            command_sub = self._command_sub
            self._command_sub = None
            services = list(self._bridge_services)
            self._bridge_services.clear()
            publishers = [self._state_pub, self._driver_pub, self._diag_pub]
            self._state_pub = None
            self._driver_pub = None
            self._diag_pub = None

        entities = []
        if timer is not None:
            entities.append(("diagnostic timer", self.destroy_timer, timer))
        if command_sub is not None:
            entities.append(("command subscription", self.destroy_subscription, command_sub))
        entities.extend(("service", self.destroy_service, service) for service in services)
        entities.extend(
            ("lifecycle publisher", self.destroy_lifecycle_publisher, publisher)
            for publisher in publishers
            if publisher is not None
        )
        for label, destroy, entity in entities:
            try:
                destroy(entity)
            except Exception as error:
                self.get_logger().warning(f"failed to destroy {label}: {error}")

    def _query_with_retry(
        self, key: str, payload: bytes, response_type: Any, deadline: float
    ) -> Any:
        last_error: Optional[str] = None
        while True:
            self._raise_if_stop_requested()
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                detail = f" ({last_error})" if last_error else ""
                raise TimeoutError(f"timed out querying {key}{detail}")
            try:
                query_timeout = _positive_timeout(
                    self.get_parameter("query_timeout_sec").value, "query_timeout_sec"
                )
                response = self._query_with_timeout(
                    key, payload, response_type, min(query_timeout, remaining)
                )
                if response is not None:
                    return response
            except Exception as error:
                last_error = str(error)
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                continue
            self._wait_interruptibly(min(0.05, remaining))

    def _wait_for_observation(self, deadline: float) -> None:
        while True:
            self._raise_if_stop_requested()
            error = self._state_observation_error()
            if error is None:
                return
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise TimeoutError(f"controller did not become ready: {error}")
            self._wait_interruptibly(min(0.01, remaining))

    def _snapshot_copy(self) -> _Snapshot:
        with self._lock:
            return _Snapshot(
                joint_state=self._snapshot.joint_state,
                driver_state=self._snapshot.driver_state,
                joint_received_at=self._snapshot.joint_received_at,
                driver_received_at=self._snapshot.driver_received_at,
            )

    def _state_observation_error(self) -> Optional[str]:
        return _observation_error(
            self._snapshot_copy(),
            time.monotonic(),
            self.get_parameter("state_timeout_sec").value,
        )

    def _state_readiness_error(self) -> Optional[str]:
        return _readiness_error(
            self._snapshot_copy(),
            time.monotonic(),
            self.get_parameter("state_timeout_sec").value,
        )

    def _state_active_ownership_error(
        self, driver_received_after: Optional[float] = None
    ) -> Optional[str]:
        return _active_ownership_error(
            self._snapshot_copy(),
            time.monotonic(),
            self.get_parameter("state_timeout_sec").value,
            pb.OPERATING_MODE_ACTIVE,
            driver_received_after,
        )

    def _wait_for_active_ownership(
        self, driver_received_after: float, deadline: float
    ) -> None:
        while True:
            self._raise_if_stop_requested()
            error = self._state_active_ownership_error(driver_received_after)
            if error is None:
                return
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise TimeoutError(f"driver did not confirm ACTIVE ownership: {error}")
            self._wait_interruptibly(min(0.01, remaining))

    def _latch_runtime_gate_if_needed(self) -> bool:
        with self._hardware_transition_lock:
            with self._lock:
                if self._runtime_gate_latched:
                    return True
                session_id = self._session_id
                hardware_active = self._hardware_active
                snapshot = _Snapshot(
                    joint_state=self._snapshot.joint_state,
                    driver_state=self._snapshot.driver_state,
                    joint_received_at=self._snapshot.joint_received_at,
                    driver_received_at=self._snapshot.driver_received_at,
                )
            if session_id == 0 or not hardware_active:
                return False
            try:
                error = _active_ownership_error(
                    snapshot,
                    time.monotonic(),
                    self.get_parameter("state_timeout_sec").value,
                    pb.OPERATING_MODE_ACTIVE,
                )
            except Exception as exception:
                error = f"runtime gate evaluation failed: {exception}"
            if error is None:
                return False

            newly_latched = False
            with self._lock:
                if (
                    self._session_id == session_id
                    and self._hardware_active
                    and not self._runtime_gate_latched
                ):
                    self._hardware_active = False
                    self._runtime_gate_latched = True
                    self._runtime_gate_reason = error
                    newly_latched = True
                latched = self._runtime_gate_latched
            if newly_latched:
                self.get_logger().error(
                    f"runtime safety gate latched, commands and internal state stopped: {error}"
                )
            return latched

    def _query(self, key: str, payload: bytes, response_type: Any) -> Any | None:
        timeout = _positive_timeout(
            self.get_parameter("query_timeout_sec").value, "query_timeout_sec"
        )
        return self._query_with_timeout(key, payload, response_type, timeout)

    def _query_mode(self, key: str, payload: bytes, response_type: Any) -> Any | None:
        """Use the management-plane timeout for serialized six-axis transitions."""
        timeout = _positive_timeout(
            self.get_parameter("mode_transition_timeout_sec").value,
            "mode_transition_timeout_sec",
        )
        return self._query_with_timeout(key, payload, response_type, timeout)

    def _query_with_timeout(
        self,
        key: str,
        payload: bytes,
        response_type: Any,
        timeout_sec: float,
        *,
        cancel_on_shutdown: bool = True,
    ) -> Optional[Any]:
        timeout = _positive_timeout(timeout_sec, "query timeout")
        if cancel_on_shutdown:
            self._raise_if_stop_requested()
        import zenoh

        cancellation_token = zenoh.CancellationToken()
        with self._lock:
            if cancel_on_shutdown and self._shutdown_requested.is_set():
                raise _BridgeShutdownRequested("bridge shutdown requested")
            session = self._session
            if cancel_on_shutdown:
                self._active_query_tokens.add(cancellation_token)
        if session is None:
            with self._lock:
                self._active_query_tokens.discard(cancellation_token)
            raise RuntimeError("Zenoh session is not open")
        try:
            # zenoh-python's CancellationToken interrupts the blocking reply
            # iterator.  Without it a SIGINT during a lifecycle transition can
            # leave the executor worker alive until the entire RPC timeout.
            replies = session.get(
                key,
                payload=payload,
                timeout=timeout,
                cancellation_token=cancellation_token,
            )
            for reply in replies:
                if cancel_on_shutdown:
                    self._raise_if_stop_requested()
                sample = reply.ok
                if sample is not None:
                    return response_type.FromString(bytes(sample.payload))
            if cancel_on_shutdown:
                self._raise_if_stop_requested()
            return None
        finally:
            with self._lock:
                self._active_query_tokens.discard(cancellation_token)

    def _on_zenoh_joint_state(self, sample: Any) -> None:
        with self._lock:
            if not self._accept_zenoh_state:
                return
        try:
            message = pb.JointState.FromString(bytes(sample.payload))
            if len(message.q) != 6 or not all(math.isfinite(value) for value in message.q):
                raise ValueError("joint state does not contain six finite positions")
        except Exception as error:
            with self._lock:
                if not self._accept_zenoh_state:
                    return
                log_detail = self._joint_state_error_log.report(
                    error, time.monotonic()
                )
            if log_detail is not None:
                self.get_logger().error(f"invalid Zenoh joint state: {log_detail}")
            return

        with self._lock:
            if not self._accept_zenoh_state:
                return
            self._snapshot.joint_state = message
            self._snapshot.joint_received_at = time.monotonic()
            recovered_from = self._joint_state_error_log.recover()
        if recovered_from is not None:
            self.get_logger().info(
                f"valid Zenoh joint state resumed after invalid input: {recovered_from}"
            )
        self._latch_runtime_gate_if_needed()

    def _on_zenoh_driver_state(self, sample: Any) -> None:
        try:
            message = pb.DriverState.FromString(bytes(sample.payload))
            with self._lock:
                if not self._accept_zenoh_state:
                    return
                self._snapshot.driver_state = message
                self._snapshot.driver_received_at = time.monotonic()
            self._latch_runtime_gate_if_needed()
        except Exception as error:
            self.get_logger().error(f"invalid Zenoh driver state: {error}")

    def _on_ros_command(self, message: RosJointState) -> None:
        with self._hardware_transition_lock:
            if self._latch_runtime_gate_if_needed():
                return
            with self._lock:
                session_id = self._session_id
                active = self._hardware_active
                session = self._session
            if not active or session_id == 0 or session is None:
                return
            try:
                q = reorder(message.position, message.name, self._joint_names)
                dq = reorder(message.velocity, message.name, self._joint_names) if message.velocity else [0.0] * 6
                command = pb.JointTrajectory(
                    session_id=session_id,
                    # Empty kp/kd intentionally delegate the reviewed per-axis defaults
                    # in the hardware profile to the Rust safety boundary.
                    # Empty gains delegate to the reviewed hardware profile.
                    # Empty tau_ff delegates gravity compensation to the Rust
                    # controller at the latest measured pose. Legacy clients
                    # may still send an explicit six-axis feed-forward vector.
                    points=[pb.JointSetpoint(q=q, dq=dq, kp=[], kd=[], tau_ff=[])],
                    t_from_start_ns=[int(float(self.get_parameter("command_period_sec").value) * 1e9)],
                    # The hardware controller advertises and accepts only the
                    # fail-closed FAULT contract. A bounded-deceleration ramp
                    # has not been commissioned, so do not claim one here.
                    on_timeout=pb.TIMEOUT_BEHAVIOR_FAULT,
                )
                session.put(f"{self.prefix}/arm/command", command.SerializeToString())
            except Exception as error:
                self.get_logger().error(f"command rejected: {error}")

    def _activate_hardware(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        with self._hardware_transition_lock:
            with self._destroy_lock:
                destroyed = self._destroyed
            if destroyed:
                response.success = False
                response.message = "activation rejected: bridge is shutting down"
                return response

            readiness_error = self._state_readiness_error()
            if readiness_error is not None:
                release_error = self._safe_release()
                response.success = False
                response.message = f"activation rejected: {readiness_error}"
                if release_error is not None:
                    response.message += f" and previous session release failed: {release_error}"
                return response

            with self._lock:
                session_id = self._session_id
                hardware_active = self._hardware_active
            if hardware_active and session_id != 0:
                ownership_error = self._state_active_ownership_error()
                if ownership_error is None:
                    response.success = True
                    response.message = "controller is already ACTIVE with the owned session"
                    return response
                self.get_logger().warning(
                    f"existing ACTIVE state is inconsistent and will be reacquired: {ownership_error}"
                )
                release_error = self._safe_release()
                if release_error is not None:
                    response.success = False
                    response.message = f"activation rejected: previous session release failed: {release_error}"
                    return response
            elif hardware_active or session_id != 0:
                release_error = self._safe_release()
                if release_error is not None:
                    response.success = False
                    response.message = f"activation rejected: previous session release failed: {release_error}"
                    return response

            acquired_session_id = 0
            try:
                acquired = self._query(
                    f"{self.prefix}/rpc/acquire_session",
                    pb.AcquireSessionRequest(client_name="ros2_control").SerializeToString(),
                    pb.AcquireSessionResponse,
                )
                if acquired is None or not acquired.ok or acquired.session_id == 0:
                    raise RuntimeError("exclusive session acquisition failed")
                acquired_session_id = acquired.session_id

                readiness_error = self._state_readiness_error()
                if readiness_error is not None:
                    raise RuntimeError(f"readiness changed after session acquisition: {readiness_error}")

                with self._lock:
                    driver_received_before_active = self._snapshot.driver_received_at
                mode = self._query_mode(
                    f"{self.prefix}/rpc/set_mode",
                    pb.SetModeRequest(
                        session_id=acquired_session_id, mode=pb.OPERATING_MODE_ACTIVE
                    ).SerializeToString(),
                    pb.GenericResponse,
                )
                if mode is None or not mode.ok:
                    detail = (
                        mode.error
                        if mode is not None and mode.HasField("error")
                        else "ACTIVE rejected"
                    )
                    raise RuntimeError(detail)

                confirmation_timeout = _positive_timeout(
                    self.get_parameter("mode_transition_timeout_sec").value,
                    "mode_transition_timeout_sec",
                )
                self._wait_for_active_ownership(
                    driver_received_before_active,
                    time.monotonic() + confirmation_timeout,
                )
            except Exception as error:
                rollback_error = None
                if acquired_session_id != 0:
                    try:
                        rollback_error = self._release_session(acquired_session_id)
                    except Exception as exception:
                        rollback_error = str(exception)
                    if rollback_error is not None:
                        with self._lock:
                            self._session_id = acquired_session_id
                            self._hardware_active = False
                            self._runtime_gate_latched = True
                            self._runtime_gate_reason = (
                                f"activation rollback release failed: {rollback_error}"
                            )
                        self.get_logger().error(
                            f"activation rollback retained session for retry: {rollback_error}"
                        )
                response.success = False
                response.message = f"activation rejected: {error}"
                if rollback_error is not None:
                    response.message += f" and rollback release failed: {rollback_error}"
                return response

            with self._lock:
                self._session_id = acquired_session_id
                self._hardware_active = True
                self._runtime_gate_latched = False
                self._runtime_gate_reason = ""
            response.success = True
            response.message = "exclusive session acquired and controller ACTIVE"
            return response

    def _deactivate_hardware(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        release_error = self._safe_release()
        response.success = release_error is None
        response.message = (
            "controller disabled and session released"
            if release_error is None
            else f"deactivation failed: {release_error}"
        )
        return response

    def _set_mode(self, request: SetOperatingMode.Request, response: SetOperatingMode.Response) -> SetOperatingMode.Response:
        if request.mode == pb.OPERATING_MODE_ACTIVE:
            response.success = False
            response.message = "ACTIVE is bound to ros2_control hardware activation"
            return response
        with self._lock:
            session_id = self._session_id
        if session_id == 0:
            response.success = False
            response.message = "no ros2_control-owned session"
            return response
        result = self._query_mode(
            f"{self.prefix}/rpc/set_mode",
            pb.SetModeRequest(session_id=session_id, mode=request.mode).SerializeToString(),
            pb.GenericResponse,
        )
        response.success = bool(result and result.ok)
        response.message = "mode changed" if response.success else "mode rejected"
        return response

    def _clear_fault(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        with self._hardware_transition_lock:
            with self._lock:
                session_id = self._session_id

            temporary_session = session_id == 0
            if temporary_session:
                try:
                    acquired = self._query(
                        f"{self.prefix}/rpc/acquire_session",
                        pb.AcquireSessionRequest(
                            client_name="ros2_fault_recovery"
                        ).SerializeToString(),
                        pb.AcquireSessionResponse,
                    )
                except Exception as error:
                    response.success = False
                    response.message = f"fault clear session acquisition failed: {error}"
                    return response
                if acquired is None or not acquired.ok or acquired.session_id == 0:
                    detail = (
                        acquired.error
                        if acquired is not None and acquired.HasField("error")
                        else "exclusive session is unavailable"
                    )
                    response.success = False
                    response.message = f"fault clear session acquisition failed: {detail}"
                    return response
                session_id = acquired.session_id

            clear_error = None
            try:
                result = self._query_mode(
                    f"{self.prefix}/rpc/clear_fault",
                    pb.ClearFaultRequest(session_id=session_id).SerializeToString(),
                    pb.GenericResponse,
                )
                clear_error = _generic_response_error(result, "fault clear request")
            except Exception as error:
                clear_error = f"fault clear request failed: {error}"

            release_error = None
            if temporary_session:
                try:
                    released = self._query(
                        f"{self.prefix}/rpc/release_session",
                        pb.ReleaseSessionRequest(session_id=session_id).SerializeToString(),
                        pb.GenericResponse,
                    )
                    release_error = _generic_response_error(
                        released, "fault recovery session release"
                    )
                except Exception as error:
                    release_error = f"fault recovery session release failed: {error}"

                if release_error is not None:
                    # Do not lose track of a lease that the controller may
                    # still own.  It remains inactive and can be released by a
                    # later clear/deactivate retry.
                    with self._lock:
                        if self._session_id == 0:
                            self._session_id = session_id
                            self._hardware_active = False
                            self._runtime_gate_latched = True
                            self._runtime_gate_reason = release_error

            errors = [error for error in (clear_error, release_error) if error]
            response.success = not errors
            response.message = (
                "fault cleared; recovery session released"
                if response.success and temporary_session
                else "fault cleared"
                if response.success
                else "; ".join(errors)
            )
            return response

    def _discover_motors(
        self, request: DiscoverMotors.Request, response: DiscoverMotors.Response
    ) -> DiscoverMotors.Response:
        result = self._query(
            f"{self.prefix}/arm/rpc/discover",
            pb.DiscoverMotorsRequest(refresh=request.refresh).SerializeToString(),
            pb.DiscoverMotorsResponse,
        )
        response.success = bool(result and result.ok)
        response.message = "discovery complete" if response.success else "discovery failed"
        if result:
            response.motors = [self._ros_motor(motor) for motor in result.motors]
        return response

    def _set_gravity(self, request: SetGravity.Request, response: SetGravity.Response) -> SetGravity.Response:
        with self._lock:
            session_id = self._session_id
        if session_id == 0:
            response.success = False
            response.message = "gravity setting requires the ros2_control-owned session"
            return response
        result = self._query(
            f"{self.prefix}/arm/rpc/set_gravity",
            pb.SetGravityRequest(
                session_id=session_id, gravity=pb.Vec3(x=request.x, y=request.y, z=request.z)
            ).SerializeToString(),
            pb.GenericResponse,
        )
        response.success = bool(result and result.ok)
        response.message = "gravity updated" if response.success else "gravity rejected"
        return response

    def _publish_snapshot(self) -> None:
        self._latch_runtime_gate_if_needed()
        with self._lock:
            joint = self._snapshot.joint_state
            driver = self._snapshot.driver_state
            joint_received_at = self._snapshot.joint_received_at
            driver_received_at = self._snapshot.driver_received_at
            gate_latched = self._runtime_gate_latched
            gate_reason = self._runtime_gate_reason
            state_pub = self._state_pub
            driver_pub = self._driver_pub
            diag_pub = self._diag_pub
        if joint is not None and state_pub is not None and not gate_latched:
            state = RosJointState()
            state.header.stamp = self.get_clock().now().to_msg()
            state.name = self._joint_names
            state.position = list(joint.q)
            state.velocity = list(joint.dq) if len(joint.dq) == 6 else [0.0] * 6
            state.effort = list(joint.tau_est) if len(joint.tau_est) == 6 else [0.0] * 6
            state_pub.publish(state)
        if driver is not None and driver_pub is not None:
            driver_pub.publish(self._ros_driver(driver))
        if diag_pub is not None:
            now = time.monotonic()
            joint_age = now - joint_received_at if joint_received_at else float("inf")
            driver_age = now - driver_received_at if driver_received_at else float("inf")
            readiness_error = _readiness_error(
                _Snapshot(joint, driver, joint_received_at, driver_received_at),
                now,
                self.get_parameter("state_timeout_sec").value,
            )
            temperature_error = _temperature_vector_error(joint)
            diag = DiagnosticArray()
            diag.header.stamp = self.get_clock().now().to_msg()
            status = DiagnosticStatus()
            status.name = "firefly_y6/driver"
            status.hardware_id = self.prefix
            status.level = (
                DiagnosticStatus.ERROR
                if gate_latched or (driver and driver.fault_latched)
                else DiagnosticStatus.WARN
                if readiness_error is not None or temperature_error is not None
                else DiagnosticStatus.OK
            )
            primary_message = (
                f"runtime gate latched: {gate_reason}"
                if gate_latched
                else driver.fault_reason
                if driver and driver.fault_latched
                else readiness_error
            )
            messages = [message for message in (primary_message, temperature_error) if message]
            status.message = "; ".join(messages) or "joint and driver state are ready"
            status.values = _diagnostic_values(joint, driver, joint_age, driver_age)
            diag.status = [status]
            diag_pub.publish(diag)

    @staticmethod
    def _ros_motor(motor: pb.MotorIdentity) -> RosMotorIdentity:
        result = RosMotorIdentity()
        result.node_id = motor.node_id
        result.vendor_id = motor.vendor_id
        result.product_code = motor.product_code
        result.revision = motor.revision
        result.serial_number = motor.serial_number
        result.model = motor.model
        result.identity_verified = motor.identity_verified
        return result

    def _ros_driver(self, driver: pb.DriverState) -> RosDriverState:
        result = RosDriverState()
        result.stamp = self.get_clock().now().to_msg()
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
        result.motors = [self._ros_motor(motor) for motor in driver.motors]
        return result

    def _release_session(self, session_id: int) -> Optional[str]:
        errors: list[str] = []
        try:
            disabled = self._query_with_timeout(
                f"{self.prefix}/rpc/set_mode",
                pb.SetModeRequest(
                    session_id=session_id, mode=pb.OPERATING_MODE_DISABLED
                ).SerializeToString(),
                pb.GenericResponse,
                _positive_timeout(
                    self.get_parameter("mode_transition_timeout_sec").value,
                    "mode_transition_timeout_sec",
                ),
                cancel_on_shutdown=False,
            )
            error = _generic_response_error(disabled, "DISABLED mode request")
            if error is not None:
                errors.append(error)
        except Exception as error:
            errors.append(f"disable failed: {error}")
        try:
            released = self._query_with_timeout(
                f"{self.prefix}/rpc/release_session",
                pb.ReleaseSessionRequest(session_id=session_id).SerializeToString(),
                pb.GenericResponse,
                _positive_timeout(
                    self.get_parameter("query_timeout_sec").value,
                    "query_timeout_sec",
                ),
                cancel_on_shutdown=False,
            )
            error = _generic_response_error(released, "session release request")
            if error is not None:
                errors.append(error)
        except Exception as error:
            errors.append(f"release failed: {error}")
        return " and ".join(errors) if errors else None

    def _safe_release(self) -> Optional[str]:
        with self._hardware_transition_lock:
            with self._lock:
                session_id = self._session_id
                self._hardware_active = False
                session_open = self._session is not None
                if session_id == 0:
                    self._runtime_gate_latched = False
                    self._runtime_gate_reason = ""
                    return None
            error = (
                self._release_session(session_id)
                if session_open
                else "Zenoh session is not open"
            )
            if error is not None:
                with self._lock:
                    if self._session_id == session_id:
                        self._runtime_gate_latched = True
                        self._runtime_gate_reason = f"session release failed: {error}"
                return error
            with self._lock:
                if self._session_id == session_id:
                    self._session_id = 0
                    self._runtime_gate_latched = False
                    self._runtime_gate_reason = ""
            return None

    def _close_zenoh(self) -> None:
        with self._hardware_transition_lock:
            with self._lock:
                subscribers = list(self._subscribers)
                self._subscribers.clear()
                session = self._session
                self._session = None
                self._accept_zenoh_state = False
                self._snapshot = _Snapshot()
                self._joint_state_error_log.reset()
            for subscriber in subscribers:
                try:
                    subscriber.undeclare()
                except Exception:
                    pass
            if session is not None:
                try:
                    session.close()
                except Exception:
                    pass
