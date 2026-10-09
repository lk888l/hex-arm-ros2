from __future__ import annotations

import threading
import time
from typing import Any

from diagnostic_msgs.msg import DiagnosticArray
from hex_arm_msgs.msg import DriverState as RosDriverState
from hex_arm_msgs.srv import DiscoverMotors, SetGravity, SetOperatingMode
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState as RosJointState
from std_srvs.srv import Trigger

from hex_arm_bridge.pb import robot_api_pb2 as pb
from hex_arm_bridge.protocol import JOINT_NAMES, require_api_major


from hex_arm_bridge.mapping import _temperature_vector_error, _diagnostic_values, _operating_mode_text
from hex_arm_bridge.readiness import (
    _Snapshot,
    _positive_timeout,
    _BridgeShutdownRequested,
    _quiesce_multithreaded_executor,
    _RepeatedErrorLog,
    _configure_zenoh_endpoint,
    _generic_response_error,
    _require_fault_timeout_support,
    _observation_error,
    _readiness_error,
    _active_ownership_error,
)
from hex_arm_bridge.stream import StreamMixin
from hex_arm_bridge.management import ManagementMixin
from hex_arm_bridge.trace import TransportTrace


class HexArmBridge(StreamMixin, ManagementMixin, LifecycleNode):
    """Owns lifecycle, session and all stream state under shared synchronization."""

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
        self.declare_parameter("stream_period_sec", 0.010)
        self.declare_parameter("diagnostic_period_sec", 0.200)
        self.declare_parameter(
            "urdf_path", "", ParameterDescriptor(description="Controller publishes this URDF to existing GUI clients"))

        self._session: Any = None
        # Keep long management RPCs from starving the command and state paths.
        # Each group is internally serialized, but the executor may run the
        # three groups concurrently.
        self._management_callback_group = MutuallyExclusiveCallbackGroup()
        self._stream_node = None
        self._stream_executor = None
        self._stream_thread = None
        self._stream_stop = threading.Event()
        self._diagnostic_stop = threading.Event()
        self._diagnostic_wakeup = threading.Event()
        self._diagnostic_thread = None
        self._diagnostic_key = None
        try:
            self._trace = TransportTrace()
        except OSError as error:
            self.get_logger().warning(f"transport trace disabled: {error}")
            self._trace = TransportTrace(directory="")
        self._trace_seq = 0
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
            stream_period = _positive_timeout(
                self.get_parameter("stream_period_sec").value, "stream_period_sec"
            )
            _positive_timeout(
                self.get_parameter("command_period_sec").value, "command_period_sec"
            )
            _positive_timeout(
                self.get_parameter("diagnostic_period_sec").value, "diagnostic_period_sec"
            )
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
            # Keep the 100 Hz data path out of Jazzy's shared worker-pool
            # wait-set churn. Management services remain on the lifecycle node.
            self._stream_node = Node(
                "hex_arm_bridge_stream", context=self.context, use_global_arguments=False)
            self._command_sub = self._stream_node.create_subscription(
                RosJointState,
                "/hex_arm/internal/command",
                self._on_ros_command,
                1,
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
            # The timer remains the high-rate observation/gate check. Human
            # diagnostic formatting runs on its own low-rate worker.
            self._diag_timer = self._stream_node.create_timer(stream_period, self._publish_snapshot)
            self._bridge_services.append(self.create_service(
                Trigger, "/hex_arm_bridge/damped_stop", self._damped_stop,
                callback_group=self._management_callback_group))
            self._start_streaming()
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
        if not self._trace.close():
            self.get_logger().warning("transport trace drain timed out; incomplete CSV retained")
        if self._trace.error is not None:
            self.get_logger().warning(f"transport trace writer failed: {self._trace.error}")
        super().destroy_node()

    def _destroy_ros_entities(self) -> None:
        self._stop_streaming()
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

        stream_owner = self._stream_node or self
        entities = []
        if timer is not None:
            entities.append(("stream timer", stream_owner.destroy_timer, timer))
        if command_sub is not None:
            entities.append(("command subscription", stream_owner.destroy_subscription, command_sub))
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
        if self._stream_node is not None:
            self._stream_node.destroy_node()
            self._stream_node = None
