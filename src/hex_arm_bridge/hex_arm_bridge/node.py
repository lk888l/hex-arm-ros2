from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Any

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from hex_arm_msgs.msg import DriverState as RosDriverState, MotorIdentity as RosMotorIdentity
from hex_arm_msgs.srv import DiscoverMotors, SetGravity, SetOperatingMode
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState as RosJointState
from std_srvs.srv import Trigger

from hex_arm_bridge.pb import robot_api_pb2 as pb
from hex_arm_bridge.protocol import JOINT_NAMES, optional_vector, reorder, require_api_major


@dataclass
class _Snapshot:
    joint_state: pb.JointState | None = None
    driver_state: pb.DriverState | None = None
    received_at: float = 0.0


class HexArmBridge(LifecycleNode):
    """Protocol adapter only; trajectory generation and motor safety live elsewhere."""

    def __init__(self) -> None:
        super().__init__("hex_arm_bridge")
        self.declare_parameter("robot_prefix", "hexmeow/wsl/firefly_y6_0")
        self.declare_parameter("zenoh_connect", "")
        self.declare_parameter("required_api_major", 0)
        self.declare_parameter("state_timeout_sec", 0.100)
        self.declare_parameter("command_period_sec", 0.010)
        self.declare_parameter("default_kp", [10.0] * 6)
        self.declare_parameter("default_kd", [1.5] * 6)
        self.declare_parameter(
            "urdf_path", "", ParameterDescriptor(description="Controller publishes this URDF to existing GUI clients"))

        self._session: Any = None
        self._subscribers: list[Any] = []
        self._session_id = 0
        self._hardware_active = False
        self._lock = threading.RLock()
        self._snapshot = _Snapshot()
        self._joint_names = list(JOINT_NAMES)
        self._state_pub = None
        self._driver_pub = None
        self._diag_pub = None
        self._diag_timer = None
        self._command_sub = None

    @property
    def prefix(self) -> str:
        return str(self.get_parameter("robot_prefix").value).rstrip("/")

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        del state
        try:
            import zenoh

            config = zenoh.Config()
            connect = str(self.get_parameter("zenoh_connect").value)
            if connect:
                config.insert_json5("connect/endpoints", f'["{connect}"]')
            self._session = zenoh.open(config)
            description = self._query(f"{self.prefix}/description", b"", pb.RobotDescription)
            if description is None or description.api_version is None:
                raise RuntimeError("controller description query returned no API version")
            require_api_major(description.api_version.major, int(self.get_parameter("required_api_major").value))
            arm = self._query(f"{self.prefix}/arm/description", b"", pb.ArmDescription)
            if arm is None or arm.dof != 6 or set(arm.joint_names) != set(JOINT_NAMES):
                raise RuntimeError("controller arm description is not the expected six-joint Firefly Y6")
            self._joint_names = list(arm.joint_names)

            self._subscribers = [
                self._session.declare_subscriber(f"{self.prefix}/arm/joint_state", self._on_zenoh_joint_state),
                self._session.declare_subscriber(f"{self.prefix}/driver_state", self._on_zenoh_driver_state),
            ]
            state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self._state_pub = self.create_lifecycle_publisher(RosJointState, "/hex_arm/internal/state", state_qos)
            self._driver_pub = self.create_lifecycle_publisher(RosDriverState, "/hex_arm/driver_state", 10)
            self._diag_pub = self.create_lifecycle_publisher(DiagnosticArray, "/diagnostics", 10)
            self._command_sub = self.create_subscription(
                RosJointState, "/hex_arm/internal/command", self._on_ros_command, 1)
            self.create_service(Trigger, "/hex_arm_bridge/activate_hardware", self._activate_hardware)
            self.create_service(Trigger, "/hex_arm_bridge/deactivate_hardware", self._deactivate_hardware)
            self.create_service(Trigger, "/hex_arm/clear_fault", self._clear_fault)
            self.create_service(DiscoverMotors, "/hex_arm/discover_motors", self._discover_motors)
            self.create_service(SetOperatingMode, "/hex_arm/set_mode", self._set_mode)
            self.create_service(SetGravity, "/hex_arm/set_gravity", self._set_gravity)
            self._diag_timer = self.create_timer(0.01, self._publish_snapshot)
            return TransitionCallbackReturn.SUCCESS
        except Exception as error:  # lifecycle boundary must fail closed
            self.get_logger().error(f"configuration rejected: {error}")
            self._close_zenoh()
            return TransitionCallbackReturn.ERROR

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        result = super().on_activate(state)
        return result

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self._safe_release()
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        del state
        self._safe_release()
        self._close_zenoh()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        del state
        self._safe_release()
        self._close_zenoh()
        return TransitionCallbackReturn.SUCCESS

    def _query(self, key: str, payload: bytes, response_type: Any) -> Any | None:
        replies = self._session.get(key, payload=payload)
        for reply in replies:
            sample = reply.ok
            if sample is not None:
                return response_type.FromString(bytes(sample.payload))
        return None

    def _on_zenoh_joint_state(self, sample: Any) -> None:
        try:
            message = pb.JointState.FromString(bytes(sample.payload))
            if len(message.q) != 6:
                raise ValueError("joint state does not contain six positions")
            with self._lock:
                self._snapshot.joint_state = message
                self._snapshot.received_at = time.monotonic()
        except Exception as error:
            self.get_logger().error(f"invalid Zenoh joint state: {error}")

    def _on_zenoh_driver_state(self, sample: Any) -> None:
        try:
            message = pb.DriverState.FromString(bytes(sample.payload))
            with self._lock:
                self._snapshot.driver_state = message
        except Exception as error:
            self.get_logger().error(f"invalid Zenoh driver state: {error}")

    def _on_ros_command(self, message: RosJointState) -> None:
        with self._lock:
            session_id = self._session_id
            active = self._hardware_active
        if not active or session_id == 0:
            return
        try:
            q = reorder(message.position, message.name, self._joint_names)
            dq = reorder(message.velocity, message.name, self._joint_names) if message.velocity else [0.0] * 6
            kp = optional_vector(self.get_parameter("default_kp").value, 6, 10.0)
            kd = optional_vector(self.get_parameter("default_kd").value, 6, 1.5)
            command = pb.JointTrajectory(
                session_id=session_id,
                points=[pb.JointSetpoint(q=q, dq=dq, kp=kp, kd=kd, tau_ff=[0.0] * 6)],
                t_from_start_ns=[int(float(self.get_parameter("command_period_sec").value) * 1e9)],
                on_timeout=pb.TIMEOUT_BEHAVIOR_RAMP_STOP,
            )
            self._session.put(f"{self.prefix}/arm/command", command.SerializeToString())
        except Exception as error:
            self.get_logger().error(f"command rejected: {error}")

    def _activate_hardware(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        with self._lock:
            age = time.monotonic() - self._snapshot.received_at
            state = self._snapshot.driver_state
        timeout = float(self.get_parameter("state_timeout_sec").value)
        if age > timeout or state is None or not state.feedback_fresh:
            response.success = False
            response.message = "activation rejected: motor feedback is stale"
            return response
        acquired = self._query(
            f"{self.prefix}/rpc/acquire_session",
            pb.AcquireSessionRequest(client_name="ros2_control").SerializeToString(),
            pb.AcquireSessionResponse,
        )
        if acquired is None or not acquired.ok or acquired.session_id == 0:
            response.success = False
            response.message = "exclusive session acquisition failed"
            return response
        mode = self._query(
            f"{self.prefix}/rpc/set_mode",
            pb.SetModeRequest(session_id=acquired.session_id, mode=pb.OPERATING_MODE_ACTIVE).SerializeToString(),
            pb.GenericResponse,
        )
        if mode is None or not mode.ok:
            self._release_session(acquired.session_id)
            response.success = False
            response.message = mode.error if mode is not None and mode.HasField("error") else "ACTIVE rejected"
            return response
        with self._lock:
            self._session_id = acquired.session_id
            self._hardware_active = True
        response.success = True
        response.message = "exclusive session acquired; controller ACTIVE"
        return response

    def _deactivate_hardware(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self._safe_release()
        response.success = True
        response.message = "controller disabled and session released"
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
        result = self._query(
            f"{self.prefix}/rpc/set_mode",
            pb.SetModeRequest(session_id=session_id, mode=request.mode).SerializeToString(),
            pb.GenericResponse,
        )
        response.success = bool(result and result.ok)
        response.message = "mode changed" if response.success else "mode rejected"
        return response

    def _clear_fault(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        with self._lock:
            session_id = self._session_id
        if session_id == 0:
            response.success = False
            response.message = "clear_fault requires the ros2_control-owned session"
            return response
        result = self._query(
            f"{self.prefix}/rpc/clear_fault",
            pb.ClearFaultRequest(session_id=session_id).SerializeToString(),
            pb.GenericResponse,
        )
        response.success = bool(result and result.ok)
        response.message = "fault cleared" if response.success else "fault remains present"
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
        with self._lock:
            joint = self._snapshot.joint_state
            driver = self._snapshot.driver_state
            received_at = self._snapshot.received_at
        if joint is not None and self._state_pub is not None:
            state = RosJointState()
            state.header.stamp = self.get_clock().now().to_msg()
            state.name = self._joint_names
            state.position = list(joint.q)
            state.velocity = list(joint.dq) if len(joint.dq) == 6 else [0.0] * 6
            state.effort = list(joint.tau_est) if len(joint.tau_est) == 6 else [0.0] * 6
            self._state_pub.publish(state)
        if driver is not None and self._driver_pub is not None:
            self._driver_pub.publish(self._ros_driver(driver))
        if self._diag_pub is not None:
            age = time.monotonic() - received_at if received_at else float("inf")
            diag = DiagnosticArray()
            diag.header.stamp = self.get_clock().now().to_msg()
            status = DiagnosticStatus()
            status.name = "firefly_y6/driver"
            status.hardware_id = self.prefix
            status.level = DiagnosticStatus.ERROR if driver and driver.fault_latched else (
                DiagnosticStatus.WARN if age > float(self.get_parameter("state_timeout_sec").value) else DiagnosticStatus.OK)
            status.message = driver.fault_reason if driver and driver.fault_latched else "feedback current"
            status.values = [KeyValue(key="feedback_age_s", value=f"{age:.6f}")]
            diag.status = [status]
            self._diag_pub.publish(diag)

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

    def _release_session(self, session_id: int) -> None:
        self._query(
            f"{self.prefix}/rpc/set_mode",
            pb.SetModeRequest(session_id=session_id, mode=pb.OPERATING_MODE_DISABLED).SerializeToString(),
            pb.GenericResponse,
        )
        self._query(
            f"{self.prefix}/rpc/release_session",
            pb.ReleaseSessionRequest(session_id=session_id).SerializeToString(),
            pb.GenericResponse,
        )

    def _safe_release(self) -> None:
        with self._lock:
            session_id = self._session_id
            self._session_id = 0
            self._hardware_active = False
        if session_id and self._session is not None:
            try:
                self._release_session(session_id)
            except Exception as error:
                self.get_logger().error(f"best-effort disable failed: {error}")

    def _close_zenoh(self) -> None:
        for subscriber in self._subscribers:
            try:
                subscriber.undeclare()
            except Exception:
                pass
        self._subscribers.clear()
        if self._session is not None:
            try:
                self._session.close()
            except Exception:
                pass
        self._session = None

