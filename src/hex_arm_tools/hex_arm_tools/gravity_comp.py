"""Exclusive hand-guiding client. The Rust driver owns dynamics and the lease."""
from __future__ import annotations

import json
import math
import signal
import threading
import time

from hex_arm_tools.pb import robot_api_pb2 as pb


def validate_damping(values):
    gains = [float(value) for value in values]
    if len(gains) != 6 or not all(math.isfinite(value) and value > 0 for value in gains):
        raise ValueError("damping must contain six finite positive values in N*m*s/rad")
    return gains


class GravityCompClient:
    """Synchronous RPCs; no background heartbeat that could outlive its owner."""

    def __init__(self, session, prefix, damping):
        self.session = session
        self.prefix = prefix.rstrip("/")
        self.damping = validate_damping(damping)
        self.session_id = 0
        self.sequence = 0

    def query(self, suffix, request, response_type=pb.GenericResponse, timeout=0.2):
        payload = request.SerializeToString() if request is not None else b""
        for reply in self.session.get(f"{self.prefix}/{suffix}", payload=payload, timeout=timeout):
            if reply.ok is not None:
                return response_type.FromString(bytes(reply.ok.payload.to_bytes()))
        raise TimeoutError(f"{suffix}: no acknowledgement")

    def checked(self, suffix, request, timeout=0.2):
        result = self.query(suffix, request, timeout=timeout)
        if not result.ok:
            raise RuntimeError(f"{suffix}: {result.error}")
        return result

    def start(self, stop_requested, timeout=30.0):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("startup timeout must be finite and positive")
        deadline = time.monotonic() + timeout
        while True:
            if stop_requested.is_set():
                raise RuntimeError("hand-guiding startup cancelled")
            try:
                description = self.query("description", None, pb.RobotDescription)
                break
            except TimeoutError:
                if time.monotonic() >= deadline:
                    raise TimeoutError("driver discovery timed out") from None
                stop_requested.wait(0.05)
        if (description.api_version.major != 0
                or description.api_version.minor < 4
                or pb.OPERATING_MODE_GRAVITY_COMP not in description.supported_modes):
            raise RuntimeError("driver does not support hand guiding with a session lease (API 0.4+)")
        acquired = self.query("rpc/acquire_session", pb.AcquireSessionRequest(
            client_name="ros2_hand_guiding"), pb.AcquireSessionResponse, timeout=1.0)
        if not acquired.ok or not acquired.session_id:
            raise RuntimeError(f"exclusive session unavailable: {acquired.error or acquired.current_holder_name}")
        self.session_id = acquired.session_id
        if stop_requested.is_set():
            raise RuntimeError("hand-guiding startup cancelled")
        # Never retry an enable request whose outcome is unknown. The caller
        # releases this session on failure; the driver's lease also expires.
        self.checked("rpc/start_gravity_comp", pb.StartGravityCompRequest(
            session_id=self.session_id, damping_nm_s_rad=self.damping), timeout=10.0)
        self.heartbeat()

    def heartbeat(self):
        self.sequence += 1
        self.checked("rpc/gravity_comp_heartbeat", pb.GravityCompHeartbeatRequest(
            session_id=self.session_id, sequence=self.sequence))

    def stop(self):
        if not self.session_id:
            return
        # release_session disables all axes and acknowledges before relinquishing
        # ownership. A missing acknowledgement must never be called success.
        self.checked("rpc/release_session", pb.ReleaseSessionRequest(
            session_id=self.session_id), timeout=3.0)
        self.session_id = 0


def main(args=None):
    import rclpy
    from rcl_interfaces.msg import ParameterDescriptor
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from std_srvs.srv import Trigger
    import zenoh

    stop_requested = threading.Event()
    previous_signals = {}
    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_signals[signum] = signal.signal(signum, lambda *_: stop_requested.set())
    node = None
    session = None
    client = None
    timer = None
    failure = None
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = Node("hex_arm_gravity_comp")
        readonly = ParameterDescriptor(read_only=True)
        for name, default in (
            ("robot_prefix", ""),
            ("zenoh_connect", "tcp/127.0.0.1:7448"),
            ("damping", [2.0] * 6),
            ("startup_timeout_sec", 30.0),
        ):
            node.declare_parameter(name, default, readonly)
        prefix = node.get_parameter("robot_prefix").value
        if not prefix:
            raise ValueError("robot_prefix is required")
        gains = validate_damping(node.get_parameter("damping").value)
        config = zenoh.Config()
        config.insert_json5("mode", '"peer"')
        config.insert_json5("connect/endpoints", json.dumps([
            node.get_parameter("zenoh_connect").value]))
        config.insert_json5("scouting/multicast/enabled", "false")
        session = zenoh.open(config)
        client = GravityCompClient(session, prefix, gains)
        node.get_logger().info("Support the arm: entering gravity compensation with zero position stiffness")
        client.start(stop_requested, node.get_parameter("startup_timeout_sec").value)

        def heartbeat():
            nonlocal failure
            if stop_requested.is_set():
                return
            try:
                client.heartbeat()
            except Exception as error:
                failure = error
                stop_requested.set()

        def stop(_request, response):
            nonlocal failure
            stop_requested.set()
            try:
                client.stop()
                response.success = True
                response.message = "all axes disabled and exclusive session released"
            except Exception as error:
                failure = error
                response.success = False
                response.message = f"disable unconfirmed: {error}"
            return response

        node.create_service(Trigger, "~/stop", stop)
        timer = node.create_timer(0.05, heartbeat)
        node.get_logger().info(
            f"Hand guiding active: Kp=0, damping={gains}; support the arm before stopping. "
            "The driver reports hand_guiding_ready when its gravity ramp completes.")
        while rclpy.ok() and not stop_requested.is_set():
            rclpy.spin_once(node, timeout_sec=0.05)
    except Exception as error:
        failure = error
    finally:
        if timer is not None:
            timer.cancel()
        if client is not None:
            try:
                client.stop()
            except Exception as error:
                failure = failure or error
                if node is not None:
                    node.get_logger().error(f"Disable unconfirmed by client: {error}")
        if session is not None:
            session.close()
        if node is not None:
            if failure:
                node.get_logger().error(str(failure))
            else:
                node.get_logger().info("Hand-guiding session stopped")
            node.destroy_node()
        rclpy.try_shutdown()
        for signum, previous in previous_signals.items():
            signal.signal(signum, previous)
    if failure:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
