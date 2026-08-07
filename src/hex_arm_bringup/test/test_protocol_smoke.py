#!/usr/bin/env python3
from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from diagnostic_msgs.msg import DiagnosticArray
from hex_arm_msgs.msg import DriverState
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
import zenoh

from hex_arm_bridge.pb import robot_api_pb2 as pb


WORKSPACE = Path("/workspaces/hex_arm_ros2")
PROFILE = WORKSPACE / "src/hex_arm_controller/test/firefly_y6.mock.yaml"
CONTROLLER = WORKSPACE / "install/hex_arm_controller/lib/hex_arm_controller/hex_arm_controller"
BRIDGE = WORKSPACE / "install/hex_arm_bridge/lib/hex_arm_bridge/hex_arm_bridge"
PREFIX = "hexmeow/wsl/firefly_y6_test"


def _query(session, key: str, message_type, timeout: float = 0.5):
    for reply in session.get(key, timeout=timeout):
        if reply.ok is not None:
            return message_type.FromString(bytes(reply.ok.payload.to_bytes()))
    return None


def _stop(process: subprocess.Popen[str] | None) -> str:
    if process is None:
        return ""
    for stop_signal, timeout in (
        (signal.SIGINT, 3.0),
        (signal.SIGTERM, 2.0),
        (signal.SIGKILL, 2.0),
    ):
        if process.poll() is not None:
            break
        os.killpg(process.pid, stop_signal)
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            continue
    return process.stdout.read() if process.stdout else ""


def _spin_future(node: Node, future, timeout: float):
    deadline = time.monotonic() + timeout
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not future.done():
        raise TimeoutError("ROS lifecycle operation timed out")
    return future.result()


class BridgeProbe(Node):
    def __init__(self) -> None:
        super().__init__("hex_arm_protocol_probe")
        self.change_state = self.create_client(ChangeState, "/hex_arm_bridge/change_state")
        self.received = {"state": False, "driver": False, "diagnostics": False}
        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            JointState,
            "/hex_arm/internal/state",
            lambda _: self.received.__setitem__("state", True),
            best_effort,
        )
        self.create_subscription(
            DriverState,
            "/hex_arm/driver_state",
            lambda _: self.received.__setitem__("driver", True),
            10,
        )
        self.create_subscription(
            DiagnosticArray,
            "/diagnostics",
            lambda _: self.received.__setitem__("diagnostics", True),
            10,
        )

    def transition(self, transition_id: int) -> None:
        request = ChangeState.Request()
        request.transition.id = transition_id
        response = _spin_future(self, self.change_state.call_async(request), 10.0)
        if response is None or not response.success:
            raise RuntimeError(f"bridge lifecycle transition {transition_id} failed")


def main() -> None:
    controller = None
    bridge = None
    session = None
    probe = None
    failed = False
    controller_log = ""
    bridge_log = ""
    try:
        controller = subprocess.Popen(
            [str(CONTROLLER), "--profile", str(PROFILE), "--mock"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        session = zenoh.open(zenoh.Config())
        deadline = time.monotonic() + 15.0
        description = None
        while description is None and time.monotonic() < deadline:
            if controller.poll() is not None:
                raise RuntimeError("mock controller exited before Zenoh discovery")
            description = _query(session, f"{PREFIX}/description", pb.RobotDescription)
        if description is None or description.api_version.major != 0:
            raise RuntimeError("robot_api description/API version was not available")

        event_log = _query(session, f"{PREFIX}/events/recent", pb.EventLog)
        if event_log is None or "driver_initialized" not in {event.code for event in event_log.events}:
            raise RuntimeError("robot_api event history did not contain driver_initialized")

        rclpy.init()
        probe = BridgeProbe()
        bridge = subprocess.Popen(
            [
                str(BRIDGE), "--ros-args", "-p", f"robot_prefix:={PREFIX}",
                "-p", "required_api_major:=0",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        if not probe.change_state.wait_for_service(timeout_sec=15.0):
            raise RuntimeError("lifecycle bridge service was not discovered")
        probe.transition(Transition.TRANSITION_CONFIGURE)
        probe.transition(Transition.TRANSITION_ACTIVATE)

        deadline = time.monotonic() + 15.0
        while not all(probe.received.values()) and time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.05)
        if not all(probe.received.values()):
            raise RuntimeError(f"bridge publications missing: {probe.received}")
    except BaseException:
        failed = True
        raise
    finally:
        bridge_log = _stop(bridge)
        controller_log = _stop(controller)
        if probe is not None:
            probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if session is not None:
            session.close()
        if failed:
            print(f"--- bridge ---\n{bridge_log}\n--- controller ---\n{controller_log}", file=sys.stderr)


if __name__ == "__main__":
    main()
