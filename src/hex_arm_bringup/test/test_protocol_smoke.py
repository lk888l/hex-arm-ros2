#!/usr/bin/env python3
"""Exercise the installed C++ plugin and Rust mock; no CAN or ROS command relay."""
from pathlib import Path
import importlib.util
import tempfile
from types import SimpleNamespace

from diagnostic_msgs.msg import DiagnosticArray
from hex_arm_msgs.msg import DriverState
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

PREFIX = "hexmeow/wsl/firefly_y6_test"


class TransportProbe(Node):
    """Read-only observer; the C++ write() loop is the sole command source."""

    def __init__(self):
        super().__init__("hex_arm_protocol_probe")
        self.received = {"state": False, "driver": False, "diagnostics": False}
        self.driver = None
        self.create_subscription(JointState, "/hex_arm/internal/state", self.state,
                                 qos_profile_sensor_data)
        self.create_subscription(DriverState, "/hex_arm/driver_state", self.driver_state, 10)
        self.create_subscription(DiagnosticArray, "/diagnostics",
                                 lambda _: self.received.__setitem__("diagnostics", True), 10)

    def state(self, message):
        self.received["state"] = len(message.position) == 6

    def driver_state(self, message):
        self.received["driver"] = True
        self.driver = message


def main():
    workspace = Path(__file__).resolve().parents[3]
    spec = importlib.util.spec_from_file_location("benchmark", workspace / "scripts/benchmark-transport.py")
    benchmark = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(benchmark)
    with tempfile.TemporaryDirectory(prefix="hex-arm-direct-protocol-") as directory:
        for scenario in ("baseline", "trajectory", "diagnostic", "management", "cpu",
                         "stopped-command", "delay-recovery", "disconnect", "restart",
                         "reactivate", "reactivate-stream"):
            args = SimpleNamespace(workspace=workspace, domain_id=73, endpoint="tcp/127.0.0.1:7459",
                                   scenario=scenario, warmup=0.5, duration=3.0, cpu_workers=2)
            benchmark.run_plugin_once(args, Path(directory) / scenario)
            print(f"direct protocol: {scenario} passed", flush=True)


if __name__ == "__main__":
    main()
