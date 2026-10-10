#!/usr/bin/env python3
"""Check a clean merged install without running ROS or touching hardware."""
from pathlib import Path
import sys


REQUIRED = (
    "local_setup.bash",
    "lib/hex_arm_controller/hex_arm_controller",
    "lib/hex_arm_tools/hex_arm_gravity_comp",
    "lib/hex_arm_bringup/graceful-real-launch.py",
    "lib/hex_arm_bringup/runtime-manifest.py",
    "lib/hex_arm_bringup/commission-shutdown-ros.py",
    "lib/hex_arm_bringup/commission-startup-ros.py",
    "lib/libhex_arm_hardware.so",
    "lib/libzenohc.so",
    "lib/hex_arm_moveit_runtime/hex_arm_move_group",
    "lib/libhex_arm_moveit_tem_shutdown.so",
    "share/hex_arm_controller/config/startup.yaml",
    "share/hex_arm_controller/proto/robot_api.proto",
    "share/hex_arm_moveit_config/launch/moveit_real.launch.py",
    "share/xpkg_urdf_firefly_y6/urdf/xpkg_urdf_firefly_y6.urdf",
)


def check(prefix):
    prefix = prefix.resolve()
    for relative in REQUIRED:
        if not (prefix / relative).is_file():
            raise RuntimeError(f"runtime artifact missing: {relative}")
    for path in prefix.rglob("*"):
        if path.is_symlink() and not path.resolve().is_relative_to(prefix):
            raise RuntimeError(f"runtime artifact links outside install tree: {path}")
    if (prefix / "lib/hex_arm_controller/hex_arm_commission").exists():
        raise RuntimeError("production install contains opt-in commissioning binary")
    if ((prefix / "share/hex_arm_bridge").exists() or (prefix / "lib/hex_arm_bridge").exists()
            or list(prefix.glob("lib/python*/site-packages/hex_arm_bridge"))):
        raise RuntimeError("production install contains the removed bridge backend")
    if (prefix / "share/hex_arm_tools/proto/robot_api.proto").exists():
        raise RuntimeError("production install contains duplicate protocol authority")
    bindings = list(prefix.glob("lib/python*/site-packages/hex_arm_tools/pb/robot_api_pb2.py"))
    if len(bindings) != 1:
        raise RuntimeError("Python protobuf binding missing or ambiguous")
    print("runtime install: programs, models, configuration and bindings are self-contained")


if __name__ == "__main__":
    check(Path(sys.argv[1]))
