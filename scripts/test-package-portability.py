#!/usr/bin/env python3
"""Build copied ROS packages without repository siblings; never connect hardware.

Run in the build environment, for example:
  python3 scripts/test-package-portability.py --underlay install
  python3 scripts/test-package-portability.py --underlay install --check-controller

The underlay supplies installed dependencies, including controller configuration,
messages and the upstream robot description. The copied source workspace contains
only protocol, hardware, description and bringup; there is no root scripts folder.
"""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time


PACKAGES = ("hex_arm_protocol", "hex_arm_hardware", "hex_arm_description", "hex_arm_bringup")
SMOKE = r'''
import json
from pathlib import Path
import runpy
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import xacro
from hex_arm_bringup import integration
from hex_arm_bringup.startup_recipe import load_recipe, ready_position

install = Path(sys.argv[1]).resolve()
assert Path(integration.__file__).resolve().is_relative_to(install), integration.__file__
for package in ("hex_arm_protocol", "hex_arm_hardware", "hex_arm_description", "hex_arm_bringup"):
    assert Path(get_package_prefix(package)).resolve() == install, package
instance = integration.InstanceNames(namespace="portable", joint_prefix="portable_", hardware_name="PortableArm")
assert instance.ros_name("hex_arm/internal/state") == "/portable/hex_arm/internal/state"
assert ready_position(load_recipe()) == [0., -1.350, 1.430, -.300, 0., 0.]
description = Path(get_package_share_directory("hex_arm_description")) / "urdf/firefly_y6.urdf.xacro"
model = ET.fromstring(xacro.process_file(str(description), mappings={
    "backend": "real", "prefix": "portable_", "ros_namespace": "/portable",
    "hardware_name": "PortableArm",
}).toxml())
hardware = model.find("ros2_control")
assert hardware is not None and hardware.attrib["name"] == "PortableArm"
assert [joint.attrib["name"] for joint in hardware.findall("joint")] == instance.joint_names
assert [joint.find("param[@name='device_joint_name']").text for joint in hardware.findall("joint")] == [
    f"joint_{index}" for index in range(1, 7)]
for filename in ("commission-startup-ros.py", "commission-meow-ros.py", "commission-shutdown-ros.py",
                 "graceful-real-launch.py", "runtime-manifest.py"):
    path = install / "lib/hex_arm_bringup" / filename
    assert path.is_file() and path.resolve().is_relative_to(install), path
    module = runpy.run_path(str(path), run_name="__portability_import__")
    assert Path(module["__file__"]).resolve().is_relative_to(install), filename
print(json.dumps({"installed_package_smoke": "passed", "joint_names": instance.joint_names,
                  "startup_order": [step["joint_index"] + 1 for step in load_recipe()["steps"]]}))
'''


def environment():
    result = dict(os.environ)
    # Every subprocess reconstructs ROS paths from installed setup files. A
    # source PYTHONPATH inherited from another test must not rescue copied code.
    for name in ("PYTHONPATH", "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "COLCON_PREFIX_PATH",
                 "LD_LIBRARY_PATH", "HEX_ARM_PROTO_FILE", "CARGO_TARGET_DIR"):
        result.pop(name, None)
    result["PYTHONDONTWRITEBYTECODE"] = "1"
    return result


def run(command, *, cwd, underlay, overlay=None, extra_env=None):
    env = environment()
    env.update(extra_env or {})
    # Fixed shell program and positional arguments avoid interpolating paths.
    script = 'source "$1"; source "$2"; if [ -n "$3" ]; then source "$3"; fi; shift 3; exec "$@"'
    args = ["bash", "-e", "-c", script, "package-portability",
            "/opt/ros/jazzy/setup.bash", str(underlay / "setup.bash"),
            str(overlay / "local_setup.bash") if overlay else "", *map(str, command)]
    subprocess.run(args, cwd=cwd, env=env, check=True)


def copy_package(source, destination):
    shutil.copytree(source, destination, symlinks=True,
                    ignore=shutil.ignore_patterns("__pycache__", "*.pyc", ".pytest_cache", "target", "build", "install", "log", ".git"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--underlay", type=Path, required=True,
                        help="Existing installed ROS dependencies, containing setup.bash")
    parser.add_argument("--source-root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--check-controller", action="store_true",
                        help="Also build a separately copied controller using only the installed schema")
    parser.add_argument("--cargo-target-dir", type=Path,
                        help="Optional existing Cargo cache; defaults to a fresh temporary target")
    parser.add_argument("--output", type=Path, help="Write the final JSON test summary")
    args = parser.parse_args()
    source, underlay = args.source_root.resolve(), args.underlay.resolve()
    if not (underlay / "setup.bash").is_file():
        parser.error("--underlay must contain an installed setup.bash")
    if args.cargo_target_dir and not args.check_controller:
        parser.error("--cargo-target-dir requires --check-controller")
    started = time.monotonic()
    with tempfile.TemporaryDirectory(prefix="hex-arm-portability-") as directory:
        workspace = Path(directory)
        for name in PACKAGES:
            copy_package(source / "src" / name, workspace / "src" / name)
        assert set(path.name for path in (workspace / "src").iterdir()) == set(PACKAGES)
        assert not (workspace / "scripts").exists()
        assert not (workspace / "src/hex_arm_controller").exists()
        install = workspace / "install"
        run(["colcon", "--log-base", workspace / "log", "build", "--base-paths", workspace / "src",
             "--build-base", workspace / "build", "--install-base", install, "--merge-install",
             "--packages-select", *PACKAGES,
             "--event-handlers", "console_direct+", "--cmake-args", "-DBUILD_TESTING=OFF"],
            cwd=workspace, underlay=underlay)
        run([sys.executable, "-c", SMOKE, install], cwd=workspace,
            underlay=underlay, overlay=install)
        if args.check_controller:
            controller = workspace / "controller-only/hex_arm_controller"
            copy_package(source / "src/hex_arm_controller", controller)
            schema_link = controller / "proto/robot_api.proto"
            schema_link.unlink()
            schema_link.symlink_to("../../missing-source-protocol/robot_api.proto")
            assert schema_link.is_symlink() and not schema_link.exists()
            schema = install / "share/hex_arm_protocol/proto/robot_api.proto"
            assert schema.is_file()
            target = args.cargo_target_dir.resolve() if args.cargo_target_dir else workspace / "cargo-target"
            run(["cargo", "build", "--release", "--locked", "--offline", "--no-default-features",
                 "--bin", "hex_arm_controller", "--manifest-path", controller / "Cargo.toml"],
                cwd=controller, underlay=underlay, overlay=install,
                extra_env={"CARGO_TARGET_DIR": str(target), "HEX_ARM_PROTO_FILE": str(schema)})
        summary = {"passed": True, "copied_packages": list(PACKAGES),
                   "repository_scripts_present": False, "controller_source_in_workspace": False,
                   "installed_model_and_script_imports": "passed",
                   "standalone_controller": "passed" if args.check_controller else "not requested",
                   "seconds": round(time.monotonic() - started, 2)}
        if args.output:
            args.output.write_text(json.dumps(summary, indent=2) + "\n")
        print(json.dumps(summary, indent=2), flush=True)


if __name__ == "__main__":
    main()
