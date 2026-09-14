#!/usr/bin/env python3
"""Import hex-gui MIT tuning offline, preserving adapter, identity and measured limits.

Requires PyYAML (python3-yaml in the ROS development image). This script never
connects to a motor. Generated output is always unvalidated and uncalibrated.
"""
import argparse
import copy
import math
from pathlib import Path
import sys

import yaml

# Source: hex-gui/src/sixMotorMitTest.ts:13-25,38-70, inspected 2026-09-05.
# The encoder snapshot is installation-specific, not a universal motor zero.
DIRECTIONS = (-1, -1, 1, 1, 1, 1)
PARK_POSITIONS_REV = (0.000269, 0.249714, 0.251193, -0.00159, -0.004028, 0.000018)
PARK_ANGLES_RAD = (0.0, -1.57, 1.57, 0.0, 0.0, 0.0)
GRAVITY_SCALES = (0.0, 0.3, 0.7, 0.7, 0.0, 0.0)
GRAVITY_LIMITS_NM = (0.2, 5.0, 5.0, 1.0, 0.2, 0.1)


def import_profile(source, park_positions_rev=PARK_POSITIONS_REV):
    """Return a new commissioning profile; never change input measured limits."""
    if (not isinstance(source, dict) or source.get("schema_version") != 3
            or source.get("joint_coordinate_version") != 2):
        raise ValueError(
            "--profile must use schema_version: 3 and joint_coordinate_version: 2"
        )
    for key in ("bus", "controller"):
        if not isinstance(source.get(key), dict):
            raise ValueError(f"input requires a {key} mapping")
    for key in ("robot_prefix", "urdf_path", "gravity_vector_base_m_s2"):
        if key not in source:
            raise ValueError(f"input requires {key}")
    joints = source.get("joints")
    if not isinstance(joints, list) or len(joints) != 6:
        raise ValueError("input must contain exactly joint_1 through joint_6")
    if len(park_positions_rev) != 6 or any(
        not math.isfinite(value) or not -128.0 <= value < 128.0
        for value in park_positions_rev
    ):
        raise ValueError("park positions must be six finite readings in [-128, 128) Rev")
    for index, joint in enumerate(joints, 1):
        if not isinstance(joint, dict) or joint.get("name") != f"joint_{index}":
            raise ValueError("input must use ordered joint_1 through joint_6")
        if joint.get("node_id") not in (0, index):
            raise ValueError("input must use direct mapping node N -> joint_N; node 0 placeholders are accepted")
        if not isinstance(joint.get("identity"), dict) or not isinstance(joint.get("limits"), dict):
            raise ValueError(f"joint_{index} requires identity and limits")
        for key in ("torque_nm", "velocity_rad_s", "acceleration_rad_s2"):
            value = joint["limits"].get(key)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"joint_{index}.limits.{key} must be finite and positive")

    result = copy.deepcopy(source)
    result.update(validated=False, calibrated=False)
    result["bus"]["protocol"] = "meow"
    result["controller"].update(loop_hz=500, gravity_startup_slew_rate_nm_s=5.0)
    for index, joint in enumerate(result["joints"]):
        joint.update(
            node_id=index + 1,
            direction=DIRECTIONS[index],
            zero_offset_rad=PARK_ANGLES_RAD[index] - DIRECTIONS[index] * math.tau * park_positions_rev[index],
            torque_scale=1.0,
            gravity_compensation_scale=GRAVITY_SCALES[index],
            gravity_compensation_limit_nm=GRAVITY_LIMITS_NM[index],
            torque_permille=650,
            kp_kd_torque_permille=500,
            default_kp=80.0,
            default_kd=15.0,
        )
    return result


def write_profile(output, profile, source, parks):
    """Exclusive creation protects both calibrated input and existing output."""
    output = Path(output)
    if not output.name.endswith(".local.yaml"):
        raise ValueError("--output must end in .local.yaml")
    if output.resolve() == Path(source).resolve():
        raise ValueError("--output must be different from --profile")
    header = (
        "# Offline hex-gui MIT parameter import (sixMotorMitTest.ts, 2026-09-05).\n"
        "# This output is NOT validated or calibrated after conversion.\n"
        "# Hardware identities, adapter, gravity vector, payload and measured limits are retained.\n"
        "# References below are valid only at the physical folded park pose:\n"
        f"# URDF q [rad]: {list(PARK_ANGLES_RAD)}\n"
        f"# Encoder [Rev]: {list(parks)}\n"
        "# Physical torque calibration is read from 0x4001 by the Meow backend.\n"
    )
    with output.open("x", encoding="utf-8", newline="\n") as stream:
        stream.write(header)
        yaml.safe_dump(profile, stream, sort_keys=False, allow_unicode=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", required=True, type=Path, help="existing hardware YAML or commissioning template")
    parser.add_argument("--output", required=True, type=Path, help="new *.local.yaml; existing files are never replaced")
    parser.add_argument(
        "--park-positions-rev", nargs=6, type=float, default=PARK_POSITIONS_REV,
        metavar=("M1", "M2", "M3", "M4", "M5", "M6"),
        help="fresh encoders taken only at physical q=[0,-1.57,1.57,0,0,0] rad; otherwise uses the GUI snapshot",
    )
    args = parser.parse_args(argv)
    try:
        with args.profile.open(encoding="utf-8") as stream:
            profile = import_profile(yaml.safe_load(stream), args.park_positions_rev)
        write_profile(args.output, profile, args.profile, args.park_positions_rev)
    except (OSError, ValueError, yaml.YAMLError) as error:
        parser.exit(2, f"error: {error}\n")
    print(f"Created {args.output}; validated=false, calibrated=false. No motor I/O performed.")
    print("References require the physical folded pose q=[0,-1.57,1.57,0,0,0] rad.")
    print("Joint      encoder Rev    zero offset rad    gravity scale    clamp Nm")
    for index, joint in enumerate(profile["joints"]):
        print(f"{joint['name']:8} {args.park_positions_rev[index]:12.6f} {joint['zero_offset_rad']:18.9f} {joint['gravity_compensation_scale']:16.3f} {joint['gravity_compensation_limit_nm']:11.3f}")
        if any(joint["identity"].get(key, 0) == 0 for key in ("vendor_id", "product_code", "serial_number")):
            print("  Review: identity still contains placeholders; fill from read-only discovery.")
        if joint["limits"]["torque_nm"] < joint["gravity_compensation_limit_nm"]:
            print(f"  Review: preserved limits.torque_nm={joint['limits']['torque_nm']} is below the imported gravity clamp.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
