#!/usr/bin/env python3
"""Prepare a separate SI hardware profile from an already calibrated arm.

This is an offline configuration tool: it never opens CAN or changes calibration.
GUI Rev/s and Rev/s^2 are both multiplied by 2*pi, exactly once.
"""
import argparse
import copy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


JOINT_NAMES = tuple(f"joint_{index}" for index in range(1, 7))


def si_rate(rad_value, rev_value):
    value = rad_value if rad_value is not None else rev_value
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError("motion rates must be finite and positive")
    return float(value) if rad_value is not None else float(value) * math.tau


def prepare(profile, urdf, velocity, acceleration, full_range=False, clear_motion_feedforward=False):
    if (profile.get("schema_version") != 3 or profile.get("joint_coordinate_version") != 2
            or profile.get("validated") is not True or profile.get("calibrated") is not True):
        raise ValueError("requires an already validated and calibrated schema-v3 arm profile")
    joints = profile.get("joints", [])
    if len(joints) != 6 or [joint["name"] for joint in joints] != list(JOINT_NAMES):
        raise ValueError("profile must contain joint_1..joint_6 in canonical order")
    velocity = si_rate(velocity, None)
    acceleration = si_rate(acceleration, None)
    model = ET.fromstring(urdf)
    model_limits = {
        element.get("name"): element.find("limit").attrib
        for element in model.findall("joint")
        if element.get("name") in JOINT_NAMES and element.find("limit") is not None
    }
    if set(model_limits) != set(JOINT_NAMES):
        raise ValueError("URDF must contain six bounded joints")
    result = copy.deepcopy(profile)
    for joint in result["joints"]:
        model_limit = model_limits[joint["name"]]
        lower, upper = float(model_limit["lower"]), float(model_limit["upper"])
        model_speed = si_rate(float(model_limit["velocity"]), None)
        if not all(math.isfinite(v) for v in (lower, upper)) or lower >= upper:
            raise ValueError("invalid URDF position range")
        if full_range:
            joint["limits"].update(position_lower_rad=lower, position_upper_rad=upper)
        # Rust validates feedback margins, single-turn seams and torque budgets.
        joint["limits"].update(velocity_rad_s=min(velocity, model_speed),
                               acceleration_rad_s2=acceleration)
        if clear_motion_feedforward:
            joint.pop("motion_feedforward", None)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--urdf", type=Path, required=True)
    velocity = parser.add_mutually_exclusive_group(required=True)
    velocity.add_argument("--velocity-rad-s", type=float)
    velocity.add_argument("--velocity-rev-s", type=float)
    acceleration = parser.add_mutually_exclusive_group(required=True)
    acceleration.add_argument("--acceleration-rad-s2", type=float)
    acceleration.add_argument("--acceleration-rev-s2", type=float)
    parser.add_argument("--full-urdf-range", action="store_true")
    parser.add_argument("--clear-motion-feedforward", action="store_true",
                        help="Remove earlier manually tuned motion assistance; keep gravity and PD gains")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("output already exists; choose a new profile path")
    profile = prepare(yaml.safe_load(args.source.read_text(encoding="utf-8")),
                      args.urdf.read_text(encoding="utf-8"),
                      si_rate(args.velocity_rad_s, args.velocity_rev_s),
                      si_rate(args.acceleration_rad_s2, args.acceleration_rev_s2),
                      args.full_urdf_range, args.clear_motion_feedforward)
    header = (f"# MoveIt deployment profile derived from {args.source.name}.\n"
              "# Calibration/identity/gains are preserved; dynamics use joint-side SI units.\n")
    with args.output.open("x", encoding="utf-8") as stream:
        stream.write(header + yaml.safe_dump(profile, sort_keys=False, allow_unicode=True))
    print(json.dumps({"output": str(args.output), "limits": {
        joint["name"]: joint["limits"] for joint in profile["joints"]}}, indent=2))


if __name__ == "__main__":
    main()
