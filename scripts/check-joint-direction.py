#!/usr/bin/env python3
"""Compare two disabled-arm captures with an independently known URDF motion.

This tool opens no CAN socket and changes no profile. The operator, not the
encoder or the commanded trajectory, must establish the physical direction.
"""
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path

import yaml


spec = importlib.util.spec_from_file_location(
    "direction_campaign", Path(__file__).with_name("commission-cia402.py"))
campaign = importlib.util.module_from_spec(spec)
spec.loader.exec_module(campaign)


def compare(profile, before, after, axis, expected_sign):
    if (profile.get("schema_version") != 3 or profile.get("joint_coordinate_version") != 2
            or profile.get("bus", {}).get("protocol") != "cia402"
            or len(profile.get("joints", [])) != 6
            or len({j.get("node_id") for j in profile["joints"]}) != 6
            or axis not in range(6) or expected_sign not in (-1, 1)):
        raise ValueError("requires six-axis CiA402 coordinates v2 and a known +/-1 URDF motion")
    for capture in (before, after):
        campaign.verify_disabled(capture)
        if capture.get("interface") != profile["bus"]["interface"]:
            raise ValueError("capture interface differs from profile")
    deltas = []
    for i, joint in enumerate(profile["joints"]):
        if joint.get("name") != f"joint_{i + 1}" or joint.get("direction") not in (-1, 1):
            raise ValueError("invalid ordered joint mapping or direction")
        samples = []
        for capture in (before, after):
            nodes = [n for n in capture["nodes"] if n["node"] == joint["node_id"]]
            if len(nodes) != 1:
                raise ValueError("missing or duplicate mapped node")
            node = nodes[0]
            identity = {k: joint["identity"][k] for k in
                        ("vendor_id", "product_code", "revision", "serial_number")}
            if node.get("identity") != identity:
                raise ValueError(f"{joint['name']}: identity mismatch")
            raw = node.get("position_rev")
            if isinstance(raw, bool) or not isinstance(raw, (int, float)) or not math.isfinite(raw):
                raise ValueError("capture requires finite raw encoder positions")
            samples.append(raw)
        delta = (samples[1] - samples[0]) * math.tau
        # Do not infer winding or direction across an encoder seam.
        if abs(delta) > 0.35:
            raise ValueError("capture moved too far or crossed an encoder seam")
        if i != axis and abs(delta) > 0.01:
            raise ValueError(f"joint_{i + 1}: another axis moved; isolate the intended joint")
        deltas.append(delta)
    if abs(deltas[axis]) < 0.02:
        raise ValueError("selected encoder movement is too small to establish direction")
    inferred = expected_sign * (1 if deltas[axis] > 0 else -1)
    configured = profile["joints"][axis]["direction"]
    return {
        "axis": f"joint_{axis + 1}", "operator_expected_urdf_sign": expected_sign,
        "encoder_delta_rad": deltas[axis], "all_encoder_deltas_rad": deltas,
        "configured_direction": configured, "inferred_direction": inferred,
        "passed": inferred == configured,
        "limitation": "Conditional on independently observed physical motion; does not validate zero, travel, or deployment.",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", required=True, type=Path)
    parser.add_argument("--before", required=True, type=Path)
    parser.add_argument("--after", required=True, type=Path)
    parser.add_argument("--axis", required=True, choices=[f"joint_{i}" for i in range(1, 7)])
    parser.add_argument("--expected-sign", required=True, type=int, choices=(-1, 1),
                        help="Sign of the independently observed motion in URDF coordinates")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    paths = {key: getattr(args, key) for key in ("profile", "before", "after")}
    if args.output.resolve() in [p.resolve() for p in paths.values()] or args.output.exists():
        parser.error("output must be new and must not replace an input")
    report = {"passed": False, "inputs": {k: {"path": str(p), "sha256":
              hashlib.sha256(p.read_bytes()).hexdigest()} for k, p in paths.items()}}
    try:
        report.update(compare(yaml.safe_load(args.profile.read_text()),
                              json.loads(args.before.read_text()), json.loads(args.after.read_text()),
                              int(args.axis[-1]) - 1, args.expected_sign))
    except (KeyError, TypeError, ValueError, RuntimeError) as error:
        report["error"] = str(error)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps(report, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
