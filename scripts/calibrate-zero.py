#!/usr/bin/env python3
"""Read-only folded-pose zero calibration for legacy CiA402 Firefly Y6 arms.

Requires Python 3, PyYAML and SocketCAN. Stop every other CAN controller first.
The operator establishes the physical reference; encoders cannot establish it.
Only SDO uploads are sent. Output profiles remain unvalidated/uncalibrated.
"""
import argparse
import copy
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import socket
import statistics
import struct
import sys
import time

import yaml

ROOT = Path(__file__).resolve().parents[1]
REFERENCE_PATH = ROOT / "src/hex_arm_controller/config/startup.yaml"
JOINT_NAMES = tuple(f"joint_{i}" for i in range(1, 7))


def script_module(name, filename):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name(filename))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


reader = script_module("zero_cia402_reader", "read-cia402-state.py")
binder = script_module("zero_can_binding", "bind-can-profile.py")


def finite_number(value):
    return (not isinstance(value, bool) and isinstance(value, (float, int))
            and math.isfinite(value))


def reference_pose(path=REFERENCE_PATH):
    recipe = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(recipe, dict) or recipe.get("schema_version") != 1:
        raise ValueError("unsupported startup reference schema")
    pose = recipe.get("folded_position_rad")
    if not isinstance(pose, list) or len(pose) != 6 or not all(map(finite_number, pose)):
        raise ValueError("startup.yaml must contain six finite folded_position_rad values")
    return pose


def prepare_profile(source, directions=None):
    """Validate calibration inputs without granting motion authority."""
    profile = copy.deepcopy(source)
    if (not isinstance(profile, dict) or profile.get("schema_version") != 3
            or profile.get("joint_coordinate_version") != 2):
        raise ValueError("requires schema_version: 3 / joint_coordinate_version: 2; migrate v2 first")
    bus = profile.get("bus")
    if (not isinstance(bus, dict) or bus.get("protocol", "cia402") != "cia402"
            or bus.get("transport") != "socket_can"):
        raise ValueError("requires a legacy CiA402 socket_can profile")
    if not isinstance(bus.get("expected_link"), dict):
        raise ValueError("profile requires bus.expected_link")
    for key in ("controller", "robot_prefix", "urdf_path", "gravity_vector_base_m_s2"):
        if key not in profile:
            raise ValueError(f"profile requires {key}")
    joints = profile.get("joints")
    if not isinstance(joints, list) or len(joints) != 6:
        raise ValueError("profile requires exactly six joints")
    if directions is not None and len(directions) != 6:
        raise ValueError("--directions requires six signs")
    nodes = set()
    for i, joint in enumerate(joints):
        if not isinstance(joint, dict) or joint.get("name") != JOINT_NAMES[i]:
            raise ValueError("joints must be ordered joint_1 through joint_6")
        node = joint.get("node_id")
        if node == 0 and bus.get("direct_joint_mapping") is True:
            node = i + 1  # Explicit direct-mapping commissioning template.
        if (type(node) is not int or not 1 <= node <= 127 or node in nodes
                or node == bus.get("heartbeat_node_id")
                or node in bus.get("auxiliary_node_ids", [])
                or (bus.get("direct_joint_mapping") is True and node != i + 1)):
            raise ValueError(f"{joint['name']}: invalid or conflicting node mapping")
        nodes.add(node)
        joint["node_id"] = node
        if directions is not None:
            joint["direction"] = directions[i]
        if type(joint.get("direction")) is not int or joint["direction"] not in (-1, 1):
            raise ValueError(f"{joint['name']}: direction unknown; supply --directions with six verified +/-1 signs")
        if not finite_number(joint.get("zero_offset_rad")):
            raise ValueError(f"{joint['name']}: invalid zero_offset_rad")
        identity = joint.get("identity")
        if not isinstance(identity, dict) or any(
            type(identity.get(k)) is not int or not 0 <= identity[k] <= 0xffffffff
            for k in reader.IDENTITY_KEYS
        ):
            raise ValueError(f"{joint['name']}: invalid identity fingerprint")
        limits = joint.get("limits")
        if (not isinstance(limits, dict) or any(not finite_number(limits.get(k)) for k in
                ("position_lower_rad", "position_upper_rad"))
                or limits["position_lower_rad"] >= limits["position_upper_rad"]):
            raise ValueError(f"{joint['name']}: invalid position limits")
    return profile


def capture(bus, joints, samples, interval):
    """Sample all axes in rounds, checking drive status throughout the capture."""
    records = []
    for joint in joints:
        node = joint["node_id"]
        record = dict(node=node, identity={}, samples_rev=[], status_words=[], error_codes=[], errors=[])
        records.append(record)
        try:
            record["identity"] = {
                key: reader.read_value(bus, node, 0x1018, sub, "<I")
                for sub, key in enumerate(reader.IDENTITY_KEYS, 1)
            }
        except (OSError, RuntimeError, ValueError) as error:
            record["errors"].append(str(error))
    for sample in range(samples + 1):
        for record in records:
            if record["errors"]:
                continue
            node = record["node"]
            try:
                record["status_words"].append(reader.read_value(bus, node, 0x6041, 0, "<H"))
                record["error_codes"].append(reader.read_value(bus, node, 0x603f, 0, "<H"))
                if sample < samples:
                    record["samples_rev"].append(reader.read_value(bus, node, 0x6064, 0, "<f"))
                else:
                    final_identity = {
                        key: reader.read_value(bus, node, 0x1018, sub, "<I")
                        for sub, key in enumerate(reader.IDENTITY_KEYS, 1)
                    }
                    if final_identity != record["identity"]:
                        record["errors"].append("identity changed during capture")
            except (OSError, RuntimeError, ValueError) as error:
                record["errors"].append(str(error))
        if sample < samples - 1:
            time.sleep(interval)
    return records


def analyse(profile, records, reference, sample_count, max_span, replace_arm=False):
    rows, errors, warnings = [], [], []
    for joint, ref in zip(profile["joints"], reference):
        name, node = joint["name"], joint["node_id"]
        found = [r for r in records if r["node"] == node]
        if len(found) != 1:
            errors.append(f"{name}: missing or duplicate node record")
            continue
        record = found[0]
        errors.extend(f"{name}: {error}" for error in record["errors"])
        if (len(record["status_words"]) != sample_count + 1
                or any((word & 0x6f) not in (0x40, 0x21, 0x23) for word in record["status_words"])):
            errors.append(f"{name}: disabled state not confirmed throughout sampling")
        if len(record["error_codes"]) != sample_count + 1 or any(record["error_codes"]):
            errors.append(f"{name}: drive error or incomplete fault checks")
        identity = record["identity"]
        if any(type(identity.get(k)) is not int or not 0 < identity[k] <= 0xffffffff
               for k in reader.IDENTITY_KEYS):
            errors.append(f"{name}: missing/incomplete live identity")
        expected = {k: joint["identity"][k] for k in reader.IDENTITY_KEYS}
        identity_matches = identity == expected
        if not identity_matches:
            message = f"{name}: identity differs from input profile"
            if replace_arm:
                warnings.append(message + "; binding candidate to observed identity")
            else:
                errors.append(message + "; use --replace-arm only when replacing/commissioning the arm")
        values = record["samples_rev"]
        if len(values) != sample_count or any(
            not finite_number(value) or not -0.5 <= value < 0.5 for value in values
        ):
            errors.append(f"{name}: incomplete or noncanonical position samples (expected [-0.5,0.5) Rev)")
            continue
        span = (max(values) - min(values)) * math.tau
        if span > max_span:
            errors.append(f"{name}: encoder span {span:.9f} rad exceeds {max_span:.9f}; moving or wrap seam")
        raw = statistics.median(values)
        direction = joint["direction"]
        before = direction * math.tau * raw + joint["zero_offset_rad"]
        offset = ref - direction * math.tau * raw
        after = direction * math.tau * raw + offset
        limits = joint["limits"]
        lower, upper = limits["position_lower_rad"], limits["position_upper_rad"]
        motor_limits = sorted(direction * (q - offset) / math.tau for q in (lower, upper))
        issues = []
        if not lower <= ref <= upper:
            issues.append("folded reference lies outside retained command limits")
        # Same 0.01 Rev guard and exclusive upper edge as profile.rs/single_turn.rs.
        if motor_limits[0] < -0.49 or motor_limits[1] >= 0.49:
            issues.append("retained limits with new offset touch the single-turn command seam")
        warnings.extend(f"{name}: {issue}" for issue in issues)
        rows.append(dict(
            name=name, node_id=node, direction=direction, identity=identity,
            previous_identity=expected, identity_matches=identity_matches,
            raw_position_rev=raw, raw_position_rad=raw * math.tau,
            previous_zero_offset_rad=joint["zero_offset_rad"], position_before_rad=before,
            reference_position_rad=ref, zero_offset_rad=offset,
            offset_change_rad=offset - joint["zero_offset_rad"], position_after_rad=after,
            existing_reference_error_rad=before - ref, span_rad=span,
            retained_position_limits_rad=[lower, upper], candidate_position_limits_rev=motor_limits,
            limit_issues=issues,
        ))
    return rows, errors, warnings


def candidate_profile(profile, rows, interface, binding):
    candidate = binder.bind_profile(profile, interface, binding)
    candidate.update(validated=False, calibrated=False)
    for joint, row in zip(candidate["joints"], rows):
        if joint["name"] != row["name"]:
            raise ValueError("incomplete calibration rows")
        old_identity = joint["identity"]
        joint["identity"] = dict(row["identity"])
        # Do not assign an old model name to a different product.
        if (old_identity["vendor_id"] == row["identity"]["vendor_id"]
                and old_identity["product_code"] == row["identity"]["product_code"]
                and old_identity.get("model") not in (None, "", "UNVERIFIED")):
            joint["identity"]["model"] = old_identity["model"]
        else:
            joint["identity"]["model"] = (
                f"CiA402 vendor=0x{row['identity']['vendor_id']:08x} "
                f"product=0x{row['identity']['product_code']:08x} (model unverified)"
            )
        joint["zero_offset_rad"] = row["zero_offset_rad"]
    if len(rows) != 6:
        raise ValueError("six calibration rows required")
    return candidate


def write_candidate(path, candidate, report_path, report):
    header = (
        "# Folded-pose SOFTWARE zero calibration; no drive objects were written.\n"
        "# Candidate only: direction, travel, tuning, gravity and payload need commissioning.\n"
        f"# Capture UTC: {report['time']}\n"
        f"# Evidence: {report_path}\n"
        f"# Source SHA256: {report['profile_sha256']}\n"
        f"# Reference SHA256: {report['reference_sha256']}\n"
        f"# Encoder medians [Rev]: {[r['raw_position_rev'] for r in report['joints']]}\n"
    )
    contents = header + yaml.safe_dump(candidate, sort_keys=False, allow_unicode=True)
    with path.open("x", encoding="utf-8") as stream:
        stream.write(contents)


def print_report(report):
    print("Reference q [rad]:", report["reference_position_rad"])
    print("Physical folded pose:", "operator confirmed" if report["pose_confirmed"] else "UNCONFIRMED (hypothetical offsets)")
    print("Raw Rev = encoder reading; before/after = software joint coordinates, not an independent pose measurement.")
    if report["mode"] == "check":
        print("Joint    Raw Rev       Current rad   Existing offset  Ref error rad Span rad")
        for row in report["joints"]:
            print(f"{row['name']:8} {row['raw_position_rev']:+.9f} {row['position_before_rad']:+.9f} "
                  f"{row['previous_zero_offset_rad']:+.9f}    {row['existing_reference_error_rad']:+.9f} {row['span_rad']:.9f}")
        print("CHECK compares existing offsets to the physical reference; no offsets are changed.")
    else:
        print("Joint    Raw Rev       Before rad    zero_offset_rad  After rad     Span rad")
        for row in report["joints"]:
            print(f"{row['name']:8} {row['raw_position_rev']:+.9f} {row['position_before_rad']:+.9f} "
                  f"{row['zero_offset_rad']:+.9f}    {row['position_after_rad']:+.9f} {row['span_rad']:.9f}")
    for warning in report["warnings"]:
        print("REVIEW:", warning)
    for error in report["errors"]:
        print("ERROR:", error, file=sys.stderr)
    print("Result:", "PASS" if report["passed"] else "FAIL", f"({report['mode']})")
    if report.get("candidate_written"):
        print(f"Created {report['candidate_written']}; validated=false, calibrated=false.")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", required=True, help="e.g. can2")
    parser.add_argument("--profile", required=True, type=Path, help="schema v3 profile or example template")
    parser.add_argument("--directions", nargs=6, type=int, metavar=("D1", "D2", "D3", "D4", "D5", "D6"),
                        help="explicit verified signs; otherwise use profile signs; cannot infer from one pose")
    parser.add_argument("--confirm-folded-pose", action="store_true", help="operator has physically established the default folded pose")
    parser.add_argument("--replace-arm", action="store_true", help="bind new candidate to current motor identities")
    parser.add_argument("--output", type=Path, help="create a new *.local.yaml (requires confirmed pose)")
    parser.add_argument("--report", type=Path, help="new JSON evidence file; default with --output: <output>.calibration.json")
    parser.add_argument("--check", action="store_true", help="verify existing offsets at the confirmed folded pose")
    parser.add_argument("--tolerance-rad", type=float, default=0.005, help="--check absolute per-axis error tolerance (default 0.005)")
    parser.add_argument("--samples", type=int, default=21, help="samples per axis (default 21)")
    parser.add_argument("--interval", type=float, default=0.05, help="seconds between complete six-axis rounds")
    parser.add_argument("--max-span-rad", type=float, default=0.001, help="max encoder spread per axis (default 0.001)")
    args = parser.parse_args(argv)
    try:
        if not 3 <= args.samples <= 100:
            raise ValueError("--samples must be within 3..100")
        for label, value, maximum in (("interval", args.interval, 1.0),
                                      ("max-span-rad", args.max_span_rad, 0.01),
                                      ("tolerance-rad", args.tolerance_rad, 0.03)):
            if not finite_number(value) or not 0 < value <= maximum:
                raise ValueError(f"--{label} must be finite and within (0,{maximum}]")
        if (args.output or args.check) and not args.confirm_folded_pose:
            raise ValueError("--output / --check requires --confirm-folded-pose; encoders cannot identify the physical reference")
        if args.check and (args.output or args.replace_arm or args.directions):
            raise ValueError("--check uses the existing profile; cannot combine with --output, --replace-arm or --directions")
        if args.output and not args.output.name.endswith(".local.yaml"):
            raise ValueError("--output must end in .local.yaml")
        if args.output and args.report is None:
            args.report = Path(str(args.output) + ".calibration.json")
        targets = [path for path in (args.output, args.report) if path]
        if len({path.resolve() for path in targets}) != len(targets):
            raise ValueError("output and report must be different files")
        for path in targets:
            if path.resolve() in (args.profile.resolve(), REFERENCE_PATH.resolve()) or path.exists() or path.is_symlink():
                raise ValueError(f"refusing to overwrite {path}")
            if not path.parent.is_dir():
                raise ValueError(f"output directory does not exist: {path.parent}")
        source_bytes = args.profile.read_bytes()
        source = yaml.safe_load(source_bytes)
        profile = prepare_profile(source, args.directions)
        reference = reference_pose()
        binding = binder.adapter_binding(args.interface)
        if args.check and (profile["bus"]["interface"] != args.interface
                           or profile["bus"]["expected_link"].get("adapter") != binding):
            raise ValueError("--check interface/adapter does not match profile binding")
    except (OSError, ValueError, yaml.YAMLError) as error:
        parser.exit(2, f"error: {error}\n")

    report = dict(
        schema_version=1, time=datetime.now(timezone.utc).isoformat(),
        mode="check" if args.check else ("calibrate" if args.output else "preview"),
        interface=args.interface, adapter=binding, profile=str(args.profile),
        profile_sha256=hashlib.sha256(source_bytes).hexdigest(),
        reference_source=str(REFERENCE_PATH), reference_sha256=hashlib.sha256(REFERENCE_PATH.read_bytes()).hexdigest(),
        reference_position_rad=reference, pose_confirmed=args.confirm_folded_pose,
        replace_arm=args.replace_arm, direction_override=args.directions,
        samples=args.samples, interval_sec=args.interval, max_span_rad=args.max_span_rad,
        tolerance_rad=args.tolerance_rad, nodes=[], joints=[], errors=[], warnings=[],
        passed=False, deployment_validated=False,
    )
    try:
        with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as bus:
            bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
            # Avoid unrelated periodic feedback starving a bounded SDO upload.
            # CAN_ERR_FLAG in a mask selects the kernel's error receive list.
            # Match standard data IDs without accidentally subscribing only to errors.
            mask = socket.CAN_SFF_MASK | socket.CAN_EFF_FLAG | socket.CAN_RTR_FLAG
            filters = b"".join(struct.pack("=II", 0x580 + j["node_id"], mask)
                               for j in profile["joints"])
            bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filters)
            bus.bind((args.interface,))
            report["nodes"] = capture(bus, profile["joints"], args.samples, args.interval)
    except (OSError, RuntimeError, ValueError) as error:
        report["errors"].append(str(error))
    rows, errors, warnings = analyse(profile, report["nodes"], reference, args.samples,
                                     args.max_span_rad, args.replace_arm)
    report["joints"] = rows
    report["errors"].extend(errors)
    report["warnings"].extend(warnings)
    if args.check:
        for row in rows:
            if abs(row["existing_reference_error_rad"]) > args.tolerance_rad:
                report["errors"].append(f"{row['name']}: existing reference error "
                                        f"{row['existing_reference_error_rad']:+.9f} rad exceeds tolerance")
    report["passed"] = len(rows) == 6 and not report["errors"]
    if args.output and report["passed"]:
        try:
            candidate = candidate_profile(profile, rows, args.interface, binding)
            write_candidate(args.output, candidate, args.report, report)
            report["candidate_written"] = str(args.output)
            report["candidate_sha256"] = hashlib.sha256(args.output.read_bytes()).hexdigest()
        except (OSError, ValueError) as error:
            report["errors"].append(str(error))
            report["passed"] = False
    if args.report:
        try:
            with args.report.open("x", encoding="utf-8") as stream:
                stream.write(json.dumps(report, indent=2, allow_nan=False) + "\n")
        except (OSError, ValueError) as error:
            report["errors"].append(f"cannot save report: {error}")
            report["passed"] = False
    print_report(report)
    if args.report:
        print("Evidence:", args.report)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
