#!/usr/bin/env python3
"""Read legacy HexMeow CiA402 state without enabling or writing drive objects.

Stop other CAN owners first. A profile is optional; when supplied, identities
and joint coordinates are checked, including the schema-v2 J3 conversion.
The JSON report is also written on a failed read or identity mismatch.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import socket
import struct
import time

import yaml


IDENTITY_KEYS = ("vendor_id", "product_code", "revision", "serial_number")


def upload(bus, node, index, subindex=0, timeout=0.5):
    if not 1 <= node <= 127 or not 0 < timeout <= 5:
        raise ValueError("invalid SDO node or timeout")
    request = struct.pack("<BHB4x", 0x40, index, subindex)
    bus.send(struct.pack("=IB3x8s", 0x600 + node, 8, request))
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        bus.settimeout(max(0.001, deadline - time.monotonic()))
        frame = bus.recv(72)
        if len(frame) < 16:
            continue
        can_id, length = struct.unpack_from("=IB", frame)
        if can_id != 0x580 + node or length != 8:
            continue
        data = frame[8:16]
        if data[1:4] != request[1:4]:
            continue
        if data[0] == 0x80:
            code = int.from_bytes(data[4:8], "little")
            raise RuntimeError(f"node {node} SDO {index:04x}:{subindex:02x} abort 0x{code:08x}")
        if data[0] not in (0x43, 0x47, 0x4b, 0x4f):
            raise RuntimeError(f"unsupported SDO response {data.hex()}")
        return data[4:8 - ((data[0] >> 2) & 3)]
    raise TimeoutError(f"node {node} SDO {index:04x}:{subindex:02x} timed out")


def read_value(bus, node, index, subindex, fmt):
    data = upload(bus, node, index, subindex)
    if len(data) != struct.calcsize(fmt):
        raise RuntimeError(f"node {node} SDO {index:04x}:{subindex:02x} unexpected size {len(data)}")
    value = struct.unpack(fmt, data)[0]
    if isinstance(value, float) and not math.isfinite(value):
        raise RuntimeError(f"node {node} SDO {index:04x}:{subindex:02x} non-finite value")
    return value


def joint_position(profile, joint, raw):
    version = (profile.get("schema_version"), profile.get("joint_coordinate_version"))
    if version not in ((2, None), (3, 2)):
        raise ValueError("requires schema v2 or schema v3 / coordinates v2")
    shift = 1.57 if version == (2, None) and joint["name"] == "joint_3" else 0.0
    return joint["direction"] * math.tau * raw + joint["zero_offset_rad"] - shift


def observe(bus, node, samples):
    uint = lambda index, sub=0: read_value(bus, node, index, sub, "<I")
    record = {"node": node, "identity": dict(zip(IDENTITY_KEYS, [uint(0x1018, i) for i in range(1, 5)]))}
    for label, index, sub, fmt in (
        ("status_word", 0x6041, 0, "<H"), ("mode", 0x6061, 0, "<b"),
        ("error_code", 0x603f, 0, "<H"), ("heartbeat_consumer", 0x1016, 1, "<I"),
        ("peak_torque_nm", 0x6076, 0, "<f"), ("mit_factor", 0x2003, 7, "<f"),
        ("driver_temp_x10", 0x2204, 1, "<h"), ("motor_temp_x10", 0x2204, 2, "<h"),
    ):
        record[label] = read_value(bus, node, index, sub, fmt)
    positions = []
    for i in range(samples):
        positions.append(read_value(bus, node, 0x6064, 0, "<f"))
        if i + 1 < samples:
            time.sleep(0.05)
    record["samples_rev"] = positions
    record["position_rev"] = sorted(positions)[len(positions) // 2]
    record["span_rad"] = (max(positions) - min(positions)) * math.tau
    record["final_status_word"] = read_value(bus, node, 0x6041, 0, "<H")
    record["disabled"] = all((word & 0x6f) in (0x40, 0x21, 0x23)
                             for word in (record["status_word"], record["final_status_word"]))
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", required=True)
    parser.add_argument("--profile", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--samples", type=int, default=7)
    args = parser.parse_args()
    if not 2 <= args.samples <= 100:
        parser.error("--samples must be within 2..100")
    profile = yaml.safe_load(args.profile.read_text()) if args.profile else None
    if profile and (profile["bus"].get("protocol", "cia402") != "cia402"
                    or profile["bus"]["interface"] != args.interface):
        parser.error("requires a CiA402 profile bound to the selected interface")
    report = {"time": datetime.now(timezone.utc).isoformat(), "interface": args.interface,
              "nodes": [], "errors": [], "passed": False}
    if profile:
        report.update(profile=str(args.profile), profile_sha256=hashlib.sha256(args.profile.read_bytes()).hexdigest())
    try:
        with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as bus:
            bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
            bus.bind((args.interface,))
            for i in range(6):
                joint = profile["joints"][i] if profile else None
                node = joint["node_id"] if joint else i + 1
                try:
                    record = observe(bus, node, args.samples)
                    report["nodes"].append(record)
                    if joint:
                        record["q_coordinate_v2_rad"] = joint_position(profile, joint, record["position_rev"])
                        if record["identity"] != {key: joint["identity"][key] for key in IDENTITY_KEYS}:
                            raise RuntimeError(f"node {node} identity mismatch")
                    if not record["disabled"] or record["error_code"] != 0:
                        raise RuntimeError(f"node {node} is enabled, faulted, or has an unconfirmed disabled state")
                except (OSError, RuntimeError, ValueError) as error:
                    report["errors"].append(str(error))
    except OSError as error:
        report["errors"].append(str(error))
    report["passed"] = len(report["nodes"]) == 6 and not report["errors"]
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
