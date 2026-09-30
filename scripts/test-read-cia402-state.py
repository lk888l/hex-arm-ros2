#!/usr/bin/env python3
"""Offline checks for the upload-only legacy motor reader."""
import importlib.util
import math
from pathlib import Path
import struct
import unittest

spec = importlib.util.spec_from_file_location("cia402_reader", Path(__file__).with_name("read-cia402-state.py"))
reader = importlib.util.module_from_spec(spec)
spec.loader.exec_module(reader)


class Bus:
    def __init__(self, frames): self.frames, self.sent = iter(frames), []
    def send(self, data): self.sent.append(data)
    def settimeout(self, _): pass
    def recv(self, _): return next(self.frames)


def response(command=0x43, index=0x6064, value=b"\0" * 4, node=1):
    return struct.pack("=IB3x8s", 0x580 + node, 8, struct.pack("<BHB", command, index, 0) + value)


class ReaderTest(unittest.TestCase):
    def test_upload_ignores_unrelated_frames_and_never_writes(self):
        bus = Bus([response(node=2), response(index=0x6041), response(value=struct.pack("<f", 0.25))])
        self.assertEqual(reader.read_value(bus, 1, 0x6064, 0, "<f"), 0.25)
        self.assertEqual(bus.sent, [struct.pack("=IB3x8s", 0x601, 8, bytes.fromhex("4064600000000000"))])

    def test_size_abort_and_nonfinite_are_rejected(self):
        for frame in (response(command=0x4b), response(command=0x80),
                      response(command=0x60), response(value=struct.pack("<f", math.nan))):
            with self.assertRaises(RuntimeError):
                reader.read_value(Bus([frame]), 1, 0x6064, 0, "<f")

    def test_status_word_upload_size(self):
        bus = Bus([response(command=0x4b, index=0x6041, value=bytes.fromhex("50020000"))])
        self.assertEqual(reader.read_value(bus, 1, 0x6041, 0, "<H"), 0x250)

    def test_j3_coordinate_migration_preserves_motor_angle(self):
        joint = {"name": "joint_3", "direction": 1, "zero_offset_rad": 1.545484}
        old = reader.joint_position({"schema_version": 2}, joint, 0.25)
        joint["zero_offset_rad"] -= 1.57
        new = reader.joint_position({"schema_version": 3, "joint_coordinate_version": 2}, joint, 0.25)
        self.assertAlmostEqual(old, new)
        with self.assertRaises(ValueError):
            reader.joint_position({"schema_version": 3}, joint, 0.25)


if __name__ == "__main__":
    unittest.main()
