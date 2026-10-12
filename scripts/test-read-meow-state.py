#!/usr/bin/env python3
"""Offline Meow wire-format and disabled snapshot regressions; no real CAN."""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("meow_reader", Path(__file__).with_name("read-meow-state.py"))
reader = importlib.util.module_from_spec(spec)
spec.loader.exec_module(reader)


def response(command=0x43, index=0x4564, value=b"\0" * 4, node=1, sub=0):
    return struct.pack("=IB3x8s", 0x580 + node, 8, struct.pack("<BHB", command, index, sub) + value)


class Bus:
    def __init__(self, frames=()):
        self.frames, self.sent = iter(frames), []

    def send(self, data): self.sent.append(data)
    def settimeout(self, _): pass
    def recv(self, _): return next(self.frames)


class MotorBus(Bus):
    def __init__(self, *, late_mode=0, fault=0, timeout_node=None, identity_change=False):
        super().__init__()
        self.late_mode, self.fault = late_mode, fault
        self.timeout_node, self.identity_change = timeout_node, identity_change
        self.counts = {}

    def __enter__(self): return self
    def __exit__(self, *_): pass
    def setsockopt(self, *_): pass
    def bind(self, _): pass

    def recv(self, _):
        frame = self.sent[-1]
        can_id, length = struct.unpack_from("=IB", frame)
        command, index, sub = struct.unpack_from("<BHB", frame, 8)
        node = can_id - 0x600
        if command != 0x40 or length != 8:
            raise AssertionError("only SDO uploads are allowed")
        if node == self.timeout_node:
            raise TimeoutError("fake timeout")
        key = node, index, sub
        count = self.counts.get(key, 0)
        self.counts[key] = count + 1
        if index == 0x1018:
            value = 100 + node if sub == 4 else sub
            data = struct.pack("<I", value + int(self.identity_change and sub == 4 and count > 0))
        elif index == 0x4402:
            data = struct.pack("<B", self.late_mode if count == 3 else 0)
        elif index == 0x453f:
            data = struct.pack("<H", self.fault)
        elif index == 0x4564:
            data = struct.pack("<i", -5 * 2**22)  # -1.25 Rev, outside the legacy single-turn window.
        elif index == 0x1016:
            data = struct.pack("<I", 0)
        elif index == 0x4001:
            data = struct.pack("<B", 7) if sub == 0 else struct.pack("<I", sub)
        else:
            raise AssertionError(f"unexpected Meow SDO object {index:#x}")
        return response(0x43 | ((4 - len(data)) << 2), index, data, node, sub)


class ReaderTest(unittest.TestCase):
    def test_signed_q824_ignores_unrelated_or_short_frames_and_only_uploads(self):
        bus = Bus([b"\0", response(node=2), response(index=0x4402),
                   response(value=struct.pack("<i", -5 * 2**22))])
        self.assertEqual(reader.read_value(bus, 1, 0x4564, 0, "<i") / 2**24, -1.25)
        self.assertEqual(bus.sent, [struct.pack("=IB3x8s", 0x601, 8, bytes.fromhex("4064450000000000"))])

    def test_wrong_size_abort_and_download_ack_are_rejected(self):
        for command in (0x4b, 0x80, 0x60):
            with self.assertRaises(RuntimeError):
                reader.read_value(Bus([response(command=command)]), 1, 0x4564, 0, "<i")

    def test_mode_is_u8_and_detailed_error_is_u16(self):
        for index, command, fmt, value in ((0x4402, 0x4f, "<B", 4), (0x453f, 0x4b, "<H", 0x1234)):
            bus = Bus([response(command, index, struct.pack(fmt, value))])
            self.assertEqual(reader.read_value(bus, 1, index, 0, fmt), value)

    def run_cli(self, bus, factory=False):
        with tempfile.TemporaryDirectory() as directory:
            profile = yaml.safe_load((ROOT / "config/hardware/firefly_y6.example.yaml").read_text())
            for joint in profile['joints']:
                joint['identity'] = dict(zip(reader.IDENTITY_KEYS, (1, 2, 3, 100 + joint['node_id'])))
            source, output = Path(directory) / 'arm.yaml', Path(directory) / 'state.json'
            source.write_text(yaml.safe_dump(profile))
            with patch.object(reader.socket, 'socket', return_value=bus), patch.object(reader.time, 'sleep'), \
                    contextlib.redirect_stdout(io.StringIO()):
                result = reader.main(['--interface', 'can0', '--profile', str(source), '--output', str(output),
                                      '--samples', '3', *(['--factory-calibration'] if factory else [])])
            return result, json.loads(output.read_text())

    def test_real_upload_path_emits_a_verified_meow_snapshot(self):
        bus = MotorBus()
        result, report = self.run_cli(bus, factory=True)
        self.assertEqual(result, 0)
        reader.verify_disabled(report)
        self.assertEqual(len(report['nodes']), 6)
        for node in report['nodes']:
            self.assertEqual(node['position_rev'], -1.25)
            self.assertEqual(node['factory_words_u32'], list(range(1, 8)))
        self.assertTrue(all(frame[8] == 0x40 for frame in bus.sent))

    def test_enabled_faulted_missing_or_changed_identity_saves_failure(self):
        for bus in (MotorBus(late_mode=4), MotorBus(fault=1), MotorBus(timeout_node=2),
                    MotorBus(identity_change=True)):
            result, report = self.run_cli(bus)
            self.assertEqual(result, 1)
            self.assertFalse(report['passed'])
            self.assertTrue(report['errors'])


if __name__ == '__main__':
    unittest.main()
