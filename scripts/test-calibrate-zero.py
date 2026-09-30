#!/usr/bin/env python3
"""Offline calibration regressions. Fake SocketCAN responds to real SDO requests."""
import contextlib
import copy
import importlib.util
import io
import json
import math
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

import yaml

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("zero_calibration", ROOT / "scripts/calibrate-zero.py")
cal = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(cal)
SIGNS = [-1, -1, 1, 1, 1, 1]
PARK = [0.005770, 0.251328, 0.253546, -0.001463, -0.001799, 0.000055]
REFERENCE = [0.0, -1.570, 1.570, 0.0, 0.0, 0.0]
BINDING = dict(driver="gs_usb", vendor_id=4617, product_id=8995, serial="A" * 32, channel=2)


def source_profile():
    source = yaml.safe_load((ROOT / "config/hardware/firefly_y6.example.yaml").read_text())
    source.update(validated=True, calibrated=True)
    source = cal.binder.bind_profile(source, "can2", BINDING)
    for i, joint in enumerate(source["joints"]):
        joint.update(node_id=i + 1, direction=SIGNS[i], zero_offset_rad=0.0)
        joint["identity"] = dict(vendor_id=1213809740, product_code=2863267842,
                                 revision=9, serial_number=100 + i, model="TEST")
    return source


def records():
    return [dict(node=i + 1, identity={key: j["identity"][key] for key in cal.reader.IDENTITY_KEYS}, samples_rev=[PARK[i]] * 3,
                 status_words=[0x250] * 4, error_codes=[0] * 4, errors=[])
            for i, j in enumerate(source_profile()["joints"])]


def analyse(source=None, observations=None, **kwargs):
    return cal.analyse(source or source_profile(), observations if observations is not None else records(),
                       REFERENCE, 3, 0.001, **kwargs)


class FakeBus:
    def __init__(self, drift=0.0, enabled=False, timeout_node=None):
        self.sent = []
        self.options = {}
        self.drift, self.enabled, self.timeout_node = drift, enabled, timeout_node
        self.source = source_profile()

    def __enter__(self): return self
    def __exit__(self, *_): pass
    def setsockopt(self, level, option, value): self.options[level, option] = value
    def bind(self, address): self.address = address
    def settimeout(self, _): pass

    def send(self, frame):
        self.sent.append(frame)

    def recv(self, _):
        request = self.sent[-1]
        can_id, length = struct.unpack_from("=IB", request)
        node = can_id - 0x600
        if node == self.timeout_node:
            raise TimeoutError("fake timeout")
        command, index, sub = struct.unpack_from("<BHB", request, 8)
        if command != 0x40 or length != 8:
            raise AssertionError("only SDO upload is allowed")
        if index == 0x1018:
            value = self.source["joints"][node - 1]["identity"][cal.reader.IDENTITY_KEYS[sub - 1]]
            data = struct.pack("<I", value)
        elif index == 0x6041:
            data = struct.pack("<H", 0x237 if self.enabled else 0x250)
        elif index == 0x603f:
            data = struct.pack("<H", 0)
        elif index == 0x6064:
            data = struct.pack("<f", PARK[node - 1] + self.drift)
        else:
            raise AssertionError(f"unexpected SDO index {index:#x}")
        payload = struct.pack("<BHB", 0x43 | ((4 - len(data)) << 2), index, sub) + data
        return struct.pack("=IB3x8s", 0x580 + node, 8, payload)


class CalibrationTest(unittest.TestCase):
    def test_reference_uses_current_joint_coordinate_version(self):
        self.assertEqual(cal.reference_pose(), REFERENCE)
        source = source_profile()
        source["schema_version"] = 2
        with self.assertRaises(ValueError):
            cal.prepare_profile(source)

    def test_reported_snapshot_maps_to_reference_and_displacements(self):
        rows, errors, _ = analyse()
        self.assertEqual(errors, [])
        # Independently specified values for the user's six-axis snapshot.
        expected = [0.036253094, 0.009140397, -0.023076502, 0.009191415, 0.011302565, -0.000345575]
        for i, row in enumerate(rows):
            self.assertAlmostEqual(row["zero_offset_rad"], expected[i], delta=0.000002)
            self.assertAlmostEqual(row["position_after_rad"], REFERENCE[i], places=12)
            # A second pose must retain displacement and sign after calibration.
            displaced = SIGNS[i] * math.tau * (PARK[i] + 0.025) + row["zero_offset_rad"]
            self.assertAlmostEqual(displaced, REFERENCE[i] + SIGNS[i] * math.pi / 20, places=12)

    def test_rejects_unknown_sign_duplicate_nodes_and_nonfinite_offset(self):
        for field, value in (("direction", 0), ("direction", True), ("node_id", 2),
                             ("zero_offset_rad", math.nan)):
            source = source_profile()
            source["joints"][0][field] = value
            with self.assertRaises(ValueError):
                cal.prepare_profile(source)

    def test_template_requires_explicit_signs_and_replacement_identity(self):
        source = yaml.safe_load((ROOT / "config/hardware/firefly_y6.example.yaml").read_text())
        with self.assertRaises(ValueError):
            cal.prepare_profile(source)
        prepared = cal.prepare_profile(source, SIGNS)
        self.assertEqual([j["node_id"] for j in prepared["joints"]], list(range(1, 7)))
        self.assertTrue(analyse(prepared)[1])
        rows, errors, _ = analyse(prepared, replace_arm=True)
        self.assertEqual(errors, [])
        candidate = cal.candidate_profile(prepared, rows, "can2", BINDING)
        self.assertEqual(candidate["joints"][0]["identity"]["serial_number"], 100)
        self.assertIn("unverified", candidate["joints"][0]["identity"]["model"])

    def test_rejects_motion_wrap_noncanonical_and_partial_samples(self):
        for values in ([0, 0.01, 0], [0.49999, -0.49999, 0.49999],
                       [math.nan] * 3, [0.5] * 3, [-0.501] * 3, [0, 0]):
            with self.subTest(values=values):
                data = records()
                data[0]["samples_rev"] = values
                self.assertTrue(analyse(observations=data)[1])

    def test_checks_every_status_and_fault_including_last(self):
        for field, value in (("status_words", 0x237), ("status_words", 0x218), ("error_codes", 0x1000)):
            for sample in (0, 1, 3):
                data = records()
                data[0][field][sample] = value
                self.assertTrue(analyse(observations=data)[1])

    def test_missing_node_and_timeout_fail_without_losing_other_rows(self):
        data = records()[1:]
        rows, errors, _ = analyse(observations=data)
        self.assertEqual(len(rows), 5)
        self.assertTrue(errors)
        bus = FakeBus(timeout_node=2)
        with patch.object(cal.time, "sleep"):
            data = cal.capture(bus, source_profile()["joints"], 3, 0.01)
        rows, errors, _ = analyse(observations=data)
        self.assertEqual(len(rows), 5)
        self.assertTrue(any("timeout" in error for error in errors))

    def test_replacement_is_explicit_and_invalid_identity_is_never_accepted(self):
        data = records()
        data[0]["identity"]["serial_number"] = 789
        self.assertTrue(analyse(observations=data)[1])
        self.assertEqual(analyse(observations=data, replace_arm=True)[1], [])
        data[0]["identity"]["serial_number"] = 0
        self.assertTrue(analyse(observations=data, replace_arm=True)[1])

    def test_candidate_revokes_authority_and_preserves_tuning_and_limits(self):
        source = source_profile()
        original = copy.deepcopy(source)
        rows, _, _ = analyse(source)
        candidate = cal.candidate_profile(source, rows, "can2", BINDING)
        self.assertFalse(candidate["validated"])
        self.assertFalse(candidate["calibrated"])
        self.assertEqual(source, original)
        for old, new in zip(source["joints"], candidate["joints"]):
            for key in ("limits", "default_kp", "default_kd", "torque_scale", "direction"):
                self.assertEqual(old[key], new[key])

    def test_incompatible_limits_are_visible_and_never_expanded(self):
        source = source_profile()
        source["joints"][5]["limits"].update(position_lower_rad=2.06, position_upper_rad=2.16)
        source["joints"][0]["limits"].update(position_lower_rad=-3.14, position_upper_rad=3.14)
        rows, errors, warnings = analyse(source)
        self.assertEqual(errors, [])
        self.assertTrue(any("joint_6" in w and "outside" in w for w in warnings))
        self.assertTrue(any("joint_1" in w and "seam" in w for w in warnings))
        candidate = cal.candidate_profile(source, rows, "can2", BINDING)
        self.assertEqual(candidate["joints"][5]["limits"], source["joints"][5]["limits"])


class CommandLineTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.source = self.directory / "input.local.yaml"
        self.source.write_text(yaml.safe_dump(source_profile()))
        self.output = self.directory / "output.local.yaml"

    def run_cli(self, extra=(), bus=None, source=None):
        bus = bus or FakeBus()
        with patch.object(cal.socket, "socket", return_value=bus) as opened, \
                patch.object(cal.binder, "adapter_binding", return_value=BINDING), \
                patch.object(cal.time, "sleep"), \
                contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            result = cal.main(["--interface", "can2", "--profile", str(source or self.source),
                               "--samples", "3", *extra])
        return result, bus, opened

    def test_live_path_emits_only_reads_and_saves_reproducible_evidence(self):
        result, bus, _ = self.run_cli(["--output", str(self.output), "--confirm-folded-pose"])
        self.assertEqual(result, 0)
        report = json.loads(Path(str(self.output) + ".calibration.json").read_text())
        candidate = yaml.safe_load(self.output.read_text())
        self.assertTrue(report["passed"])
        self.assertFalse(report["deployment_validated"])
        self.assertFalse(candidate["calibrated"])
        for frame in bus.sent:
            self.assertEqual(frame[8], 0x40)
            self.assertTrue(0x601 <= struct.unpack_from("=I", frame)[0] <= 0x606)
        filters = bus.options[cal.socket.SOL_CAN_RAW, cal.socket.CAN_RAW_FILTER]
        for i, (can_id, mask) in enumerate(struct.iter_unpack("=II", filters), 1):
            self.assertEqual(can_id, 0x580 + i)
            self.assertEqual(mask, 0xc00007ff)  # Standard data frames, excluding error receive list.
        # Check the conversion as f32, as used by the real Rust controller.
        f32 = lambda x: struct.unpack("<f", struct.pack("<f", x))[0]
        for joint, row, expected in zip(candidate["joints"], report["joints"], REFERENCE):
            q = f32(f32(f32(joint["direction"] * f32(math.tau)) * f32(row["raw_position_rev"]))
                    + f32(joint["zero_offset_rad"]))
            self.assertAlmostEqual(q, expected, delta=2e-7)

    def test_fresh_read_check_succeeds_and_drift_fails_without_auto_recalibration(self):
        self.run_cli(["--output", str(self.output), "--confirm-folded-pose"])
        contents = self.output.read_bytes()
        args = ["--check", "--confirm-folded-pose"]
        self.assertEqual(self.run_cli(args, source=self.output)[0], 0)
        self.assertEqual(self.run_cli(args, source=self.output, bus=FakeBus(drift=0.01))[0], 1)
        self.assertEqual(contents, self.output.read_bytes())

    def test_fault_writes_failure_report_and_no_candidate(self):
        result, _, _ = self.run_cli(["--output", str(self.output), "--confirm-folded-pose"], bus=FakeBus(enabled=True))
        self.assertEqual(result, 1)
        self.assertFalse(self.output.exists())
        report = json.loads(Path(str(self.output) + ".calibration.json").read_text())
        self.assertFalse(report["passed"])
        self.assertTrue(report["errors"])

    def test_preview_does_not_require_pose_claim_or_create_profile(self):
        report_path = self.directory / "preview.json"
        self.assertEqual(self.run_cli(["--report", str(report_path)])[0], 0)
        report = json.loads(report_path.read_text())
        self.assertFalse(report["pose_confirmed"])
        self.assertEqual(report["mode"], "preview")
        self.assertNotIn("candidate_written", report)

    def test_invalid_cli_and_existing_files_rejected_before_can(self):
        self.output.write_text("preserve me")
        for args in (["--output", str(self.output)], ["--check"],
                     ["--check", "--confirm-folded-pose", "--replace-arm"],
                     ["--output", str(self.output), "--confirm-folded-pose"],
                     ["--report", str(self.source)], ["--interval", "nan"],
                     ["--max-span-rad", "0"], ["--samples", "2"]):
            with self.subTest(args=args), patch.object(cal.socket, "socket") as opened:
                with self.assertRaises(SystemExit) as raised, contextlib.redirect_stderr(io.StringIO()):
                    cal.main(["--interface", "can2", "--profile", str(self.source), *args])
                self.assertEqual(raised.exception.code, 2)
                opened.assert_not_called()
        self.assertEqual(self.output.read_text(), "preserve me")


if __name__ == "__main__":
    unittest.main()
