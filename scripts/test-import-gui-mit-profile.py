#!/usr/bin/env python3
"""Offline regression tests for MIT reference mapping and preservation."""
import copy
import importlib.util
import math
from pathlib import Path
import tempfile
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("gui_mit_import", ROOT / "scripts/import-gui-mit-profile.py")
IMPORTER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(IMPORTER)


def source_profile():
    with (ROOT / "config/hardware/firefly_y6.example.yaml").open(encoding="utf-8") as stream:
        profile = yaml.safe_load(stream)
    profile.update(validated=True, calibrated=True)
    for index, joint in enumerate(profile["joints"], 1):
        joint["identity"] = {"vendor_id": 123, "product_code": 456, "revision": 789, "serial_number": index, "model": "TEST"}
        joint["node_id"] = index
    return profile


class ImportGuiMitProfileTest(unittest.TestCase):
    def test_maps_park_and_arbitrary_positions_in_both_directions(self):
        parks = [0.2, -0.15, 0.44, -0.3, 0.02, 0.07]
        converted = IMPORTER.import_profile(source_profile(), parks)
        for i, joint in enumerate(converted["joints"]):
            for delta in (0.0, 0.01, -0.05):
                q = joint["direction"] * math.tau * (parks[i] + delta) + joint["zero_offset_rad"]
                expected = IMPORTER.PARK_ANGLES_RAD[i] + IMPORTER.DIRECTIONS[i] * math.tau * delta
                self.assertAlmostEqual(q, expected, places=12)

    def test_preserves_hardware_identity_limits_and_input(self):
        source = source_profile()
        original = copy.deepcopy(source)
        converted = IMPORTER.import_profile(source)
        self.assertEqual(source, original)
        self.assertFalse(converted["validated"])
        self.assertFalse(converted["calibrated"])
        bus = dict(converted["bus"])
        self.assertEqual(bus.pop("protocol"), "meow")
        self.assertEqual(bus, source["bus"])
        self.assertEqual(converted["controller"]["loop_hz"], 500)
        for index, joint in enumerate(converted["joints"]):
            self.assertEqual(joint["identity"], source["joints"][index]["identity"])
            self.assertEqual(joint["limits"], source["joints"][index]["limits"])
            self.assertEqual(joint["torque_scale"], 1.0)
            self.assertEqual(joint["gravity_compensation_limit_nm"], IMPORTER.GRAVITY_LIMITS_NM[index])
            self.assertEqual((joint["default_kp"], joint["default_kd"]), (80.0, 15.0))

    def test_refuses_invalid_reference_and_node_mapping(self):
        for invalid in (math.nan, math.inf, -128.001, 128.0):
            with self.assertRaises(ValueError):
                IMPORTER.import_profile(source_profile(), [invalid, 0, 0, 0, 0, 0])
        source = source_profile()
        source["joints"][1]["node_id"] = 3
        with self.assertRaises(ValueError):
            IMPORTER.import_profile(source)

    def test_accepts_placeholder_example_without_fabricating_identity(self):
        with (ROOT / "config/hardware/firefly_y6.example.yaml").open(encoding="utf-8") as stream:
            source = yaml.safe_load(stream)
        converted = IMPORTER.import_profile(source)
        self.assertEqual([j["node_id"] for j in converted["joints"]], [1, 2, 3, 4, 5, 6])
        self.assertFalse(converted["validated"])
        for old, new in zip(source["joints"], converted["joints"]):
            self.assertEqual(old["identity"], new["identity"])

    def test_exclusive_output_and_yaml_round_trip(self):
        converted = IMPORTER.import_profile(source_profile())
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "source.local.yaml"
            source.write_text("original\n", encoding="utf-8")
            target = Path(directory) / "mit.local.yaml"
            IMPORTER.write_profile(target, converted, source, IMPORTER.PARK_POSITIONS_REV)
            self.assertEqual(yaml.safe_load(target.read_text(encoding="utf-8")), converted)
            with self.assertRaises(FileExistsError):
                IMPORTER.write_profile(target, converted, source, IMPORTER.PARK_POSITIONS_REV)
            with self.assertRaises(ValueError):
                IMPORTER.write_profile(source, converted, source, IMPORTER.PARK_POSITIONS_REV)
            with self.assertRaises(ValueError):
                IMPORTER.write_profile(Path(directory) / "public.yaml", converted, source, IMPORTER.PARK_POSITIONS_REV)
            self.assertEqual(source.read_text(encoding="utf-8"), "original\n")


if __name__ == "__main__":
    unittest.main()
