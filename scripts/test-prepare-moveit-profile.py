#!/usr/bin/env python3
"""Offline regression checks for GUI-to-SI deployment preparation."""
import copy
import importlib.util
import math
from pathlib import Path
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("prepare_profile", ROOT / "scripts/prepare-moveit-profile.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class PrepareProfileTest(unittest.TestCase):
    def setUp(self):
        self.profile = yaml.safe_load((ROOT / "config/hardware/firefly_y6.meow_mit.example.yaml").read_text())
        self.profile.update(validated=True, calibrated=True)
        self.urdf = (ROOT / "src/xpkg_urdf_firefly_y6/urdf/xpkg_urdf_firefly_y6.urdf").read_text()

    def test_rev_rates_are_converted_exactly_once(self):
        self.assertAlmostEqual(module.si_rate(None, 0.2), 1.2566370614359172)
        self.assertEqual(module.si_rate(1.25, None), 1.25)
        for value in [0, -1, float("nan"), float("inf"), True]:
            with self.assertRaises(ValueError):
                module.si_rate(None, value)

    def test_identity_calibration_and_gravity_survive_full_range_expansion(self):
        previous = copy.deepcopy(self.profile)
        value = module.si_rate(None, 0.2)
        result = module.prepare(self.profile, self.urdf, value, value, True, True)
        self.assertEqual(self.profile, previous)
        self.assertEqual(result["bus"], previous["bus"])
        self.assertEqual(result["controller"], previous["controller"])
        for before, after in zip(previous["joints"], result["joints"]):
            for key in ("identity", "direction", "zero_offset_rad", "gravity_compensation_scale", "default_kp", "default_kd"):
                self.assertEqual(before[key], after[key])
            self.assertEqual(after["limits"]["velocity_rad_s"], value)
            self.assertEqual(after["limits"]["acceleration_rad_s2"], value)
            self.assertNotIn("motion_feedforward", after)
        self.assertEqual(result["joints"][0]["limits"]["position_lower_rad"], -2.86)
        self.assertEqual(result["joints"][1]["limits"]["position_upper_rad"], 2.09)
        self.assertEqual(result["joints"][5]["limits"]["position_upper_rad"], 2.79)

    def test_default_preserves_positions_and_urdf_still_caps_speed(self):
        result = module.prepare(self.profile, self.urdf, 20.0, 2.0)
        for before, after in zip(self.profile["joints"], result["joints"]):
            self.assertEqual(before["limits"]["position_lower_rad"], after["limits"]["position_lower_rad"])
            self.assertEqual(after["limits"]["velocity_rad_s"], 6.0)

    def test_missing_calibration_or_joint_cannot_be_promoted(self):
        for fault in ("calibrated", "validated", "schema_version", "joints"):
            profile = copy.deepcopy(self.profile)
            if fault == "joints": profile["joints"].pop()
            else: profile[fault] = False
            with self.assertRaises(ValueError):
                module.prepare(profile, self.urdf, 1.0, 1.0)


if __name__ == "__main__":
    unittest.main()
