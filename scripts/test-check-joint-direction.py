#!/usr/bin/env python3
"""Offline tests of independently observed direction evidence."""
import copy
import importlib.util
import math
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location(
    "direction_check", Path(__file__).with_name("check-joint-direction.py"))
check = importlib.util.module_from_spec(spec)
spec.loader.exec_module(check)


def fixture():
    joints = [{"name": f"joint_{i}", "node_id": i, "direction": 1,
               "identity": dict(vendor_id=1, product_code=2, revision=9, serial_number=100+i)}
              for i in range(1, 7)]
    profile = {"schema_version": 3, "joint_coordinate_version": 2,
               "bus": {"protocol": "meow", "interface": "can2"}, "joints": joints}
    before = {"protocol": "meow", "passed": True, "errors": [], "interface": "can2", "nodes": [
        {"node": j["node_id"], "identity": j["identity"].copy(), "position_rev": .25,
         "disabled": True, "modes": [0] * 4, "error_codes": [0] * 4, "samples_rev": [.25] * 3,
         "heartbeat_consumer": 0, "final_heartbeat_consumer": 0, "span_rad": 0}
        for j in joints]}
    after = copy.deepcopy(before)
    after["nodes"][2]["position_rev"] -= .08 / math.tau
    after["nodes"][2]["samples_rev"] = [after["nodes"][2]["position_rev"]] * 3
    return profile, before, after


class DirectionTest(unittest.TestCase):
    def test_default_meow_and_explicit_legacy_rejection(self):
        profile, before, after = fixture()
        del profile["bus"]["protocol"]
        self.assertTrue(check.compare(profile, before, after, 2, -1)["passed"])
        profile["bus"]["protocol"] = "cia402"
        with self.assertRaisesRegex(ValueError, "Meow"):
            check.compare(profile, before, after, 2, -1)

    def test_multi_turn_positions_do_not_use_legacy_wrap(self):
        profile, before, after = fixture()
        for capture in (before, after):
            for node in capture["nodes"]:
                node["position_rev"] += 1.25
                node["samples_rev"] = [node["position_rev"]] * 3
        self.assertTrue(check.compare(profile, before, after, 2, -1)["passed"])

    def test_agreement_and_opposite_physical_direction_do_not_modify_profile(self):
        profile, before, after = fixture()
        original = copy.deepcopy(profile)
        self.assertTrue(check.compare(profile, before, after, 2, -1)["passed"])
        mismatch = check.compare(profile, before, after, 2, 1)
        self.assertFalse(mismatch["passed"])
        self.assertEqual(mismatch["inferred_direction"], -1)
        self.assertEqual(profile, original)

    def test_rejects_ambiguous_small_wrapped_or_other_joint_motion(self):
        for delta in (0, .001, .4, -6.2, float("nan")):
            profile, before, after = fixture()
            after["nodes"][2]["position_rev"] = .25 + delta / math.tau
            after["nodes"][2]["samples_rev"] = [after["nodes"][2]["position_rev"]] * 3
            with self.assertRaises((ValueError, RuntimeError)):
                check.compare(profile, before, after, 2, -1)
        profile, before, after = fixture()
        after["nodes"][1]["position_rev"] += .02
        after["nodes"][1]["samples_rev"] = [after["nodes"][1]["position_rev"]] * 3
        with self.assertRaises(ValueError):
            check.compare(profile, before, after, 2, -1)

    def test_rejects_enabled_faulted_moving_or_different_arm_captures(self):
        for field, value in (("disabled", False), ("error_codes", [0, 1, 0, 0]),
                             ("modes", [0, 4, 0, 0]), ("final_heartbeat_consumer", 1),
                             ("heartbeat_consumer", 1), ("span_rad", .002),
                             ("identity", {})):
            profile, before, after = fixture()
            after["nodes"][2][field] = value
            with self.assertRaises((ValueError, RuntimeError)):
                check.compare(profile, before, after, 2, -1)
        profile, before, after = fixture()
        profile["joints"][3]["node_id"] = 3
        with self.assertRaises(ValueError):
            check.compare(profile, before, after, 2, -1)


if __name__ == "__main__":
    unittest.main()
