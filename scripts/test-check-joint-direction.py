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
               "bus": {"protocol": "cia402", "interface": "can2"}, "joints": joints}
    before = {"passed": True, "errors": [], "interface": "can2", "nodes": [
        {"node": j["node_id"], "identity": j["identity"].copy(), "position_rev": .25,
         "disabled": True, "error_code": 0, "heartbeat_consumer": 0, "span_rad": 0}
        for j in joints]}
    after = copy.deepcopy(before)
    after["nodes"][2]["position_rev"] -= .08 / math.tau
    return profile, before, after


class DirectionTest(unittest.TestCase):
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
            with self.assertRaises(ValueError):
                check.compare(profile, before, after, 2, -1)
        profile, before, after = fixture()
        after["nodes"][1]["position_rev"] += .02
        with self.assertRaises(ValueError):
            check.compare(profile, before, after, 2, -1)

    def test_rejects_enabled_faulted_moving_or_different_arm_captures(self):
        for field, value in (("disabled", False), ("error_code", 1),
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
