#!/usr/bin/env python3
"""Offline campaign orchestration tests; no sockets or motors are opened."""
import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import yaml

spec = importlib.util.spec_from_file_location("campaign", Path(__file__).with_name("commission-cia402.py"))
campaign = importlib.util.module_from_spec(spec)
spec.loader.exec_module(campaign)


def disabled():
    return {"passed": True, "errors": [], "nodes": [
        {"node": i, "disabled": True, "error_code": 0, "heartbeat_consumer": 0, "span_rad": 0}
        for i in range(1, 7)]}


class CampaignTest(unittest.TestCase):
    def test_stages_reject_jumps_nonfinite_and_excess_travel(self):
        campaign.validate_steps([0], 5, allow_hold=True)
        with self.assertRaises(ValueError):
            campaign.validate_steps([0, .015], 5, allow_hold=True)
        campaign.validate_steps([.015, -.03, .06, -.12, .24], 20)
        for values in ([.04], [.03, .061], [.03, float("nan")], [0], [.3]):
            with self.assertRaises(ValueError):
                campaign.validate_steps(values, 20)
        with self.assertRaises(ValueError):
            campaign.validate_steps([.015], float("inf"))

    def test_independent_shutdown_requires_every_node_and_disarmed_heartbeat(self):
        campaign.verify_disabled(disabled())
        for key, value in (("heartbeat_consumer", 1), ("disabled", False),
                           ("error_code", 1), ("span_rad", .002), ("span_rad", float("nan"))):
            state = disabled()
            state["nodes"][5][key] = value
            with self.assertRaises(RuntimeError):
                campaign.verify_disabled(state)
        state = disabled()
        state["nodes"].pop()
        with self.assertRaises(RuntimeError):
            campaign.verify_disabled(state)

    def run_campaign(self, motion_rc=0, bad_initial=False, bad_final=False, supports=(), preparation=(), temperature_rise=2):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            profile = root / "arm.local.yaml"
            profile.write_text(yaml.safe_dump({"schema_version": 3, "joint_coordinate_version": 2,
                "bus": {"protocol": "cia402", "transport": "socket_can", "interface": "can2"}}))
            binary = root / "controller"
            binary.write_text("test double only")
            output = root / "report"
            argv = ["campaign", "--profile", str(profile), "--axis", "joint_6", "--deltas-rad",
                    ".03", ".06", "--binary", str(binary), "--output-dir", str(output), "--allow-motion"]
            argv.extend(["--max-temperature-rise-c", str(temperature_rise)])
            if supports:
                argv.extend(["--support-axes", *supports])
            if preparation:
                argv.extend(["--prepare-supports", *preparation])
            commands = []
            def run(command, log, timeout):
                commands.append(command)
                log.write_text("")
                if "--output" in command:
                    state = copy.deepcopy(disabled())
                    if (bad_initial and len(commands) == 1) or (bad_final and len(commands) > 1):
                        state["nodes"][0]["heartbeat_consumer"] = 123
                    Path(command[command.index("--output") + 1]).write_text(json.dumps(state))
                    return 0
                return motion_rc
            with patch.object(campaign.sys, "argv", argv), patch.object(campaign, "run_process", run):
                rc = campaign.main()
            return rc, commands, json.loads((output / "result.json").read_text())

    def test_failed_initial_state_sends_no_motion(self):
        rc, commands, result = self.run_campaign(bad_initial=True)
        self.assertEqual(rc, 1)
        self.assertEqual(len(commands), 1)
        self.assertEqual(result["trials"], [])
        self.assertEqual(result["initial_state"]["nodes"][0]["heartbeat_consumer"], 123)
        self.assertIn("initial_state_error", result)

    def test_failed_motion_still_reads_shutdown_and_withholds_later_steps(self):
        for motion_rc in (1, 124, 143):
            rc, commands, result = self.run_campaign(motion_rc=motion_rc)
            self.assertEqual(rc, 1)
            self.assertEqual(len(commands), 3)
            self.assertEqual(len(result["trials"]), 1)
            self.assertFalse(result["passed"])
            self.assertTrue(result["trials"][0]["final_state"]["passed"])

    def test_failed_final_state_withholds_later_steps(self):
        rc, commands, result = self.run_campaign(bad_final=True)
        self.assertEqual(rc, 1)
        self.assertEqual(len(commands), 3)
        self.assertFalse(result["passed"])

        trial = result["trials"][0]
        self.assertEqual(trial["final_state"]["nodes"][0]["heartbeat_consumer"], 123)
        self.assertIn("final_state_error", trial)

    def test_successful_campaign_always_uses_strict_travel_gates(self):
        rc, commands, result = self.run_campaign()
        self.assertEqual(rc, 0)
        self.assertTrue(result["passed"])
        self.assertEqual(len(commands), 5)
        for command in (commands[1], commands[3]):
            self.assertIn("--allow-expanded-motion", command)

    def test_explicit_temperature_rise_is_forwarded_and_recorded(self):
        rc, commands, result = self.run_campaign(supports=("joint_4",), temperature_rise=10)
        self.assertEqual(rc, 0)
        self.assertEqual(result["max_temperature_rise_c"], 10)
        for command in (commands[1], commands[3]):
            self.assertEqual(command[command.index("--max-temperature-rise-c")+1], "10.0")
        for rise in (1, 11, float("nan"), float("inf")):
            with self.assertRaises(SystemExit):
                self.run_campaign(supports=("joint_4",), temperature_rise=rise)
        with self.assertRaises(SystemExit):
            self.run_campaign(temperature_rise=10)

    def test_supported_trial_preserves_named_authority_and_stops_after_failure(self):
        rc, commands, result = self.run_campaign(motion_rc=1, supports=("joint_4",),
                                                  preparation=("joint_4:-0.03",))
        self.assertEqual(rc, 1)
        self.assertEqual(len(commands), 3)
        self.assertEqual(result["support_axes"], ["joint_4"])
        self.assertEqual(result["prepare_supports"], ["joint_4:-0.03"])
        self.assertEqual(commands[1][-4:], ["--support-axes", "joint_4", "--prepare-supports", "joint_4:-0.03"])
        self.assertTrue(result["trials"][0]["final_state"]["passed"])


if __name__ == "__main__":
    unittest.main()
