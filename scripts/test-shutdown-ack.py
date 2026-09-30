#!/usr/bin/env python3
"""Exercise the supervisor's actual JSON verification code without ROS/CAN."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

SOURCE = (Path(__file__).resolve().parent / "supervised-real-launch.sh").read_text()
VERIFIER = SOURCE.split("verification_status=0", 1)[1].split("\n", 2)[2].split("\nPY\n", 1)[0]
LOG = (
    "[hex_arm_controller-1] process started with pid [1234]\n"
    "[hex_arm_controller-1] process has finished cleanly [pid 1234]\n"
)


class ShutdownAcknowledgement(unittest.TestCase):
    def verify(self, state="disabled_confirmed", pid=1234, error=None,
               schema=1, log=LOG, malformed=False, missing=False):
        with tempfile.TemporaryDirectory(prefix="hex-arm-ack-test.") as directory:
            directory = Path(directory)
            report = directory / "driver-shutdown.json"
            log_path = directory / "launch.log"
            log_path.write_text(log)
            if not missing:
                report.write_text("{" if malformed else json.dumps({
                    "schema_version": schema, "pid": pid, "state": state, "error": error,
                }))
            return subprocess.run(
                [sys.executable, "-", str(log_path), str(report), "moveit"],
                input=VERIFIER, text=True, capture_output=True, timeout=3)

    def test_confirmation_does_not_require_old_ready_or_shutdown_log_markers(self):
        result = self.verify()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("structured disabled_confirmed", result.stdout)

    def test_missing_invalid_unconfirmed_or_stale_acknowledgements_fail(self):
        for kwargs in [
            {"missing": True}, {"malformed": True}, {"state": "starting"},
            {"state": "disable_unconfirmed"}, {"pid": 9999}, {"schema": 2},
            {"error": "bus lost"}, {"log": ""},
        ]:
            with self.subTest(kwargs=kwargs):
                self.assertNotEqual(self.verify(**kwargs).returncode, 0)

    def test_drive_confirmation_never_hides_a_ros_child_crash(self):
        result = self.verify(log=LOG + "[move_group] process has died [pid 4567, exit code -11]\n")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("VERIFIED structured disabled_confirmed", result.stdout)
        self.assertIn("a supervised ROS child failed", result.stderr)

    def test_startup_failure_and_successful_disable_are_reported_independently(self):
        result = self.verify(log=LOG + "[python3-8] process has died [pid 4567, exit code 1]\n"
                             "[launch.user] ERROR: controller startup failed with exit code 1\n")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("VERIFIED structured disabled_confirmed", result.stdout)
        self.assertIn("startup verification failed; final drive disable was verified", result.stderr)
        self.assertNotIn("crashed during shutdown", result.stderr)


if __name__ == "__main__":
    unittest.main()
