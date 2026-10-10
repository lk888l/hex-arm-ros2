#!/usr/bin/env python3
"""Adversarial protocol tests against the installed C++ plugin; never opens CAN.

Run inside the sourced ROS workspace. A local fake API deliberately violates
activation acknowledgements/ownership; real Rust integration is covered by
benchmark-transport.py. No hardware profile or motor backend is used here.
"""
import json
import os
from pathlib import Path
import socket
import subprocess
import threading
import time
import unittest

import zenoh
from hex_arm_tools.pb import robot_api_pb2 as pb

ROOT = Path(__file__).resolve().parents[1]
PROBE = ROOT / "install/hex_arm_hardware/lib/hex_arm_hardware/transport_probe"


class FakeApi:
    def __init__(self, failure):
        self.failure = failure
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.owner, self.mode = 0, pb.OPERATING_MODE_DISABLED
        self.acquires = self.enables = self.releases = self.commands = 0
        self.prefix = "test/cpp/activation"
        with socket.socket() as reserve:
            reserve.bind(("127.0.0.1", 0))
            port = reserve.getsockname()[1]
        self.endpoint = f"tcp/127.0.0.1:{port}"
        config = zenoh.Config()
        config.insert_json5("mode", '"peer"')
        config.insert_json5("listen/endpoints", json.dumps([self.endpoint]))
        config.insert_json5("scouting/multicast/enabled", "false")
        self.session = zenoh.open(config)
        self.resources = [self.session.declare_queryable(self.prefix + "/" + suffix, self.query)
                          for suffix in ("description", "arm/description", "rpc/acquire_session",
                                         "rpc/set_mode", "rpc/release_session")]
        self.resources.append(self.session.declare_subscriber(self.prefix + "/arm/command", self.command))
        self.worker = threading.Thread(target=self.publish)
        self.worker.start()

    def command(self, _):
        with self.lock:
            self.commands += 1

    def query(self, query):
        suffix = str(query.key_expr)[len(self.prefix) + 1:]
        with self.lock:
            result = pb.GenericResponse(ok=True)
            if suffix == "description":
                result = pb.RobotDescription(api_version=pb.ApiVersion(major=0))
            elif suffix == "arm/description":
                result = pb.ArmDescription(dof=6, joint_names=[f"joint_{i}" for i in range(1, 7)],
                                           supported_timeouts=[pb.TIMEOUT_BEHAVIOR_FAULT])
            elif suffix == "rpc/acquire_session":
                self.acquires += 1
                self.owner = 7
                if self.failure == "lost_acquire_reply":
                    return
                result = pb.AcquireSessionResponse(ok=True, session_id=7)
            elif suffix == "rpc/set_mode":
                self.enables += 1
                if self.failure == "mode_rejected":
                    result = pb.GenericResponse(ok=False, error="injected mode failure")
                elif self.failure == "missing_active_confirmation":
                    pass  # A successful RPC alone is insufficient to enable writes.
                elif self.failure == "different_holder":
                    self.mode = pb.OPERATING_MODE_ACTIVE
                    self.owner = 99
                else:
                    raise AssertionError(self.failure)
            elif suffix == "rpc/release_session":
                self.releases += 1
                request = pb.ReleaseSessionRequest.FromString(bytes(query.payload))
                if request.session_id == self.owner:
                    self.owner = 0
                    self.mode = pb.OPERATING_MODE_DISABLED
                else:
                    result = pb.GenericResponse(ok=False, error="wrong holder")
            query.reply(query.key_expr, result.SerializeToString())

    def publish(self):
        started = time.monotonic_ns()
        while not self.stop.wait(.01):
            with self.lock:
                stamp = time.monotonic_ns() - started
                header = pb.Header(stamp_ns=stamp)
                joint = pb.JointState(header=header, q=[0.] * 6, dq=[0.] * 6, tau_est=[0.] * 6)
                driver = pb.DriverState(header=header, mode=self.mode, session_owned=bool(self.owner),
                    profile_valid=True, calibrated=True, all_motors_online=True, feedback_fresh=True)
                status = pb.RobotStatus(header=header, session_holder=self.owner)
            for key, value in (("arm/joint_state", joint), ("driver_state", driver), ("status", status)):
                self.session.put(self.prefix + "/" + key, value.SerializeToString())

    def close(self):
        self.stop.set()
        self.worker.join()
        for resource in self.resources:
            resource.undeclare()
        self.session.close()


class ActivationContract(unittest.TestCase):
    def run_case(self, failure):
        api = FakeApi(failure)
        try:
            result = subprocess.run([str(PROBE), api.endpoint, api.prefix,
                                     "1", "0", "baseline"],
                                    env=dict(os.environ, ROS_DOMAIN_ID="89"),
                                    text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    timeout=12)
            self.assertEqual(result.returncode, 4, result.stdout)
            self.assertEqual(api.commands, 0)
            self.assertEqual(api.acquires, 1)
            if failure == "lost_acquire_reply":
                self.assertEqual(api.enables, 0)
                self.assertEqual(api.releases, 0)
            else:
                self.assertGreaterEqual(api.releases, 1, result.stdout)
                if failure != "different_holder":
                    self.assertEqual(api.owner, 0)
                    self.assertEqual(api.mode, pb.OPERATING_MODE_DISABLED)
                else:
                    self.assertEqual(api.owner, 99)
        finally:
            api.close()

    def test_observation_reconfigure_and_context_shutdown_never_acquire(self):
        api = FakeApi("observation")
        try:
            result = subprocess.run([str(PROBE), api.endpoint, api.prefix, "1", "0", "lifecycle"],
                env=dict(os.environ, ROS_DOMAIN_ID="89"), text=True,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=15)
            self.assertEqual(result.returncode, 0, result.stdout)
            self.assertEqual((api.acquires, api.enables, api.releases, api.commands), (0, 0, 0, 0))
        finally:
            api.close()

    def test_mode_failure_rolls_back_acquired_lease(self):
        self.run_case("mode_rejected")

    def test_success_reply_requires_new_active_feedback(self):
        self.run_case("missing_active_confirmation")

    def test_session_owned_boolean_is_not_holder_identity(self):
        self.run_case("different_holder")

    def test_unconfirmed_acquisition_cannot_enable_or_guess_a_lease(self):
        self.run_case("lost_acquire_reply")


if __name__ == "__main__":
    unittest.main()
