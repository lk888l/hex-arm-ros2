#!/usr/bin/env python3
"""Regression checks for install completeness and configuration audit identity."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


def module(name):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name(name + ".py"))
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


install = module("test-runtime-install")
manifest = module("runtime-manifest")


class RuntimePackagingTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.prefix = Path(self.directory.name) / "install"
        for relative in install.REQUIRED:
            self.write(relative)
        self.write("lib/python3.12/site-packages/hex_arm_bridge/pb/robot_api_pb2.py")
        self.write("share/hex_arm_moveit_config/config/joint_limits_commissioning.yaml")
        build_inputs = {"base_image": "reviewed-base"}
        self.write("share/hex_arm_runtime/build-manifest.json",
                   json.dumps({"inputs": build_inputs, "inputs_sha256": manifest.content_hash(build_inputs)}))
        self.profile = Path(self.directory.name) / "profile.yaml"
        self.profile.write_text("profile: original\n")

    def write(self, relative, value="fixture\n"):
        path = self.prefix / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(value)
        return path

    def test_complete_install(self):
        install.check(self.prefix)

    def test_moveit_runtime_and_interposer_are_mandatory(self):
        for relative in ("lib/hex_arm_moveit_runtime/hex_arm_move_group", "lib/libhex_arm_moveit_tem_shutdown.so"):
            with self.subTest(relative=relative):
                path = self.prefix / relative
                path.unlink()
                with self.assertRaisesRegex(RuntimeError, "artifact missing"):
                    install.check(self.prefix)
                self.write(relative)

    def test_external_symlink_rejected(self):
        (self.prefix / "external").symlink_to(self.profile)
        with self.assertRaisesRegex(RuntimeError, "outside install"):
            install.check(self.prefix)

    def test_local_tag_allowed_but_release_requires_review(self):
        manifest.validate_base("hex-arm-jazzy:local")
        with self.assertRaisesRegex(ValueError, "digest"):
            manifest.validate_base("hex-arm-jazzy:local", release=True)
        review = Path(__file__).parents[1] / "docker/release-base.json"
        reviewed = json.loads(review.read_text())["build_image"]
        manifest.validate_base(reviewed, release=True, review_file=review)
        with self.assertRaisesRegex(ValueError, "review"):
            manifest.validate_base("other@sha256:" + "0" * 64, release=True, review_file=review)

    def test_run_identity_changes_with_each_configuration_input(self):
        original = manifest.run_manifest(self.prefix, self.profile)["inputs_sha256"]
        paths = [self.profile, self.prefix / "share/hex_arm_controller/config/startup.yaml",
                 self.prefix / "share/xpkg_urdf_firefly_y6/urdf/xpkg_urdf_firefly_y6.urdf",
                 self.prefix / "share/hex_arm_moveit_config/config/joint_limits_commissioning.yaml"]
        for path in paths:
            with self.subTest(path=path):
                prior = path.read_text()
                path.write_text(prior + "changed\n")
                self.assertNotEqual(original, manifest.run_manifest(self.prefix, self.profile)["inputs_sha256"])
                path.write_text(prior)
        self.assertNotEqual(original, manifest.run_manifest(self.prefix, self.profile, "sha256:new-image")["inputs_sha256"])

    def test_missing_build_manifest_requires_explicit_development_mode(self):
        (self.prefix / "share/hex_arm_runtime/build-manifest.json").unlink()
        with self.assertRaisesRegex(ValueError, "production build manifest missing"):
            manifest.run_manifest(self.prefix, self.profile)
        result = manifest.run_manifest(self.prefix, self.profile, allow_development=True, scope="startup",
                                       launch_argument=["ros2", "launch", "startup.launch.py"])
        self.assertEqual(result["inputs"]["provenance"]["kind"], "development-install")
        self.assertEqual(result["inputs"]["launch"]["scope"], "startup")
        self.assertEqual(result["inputs"]["launch"]["arguments"][-1], "startup.launch.py")

    def test_modified_build_manifest_rejected(self):
        self.write("share/hex_arm_runtime/build-manifest.json", json.dumps({"inputs": {}, "inputs_sha256": "stale"}))
        with self.assertRaisesRegex(ValueError, "hash"):
            manifest.run_manifest(self.prefix, self.profile)

    def test_custom_planning_limits_are_hashed_by_contents(self):
        limits = Path(self.directory.name) / "custom-limits.yaml"
        limits.write_text("velocity: 0.1\n")
        arguments = [f"planning_limits_file:={limits}"]
        original = manifest.run_manifest(self.prefix, self.profile, launch_argument=arguments)
        limits.write_text("velocity: 0.2\n")
        changed = manifest.run_manifest(self.prefix, self.profile, launch_argument=arguments)
        self.assertNotEqual(original["inputs_sha256"], changed["inputs_sha256"])

    def test_installed_binary_change_alters_developer_run_identity(self):
        (self.prefix / "share/hex_arm_runtime/build-manifest.json").unlink()
        original = manifest.run_manifest(self.prefix, self.profile, allow_development=True)
        self.write("lib/hex_arm_controller/hex_arm_controller", "new installed binary\n")
        changed = manifest.run_manifest(self.prefix, self.profile, allow_development=True)
        self.assertNotEqual(original["inputs_sha256"], changed["inputs_sha256"])
        self.assertIn("hex_arm_controller/lib/hex_arm_controller/hex_arm_controller",
                      changed["inputs"]["artifact_sha256"])


if __name__ == "__main__":
    unittest.main()
