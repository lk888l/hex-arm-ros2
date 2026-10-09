#!/usr/bin/env python3
"""Record immutable build inputs and the exact configuration of one arm run."""
import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
import re
import subprocess
import tomllib


REVIEWED_ABI = {"moveit": "2.12.4", "rclcpp": "28.1.21"}
DIGEST = re.compile(r"[^\s@]+@sha256:[0-9a-f]{64}\Z")


def file_record(path):
    path = Path(path)
    return {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def content_hash(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def validate_base(build_image, release=False, review_file=None):
    if not release:
        return
    if not DIGEST.fullmatch(build_image):
        raise ValueError("release BUILD_IMAGE must be a reviewed name@sha256 digest")
    if review_file is None:
        raise ValueError("release base image requires a checked-in ABI review")
    review = json.loads(Path(review_file).read_text())
    if review.get("build_image") != build_image or review.get("abi") != REVIEWED_ABI:
        raise ValueError("release base image does not match the checked-in ABI review")


def installed_inputs(prefix):
    prefix = Path(prefix)
    def share(package):
        candidates = [prefix / "share" / package, prefix / package / "share" / package,
                      prefix.parent / package / "share" / package]
        return next((path for path in candidates if path.is_dir()), candidates[0])

    paths = [("hex_arm_controller", share("hex_arm_controller") / "config/startup.yaml"),
             ("xpkg_urdf_firefly_y6", share("xpkg_urdf_firefly_y6") / "urdf/xpkg_urdf_firefly_y6.urdf")]
    for package in ("hex_arm_bringup", "hex_arm_moveit_config"):
        paths += [(package, path) for path in sorted((share(package) / "config").rglob("*.yaml"))]
        paths += [(package, path) for path in sorted((share(package) / "config").rglob("*.srdf"))]
        paths += [(package, path) for path in sorted((share(package) / "launch").glob("*.launch.py"))]
    return {f"share/{package}/{path.relative_to(share(package))}": file_record(path)["sha256"]
            for package, path in paths}


def command(*args):
    return subprocess.check_output(args, text=True).strip()


def installed_artifacts(prefix):
    """Hash actual installed code, including symlink-install Python modules."""
    prefix = Path(prefix)
    result = {}
    artifacts = {
        "hex_arm_controller": ["lib/hex_arm_controller/hex_arm_controller"],
        "hex_arm_hardware": ["lib/libhex_arm_hardware.so"],
        "hex_arm_moveit_runtime": ["lib/hex_arm_moveit_runtime/hex_arm_move_group",
                                    "lib/libhex_arm_moveit_tem_shutdown.so"],
        "hex_arm_bridge": [],
    }
    for package, names in artifacts.items():
        candidates = [prefix, prefix / package, prefix.parent / package]
        package_prefix = next((path for path in candidates if (path / "share" / package).is_dir()), candidates[1])
        paths = [package_prefix / name for name in names]
        if package == "hex_arm_bridge":
            paths += sorted(package_prefix.glob("lib/python*/site-packages/hex_arm_bridge/**/*.py"))
        for path in paths:
            if path.is_file():
                result[f"{package}/{path.relative_to(package_prefix)}"] = file_record(path)["sha256"]
    return result


def build_manifest(source, prefix, build_image, source_revision, build_flags, release=False, review_file=None):
    validate_base(build_image, release, review_file)
    if release and not re.fullmatch(r"(?:[0-9a-f]{40}|[0-9a-f]{64})", source_revision):
        raise ValueError("release source revision must be the full committed revision")
    source = Path(source)
    dependencies = tomllib.loads((source / "src/hex_arm_controller/Cargo.lock").read_text())["package"]
    dpkg = command("dpkg-query", "-W", "-f=${binary:Package}\t${Version}\n")
    source_files = {}
    for directory in ("src", "config", "scripts", "docker"):
        for path in sorted((source / directory).rglob("*")):
            if (path.is_file() and not any(part in {"target", "__pycache__", ".pytest_cache"} for part in path.parts)
                    and not path.name.endswith((".pyc", ".local.yaml"))):
                source_files[str(path.relative_to(source))] = file_record(path)["sha256"]
    inputs = {"base_image": build_image, "source_revision": source_revision,
              "source_files_sha256": content_hash(source_files), "build_flags": build_flags,
              "dpkg_versions": dict(line.split("\t", 1) for line in dpkg.splitlines()),
              "rustc": command("rustc", "--version"), "cargo": command("cargo", "--version"),
              "python_zenoh": importlib.metadata.version("eclipse-zenoh"),
              "rust_dependencies": {item["name"]: item["version"] for item in dependencies
                                    if item["name"] in {"zenoh", "prost", "tokio"}},
              "configuration_sha256": installed_inputs(prefix)}
    return {"schema_version": 1, "kind": "build", "release": release,
            "reviewed_abi": REVIEWED_ABI, "inputs": inputs, "inputs_sha256": content_hash(inputs)}


def development_provenance(prefix, source_root=None):
    candidates = [Path(source_root)] if source_root else [Path(prefix), *Path(prefix).parents]
    source = next((path for path in candidates if (path / ".git").exists()), None)
    provenance = {"kind": "development-install", "source_revision": None, "source_dirty": None}
    if source:
        try:
            source = source.resolve()
            # Bind-mounted developer checkouts may have a different owner in
            # Docker. Trust only this explicit checkout for these read calls.
            git = ["git", "-c", f"safe.directory={source}", "-C", str(source)]
            provenance.update(source_root=str(source), source_revision=command(*git, "rev-parse", "HEAD"),
                              source_dirty=bool(command(*git, "status", "--porcelain")))
        except (OSError, subprocess.CalledProcessError):
            pass
    return provenance


def run_manifest(prefix, profile, image_identity=None, allow_development=False, scope="moveit",
                 launch_argument=None, source_root=None):
    prefix = Path(prefix)
    build_path = prefix / "share/hex_arm_runtime/build-manifest.json"
    if build_path.is_file():
        build = json.loads(build_path.read_text())
        if build["inputs_sha256"] != content_hash(build["inputs"]):
            raise ValueError("build manifest input hash does not match its contents")
        provenance = {"kind": "runtime-image", "build_manifest": file_record(build_path),
                      "build_inputs_sha256": build["inputs_sha256"]}
    elif allow_development:
        provenance = development_provenance(prefix, source_root)
    else:
        raise ValueError("production build manifest missing; use --allow-development for an explicit development install")
    arguments = list(launch_argument or [])
    external_inputs = {}
    for argument in arguments:
        name, separator, value = argument.partition(":=")
        if separator and value and name in {"planning_limits_file", "startup_sequence"}:
            external_inputs[name] = file_record(value)
    inputs = {"provenance": provenance,
              "image_identity": image_identity or "unreported",
              "hardware_profile": file_record(profile), "configuration_sha256": installed_inputs(prefix),
              "artifact_sha256": installed_artifacts(prefix),
              "external_configuration": external_inputs,
              "launch": {"scope": scope, "arguments": arguments}}
    return {"schema_version": 1, "kind": "run", "inputs": inputs, "inputs_sha256": content_hash(inputs)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    build = sub.add_parser("build")
    build.add_argument("--source", type=Path, required=True)
    build.add_argument("--prefix", type=Path, required=True)
    build.add_argument("--source-revision", required=True)
    build.add_argument("--build-flags", required=True)
    build.add_argument("--output", type=Path, required=True)
    for target in (build, sub.add_parser("validate-base")):
        target.add_argument("--build-image", required=True)
        target.add_argument("--release", action="store_true")
        target.add_argument("--review-file", type=Path)
    run = sub.add_parser("run")
    run.add_argument("--prefix", type=Path, required=True)
    run.add_argument("--profile", type=Path, required=True)
    run.add_argument("--image-identity")
    run.add_argument("--allow-development", action="store_true")
    run.add_argument("--source-root", type=Path)
    run.add_argument("--scope", choices=["moveit", "bringup", "startup"], default="moveit")
    run.add_argument("--launch-argument", action="append", help="repeat with =VALUE to preserve arguments starting with --")
    run.add_argument("--output", type=Path, required=True)
    args = vars(parser.parse_args())
    action = args.pop("action")
    if action == "validate-base":
        validate_base(**args)
        return
    output = args.pop("output")
    result = build_manifest(**args) if action == "build" else run_manifest(**args)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")


if __name__ == "__main__":
    main()
