"""Shared folded-to-ready recipe. No ROS node or motor IO is created here."""
import math
from pathlib import Path

import yaml

_KEYS = {
    "schema_version", "folded_position_rad", "ros_folded_tolerance_rad",
    "commissioning_folded_tolerance_rad", "stopped_velocity_rad_s", "steps",
}
JOINT_NAMES = tuple(f"joint_{index}" for index in range(1, 7))
_RATE_KEYS = {"velocity_rad_s", "acceleration_rad_s2"}


def resolve_motion_limits(profile, limits=None):
    """Use the launch's planning dynamics, bounded by the hardware authority."""
    joints = profile.get("joints", [])
    if len(joints) != 6 or {joint["name"] for joint in joints} != set(JOINT_NAMES):
        raise ValueError("motion limits require exactly six hardware joints")
    if limits is not None and (not isinstance(limits, dict) or set(limits) != set(JOINT_NAMES)):
        raise ValueError("motion limits require exactly six planning joints")
    result = {}
    for joint in joints:
        name = joint["name"]
        rates = joint["limits"] if limits is None else limits[name]
        if not isinstance(rates, dict) or (limits is not None and set(rates) != _RATE_KEYS):
            raise ValueError(f"invalid motion limits for {name}")
        result[name] = {}
        for key in _RATE_KEYS:
            value, hardware = rates[key], joint["limits"][key]
            if any(isinstance(v, bool) or not isinstance(v, (int, float))
                   or not math.isfinite(v) or v <= 0 for v in (value, hardware)):
                raise ValueError(f"invalid {key} for {name}")
            if value > hardware:
                raise ValueError(f"{name} motion limits exceed hardware {key}")
            result[name][key] = float(value)
    return result


def rest_to_rest_duration(initial, target, limits):
    """Time a zero-derivative quintic using the normal velocity/acceleration caps."""
    if len(initial) != 6 or len(target) != 6 or any(
            not math.isfinite(q) for q in (*initial, *target)):
        raise ValueError("trajectory timing requires finite six-axis positions")
    duration = 0.01  # One 100 Hz controller period, including a zero-distance goal.
    for name, start, end in zip(JOINT_NAMES, initial, target):
        distance = abs(end - start)
        rates = limits[name]
        duration = max(duration, 1.875 * distance / rates["velocity_rad_s"],
                       math.sqrt(5.774 * distance / rates["acceleration_rad_s2"]))
    if not math.isfinite(duration) or duration > 120:
        raise ValueError("trajectory timing exceeds the 120 second execution budget")
    # Round up to ROS nanoseconds so serialization cannot increase the peak rates.
    return math.ceil(duration * 1_000_000_000) / 1_000_000_000


def validate_recipe(recipe):
    if (not isinstance(recipe, dict) or set(recipe) != _KEYS
            or type(recipe["schema_version"]) is not int or recipe["schema_version"] != 1):
        raise ValueError("unsupported startup recipe schema")
    for key, count, positive in (
        ("folded_position_rad", 6, False),
        ("ros_folded_tolerance_rad", 5, True),
        ("commissioning_folded_tolerance_rad", 5, True),
    ):
        values = recipe[key]
        if not isinstance(values, list) or len(values) != count or any(
            isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
            or (positive and v <= 0) for v in values
        ):
            raise ValueError(f"invalid startup {key}")
    speed = recipe["stopped_velocity_rad_s"]
    if isinstance(speed, bool) or not isinstance(speed, (int, float)) or not math.isfinite(speed) or speed <= 0:
        raise ValueError("invalid startup stationary speed")
    steps = recipe["steps"]
    if not isinstance(steps, list) or len(steps) != 3:
        raise ValueError("startup requires three ordered steps")
    for step, axis in zip(steps, (1, 3, 2)):
        if (not isinstance(step, dict)
                or set(step) != {"joint_index", "target_rad", "duration_sec"}
                or type(step["joint_index"]) is not int or step["joint_index"] != axis):
            raise ValueError("startup must execute J2 -> J4 -> J3")
        for key in ("target_rad", "duration_sec"):
            value = step[key]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"invalid startup {key}")
        if step["duration_sec"] <= 0:
            raise ValueError("invalid startup duration")
    return recipe


def load_recipe(path=None):
    if path is None:
        # Source tests and symlink development installs use this exact source.
        path = Path(__file__).resolve().parents[2] / "hex_arm_controller/config/startup.yaml"
        if not path.is_file():
            from ament_index_python.packages import get_package_share_directory
            path = Path(get_package_share_directory("hex_arm_controller")) / "config/startup.yaml"
    return validate_recipe(yaml.safe_load(Path(path).read_text(encoding="utf-8")))


def ready_position(recipe):
    target = list(recipe["folded_position_rad"])
    for axis in (0, 4, 5):
        target[axis] = 0.0
    for step in recipe["steps"]:
        target[step["joint_index"]] = step["target_rad"]
    return target
