"""Shared folded-to-ready recipe. No ROS node or motor IO is created here."""
import math
from pathlib import Path

import yaml

_KEYS = {
    "schema_version", "folded_position_rad", "ros_folded_tolerance_rad",
    "commissioning_folded_tolerance_rad", "stopped_velocity_rad_s", "steps",
}


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
