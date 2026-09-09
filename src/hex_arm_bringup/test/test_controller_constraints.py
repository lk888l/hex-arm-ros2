from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONTROLLERS = PACKAGE_ROOT / "config" / "controllers.yaml"
JOINTS = [f"joint_{index}" for index in range(1, 7)]


def _constraints() -> dict:
    document = yaml.safe_load(CONTROLLERS.read_text(encoding="utf-8"))
    return document["firefly_arm_controller"]["ros__parameters"]["constraints"]


def test_commissioning_controller_has_strict_six_axis_tolerances() -> None:
    constraints = _constraints()

    assert set(constraints) == {"stopped_velocity_tolerance", "goal_time", *JOINTS}
    assert constraints["stopped_velocity_tolerance"] == 0.02
    assert constraints["goal_time"] == 1.0
    for joint in JOINTS:
        assert constraints[joint] == {"trajectory": 0.02, "goal": 0.005}


def test_joint_two_undertracking_cannot_pass_the_goal_phase() -> None:
    constraints = _constraints()
    observed_goal_error_rad = 0.016

    # JointTrajectoryController requires the final absolute position error to
    # be within `goal` during its goal-time window.  This measured-scale lag is
    # below the former 0.05 rad tolerance but must fail the commissioning gate.
    assert observed_goal_error_rad < 0.05
    assert observed_goal_error_rad > constraints["joint_2"]["goal"]


def test_real_stream_keeps_velocity_without_relaxing_tolerances() -> None:
    override = yaml.safe_load((PACKAGE_ROOT / "config" / "controllers_real.yaml").read_text())
    real = override["firefly_arm_controller"]["ros__parameters"]
    assert real["command_interfaces"] == ["position", "velocity"]
    assert real["interpolate_from_desired_state"] is True
    assert not real.get("open_loop_control", False)
    assert "constraints" not in real
    assert _constraints()["joint_3"]["goal"] == 0.005
