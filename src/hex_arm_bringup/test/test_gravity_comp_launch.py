import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node


spec = importlib.util.spec_from_file_location("guiding_launch",
    Path(__file__).resolve().parents[1] / "launch/gravity_comp.launch.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def profile_file(tmp_path, **changes):
    profile = {"schema_version": 3, "joint_coordinate_version": 2, "validated": True,
               "calibrated": True, "robot_prefix": "test/hand_guiding",
               "gravity_vector_base_m_s2": [0, 0, -9.81], "bus": {"protocol": "meow"},
               "joints": [{"name": f"joint_{i}"} for i in range(1, 7)]}
    profile.update(changes)
    path = tmp_path / "profile.yaml"
    path.write_text(yaml.safe_dump(profile))
    return path


def test_launch_requires_explicit_activation_and_defaults_to_no_rviz():
    args = {a.name: a for a in module.generate_launch_description().entities
            if isinstance(a, DeclareLaunchArgument)}
    context = LaunchContext()
    for name in ("activate_hardware", "mock", "use_rviz"):
        assert "".join(s.perform(context) for s in args[name].default_value) == "false"


@pytest.mark.parametrize("changes", [{"calibrated": False}, {"validated": False},
    {"schema_version": 2}, {"joint_coordinate_version": 1}, {"joints": []},
    {"bus": {"protocol": "unknown"}}])
def test_bad_real_profile_rejected(tmp_path, changes):
    with pytest.raises(RuntimeError):
        module._load_profile(profile_file(tmp_path, **changes), True, False)


@pytest.mark.parametrize("value", ["[1,2]", "0", "[1,1,1,1,1,0]", "[1,1,1,1,1,.nan]",
                                   "[true,1,1,1,1,1]"])
def test_bad_damping_rejected(value):
    with pytest.raises(RuntimeError):
        module._damping(value)


def test_observation_allows_uncalibrated_but_valid_profile(tmp_path):
    module._load_profile(profile_file(tmp_path, calibrated=False), False, False)


def test_mock_flag_reaches_driver_and_no_trajectory_stack_is_started(tmp_path):
    context = LaunchContext()
    context.launch_configurations.update({
        "hardware_profile": str(profile_file(tmp_path)), "activate_hardware": "true",
        "mock": "true", "damping": "[1,1,1,1,1,1]", "zenoh_endpoint": "tcp/127.0.0.1:7451",
        "use_rviz": "false",
    })
    actions = module._nodes(context)
    drivers = [action for action in actions if isinstance(action, ExecuteProcess) and not isinstance(action, Node)]
    assert len(drivers) == 1
    from launch.utilities import perform_substitutions
    command = [perform_substitutions(context, part) for part in drivers[0].cmd]
    assert "--mock" in command
    assert "--zenoh-listen" in command
    # Recursively walk event handlers too, so a deferred trajectory controller
    # cannot hide in an OnStateTransition callback.
    from test_real_launch_safety import _walk_nodes
    from launch.utilities import normalize_to_list_of_substitutions
    names = {perform_substitutions(context, normalize_to_list_of_substitutions(node.node_executable))
             for node in _walk_nodes(actions)}
    assert "hex_arm_gravity_comp" in names
    assert not names.intersection({"ros2_control_node", "spawner", "move_group"})
