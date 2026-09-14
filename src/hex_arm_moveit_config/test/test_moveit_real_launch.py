from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
from types import SimpleNamespace
import pytest

from launch import LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_ROOT / "launch" / "moveit_real.launch.py"


def _module():
    spec = spec_from_file_location("hex_arm_moveit_real_launch", LAUNCH_FILE)
    assert spec is not None and spec.loader is not None
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class _FakeMoveItConfig:
    robot_description = {"robot_description": "test"}
    robot_description_semantic = {"robot_description_semantic": "test"}
    robot_description_kinematics = {"robot_description_kinematics": {}}
    planning_pipelines = {"planning_pipelines": ["ompl"]}
    joint_limits = {"robot_description_planning": {}}

    @staticmethod
    def to_dict():
        return {"moveit_config": "test"}


def _context(enable_execution: bool, publish_world_tf: bool = True) -> LaunchContext:
    context = LaunchContext()
    context.launch_configurations.update(
        {
            "hardware_profile": "/tmp/validated.local.yaml",
            "zenoh_connect": "",
            "enable_execution": str(enable_execution).lower(),
            "startup_ready": "true",
            "publish_world_tf": str(publish_world_tf).lower(),
            "use_rviz": "false",
        }
    )
    return context


def test_public_launch_defaults_to_plan_only() -> None:
    module = _module()
    declarations = {
        entity.name: entity
        for entity in module.generate_launch_description().entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    assert set(declarations) == {
        "hardware_profile",
        "zenoh_connect",
        "enable_execution",
        "startup_ready",
        "publish_world_tf",
        "use_rviz",
    }
    context = LaunchContext()
    assert perform_substitutions(
        context, declarations["enable_execution"].default_value
    ) == "false"
    assert declarations["enable_execution"].choices == ["true", "false"]
    assert perform_substitutions(context, declarations["startup_ready"].default_value) == "true"
    assert declarations["hardware_profile"].description == (
        "Absolute path to a validated hardware profile. Calibration is required only "
        "when enable_execution is true."
    )
    assert declarations["zenoh_connect"].description == (
        "Optional explicit Zenoh endpoint. Empty uses the deterministic "
        "controller-to-bridge loopback endpoint tcp/127.0.0.1:7448."
    )


def test_one_switch_gates_hardware_and_moveit_execution() -> None:
    module = _module()
    for enabled in (False, True):
        arguments = module._bringup_arguments("/tmp/profile.yaml", "", enabled)
        runtime = module._move_group_runtime_parameters(enabled)
        expected = "true" if enabled else "false"
        assert arguments["activate_hardware"] == expected
        assert arguments["startup_ready"] == expected
        if enabled:
            with pytest.raises(RuntimeError, match="automatic J2"):
                module._bringup_arguments("/tmp/profile.yaml", "", enabled, False)
        else:
            assert module._bringup_arguments("/tmp/profile.yaml", "", False, False)["startup_ready"] == "false"
        assert arguments["use_rviz"] == "false"
        assert runtime["allow_trajectory_execution"] is enabled


def test_move_group_exit_is_a_global_shutdown_boundary() -> None:
    module = _module()
    context = LaunchContext()

    clean = module._shutdown_after_exit("move_group")(
        SimpleNamespace(returncode=0), context
    )
    assert len(clean) == 2
    assert isinstance(clean[0], LogInfo)
    assert isinstance(clean[1], EmitEvent)
    clean_message = perform_substitutions(context, clean[0].msg)
    assert "exited cleanly" in clean_message
    assert "ERROR" not in clean_message
    assert clean[1].event.reason == "critical process move_group exited cleanly"

    failed = module._shutdown_after_exit("move_group")(
        SimpleNamespace(returncode=-11), context
    )
    assert perform_substitutions(context, failed[0].msg).startswith("ERROR:")
    assert "code -11" in failed[1].event.reason


def test_semantic_model_is_relaxed_only_for_non_executable_plan_only_mode() -> None:
    module = _module()
    assert module._semantic_file(False) == "config/firefly_y6.plan_only.srdf"
    assert module._semantic_file(True) == "config/firefly_y6.srdf"
    source = LAUNCH_FILE.read_text(encoding="utf-8")
    assert (
        ".robot_description_semantic(file_path=_semantic_file(enable_execution))"
        in source
    )


def test_plan_only_config_does_not_configure_a_controller_manager() -> None:
    module = _module()
    plan_only = module._build_moveit_config(False).to_dict()
    execution = module._build_moveit_config(True).to_dict()

    execution_only_parameters = {
        "moveit_controller_manager",
        "moveit_manage_controllers",
        "moveit_simple_controller_manager",
        "trajectory_execution",
    }
    assert execution_only_parameters.isdisjoint(plan_only)
    assert execution_only_parameters <= execution.keys()
    assert execution["moveit_controller_manager"] == (
        "moveit_simple_controller_manager/MoveItSimpleControllerManager"
    )


def test_execution_launch_builds_only_the_strict_semantic_model(monkeypatch) -> None:
    module = _module()
    selected_execution_modes = []

    def _fake_config(enable_execution: bool, hardware_profile=None):
        selected_execution_modes.append(enable_execution)
        return _FakeMoveItConfig()

    monkeypatch.setattr(module, "_build_moveit_config", _fake_config)
    monkeypatch.setattr(
        module,
        "_move_group_environment",
        lambda: {"LD_PRELOAD": "/tmp/libhex_arm_moveit_tem_shutdown.so"},
    )
    monkeypatch.setattr(module, "get_package_share_directory", lambda _package: "/tmp")
    module._launch_setup(_context(enable_execution=True))
    assert selected_execution_modes == [True]


def test_launch_composes_bringup_without_duplicate_control_or_rsp(monkeypatch) -> None:
    module = _module()
    selected_execution_modes = []

    def _fake_config(enable_execution: bool, hardware_profile=None):
        selected_execution_modes.append(enable_execution)
        return _FakeMoveItConfig()

    monkeypatch.setattr(module, "_build_moveit_config", _fake_config)
    monkeypatch.setattr(
        module,
        "_move_group_environment",
        lambda: {"LD_PRELOAD": "/tmp/libhex_arm_moveit_tem_shutdown.so"},
    )
    monkeypatch.setattr(
        module,
        "get_package_share_directory",
        lambda package: str(
            PACKAGE_ROOT if package == "hex_arm_moveit_config" else "/tmp/bringup"
        ),
    )

    context = _context(enable_execution=False)
    actions = module._launch_setup(context)
    assert selected_execution_modes == [False]
    includes = [action for action in actions if isinstance(action, IncludeLaunchDescription)]
    nodes = [action for action in actions if isinstance(action, Node)]
    handlers = [action for action in actions if isinstance(action, RegisterEventHandler)]
    assert len(includes) == 1
    assert len(handlers) == 1
    include_source = includes[0].launch_description_source
    include_path = perform_substitutions(
        context, vars(include_source)["_LaunchDescriptionSource__location"]
    )
    assert include_path.endswith(
        "/launch/real.launch.py"
    )

    packages = [node.node_package for node in nodes]
    assert packages == ["hex_arm_moveit_runtime", "tf2_ros", "rviz2"]
    assert "controller_manager" not in packages
    assert "robot_state_publisher" not in packages


def test_world_transform_can_be_delegated_to_an_external_owner(monkeypatch) -> None:
    module = _module()
    monkeypatch.setattr(
        module, "_build_moveit_config", lambda _enable_execution, _profile=None: _FakeMoveItConfig()
    )
    monkeypatch.setattr(
        module,
        "_move_group_environment",
        lambda: {"LD_PRELOAD": "/tmp/libhex_arm_moveit_tem_shutdown.so"},
    )
    monkeypatch.setattr(module, "get_package_share_directory", lambda _package: "/tmp")
    actions = module._launch_setup(
        _context(enable_execution=False, publish_world_tf=False)
    )
    packages = [
        action.node_package for action in actions if isinstance(action, Node)
    ]
    assert "tf2_ros" not in packages


def test_real_launch_hard_codes_commissioning_limits_and_state_topic() -> None:
    source = LAUNCH_FILE.read_text(encoding="utf-8")
    assert '.joint_limits(file_path="config/joint_limits_commissioning.yaml")' in source
    assert "limits_profile" not in source
    assert 'STATE_TOPIC = "/hex_arm/internal/state"' in source
    assert 'remappings=[("joint_states", STATE_TOPIC)]' in source
    assert 'package="controller_manager"' not in source
    assert 'package="robot_state_publisher"' not in source
    assert "target_action=move_group" in source
    assert 'on_exit=_shutdown_after_exit("move_group")' in source


def test_mock_launch_can_never_select_the_plan_only_semantic_model() -> None:
    source = (PACKAGE_ROOT / "launch" / "moveit_mock.launch.py").read_text(
        encoding="utf-8"
    )
    assert '.robot_description_semantic(file_path="config/firefly_y6.srdf")' in source
    assert "firefly_y6.plan_only.srdf" not in source


def test_plan_only_collision_fixture_is_offline_by_construction() -> None:
    source = (PACKAGE_ROOT / "test" / "plan_only_move_group.launch.py").read_text(
        encoding="utf-8"
    )
    assert 'mappings={"backend": "view", "controllers_file": ""}' in source
    assert 'file_path="config/firefly_y6.plan_only.srdf"' in source
    for forbidden in (
        "hex_arm_bringup",
        "controller_manager",
        "ros2_control_node",
        "hex_arm_bridge",
        "SocketCAN",
    ):
        assert forbidden not in source


def test_outer_rviz_request_survives_inner_bringup_arguments(monkeypatch):
    module = _module()
    monkeypatch.setattr(module, "_build_moveit_config", lambda _, _profile=None: _FakeMoveItConfig())
    context = _context(enable_execution=False)
    context.launch_configurations["use_rviz"] = "true"
    actions = module._launch_setup(context)
    rviz = next(action for action in actions if isinstance(action, Node) and action.node_package == "rviz2")
    # IncludeLaunchDescription writes its use_rviz=false launch argument later.
    context.launch_configurations["use_rviz"] = "false"
    assert rviz.condition.evaluate(context)


def test_hardware_limits_only_narrow_the_commissioning_planner():
    import copy
    import yaml
    module = _module()
    profile = yaml.safe_load((PACKAGE_ROOT.parents[1] / "config/hardware/firefly_y6.meow_mit.example.yaml").read_text())
    config = module._build_moveit_config(True)
    original = copy.deepcopy(config.joint_limits["robot_description_planning"]["joint_limits"])
    profile["joints"][0]["limits"].update(position_lower_rad=-0.01, position_upper_rad=0.01, velocity_rad_s=0.04)
    module._restrict_planning_limits(config, profile)
    result = config.joint_limits["robot_description_planning"]["joint_limits"]
    assert result["joint_1"]["min_position"] == -0.01
    assert result["joint_1"]["max_position"] == 0.01
    assert result["joint_1"]["max_velocity"] == 0.04
    for name, limits in result.items():
        assert limits["min_position"] >= original[name]["min_position"]
        assert limits["max_position"] <= original[name]["max_position"]
        assert limits["max_velocity"] <= original[name]["max_velocity"]
        assert limits["max_acceleration"] <= original[name]["max_acceleration"]


def test_disjoint_hardware_planning_windows_are_rejected():
    import yaml
    import pytest
    module = _module()
    profile = yaml.safe_load((PACKAGE_ROOT.parents[1] / "config/hardware/firefly_y6.meow_mit.example.yaml").read_text())
    profile["joints"][0]["limits"].update(position_lower_rad=1.0, position_upper_rad=2.0)
    with pytest.raises(RuntimeError, match="intersection"):
        module._restrict_planning_limits(module._build_moveit_config(True), profile)
