from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
from types import SimpleNamespace
import pytest
import copy
import json
import yaml

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
    joint_limits = {"robot_description_planning": {"joint_limits": {
        f"joint_{index}": {"max_velocity": 1.2566370614359172, "max_acceleration": .6}
        for index in range(1, 7)
    }}}

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
        "position_limits",
        "dynamics_limits",
        "planning_limits_file",
        "startup_ready",
        "startup_sequence",
        "startup_trial",
        "align_folded",
        "allow_enable_transient",
        "publish_world_tf",
        "use_rviz",
    }
    context = LaunchContext()
    assert perform_substitutions(
        context, declarations["enable_execution"].default_value
    ) == "false"
    assert declarations["enable_execution"].choices == ["true", "false"]
    assert perform_substitutions(context, declarations["dynamics_limits"].default_value) == "commissioning"
    assert declarations["dynamics_limits"].choices == ["commissioning", "hardware", "custom"]
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
            with pytest.raises(RuntimeError, match="protocol startup procedure"):
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

    def _fake_config(enable_execution: bool, hardware_profile=None, position_limits="commissioning",
                     dynamics_limits="commissioning", planning_limits_file=""):
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

    def _fake_config(enable_execution: bool, hardware_profile=None, position_limits="commissioning",
                     dynamics_limits="commissioning", planning_limits_file=""):
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
        module, "_build_moveit_config", lambda *args: _FakeMoveItConfig()
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


def test_real_launch_selects_dynamics_and_keeps_the_authoritative_state_topic() -> None:
    source = LAUNCH_FILE.read_text(encoding="utf-8")
    assert '.joint_limits(file_path=dynamics_file)' in source
    assert '"hardware": "config/joint_limits_hardware.yaml"' in source
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
    monkeypatch.setattr(module, "_build_moveit_config", lambda *args: _FakeMoveItConfig())
    context = _context(enable_execution=False)
    context.launch_configurations["use_rviz"] = "true"
    actions = module._launch_setup(context)
    rviz = next(action for action in actions if isinstance(action, Node) and action.node_package == "rviz2")
    # IncludeLaunchDescription writes its use_rviz=false launch argument later.
    context.launch_configurations["use_rviz"] = "false"
    assert rviz.condition.evaluate(context)


def test_executing_launch_delays_rviz_until_its_verified_startup_event(monkeypatch):
    module = _module()
    token = "a" * 32
    monkeypatch.setattr(module, "_build_moveit_config", lambda *args: _FakeMoveItConfig())
    monkeypatch.setattr(module, "get_package_share_directory", lambda _package: "/tmp")
    monkeypatch.setattr(module, "_move_group_environment", lambda: {})
    monkeypatch.setattr(module.uuid, "uuid4", lambda: SimpleNamespace(hex=token))
    context = _context(enable_execution=True)
    context.launch_configurations["use_rviz"] = "true"
    actions = module._launch_setup(context)
    assert not any(isinstance(action, Node) and action.node_package == "rviz2" for action in actions)
    includes = [action for action in actions if isinstance(action, IncludeLaunchDescription)]
    assert dict(includes[0].launch_arguments)["moveit_ready_token"] == token
    inherited_rates = json.loads(dict(includes[0].launch_arguments)["startup_motion_limits"])
    assert inherited_rates == {
        f"joint_{index}": {"velocity_rad_s": 1.2566370614359172, "acceleration_rad_s2": .6}
        for index in range(1, 7)
    }
    handlers = [action.event_handler for action in actions if isinstance(action, RegisterEventHandler)]
    event = module.StartupVerified("/tmp/validated.local.yaml", token)
    handler = next(handler for handler in handlers if handler.matches(event))
    assert not handler.matches(module.StartupVerified("/tmp/other.yaml", token))
    assert not handler.matches(module.StartupVerified("/tmp/validated.local.yaml", "b" * 32))
    assert not handler.matches(module.Shutdown(reason="startup failed"))
    rviz = next(action for action in handler.entities if isinstance(action, Node))
    context.launch_configurations["use_rviz"] = "false"
    assert rviz.node_package == "rviz2" and rviz.condition.evaluate(context)


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


def test_hardware_position_window_retains_urdf_and_slow_dynamics():
    import yaml
    module = _module()
    profile = yaml.safe_load((PACKAGE_ROOT.parents[1] / "config/hardware/firefly_y6.example.yaml").read_text())
    profile["joints"][5]["limits"].update(position_lower_rad=2.06, position_upper_rad=2.16)
    profile["joints"][0]["limits"].update(position_lower_rad=-10.0, position_upper_rad=10.0,
                                          velocity_rad_s=3.0, acceleration_rad_s2=4.0)
    config = module._build_moveit_config(True)
    module._restrict_planning_limits(config, profile, "hardware")
    limits = config.joint_limits["robot_description_planning"]["joint_limits"]
    assert limits["joint_6"]["min_position"] == 2.06
    assert limits["joint_6"]["max_position"] == 2.16
    assert limits["joint_1"]["min_position"] == -2.86
    assert limits["joint_1"]["max_position"] == 2.86
    assert limits["joint_1"]["max_velocity"] == 0.1
    assert limits["joint_1"]["max_acceleration"] == 0.1


def test_hardware_position_window_requires_profile_and_known_selection():
    module = _module()
    with pytest.raises(RuntimeError, match="explicit hardware profile"):
        module._build_moveit_config(True, position_limits="hardware")
    with pytest.raises(RuntimeError, match="position_limits"):
        module._build_moveit_config(True, position_limits="unlimited")


def _deployment_profile(tmp_path):
    profile = yaml.safe_load((PACKAGE_ROOT.parents[1] / "config/hardware/firefly_y6.example.yaml").read_text())
    for joint in profile["joints"]:
        joint["limits"].update(position_lower_rad=-2.0, position_upper_rad=2.0,
                               velocity_rad_s=1.25, acceleration_rad_s2=2.0)
    path = tmp_path / "hardware.yaml"
    path.write_text(yaml.safe_dump(profile))
    return profile, path


@pytest.mark.parametrize("positions", ["commissioning", "hardware"])
def test_hardware_dynamics_have_no_hidden_commissioning_cap(tmp_path, positions):
    module = _module()
    _, path = _deployment_profile(tmp_path)
    config = module._build_moveit_config(True, str(path), positions, "hardware")
    planning = config.joint_limits["robot_description_planning"]
    assert planning["default_velocity_scaling_factor"] == 1.0
    assert planning["default_acceleration_scaling_factor"] == 1.0
    for limits in planning["joint_limits"].values():
        assert limits["max_velocity"] == 1.25
        assert limits["max_acceleration"] == 2.0
    assert planning["joint_limits"]["joint_1"]["min_position"] == (-0.25 if positions == "commissioning" else -2.0)


def test_urdf_velocity_remains_a_cap_in_hardware_mode(tmp_path):
    module = _module()
    profile, path = _deployment_profile(tmp_path)
    profile["joints"][0]["limits"]["velocity_rad_s"] = 20.0
    path.write_text(yaml.safe_dump(profile))
    config = module._build_moveit_config(True, str(path), "hardware", "hardware")
    assert config.joint_limits["robot_description_planning"]["joint_limits"]["joint_1"]["max_velocity"] == 6.0


def test_custom_dynamics_intersect_hardware_without_yaml_alias_cross_talk(tmp_path):
    module = _module()
    profile, path = _deployment_profile(tmp_path)
    profile["joints"][0]["limits"]["velocity_rad_s"] = 0.35
    path.write_text(yaml.safe_dump(profile))
    custom = tmp_path / "planning.yaml"
    custom.write_text("""default_velocity_scaling_factor: 0.8
default_acceleration_scaling_factor: 0.7
joint_limits:
  joint_1: &limits {has_velocity_limits: true, max_velocity: 0.8, has_acceleration_limits: true, max_acceleration: 3.0}
  joint_2: *limits
  joint_3: *limits
  joint_4: *limits
  joint_5: *limits
  joint_6: *limits
""")
    config = module._build_moveit_config(True, str(path), "hardware", "custom", str(custom))
    parameters = config.joint_limits["robot_description_planning"]
    limits = parameters["joint_limits"]
    assert parameters["default_velocity_scaling_factor"] == 0.8
    assert limits["joint_1"]["max_velocity"] == 0.35
    assert limits["joint_2"]["max_velocity"] == 0.8
    assert limits["joint_2"]["max_acceleration"] == 2.0
    assert limits["joint_1"]["max_position"] == 2.0
    assert limits["joint_2"]["max_position"] == 2.0
    assert limits["joint_3"]["max_position"] == 1.57
    assert len({id(value) for value in limits.values()}) == 6


@pytest.mark.parametrize("value", [float("nan"), float("inf"), 0.0, -1.0, True])
@pytest.mark.parametrize("field", ["velocity_rad_s", "acceleration_rad_s2"])
def test_invalid_hardware_dynamics_are_rejected(tmp_path, field, value):
    module = _module()
    profile, path = _deployment_profile(tmp_path)
    profile["joints"][0]["limits"][field] = value
    path.write_text(yaml.safe_dump(profile))
    with pytest.raises(RuntimeError, match="hardware"):
        module._build_moveit_config(True, str(path), "hardware", "hardware")


@pytest.mark.parametrize("value", [float("nan"), float("inf"), 0.0, -1.0, True])
def test_invalid_custom_caps_cannot_be_hidden_by_minimum_merge(tmp_path, value):
    module = _module()
    _, path = _deployment_profile(tmp_path)
    custom = tmp_path / "custom.yaml"
    content = yaml.safe_load((PACKAGE_ROOT / "config/joint_limits_verified.yaml").read_text())
    content["joint_limits"]["joint_1"]["max_velocity"] = value
    custom.write_text(yaml.safe_dump(content))
    with pytest.raises(RuntimeError, match="planning max_velocity"):
        module._build_moveit_config(True, str(path), "hardware", "custom", str(custom))


def test_custom_dynamics_require_a_profile_complete_caps_and_valid_scaling(tmp_path):
    module = _module()
    _, path = _deployment_profile(tmp_path)
    custom = tmp_path / "planning.yaml"
    original = yaml.safe_load((PACKAGE_ROOT / "config/joint_limits_commissioning.yaml").read_text())
    for fault in ("missing_joint", "disabled_velocity", "disabled_acceleration", "scaling", "missing_cap"):
        content = copy.deepcopy(original)
        if fault == "missing_joint": content["joint_limits"].pop("joint_6")
        if fault == "disabled_velocity": content["joint_limits"]["joint_1"]["has_velocity_limits"] = False
        if fault == "disabled_acceleration": content["joint_limits"]["joint_1"]["has_acceleration_limits"] = False
        if fault == "scaling": content["default_acceleration_scaling_factor"] = 1.1
        if fault == "missing_cap": content["joint_limits"]["joint_1"].pop("max_acceleration")
        custom.write_text(yaml.safe_dump(content))
        with pytest.raises(RuntimeError):
            module._build_moveit_config(True, str(path), "hardware", "custom", str(custom))
    with pytest.raises(RuntimeError, match="explicit hardware profile"):
        module._build_moveit_config(True, dynamics_limits="hardware")
    for selector, filename in (("custom", ""), ("custom", "relative.yaml"), ("unknown", ""), ("hardware", str(custom))):
        with pytest.raises(RuntimeError):
            module._dynamics_file(selector, filename)


def test_launch_forwards_the_selected_dynamic_configuration(monkeypatch):
    module = _module()
    selected = []
    monkeypatch.setattr(module, "_build_moveit_config", lambda *args: selected.append(args) or _FakeMoveItConfig())
    monkeypatch.setattr(module, "_move_group_environment", lambda: {})
    monkeypatch.setattr(module, "get_package_share_directory", lambda _: "/tmp")
    context = _context(True)
    context.launch_configurations.update(position_limits="hardware", dynamics_limits="custom",
                                         planning_limits_file="/tmp/dynamics.yaml")
    module._launch_setup(context)
    assert selected == [(True, "/tmp/validated.local.yaml", "hardware", "custom", "/tmp/dynamics.yaml")]


def test_arm_bound_startup_recipe_is_forwarded_without_widening_execution_authority():
    module = _module()
    for enabled in (False, True):
        args = module._bringup_arguments('/arm.yaml', '', enabled, True, '/arm-sequence.yaml')
        assert args['startup_sequence'] == '/arm-sequence.yaml'
        assert args['activate_hardware'] == str(enabled).lower()
    with pytest.raises(RuntimeError, match='explicit execution'):
        module._bringup_arguments('/arm.yaml', '', False, True, '/arm-sequence.yaml', True)
    with pytest.raises(RuntimeError, match='arm-bound sequence'):
        module._bringup_arguments('/arm.yaml', '', True, True, '', True)
    args = module._bringup_arguments('/arm.yaml', '', True, True, '/arm-sequence.yaml', True)
    assert args['startup_trial'] == args['activate_hardware'] == 'true'
