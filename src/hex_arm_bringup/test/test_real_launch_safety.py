from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Iterator

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_ROOT / "launch" / "real.launch.py"


def _load_launch_module():
    spec = importlib.util.spec_from_file_location("hex_arm_real_launch", LAUNCH_FILE)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _profile(tmp_path: Path, *, calibrated: bool = True) -> Path:
    profile = tmp_path / "verified.local.yaml"
    profile.write_text(
        "\n".join(
            [
                "schema_version: 3",
                "joint_coordinate_version: 2",
                "validated: true",
                f"calibrated: {'true' if calibrated else 'false'}",
                "robot_prefix: test/firefly_y6",
                "bus: {protocol: meow}",
                "gravity_vector_base_m_s2: [0.0, 0.0, -9.81]",
                "joints:",
                *[f"  - {{name: joint_{index}}}" for index in range(1, 7)],
            ]
        ),
        encoding="utf-8",
    )
    return profile


def _context(
    profile: Path,
    activate_hardware: str,
    zenoh_connect: str = "",
    bridge_startup_timeout_sec: str = "30.0",
) -> LaunchContext:
    context = LaunchContext()
    context.launch_configurations.update(
        {
            "hardware_profile": str(profile),
            "zenoh_connect": zenoh_connect,
            "bridge_startup_timeout_sec": bridge_startup_timeout_sec,
            "activate_hardware": activate_hardware,
            "use_rviz": "false",
        }
    )
    return context


def _walk_nodes(value: Any, seen: set[int] | None = None) -> Iterator[Node]:
    if seen is None:
        seen = set()
    identity = id(value)
    if identity in seen:
        return
    seen.add(identity)

    if isinstance(value, Node):
        yield value
        return
    if isinstance(value, dict):
        for item in value.values():
            yield from _walk_nodes(item, seen)
        return
    if isinstance(value, (list, tuple, set)):
        for item in value:
            yield from _walk_nodes(item, seen)
        return
    if hasattr(value, "__dict__"):
        yield from _walk_nodes(vars(value), seen)


def test_real_launch_defaults_to_observation_only() -> None:
    module = _load_launch_module()
    description = module.generate_launch_description()
    arguments = {
        action.name: action
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
    }
    activation = arguments["activate_hardware"]
    assert perform_substitutions(LaunchContext(), activation.default_value) == "false"
    assert perform_substitutions(LaunchContext(), arguments["startup_ready"].default_value) == "true"
    assert set(activation.choices) == {"true", "false"}


def test_clean_critical_exit_is_not_mislabeled_as_an_error() -> None:
    module = _load_launch_module()
    context = LaunchContext()

    clean_actions = module._shutdown_after_exit("hex_arm_controller")(
        SimpleNamespace(returncode=0), context
    )
    assert len(clean_actions) == 2
    assert isinstance(clean_actions[0], LogInfo)
    assert isinstance(clean_actions[1], EmitEvent)
    clean_message = perform_substitutions(context, clean_actions[0].msg)
    assert "exited cleanly" in clean_message
    assert "ERROR" not in clean_message
    assert clean_actions[1].event.reason == "critical process hex_arm_controller exited cleanly"

    failed_actions = module._shutdown_after_exit("hex_arm_controller")(
        SimpleNamespace(returncode=7), context
    )
    failed_message = perform_substitutions(context, failed_actions[0].msg)
    assert failed_message.startswith("ERROR:")
    assert "exit" in failed_actions[1].event.reason


def test_observe_mode_contains_no_controller_manager_process(tmp_path: Path) -> None:
    module = _load_launch_module()
    context = _context(_profile(tmp_path), "false")
    actions = module._real_nodes(context)
    nodes = list(_walk_nodes(actions))

    assert "controller_manager" not in {node.node_package for node in nodes}
    assert {"hex_arm_bridge", "robot_state_publisher"} <= {
        node.node_package for node in nodes
    }
    controller_processes = [
        action
        for action in actions
        if isinstance(action, ExecuteProcess) and not isinstance(action, Node)
    ]
    assert len(controller_processes) == 1
    controller_cmd = " ".join(
        perform_substitutions(context, token) for token in controller_processes[0].cmd
    )
    assert "/lib/hex_arm_controller/hex_arm_controller" in controller_cmd
    assert "--ros-args" not in controller_cmd
    assert "--zenoh-listen tcp/127.0.0.1:7448" in controller_cmd
    assert "--zenoh-connect" not in controller_cmd

    state_publisher = next(
        node for node in nodes if node.node_package == "robot_state_publisher"
    )
    state_publisher._perform_substitutions(context)
    assert ("joint_states", "/hex_arm/internal/state") in state_publisher.expanded_remapping_rules


def test_explicit_zenoh_route_connects_both_processes_without_scouting(
    tmp_path: Path,
) -> None:
    module = _load_launch_module()
    endpoint = "tcp/router.example:7447"
    context = _context(_profile(tmp_path), "false", endpoint)
    actions = module._real_nodes(context)
    controller = next(
        action
        for action in actions
        if isinstance(action, ExecuteProcess) and not isinstance(action, Node)
    )
    controller_cmd = " ".join(
        perform_substitutions(context, token) for token in controller.cmd
    )

    assert f"--zenoh-connect {endpoint}" in controller_cmd
    assert "--zenoh-listen" not in controller_cmd
    assert module._zenoh_routes(endpoint) == (["--zenoh-connect", endpoint], endpoint)


def test_empty_zenoh_route_is_a_self_contained_loopback_pair() -> None:
    module = _load_launch_module()

    assert module._zenoh_routes("") == (
        ["--zenoh-listen", "tcp/127.0.0.1:7448"],
        "tcp/127.0.0.1:7448",
    )


def test_real_bridge_startup_budget_covers_six_axis_initialization() -> None:
    module = _load_launch_module()
    description = module.generate_launch_description()
    arguments = {
        action.name: action
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
    }

    timeout_argument = arguments["bridge_startup_timeout_sec"]
    timeout = float(
        perform_substitutions(LaunchContext(), timeout_argument.default_value)
    )
    assert timeout == module.DEFAULT_BRIDGE_STARTUP_TIMEOUT_SEC == 30.0
    assert module._bridge_parameters(
        {"robot_prefix": "test/firefly_y6"},
        module.DEFAULT_ZENOH_DIRECT_ENDPOINT,
        timeout,
    )["startup_timeout_sec"] == 30.0

    for invalid in ("0", "-1", "nan", "inf", "not-a-number"):
        try:
            module._positive_seconds(invalid, "bridge_startup_timeout_sec")
        except RuntimeError as error:
            assert "finite positive" in str(error)
        else:
            raise AssertionError(f"invalid startup timeout {invalid!r} was accepted")


def test_uncalibrated_profile_is_observable_but_cannot_activate(tmp_path: Path) -> None:
    module = _load_launch_module()
    profile = _profile(tmp_path, calibrated=False)

    actions = module._real_nodes(_context(profile, "false"))
    assert "controller_manager" not in {
        node.node_package for node in _walk_nodes(actions)
    }

    try:
        module._real_nodes(_context(profile, "true"))
    except RuntimeError as error:
        assert "not marked calibrated" in str(error)
    else:
        raise AssertionError("uncalibrated hardware activation was not rejected")


def test_real_execution_cannot_skip_ordered_startup(tmp_path: Path) -> None:
    import pytest
    module = _load_launch_module()
    context = _context(_profile(tmp_path), "true")
    context.launch_configurations["startup_ready"] = "false"
    with pytest.raises(RuntimeError, match="protocol startup procedure"):
        module._real_nodes(context)


def test_real_execution_selects_protocol_startup(tmp_path: Path) -> None:
    module = _load_launch_module()
    context = _context(_profile(tmp_path), "true")
    # The startup process is nested in an event handler; inspect the source
    # alongside default/disable-path tests without running a ROS process.
    module._real_nodes(context)
    source = LAUNCH_FILE.read_text()
    assert '"--allow-motion", "--activate-controllers"' in source
    assert '"--hold-current"' in source


def test_effective_moveit_dynamics_reach_the_startup_process_as_one_json_argument(tmp_path: Path):
    import json
    module = _load_launch_module()
    context = _context(_profile(tmp_path), "true")
    rates = json.dumps({
        f"joint_{index}": {"velocity_rad_s": 1.2566370614359172, "acceleration_rad_s2": .6}
        for index in range(1, 7)
    })
    context.launch_configurations['startup_motion_limits'] = rates
    actions = module._real_nodes(context)
    processes = []
    seen = set()
    def walk(value):
        if id(value) in seen:
            return
        seen.add(id(value))
        if isinstance(value, ExecuteProcess):
            if not isinstance(value, Node):
                processes.append(value)
        elif isinstance(value, dict):
            for item in value.values(): walk(item)
        elif isinstance(value, (list, tuple)):
            for item in value: walk(item)
        elif hasattr(value, '__dict__'):
            for item in vars(value).values(): walk(item)
    walk(actions)
    commands = [[perform_substitutions(context, token) for token in process.cmd]
                for process in processes]
    startup = next(command for command in commands if any(
        token.endswith('commission-startup-ros.py') for token in command))
    assert startup[startup.index('--motion-limits') + 1] == rates


def test_real_launch_rejects_v1_and_missing_gravity_installation_data(
    tmp_path: Path,
) -> None:
    module = _load_launch_module()
    profile = _profile(tmp_path)
    contents = profile.read_text(encoding="utf-8")

    profile.write_text(contents.replace("schema_version: 3", "schema_version: 2"), encoding="utf-8")
    try:
        module._real_nodes(_context(profile, "false"))
    except RuntimeError as error:
        assert "schema v3" in str(error)
    else:
        raise AssertionError("schema v1 real profile was not rejected")

    profile.write_text(
        "\n".join(
            line
            for line in contents.splitlines()
            if not line.startswith("gravity_vector_base_m_s2:")
        ),
        encoding="utf-8",
    )
    try:
        module._real_nodes(_context(profile, "false"))
    except RuntimeError as error:
        assert "gravity_vector_base_m_s2" in str(error)
    else:
        raise AssertionError("schema v3 profile without gravity vector was not rejected")


def test_controller_manager_chain_requires_explicit_opt_in(tmp_path: Path) -> None:
    module = _load_launch_module()
    context = _context(_profile(tmp_path), "true")
    actions = module._real_nodes(context)
    controller_manager_executables = {
        node.node_executable
        for node in _walk_nodes(actions)
        if node.node_package == "controller_manager"
    }

    assert controller_manager_executables == {"ros2_control_node"}


def test_fixed_startup_requires_explicit_motion_acknowledgement(tmp_path: Path) -> None:
    import pytest

    path = PACKAGE_ROOT / "launch" / "startup.launch.py"
    spec = importlib.util.spec_from_file_location("hex_arm_startup_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    arguments = {
        action.name: action
        for action in module.generate_launch_description().entities
        if isinstance(action, DeclareLaunchArgument)
    }
    context = LaunchContext()
    default = perform_substitutions(
        context, arguments["allow_startup_motion"].default_value
    )
    assert default == "false"
    context.launch_configurations.update({
        "allow_startup_motion": default,
        "hardware_profile": str(_profile(tmp_path, calibrated=False)),
    })
    with pytest.raises(RuntimeError, match="requires allow_startup_motion"):
        module._setup(context)

    context.launch_configurations["allow_startup_motion"] = "true"
    actions = module._setup(context)
    processes = [action for action in actions if isinstance(action, ExecuteProcess)]
    assert len(processes) == 1
    assert not list(_walk_nodes(actions))
    command = [
        perform_substitutions(context, token) for token in processes[0].cmd
    ]
    assert command[-2:] == ["--startup-sequence", "--allow-startup-motion"]


def test_startup_failure_stops_owner_but_success_keeps_holding():
    module = _load_launch_module()
    context = LaunchContext()
    handler = module._shutdown_after_failure("ordered startup_ready")
    assert handler(SimpleNamespace(returncode=0), context) == []
    assert any(isinstance(a, EmitEvent) for a in handler(SimpleNamespace(returncode=1), context))


def test_startup_event_requires_a_successful_profile_bound_report(tmp_path):
    import hashlib
    import json
    module = _load_launch_module()
    context = LaunchContext()
    profile = _profile(tmp_path)
    report_path = tmp_path / "startup.json"
    report = {"passed": True, "profile_sha256": hashlib.sha256(profile.read_bytes()).hexdigest(),
              "moveit_execution_unlocked": True}
    token = "a" * 32
    handler = module._after_startup(report_path, profile, token)
    report_path.write_text(json.dumps(report))
    success = handler(SimpleNamespace(returncode=0), context)
    assert len(success) == 1 and isinstance(success[0].event, module.StartupVerified)
    assert success[0].event.profile == str(profile) and success[0].event.readiness_token == token
    for changes in ({"passed": False}, {"profile_sha256": "stale"}, {"moveit_execution_unlocked": False}):
        report_path.write_text(json.dumps({**report, **changes}))
        failed = handler(SimpleNamespace(returncode=0), context)
        assert any(isinstance(action, EmitEvent) and isinstance(action.event, module.Shutdown)
                   for action in failed)
    report_path.write_text(json.dumps({**report, "deactivated": True}))
    assert not any(isinstance(action, EmitEvent) for action in handler(SimpleNamespace(returncode=0), context))
    report_path.unlink()
    assert any(isinstance(action, EmitEvent) and isinstance(action.event, module.Shutdown)
               for action in handler(SimpleNamespace(returncode=0), context))
    assert any(isinstance(action, EmitEvent) and isinstance(action.event, module.Shutdown)
               for action in handler(SimpleNamespace(returncode=1), context))


def test_bridge_command_and_snapshot_period_follow_controller_rate(tmp_path):
    module = _load_launch_module()
    config = tmp_path / "controllers.yaml"
    config.write_text("controller_manager: {ros__parameters: {update_rate: 250}}\n")
    rate = module._controller_update_rate(config)
    parameters = module._bridge_parameters({"robot_prefix": "test/arm"}, "tcp/localhost:7448", 30.0, rate)
    assert parameters["command_period_sec"] == 0.004
    assert parameters["stream_period_sec"] == parameters["command_period_sec"]
    for invalid in ("0", "-1", "true", "2.5", "null"):
        config.write_text("controller_manager: {ros__parameters: {update_rate: " + invalid + "}}\n")
        try:
            module._controller_update_rate(config)
        except RuntimeError:
            pass
        else:
            raise AssertionError(f"invalid controller rate was accepted: {invalid}")
