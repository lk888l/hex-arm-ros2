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
                "schema_version: 2",
                "validated: true",
                f"calibrated: {'true' if calibrated else 'false'}",
                "robot_prefix: test/firefly_y6",
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


def test_real_launch_rejects_v1_and_missing_gravity_installation_data(
    tmp_path: Path,
) -> None:
    module = _load_launch_module()
    profile = _profile(tmp_path)
    contents = profile.read_text(encoding="utf-8")

    profile.write_text(contents.replace("schema_version: 2", "schema_version: 1"), encoding="utf-8")
    try:
        module._real_nodes(_context(profile, "false"))
    except RuntimeError as error:
        assert "schema v2" in str(error)
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
        raise AssertionError("schema v2 profile without gravity vector was not rejected")


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
