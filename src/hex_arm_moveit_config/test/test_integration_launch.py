"""Execution gates and ownership boundaries when embedding this arm."""
import hashlib
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.events import Shutdown
from launch_ros.actions import Node

from hex_arm_bringup.integration import InstanceNames
from hex_arm_bringup.launch_support import ArmFault
from hex_arm_bringup.startup_event import StartupVerified


def load(package, filename):
    path = Path(get_package_share_directory(package)) / "launch" / filename
    spec = importlib.util.spec_from_file_location(filename, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def context(**overrides):
    ctx = LaunchContext()
    ctx.launch_configurations.update(dict(
        hardware_profile="/tmp/unused-profile.yaml", zenoh_connect="", enable_execution="true",
        startup_ready="true", publish_world_tf="false", use_rviz="false", shutdown_scope="arm"))
    ctx.launch_configurations.update(overrides)
    return ctx


def fake_config():
    return SimpleNamespace(
        joint_limits={"robot_description_planning": {"joint_limits": {
            f"joint_{i}": {"max_velocity": .1, "max_acceleration": .1} for i in range(1, 7)}}},
        robot_description={}, robot_description_semantic={}, robot_description_kinematics={},
        planning_pipelines={}, to_dict=lambda: {})


def test_fault_is_scoped_to_the_selected_arm():
    module = load("hex_arm_moveit_config", "moveit_real.launch.py")
    actions = module._shutdown_after_exit("move_group", "/left", "arm")(
        SimpleNamespace(returncode=-11), LaunchContext())
    event = actions[-1].event
    assert isinstance(event, ArmFault)
    assert not isinstance(event, Shutdown)
    assert event.namespace == "/left"
    assert "-11" in event.reason


def test_standard_moveit_execution_starts_only_after_matching_startup(monkeypatch):
    module = load("hex_arm_moveit_config", "moveit_real.launch.py")
    monkeypatch.setattr(module, "_build_moveit_config", lambda *args: fake_config())
    monkeypatch.setattr(module, "_move_group_environment", lambda: pytest.fail("custom runtime used"))
    actions = module._launch_setup(context(use_custom_runtime="false"))
    assert all(node.node_package != "moveit_ros_move_group" for node in actions if isinstance(node, Node))
    handlers = [item.event_handler for item in actions if isinstance(item, RegisterEventHandler)]
    event = StartupVerified("/tmp/unused-profile.yaml", "", "")
    ready = [handler for handler in handlers if handler.matches(event)]
    assert len(ready) == 1
    started = ready[0].handle(event, LaunchContext())
    assert any(isinstance(node, Node) and node.node_package == "moveit_ros_move_group" for node in started)
    assert not ready[0].matches(StartupVerified("/tmp/unused-profile.yaml", "", "/other"))
    include = next(item for item in actions if isinstance(item, IncludeLaunchDescription))
    assert dict(include.launch_arguments).get("moveit_ready_token", "") == ""


def test_external_executing_moveit_requires_a_matching_gate_token(monkeypatch):
    module = load("hex_arm_moveit_config", "moveit_real.launch.py")
    monkeypatch.setattr(module, "_build_moveit_config", lambda *args: fake_config())
    with pytest.raises(ValueError, match="readiness token"):
        module._launch_setup(context(start_move_group="false"))
    actions = module._launch_setup(context(start_move_group="false", external_readiness_token="a" * 32))
    assert all(node.node_package != "hex_arm_moveit_runtime" for node in actions if isinstance(node, Node))
    include = next(item for item in actions if isinstance(item, IncludeLaunchDescription))
    assert dict(include.launch_arguments)["moveit_ready_token"] == "a" * 32


def test_startup_report_from_other_instance_cannot_unlock(tmp_path):
    module = load("hex_arm_bringup", "real.launch.py")
    profile = tmp_path / "profile.yaml"
    profile.write_text("hardware: same\n")
    report = tmp_path / "startup.json"
    report.write_text(json.dumps({"passed": True, "moveit_execution_unlocked": True,
        "profile_sha256": hashlib.sha256(profile.read_bytes()).hexdigest(),
        "instance": InstanceNames(namespace="right").as_dict()}))
    actions = module._after_startup(report, profile, "a" * 32,
        InstanceNames(namespace="left"), "arm")(SimpleNamespace(returncode=0), LaunchContext())
    assert isinstance(actions[-1].event, ArmFault)
    assert actions[-1].event.namespace == "/left"


def test_external_manager_and_model_are_not_started_again(tmp_path):
    module = load("hex_arm_bringup", "real.launch.py")
    profile = tmp_path / "profile.yaml"
    profile.write_text("schema_version: 3\njoint_coordinate_version: 2\nvalidated: true\n"
        "calibrated: true\nrobot_prefix: test/left\ngravity_vector_base_m_s2: [0,0,-9.81]\n"
        "joints:\n" + "".join(f"- {{name: joint_{i}}}\n" for i in range(1, 7)))
    ctx = context(hardware_profile=str(profile), activate_hardware="false",
        hardware_startup_timeout_sec="30", start_controller_manager="false",
        controller_update_rate="100",
        start_driver="false", publish_robot_description="false", use_operation_server="false",
        namespace="/left", joint_prefix="left_", hardware_name="LeftArm",
        controller_manager="/controller_manager", controller_name="left_controller")
    actions = module._real_nodes(ctx)
    packages = [node.node_package for node in actions if isinstance(node, Node)]
    assert "controller_manager" not in packages
    assert "robot_state_publisher" not in packages
