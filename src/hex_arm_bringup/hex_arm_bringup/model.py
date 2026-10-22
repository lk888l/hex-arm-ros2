"""Build per-instance controller and MoveIt configuration from installed data."""
from __future__ import annotations

import atexit
from copy import deepcopy
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import yaml


def _qualified(namespace: str, name: str) -> str:
    if name.startswith("/"):
        return "/" + name.strip("/")
    return "/" + "/".join(part for part in (namespace.strip("/"), name) if part)


def controller_parameters(controllers_path, *, namespace="", joint_prefix="",
                          controller_name="firefly_arm_controller",
                          controller_manager="controller_manager",
                          state_broadcaster_name="joint_state_broadcaster",
                          use_sim_time=False, overrides_path=None):
    """Return a ROS parameter-file document with explicit per-instance node keys."""
    source = yaml.safe_load(Path(controllers_path).read_text())
    def merge(target, override):
        for key, value in override.items():
            if isinstance(value, dict) and isinstance(target.get(key), dict):
                merge(target[key], value)
            else:
                target[key] = deepcopy(value)
    if overrides_path:
        merge(source, yaml.safe_load(Path(overrides_path).read_text()))
    manager = deepcopy(source["controller_manager"]["ros__parameters"])
    manager["use_sim_time"] = bool(use_sim_time)
    manager[controller_name] = manager.pop("firefly_arm_controller")
    manager[state_broadcaster_name] = manager.pop("joint_state_broadcaster")
    controller = deepcopy(source["firefly_arm_controller"]["ros__parameters"])
    joints = controller["joints"]
    controller["joints"] = [joint_prefix + joint for joint in joints]
    controller["constraints"] = {
        joint_prefix + key if key in joints else key: value
        for key, value in controller["constraints"].items()
    }
    controller["use_sim_time"] = bool(use_sim_time)
    # A shared manager may host several arms. Each broadcaster is restricted to
    # its own joints; sensor consumers can deliberately aggregate those streams.
    # ROS's C YAML parser rejects YAML aliases; do not share list identities.
    broadcaster = {"joints": list(controller["joints"]), "use_sim_time": bool(use_sim_time)}
    manager_path = _qualified(namespace, controller_manager)
    controller_namespace = manager_path.rsplit("/", 1)[0]
    return {
        manager_path: {"ros__parameters": manager},
        _qualified(controller_namespace, controller_name): {"ros__parameters": controller},
        _qualified(controller_namespace, state_broadcaster_name): {"ros__parameters": broadcaster},
    }


def write_controller_config(controllers_path, *, directory=None, **kwargs) -> str:
    """Write instance-specific parameters for controller_manager and spawners.

    The file lives as long as the launch process, including delayed spawners.
    No shared source or installed configuration is modified.
    """
    if directory:
        Path(directory).mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(mode="w", prefix="hex_arm_controllers_", suffix=".yaml", dir=directory, delete=False) as stream:
        yaml.safe_dump(controller_parameters(controllers_path, **kwargs), stream)
        path = Path(stream.name)
    atexit.register(path.unlink, missing_ok=True)
    return str(path)


def prefix_moveit_config(config, prefix="", controller_name="firefly_arm_controller",
                         planning_group="arm", tip_link=None):
    """Adapt a MoveItConfigs object whose URDF was already expanded with prefix.

    Group names stay ``arm`` unless explicitly overridden. The optional tip_link
    is the arm-relative name of an existing TCP frame. Controller action names
    remain relative to the MoveIt node namespace.
    """
    semantic = ET.fromstring(config.robot_description_semantic["robot_description_semantic"])
    joints = {f"joint_{index}" for index in range(1, 7)}
    links = {"base_link"} | {f"link_{index}" for index in range(1, 7)}
    for element in semantic.iter():
        for key, value in list(element.attrib.items()):
            if key in ("link", "link1", "link2", "base_link", "tip_link", "child_link") and value in links:
                element.set(key, prefix + value)
            elif key == "name" and element.tag in ("joint", "virtual_joint"):
                element.set(key, prefix + value)
            elif (key == "group" or (key == "name" and element.tag == "group")) and value == "arm":
                element.set(key, planning_group)
        if element.tag == "chain" and tip_link:
            element.set("tip_link", prefix + tip_link)
    config.robot_description_semantic["robot_description_semantic"] = ET.tostring(semantic, encoding="unicode")

    def transform(value):
        if isinstance(value, dict):
            return {
                prefix + key if key in joints else planning_group if key == "arm" else key: transform(item)
                for key, item in value.items()
            }
        if isinstance(value, list):
            return [transform(item) for item in value]
        return prefix + value if isinstance(value, str) and value in joints else value

    config.joint_limits = transform(config.joint_limits)
    config.robot_description_kinematics = transform(config.robot_description_kinematics)
    config.planning_pipelines = transform(config.planning_pipelines)
    config.trajectory_execution = transform(config.trajectory_execution)
    manager = config.trajectory_execution.get("moveit_simple_controller_manager")
    if manager and "firefly_arm_controller" in manager:
        manager[controller_name] = manager.pop("firefly_arm_controller")
        manager["controller_names"] = [controller_name]
    return config


def write_rviz_config(source, *, namespace="", joint_prefix="", planning_group="arm", use_sim_time=False,
                      robot_description_topic=""):
    """Point RViz's saved displays and MoveIt panel at the selected instance."""
    config = yaml.safe_load(Path(source).read_text())
    manager = config["Visualization Manager"]
    if manager["Global Options"].get("Fixed Frame") == "base_link":
        manager["Global Options"]["Fixed Frame"] = joint_prefix + "base_link"
    for display in manager.get("Displays", []):
        if display.get("Class") == "rviz_default_plugins/RobotModel":
            display["Description Topic"] = {"Value": robot_description_topic or _qualified(namespace, "robot_description")}
        if display.get("Class") == "moveit_rviz_plugin/MotionPlanning":
            display["Move Group Namespace"] = namespace
            display["Planning Request"]["Planning Group"] = planning_group
            display["Planning Scene Topic"] = _qualified(namespace, "monitored_planning_scene")
            display["Planned Path"]["Trajectory Topic"] = _qualified(namespace, "display_planned_path")
            display["Planned Path"]["Use Sim Time"] = bool(use_sim_time)
    with tempfile.NamedTemporaryFile(mode="w", prefix="hex_arm_rviz_", suffix=".rviz", delete=False) as stream:
        yaml.safe_dump(config, stream)
        path = Path(stream.name)
    atexit.register(path.unlink, missing_ok=True)
    return str(path)
