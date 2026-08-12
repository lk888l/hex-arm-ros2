from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE_ROOT / "config"
JOINTS = [f"joint_{index}" for index in range(1, 7)]


def _yaml(name: str):
    return yaml.safe_load((CONFIG / name).read_text(encoding="utf-8"))


def test_srdf_keeps_non_adjacent_self_collisions_enabled() -> None:
    root = ET.parse(CONFIG / "firefly_y6.srdf").getroot()
    group = root.find("./group[@name='arm']/chain")
    assert group is not None
    assert group.attrib == {"base_link": "base_link", "tip_link": "link_6"}
    assert root.find("./virtual_joint[@parent_frame='world'][@child_link='base_link']") is not None

    disabled = {
        (item.attrib["link1"], item.attrib["link2"])
        for item in root.findall("disable_collisions")
    }
    assert disabled == {
        ("base_link", "link_1"),
        ("link_1", "link_2"),
        ("link_2", "link_3"),
        ("link_3", "link_4"),
        ("link_4", "link_5"),
        ("link_5", "link_6"),
    }


def test_velocity_profiles_match_the_staged_policy() -> None:
    expected = {
        "joint_limits_sim.yaml": (6.0, 10.0),
        "joint_limits_commissioning.yaml": (0.2, 0.2),
        "joint_limits_verified.yaml": (6.0, 0.2),
    }
    for filename, (maximum, acceleration) in expected.items():
        limits = _yaml(filename)["joint_limits"]
        assert list(limits) == JOINTS
        assert all(item["has_velocity_limits"] for item in limits.values())
        assert all(item["max_velocity"] == maximum for item in limits.values())
        assert all(item["has_acceleration_limits"] for item in limits.values())
        assert all(item["max_acceleration"] == acceleration for item in limits.values())


def test_moveit_controller_matches_ros2_control_action() -> None:
    config = _yaml("moveit_controllers.yaml")
    manager = config["moveit_simple_controller_manager"]
    assert manager["controller_names"] == ["firefly_arm_controller"]
    controller = manager["firefly_arm_controller"]
    assert controller["type"] == "FollowJointTrajectory"
    assert controller["action_ns"] == "follow_joint_trajectory"
    assert controller["joints"] == JOINTS


def test_rviz_uses_motion_planning_as_a_display_only() -> None:
    config = _yaml("moveit.rviz")
    panel_classes = {panel["Class"] for panel in config["Panels"]}
    display_classes = {
        display["Class"] for display in config["Visualization Manager"]["Displays"]
    }
    assert "moveit_rviz_plugin/MotionPlanning" not in panel_classes
    assert "moveit_rviz_plugin/MotionPlanning" in display_classes
