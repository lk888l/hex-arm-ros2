from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE_ROOT / "config"
JOINTS = [f"joint_{index}" for index in range(1, 7)]
LINKS = ["base_link", *(f"link_{index}" for index in range(1, 7))]
ADJACENT = {
    ("base_link", "link_1"),
    ("link_1", "link_2"),
    ("link_2", "link_3"),
    ("link_3", "link_4"),
    ("link_4", "link_5"),
    ("link_5", "link_6"),
}
PLAN_ONLY_SURVEYED_FOLD = {
    ("link_1", "link_5"),
    ("link_2", "link_4"),
}


def _yaml(name: str):
    return yaml.safe_load((CONFIG / name).read_text(encoding="utf-8"))


def _disabled(root: ET.Element) -> dict[tuple[str, str], str]:
    return {
        (item.attrib["link1"], item.attrib["link2"]): item.attrib["reason"]
        for item in root.findall("disable_collisions")
    }


def _semantic_xml(element: ET.Element | None) -> bytes:
    assert element is not None
    # XML indentation belongs to the surrounding document, not the semantic
    # element. Ignore only that parent-owned tail while comparing overlays.
    element.tail = None
    return ET.tostring(element)


def test_strict_srdf_disables_only_kinematically_adjacent_links() -> None:
    root = ET.parse(CONFIG / "firefly_y6.srdf").getroot()
    group = root.find("./group[@name='arm']/chain")
    assert group is not None
    assert group.attrib == {"base_link": "base_link", "tip_link": "link_6"}
    assert root.find("./virtual_joint[@parent_frame='world'][@child_link='base_link']") is not None

    disabled = _disabled(root)
    assert disabled == {pair: "Adjacent" for pair in ADJACENT}

    # The execution and mock semantic model must keep every one of the 15
    # non-adjacent link pairs under FCL collision checking.
    all_pairs = {
        (link_1, link_2)
        for index, link_1 in enumerate(LINKS)
        for link_2 in LINKS[index + 1:]
    }
    assert len(all_pairs - set(disabled)) == 15
    assert all_pairs - set(disabled) == all_pairs - ADJACENT
    assert PLAN_ONLY_SURVEYED_FOLD.isdisjoint(disabled)


def test_plan_only_srdf_is_an_explicit_two_pair_overlay() -> None:
    strict = ET.parse(CONFIG / "firefly_y6.srdf").getroot()
    plan_only_path = CONFIG / "firefly_y6.plan_only.srdf"
    plan_only = ET.parse(plan_only_path).getroot()

    assert plan_only.attrib == strict.attrib
    assert _semantic_xml(plan_only.find("virtual_joint")) == _semantic_xml(
        strict.find("virtual_joint")
    )
    assert _semantic_xml(plan_only.find("group")) == _semantic_xml(strict.find("group"))
    assert _semantic_xml(plan_only.find("group_state")) == _semantic_xml(
        strict.find("group_state")
    )
    assert _disabled(plan_only) == {
        **{pair: "Adjacent" for pair in ADJACENT},
        **{pair: "PlanOnlySurveyedFold" for pair in PLAN_ONLY_SURVEYED_FOLD},
    }

    # CMake installs the whole config directory; assert the overlay is present
    # in the package share used by actual ros2 launch resolution as well.
    installed = (
        Path(get_package_share_directory("hex_arm_moveit_config"))
        / "config"
        / plan_only_path.name
    )
    assert installed.is_file()


def test_srdf_exposes_only_an_in_bounds_commissioning_reference() -> None:
    root = ET.parse(CONFIG / "firefly_y6.srdf").getroot()
    states = root.findall("./group_state[@group='arm']")
    assert [state.attrib["name"] for state in states] == ["commissioning_start", "startup_ready"]
    assert root.find("./group_state[@name='ready']") is None

    positions = {
        joint.attrib["name"]: float(joint.attrib["value"])
        for joint in states[0].findall("joint")
    }
    assert positions == {
        "joint_1": 0.0,
        "joint_2": -1.56,
        "joint_3": 1.56,
        "joint_4": 0.0,
        "joint_5": 0.0,
        "joint_6": 0.0,
    }

    limits = _yaml("joint_limits_commissioning.yaml")["joint_limits"]
    for name, position in positions.items():
        assert limits[name]["min_position"] < position
        assert position < limits[name]["max_position"]


def test_velocity_profiles_match_the_staged_policy() -> None:
    expected = {
        "joint_limits_sim.yaml": (6.0, 10.0),
        "joint_limits_commissioning.yaml": (0.1, 0.1),
        "joint_limits_verified.yaml": (6.0, 0.2),
        "joint_limits_deployment.yaml": (1.2566370614359172, 0.6),
    }
    for filename, (maximum, acceleration) in expected.items():
        limits = _yaml(filename)["joint_limits"]
        assert list(limits) == JOINTS
        assert all(item["has_velocity_limits"] for item in limits.values())
        assert all(item["max_velocity"] == maximum for item in limits.values())
        assert all(item["has_acceleration_limits"] for item in limits.values())
        assert all(item["max_acceleration"] == acceleration for item in limits.values())


def test_real_commissioning_positions_match_the_surveyed_narrow_window() -> None:
    limits = _yaml("joint_limits_commissioning.yaml")["joint_limits"]
    expected = {
        "joint_1": (-0.25, 0.25),
        "joint_2": (-1.57, -1.30),
        "joint_3": (1.28, 1.57),
        "joint_4": (-0.35, 0.25),
        "joint_5": (-0.25, 0.25),
        "joint_6": (-0.25, 0.35),
    }
    for name, (lower, upper) in expected.items():
        assert limits[name]["has_position_limits"]
        assert limits[name]["min_position"] == lower
        assert limits[name]["max_position"] == upper


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


def test_mock_launch_uses_the_ordered_shutdown_runtime() -> None:
    launch_source = (PACKAGE_ROOT / "launch" / "moveit_mock.launch.py").read_text(
        encoding="utf-8"
    )
    assert 'package="hex_arm_moveit_runtime"' in launch_source
    assert 'executable="hex_arm_move_group"' in launch_source
    assert "additional_env=_move_group_environment()" in launch_source
    assert 'package="moveit_ros_move_group"' not in launch_source
