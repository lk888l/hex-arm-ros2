from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import xacro
import yaml
from types import SimpleNamespace

from hex_arm_bringup.model import controller_parameters, prefix_moveit_config, write_rviz_config


def test_external_model_contains_two_disjoint_arms_and_tools(tmp_path):
    source = Path(get_package_share_directory("hex_arm_integration_examples")) / "urdf/dual_arm.urdf.xacro"
    xml = xacro.process_file(str(source)).toxml()
    model = ET.fromstring(xml)
    links = [element.attrib["name"] for element in model.findall("link")]
    joints = [element.attrib["name"] for element in model.findall("joint")]
    assert len(links) == len(set(links))
    assert len(joints) == len(set(joints))
    assert {element.attrib["name"] for element in model.findall("ros2_control")} == {"LeftArmSystem", "RightArmSystem"}
    assert len([element for element in model.findall("joint") if element.attrib["type"] != "fixed"]) == 12
    for side in ("left", "right"):
        assert model.find(f"joint[@name='{side}_mount_joint']/parent").attrib["link"] == "chassis"
        assert model.find(f"link[@name='{side}_tool']/collision/geometry/box") is not None
        assert model.find(f"joint[@name='{side}_tcp_joint']/origin").attrib["xyz"] == "0 0 0.10"
        control = model.find(f"ros2_control[@name='{side.title()}ArmSystem']")
        for index in range(1, 7):
            assert control.find(f"joint[@name='{side}_joint_{index}']/param[@name='device_joint_name']").text == f"joint_{index}"
    path = tmp_path / "external_robot.urdf"
    path.write_text(xml)
    subprocess.run(["check_urdf", str(path)], check=True, capture_output=True, text=True)


def test_external_manager_parameters_preserve_safety_overrides():
    share = Path(get_package_share_directory("hex_arm_bringup"))
    parameters = controller_parameters(share / "config/controllers.yaml", namespace="left",
        joint_prefix="left_", controller_name="left_arm", controller_manager="/robot/control",
        overrides_path=share / "config/controllers_real.yaml")
    assert set(parameters) == {"/robot/control", "/robot/left_arm", "/robot/joint_state_broadcaster"}
    controller = parameters["/robot/left_arm"]["ros__parameters"]
    assert controller["joints"] == [f"left_joint_{index}" for index in range(1, 7)]
    assert controller["command_interfaces"] == ["position", "velocity"]
    assert controller["constraints"]["left_joint_1"] == {"trajectory": 0.04, "goal": 0.015}
    assert "&id" not in yaml.safe_dump(parameters)
    assert "*id" not in yaml.safe_dump(parameters)


def test_moveit_prefixes_joint_links_and_groups_but_not_urdf_twice():
    model = SimpleNamespace(robot_description={"robot_description": "already_left_prefixed"},
        robot_description_semantic={"robot_description_semantic": '<robot name="robot"><virtual_joint name="world_joint" child_link="base_link" parent_frame="world"/><group name="arm"><chain base_link="base_link" tip_link="link_6"/></group><group_state name="ready" group="arm"><joint name="joint_1" value="0"/></group_state><disable_collisions link1="link_1" link2="link_2"/></robot>'},
        joint_limits={"robot_description_planning": {"joint_limits": {"joint_1": {"max_velocity": 1.0}}}},
        robot_description_kinematics={"robot_description_kinematics": {"arm": {"kinematics_solver": "solver"}}},
        planning_pipelines={"ompl": {"arm": {"planner_configs": ["RRTConnect"]}}},
        trajectory_execution={"moveit_simple_controller_manager": {
            "controller_names": ["firefly_arm_controller"], "firefly_arm_controller": {"joints": ["joint_1"]}}})
    prefix_moveit_config(model, "left_", "left_arm", "left_group", "tcp")
    assert model.robot_description["robot_description"] == "already_left_prefixed"
    semantic = ET.fromstring(model.robot_description_semantic["robot_description_semantic"])
    assert semantic.find("virtual_joint").attrib["parent_frame"] == "world"
    assert semantic.find("group[@name='left_group']/chain").attrib == {"base_link": "left_base_link", "tip_link": "left_tcp"}
    assert semantic.find("group_state/joint").attrib["name"] == "left_joint_1"
    assert "left_joint_1" in model.joint_limits["robot_description_planning"]["joint_limits"]
    assert model.trajectory_execution["moveit_simple_controller_manager"]["left_arm"]["joints"] == ["left_joint_1"]
    model.trajectory_execution = {}
    prefix_moveit_config(model)  # A plan-only MoveIt configuration has no controller manager.


def test_rviz_model_display_selects_the_instance():
    share = Path(get_package_share_directory("hex_arm_bringup"))
    config = yaml.safe_load(Path(write_rviz_config(share / "rviz/firefly_y6.rviz",
        namespace="/robot/left", joint_prefix="left_")).read_text())
    manager = config["Visualization Manager"]
    assert manager["Global Options"]["Fixed Frame"] == "left_base_link"
    display = next(display for display in manager["Displays"] if display["Class"] == "rviz_default_plugins/RobotModel")
    assert display["Description Topic"]["Value"] == "/robot/left/robot_description"
