"""The whole robot owns its model and a single controller manager."""
import atexit
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
import yaml

from hex_arm_bringup.model import controller_parameters


def generate_launch_description():
    examples = Path(get_package_share_directory("hex_arm_integration_examples"))
    bringup = Path(get_package_share_directory("hex_arm_bringup"))
    description = {"robot_description": xacro.process_file(
        str(examples / "urdf/dual_arm.urdf.xacro"), mappings={"backend": "mock"}).toxml()}
    parameters = {}
    for side in ("left", "right"):
        arm = controller_parameters(bringup / "config/controllers.yaml",
                                    joint_prefix=side + "_", controller_name=side + "_arm_controller")
        for node, values in arm.items():
            parameters.setdefault(node, {"ros__parameters": {}})["ros__parameters"].update(values["ros__parameters"])
    parameters["/joint_state_broadcaster"]["ros__parameters"]["joints"] = [
        f"{side}_joint_{index}" for side in ("left", "right") for index in range(1, 7)]
    with tempfile.NamedTemporaryFile(mode="w", prefix="hex_dual_arm_", suffix=".yaml", delete=False) as stream:
        yaml.safe_dump(parameters, stream)
        parameter_file = stream.name
    atexit.register(Path(parameter_file).unlink, missing_ok=True)
    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[description]),
        Node(package="controller_manager", executable="ros2_control_node", parameters=[description, parameter_file]),
        Node(package="controller_manager", executable="spawner", arguments=[
            "joint_state_broadcaster", "left_arm_controller", "right_arm_controller",
            "--controller-manager", "/controller_manager", "--param-file", parameter_file]),
    ])
