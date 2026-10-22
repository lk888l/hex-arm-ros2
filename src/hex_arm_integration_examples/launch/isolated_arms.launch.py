"""One robot description, two independent arm controller managers."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    examples = Path(get_package_share_directory("hex_arm_integration_examples"))
    bringup = Path(get_package_share_directory("hex_arm_bringup"))
    description = xacro.process_file(str(examples / "urdf/dual_arm.urdf.xacro"),
                                     mappings={"backend": "view"}).toxml()
    actions = [Node(package="robot_state_publisher", executable="robot_state_publisher",
                    parameters=[{"robot_description": description}]),
               Node(package="hex_arm_integration_examples", executable="joint_state_mux.py",
                    parameters=[{"sources": ["/left/joint_states", "/right/joint_states"]}])]
    for side in ("left", "right"):
        actions.append(GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / "launch/mock.launch.py")),
            launch_arguments={"namespace": side, "joint_prefix": side + "_",
                              "hardware_name": side.title() + "ArmSystem",
                              "start_robot_state_publisher": "false", "use_rviz": "false",
                              "shutdown_scope": "arm"}.items(),
        )]))
    return LaunchDescription(actions)
