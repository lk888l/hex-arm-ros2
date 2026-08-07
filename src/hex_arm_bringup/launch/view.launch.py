from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_rviz = LaunchConfiguration("use_rviz")
    model = PathJoinSubstitution([FindPackageShare("hex_arm_description"), "urdf", "firefly_y6.urdf.xacro"])
    robot_description = {
        "robot_description": Command([FindExecutable(name="xacro"), " ", model, " backend:=view"])
    }
    rviz_config = PathJoinSubstitution([FindPackageShare("hex_arm_bringup"), "rviz", "firefly_y6.rviz"])
    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[robot_description]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
        Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_config], condition=IfCondition(use_rviz)),
    ])

