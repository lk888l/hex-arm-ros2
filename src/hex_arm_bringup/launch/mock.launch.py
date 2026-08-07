from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_rviz = LaunchConfiguration("use_rviz")
    xacro_file = PathJoinSubstitution([FindPackageShare("hex_arm_description"), "urdf", "firefly_y6.urdf.xacro"])
    controllers = PathJoinSubstitution([FindPackageShare("hex_arm_bringup"), "config", "controllers.yaml"])
    description = {"robot_description": Command([
        FindExecutable(name="xacro"), " ", xacro_file, " backend:=mock controllers_file:=", controllers
    ])}
    control = Node(
        package="controller_manager", executable="ros2_control_node",
        parameters=[description, controllers], output="screen")
    joint_state_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"])
    arm_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["firefly_arm_controller", "--controller-manager", "/controller_manager"])
    rviz_config = PathJoinSubstitution([FindPackageShare("hex_arm_bringup"), "rviz", "firefly_y6.rviz"])
    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        control,
        Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[description]),
        joint_state_spawner,
        RegisterEventHandler(OnProcessExit(target_action=joint_state_spawner, on_exit=[arm_spawner])),
        Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_config], condition=IfCondition(use_rviz)),
    ])

