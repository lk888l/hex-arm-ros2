from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")
    xacro_file = PathJoinSubstitution([FindPackageShare("hex_arm_description"), "urdf", "firefly_y6.urdf.xacro"])
    controllers = PathJoinSubstitution([FindPackageShare("hex_arm_bringup"), "config", "gz_controllers.yaml"])
    world = PathJoinSubstitution([FindPackageShare("hex_arm_bringup"), "worlds", "empty.sdf"])
    resource_path = PathJoinSubstitution([FindPackageShare("xpkg_urdf_firefly_y6"), ".."])
    description = {"robot_description": Command([
        FindExecutable(name="xacro"), " ", xacro_file,
        " backend:=gz controllers_file:=", controllers,
    ])}
    gz_launch = PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
    spawn = Node(
        package="ros_gz_sim", executable="create",
        arguments=["-name", "firefly_y6", "-topic", "robot_description", "-z", "0.0"], output="screen")
    clock_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen")
    delayed_controllers = TimerAction(period=5.0, actions=[
        Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"]),
        Node(package="controller_manager", executable="spawner", arguments=["firefly_arm_controller"]),
    ])
    rviz_config = PathJoinSubstitution([FindPackageShare("hex_arm_bringup"), "rviz", "firefly_y6.rviz"])
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={"gz_args": ["-r -s ", world]}.items(), condition=IfCondition(headless)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={"gz_args": ["-r ", world]}.items(), condition=UnlessCondition(headless)),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[description, {"use_sim_time": True}]),
        spawn,
        clock_bridge,
        delayed_controllers,
        Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_config],
             parameters=[{"use_sim_time": True}], condition=IfCondition(use_rviz)),
    ])

