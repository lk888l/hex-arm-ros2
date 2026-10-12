"""Enable the real ROS stack by holding its existing measured pose."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _setup(context):
    if LaunchConfiguration("allow_startup_motion").perform(context).lower() != "false":
        raise RuntimeError("startup motion is disabled; allow_startup_motion must be false")
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("hex_arm_bringup"), "launch", "real.launch.py",
        ])),
        launch_arguments={
            "hardware_profile": LaunchConfiguration("hardware_profile"),
            "zenoh_connect": LaunchConfiguration("zenoh_connect"),
            "activate_hardware": "true",
            "startup_ready": "true",
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("hardware_profile"),
        DeclareLaunchArgument("zenoh_connect", default_value=""),
        DeclareLaunchArgument("use_rviz", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument(
            "allow_startup_motion", default_value="false", choices=["true", "false"],
            description="Retired startup motion option; must be false",
        ),
        OpaqueFunction(function=_setup),
    ])
