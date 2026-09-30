"""Offline strict planner using the production hardware-limit merge.

Only MoveGroup is launched. No motor driver, bridge or hardware process exists.
"""
import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = Path(get_package_share_directory("hex_arm_moveit_config"))
    spec = importlib.util.spec_from_file_location("deployment_configuration", share / "launch/moveit_real.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    config = module._build_moveit_config(True, LaunchConfiguration("hardware_profile").perform(context),
                                          "hardware", "hardware")
    readiness_token = LaunchConfiguration("startup_readiness_token").perform(context)
    if not readiness_token:
        config.trajectory_execution = {}
    return [Node(package="hex_arm_moveit_runtime", executable="hex_arm_move_group",
                 output="screen", additional_env=module._move_group_environment(),
                 parameters=[config.to_dict(), module._move_group_runtime_parameters(bool(readiness_token)),
                             {"startup_readiness_token": readiness_token}])]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument("hardware_profile"),
                              DeclareLaunchArgument("startup_readiness_token", default_value=""),
                              OpaqueFunction(function=setup)])
