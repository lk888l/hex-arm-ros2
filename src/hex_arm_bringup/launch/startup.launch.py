"""Explicit commissioning of the operator-verified folded-to-ready path."""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix


def _setup(context):
    if LaunchConfiguration("allow_startup_motion").perform(context).lower() != "true":
        raise RuntimeError(
            "startup.launch.py moves real motors; requires allow_startup_motion:=true"
        )
    process = ExecuteProcess(
        cmd=[
            PathJoinSubstitution([
                FindPackagePrefix("hex_arm_controller"),
                "lib",
                "hex_arm_controller",
                "hex_arm_controller",
            ]),
            "--profile",
            LaunchConfiguration("hardware_profile"),
            "--startup-sequence",
            "--allow-startup-motion",
        ],
        output="screen",
        emulate_tty=True,
    )
    return [
        RegisterEventHandler(OnProcessExit(
            target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason="bounded startup finished"))],
        )),
        process,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("hardware_profile"),
        DeclareLaunchArgument(
            "allow_startup_motion", default_value="false", choices=["true", "false"]
        ),
        OpaqueFunction(function=_setup),
    ])
