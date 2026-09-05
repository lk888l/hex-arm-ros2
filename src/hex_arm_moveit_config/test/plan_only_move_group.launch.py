"""Offline move_group fixture for the deliberately relaxed plan-only SRDF.

This launch description has no ros2_control, bridge, hardware bringup, or CAN
process.  It exists only so the collision contract can exercise FCL and OMPL.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description() -> LaunchDescription:
    description_share = Path(get_package_share_directory("hex_arm_description"))
    xacro_file = description_share / "urdf" / "firefly_y6.urdf.xacro"

    moveit_config = (
        MoveItConfigsBuilder("firefly_y6", package_name="hex_arm_moveit_config")
        .robot_description(
            file_path=str(xacro_file),
            mappings={"backend": "view", "controllers_file": ""},
        )
        .robot_description_semantic(
            file_path="config/firefly_y6.plan_only.srdf"
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .joint_limits(file_path="config/joint_limits_commissioning.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )
    # MoveItConfigsBuilder auto-discovers moveit_controllers.yaml even when
    # trajectory_execution() is not requested. This fixture models the real
    # non-executable launch, which must not configure a controller plugin or
    # advertise a FollowJointTrajectory client.
    moveit_config.trajectory_execution = {}
    runtime_prefix = Path(get_package_prefix("hex_arm_moveit_runtime"))
    preload = runtime_prefix / "lib" / "libhex_arm_moveit_tem_shutdown.so"
    if not preload.is_file():
        raise RuntimeError(f"ordered-shutdown MoveIt preload is missing: {preload}")

    inherited = os.environ.get("LD_PRELOAD")
    preload_value = f"{inherited}:{preload}" if inherited else str(preload)

    return LaunchDescription(
        [
            Node(
                package="hex_arm_moveit_runtime",
                executable="hex_arm_move_group",
                output="screen",
                additional_env={"LD_PRELOAD": preload_value},
                parameters=[
                    moveit_config.to_dict(),
                    {
                        "allow_trajectory_execution": False,
                        "publish_planning_scene": True,
                        "publish_geometry_updates": True,
                        "publish_state_updates": True,
                        "publish_transforms_updates": True,
                        "monitor_dynamics": False,
                    },
                ],
            )
        ]
    )
