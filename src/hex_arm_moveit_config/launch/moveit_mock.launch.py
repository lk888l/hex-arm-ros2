import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


LIMIT_PROFILES = ("sim", "commissioning", "verified")


def _move_group_environment() -> dict[str, str]:
    runtime_prefix = Path(get_package_prefix("hex_arm_moveit_runtime"))
    preload = runtime_prefix / "lib" / "libhex_arm_moveit_tem_shutdown.so"
    if not preload.is_file():
        raise RuntimeError(f"ordered-shutdown MoveIt preload is missing: {preload}")
    inherited = os.environ.get("LD_PRELOAD")
    value = f"{inherited}:{preload}" if inherited else str(preload)
    return {"LD_PRELOAD": value}


def _launch_setup(context):
    profile = LaunchConfiguration("limits_profile").perform(context)
    if profile not in LIMIT_PROFILES:
        raise RuntimeError(
            f"limits_profile must be one of {', '.join(LIMIT_PROFILES)}; got {profile!r}"
        )

    description_share = Path(get_package_share_directory("hex_arm_description"))
    bringup_share = Path(get_package_share_directory("hex_arm_bringup"))
    moveit_share = Path(get_package_share_directory("hex_arm_moveit_config"))
    xacro_file = description_share / "urdf" / "firefly_y6.urdf.xacro"
    controllers_file = bringup_share / "config" / "controllers.yaml"

    moveit_config = (
        MoveItConfigsBuilder("firefly_y6", package_name="hex_arm_moveit_config")
        .robot_description(
            file_path=str(xacro_file),
            mappings={
                "backend": "mock",
                "controllers_file": str(controllers_file),
                "initial_positions_file": str(moveit_share / "config" / "initial_positions.yaml"),
            },
        )
        .robot_description_semantic(file_path="config/firefly_y6.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
        .joint_limits(file_path=f"config/joint_limits_{profile}.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, str(controllers_file)],
        output="screen",
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],
        output="screen",
    )
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "firefly_arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )
    world_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_firefly_base",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
        output="log",
    )
    move_group = Node(
        package="hex_arm_moveit_runtime",
        executable="hex_arm_move_group",
        output="screen",
        additional_env=_move_group_environment(),
        parameters=[
            moveit_config.to_dict(),
            {
                "allow_trajectory_execution": True,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
                "monitor_dynamics": False,
            },
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="moveit_rviz",
        output="screen",
        arguments=["-d", str(moveit_share / "config" / "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    def _next_or_shutdown(event, callback_context, next_actions, step):
        if callback_context.is_shutdown:
            return []
        if event.returncode == 0:
            return next_actions
        reason = f"{step} failed with exit code {event.returncode}"
        return [
            LogInfo(msg=f"ERROR: {reason}"),
            EmitEvent(event=Shutdown(reason=reason)),
        ]

    def _shutdown_after_exit(event, callback_context, step):
        if callback_context.is_shutdown:
            return []
        if event.returncode == 0:
            reason = f"critical process {step} exited cleanly"
            return [
                LogInfo(msg=f"{reason}; shutting down dependent processes"),
                EmitEvent(event=Shutdown(reason=reason)),
            ]
        reason = f"critical process {step} exited with code {event.returncode}"
        return [
            LogInfo(msg=f"ERROR: {reason}"),
            EmitEvent(event=Shutdown(reason=reason)),
        ]

    return [
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_spawner,
                on_exit=lambda event, callback_context: _next_or_shutdown(
                    event,
                    callback_context,
                    [arm_spawner],
                    "joint_state_broadcaster startup",
                ),
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=arm_spawner,
                on_exit=lambda event, callback_context: _next_or_shutdown(
                    event,
                    callback_context,
                    [move_group, rviz],
                    "firefly_arm_controller startup",
                ),
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=control,
                on_exit=lambda event, callback_context: _shutdown_after_exit(
                    event, callback_context, "ros2_control_node"
                ),
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=move_group,
                on_exit=lambda event, callback_context: _shutdown_after_exit(
                    event, callback_context, "move_group"
                ),
            )
        ),
        control,
        robot_state_publisher,
        joint_state_spawner,
        world_to_base,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                choices=["true", "false"],
                description="Start RViz with the MoveIt MotionPlanning panel.",
            ),
            DeclareLaunchArgument(
                "limits_profile",
                default_value="sim",
                choices=list(LIMIT_PROFILES),
                description="MoveIt velocity limit profile; mock simulation defaults to sim.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
