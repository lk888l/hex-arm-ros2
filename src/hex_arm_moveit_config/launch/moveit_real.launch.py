import os
import math

import yaml
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


STATE_TOPIC = "/hex_arm/internal/state"


def _shutdown_after_exit(step: str):
    """Stop the complete real stack when a critical MoveIt process exits."""

    def _handler(event, context):
        if context.is_shutdown:
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

    return _handler


def _move_group_environment() -> dict[str, str]:
    runtime_prefix = Path(get_package_prefix("hex_arm_moveit_runtime"))
    preload = runtime_prefix / "lib" / "libhex_arm_moveit_tem_shutdown.so"
    if not preload.is_file():
        raise RuntimeError(f"ordered-shutdown MoveIt preload is missing: {preload}")
    inherited = os.environ.get("LD_PRELOAD")
    value = f"{inherited}:{preload}" if inherited else str(preload)
    return {"LD_PRELOAD": value}


def _boolean_argument(context: LaunchContext, name: str) -> bool:
    value = LaunchConfiguration(name).perform(context).lower()
    if value not in ("true", "false"):
        raise RuntimeError(f"{name} must be 'true' or 'false'")
    return value == "true"


def _semantic_file(enable_execution: bool) -> str:
    # The surveyed-fold ACM exceptions are visualization/planning aids only.
    # Any execution-capable process must load the strict semantic model.
    return (
        "config/firefly_y6.srdf"
        if enable_execution
        else "config/firefly_y6.plan_only.srdf"
    )


def _restrict_planning_limits(moveit_config, profile):
    """Use the intersection of commissioning and verified hardware authority."""
    planning = moveit_config.joint_limits["robot_description_planning"]["joint_limits"]
    joints = profile.get("joints", [])
    names = [joint["name"] for joint in joints]
    if len(names) != len(planning) or set(names) != set(planning):
        raise RuntimeError("hardware planning limits require exactly the configured six joints")
    for joint in joints:
        limits = joint["limits"]
        target = planning[joint["name"]]
        lower = max(float(target["min_position"]), float(limits["position_lower_rad"]))
        upper = min(float(target["max_position"]), float(limits["position_upper_rad"]))
        velocity = min(float(target["max_velocity"]), float(limits["velocity_rad_s"]))
        acceleration = min(float(target["max_acceleration"]), float(limits["acceleration_rad_s2"]))
        values = [*map(float, (limits["position_lower_rad"], limits["position_upper_rad"],
                              limits["velocity_rad_s"], limits["acceleration_rad_s2"])),
                  lower, upper, velocity, acceleration]
        if not all(math.isfinite(v) for v in values) or lower >= upper or min(velocity, acceleration) <= 0:
            raise RuntimeError(f"empty or invalid planning limit intersection for {joint['name']}")
        target.update(has_position_limits=True, min_position=lower, max_position=upper,
                      has_velocity_limits=True, max_velocity=velocity,
                      has_acceleration_limits=True, max_acceleration=acceleration)


def _build_moveit_config(enable_execution: bool, hardware_profile=None):
    description_share = Path(get_package_share_directory("hex_arm_description"))
    bringup_share = Path(get_package_share_directory("hex_arm_bringup"))
    xacro_file = description_share / "urdf" / "firefly_y6.urdf.xacro"
    controllers_file = bringup_share / "config" / "controllers.yaml"

    builder = (
        MoveItConfigsBuilder("firefly_y6", package_name="hex_arm_moveit_config")
        .robot_description(
            file_path=str(xacro_file),
            mappings={
                "backend": "real",
                "controllers_file": str(controllers_file),
            },
        )
        .robot_description_semantic(file_path=_semantic_file(enable_execution))
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        # Real hardware always starts with the deliberately slow commissioning
        # profile. This launch file intentionally exposes no profile override.
        .joint_limits(file_path="config/joint_limits_commissioning.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
    )
    if enable_execution:
        # Controller-manager parameters and FollowJointTrajectory clients must
        # exist only across the explicitly enabled execution boundary.
        builder = builder.trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
    moveit_config = builder.to_moveit_configs()
    if not enable_execution:
        # to_moveit_configs() auto-discovers *_controllers.yaml when this field
        # is empty, so merely omitting trajectory_execution() is insufficient.
        # Clear the auto-discovered controller plugin before Node parameters
        # are materialized.
        moveit_config.trajectory_execution = {}
    if hardware_profile is not None:
        with open(hardware_profile, encoding="utf-8") as profile_file:
            _restrict_planning_limits(moveit_config, yaml.safe_load(profile_file))
    return moveit_config


def _move_group_runtime_parameters(enable_execution: bool) -> dict[str, bool]:
    return {
        "allow_trajectory_execution": enable_execution,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
    }


def _bringup_arguments(
    hardware_profile: str, zenoh_connect: str, enable_execution: bool, startup_ready: bool = True
) -> dict[str, str]:
    return {
        "hardware_profile": hardware_profile,
        "zenoh_connect": zenoh_connect,
        # One switch controls both sides of the execution boundary: MoveIt can
        # execute only when the hardware and trajectory controller are active.
        "activate_hardware": "true" if enable_execution else "false",
        "startup_ready": "true" if enable_execution and startup_ready else "false",
        # The outer launch owns the single MoveIt-configured RViz process.
        "use_rviz": "false",
    }


def _launch_setup(context: LaunchContext):
    enable_execution = _boolean_argument(context, "enable_execution")
    use_rviz = _boolean_argument(context, "use_rviz")
    publish_world_tf = _boolean_argument(context, "publish_world_tf")
    startup_ready = _boolean_argument(context, "startup_ready")
    hardware_profile = LaunchConfiguration("hardware_profile").perform(context)
    zenoh_connect = LaunchConfiguration("zenoh_connect").perform(context)

    bringup_share = Path(get_package_share_directory("hex_arm_bringup"))
    moveit_share = Path(get_package_share_directory("hex_arm_moveit_config"))
    moveit_config = _build_moveit_config(enable_execution, hardware_profile)

    real_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(bringup_share / "launch" / "real.launch.py")
        ),
        launch_arguments=_bringup_arguments(
            hardware_profile, zenoh_connect, enable_execution, startup_ready
        ).items(),
    )
    move_group = Node(
        package="hex_arm_moveit_runtime",
        executable="hex_arm_move_group",
        output="screen",
        additional_env=_move_group_environment(),
        parameters=[
            moveit_config.to_dict(),
            _move_group_runtime_parameters(enable_execution),
        ],
        remappings=[("joint_states", STATE_TOPIC)],
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
        # The included bringup sets its own use_rviz=false in this context.
        # Capture the outer request before that include executes.
        condition=IfCondition("true" if use_rviz else "false"),
    )

    # A FollowJointTrajectory goal already accepted by ros2_control can outlive
    # move_group. Treat move_group as critical so any clean or failed exit
    # tears down bringup and reaches the controller's confirmed-disable path.
    actions = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=move_group,
                on_exit=_shutdown_after_exit("move_group"),
            )
        ),
        real_bringup,
        move_group,
    ]
    if publish_world_tf:
        # real.launch.py deliberately owns robot_state_publisher but publishes
        # no world transform. Keep this one publisher outside bringup so the
        # combined launch never duplicates either RSP or world -> base_link.
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="moveit_world_to_firefly_base",
                arguments=[
                    "--frame-id",
                    "world",
                    "--child-frame-id",
                    "base_link",
                ],
                output="log",
            )
        )
    actions.append(rviz)
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hardware_profile",
                default_value="",
                description=(
                    "Absolute path to a validated hardware profile. Calibration is "
                    "required only when enable_execution is true."
                ),
            ),
            DeclareLaunchArgument(
                "zenoh_connect",
                default_value="",
                description=(
                    "Optional explicit Zenoh endpoint. Empty uses the deterministic "
                    "controller-to-bridge loopback endpoint tcp/127.0.0.1:7448."
                ),
            ),
            DeclareLaunchArgument(
                "enable_execution",
                default_value="false",
                choices=["true", "false"],
                description=(
                    "Explicitly activate hardware/controllers and allow MoveIt execution. "
                    "The default is observation and planning only."
                ),
            ),
            DeclareLaunchArgument(
                "startup_ready",
                default_value="true",
                choices=["true", "false"],
                description=(
                    "With enable_execution=true, run the verified J2 -> J4 -> J3 fold exit "
                    "and hold [0,-1.35,3,-0.3,0,0]. False keeps measured-pose activation."
                ),
            ),
            DeclareLaunchArgument(
                "publish_world_tf",
                default_value="true",
                choices=["true", "false"],
                description=(
                    "Publish the fixed world -> base_link transform. Disable only when "
                    "another localization or cell node owns that transform."
                ),
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                choices=["true", "false"],
                description="Start RViz with the MoveIt MotionPlanning display.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
