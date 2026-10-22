import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

from hex_arm_bringup.launch_support import (arm_faulted, boolean, failure_actions, instance_arguments, instance_from_context, stop_owned_processes, value)
from hex_arm_bringup.model import prefix_moveit_config, write_controller_config, write_rviz_config


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
    instance = instance_from_context(context)
    sim_time = boolean(context, "use_sim_time")
    custom_runtime = boolean(context, "use_custom_runtime", True)
    shutdown_scope = value(context, "shutdown_scope", "launch")
    broadcaster_name = value(context, "state_broadcaster_name", "joint_state_broadcaster")
    profile = LaunchConfiguration("limits_profile").perform(context)
    if profile not in LIMIT_PROFILES:
        raise RuntimeError(
            f"limits_profile must be one of {', '.join(LIMIT_PROFILES)}; got {profile!r}"
        )

    description_share = Path(get_package_share_directory("hex_arm_description"))
    bringup_share = Path(get_package_share_directory("hex_arm_bringup"))
    moveit_share = Path(get_package_share_directory("hex_arm_moveit_config"))
    xacro_file = description_share / "urdf" / "firefly_y6.urdf.xacro"
    controllers_file = write_controller_config(
        bringup_share / "config/controllers.yaml", namespace=instance.namespace,
        joint_prefix=instance.joint_prefix, controller_name=instance.controller_name,
        controller_manager=instance.controller_manager, use_sim_time=sim_time,
        state_broadcaster_name=broadcaster_name,
    )

    moveit_config = (
        MoveItConfigsBuilder("firefly_y6", package_name="hex_arm_moveit_config")
        .robot_description(
            file_path=str(xacro_file),
            mappings={
                "backend": "mock",
                "prefix": instance.joint_prefix,
                "hardware_name": instance.hardware_name,
                "ros_namespace": instance.namespace,
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

    prefix_moveit_config(moveit_config, instance.joint_prefix, instance.controller_name, instance.planning_group)
    controller_manager_config = moveit_config.trajectory_execution["moveit_simple_controller_manager"]
    controller_manager_config[instance.controller_ros_name()] = controller_manager_config.pop(instance.controller_name)
    controller_manager_config["controller_names"] = [instance.controller_ros_name()]
    manager_path = instance.ros_name(instance.controller_manager)
    manager_ns, manager_name = manager_path.rsplit("/", 1)
    clock_parameters = {"use_sim_time": sim_time}

    control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name=manager_name, namespace=manager_ns,
        remappings=[("robot_description", instance.ros_name("robot_description"))],
        parameters=[moveit_config.robot_description, clock_parameters, str(controllers_file)],
        output="screen",
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=instance.namespace,
        remappings=[("joint_states", manager_ns + "/joint_states")],
        parameters=[moveit_config.robot_description, clock_parameters],
        output="screen",
    )
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=instance.namespace,
        arguments=[
            broadcaster_name,
            "--controller-manager",
            manager_path,
            "--param-file", str(controllers_file),
        ],
        output="screen",
    )
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=instance.namespace,
        arguments=[
            instance.controller_name,
            "--controller-manager",
            manager_path,
            "--param-file", str(controllers_file),
        ],
        output="screen",
    )
    world_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_firefly_base", namespace=instance.namespace,
        parameters=[clock_parameters],
        arguments=["--frame-id", "world", "--child-frame-id", instance.joint_prefix + "base_link"],
        output="log",
    )
    move_group = Node(
        package="hex_arm_moveit_runtime" if custom_runtime else "moveit_ros_move_group",
        executable="hex_arm_move_group" if custom_runtime else "move_group",
        name=instance.move_group_name, namespace=instance.namespace,
        remappings=[("joint_states", manager_ns + "/joint_states")],
        output="screen",
        additional_env=_move_group_environment() if custom_runtime else {},
        parameters=[
            moveit_config.to_dict(),
            clock_parameters,
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
        name="moveit_rviz", namespace=instance.namespace,
        output="screen",
        arguments=["-d", write_rviz_config(moveit_share / "config/moveit.rviz",
            namespace=instance.namespace, joint_prefix=instance.joint_prefix,
            planning_group=instance.planning_group, use_sim_time=sim_time)],
        parameters=[
            clock_parameters,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    def _next_or_shutdown(event, callback_context, next_actions, step):
        if arm_faulted(callback_context, instance.namespace):
            return []
        if event.returncode == 0:
            return next_actions
        reason = f"{step} failed with exit code {event.returncode}"
        return failure_actions(reason, instance.namespace, shutdown_scope)

    def _shutdown_after_exit(event, callback_context, step):
        if arm_faulted(callback_context, instance.namespace):
            return []
        return failure_actions(f"critical process {step} exited with code {event.returncode}",
                               instance.namespace, shutdown_scope)

    return [
        stop_owned_processes(instance.namespace, [control, robot_state_publisher, joint_state_spawner, arm_spawner, world_to_base, move_group, rviz], request_deactivation=False),
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
        instance_arguments() + [
            DeclareLaunchArgument("use_sim_time", default_value="false", choices=["true", "false"]),
            DeclareLaunchArgument("state_broadcaster_name", default_value="joint_state_broadcaster"),
            DeclareLaunchArgument("use_custom_runtime", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("shutdown_scope", default_value="launch", choices=["launch", "arm"]),
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
