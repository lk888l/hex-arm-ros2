from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.substitutions import FindPackageShare


def _shutdown_after_failure(step):
    def _handler(event, context):
        if context.is_shutdown:
            return []
        if event.returncode == 0:
            return []
        reason = f"{step} failed with exit code {event.returncode}"
        return [LogInfo(msg=f"ERROR: {reason}"), EmitEvent(event=Shutdown(reason=reason))]

    return _handler


def _shutdown_after_exit(step):
    def _handler(event, context):
        if context.is_shutdown:
            return []
        reason = f"critical process {step} exited with code {event.returncode}"
        return [LogInfo(msg=f"ERROR: {reason}"), EmitEvent(event=Shutdown(reason=reason))]

    return _handler


def _shutdown_actions(reason):
    return [LogInfo(msg=f"ERROR: {reason}"), EmitEvent(event=Shutdown(reason=reason))]


def _guarded_shutdown(reason):
    def _handler(context):
        if context.is_shutdown:
            return []
        return _shutdown_actions(reason)

    return OpaqueFunction(function=_handler)


def _real_nodes(context):
    profile_path = Path(LaunchConfiguration("hardware_profile").perform(context))
    if not profile_path.is_file():
        raise RuntimeError("real bringup requires hardware_profile:=/absolute/path/to/verified.yaml")
    profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    if profile.get("schema_version") != 1 or not profile.get("validated") or not profile.get("calibrated"):
        raise RuntimeError("real bringup refuses profiles not marked validated and calibrated")
    if len(profile.get("joints", [])) != 6:
        raise RuntimeError("real bringup requires six configured joints")

    xacro_file = PathJoinSubstitution([FindPackageShare("hex_arm_description"), "urdf", "firefly_y6.urdf.xacro"])
    controllers = PathJoinSubstitution([FindPackageShare("hex_arm_bringup"), "config", "controllers.yaml"])
    description = {"robot_description": Command([
        FindExecutable(name="xacro"), " ", xacro_file, " backend:=real controllers_file:=", controllers
    ])}
    controller = Node(
        package="hex_arm_controller", executable="hex_arm_controller",
        arguments=["--profile", str(profile_path), "--zenoh-connect", LaunchConfiguration("zenoh_connect")],
        output="screen", emulate_tty=True)
    bridge = LifecycleNode(
        package="hex_arm_bridge", executable="hex_arm_bridge", name="hex_arm_bridge",
        namespace="", autostart=True,
        parameters=[{
            "robot_prefix": profile["robot_prefix"],
            "zenoh_connect": LaunchConfiguration("zenoh_connect"),
            "required_api_major": 0,
        }], output="screen")
    control = Node(
        package="controller_manager", executable="ros2_control_node",
        parameters=[
            description,
            controllers,
            {"hardware_components_initial_state": {
                "inactive": ["FireflyY6System"],
                "shutdown_on_initial_state_failure": True,
            }},
        ],
        output="screen")
    hardware_spawner = Node(
        package="controller_manager", executable="hardware_spawner",
        arguments=[
            "FireflyY6System", "--activate",
            "--controller-manager-timeout", "10.0",
        ],
        output="screen")
    controller_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=[
            "joint_state_broadcaster", "firefly_arm_controller",
            "--activate-as-group",
            "--controller-manager-timeout", "10.0",
            "--switch-timeout", "10.0",
        ],
        output="screen")

    def _after_hardware_activation(event, context):
        if context.is_shutdown:
            return []
        if event.returncode != 0:
            reason = f"hardware activation failed with exit code {event.returncode}"
            return _shutdown_actions(reason)
        return [
            LogInfo(msg="FireflyY6System is active: activating controllers as a group"),
            controller_spawner,
        ]

    startup_handlers = [
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=bridge,
            goal_state="active",
            entities=[LogInfo(msg="hex_arm_bridge is active: starting ros2_control"), control],
        )),
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=bridge,
            start_state="configuring",
            goal_state="unconfigured",
            entities=[_guarded_shutdown("hex_arm_bridge configuration failed")],
        )),
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=bridge,
            start_state="activating",
            goal_state="inactive",
            entities=[_guarded_shutdown("hex_arm_bridge activation failed")],
        )),
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=bridge,
            goal_state="errorprocessing",
            entities=[_guarded_shutdown("hex_arm_bridge entered error processing")],
        )),
        RegisterEventHandler(OnProcessStart(
            target_action=control,
            on_start=[hardware_spawner],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=hardware_spawner,
            on_exit=_after_hardware_activation,
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=controller_spawner,
            on_exit=_shutdown_after_failure("controller activation"),
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=controller,
            on_exit=_shutdown_after_exit("hex_arm_controller"),
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=bridge,
            on_exit=_shutdown_after_exit("hex_arm_bridge"),
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=control,
            on_exit=_shutdown_after_exit("ros2_control_node"),
        )),
    ]
    rviz_config = PathJoinSubstitution([FindPackageShare("hex_arm_bringup"), "rviz", "firefly_y6.rviz"])
    return [
        *startup_handlers,
        controller, bridge,
        Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[description]),
        Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_config],
             condition=IfCondition(LaunchConfiguration("use_rviz"))),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("hardware_profile", default_value=""),
        DeclareLaunchArgument("zenoh_connect", default_value=""),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        OpaqueFunction(function=_real_nodes),
    ])
