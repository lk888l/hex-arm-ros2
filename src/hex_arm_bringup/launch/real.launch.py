from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


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
        parameters=[{
            "robot_prefix": profile["robot_prefix"],
            "zenoh_connect": LaunchConfiguration("zenoh_connect"),
            "required_api_major": 0,
        }], output="screen")
    configure = TimerAction(period=1.0, actions=[EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(bridge), transition_id=Transition.TRANSITION_CONFIGURE))])
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=bridge, goal_state="inactive",
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(bridge), transition_id=Transition.TRANSITION_ACTIVATE))]))
    control = Node(
        package="controller_manager", executable="ros2_control_node",
        parameters=[description, controllers], output="screen")
    spawners = TimerAction(period=5.0, actions=[
        Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"]),
        Node(package="controller_manager", executable="spawner", arguments=["firefly_arm_controller"]),
    ])
    rviz_config = PathJoinSubstitution([FindPackageShare("hex_arm_bringup"), "rviz", "firefly_y6.rviz"])
    return [
        controller, bridge, configure, activate, control,
        Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[description]),
        spawners,
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

