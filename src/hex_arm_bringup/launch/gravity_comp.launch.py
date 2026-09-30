"""Hand guiding from the measured pose; no trajectory controller or unfolding."""
import math
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.logging import launch_config
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def _boolean(context, name):
    value = LaunchConfiguration(name).perform(context).lower()
    if value not in ("true", "false"):
        raise RuntimeError(f"{name} must be true or false")
    return value == "true"


def _load_profile(path, activate, mock):
    path = Path(path)
    if not path.is_absolute() or not path.is_file():
        raise RuntimeError("hardware_profile must be an existing absolute path")
    profile = yaml.safe_load(path.read_text(encoding="utf-8"))
    if (not isinstance(profile, dict) or profile.get("schema_version") != 3
            or profile.get("joint_coordinate_version") != 2
            or profile.get("validated") is not True
            or "gravity_vector_base_m_s2" not in profile
            or len(profile.get("joints", [])) != 6):
        raise RuntimeError("hand guiding requires a validated schema v3 six-axis profile with installation gravity")
    if activate and profile.get("calibrated") is not True:
        raise RuntimeError("hand guiding requires calibrated joint coordinates")
    if not mock and profile.get("bus", {}).get("protocol") not in ("meow", "cia402"):
        raise RuntimeError("real hand guiding requires an explicit meow or cia402 protocol")
    return profile


def _damping(value):
    values = yaml.safe_load(value)
    if (not isinstance(values, list) or len(values) != 6
            or any(isinstance(v, bool) or not isinstance(v, (int, float))
                   or not math.isfinite(v) or v <= 0 for v in values)):
        raise RuntimeError("damping must be a list of six finite positive N*m*s/rad gains")
    return [float(v) for v in values]


def _nodes(context):
    activate = _boolean(context, "activate_hardware")
    mock = _boolean(context, "mock")
    profile_path = LaunchConfiguration("hardware_profile").perform(context)
    profile = _load_profile(profile_path, activate, mock)
    damping = _damping(LaunchConfiguration("damping").perform(context))
    endpoint = LaunchConfiguration("zenoh_endpoint").perform(context)
    urdf = PathJoinSubstitution([
        FindPackageShare("xpkg_urdf_firefly_y6"), "urdf", "xpkg_urdf_firefly_y6.urdf"
    ]).perform(context)
    driver = ExecuteProcess(cmd=[
        PathJoinSubstitution([FindPackagePrefix("hex_arm_controller"), "lib", "hex_arm_controller", "hex_arm_controller"]),
        "--profile", profile_path, "--urdf", urdf, "--zenoh-listen", endpoint,
        "--shutdown-report", str(Path(launch_config.log_dir) / "hand-guiding-shutdown.json"),
        *(["--mock"] if mock else []),
    ], output="screen", emulate_tty=True)
    # The existing bridge is an observer here: only the dedicated owner below
    # acquires a session. No ros2_control hardware component is launched.
    bridge = LifecycleNode(package="hex_arm_bridge", executable="hex_arm_bridge",
        name="hex_arm_bridge", namespace="", autostart=True,
        parameters=[{"robot_prefix": profile["robot_prefix"], "zenoh_connect": endpoint,
                     "required_api_major": 0, "startup_timeout_sec": 30.0}], output="screen")
    owner = Node(package="hex_arm_bridge", executable="hex_arm_gravity_comp",
        name="hex_arm_gravity_comp", parameters=[{
            "robot_prefix": profile["robot_prefix"], "zenoh_connect": endpoint,
            "damping": damping, "startup_timeout_sec": 30.0,
        }], output="screen")

    def stopped(event, context):
        if context.is_shutdown:
            return []
        return [EmitEvent(event=Shutdown(reason=f"hand-guiding process exited ({event.returncode})"))]

    handlers = [RegisterEventHandler(OnProcessExit(target_action=process, on_exit=stopped))
                for process in (driver, bridge, owner)]
    for start, goal in (("configuring", "unconfigured"), ("activating", "inactive")):
        handlers.append(RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=bridge, start_state=start, goal_state=goal,
            entities=[EmitEvent(event=Shutdown(reason="hand-guiding observer initialization failed"))])))
    handlers.append(RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=bridge, goal_state="errorprocessing",
        entities=[EmitEvent(event=Shutdown(reason="hand-guiding observer failed"))])))
    if activate:
        handlers.append(RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=bridge, goal_state="active", entities=[owner])))
    else:
        handlers.append(LogInfo(msg="OBSERVE ONLY: pass activate_hardware:=true to start hand guiding"))
    return [*handlers, driver, bridge,
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": Path(urdf).read_text(encoding="utf-8")}],
             remappings=[("joint_states", "/hex_arm/internal/state")]),
        Node(package="rviz2", executable="rviz2", arguments=["-d", PathJoinSubstitution([
            FindPackageShare("hex_arm_bringup"), "rviz", "firefly_y6.rviz"])],
            condition=IfCondition(LaunchConfiguration("use_rviz"))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("hardware_profile", default_value=""),
        DeclareLaunchArgument("activate_hardware", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("mock", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("damping", default_value="[2.0, 2.0, 2.0, 2.0, 2.0, 2.0]",
                              description="Joint-side viscous damping in N*m*s/rad, J1..J6"),
        DeclareLaunchArgument("zenoh_endpoint", default_value="tcp/127.0.0.1:7448"),
        DeclareLaunchArgument("use_rviz", default_value="false", choices=["true", "false"]),
        OpaqueFunction(function=_nodes),
    ])
