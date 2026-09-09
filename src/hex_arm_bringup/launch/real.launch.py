import math
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch.logging import launch_config
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


DEFAULT_ZENOH_DIRECT_ENDPOINT = "tcp/127.0.0.1:7448"
DEFAULT_BRIDGE_STARTUP_TIMEOUT_SEC = 30.0


def _zenoh_routes(requested_connect):
    connect = str(requested_connect).strip()
    if connect:
        return ["--zenoh-connect", connect], connect
    return ["--zenoh-listen", DEFAULT_ZENOH_DIRECT_ENDPOINT], DEFAULT_ZENOH_DIRECT_ENDPOINT


def _positive_seconds(value, name):
    try:
        seconds = float(value)
    except (TypeError, ValueError) as error:
        raise RuntimeError(f"{name} must be a finite positive number") from error
    if not math.isfinite(seconds) or seconds <= 0.0:
        raise RuntimeError(f"{name} must be a finite positive number")
    return seconds


def _bridge_parameters(profile, zenoh_connect, startup_timeout_sec):
    return {
        "robot_prefix": profile["robot_prefix"],
        "zenoh_connect": zenoh_connect,
        "required_api_major": 0,
        # One deadline covers API discovery plus the first fresh state.  The
        # six-axis CAN initialization measured about 16.3 s on the real arm,
        # so the launch default deliberately leaves a conservative margin.
        "startup_timeout_sec": startup_timeout_sec,
    }


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
        if event.returncode == 0:
            reason = f"critical process {step} exited cleanly"
            return [
                LogInfo(msg=f"{reason}; shutting down dependent processes"),
                EmitEvent(event=Shutdown(reason=reason)),
            ]
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
    activate_hardware = LaunchConfiguration("activate_hardware").perform(context).lower()
    if activate_hardware not in ("true", "false"):
        raise RuntimeError("activate_hardware must be 'true' or 'false'")
    activate_hardware = activate_hardware == "true"
    startup_ready = LaunchConfiguration("startup_ready", default="false").perform(context).lower()
    if startup_ready not in ("true", "false"):
        raise RuntimeError("startup_ready must be 'true' or 'false'")
    startup_ready = activate_hardware and startup_ready == "true"
    if startup_ready and profile.get("bus", {}).get("protocol") != "meow":
        raise RuntimeError("ordered startup_ready requires a Meow hardware profile")
    controller_zenoh_args, bridge_zenoh_connect = _zenoh_routes(
        LaunchConfiguration("zenoh_connect").perform(context)
    )
    bridge_startup_timeout_sec = _positive_seconds(
        LaunchConfiguration("bridge_startup_timeout_sec").perform(context),
        "bridge_startup_timeout_sec",
    )

    if profile.get("schema_version") != 2 or not profile.get("validated"):
        raise RuntimeError(
            "real bringup requires a validated schema v2 profile with an explicit "
            "gravity_vector_base_m_s2"
        )
    if "gravity_vector_base_m_s2" not in profile:
        raise RuntimeError(
            "real bringup requires the schema v2 gravity_vector_base_m_s2 installation parameter"
        )
    if activate_hardware and not profile.get("calibrated"):
        raise RuntimeError(
            "hardware activation refuses profiles not marked calibrated; "
            "use activate_hardware:=false for non-actuating observation"
        )
    if len(profile.get("joints", [])) != 6:
        raise RuntimeError("real bringup requires six configured joints")

    xacro_file = PathJoinSubstitution([FindPackageShare("hex_arm_description"), "urdf", "firefly_y6.urdf.xacro"])
    controllers = PathJoinSubstitution([FindPackageShare("hex_arm_bringup"), "config", "controllers.yaml"])
    real_commands = PathJoinSubstitution([FindPackageShare("hex_arm_bringup"), "config", "controllers_real.yaml"])
    description = {"robot_description": Command([
        FindExecutable(name="xacro"), " ", xacro_file, " backend:=real controllers_file:=", controllers
    ])}
    controller_executable = PathJoinSubstitution(
        [
            FindPackagePrefix("hex_arm_controller"),
            "lib",
            "hex_arm_controller",
            "hex_arm_controller",
        ]
    )
    # This is a plain Rust/Zenoh process, not an rclcpp/rclpy node. Using
    # launch_ros.actions.Node would append `--ros-args`, which clap correctly
    # rejects before the controller can initialize.
    controller = ExecuteProcess(
        cmd=[
            controller_executable,
            "--profile",
            str(profile_path),
            *controller_zenoh_args,
        ],
        output="screen",
        emulate_tty=True,
    )
    bridge = LifecycleNode(
        package="hex_arm_bridge", executable="hex_arm_bridge", name="hex_arm_bridge",
        namespace="", autostart=True,
        parameters=[_bridge_parameters(
            profile, bridge_zenoh_connect, bridge_startup_timeout_sec
        )], output="screen")
    control = Node(
        package="controller_manager", executable="ros2_control_node",
        parameters=[
            description,
            controllers,
            real_commands,
            {"hardware_components_initial_state": {
                "inactive": ["FireflyY6System"],
                "shutdown_on_initial_state_failure": True,
            }},
        ],
        output="screen")
    startup = None
    if activate_hardware:
        report = str(Path(launch_config.log_dir) / "startup-ready.json")
        startup_script = PathJoinSubstitution([
            FindPackagePrefix("hex_arm_bringup"), "lib", "hex_arm_bringup", "commission-startup-ros.py"
        ]).perform(context)
        with open(startup_script, "rb") as installed_script:
            installed_script.read(1)
        startup = ExecuteProcess(
            cmd=[
                FindExecutable(name="python3"), startup_script,
                "--profile", str(profile_path), "--allow-motion", "--activate-controllers",
                "--output", report,
                *([] if startup_ready else ["--hold-current"]),
            ],
            output="screen",
        )

    startup_handlers = [
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
        RegisterEventHandler(OnProcessExit(
            target_action=controller,
            on_exit=_shutdown_after_exit("hex_arm_controller"),
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=bridge,
            on_exit=_shutdown_after_exit("hex_arm_bridge"),
        )),
    ]

    if activate_hardware:
        startup_handlers.extend([
            RegisterEventHandler(OnStateTransition(
                target_lifecycle_node=bridge,
                goal_state="active",
                entities=[
                    LogInfo(msg="hex_arm_bridge is active: explicit hardware activation requested"),
                    control,
                ],
            )),
            RegisterEventHandler(OnProcessStart(
                target_action=control,
                on_start=[startup],
            )),
            RegisterEventHandler(OnProcessExit(
                target_action=startup,
                on_exit=_shutdown_after_failure("controller startup"),
            )),
            RegisterEventHandler(OnProcessExit(
                target_action=control,
                on_exit=_shutdown_after_exit("ros2_control_node"),
            )),
        ])
    else:
        startup_handlers.append(RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=bridge,
            goal_state="active",
            entities=[LogInfo(msg=(
                "OBSERVE MODE: ros2_control, hardware activation, and trajectory controllers "
                "remain stopped; RViz follows /hex_arm/internal/state"
            ))],
        )))

    rviz_config = PathJoinSubstitution([FindPackageShare("hex_arm_bringup"), "rviz", "firefly_y6.rviz"])
    return [
        *startup_handlers,
        controller, bridge,
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[description],
            remappings=[("joint_states", "/hex_arm/internal/state")],
        ),
        Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_config],
             condition=IfCondition(LaunchConfiguration("use_rviz"))),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("hardware_profile", default_value=""),
        DeclareLaunchArgument("zenoh_connect", default_value=""),
        DeclareLaunchArgument(
            "bridge_startup_timeout_sec",
            default_value=str(DEFAULT_BRIDGE_STARTUP_TIMEOUT_SEC),
            description=(
                "Deadline for the bridge to discover the initialized API and receive fresh state. "
                "This must cover all six real-drive initialization steps."
            ),
        ),
        DeclareLaunchArgument(
            "activate_hardware",
            default_value="false",
            choices=["true", "false"],
            description=(
                "Explicitly activate ros2_control hardware and trajectory controllers. "
                "The safe default is observation only."
            ),
        ),
        DeclareLaunchArgument(
            "startup_ready", default_value="false", choices=["true", "false"],
            description="After explicit activation, align J6 if needed and run J2 -> J4 -> J3, then hold.",
        ),
        DeclareLaunchArgument("use_rviz", default_value="true", choices=["true", "false"]),
        OpaqueFunction(function=_real_nodes),
    ])
