import math
import os
import hashlib
import json
import re
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
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare
from hex_arm_bringup.startup_event import StartupVerified


DEFAULT_ZENOH_DIRECT_ENDPOINT = "tcp/127.0.0.1:7448"
DEFAULT_HARDWARE_STARTUP_TIMEOUT_SEC = 30.0


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


def _controller_update_rate(controllers_path):
    try:
        config = yaml.safe_load(Path(controllers_path).read_text())
        rate = config["controller_manager"]["ros__parameters"]["update_rate"]
    except (OSError, KeyError, TypeError, yaml.YAMLError) as error:
        raise RuntimeError("cannot read controller_manager update_rate") from error
    if type(rate) is not int or rate <= 0:
        raise RuntimeError("controller_manager update_rate must be a positive integer")
    return rate


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


def _after_startup(report_path, profile_path, readiness_token):
    def _handler(event, context):
        if context.is_shutdown:
            return []
        if event.returncode != 0:
            return _shutdown_actions(f"controller startup failed with exit code {event.returncode}")
        try:
            report = json.loads(Path(report_path).read_text())
            expected_hash = hashlib.sha256(Path(profile_path).read_bytes()).hexdigest()
            if (not isinstance(report, dict) or report.get("passed") is not True
                    or report.get("profile_sha256") != expected_hash
                    or (readiness_token and report.get("moveit_execution_unlocked") is not True)):
                raise ValueError("startup report does not confirm this profile and execution handoff")
        except (OSError, ValueError) as error:
            return _shutdown_actions(f"controller startup report verification failed: {error}")
        if report.get("deactivated") is True:
            return [LogInfo(msg="Startup trial verified; hardware is INACTIVE")]
        return [EmitEvent(event=StartupVerified(profile_path, readiness_token))]

    return _handler


def _shutdown_actions(reason):
    return [LogInfo(msg=f"ERROR: {reason}"), EmitEvent(event=Shutdown(reason=reason))]


def _real_nodes(context):
    profile_path = Path(LaunchConfiguration("hardware_profile").perform(context))
    if not profile_path.is_file():
        raise RuntimeError("real bringup requires hardware_profile:=/absolute/path/to/verified.yaml")
    profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    activate_hardware = LaunchConfiguration("activate_hardware").perform(context).lower()
    if activate_hardware not in ("true", "false"):
        raise RuntimeError("activate_hardware must be 'true' or 'false'")
    activate_hardware = activate_hardware == "true"
    startup_ready = LaunchConfiguration("startup_ready", default="true").perform(context).lower()
    if startup_ready not in ("true", "false"):
        raise RuntimeError("startup_ready must be 'true' or 'false'")
    if activate_hardware and startup_ready != "true":
        raise RuntimeError("real execution requires the protocol startup procedure; startup_ready cannot be disabled")
    motor_protocol = profile.get("bus", {}).get("protocol")
    if motor_protocol not in ("meow", "cia402"):
        raise RuntimeError("real bringup requires an explicit meow or cia402 motor protocol")
    startup_sequence = LaunchConfiguration("startup_sequence", default="").perform(context)
    if startup_sequence and (motor_protocol != "cia402" or not Path(startup_sequence).is_file()):
        raise RuntimeError("startup_sequence requires CiA402 and an existing arm-bound recipe")
    startup_trial = LaunchConfiguration("startup_trial", default="false").perform(context).lower()
    if startup_trial not in ("true", "false"):
        raise RuntimeError("startup_trial must be true or false")
    startup_trial = startup_trial == "true"
    readiness_token = LaunchConfiguration("moveit_ready_token", default="").perform(context)
    startup_motion_limits = LaunchConfiguration("startup_motion_limits", default="").perform(context)
    if readiness_token and (not activate_hardware or not re.fullmatch(r"[0-9a-f]{32}", readiness_token)):
        raise RuntimeError("moveit_ready_token requires activation and this launch's 32-digit token")
    if startup_trial and (not activate_hardware or not startup_sequence):
        raise RuntimeError("startup_trial requires explicit activation and an arm-bound sequence")
    align_folded = LaunchConfiguration("align_folded", default="false").perform(context).lower()
    if align_folded not in ("true", "false"):
        raise RuntimeError("align_folded must be true or false")
    align_folded = align_folded == "true"
    allow_enable_transient = LaunchConfiguration("allow_enable_transient", default="false").perform(context).lower()
    if allow_enable_transient not in ("true", "false"):
        raise RuntimeError("allow_enable_transient must be true or false")
    allow_enable_transient = allow_enable_transient == "true"
    if allow_enable_transient and (motor_protocol != "meow" or not activate_hardware):
        raise RuntimeError("enable transient allowance requires explicit Meow activation")
    if align_folded and (motor_protocol != "meow" or not activate_hardware):
        raise RuntimeError("align_folded requires explicit Meow activation")
    controller_zenoh_args, client_zenoh_connect = _zenoh_routes(
        LaunchConfiguration("zenoh_connect").perform(context)
    )
    hardware_startup_timeout_sec = _positive_seconds(
        LaunchConfiguration("hardware_startup_timeout_sec").perform(context),
        "hardware_startup_timeout_sec",
    )

    if (profile.get("schema_version") != 3
            or profile.get("joint_coordinate_version") != 2
            or not profile.get("validated")):
        raise RuntimeError(
            "real bringup requires a validated schema v3 profile with "
            "joint_coordinate_version 2 and an explicit gravity_vector_base_m_s2"
        )
    if "gravity_vector_base_m_s2" not in profile:
        raise RuntimeError(
            "real bringup requires the schema v3 gravity_vector_base_m_s2 installation parameter"
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
    update_rate = _controller_update_rate(controllers.perform(context))
    description = {"robot_description": Command([
        FindExecutable(name="xacro"), " ", xacro_file, " backend:=real controllers_file:=", controllers,
        " zenoh_connect:=", client_zenoh_connect,
        " robot_prefix:=", profile["robot_prefix"], " command_period_sec:=", str(1.0 / update_rate),
        " startup_timeout_sec:=", str(hardware_startup_timeout_sec),
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
            "--urdf",
            PathJoinSubstitution([
                FindPackageShare("xpkg_urdf_firefly_y6"), "urdf", "xpkg_urdf_firefly_y6.urdf"
            ]),
            "--shutdown-report",
            os.environ.get("HEX_ARM_SHUTDOWN_REPORT",
                           str(Path(launch_config.log_dir) / "driver-shutdown.json")),
            *controller_zenoh_args,
        ],
        output="screen",
        emulate_tty=True,
    )
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
        report = os.environ.get("HEX_ARM_STARTUP_REPORT", str(Path(launch_config.log_dir) / "startup-ready.json"))
        startup_script = PathJoinSubstitution([
            FindPackagePrefix("hex_arm_bringup"), "lib", "hex_arm_bringup", "commission-startup-ros.py"
        ]).perform(context)
        with open(startup_script, "rb") as installed_script:
            installed_script.read(1)
        startup = ExecuteProcess(
            cmd=[
                FindExecutable(name="python3"), startup_script,
                "--profile", str(profile_path), "--allow-motion", "--activate-controllers",
                "--hardware-startup-timeout-sec", str(hardware_startup_timeout_sec + 5.0),
                *(["--hold-current"] if motor_protocol == "cia402" else []),
                *(["--cia402-sequence", startup_sequence] if startup_sequence else []),
                *(["--moveit", "--return-to-start", "--deactivate-after"] if startup_trial else []),
                *(["--allow-enable-transient"] if allow_enable_transient else []),
                *(["--align-folded"] if align_folded else []),
                *(["--moveit-ready-token", readiness_token] if readiness_token else []),
                *(["--motion-limits", startup_motion_limits] if startup_motion_limits else []),
                "--output", report,
            ],
            output="screen",
        )

    # The direct plugin owns the control session, services and diagnostics.
    handlers = [
        RegisterEventHandler(OnProcessExit(
            target_action=controller, on_exit=_shutdown_after_exit("hex_arm_controller"))),
        RegisterEventHandler(OnProcessExit(
            target_action=control, on_exit=_shutdown_after_exit("ros2_control_node"))),
    ]
    if activate_hardware:
        handlers.extend([
            RegisterEventHandler(OnProcessStart(target_action=control, on_start=[startup])),
            RegisterEventHandler(OnProcessExit(
                target_action=startup, on_exit=_after_startup(report, profile_path, readiness_token))),
        ])
    return [*handlers, controller, control,
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[description], remappings=[("joint_states", "/hex_arm/internal/state")]),
        Node(package="rviz2", executable="rviz2", arguments=["-d", PathJoinSubstitution([
            FindPackageShare("hex_arm_bringup"), "rviz", "firefly_y6.rviz"])],
             condition=IfCondition(LaunchConfiguration("use_rviz"))),
    ]



def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("hardware_profile", default_value=""),
        DeclareLaunchArgument("zenoh_connect", default_value=""),
        DeclareLaunchArgument("startup_sequence", default_value="",
                              description="Optional qualified CiA402 recipe; requires strict MoveIt services"),
        DeclareLaunchArgument("startup_trial", default_value="false", choices=["true", "false"],
                              description="Validate CiA402 startup and MoveIt, return to entry, then disable"),
        DeclareLaunchArgument("moveit_ready_token", default_value="",
                              description="Internal per-launch handoff to the gated MoveIt runtime"),
        DeclareLaunchArgument("startup_motion_limits", default_value="",
                              description="Internal JSON dynamics inherited from the effective MoveIt limits"),
        DeclareLaunchArgument("allow_enable_transient", default_value="false", choices=["true", "false"],
                              description="Explicit Meow enable/ramp allowance: 0.15 rad/s for 0.25 s, then 0.05; settle before trajectories"),
        DeclareLaunchArgument("align_folded", default_value="false", choices=["true", "false"],
                              description="Explicit bounded Meow base/wrist alignment before unfolding"),
        DeclareLaunchArgument(
            "hardware_startup_timeout_sec",
            default_value=str(DEFAULT_HARDWARE_STARTUP_TIMEOUT_SEC),
            description=(
                "Deadline for the direct transport to discover the initialized API and receive fresh state. "
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
            "startup_ready", default_value="true", choices=["true", "false"],
            description=(
                "Mandatory for real activation: Meow runs the verified J2 -> J4 -> J3 "
                "sequence; CiA402 verifies and holds the measured pose."
            ),
        ),
        DeclareLaunchArgument("use_rviz", default_value="true", choices=["true", "false"]),
        OpaqueFunction(function=_real_nodes),
    ])
