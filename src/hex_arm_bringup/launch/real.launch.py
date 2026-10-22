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
from hex_arm_bringup.integration import InstanceNames, verify_report_instance
from hex_arm_bringup.launch_support import (
    arm_faulted, boolean, failure_actions, instance_arguments, instance_from_context,
    stop_owned_processes, value,
)
from hex_arm_bringup.model import write_controller_config, write_rviz_config
import xacro


DEFAULT_ZENOH_DIRECT_ENDPOINT = "tcp/127.0.0.1:7448"
DEFAULT_HARDWARE_STARTUP_TIMEOUT_SEC = 30.0


def _zenoh_routes(requested_connect, requested_listen=""):
    connect = str(requested_connect).strip()
    if connect and requested_listen:
        raise RuntimeError("choose zenoh_connect or zenoh_listen, not both")
    if connect:
        return ["--zenoh-connect", connect], connect
    endpoint = requested_listen or DEFAULT_ZENOH_DIRECT_ENDPOINT
    return ["--zenoh-listen", endpoint], endpoint


def _positive_seconds(value, name):
    try:
        seconds = float(value)
    except (TypeError, ValueError) as error:
        raise RuntimeError(f"{name} must be a finite positive number") from error
    if not math.isfinite(seconds) or seconds <= 0.0:
        raise RuntimeError(f"{name} must be a finite positive number")
    return seconds


def _controller_update_rate(controllers_path, manager="/controller_manager"):
    try:
        config = yaml.safe_load(Path(controllers_path).read_text())
        parameters = {}
        # ROS parameter files can address this node by FQN, a relative path,
        # a basename, or a wildcard. Prefer the most specific declaration.
        for key in dict.fromkeys(("/**", manager.rsplit("/", 1)[-1], manager.lstrip("/"), manager)):
            parameters.update(config.get(key, {}).get("ros__parameters", {}))
        rate = parameters["update_rate"]
    except (OSError, KeyError, TypeError, AttributeError, yaml.YAMLError) as error:
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


def _shutdown_after_exit(step, namespace="", scope="launch"):
    def _handler(event, context):
        if context.is_shutdown:
            return []
        if event.returncode == 0:
            reason = f"critical process {step} exited cleanly"
            return failure_actions(reason, namespace, scope)
        reason = f"critical process {step} exited with code {event.returncode}"
        return failure_actions(f"ERROR: {reason}", namespace, scope)

    return _handler


def _after_startup(report_path, profile_path, readiness_token, instance=None, scope="launch"):
    namespace = instance.namespace if instance else ""
    def _handler(event, context):
        if arm_faulted(context, namespace):
            return []
        if event.returncode != 0:
            return failure_actions(f"controller startup failed with exit code {event.returncode}", namespace, scope)
        try:
            report = json.loads(Path(report_path).read_text())
            expected_hash = hashlib.sha256(Path(profile_path).read_bytes()).hexdigest()
            if (not isinstance(report, dict) or report.get("passed") is not True
                    or report.get("profile_sha256") != expected_hash
                    or (readiness_token and report.get("moveit_execution_unlocked") is not True)):
                raise ValueError("startup report does not confirm this profile and execution handoff")
            if instance is not None:
                verify_report_instance(report, instance)
        except (OSError, ValueError, TypeError, AttributeError, RuntimeError) as error:
            return failure_actions(f"controller startup report verification failed: {error}", namespace, scope)
        if report.get("deactivated") is True:
            return [LogInfo(msg="Startup trial verified; hardware is INACTIVE")]
        return [EmitEvent(event=StartupVerified(profile_path, readiness_token, namespace))]

    return _handler


def _shutdown_actions(reason):
    return [LogInfo(msg=f"ERROR: {reason}"), EmitEvent(event=Shutdown(reason=reason))]


def _real_nodes(context):
    instance = instance_from_context(context)
    scope = value(context, "shutdown_scope", "launch")
    if scope not in ("launch", "arm"):
        raise ValueError("shutdown_scope must be launch or arm")
    start_driver = boolean(context, "start_driver", True)
    start_manager = boolean(context, "start_controller_manager", True)
    publish_description = boolean(context, "publish_robot_description", True)
    description_topic = value(context, "robot_description_topic")
    if start_manager and not publish_description and not description_topic:
        raise ValueError("an external model publisher requires robot_description_topic")
    if boolean(context, "use_sim_time", False):
        raise ValueError("real hardware requires use_sim_time:=false")
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
    motor_protocol = profile.get("bus", {}).get("protocol", "meow")
    if motor_protocol not in ("meow", "cia402"):
        raise RuntimeError("real bringup requires meow (default) or explicit legacy cia402 motor protocol")
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
        LaunchConfiguration("zenoh_connect").perform(context), value(context, "zenoh_listen")
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
    controller_file = value(context, "controllers_file") or write_controller_config(
        controllers.perform(context), overrides_path=real_commands.perform(context),
        namespace=instance.namespace, joint_prefix=instance.joint_prefix,
        controller_name=instance.controller_name, controller_manager=instance.controller_manager)
    configured_rate = _controller_update_rate(controller_file, instance.ros_name(instance.controller_manager))
    requested_rate = value(context, "controller_update_rate")
    if not start_manager and not requested_rate:
        raise ValueError("an external controller manager requires its controller_update_rate")
    if requested_rate:
        if not requested_rate.isdecimal() or int(requested_rate) <= 0:
            raise ValueError("controller_update_rate must be a positive integer")
        update_rate = int(requested_rate)
        if start_manager and update_rate != configured_rate:
            raise ValueError("controller_update_rate differs from the loaded controller parameter file")
    else:
        update_rate = configured_rate
    supplied_description = value(context, "robot_description_file")
    if supplied_description:
        description_xml = Path(supplied_description).read_text(encoding="utf-8")
        _validate_external_description(description_xml, instance, profile, client_zenoh_connect, update_rate)
    else:
        description_xml = xacro.process_file(xacro_file.perform(context), mappings={
            "backend": "real", "prefix": instance.joint_prefix,
            "hardware_name": instance.hardware_name, "ros_namespace": instance.namespace,
            "zenoh_connect": client_zenoh_connect, "robot_prefix": profile["robot_prefix"],
            "command_period_sec": str(1.0 / update_rate),
            "startup_timeout_sec": str(hardware_startup_timeout_sec),
        }).toxml()
    description = {"robot_description": description_xml}
    # Each instance owns its receipts, including when several arms share a launch.
    run_directory = Path(launch_config.log_dir) / (instance.namespace.strip("/").replace("/", "_") or "arm")
    run_directory.mkdir(parents=True, exist_ok=True)
    report = value(context, "startup_report_file") or os.environ.get(
        "HEX_ARM_STARTUP_REPORT", str(run_directory / "startup-ready.json"))
    shutdown_report = value(context, "shutdown_report_file") or os.environ.get(
        "HEX_ARM_SHUTDOWN_REPORT", str(run_directory / "driver-shutdown.json"))
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
            shutdown_report,
            *controller_zenoh_args,
        ],
        output="screen",
        emulate_tty=True,
    )
    control = Node(
        package="controller_manager", executable="ros2_control_node",
        name=instance.ros_name(instance.controller_manager).rsplit("/", 1)[1],
        namespace=instance.ros_name(instance.controller_manager).rsplit("/", 1)[0] or "/",
        remappings=[("robot_description", description_topic or instance.ros_name("robot_description"))],
        parameters=[
            description,
            controller_file,
            {"hardware_components_initial_state": {
                "inactive": [instance.hardware_name],
                "shutdown_on_initial_state_failure": True,
            }},
        ],
        output="screen")
    startup = None
    if activate_hardware:
        startup_script = PathJoinSubstitution([
            FindPackagePrefix("hex_arm_bringup"), "lib", "hex_arm_bringup", "commission-startup-ros.py"
        ]).perform(context)
        with open(startup_script, "rb") as installed_script:
            installed_script.read(1)
        startup = ExecuteProcess(
            cmd=[
                FindExecutable(name="python3"), startup_script,
                "--profile", str(profile_path), "--allow-motion", "--activate-controllers",
                *instance.cli_args(),
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

    # The direct plugin owns control. Shared manager/model processes are optional.
    owned = []
    handlers = []
    if start_driver:
        owned.append(controller)
        handlers.append(RegisterEventHandler(OnProcessExit(
            target_action=controller, on_exit=_shutdown_after_exit("hex_arm_controller", instance.namespace, scope))))
    if start_manager:
        owned.append(control)
        handlers.append(RegisterEventHandler(OnProcessExit(
            target_action=control, on_exit=_shutdown_after_exit("ros2_control_node", instance.namespace, scope))))
    if activate_hardware:
        handlers.append(RegisterEventHandler(OnProcessExit(
            target_action=startup, on_exit=_after_startup(report, profile_path, readiness_token, instance, scope))))
        # The client's INACTIVE/fresh-feedback wait also covers external managers.
        owned.append(startup)
    if publish_description:
        owned.append(Node(package="robot_state_publisher", executable="robot_state_publisher",
             namespace=instance.namespace, parameters=[description],
             remappings=[("joint_states", instance.ros_name("hex_arm/internal/state"))]))
    rviz_config = PathJoinSubstitution([
        FindPackageShare("hex_arm_bringup"), "rviz", "firefly_y6.rviz"]).perform(context)
    if instance.namespace or instance.joint_prefix or description_topic:
        rviz_config = write_rviz_config(rviz_config, namespace=instance.namespace,
            joint_prefix=instance.joint_prefix, robot_description_topic=description_topic)
    owned.append(Node(package="rviz2", executable="rviz2", namespace=instance.namespace,
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("use_rviz"))))
    if boolean(context, "use_operation_server", True):
        owned.append(Node(package="hex_arm_bringup", executable="hex_arm_operations",
            namespace=instance.namespace, parameters=[{
                **instance.as_dict(), "hardware_profile": str(profile_path),
                "startup_report_file": report, "shutdown_report_file": shutdown_report,
            }], output="screen"))
    return [*handlers, stop_owned_processes(instance.namespace, owned), *owned]


def _validate_external_description(xml, instance, profile, endpoint, update_rate=None):
    import xml.etree.ElementTree as ET
    root = ET.fromstring(xml)
    matches = [item for item in root.findall("ros2_control") if item.get("name") == instance.hardware_name]
    if len(matches) != 1:
        raise ValueError("external description must contain exactly this hardware instance")
    component = matches[0]
    if component.findtext("hardware/plugin") != "hex_arm_hardware/HexArmSystem":
        raise ValueError("external description must use HexArmSystem for this arm")
    params = {item.get("name"): item.text or "" for item in component.findall("hardware/param")}
    params["ros_namespace"] = InstanceNames(namespace=params.get("ros_namespace", "")).namespace
    for key, expected in (("ros_namespace", instance.namespace), ("robot_prefix", profile["robot_prefix"]),
                          ("zenoh_connect", endpoint)):
        if params.get(key, "") != expected:
            raise ValueError(f"external description {key} differs from this arm configuration")
    if [item.get("name") for item in component.findall("joint")] != instance.joint_names:
        raise ValueError("external description joints differ from this arm instance")
    if update_rate is not None:
        period = float(params.get("command_period_sec", "nan"))
        if not math.isclose(period, 1.0 / update_rate, rel_tol=1e-6):
            raise ValueError("external description command_period_sec differs from controller_update_rate")



def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        *instance_arguments(),
        DeclareLaunchArgument("zenoh_listen", default_value="",
                              description="Dedicated daemon listener; assign different ports to separate arms"),
        DeclareLaunchArgument("shutdown_scope", default_value="launch", choices=["launch", "arm"]),
        DeclareLaunchArgument("start_driver", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("start_controller_manager", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("publish_robot_description", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("robot_description_file", default_value="",
                              description="Optional expanded whole-robot URDF, including this configured hardware"),
        DeclareLaunchArgument("robot_description_topic", default_value="",
                              description="Existing model topic when using an external robot_state_publisher"),
        DeclareLaunchArgument("controllers_file", default_value="",
                              description="Optional ROS controller parameter file; external managers must load it themselves"),
        DeclareLaunchArgument("controller_update_rate", default_value="",
                              description="Actual manager rate in Hz; required for an external manager"),
        DeclareLaunchArgument("use_sim_time", default_value="false", choices=["false"]),
        DeclareLaunchArgument("use_operation_server", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("startup_report_file", default_value=""),
        DeclareLaunchArgument("shutdown_report_file", default_value=""),
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
