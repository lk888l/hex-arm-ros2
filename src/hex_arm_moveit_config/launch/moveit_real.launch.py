import os
import math
import json
import xml.etree.ElementTree as ET
import uuid

import yaml
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import EventHandler, LaunchContext, LaunchDescription
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
from hex_arm_bringup.startup_event import StartupVerified


STATE_TOPIC = "/hex_arm/internal/state"
JOINT_NAMES = tuple(f"joint_{index}" for index in range(1, 7))
DYNAMICS_FILES = {
    "commissioning": "config/joint_limits_commissioning.yaml",
    "hardware": "config/joint_limits_hardware.yaml",
}


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


def _dynamics_file(dynamics_limits, planning_limits_file):
    if dynamics_limits == "custom":
        path = Path(planning_limits_file)
        if not planning_limits_file or not path.is_absolute() or not path.is_file():
            raise RuntimeError("custom dynamics require an existing absolute planning_limits_file")
        return str(path)
    if dynamics_limits not in DYNAMICS_FILES:
        raise RuntimeError("dynamics_limits must be commissioning, hardware or custom")
    if planning_limits_file:
        raise RuntimeError("planning_limits_file requires dynamics_limits:=custom")
    return DYNAMICS_FILES[dynamics_limits]


def _positive(value, label, maximum=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RuntimeError(f"{label} must be a finite positive number")
    result = float(value)
    if not math.isfinite(result) or result <= 0 or (maximum is not None and result > maximum):
        raise RuntimeError(f"{label} must be finite and within (0, {maximum or 'infinity'}]")
    return result


def _restrict_planning_limits(moveit_config, profile, position_limits="commissioning",
                              dynamics_limits="commissioning"):
    """Intersect SI planning limits with the URDF and independent Rust profile."""
    if position_limits not in ("commissioning", "hardware"):
        raise RuntimeError("position_limits must be commissioning or hardware")
    if dynamics_limits not in (*DYNAMICS_FILES, "custom"):
        raise RuntimeError("dynamics_limits must be commissioning, hardware or custom")
    parameters = moveit_config.joint_limits["robot_description_planning"]
    planning = parameters["joint_limits"]
    if dynamics_limits != "hardware" and set(planning) != set(JOINT_NAMES):
        raise RuntimeError("planning dynamics require exactly the configured six joints")
    for name in ("default_velocity_scaling_factor", "default_acceleration_scaling_factor"):
        parameters[name] = _positive(parameters.get(name, 1.0), name, 1.0)
    model_limits = {}
    model = ET.fromstring(moveit_config.robot_description["robot_description"])
    for element in model.findall("joint"):
        limit = element.find("limit")
        if element.get("name") in JOINT_NAMES and limit is not None:
            model_limits[element.get("name")] = (
                float(limit.attrib["lower"]), float(limit.attrib["upper"]),
                _positive(float(limit.attrib["velocity"]), "URDF velocity"))
    if set(model_limits) != set(JOINT_NAMES):
        raise RuntimeError("planning limits require six bounded URDF joints")
    commissioning = {}
    if position_limits == "commissioning":
        path = Path(get_package_share_directory("hex_arm_moveit_config")) / DYNAMICS_FILES["commissioning"]
        with path.open(encoding="utf-8") as stream:
            commissioning = yaml.safe_load(stream)["joint_limits"]
    joints = profile.get("joints", [])
    names = [joint["name"] for joint in joints]
    if len(names) != len(JOINT_NAMES) or set(names) != set(JOINT_NAMES):
        raise RuntimeError("hardware planning limits require exactly the configured six joints")
    merged = {}
    for joint in joints:
        limits = joint["limits"]
        # Copy each entry: ordinary MoveIt YAML files may alias all six limits.
        target = dict(planning.get(joint["name"], {}))
        model_lower, model_upper, model_velocity = model_limits[joint["name"]]
        if position_limits == "commissioning":
            window = commissioning[joint["name"]]
            model_lower = max(model_lower, float(window["min_position"]))
            model_upper = min(model_upper, float(window["max_position"]))
        lower = max(model_lower, float(limits["position_lower_rad"]))
        upper = min(model_upper, float(limits["position_upper_rad"]))
        velocity = min(model_velocity, _positive(limits["velocity_rad_s"], "hardware velocity_rad_s"))
        acceleration = _positive(limits["acceleration_rad_s2"], "hardware acceleration_rad_s2")
        if dynamics_limits != "hardware":
            if target.get("has_velocity_limits") is not True or target.get("has_acceleration_limits") is not True:
                raise RuntimeError(f"planning dynamics must enable velocity and acceleration limits for {joint['name']}")
            velocity = min(velocity, _positive(target.get("max_velocity"), "planning max_velocity"))
            acceleration = min(acceleration, _positive(target.get("max_acceleration"), "planning max_acceleration"))
        values = [*map(float, (limits["position_lower_rad"], limits["position_upper_rad"],
                              limits["velocity_rad_s"], limits["acceleration_rad_s2"])),
                  model_lower, model_upper, lower, upper, velocity, acceleration]
        if not all(math.isfinite(v) for v in values) or lower >= upper or min(velocity, acceleration) <= 0:
            raise RuntimeError(f"empty or invalid planning limit intersection for {joint['name']}")
        target.update(has_position_limits=True, min_position=lower, max_position=upper,
                      has_velocity_limits=True, max_velocity=velocity,
                      has_acceleration_limits=True, max_acceleration=acceleration)
        merged[joint["name"]] = target
    parameters["joint_limits"] = merged


def _build_moveit_config(enable_execution: bool, hardware_profile=None, position_limits="commissioning",
                         dynamics_limits="commissioning", planning_limits_file=""):
    if position_limits not in ("commissioning", "hardware"):
        raise RuntimeError("position_limits must be commissioning or hardware")
    dynamics_file = _dynamics_file(dynamics_limits, planning_limits_file)
    if (position_limits == "hardware" or dynamics_limits != "commissioning") and not hardware_profile:
        raise RuntimeError("hardware/custom planning limits require an explicit hardware profile")
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
        .joint_limits(file_path=dynamics_file)
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
            _restrict_planning_limits(moveit_config, yaml.safe_load(profile_file), position_limits, dynamics_limits)
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
    hardware_profile: str, zenoh_connect: str, enable_execution: bool, startup_ready: bool = True,
    startup_sequence: str = "",
    startup_trial: bool = False,
    align_folded: bool = False,
    allow_enable_transient: bool = False,
    readiness_token: str = "",
    startup_motion_limits: str = "",
) -> dict[str, str]:
    if enable_execution and not startup_ready:
        raise RuntimeError("real execution requires the protocol startup procedure")
    if allow_enable_transient and not enable_execution:
        raise RuntimeError("enable transient allowance requires explicit execution")
    if align_folded and not enable_execution:
        raise RuntimeError("folded alignment requires explicit execution")
    if startup_trial and (not enable_execution or not startup_sequence):
        raise RuntimeError("startup trial requires explicit execution and an arm-bound sequence")
    return {
        "hardware_profile": hardware_profile,
        "zenoh_connect": zenoh_connect,
        # One switch controls both sides of the execution boundary: MoveIt can
        # execute only when the hardware and trajectory controller are active.
        "activate_hardware": "true" if enable_execution else "false",
        "startup_ready": "true" if enable_execution and startup_ready else "false",
        **({"startup_sequence": startup_sequence} if startup_sequence else {}),
        **({"startup_trial": "true"} if startup_trial else {}),
        **({"allow_enable_transient": "true"} if allow_enable_transient else {}),
        **({"align_folded": "true"} if align_folded else {}),
        **({"moveit_ready_token": readiness_token} if readiness_token else {}),
        **({"startup_motion_limits": startup_motion_limits} if startup_motion_limits else {}),
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
    position_limits = LaunchConfiguration("position_limits", default="commissioning").perform(context)
    dynamics_limits = LaunchConfiguration("dynamics_limits", default="commissioning").perform(context)
    planning_limits_file = LaunchConfiguration("planning_limits_file", default="").perform(context)
    startup_sequence = LaunchConfiguration("startup_sequence", default="").perform(context)
    startup_trial = LaunchConfiguration("startup_trial", default="false").perform(context).lower()
    if startup_trial not in ("true", "false"):
        raise RuntimeError("startup_trial must be true or false")
    startup_trial = startup_trial == "true"
    align_folded = LaunchConfiguration("align_folded", default="false").perform(context).lower()
    if align_folded not in ("true", "false"):
        raise RuntimeError("align_folded must be true or false")
    align_folded = align_folded == "true"
    allow_enable_transient = LaunchConfiguration("allow_enable_transient", default="false").perform(context).lower()
    if allow_enable_transient not in ("true", "false"):
        raise RuntimeError("allow_enable_transient must be true or false")
    allow_enable_transient = allow_enable_transient == "true"

    bringup_share = Path(get_package_share_directory("hex_arm_bringup"))
    moveit_share = Path(get_package_share_directory("hex_arm_moveit_config"))
    moveit_config = _build_moveit_config(enable_execution, hardware_profile, position_limits,
                                       dynamics_limits, planning_limits_file)
    readiness_token = uuid.uuid4().hex if enable_execution else ""
    startup_motion_limits = ""
    if enable_execution:
        planning = moveit_config.joint_limits["robot_description_planning"]["joint_limits"]
        startup_motion_limits = json.dumps({
            name: {"velocity_rad_s": planning[name]["max_velocity"],
                   "acceleration_rad_s2": planning[name]["max_acceleration"]}
            for name in JOINT_NAMES
        })

    real_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(bringup_share / "launch" / "real.launch.py")
        ),
        launch_arguments=_bringup_arguments(
            hardware_profile, zenoh_connect, enable_execution, startup_ready, startup_sequence,
            startup_trial, align_folded, allow_enable_transient, readiness_token, startup_motion_limits
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
            {"startup_readiness_token": readiness_token},
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
        LogInfo(msg=f"MoveIt limits: positions={position_limits}, dynamics={dynamics_limits}, "
                    f"effective SI limits={moveit_config.joint_limits['robot_description_planning']}"),
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
    if enable_execution:
        # A startup trajectory controller is already active during the ten-second
        # ready hold. The runtime gates its action servers; open the UI only after
        # this exact startup process has also written its successful report.
        actions.append(LogInfo(msg="MoveIt startup in progress: execution and RViz wait for verified hold"))
        actions.append(RegisterEventHandler(EventHandler(
            matcher=lambda event: (isinstance(event, StartupVerified)
                                   and Path(event.profile) == Path(hardware_profile)
                                   and event.readiness_token == readiness_token),
            entities=[LogInfo(msg="MoveIt startup verified: execution available"
                                  + ("; opening RViz" if use_rviz else "")), rviz],
        )))
    else:
        actions.append(rviz)
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("allow_enable_transient", default_value="false", choices=["true", "false"],
                                  description="Explicit Meow enable/ramp allowance: 0.15 rad/s for 0.25 s, then 0.05; settle before trajectories"),
            DeclareLaunchArgument("align_folded", default_value="false", choices=["true", "false"],
                                  description="Explicit bounded Meow base/wrist alignment"),
            DeclareLaunchArgument("startup_sequence", default_value="",
                                  description="Optional arm-bound CiA402 unfolded startup recipe"),
            DeclareLaunchArgument("startup_trial", default_value="false", choices=["true", "false"],
                                  description="Validate startup and MoveIt, return to entry, then disable"),
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
                "position_limits",
                default_value="commissioning",
                choices=["commissioning", "hardware"],
                description=(
                    "commissioning retains the original narrow position window; hardware uses "
                    "the selected hardware profile intersected with URDF travel. "
                    "Select planning speed/acceleration separately with dynamics_limits."
                ),
            ),
            DeclareLaunchArgument(
                "dynamics_limits", default_value="commissioning",
                choices=["commissioning", "hardware", "custom"],
                description=("commissioning retains 0.1 rad/s and 0.1 rad/s^2 caps; hardware uses "
                             "profile dynamics and URDF velocity; custom additionally intersects "
                             "planning_limits_file. All values are joint-side SI units."),
            ),
            DeclareLaunchArgument(
                "planning_limits_file", default_value="",
                description="Absolute MoveIt joint-limits YAML for dynamics_limits:=custom, including default scaling factors.",
            ),
            DeclareLaunchArgument(
                "startup_ready",
                default_value="true",
                choices=["true", "false"],
                description=(
                    "Required for execution: Meow runs the verified J2 -> J4 -> J3 fold exit; "
                    "CiA402 verifies and holds the measured pose."
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
