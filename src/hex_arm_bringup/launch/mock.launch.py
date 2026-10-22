"""An independently namespaced arm, or controllers for an existing robot manager."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import xacro

from hex_arm_bringup.launch_support import (
    arm_faulted, boolean, failure_actions, instance_arguments, instance_from_context,
    stop_owned_processes, value,
)
from hex_arm_bringup.model import write_controller_config, write_rviz_config


def _launch_setup(context):
    instance = instance_from_context(context)
    sim_time = boolean(context, "use_sim_time")
    broadcaster_name = value(context, "state_broadcaster_name", "joint_state_broadcaster")
    shutdown_scope = value(context, "shutdown_scope", "launch")
    bringup = Path(get_package_share_directory("hex_arm_bringup"))
    source = value(context, "description_file") or str(
        Path(get_package_share_directory("hex_arm_description")) / "urdf/firefly_y6.urdf.xacro")
    controllers = write_controller_config(
        bringup / "config/controllers.yaml", namespace=instance.namespace,
        joint_prefix=instance.joint_prefix, controller_name=instance.controller_name,
        controller_manager=instance.controller_manager, use_sim_time=sim_time,
        state_broadcaster_name=broadcaster_name,
    )
    description = {"robot_description": xacro.process_file(source, mappings={
        "backend": "mock", "prefix": instance.joint_prefix,
        "hardware_name": instance.hardware_name, "ros_namespace": instance.namespace,
        "initial_positions_file": value(context, "initial_positions_file"),
    }).toxml(), "use_sim_time": sim_time}
    manager_path = instance.ros_name(instance.controller_manager)
    manager_ns, manager_name = manager_path.rsplit("/", 1)
    control = Node(package="controller_manager", executable="ros2_control_node",
                   namespace=manager_ns, name=manager_name,
                   remappings=[("robot_description", instance.ros_name("robot_description"))],
                   parameters=[description, controllers], output="screen")
    publisher = Node(package="robot_state_publisher", executable="robot_state_publisher",
                     namespace=instance.namespace, parameters=[description], output="screen",
                     remappings=[("joint_states", manager_ns + "/joint_states")])
    description_publisher = Node(package="hex_arm_bringup", executable="robot_description_publisher",
                                 namespace=instance.namespace, parameters=[description], output="screen")
    state = Node(package="controller_manager", executable="spawner", namespace=instance.namespace,
                 arguments=[broadcaster_name, "--controller-manager", manager_path,
                            "--param-file", controllers], output="screen")
    arm = Node(package="controller_manager", executable="spawner", namespace=instance.namespace,
               arguments=[instance.controller_name, "--controller-manager", manager_path,
                          "--param-file", controllers], output="screen")
    rviz = Node(package="rviz2", executable="rviz2", namespace=instance.namespace,
                parameters=[{"use_sim_time": sim_time}],
                arguments=["-d", write_rviz_config(bringup / "rviz/firefly_y6.rviz",
                    namespace=instance.namespace, joint_prefix=instance.joint_prefix, use_sim_time=sim_time)])
    owned = [state, arm]
    if boolean(context, "start_controller_manager", True):
        owned.append(control)
    if boolean(context, "start_robot_state_publisher", True):
        owned.append(publisher)
    elif boolean(context, "start_controller_manager", True):
        owned.append(description_publisher)
    if boolean(context, "use_rviz", True):
        owned.append(rviz)

    def finished(event, callback_context, next_actions, step):
        if arm_faulted(callback_context, instance.namespace):
            return []
        return next_actions if event.returncode == 0 else failure_actions(
            f"{step} failed ({event.returncode})", instance.namespace, shutdown_scope)

    actions = [
        stop_owned_processes(instance.namespace, owned, request_deactivation=False),
        RegisterEventHandler(OnProcessExit(target_action=state, on_exit=lambda e, c: finished(e, c, [arm], "state broadcaster"))),
        RegisterEventHandler(OnProcessExit(target_action=arm, on_exit=lambda e, c: finished(e, c, [], "arm controller"))),
    ]
    if control in owned:
        actions += [RegisterEventHandler(OnProcessExit(target_action=control, on_exit=lambda e, c: [] if c.is_shutdown else failure_actions(
            "mock controller manager exited", instance.namespace, shutdown_scope))), control]
    actions += [action for action in (publisher, description_publisher, rviz) if action in owned]
    actions.append(state if boolean(context, "start_joint_state_broadcaster", True) else arm)
    return actions


def generate_launch_description():
    return LaunchDescription(instance_arguments() + [
        DeclareLaunchArgument("use_rviz", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("use_sim_time", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("start_controller_manager", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("start_robot_state_publisher", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("start_joint_state_broadcaster", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("state_broadcaster_name", default_value="joint_state_broadcaster"),
        DeclareLaunchArgument("description_file", default_value=""),
        DeclareLaunchArgument("initial_positions_file", default_value=""),
        DeclareLaunchArgument("shutdown_scope", default_value="launch", choices=["launch", "arm"]),
        OpaqueFunction(function=_launch_setup),
    ])
