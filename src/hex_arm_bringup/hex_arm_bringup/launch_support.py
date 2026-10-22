"""Composition helpers; a subsystem fault never implies a global launch exit."""
import signal

from launch import Event
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handler import EventHandler
from launch.events import Shutdown, matches_action
from launch.events.process import SignalProcess
from launch.substitutions import LaunchConfiguration

from .integration import InstanceNames


class ArmFault(Event):
    def __init__(self, namespace, reason):
        super().__init__()
        self.namespace = namespace
        self.reason = reason


def value(context, name, default=""):
    return LaunchConfiguration(name, default=str(default)).perform(context)


def boolean(context, name, default=False):
    result = value(context, name, str(default).lower()).lower()
    if result not in ("true", "false"):
        raise ValueError(f"{name} must be true or false")
    return result == "true"


def instance_from_context(context):
    return InstanceNames(**{key: value(context, key, default)
                            for key, default in InstanceNames().as_dict().items()})


def instance_arguments():
    return [DeclareLaunchArgument(key, default_value=default)
            for key, default in InstanceNames().as_dict().items()]


def failure_actions(reason, namespace="", scope="launch"):
    if scope not in ("launch", "arm"):
        raise ValueError("shutdown_scope must be launch or arm")
    event = Shutdown(reason=reason) if scope == "launch" else ArmFault(namespace, reason)
    return [LogInfo(msg=reason), EmitEvent(event=event)]


def arm_faulted(context, namespace):
    return context.is_shutdown or namespace in getattr(context, "_hex_arm_faulted_namespaces", set())


def stop_owned_processes(namespace, processes, request_deactivation=True):
    """Stop only actions owned by this arm; external managers remain alive.

    Both the driver and its ROS clients receive SIGINT. Driver teardown remains
    responsible for the motor disable acknowledgment. Never initiate a return
    trajectory after an unexpected process exit.
    """
    stopped = False

    def on_fault(context):
        nonlocal stopped
        if stopped or context.is_shutdown:
            return []
        stopped = True
        # Keep the marker outside scoped launch substitutions: a later spawner
        # exit or readiness event must not resurrect an arm after its fault.
        if not hasattr(context, "_hex_arm_faulted_namespaces"):
            context._hex_arm_faulted_namespaces = set()
        context._hex_arm_faulted_namespaces.add(namespace)
        disable = ExecuteProcess(cmd=["timeout", "15", "ros2", "service", "call",
            f"{namespace}/hex_arm/deactivate_hardware", "std_srvs/srv/Trigger", "{}"], output="screen")
        return [*([disable] if request_deactivation else []), *[EmitEvent(event=SignalProcess(
            signal_number=signal.SIGINT, process_matcher=matches_action(process)))
                for process in processes]]

    return RegisterEventHandler(EventHandler(
        matcher=lambda event: isinstance(event, ArmFault) and event.namespace == namespace,
        entities=[OpaqueFunction(function=on_fault)]))
