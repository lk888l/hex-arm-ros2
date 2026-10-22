"""Real ROS actions and fake driver/stop helpers; no controller or CAN is opened."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
from types import SimpleNamespace
import uuid

import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from action_msgs.msg import GoalStatus
from hex_arm_msgs.action import StopArm
from hex_arm_msgs.msg import ArmStatus, DriverState
from std_srvs.srv import Trigger

from hex_arm_bringup.integration import InstanceNames
from hex_arm_bringup.operation_lock import OperationLock
from hex_arm_bringup.operations import ArmOperations


def wait_for(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = predicate()
        if result:
            return result
        time.sleep(0.01)
    raise AssertionError("condition did not become true before steady deadline")


def completed(future):
    wait_for(future.done)
    return future.result()


@pytest.fixture
def arms(tmp_path):
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=8)
    nodes, servers, clients = [], [], []
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def create(*, delay=0.05, report_passed=True, child_exit=0, disable_success=True,
               disable_mode=DriverState.MODE_DISABLED, retain_session=False):
        namespace = "/test_arm_" + uuid.uuid4().hex
        instance = InstanceNames(namespace=namespace, joint_prefix="test_", hardware_name="TestArm")
        directory = tmp_path / namespace.strip("/")
        directory.mkdir()
        profile = directory / "profile.yaml"
        profile.write_text("bus: {protocol: meow}\ncontroller: {}\n")
        startup = directory / "startup.json"
        receipt = directory / "driver-shutdown.json"
        state = SimpleNamespace(mode=DriverState.MODE_ACTIVE, session=True, fault=False,
                                disable_calls=0, disable_success=disable_success)
        parameters = {**instance.as_dict(), "hardware_profile": str(profile),
                      "startup_report_file": str(startup), "shutdown_report_file": str(receipt),
                      "service_timeout_sec": 0.7, "stop_timeout_sec": 4.0}
        def command(output):
            report = {"passed": report_passed, "deactivated": True,
                      "profile_sha256": hashlib.sha256(profile.read_bytes()).hexdigest(),
                      "startup_report": str(startup), "instance": instance.as_dict()}
            code = ("import pathlib,sys,time; time.sleep(float(sys.argv[1])); "
                    "pathlib.Path(sys.argv[2]).write_text(sys.argv[3]); sys.exit(int(sys.argv[4]))")
            return [sys.executable, "-c", code, str(delay), str(output), json.dumps(report), str(child_exit)]
        server = ArmOperations(namespace=namespace, parameter_overrides=[
            Parameter(name, value=value) for name, value in parameters.items()], command_builder=command)
        io = Node("fake_driver", namespace=namespace)
        publisher = io.create_publisher(DriverState, "hex_arm/driver_state", 10)
        def publish():
            message = DriverState()
            message.stamp = io.get_clock().now().to_msg()
            message.mode, message.session_owned = state.mode, state.session
            message.fault_latched = state.fault
            message.profile_valid = message.calibrated = message.all_motors_online = message.feedback_fresh = True
            publisher.publish(message)
        def disable(_, response):
            state.disable_calls += 1
            response.success = state.disable_success
            response.message = "fake drive disable confirmed" if state.disable_success else "fake disable failed"
            if state.disable_success:
                state.mode, state.session = disable_mode, retain_session
            return response
        service = io.create_service(Trigger, "hex_arm/deactivate_hardware", disable)
        timer = io.create_timer(0.02, publish)
        client = ActionClient(io, StopArm, "hex_arm/stop")
        for node in (server, io):
            nodes.append(node)
            executor.add_node(node)
        servers.append(server)
        clients.append(client)
        assert client.wait_for_server(timeout_sec=3)
        arm = SimpleNamespace(server=server, io=io, state=state, client=client, timer=timer,
                              service=service, instance=instance, profile=profile, startup=startup,
                              parameters=parameters)
        def evidence(**overrides):
            report = {"instance": instance.as_dict(), "passed": True, "ready_hold": {},
                      "profile_sha256": hashlib.sha256(profile.read_bytes()).hexdigest()}
            report.update(overrides)
            startup.write_text(json.dumps(report))
        arm.evidence = evidence
        arm.ready = lambda: (evidence(), wait_for(lambda: server.readiness()[0]))
        return arm

    yield create
    for server in servers:
        server.close()
    for client in clients:
        client.destroy()
    executor.shutdown(timeout_sec=5)
    thread.join(timeout=5)
    for node in nodes:
        node.destroy_node()
    rclpy.shutdown()


def goal(arm, mode=StopArm.Goal.NORMAL, feedback=None):
    return completed(arm.client.send_goal_async(StopArm.Goal(mode=mode), feedback_callback=feedback))


def test_normal_requires_current_matching_startup_and_fresh_driver(arms):
    arm = arms()
    assert not goal(arm).accepted
    arm.startup.write_text("[]")
    assert not goal(arm).accepted
    arm.evidence()
    old = arm.server.started_wall_ns - 1_000_000
    os.utime(arm.startup, ns=(old, old))
    assert not goal(arm).accepted
    arm.evidence(instance=InstanceNames(namespace="other").as_dict())
    assert not goal(arm).accepted
    arm.evidence(profile_sha256="wrong")
    assert not goal(arm).accepted
    arm.ready()
    statuses = []
    subscription = arm.io.create_subscription(
        ArmStatus, "hex_arm/status", statuses.append,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    wait_for(lambda: any(message.state == ArmStatus.READY and message.execution_ready for message in statuses))
    arm.timer.cancel()
    wait_for(lambda: not arm.server.readiness()[0])
    assert not goal(arm).accepted
    arm.timer.reset()
    arm.io.destroy_subscription(subscription)


def test_concurrent_goal_rejected_and_cancel_kills_helper_then_disables(arms):
    arm = arms(delay=30)
    arm.ready()
    handle = goal(arm)
    assert handle.accepted
    child = wait_for(lambda: arm.server._child)
    assert not goal(arm, StopArm.Goal.IMMEDIATE).accepted
    with pytest.raises(RuntimeError, match="already owns"):
        OperationLock(arm.instance)
    cancellation = completed(handle.cancel_goal_async())
    assert cancellation.goals_canceling
    wrapped = completed(handle.get_result_async())
    assert wrapped.status == GoalStatus.STATUS_CANCELED
    assert not wrapped.result.success and wrapped.result.disabled_confirmed
    assert child.poll() is not None
    assert arm.state.disable_calls >= 1
    assert not arm.server.readiness()[0]


@pytest.mark.parametrize("passed,exit_code,ack", [(False, 0, True), (True, 7, True), (True, 0, False)])
def test_exit_code_or_report_or_disable_failure_never_becomes_success(arms, passed, exit_code, ack):
    arm = arms(report_passed=passed, child_exit=exit_code, disable_success=ack)
    arm.ready()
    handle = goal(arm)
    assert handle.accepted
    wrapped = completed(handle.get_result_async())
    assert wrapped.status == GoalStatus.STATUS_ABORTED
    assert not wrapped.result.success
    assert wrapped.result.disabled_confirmed == ack
    assert arm.state.disable_calls >= 1


def test_confirmed_normal_stop_publishes_result_and_preserves_driver_receipt(arms):
    arm = arms()
    arm.ready()
    phases = []
    handle = goal(arm, feedback=lambda event: phases.append(event.feedback.phase))
    assert handle.accepted
    wrapped = completed(handle.get_result_async())
    assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
    assert wrapped.result.success and wrapped.result.disabled_confirmed
    assert "normal_stop" in phases and "confirming_disable" in phases
    assert not arm.server.shutdown_path.exists()
    assert len(list(arm.server.shutdown_path.parent.glob("operations-stop-*.json"))) == 1
    assert not goal(arm).accepted  # Old successful startup cannot re-arm a stopped instance.


def test_fault_during_normal_stop_aborts_and_still_disables(arms):
    arm = arms(delay=30)
    arm.ready()
    handle = goal(arm)
    child = wait_for(lambda: arm.server._child)
    arm.state.fault = True
    wrapped = completed(handle.get_result_async())
    assert wrapped.status == GoalStatus.STATUS_ABORTED
    assert not wrapped.result.success and wrapped.result.disabled_confirmed
    assert "fault" in wrapped.result.message
    assert child.poll() is not None


def test_immediate_stop_is_available_before_startup_and_isolates_instances(arms):
    left, right = arms(), arms()
    with pytest.raises(RuntimeError, match="already owns"):
        OperationLock(left.instance, "server")
    handle = goal(left, StopArm.Goal.IMMEDIATE)
    assert handle.accepted
    result = completed(handle.get_result_async()).result
    assert result.success and result.disabled_confirmed
    assert left.state.disable_calls == 1 and right.state.disable_calls == 0
    assert right.state.mode == DriverState.MODE_ACTIVE


def test_shutdown_reaps_running_helper_and_confirms_disable(arms):
    arm = arms(delay=30)
    arm.ready()
    handle = goal(arm)
    child = wait_for(lambda: arm.server._child)
    arm.server.close()
    wrapped = completed(handle.get_result_async())
    assert not wrapped.result.success and wrapped.result.disabled_confirmed
    assert child.poll() is not None
    assert arm.state.disable_calls >= 1


def test_inherited_stop_lock_remains_owned_until_parent_releases():
    instance = InstanceNames(namespace="lock_" + uuid.uuid4().hex)
    command = [sys.executable, "-c",
               "from hex_arm_bringup.operation_lock import OperationLock; "
               "from hex_arm_bringup.integration import InstanceNames; "
               f"lock=OperationLock(InstanceNames(namespace={instance.namespace!r}),inherit=True); lock.close()"]
    with OperationLock(instance) as owned:
        subprocess.run(command, check=True, timeout=5, pass_fds=(owned.fd,),
                       env={**os.environ, "HEX_ARM_OPERATION_LOCK_FD": str(owned.fd)})
        with pytest.raises(RuntimeError, match="already owns"):
            OperationLock(instance)
    with OperationLock(instance):
        pass


def test_fault_without_actual_disable_ack_is_not_confirmed(arms):
    arm = arms(disable_success=False)
    arm.state.mode, arm.state.session, arm.state.fault = DriverState.MODE_FAULT, False, True
    handle = goal(arm, StopArm.Goal.IMMEDIATE)
    assert handle.accepted
    result = completed(handle.get_result_async()).result
    assert not result.success and not result.disabled_confirmed


def test_acknowledged_fault_preserving_release_is_confirmed(arms):
    arm = arms(disable_mode=DriverState.MODE_FAULT)
    arm.state.fault = True
    handle = goal(arm, StopArm.Goal.IMMEDIATE)
    assert handle.accepted
    result = completed(handle.get_result_async()).result
    assert result.success and result.disabled_confirmed


def test_terminal_damping_retains_session_but_real_disable_ack_is_valid(arms):
    arm = arms(retain_session=True)
    arm.ready()
    handle = goal(arm)
    assert handle.accepted
    result = completed(handle.get_result_async()).result
    assert result.success and result.disabled_confirmed
    assert arm.state.session


def test_close_reuses_current_disable_after_driver_and_service_exit(arms):
    arm = arms()
    handle = goal(arm, StopArm.Goal.IMMEDIATE)
    assert completed(handle.get_result_async()).result.disabled_confirmed
    assert arm.state.disable_calls == 1
    arm.timer.cancel()
    arm.io.destroy_service(arm.service)
    arm.server._driver_at = 0.0
    began = time.monotonic()
    arm.server.close()
    assert time.monotonic() - began < .3
    assert arm.state.disable_calls == 1


def test_reactivation_invalidates_previous_disable_receipt(arms):
    arm = arms()
    assert completed(goal(arm, StopArm.Goal.IMMEDIATE).get_result_async()).result.disabled_confirmed
    arm.state.mode, arm.state.session = DriverState.MODE_ACTIVE, True
    wait_for(lambda: not arm.server._disabled_ack)
    arm.server.close()
    assert arm.state.disable_calls == 2


@pytest.mark.parametrize('case', ['current', 'different_pid', 'old', 'unconfirmed'])
def test_final_driver_receipt_is_bound_to_observed_run(arms, case):
    arm = arms()
    arm.server._driver_run = (123, arm.server.started_wall_ns // 1_000_000)
    receipt = dict(schema_version=1, pid=123, state='disabled_confirmed', error=None,
                   unix_time_ms=time.time_ns() // 1_000_000)
    if case == 'different_pid': receipt['pid'] = 124
    if case == 'old': receipt['unix_time_ms'] = 1
    if case == 'unconfirmed': receipt['state'] = 'disable_unconfirmed'
    arm.server.shutdown_path.write_text(json.dumps(receipt))
    arm.server._closing.set()
    assert arm.server._cached_disable()[0] == (case == 'current')


def test_stop_result_keeps_motion_and_final_disable_separate(arms):
    arm = arms(disable_success=False)
    arm.ready()
    result = completed(goal(arm).get_result_async()).result
    assert result.motion_completed and not result.disabled_confirmed and not result.success
    assert Path(result.report_file).is_file()


def test_stop_action_clears_execution_readiness_while_helper_runs(arms):
    arm = arms(delay=30)
    arm.ready()
    statuses = []
    subscription = arm.io.create_subscription(
        ArmStatus, 'hex_arm/status', statuses.append,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    handle = goal(arm)
    wait_for(lambda: any(s.state == ArmStatus.STOPPING and not s.execution_ready for s in statuses))
    completed(handle.cancel_goal_async())
    completed(handle.get_result_async())
    arm.io.destroy_subscription(subscription)


@pytest.mark.parametrize('disable_success', [True, False])
def test_wrapper_client_waits_for_motion_and_disable_results(arms, disable_success):
    arm = arms(disable_success=disable_success)
    arm.ready()
    output = arm.startup.with_name('soft-stop.json')
    script = Path(__file__).resolve().parents[1] / 'scripts/request-stop-ros.py'
    process = subprocess.run([sys.executable, str(script), '--output', str(output),
                              *arm.instance.cli_args()], capture_output=True, text=True, timeout=10)
    assert process.returncode == (0 if disable_success else 1), process.stderr
    report = json.loads(output.read_text())
    assert report['motion_completed']
    assert report['disabled_confirmed'] == disable_success
    assert report['passed'] == disable_success
    assert Path(report['motion_report']).is_file()


@pytest.mark.parametrize('already_disabled', [True, False])
def test_wrapper_rejected_normal_stop_confirms_only_an_already_disabled_arm(arms, already_disabled):
    arm = arms()
    if already_disabled:
        arm.state.mode, arm.state.session = DriverState.MODE_DISABLED, False
    wait_for(lambda: arm.server._fresh_driver())
    # No startup evidence: normal movement is forbidden in both cases.
    output = arm.startup.with_name('soft-stop.json')
    script = Path(__file__).resolve().parents[1] / 'scripts/request-stop-ros.py'
    process = subprocess.run([sys.executable, str(script), '--output', str(output),
                              *arm.instance.cli_args()], capture_output=True, text=True, timeout=10)
    assert process.returncode == (0 if already_disabled else 1), process.stderr
    report = json.loads(output.read_text())
    assert not report['motion_completed']
    assert report['disabled_confirmed'] == already_disabled
    assert report['passed'] == already_disabled
    assert arm.state.disable_calls == int(already_disabled)
    assert arm.server._child is None
