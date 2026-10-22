"""Audited readiness and cancellable stop operations for one arm instance."""
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import uuid

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from hex_arm_msgs.action import StopArm
from hex_arm_msgs.msg import ArmStatus, DriverState
from std_srvs.srv import Trigger
import yaml

from .integration import InstanceNames
from .operation_lock import OperationLock


class ArmOperations(Node):
    def __init__(self, *, command_builder=None, **node_arguments):
        self.started_wall_ns = time.time_ns()
        super().__init__("hex_arm_operations", **node_arguments)
        defaults = InstanceNames().as_dict()
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.instance = InstanceNames(**{name: self.get_parameter(name).value for name in defaults})
        actual_namespace = self.get_namespace().rstrip("/")
        if actual_namespace != self.instance.namespace:
            raise ValueError("operations node namespace must match its explicit instance namespace")
        for name in ("hardware_profile", "startup_report_file", "shutdown_report_file"):
            self.declare_parameter(name, "")
            if not self.get_parameter(name).value:
                raise ValueError(f"{name} is required")
        self.profile_path = Path(self.get_parameter("hardware_profile").value).resolve()
        self.startup_path = Path(self.get_parameter("startup_report_file").value).resolve()
        self.shutdown_path = Path(self.get_parameter("shutdown_report_file").value).resolve()
        if len({self.profile_path, self.startup_path, self.shutdown_path}) != 3:
            raise ValueError("profile, startup evidence and driver shutdown receipt must use different files")
        self.profile_hash = hashlib.sha256(self.profile_path.read_bytes()).hexdigest()
        self.profile = yaml.safe_load(self.profile_path.read_text())
        self.shutdown_path.parent.mkdir(parents=True, exist_ok=True)
        for name, default in (("driver_timeout_sec", 0.3), ("service_timeout_sec", 5.0),
                              ("stop_timeout_sec", 120.0)):
            self.declare_parameter(name, default)
            value = self.get_parameter(name).value
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        self.driver_timeout = self.get_parameter("driver_timeout_sec").value
        self.service_timeout = self.get_parameter("service_timeout_sec").value
        self.stop_timeout = self.get_parameter("stop_timeout_sec").value
        self._guard = threading.RLock()
        self._disable_guard = threading.Lock()
        self._closing = threading.Event()
        self._idle = threading.Event()
        self._idle.set()
        self._cancel = threading.Event()
        self._busy = False
        self._stopped = False
        self._ever_ready = False
        self._disabled_ack = False
        self._disabled_message = ""
        self._driver_run = None
        self._error = ""
        self._driver = None
        self._driver_at = 0.0
        self._driver_stamp = None
        self._child = None
        self._normal_lock = None
        self._reserved_report = None
        self._server_lock = OperationLock(self.instance, "server")
        self._command_builder = command_builder or self._installed_stop_command
        self._callbacks = ReentrantCallbackGroup()
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_publisher = self.create_publisher(ArmStatus, "hex_arm/status", qos)
        self.driver_subscription = self.create_subscription(
            DriverState, "hex_arm/driver_state", self._on_driver, 10,
            callback_group=self._callbacks)
        self.disable_client = self.create_client(
            Trigger, "hex_arm/deactivate_hardware", callback_group=self._callbacks)
        self.action = ActionServer(
            self, StopArm, "hex_arm/stop", execute_callback=self._execute,
            goal_callback=self._goal, cancel_callback=self._cancel_goal,
            callback_group=self._callbacks)
        self.timer = self.create_timer(0.1, self._publish_status, callback_group=self._callbacks)

    def _on_driver(self, message):
        with self._guard:
            stamp = (message.stamp.sec, message.stamp.nanosec)
            if self._driver_stamp is not None and stamp < self._driver_stamp:
                self._disabled_ack = False
                self._driver_run = None
            if stamp != self._driver_stamp:
                self._driver_at = time.monotonic()
                self._driver_stamp = stamp
            self._driver = message
            if message.mode not in (DriverState.MODE_DISABLED, DriverState.MODE_FAULT):
                self._disabled_ack = False
            if self._driver_run is None:
                try:
                    receipt = json.loads(self.shutdown_path.read_text())
                    if receipt.get("schema_version") == 1 and receipt.get("state") == "starting":
                        self._driver_run = (receipt["pid"], receipt["unix_time_ms"])
                except (OSError, ValueError, KeyError, TypeError):
                    pass

    def _fresh_driver(self):
        return (self._driver is not None and self._driver_stamp != (0, 0)
                and time.monotonic() - self._driver_at <= self.driver_timeout
                and self._driver.feedback_fresh)

    def _startup_evidence(self):
        metadata = self.startup_path.stat()
        if metadata.st_mtime_ns < self.started_wall_ns:
            raise RuntimeError("startup report predates this operations server")
        payload = self.startup_path.read_bytes()
        report = json.loads(payload)
        if not isinstance(report, dict):
            raise RuntimeError("startup report must be an object")
        if (report.get("instance") != self.instance.as_dict()
                or report.get("profile_sha256") != self.profile_hash
                or hashlib.sha256(self.profile_path.read_bytes()).hexdigest() != self.profile_hash):
            raise RuntimeError("startup report profile or instance does not match this arm")
        if (report.get("passed") is not True or report.get("deactivated")
                or "ready_hold" not in report):
            raise RuntimeError("startup has not completed verified ready hold")
        return (metadata.st_ino, metadata.st_mtime_ns, hashlib.sha256(payload).hexdigest())

    def readiness(self):
        with self._guard:
            if self._closing.is_set() or self._stopped:
                return False, "stop requested; a new verified startup is required"
            if not self._fresh_driver():
                return False, "driver feedback missing or stale"
            driver = self._driver
            if not (driver.mode == DriverState.MODE_ACTIVE and driver.session_owned
                    and driver.profile_valid and driver.calibrated and driver.all_motors_online
                    and not driver.fault_latched):
                return False, driver.fault_reason or "driver is not healthy and ACTIVE"
            try:
                self._startup_evidence()
            except (OSError, ValueError, RuntimeError) as error:
                return False, str(error)
            self._ever_ready = True
            return True, "verified startup and fresh ACTIVE driver"

    def _publish_status(self):
        with self._guard:
            ready, reason = self.readiness()
            message = ArmStatus()
            message.stamp = self.get_clock().now().to_msg()
            message.instance = self.instance.ros_name(self.instance.hardware_name)
            message.execution_ready = ready and not self._busy
            message.disabled_confirmed = self._disabled_ack and self._fresh_driver()
            if self._busy:
                message.state = ArmStatus.STOPPING
            elif self._error or (self._driver is not None and self._driver.fault_latched):
                message.state = ArmStatus.FAULT
            elif message.disabled_confirmed:
                message.state = ArmStatus.DISABLED
            elif self._stopped or (self._ever_ready and not ready):
                message.state = ArmStatus.FAULT
            elif ready:
                message.state = ArmStatus.READY
            else:
                message.state = ArmStatus.STARTING
            message.reason = self._error or reason
            self.status_publisher.publish(message)

    def _goal(self, request):
        with self._guard:
            if self._busy or self._closing.is_set() or request.mode not in (StopArm.Goal.NORMAL, StopArm.Goal.IMMEDIATE):
                return GoalResponse.REJECT
            if request.mode == StopArm.Goal.NORMAL:
                if not self.readiness()[0]:
                    return GoalResponse.REJECT
                try:
                    self._reserved_report = self._startup_evidence()
                    self._normal_lock = OperationLock(self.instance)
                except (OSError, ValueError, RuntimeError):
                    return GoalResponse.REJECT
            self._busy = True
            self._cancel.clear()
            self._idle.clear()
            return GoalResponse.ACCEPT

    def _cancel_goal(self, _):
        self._cancel.set()
        return CancelResponse.ACCEPT

    def _installed_stop_command(self, output):
        path = Path(get_package_prefix("hex_arm_bringup")) / "lib/hex_arm_bringup/graceful-real-launch.py"
        spec = importlib.util.spec_from_file_location("hex_arm_graceful_command", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module.stop_command(self.profile, self.profile_path, self.startup_path, output, self.instance)

    def _cached_disable(self):
        with self._guard:
            # A receipt is historical evidence for this stop, even after ROS
            # publishers exit. Observing reactivation invalidates it above.
            if self._disabled_ack and (self._closing.is_set() or self._fresh_driver()):
                return True, self._disabled_message
            if self._closing.is_set() and self._driver_run is not None:
                try:
                    receipt = json.loads(self.shutdown_path.read_text())
                    if (receipt.get("schema_version") == 1
                            and receipt.get("state") == "disabled_confirmed" and not receipt.get("error")
                            and receipt.get("pid") == self._driver_run[0]
                            and receipt["unix_time_ms"] >= max(
                                self._driver_run[1], self.started_wall_ns // 1_000_000)):
                        self._disabled_ack = True
                        self._disabled_message = "this driver run's final disable receipt confirmed"
                        return True, self._disabled_message
                except (OSError, ValueError, KeyError, TypeError):
                    pass
        return False, ""

    def _disable(self):
        # The action's finally block and close() may reach this concurrently.
        with self._disable_guard:
            cached = self._cached_disable()
            return cached if cached[0] else self._request_disable()

    def _request_disable(self):
        requested_at = time.monotonic()
        with self._guard:
            self._disabled_ack = False
        deadline = requested_at + self.service_timeout
        while not self.disable_client.service_is_ready():
            cached = self._cached_disable()
            if cached[0]:
                return cached
            if time.monotonic() >= deadline:
                return False, "disable service unavailable"
            time.sleep(0.02)
        try:
            future = self.disable_client.call_async(Trigger.Request())
            completed = threading.Event()
            future.add_done_callback(lambda _: completed.set())
            while not completed.wait(min(0.02, max(0.0, deadline - time.monotonic()))):
                cached = self._cached_disable()
                if cached[0]:
                    return cached
                if time.monotonic() >= deadline:
                    return False, "disable acknowledgement timed out"
            response = future.result()
            if response is None or not response.success:
                return False, response.message if response else "disable response missing"
            # deactivate_hardware now acknowledges only an actual successful
            # owned release/damped-stop RPC plus its current inactive state.
            # Damping retains its terminal lease, and release may preserve FAULT;
            # neither a mode bit nor session absence can replace that receipt.
            while time.monotonic() < deadline:
                with self._guard:
                    if (self._fresh_driver() and self._driver_at >= requested_at
                            and self._driver.mode in (DriverState.MODE_DISABLED, DriverState.MODE_FAULT)):
                        self._disabled_ack = True
                        self._disabled_message = response.message
                        return True, response.message
                time.sleep(0.02)
            return False, "disable acknowledged but fresh inactive driver state not confirmed"
        except Exception as error:
            return False, f"disable request failed: {error}"

    def _kill_child(self):
        with self._guard:
            child = self._child
            if child is not None and child.poll() is None:
                # The helper owns its process group. Reap it before asking the
                # hardware to disable, so it cannot issue another movement.
                try:
                    os.killpg(child.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                child.wait(timeout=5)
            self._child = None

    @staticmethod
    def _feedback(goal, phase):
        try:
            goal.publish_feedback(StopArm.Feedback(phase=phase))
        except Exception:
            # A vanished action client must never prevent the disable request.
            pass

    def _execute(self, goal):
        result = StopArm.Result()
        normal_ok = False
        failure = ""
        try:
            self._feedback(goal, "preflight")
            if goal.request.mode == StopArm.Goal.NORMAL:
                if self._cancel.is_set() or self._closing.is_set():
                    raise RuntimeError("stop canceled before motion")
                if not self.readiness()[0] or self._startup_evidence() != self._reserved_report:
                    raise RuntimeError("startup or driver readiness changed before normal stop")
                output = self.shutdown_path.with_name(f"operations-stop-{uuid.uuid4().hex}.json")
                result.report_file = str(output)
                began = time.time_ns()
                command = self._command_builder(output)
                with self._guard:
                    if self._cancel.is_set() or self._closing.is_set():
                        raise RuntimeError("stop canceled before helper launch")
                    self._child = subprocess.Popen(
                        command, start_new_session=True,
                        env={**os.environ, "HEX_ARM_OPERATION_LOCK_FD": str(self._normal_lock.fd)},
                        pass_fds=(self._normal_lock.fd,))
                    child = self._child
                self._feedback(goal, "normal_stop")
                deadline = time.monotonic() + self.stop_timeout
                while child.poll() is None:
                    if self._cancel.wait(0.02) or self._closing.is_set():
                        raise RuntimeError("normal stop interrupted")
                    if time.monotonic() >= deadline:
                        raise TimeoutError("normal stop timed out")
                    with self._guard:
                        if not self._fresh_driver() or self._driver.fault_latched:
                            raise RuntimeError("driver fault or stale feedback during normal stop")
                report = json.loads(output.read_text())
                matching = (output.stat().st_mtime_ns >= began
                             and report.get("instance") == self.instance.as_dict()
                             and report.get("profile_sha256") == self.profile_hash
                             and Path(report.get("startup_report", "")).resolve() == self.startup_path)
                result.motion_completed = matching and bool(report.get("motion_completed", report.get("passed")))
                if child.returncode != 0:
                    raise RuntimeError(f"normal stop helper exited {child.returncode}: {report.get('error', '')}")
                normal_ok = (matching and report.get("passed") is True and
                             (report.get("deactivated") is True or report.get("phase") == "disabled_confirmed"))
                if not normal_ok:
                    raise RuntimeError("normal stop has no matching confirmed-disable report")
        except Exception as error:
            failure = str(error)
        finally:
            try:
                self._kill_child()
            except Exception as error:
                failure = failure or f"stop helper cleanup failed: {error}"
            self._feedback(goal, "confirming_disable")
            disabled, disable_message = self._disable()
            with self._guard:
                if (goal.request.mode == StopArm.Goal.NORMAL and self._driver is not None
                        and self._driver.fault_latched):
                    failure = failure or "driver fault during normal stop"
                self._stopped = True
                self._error = failure or ("" if disabled else disable_message)
            result.disabled_confirmed = disabled
            result.success = disabled and not failure and (
                goal.request.mode == StopArm.Goal.IMMEDIATE or normal_ok)
            result.message = failure or ("stop completed; disable acknowledged" if result.success else disable_message)
            if failure and not disabled:
                result.message += "; " + disable_message
            if self._normal_lock is not None:
                self._normal_lock.close()
                self._normal_lock = None
            if goal.is_cancel_requested:
                result.success = False
                goal.canceled()
            elif result.success:
                goal.succeed()
            else:
                goal.abort()
            with self._guard:
                self._busy = False
                self._idle.set()
        return result

    def close(self):
        """Call while the executor is still serving callbacks, before rclpy shutdown."""
        self._closing.set()
        self._cancel.set()
        try:
            self._kill_child()
        except Exception as error:
            self.get_logger().error(f"stop helper cleanup failed: {error}")
        finally:
            self._idle.wait(self.service_timeout + 6.0)
            acknowledged, reason = self._disable()
            if not acknowledged:
                self.get_logger().error(reason)
            self._server_lock.close()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop = threading.Event()
    previous = {}
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        previous[signum] = signal.signal(signum, lambda *_: stop.set())
    node = None
    executor = MultiThreadedExecutor(num_threads=4)
    worker = None
    try:
        node = ArmOperations()
        executor.add_node(node)
        worker = threading.Thread(target=executor.spin, daemon=True)
        worker.start()
        while rclpy.ok() and not stop.wait(0.1):
            pass
    finally:
        if node is not None:
            node.close()
        executor.shutdown(timeout_sec=8.0)
        if worker is not None:
            worker.join(timeout=8.0)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for signum, handler in previous.items():
            signal.signal(signum, handler)
