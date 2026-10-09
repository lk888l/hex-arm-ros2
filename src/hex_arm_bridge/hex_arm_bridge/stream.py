from __future__ import annotations

import math
import threading
import time
from typing import Any, Optional

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from hex_arm_msgs.msg import DriverState as RosDriverState, MotorIdentity as RosMotorIdentity
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from sensor_msgs.msg import JointState as RosJointState

from hex_arm_bridge.pb import robot_api_pb2 as pb
from hex_arm_bridge.protocol import reorder


from hex_arm_bridge.mapping import _temperature_vector_error, _diagnostic_values, ros_motor, ros_driver
from hex_arm_bridge.readiness import _Snapshot, _positive_timeout, _observation_error, _readiness_error, _active_ownership_error


class StreamMixin:
    """Methods borrowing the sole HexArmBridge state owner; no duplicate session state."""

    def _start_streaming(self) -> None:
        self._stream_stop.clear()
        self._stream_executor = SingleThreadedExecutor(context=self.context)
        self._stream_executor.add_node(self._stream_node)

        def spin():
            try:
                while not self._stream_stop.is_set():
                    self._stream_executor.spin_once(timeout_sec=0.01)
            except ExternalShutdownException:
                pass
            except Exception as error:
                self.get_logger().error(f"stream executor failed: {error}")
                self.request_stop()

        self._stream_thread = threading.Thread(target=spin, name="hex_arm_stream")
        self._stream_thread.start()
        self._diagnostic_stop.clear()
        self._diagnostic_wakeup.clear()
        period = _positive_timeout(
            self.get_parameter("diagnostic_period_sec").value, "diagnostic_period_sec"
        )

        def publish_diagnostics():
            try:
                while not self._diagnostic_stop.is_set():
                    self._publish_diagnostics()
                    self._diagnostic_wakeup.wait(period)
                    self._diagnostic_wakeup.clear()
            except Exception as error:
                if not self._diagnostic_stop.is_set() and not self._shutdown_requested.is_set():
                    self.get_logger().error(f"diagnostic worker failed: {error}")
                    self.request_stop()

        self._diagnostic_thread = threading.Thread(
            target=publish_diagnostics, name="hex_arm_diagnostics"
        )
        self._diagnostic_thread.start()

    def _stop_streaming(self) -> None:
        self._diagnostic_stop.set()
        self._diagnostic_wakeup.set()
        if self._diagnostic_thread is not None:
            self._diagnostic_thread.join()
            self._diagnostic_thread = None
        self._stream_stop.set()
        if self._stream_executor is not None:
            self._stream_executor.wake()
        if self._stream_thread is not None:
            self._stream_thread.join()
            self._stream_thread = None
        if self._stream_executor is not None:
            self._stream_executor.remove_node(self._stream_node)
            self._stream_executor.shutdown()
            self._stream_executor = None

    def _snapshot_copy(self) -> _Snapshot:
        with self._lock:
            return _Snapshot(
                joint_state=self._snapshot.joint_state,
                driver_state=self._snapshot.driver_state,
                joint_received_at=self._snapshot.joint_received_at,
                driver_received_at=self._snapshot.driver_received_at,
            )

    def _state_observation_error(self) -> Optional[str]:
        return _observation_error(
            self._snapshot_copy(),
            time.monotonic(),
            self.get_parameter("state_timeout_sec").value,
        )

    def _state_readiness_error(self) -> Optional[str]:
        return _readiness_error(
            self._snapshot_copy(),
            time.monotonic(),
            self.get_parameter("state_timeout_sec").value,
        )

    def _state_active_ownership_error(
        self, driver_received_after: Optional[float] = None
    ) -> Optional[str]:
        return _active_ownership_error(
            self._snapshot_copy(),
            time.monotonic(),
            self.get_parameter("state_timeout_sec").value,
            pb.OPERATING_MODE_ACTIVE,
            driver_received_after,
        )

    def _latch_runtime_gate_if_needed(self) -> bool:
        # Zenoh callbacks must never wait on a management transaction. A mode
        # RPC can need those same receive workers to deliver its reply/state.
        # This gate only reads the snapshot and revokes forwarding; the short
        # data lock makes that atomic without holding the hardware RPC lock.
        with self._lock:
            if self._runtime_gate_latched:
                return True
            if self._session_id == 0 or not self._hardware_active:
                return False
            snapshot = _Snapshot(
                joint_state=self._snapshot.joint_state,
                driver_state=self._snapshot.driver_state,
                joint_received_at=self._snapshot.joint_received_at,
                driver_received_at=self._snapshot.driver_received_at,
            )
            try:
                error = _active_ownership_error(
                    snapshot,
                    time.monotonic(),
                    self.get_parameter("state_timeout_sec").value,
                    pb.OPERATING_MODE_ACTIVE,
                )
            except Exception as exception:
                error = f"runtime gate evaluation failed: {exception}"
            if error is None:
                return False
            self._hardware_active = False
            self._runtime_gate_latched = True
            self._runtime_gate_reason = error
        self.get_logger().error(
            f"runtime safety gate latched, commands and internal state stopped: {error}"
        )
        wakeup = getattr(self, "_diagnostic_wakeup", None)
        if wakeup is not None:
            wakeup.set()
        return True

    def _on_zenoh_joint_state(self, sample: Any) -> None:
        with self._lock:
            if not self._accept_zenoh_state:
                return
        try:
            message = pb.JointState.FromString(bytes(sample.payload))
            if len(message.q) != 6 or not all(math.isfinite(value) for value in message.q):
                raise ValueError("joint state does not contain six finite positions")
        except Exception as error:
            with self._lock:
                if not self._accept_zenoh_state:
                    return
                log_detail = self._joint_state_error_log.report(
                    error, time.monotonic()
                )
            if log_detail is not None:
                self.get_logger().error(f"invalid Zenoh joint state: {log_detail}")
            return

        with self._lock:
            if not self._accept_zenoh_state:
                return
            self._snapshot.joint_state = message
            self._snapshot.joint_received_at = time.monotonic()
            recovered_from = self._joint_state_error_log.recover()
        if recovered_from is not None:
            self.get_logger().info(
                f"valid Zenoh joint state resumed after invalid input: {recovered_from}"
            )
        self._latch_runtime_gate_if_needed()
        trace = getattr(self, "_trace", None)
        if trace is not None and trace.enabled:
            trace.emit("state_receive", message.header.seq)

    def _on_zenoh_driver_state(self, sample: Any) -> None:
        try:
            message = pb.DriverState.FromString(bytes(sample.payload))
            with self._lock:
                if not self._accept_zenoh_state:
                    return
                self._snapshot.driver_state = message
                self._snapshot.driver_received_at = time.monotonic()
                key = (message.mode, message.fault_latched, message.fault_code,
                       message.fault_reason, message.calibrated,
                       message.feedback_fresh, message.all_motors_online,
                       message.profile_valid, message.session_owned)
                diagnostic_changed = key != getattr(self, "_diagnostic_key", None)
                self._diagnostic_key = key
            self._latch_runtime_gate_if_needed()
            wakeup = getattr(self, "_diagnostic_wakeup", None)
            if diagnostic_changed and wakeup is not None:
                wakeup.set()
        except Exception as error:
            self.get_logger().error(f"invalid Zenoh driver state: {error}")

    def _on_ros_command(self, message: RosJointState) -> None:
        trace = getattr(self, "_trace", None)
        seq = source_stamp_ns = 0
        if trace is not None and trace.enabled:
            self._trace_seq += 1
            seq = self._trace_seq
            source_stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            trace.emit("python_receive", seq, self._session_id, source_stamp_ns)
        # During a serialized activation RPC the stream must keep publishing
        # observations instead of waiting for the management lock.
        with self._lock:
            if not self._hardware_active:
                return
        with self._hardware_transition_lock:
            if self._latch_runtime_gate_if_needed():
                return
            with self._lock:
                session_id = self._session_id
                active = self._hardware_active
                session = self._session
            if not active or session_id == 0 or session is None:
                return
            try:
                q = reorder(message.position, message.name, self._joint_names)
                dq = reorder(message.velocity, message.name, self._joint_names) if message.velocity else [0.0] * 6
                command = pb.JointTrajectory(
                    session_id=session_id,
                    # Empty kp/kd intentionally delegate the reviewed per-axis defaults
                    # in the hardware profile to the Rust safety boundary.
                    # Empty gains delegate to the reviewed hardware profile.
                    # Empty tau_ff delegates gravity compensation to the Rust
                    # controller at the latest measured pose. Legacy clients
                    # may still send an explicit six-axis feed-forward vector.
                    points=[pb.JointSetpoint(q=q, dq=dq, kp=[], kd=[], tau_ff=[])],
                    t_from_start_ns=[int(float(self.get_parameter("command_period_sec").value) * 1e9)],
                    # The hardware controller advertises and accepts only the
                    # fail-closed FAULT contract. A bounded-deceleration ramp
                    # has not been commissioned, so do not claim one here.
                    on_timeout=pb.TIMEOUT_BEHAVIOR_FAULT,
                )
                if trace is not None and trace.enabled:
                    command.header.seq = seq
                    command.header.stamp_ns = time.monotonic_ns()
                session.put(f"{self.prefix}/arm/command", command.SerializeToString())
                if trace is not None and trace.enabled:
                    trace.emit("zenoh_put", seq, session_id, source_stamp_ns)
            except Exception as error:
                self.get_logger().error(f"command rejected: {error}")

    def _publish_snapshot(self) -> None:
        self._latch_runtime_gate_if_needed()
        with self._lock:
            joint = self._snapshot.joint_state
            driver = self._snapshot.driver_state
            gate_latched = self._runtime_gate_latched
            state_pub = self._state_pub
            driver_pub = self._driver_pub
        if joint is not None and state_pub is not None and not gate_latched:
            state = RosJointState()
            state.header.stamp = self.get_clock().now().to_msg()
            state.name = self._joint_names
            state.position = list(joint.q)
            state.velocity = list(joint.dq) if len(joint.dq) == 6 else [0.0] * 6
            state.effort = list(joint.tau_est) if len(joint.tau_est) == 6 else [0.0] * 6
            trace = getattr(self, "_trace", None)
            if trace is not None and trace.enabled:
                source_stamp = state.header.stamp.sec * 1_000_000_000 + state.header.stamp.nanosec
                trace.emit("python_state_publish", joint.header.seq, 0, source_stamp)
            state_pub.publish(state)
        if driver is not None and driver_pub is not None:
            driver_pub.publish(self._ros_driver(driver))

    def _publish_diagnostics(self) -> None:
        # Read a snapshot under the same owner lock, then format off the
        # stream executor. Safety checks still run at the stream rate.
        with self._lock:
            joint = self._snapshot.joint_state
            driver = self._snapshot.driver_state
            joint_received_at = self._snapshot.joint_received_at
            driver_received_at = self._snapshot.driver_received_at
            gate_latched = self._runtime_gate_latched
            gate_reason = self._runtime_gate_reason
            diag_pub = self._diag_pub
        if diag_pub is not None:
            now = time.monotonic()
            joint_age = now - joint_received_at if joint_received_at else float("inf")
            driver_age = now - driver_received_at if driver_received_at else float("inf")
            readiness_error = _readiness_error(
                _Snapshot(joint, driver, joint_received_at, driver_received_at),
                now,
                self.get_parameter("state_timeout_sec").value,
            )
            temperature_error = _temperature_vector_error(joint)
            diag = DiagnosticArray()
            diag.header.stamp = self.get_clock().now().to_msg()
            status = DiagnosticStatus()
            status.name = "firefly_y6/driver"
            status.hardware_id = self.prefix
            status.level = (
                DiagnosticStatus.ERROR
                if gate_latched or (driver and driver.fault_latched)
                else DiagnosticStatus.WARN
                if readiness_error is not None or temperature_error is not None
                else DiagnosticStatus.OK
            )
            primary_message = (
                f"runtime gate latched: {gate_reason}"
                if gate_latched
                else driver.fault_reason
                if driver and driver.fault_latched
                else readiness_error
            )
            messages = [message for message in (primary_message, temperature_error) if message]
            status.message = "; ".join(messages) or "joint and driver state are ready"
            status.values = _diagnostic_values(joint, driver, joint_age, driver_age)
            diag.status = [status]
            diag_pub.publish(diag)


    @staticmethod
    def _ros_motor(motor: pb.MotorIdentity) -> RosMotorIdentity:
        return ros_motor(motor)

    def _ros_driver(self, driver: pb.DriverState) -> RosDriverState:
        return ros_driver(driver, self.get_clock().now().to_msg())
