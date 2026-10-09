from __future__ import annotations

import time
from typing import Any, Optional

from hex_arm_msgs.srv import DiscoverMotors, SetGravity, SetOperatingMode
from std_srvs.srv import Trigger

from hex_arm_bridge.pb import robot_api_pb2 as pb


from hex_arm_bridge.readiness import _Snapshot, _positive_timeout, _BridgeShutdownRequested, _generic_response_error


class ManagementMixin:
    """Methods borrowing the sole HexArmBridge state owner; no duplicate session state."""

    def _query_with_retry(
        self, key: str, payload: bytes, response_type: Any, deadline: float
    ) -> Any:
        last_error: Optional[str] = None
        while True:
            self._raise_if_stop_requested()
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                detail = f" ({last_error})" if last_error else ""
                raise TimeoutError(f"timed out querying {key}{detail}")
            try:
                query_timeout = _positive_timeout(
                    self.get_parameter("query_timeout_sec").value, "query_timeout_sec"
                )
                response = self._query_with_timeout(
                    key, payload, response_type, min(query_timeout, remaining)
                )
                if response is not None:
                    return response
            except Exception as error:
                last_error = str(error)
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                continue
            self._wait_interruptibly(min(0.05, remaining))

    def _wait_for_observation(self, deadline: float) -> None:
        while True:
            self._raise_if_stop_requested()
            error = self._state_observation_error()
            if error is None:
                return
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise TimeoutError(f"controller did not become ready: {error}")
            self._wait_interruptibly(min(0.01, remaining))

    def _wait_for_active_ownership(
        self, driver_received_after: float, deadline: float
    ) -> None:
        while True:
            self._raise_if_stop_requested()
            error = self._state_active_ownership_error(driver_received_after)
            if error is None:
                return
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise TimeoutError(f"driver did not confirm ACTIVE ownership: {error}")
            self._wait_interruptibly(min(0.01, remaining))

    def _query(self, key: str, payload: bytes, response_type: Any) -> Any | None:
        timeout = _positive_timeout(
            self.get_parameter("query_timeout_sec").value, "query_timeout_sec"
        )
        return self._query_with_timeout(key, payload, response_type, timeout)

    def _query_mode(self, key: str, payload: bytes, response_type: Any) -> Any | None:
        """Use the management-plane timeout for serialized six-axis transitions."""
        timeout = _positive_timeout(
            self.get_parameter("mode_transition_timeout_sec").value,
            "mode_transition_timeout_sec",
        )
        return self._query_with_timeout(key, payload, response_type, timeout)

    def _query_with_timeout(
        self,
        key: str,
        payload: bytes,
        response_type: Any,
        timeout_sec: float,
        *,
        cancel_on_shutdown: bool = True,
    ) -> Optional[Any]:
        timeout = _positive_timeout(timeout_sec, "query timeout")
        if cancel_on_shutdown:
            self._raise_if_stop_requested()
        import zenoh

        cancellation_token = zenoh.CancellationToken()
        with self._lock:
            if cancel_on_shutdown and self._shutdown_requested.is_set():
                raise _BridgeShutdownRequested("bridge shutdown requested")
            session = self._session
            if cancel_on_shutdown:
                self._active_query_tokens.add(cancellation_token)
        if session is None:
            with self._lock:
                self._active_query_tokens.discard(cancellation_token)
            raise RuntimeError("Zenoh session is not open")
        try:
            # zenoh-python's CancellationToken interrupts the blocking reply
            # iterator.  Without it a SIGINT during a lifecycle transition can
            # leave the executor worker alive until the entire RPC timeout.
            replies = session.get(
                key,
                payload=payload,
                timeout=timeout,
                cancellation_token=cancellation_token,
            )
            for reply in replies:
                if cancel_on_shutdown:
                    self._raise_if_stop_requested()
                sample = reply.ok
                if sample is not None:
                    return response_type.FromString(bytes(sample.payload))
            if cancel_on_shutdown:
                self._raise_if_stop_requested()
            return None
        finally:
            with self._lock:
                self._active_query_tokens.discard(cancellation_token)

    def _activate_hardware(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        with self._hardware_transition_lock:
            with self._destroy_lock:
                destroyed = self._destroyed
            if destroyed:
                response.success = False
                response.message = "activation rejected: bridge is shutting down"
                return response

            readiness_error = self._state_readiness_error()
            if readiness_error is not None:
                release_error = self._safe_release()
                response.success = False
                response.message = f"activation rejected: {readiness_error}"
                if release_error is not None:
                    response.message += f" and previous session release failed: {release_error}"
                return response

            with self._lock:
                session_id = self._session_id
                hardware_active = self._hardware_active
            if hardware_active and session_id != 0:
                ownership_error = self._state_active_ownership_error()
                if ownership_error is None:
                    response.success = True
                    response.message = "controller is already ACTIVE with the owned session"
                    return response
                self.get_logger().warning(
                    f"existing ACTIVE state is inconsistent and will be reacquired: {ownership_error}"
                )
                release_error = self._safe_release()
                if release_error is not None:
                    response.success = False
                    response.message = f"activation rejected: previous session release failed: {release_error}"
                    return response
            elif hardware_active or session_id != 0:
                release_error = self._safe_release()
                if release_error is not None:
                    response.success = False
                    response.message = f"activation rejected: previous session release failed: {release_error}"
                    return response

            acquired_session_id = 0
            try:
                acquired = self._query(
                    f"{self.prefix}/rpc/acquire_session",
                    pb.AcquireSessionRequest(client_name="ros2_control").SerializeToString(),
                    pb.AcquireSessionResponse,
                )
                if acquired is None or not acquired.ok or acquired.session_id == 0:
                    raise RuntimeError("exclusive session acquisition failed")
                acquired_session_id = acquired.session_id

                readiness_error = self._state_readiness_error()
                if readiness_error is not None:
                    raise RuntimeError(f"readiness changed after session acquisition: {readiness_error}")

                with self._lock:
                    driver_received_before_active = self._snapshot.driver_received_at
                mode = self._query_mode(
                    f"{self.prefix}/rpc/set_mode",
                    pb.SetModeRequest(
                        session_id=acquired_session_id, mode=pb.OPERATING_MODE_ACTIVE
                    ).SerializeToString(),
                    pb.GenericResponse,
                )
                if mode is None or not mode.ok:
                    detail = (
                        mode.error
                        if mode is not None and mode.HasField("error")
                        else "ACTIVE rejected"
                    )
                    raise RuntimeError(detail)

                confirmation_timeout = _positive_timeout(
                    self.get_parameter("mode_transition_timeout_sec").value,
                    "mode_transition_timeout_sec",
                )
                self._wait_for_active_ownership(
                    driver_received_before_active,
                    time.monotonic() + confirmation_timeout,
                )
            except Exception as error:
                rollback_error = None
                if acquired_session_id != 0:
                    try:
                        rollback_error = self._release_session(acquired_session_id)
                    except Exception as exception:
                        rollback_error = str(exception)
                    if rollback_error is not None:
                        with self._lock:
                            self._session_id = acquired_session_id
                            self._hardware_active = False
                            self._runtime_gate_latched = True
                            self._runtime_gate_reason = (
                                f"activation rollback release failed: {rollback_error}"
                            )
                        self.get_logger().error(
                            f"activation rollback retained session for retry: {rollback_error}"
                        )
                response.success = False
                response.message = f"activation rejected: {error}"
                if rollback_error is not None:
                    response.message += f" and rollback release failed: {rollback_error}"
                return response

            with self._lock:
                self._session_id = acquired_session_id
                self._hardware_active = True
                self._runtime_gate_latched = False
                self._runtime_gate_reason = ""
            response.success = True
            response.message = "exclusive session acquired and controller ACTIVE"
            return response

    def _deactivate_hardware(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        release_error = self._safe_release()
        response.success = release_error is None
        response.message = (
            "controller disabled and session released"
            if release_error is None
            else f"deactivation failed: {release_error}"
        )
        return response

    def _damped_stop(self, request, response):
        del request
        with self._hardware_transition_lock:
            with self._lock:
                session_id = self._session_id
                if not self._hardware_active or not session_id:
                    response.success = False
                    response.message = "damped stop requires active ros2_control ownership"
                    return response
                self._hardware_active = False
            try:
                result = self._query_with_timeout(
                    f"{self.prefix}/rpc/damped_stop",
                    pb.DampedStopRequest(session_id=session_id).SerializeToString(),
                    pb.GenericResponse, 35.0)
                response.success = bool(result and result.ok)
                response.message = ("folded, settled and disabled" if response.success else
                                    (result.error if result else "damped stop acknowledgement timed out"))
                if response.success:
                    with self._lock:
                        self._session_id = 0
            except Exception as error:
                response.success = False
                response.message = str(error)
            # A failed call is terminal too: never resume an old trajectory stream.
            return response

    def _set_mode(self, request: SetOperatingMode.Request, response: SetOperatingMode.Response) -> SetOperatingMode.Response:
        if request.mode == pb.OPERATING_MODE_ACTIVE:
            response.success = False
            response.message = "ACTIVE is bound to ros2_control hardware activation"
            return response
        with self._lock:
            session_id = self._session_id
        if session_id == 0:
            response.success = False
            response.message = "no ros2_control-owned session"
            return response
        result = self._query_mode(
            f"{self.prefix}/rpc/set_mode",
            pb.SetModeRequest(session_id=session_id, mode=request.mode).SerializeToString(),
            pb.GenericResponse,
        )
        response.success = bool(result and result.ok)
        response.message = "mode changed" if response.success else "mode rejected"
        return response

    def _clear_fault(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        with self._hardware_transition_lock:
            with self._lock:
                session_id = self._session_id

            temporary_session = session_id == 0
            if temporary_session:
                try:
                    acquired = self._query(
                        f"{self.prefix}/rpc/acquire_session",
                        pb.AcquireSessionRequest(
                            client_name="ros2_fault_recovery"
                        ).SerializeToString(),
                        pb.AcquireSessionResponse,
                    )
                except Exception as error:
                    response.success = False
                    response.message = f"fault clear session acquisition failed: {error}"
                    return response
                if acquired is None or not acquired.ok or acquired.session_id == 0:
                    detail = (
                        acquired.error
                        if acquired is not None and acquired.HasField("error")
                        else "exclusive session is unavailable"
                    )
                    response.success = False
                    response.message = f"fault clear session acquisition failed: {detail}"
                    return response
                session_id = acquired.session_id

            clear_error = None
            try:
                result = self._query_mode(
                    f"{self.prefix}/rpc/clear_fault",
                    pb.ClearFaultRequest(session_id=session_id).SerializeToString(),
                    pb.GenericResponse,
                )
                clear_error = _generic_response_error(result, "fault clear request")
            except Exception as error:
                clear_error = f"fault clear request failed: {error}"

            release_error = None
            if temporary_session:
                try:
                    released = self._query(
                        f"{self.prefix}/rpc/release_session",
                        pb.ReleaseSessionRequest(session_id=session_id).SerializeToString(),
                        pb.GenericResponse,
                    )
                    release_error = _generic_response_error(
                        released, "fault recovery session release"
                    )
                except Exception as error:
                    release_error = f"fault recovery session release failed: {error}"

                if release_error is not None:
                    # Do not lose track of a lease that the controller may
                    # still own.  It remains inactive and can be released by a
                    # later clear/deactivate retry.
                    with self._lock:
                        if self._session_id == 0:
                            self._session_id = session_id
                            self._hardware_active = False
                            self._runtime_gate_latched = True
                            self._runtime_gate_reason = release_error

            errors = [error for error in (clear_error, release_error) if error]
            response.success = not errors
            response.message = (
                "fault cleared; recovery session released"
                if response.success and temporary_session
                else "fault cleared"
                if response.success
                else "; ".join(errors)
            )
            return response

    def _discover_motors(
        self, request: DiscoverMotors.Request, response: DiscoverMotors.Response
    ) -> DiscoverMotors.Response:
        result = self._query(
            f"{self.prefix}/arm/rpc/discover",
            pb.DiscoverMotorsRequest(refresh=request.refresh).SerializeToString(),
            pb.DiscoverMotorsResponse,
        )
        response.success = bool(result and result.ok)
        response.message = "discovery complete" if response.success else "discovery failed"
        if result:
            response.motors = [self._ros_motor(motor) for motor in result.motors]
        return response

    def _set_gravity(self, request: SetGravity.Request, response: SetGravity.Response) -> SetGravity.Response:
        with self._lock:
            session_id = self._session_id
        if session_id == 0:
            response.success = False
            response.message = "gravity setting requires the ros2_control-owned session"
            return response
        result = self._query(
            f"{self.prefix}/arm/rpc/set_gravity",
            pb.SetGravityRequest(
                session_id=session_id, gravity=pb.Vec3(x=request.x, y=request.y, z=request.z)
            ).SerializeToString(),
            pb.GenericResponse,
        )
        response.success = bool(result and result.ok)
        response.message = "gravity updated" if response.success else "gravity rejected"
        return response

    def _release_session(self, session_id: int) -> Optional[str]:
        errors: list[str] = []
        try:
            disabled = self._query_with_timeout(
                f"{self.prefix}/rpc/set_mode",
                pb.SetModeRequest(
                    session_id=session_id, mode=pb.OPERATING_MODE_DISABLED
                ).SerializeToString(),
                pb.GenericResponse,
                _positive_timeout(
                    self.get_parameter("mode_transition_timeout_sec").value,
                    "mode_transition_timeout_sec",
                ),
                cancel_on_shutdown=False,
            )
            error = _generic_response_error(disabled, "DISABLED mode request")
            if error is not None:
                errors.append(error)
        except Exception as error:
            errors.append(f"disable failed: {error}")
        try:
            released = self._query_with_timeout(
                f"{self.prefix}/rpc/release_session",
                pb.ReleaseSessionRequest(session_id=session_id).SerializeToString(),
                pb.GenericResponse,
                _positive_timeout(
                    self.get_parameter("query_timeout_sec").value,
                    "query_timeout_sec",
                ),
                cancel_on_shutdown=False,
            )
            error = _generic_response_error(released, "session release request")
            if error is not None:
                errors.append(error)
        except Exception as error:
            errors.append(f"release failed: {error}")
        return " and ".join(errors) if errors else None

    def _safe_release(self) -> Optional[str]:
        with self._hardware_transition_lock:
            with self._lock:
                session_id = self._session_id
                self._hardware_active = False
                session_open = self._session is not None
                if session_id == 0:
                    self._runtime_gate_latched = False
                    self._runtime_gate_reason = ""
                    return None
            error = (
                self._release_session(session_id)
                if session_open
                else "Zenoh session is not open"
            )
            if error is not None:
                with self._lock:
                    if self._session_id == session_id:
                        self._runtime_gate_latched = True
                        self._runtime_gate_reason = f"session release failed: {error}"
                return error
            with self._lock:
                if self._session_id == session_id:
                    self._session_id = 0
                    self._runtime_gate_latched = False
                    self._runtime_gate_reason = ""
            return None

    def _close_zenoh(self) -> None:
        with self._hardware_transition_lock:
            with self._lock:
                subscribers = list(self._subscribers)
                self._subscribers.clear()
                session = self._session
                self._session = None
                self._accept_zenoh_state = False
                self._snapshot = _Snapshot()
                self._joint_state_error_log.reset()
            for subscriber in subscribers:
                try:
                    subscriber.undeclare()
                except Exception:
                    pass
            if session is not None:
                try:
                    session.close()
                except Exception:
                    pass
