from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Any, Optional
from hex_arm_bridge.pb import robot_api_pb2 as pb

@dataclass
class _Snapshot:
    joint_state: pb.JointState | None = None
    driver_state: pb.DriverState | None = None
    joint_received_at: float = 0.0
    driver_received_at: float = 0.0


def _positive_timeout(value: Any, name: str) -> float:
    timeout = float(value)
    if not math.isfinite(timeout) or timeout <= 0.0:
        raise ValueError(f"{name} must be finite and greater than zero")
    return timeout


class _BridgeShutdownRequested(RuntimeError):
    """Internal control-flow exception used to abort blocking lifecycle work."""


def _quiesce_multithreaded_executor(executor: Any) -> list[BaseException]:
    """Stop queued callbacks before any node entity is destroyed.

    ROS 2 Jazzy's ``MultiThreadedExecutor.shutdown()`` can destroy its guard
    condition while callbacks submitted to its Python thread pool are still
    queued.  A queued callback which starts after ``destroy_node()`` then
    raises ``InvalidHandle`` ("cannot use Destroyable ..."), and the executor
    never retrieves that task's exception because spinning has already
    stopped.  Quiesce the worker pool first, cancel work which has not started,
    and explicitly retrieve results from every callback which did run.

    This deliberately uses the Jazzy executor's private worker/future lists;
    callers feature-check them and fail closed if the executor implementation
    no longer exposes the required shutdown contract.
    """
    worker_pool = getattr(executor, "_executor", None)
    callback_futures = getattr(executor, "_futures", None)
    if worker_pool is None or callback_futures is None:
        raise RuntimeError(
            "MultiThreadedExecutor does not expose the callback-drain API "
            "required for safe bridge shutdown"
        )

    # No more spin_once() calls are allowed once this starts.  Running bridge
    # callbacks have already received request_stop(), so their Zenoh waits are
    # cooperative; pending callbacks are cancelled without touching ROS
    # entities which are about to be destroyed.
    worker_pool.shutdown(wait=True, cancel_futures=True)

    errors: list[BaseException] = []
    for future in list(callback_futures):
        if not future.done():
            # The corresponding ThreadPoolExecutor item was cancelled before
            # invoking the rclpy Task.  Cancel the rclpy Task as well so its
            # never-started handler coroutine is closed instead of producing
            # a separate "coroutine ... was never awaited" warning.
            future.cancel()
        try:
            future.result()
        except BaseException as error:  # Future.__del__ warns for any BaseException.
            errors.append(error)
    callback_futures.clear()
    return errors


@dataclass
class _RepeatedErrorLog:
    """Keep a high-rate repeated input error visible without flooding logs."""

    interval_sec: float = 5.0
    _current_error: str | None = None
    _last_logged_at: float = 0.0
    _suppressed_count: int = 0

    def __post_init__(self) -> None:
        self.interval_sec = _positive_timeout(
            self.interval_sec, "error log interval"
        )

    def report(self, error: Any, now: float) -> Optional[str]:
        detail = str(error).strip() or type(error).__name__
        error_changed = detail != self._current_error
        interval_elapsed = (
            self._current_error is not None
            and (
                not math.isfinite(now)
                or now < self._last_logged_at
                or now - self._last_logged_at >= self.interval_sec
            )
        )
        if error_changed or interval_elapsed:
            suffix = (
                f" ({self._suppressed_count} repeated samples suppressed)"
                if not error_changed and self._suppressed_count
                else ""
            )
            self._current_error = detail
            self._last_logged_at = now
            self._suppressed_count = 0
            return f"{detail}{suffix}"
        self._suppressed_count += 1
        return None

    def recover(self) -> Optional[str]:
        if self._current_error is None:
            return None
        suffix = (
            f" ({self._suppressed_count} additional repeated samples suppressed)"
            if self._suppressed_count
            else ""
        )
        detail = f"{self._current_error}{suffix}"
        self.reset()
        return detail

    def reset(self) -> None:
        self._current_error = None
        self._last_logged_at = 0.0
        self._suppressed_count = 0


def _configure_zenoh_endpoint(config: Any, connect: Any) -> None:
    """Use a deterministic direct peer route when an endpoint is provided."""
    endpoint = str(connect).strip()
    if not endpoint:
        return
    # A peer can connect directly to the controller's peer listener and still
    # declare/query the bridge API. Zenoh client mode expects a router endpoint
    # and therefore cannot establish this controller-to-bridge peer link.
    config.insert_json5("mode", '"peer"')
    config.insert_json5("connect/endpoints", json.dumps([endpoint]))
    config.insert_json5("scouting/multicast/enabled", "false")


def _generic_response_error(response: Any, operation: str) -> Optional[str]:
    if response is None:
        return f"{operation} returned no response"
    if bool(getattr(response, "ok", False)):
        return None
    detail = str(getattr(response, "error", "")).strip()
    return f"{operation} rejected: {detail}" if detail else f"{operation} rejected"


def _require_fault_timeout_support(supported_timeouts: Any) -> None:
    supported = {int(value) for value in supported_timeouts}
    if pb.TIMEOUT_BEHAVIOR_FAULT not in supported:
        raise RuntimeError(
            "controller does not advertise TIMEOUT_BEHAVIOR_FAULT; "
            "refusing to stream real-hardware commands"
        )


def _observation_error(
    snapshot: _Snapshot, now: float, state_timeout_sec: float
) -> Optional[str]:
    """Return errors that make even read-only live-state observation unreliable."""
    timeout = _positive_timeout(state_timeout_sec, "state_timeout_sec")
    if not math.isfinite(now):
        return "monotonic clock is invalid"

    joint = snapshot.joint_state
    if joint is None:
        return "joint state has not been received"
    positions = getattr(joint, "q", ())
    if len(positions) != 6 or not all(math.isfinite(float(value)) for value in positions):
        return "joint state does not contain six finite positions"
    joint_age = now - snapshot.joint_received_at
    if (
        not math.isfinite(snapshot.joint_received_at)
        or snapshot.joint_received_at <= 0.0
        or joint_age < 0.0
        or joint_age > timeout
    ):
        return "joint state is stale"

    driver = snapshot.driver_state
    if driver is None:
        return "driver state has not been received"
    driver_age = now - snapshot.driver_received_at
    if (
        not math.isfinite(snapshot.driver_received_at)
        or snapshot.driver_received_at <= 0.0
        or driver_age < 0.0
        or driver_age > timeout
    ):
        return "driver state is stale"
    if not bool(getattr(driver, "profile_valid", False)):
        return "hardware profile is not valid"
    if not bool(getattr(driver, "all_motors_online", False)):
        return "not all motors are online"
    if not bool(getattr(driver, "feedback_fresh", False)):
        return "motor feedback is not fresh"
    return None


def _readiness_error(
    snapshot: _Snapshot, now: float, state_timeout_sec: float
) -> Optional[str]:
    """Return the first fail-closed error that blocks torque-producing modes."""
    observation_error = _observation_error(snapshot, now, state_timeout_sec)
    if observation_error is not None:
        return observation_error
    driver = snapshot.driver_state
    assert driver is not None
    if not bool(getattr(driver, "calibrated", False)):
        return "hardware is not calibrated"
    if bool(getattr(driver, "fault_latched", True)):
        reason = str(getattr(driver, "fault_reason", "")).strip()
        return f"controller fault is latched: {reason}" if reason else "controller fault is latched"
    return None


def _active_ownership_error(
    snapshot: _Snapshot,
    now: float,
    state_timeout_sec: float,
    active_mode: int,
    driver_received_after: Optional[float] = None,
) -> Optional[str]:
    readiness_error = _readiness_error(snapshot, now, state_timeout_sec)
    if readiness_error is not None:
        return readiness_error
    if (
        driver_received_after is not None
        and snapshot.driver_received_at <= driver_received_after
    ):
        return "no new driver state received after ACTIVE request"
    driver = snapshot.driver_state
    if not bool(getattr(driver, "session_owned", False)):
        return "driver does not confirm session ownership"
    if int(getattr(driver, "mode", -1)) != int(active_mode):
        return "driver does not confirm ACTIVE mode"
    return None
