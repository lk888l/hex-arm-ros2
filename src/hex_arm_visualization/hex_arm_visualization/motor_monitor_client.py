"""Read-only Zenoh observations. This module has no actuator/session-owner API."""
from __future__ import annotations

from dataclasses import dataclass, replace
import json
import threading
import time

from google.protobuf.message import DecodeError

from hex_arm_tools.pb import robot_api_pb2 as pb


DOF = 6


@dataclass(frozen=True)
class Snapshot:
    # Protobuf messages are immutable by convention after entering this cache.
    joint: pb.JointState | None = None
    driver: pb.DriverState | None = None
    joint_at: float | None = None
    driver_at: float | None = None
    names: tuple[str, ...] = ()
    error: str = ""


class MonitorCache:
    def __init__(self, stale_timeout=0.5):
        self.stale_timeout = stale_timeout
        self._lock = threading.Lock()
        self._snapshot = Snapshot()

    def snapshot(self):
        with self._lock:
            return self._snapshot

    def joint(self, message, now=None):
        if not message.HasField("header") or any(
                len(values) != DOF for values in (message.q, message.dq, message.tau_est)):
            raise ValueError("关节状态缺少时间戳或六关节数据")
        now = time.monotonic() if now is None else now
        with self._lock:
            previous = self._snapshot
            restarted = (previous.joint is not None
                         and message.header.stamp_ns < previous.joint.header.stamp_ns)
            recovered = previous.joint_at is None or now - previous.joint_at > self.stale_timeout
            self._snapshot = replace(
                previous, joint=message, joint_at=now, error="",
                names=() if restarted else previous.names,
                driver=None if restarted else previous.driver,
                driver_at=None if restarted else previous.driver_at)
        return restarted or recovered

    def driver(self, message, now=None):
        if not message.HasField("header"):
            raise ValueError("驱动状态缺少时间戳")
        now = time.monotonic() if now is None else now
        with self._lock:
            previous = self._snapshot
            restarted = (previous.driver is not None
                         and message.header.stamp_ns < previous.driver.header.stamp_ns)
            self._snapshot = replace(previous, driver=message, driver_at=now,
                                     names=() if restarted else previous.names)
        return restarted

    def description(self, message):
        names = tuple(message.joint_names)
        if message.dof != DOF or len(names) != DOF or len(set(names)) != DOF or not all(names):
            raise ValueError("设备描述必须包含六个不同的关节名称")
        with self._lock:
            self._snapshot = replace(self._snapshot, names=names, error="")

    def error(self, text):
        with self._lock:
            self._snapshot = replace(self._snapshot, error=text)


class MonitorClient:
    def __init__(self, cache, prefix, endpoint="tcp/127.0.0.1:7448"):
        self.cache = cache
        self.prefix = prefix.rstrip("/")
        self.endpoint = endpoint
        self._stop = threading.Event()
        self._metadata = threading.Event()
        self._thread = threading.Thread(target=self._run, name="motor-monitor-io", daemon=True)

    def start(self):
        self._thread.start()

    def close(self):
        self._stop.set()
        if self._thread.ident is not None:
            self._thread.join(timeout=3.0)
        return not self._thread.is_alive()

    def _receive(self, sample, message_type, update):
        if self._stop.is_set():
            return
        try:
            message = message_type.FromString(bytes(sample.payload.to_bytes()))
            if update(message):
                self._metadata.set()
        except (DecodeError, ValueError) as error:
            self.cache.error(f"状态消息无效：{error}")

    def _describe(self, session):
        # This is the only query issued by the monitor. It never acquires,
        # renews or releases a control session, nor writes an arm command.
        for reply in session.get(f"{self.prefix}/arm/description", timeout=0.3):
            if self._stop.is_set():
                return
            if reply.ok is not None:
                self.cache.description(pb.ArmDescription.FromString(bytes(reply.ok.payload.to_bytes())))
                return
        raise TimeoutError("等待设备描述；请核对 robot-prefix 和 zenoh-connect")

    def _run(self):
        import zenoh

        while not self._stop.is_set():
            session = None
            subscribers = []
            try:
                config = zenoh.Config()
                config.insert_json5("mode", '"peer"')
                config.insert_json5("connect/endpoints", json.dumps([self.endpoint]))
                config.insert_json5("connect/timeout_ms", "1000")
                config.insert_json5("connect/exit_on_failure", "true")
                config.insert_json5("scouting/multicast/enabled", "false")
                session = zenoh.open(config)
                subscribers.append(session.declare_subscriber(
                    f"{self.prefix}/arm/joint_state",
                    lambda sample: self._receive(sample, pb.JointState, self.cache.joint)))
                subscribers.append(session.declare_subscriber(
                    f"{self.prefix}/driver_state",
                    lambda sample: self._receive(sample, pb.DriverState, self.cache.driver)))
                self._metadata.set()
                while not self._stop.is_set():
                    if self._metadata.is_set():
                        self._metadata.clear()
                        try:
                            self._describe(session)
                        except Exception as error:
                            self.cache.error(str(error))
                            self._metadata.set()
                            if self._stop.wait(1.0):
                                break
                    self._stop.wait(0.1)
            except Exception as error:
                self.cache.error(f"连接失败，正在重试：{error}")
            finally:
                for subscriber in subscribers:
                    subscriber.undeclare()
                if session is not None:
                    session.close()
            self._stop.wait(1.0)
