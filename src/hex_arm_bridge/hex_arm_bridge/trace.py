"""Opt-in bounded transport tracing; CSV I/O never runs in a stream callback."""
from __future__ import annotations

import csv
import os
import queue
import threading
import time
from pathlib import Path


class TransportTrace:
    def __init__(self, capacity: int = 8192, directory: str | None = None) -> None:
        self.enabled = False
        self.dropped = 0
        self.error: str | None = None
        self._pid = os.getpid()
        self._stop = threading.Event()
        self._queue: queue.Queue = queue.Queue(maxsize=capacity)
        self._thread = None
        directory = (os.environ.get("HEX_ARM_TRACE_DIR", "")
                     if directory is None else directory).strip()
        if not directory:
            return
        path = Path(directory)
        path.mkdir(parents=True, exist_ok=True)
        self._file = (path / f"python-{os.getpid()}-{time.monotonic_ns()}.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self._writer = csv.writer(self._file)
        self._writer.writerow(
            ("timestamp_ns", "pid", "stage", "seq", "generation", "source_stamp_ns")
        )
        self.enabled = True
        self._thread = threading.Thread(target=self._drain, name="hex_arm_trace", daemon=True)
        self._thread.start()

    def emit(self, stage: str, seq: int = 0, generation: int = 0,
             source_stamp_ns: int = 0) -> None:
        if not self.enabled:
            return
        event = (time.monotonic_ns(), self._pid, stage, seq, generation, source_stamp_ns)
        try:
            self._queue.put_nowait(event)
        except queue.Full:
            self.dropped += 1

    def _drain(self) -> None:
        try:
            while not self._stop.is_set() or not self._queue.empty():
                try:
                    self._writer.writerow(self._queue.get(timeout=0.05))
                except queue.Empty:
                    self._file.flush()
            if self.dropped:
                self._writer.writerow((time.monotonic_ns(), os.getpid(),
                                       "trace_dropped", self.dropped, 0, 0))
        except Exception as error:
            self.error = str(error)
            self.enabled = False
        finally:
            try:
                self._file.close()
            except Exception as error:
                self.error = str(error)

    def close(self, timeout_sec: float = 0.5) -> bool:
        self.enabled = False
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout_sec)
            drained = not self._thread.is_alive()
            self._thread = None
            return drained
        return True
