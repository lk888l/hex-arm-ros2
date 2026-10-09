#!/usr/bin/env python3
"""Trace-analysis contracts, independent of ROS or attached hardware."""
import csv
import importlib.util
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location("transport_benchmark", Path(__file__).with_name("benchmark-transport.py"))
benchmark = importlib.util.module_from_spec(spec)
spec.loader.exec_module(benchmark)
FIELDS = ("timestamp_ns", "pid", "stage", "seq", "generation", "source_stamp_ns", "span_id")


def write_trace(directory, name, rows):
    with (directory / name).open("w", newline="") as output:
        writer = csv.writer(output)
        writer.writerow(FIELDS)
        for timestamp, pid, stage, seq, generation, source, *span in rows:
            writer.writerow((timestamp, pid, stage, seq, generation, source, span[0] if span else 0))


class TransportAnalysis(unittest.TestCase):
    def test_first_command_consumption_and_send_uses_shared_seq(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            write_trace(directory, "python.csv", [(1_000_000, 1, "python_receive", 5, 42, 123),
                                                  (2_000_000, 1, "zenoh_put", 5, 42, 123)])
            write_trace(directory, "rust.csv", [(3_000_000, 2, "rust_accept", 5, 1, 0),
                (4_000_000, 2, "control_consume", 5, 1, 0),
                (5_000_000, 2, "mailbox_write", 5, 1, 0),
                (6_000_000, 2, "can_send", 5, 1, 0),
                (100_000_000, 2, "can_send", 5, 1, 0)])
            report = benchmark.analyze_run(directory)
            self.assertEqual(report["latencies"]["python_receive->can_send"]["count"], 1)
            self.assertEqual(report["latencies"]["python_receive->can_send"]["p99_ms"], 5.)
            self.assertFalse(report["coverage"]["end_to_end"])

    def test_complete_latency_joins_cxx_stamp_to_independent_python_sequence(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            write_trace(directory, "all.csv", [(1_000_000, 10, "cxx_write", 99, 1, 123456),
                (2_000_000, 11, "python_receive", 7, 888, 123456),
                (3_000_000, 12, "can_send", 7, 200, 0)])
            report = benchmark.analyze_run(directory)
            self.assertEqual(report["latencies"]["cxx_write->can_send"]["p50_ms"], 2.)
            self.assertTrue(report["coverage"]["end_to_end"])
            write_trace(directory, "all.csv", [(1_000_000, 10, "cxx_write", 99, 1, 123456),
                (2_000_000, 11, "python_receive", 7, 888, 654321),
                (3_000_000, 12, "can_send", 7, 200, 0)])
            report = benchmark.analyze_run(directory)
            self.assertTrue(report["coverage"]["cxx"])
            self.assertTrue(report["coverage"]["can"])
            self.assertFalse(report["coverage"]["end_to_end"])

    def test_interleaved_lock_waits_require_span_ids(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            write_trace(directory, "rust.csv", [(1_000_000, 12, "gate_wait_begin", 0, 0, 0, 1),
                (2_000_000, 12, "gate_wait_begin", 0, 0, 0, 2),
                (3_000_000, 12, "gate_wait_end", 0, 0, 0, 2),
                (7_000_000, 12, "gate_wait_end", 0, 0, 0, 1)])
            metrics = benchmark.analyze_run(directory)["latencies"]["gate_wait_begin->gate_wait_end"]
            self.assertEqual(metrics["count"], 2)
            self.assertEqual(metrics["max_ms"], 6.)
            self.assertEqual(metrics["p50_ms"], 3.5)

    def test_feedback_consumption_joins_rust_sequence_to_the_copied_ros_frame(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            # Frame 7 arrives but is replaced by frame 8 before a controller
            # read. Repeated reads of frame 8 measure its first consumption.
            write_trace(directory, "all.csv", [
                (1_000_000, 1, "rust_state_publish", 7, 0, 0),
                (2_000_000, 2, "state_receive", 7, 0, 0),
                (3_000_000, 2, "python_state_publish", 7, 0, 777),
                (4_000_000, 3, "cxx_state_receive", 0, 5, 777),
                (5_000_000, 1, "rust_state_publish", 8, 0, 0),
                (6_000_000, 2, "state_receive", 8, 0, 0),
                (7_000_000, 2, "python_state_publish", 8, 0, 888),
                (8_000_000, 3, "cxx_state_receive", 0, 5, 888),
                (9_000_000, 3, "cxx_read", 0, 5, 888),
                (100_000_000, 3, "cxx_read", 0, 5, 888)])
            report = benchmark.analyze_run(directory)
            self.assertTrue(report["coverage"]["feedback_end_to_end"])
            self.assertTrue(report["coverage"]["feedback_received"])
            self.assertTrue(report["coverage"]["feedback_consumed"])
            delivered = report["latencies"]["rust_state_publish->cxx_state_receive"]
            consumed = report["latencies"]["rust_state_publish->cxx_read"]
            self.assertEqual(delivered["count"], 2)
            self.assertEqual(consumed["count"], 1)
            self.assertEqual(consumed["unmatched_start_count"], 1)
            self.assertEqual(consumed["max_ms"], 4.)
            self.assertEqual(report["latencies"]["state_receive->cxx_read"]["max_ms"], 3.)

    def test_received_feedback_without_a_controller_read_is_not_consumed(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            write_trace(directory, "all.csv", [
                (1, 1, "rust_state_publish", 7, 0, 0),
                (2, 2, "state_receive", 7, 0, 0),
                (3, 2, "python_state_publish", 7, 0, 777),
                (4, 3, "cxx_state_receive", 0, 5, 777)])
            report = benchmark.analyze_run(directory)
            self.assertTrue(report["coverage"]["feedback_end_to_end"])
            self.assertTrue(report["coverage"]["feedback_received"])
            self.assertFalse(report["coverage"]["feedback_consumed"])
            self.assertEqual(report["latencies"]["rust_state_publish->cxx_read"]["count"], 0)

    def test_sampling_window_excludes_warmup_and_retains_terminal_silence(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            write_trace(directory, "python.csv", [(900_000_000, 1, "python_receive", 1, 1, 0),
                (1_100_000_000, 1, "python_receive", 2, 1, 0),
                (1_200_000_000, 1, "python_receive", 3, 1, 0),
                (3_100_000_000, 1, "python_receive", 4, 1, 0)])
            report = benchmark.analyze_run(directory, begin_ns=1_000_000_000, end_ns=3_000_000_000)
            spacing = report["stage_spacing"]["python_receive"]
            self.assertEqual(spacing["samples"], 2)
            self.assertEqual(spacing["max_silent_interval_ms"], 1800.)

    def test_cumulative_trace_drop_snapshots_count_once_per_file(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            write_trace(directory, "rust.csv", [(1, 1, "trace_dropped", 3, 0, 0),
                                                (2, 1, "trace_dropped", 10, 0, 0)])
            write_trace(directory, "cxx.csv", [(3, 2, "trace_dropped", 2, 0, 0),
                                               (4, 2, "cxx_skip", 3, 1, 42)])
            report = benchmark.analyze_run(directory)
            self.assertEqual(report["trace_records_dropped"], 12)
            self.assertEqual(report["command_publish_skipped"], 1)

    def test_overwritten_commands_remain_visible_as_unmatched(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            write_trace(directory, "rust.csv", [(1, 1, "rust_accept", 3, 3, 0),
                (2, 1, "rust_accept", 4, 4, 0), (3, 1, "control_consume", 4, 4, 0)])
            metrics = benchmark.analyze_run(directory)["latencies"]["rust_accept->control_consume"]
            self.assertEqual(metrics["count"], 1)
            self.assertEqual(metrics["unmatched_start_count"], 1)


if __name__ == "__main__":
    unittest.main()
