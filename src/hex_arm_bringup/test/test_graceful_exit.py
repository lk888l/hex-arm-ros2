"""Test real process signals and ordering without ROS or motor access."""
import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location("graceful", ROOT / "scripts/graceful-real-launch.py")
graceful = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(graceful)


@pytest.mark.parametrize("scenario", ["normal", "return_failure", "term", "second_interrupt", "startup_incomplete", "disable_failure"])
def test_exit_order_and_interrupts(tmp_path, scenario):
    wrapper = tmp_path / "graceful-real-launch.py"
    wrapper.write_text((ROOT / "scripts/graceful-real-launch.py").read_text())
    events = tmp_path / "events"
    profile = tmp_path / "profile.yaml"
    profile.write_text("controller: {shutdown_damping: {}}\n")
    launch = tmp_path / "fake_launch.py"
    launch.write_text('''import os, signal, time, json
from pathlib import Path
events = Path(os.environ['EVENTS'])
def record(s):
    with events.open('a') as f: f.write(s + '\\n')
def stop(*args):
    record('disabled')
    raise SystemExit(1 if os.environ['SCENARIO'] == 'disable_failure' else 0)
signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)
if os.environ['SCENARIO'] != 'startup_incomplete':
    Path(os.environ['HEX_ARM_STARTUP_REPORT']).write_text(json.dumps({'passed': True, 'ready_hold': {}}))
record('running')
while True: time.sleep(0.02)
''')
    (tmp_path / "commission-shutdown-ros.py").write_text('''import os, sys, time
from pathlib import Path
events = Path(os.environ['EVENTS'])
def record(s):
    with events.open('a') as f: f.write(s + '\\n')
record('return_ready')
if os.environ['SCENARIO'] == 'return_failure': sys.exit(1)
if os.environ['SCENARIO'] == 'second_interrupt': time.sleep(20)
record('damping')
''')
    with (tmp_path / "output.log").open("w") as output:
        process = subprocess.Popen([sys.executable, str(wrapper), "--profile", str(profile),
            "--scope", "moveit", "--", sys.executable, str(launch)],
            env={**os.environ, "EVENTS": str(events), "SCENARIO": scenario},
            stdout=output, stderr=output, start_new_session=True)
        try:
            deadline = time.monotonic() + 5
            while not events.exists() and time.monotonic() < deadline:
                time.sleep(0.02)
            assert events.exists()
            process.send_signal(signal.SIGTERM if scenario == "term" else signal.SIGINT)
            if scenario == "second_interrupt":
                while "return_ready" not in events.read_text() and time.monotonic() < deadline:
                    time.sleep(0.02)
                process.send_signal(signal.SIGINT)
            code = process.wait(timeout=10)
            order = events.read_text().splitlines()
            assert order == ({
                "normal": ["running", "return_ready", "damping", "disabled"],
                "return_failure": ["running", "return_ready", "disabled"],
                "second_interrupt": ["running", "return_ready", "disabled"],
                "term": ["running", "disabled"],
                "startup_incomplete": ["running", "disabled"],
                "disable_failure": ["running", "return_ready", "damping", "disabled"],
            }[scenario])
            assert code == (1 if scenario in ("return_failure", "second_interrupt", "disable_failure") else 0)
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGTERM)
                process.wait(timeout=10)


def test_cia402_normal_stop_requires_successful_sequence_and_live_holding(tmp_path):
    import json
    profile = {'bus': {'protocol': 'cia402'}}
    path = tmp_path/'startup.json'
    report = {'passed': True, 'ready_hold': {}, 'measured_hold': {}, 'sequence': {}}
    path.write_text(json.dumps(report))
    assert graceful.eligible(profile, path)
    for field in ('sequence', 'measured_hold', 'ready_hold'):
        incomplete = report.copy(); incomplete.pop(field)
        path.write_text(json.dumps(incomplete))
        assert not graceful.eligible(profile, path)
    for field, value in (('passed', False), ('deactivated', True)):
        path.write_text(json.dumps({**report, field: value}))
        assert not graceful.eligible(profile, path)


def test_meow_normal_stop_uses_verified_folded_return(tmp_path):
    import json
    profile = {'bus': {'protocol': 'meow'}}
    path = tmp_path/'startup.json'
    path.write_text(json.dumps({'passed': True, 'ready_hold': {}, 'steps': [{}]}))
    assert graceful.eligible(profile, path)
    command = graceful.stop_command(profile, 'arm.yaml', path, 'stop.json')
    assert Path(command[1]).name == 'commission-meow-ros.py'
    assert '--allow-motion' in command
    profile['controller'] = {'shutdown_damping': {}}
    assert Path(graceful.stop_command(profile, 'arm.yaml', path, 'stop.json')[1]).name == 'commission-shutdown-ros.py'
    for report in ({'passed': True, 'ready_hold': {}, 'steps': []},
                   {'passed': True, 'ready_hold': {}, 'steps': [{}], 'deactivated': True}):
        path.write_text(json.dumps(report))
        assert not graceful.eligible({'bus': {'protocol': 'meow'}}, path)
