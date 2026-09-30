"""Exercise the real shutdown workflow with fake endpoints, never motor access."""
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import pytest

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location("shutdown_client", ROOT / "scripts/commission-shutdown-ros.py")
client = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(client)


@pytest.mark.parametrize("scenario", ["return", "already_ready", "planning_failure", "damping_failure", "startup_incomplete"])
def test_ready_verification_precedes_damping(tmp_path, monkeypatch, scenario):
    report = tmp_path / "result.json"
    startup = tmp_path / "startup.json"
    startup.write_text(json.dumps({"passed": scenario != "startup_incomplete", "ready_hold": {}}))
    profile = tmp_path / "profile.yaml"
    profile.write_text("controller: {shutdown_damping: {unload_sec: 3}}\n")
    monkeypatch.setattr(sys, "argv", ["shutdown", "--profile", str(profile),
        "--startup-report", str(startup), "--output", str(report)])
    events = []
    q = list(client.startup.READY)
    if scenario != "already_ready":
        q[2] -= client.startup.HOLD_POSITION_TOLERANCE_RAD + 0.005
    class FakeProbe:
        def __init__(self, _):
            events.append("init")
            self.positions = dict(zip(client.startup.JOINTS, q))
            self.driver = object()
            self.active_goal = None
            self.validity = NS(wait_for_service=lambda **_: True)
            self.move_group = self.trajectory_executor = NS(wait_for_server=lambda **_: True)
        def q(self): return q.copy()
        def check_live(self): events.append("healthy")
        def cancel_motion(self): events.append("cancel")
        def wait_stationary(self, target=None):
            if target is not None:
                assert q == list(client.startup.READY)
            events.append("ready_verified" if target is not None else "stationary")
        def plan_and_execute(self, target):
            events.append("plan")
            if scenario == "planning_failure":
                raise RuntimeError("planning failed")
            q[:] = target
            events.append("execute_ready")
            return {"ok": True}
        def create_client(self, _, name):
            assert name == "/hex_arm_bridge/damped_stop"
            def call(_):
                assert "ready_verified" in events
                events.append("damping")
                return NS(success=scenario != "damping_failure", message="damping failed")
            return NS(wait_for_service=lambda **_: True, call_async=call)
        def wait(self, future, _): return future
        def destroy_node(self): events.append("cleanup")
    monkeypatch.setattr(client, "StopProbe", FakeProbe)
    monkeypatch.setattr(client.rclpy, "init", lambda: None)
    monkeypatch.setattr(client.rclpy, "ok", lambda: True)
    monkeypatch.setattr(client.rclpy, "shutdown", lambda: None)
    if scenario in ("planning_failure", "damping_failure", "startup_incomplete"):
        with pytest.raises(RuntimeError):
            client.main()
    else:
        client.main()
    if scenario == "startup_incomplete":
        assert not events
        return
    result = json.loads(report.read_text())
    assert result["passed"] == (scenario in ("return", "already_ready"))
    assert events[:4] == ["init", "healthy", "cancel", "stationary"]
    assert events[-1] == "cleanup"
    if scenario == "planning_failure":
        assert "damping" not in events and "ready_verified" not in events
    else:
        assert events.index("ready_verified") < events.index("damping")
        assert ("execute_ready" in events) == (scenario != "already_ready")


@pytest.mark.parametrize('failure', [None, 'profile_changed', 'reverse_failed'])
def test_cia402_stop_reverses_qualified_sequence_before_disable(tmp_path, monkeypatch, failure):
    import hashlib
    import yaml
    config = yaml.safe_load((ROOT/'config/hardware/firefly_y6.meow_mit.example.yaml').read_text())
    config['bus']['protocol'] = 'cia402'
    config['controller'].pop('shutdown_damping', None)
    profile = tmp_path/'profile.yaml'; profile.write_text(yaml.safe_dump(config))
    digest = hashlib.sha256(profile.read_bytes()).hexdigest()
    recipe = dict(schema_version=1, profile_sha256=digest,
                  entry_position_rad=client.startup.FOLDED.copy(), steps=[
                      dict(joint='joint_3', delta_rad=-.06, duration_sec=20),
                      dict(joint='joint_4', delta_rad=-.06, duration_sec=20),
                      dict(joint='joint_2', delta_rad=.06, duration_sec=20)])
    steps = client.startup.cia402_steps(recipe, config, client.startup.FOLDED, digest)
    prior = dict(passed=True, ready_hold={}, sequence=recipe,
                 measured_hold={'reference_q': client.startup.FOLDED.copy()},
                 profile_sha256='changed' if failure == 'profile_changed' else digest)
    startup = tmp_path/'startup.json'; startup.write_text(json.dumps(prior))
    report = tmp_path/'result.json'
    monkeypatch.setattr(sys, 'argv', ['shutdown', '--profile', str(profile),
        '--startup-report', str(startup), '--output', str(report)])
    events = []; q = steps[-1][1].copy()
    class FakeProbe:
        def __init__(self, _):
            self.positions = dict(zip(client.startup.JOINTS, q))
            self.driver = object(); self.active_goal = None
        def q(self): return q.copy()
        def check_live(self): events.append('healthy')
        def cancel_motion(self): events.append('cancel')
        def wait_stationary(self, target=None): events.append('stationary')
        def check_sequence_path(self, initial, path):
            assert initial == client.startup.FOLDED and path == steps
            events.append('check_path')
        def direct_step(self, target, duration, label):
            events.append(label)
            if failure == 'reverse_failed': raise RuntimeError('reverse failed')
            q[:] = target
        def is_valid(self, q, allowed): events.append('collision_check')
        def deactivate(self):
            assert q == client.startup.FOLDED
            events.append('disable')
        def destroy_node(self): events.append('cleanup')
    monkeypatch.setattr(client, 'StopProbe', FakeProbe)
    monkeypatch.setattr(client.rclpy, 'init', lambda: None)
    monkeypatch.setattr(client.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(client.rclpy, 'shutdown', lambda: None)
    if failure:
        with pytest.raises(RuntimeError): client.main()
    else:
        client.main()
    if failure == 'profile_changed':
        assert not events
        return
    result = json.loads(report.read_text())
    assert result['passed'] == (failure is None)
    if failure is None:
        assert [e for e in events if e.startswith('return_')] == [
            'return_cia402_joint_2', 'return_cia402_joint_4', 'return_cia402_joint_3']
        assert events[-2:] == ['disable', 'cleanup']
    else:
        assert 'disable' not in events  # Owner handles emergency disable after helper failure.
