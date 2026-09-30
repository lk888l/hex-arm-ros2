"""Validate arm-bound Meow folded returns without motor IO."""
import copy
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location('meow_client', ROOT/'scripts/commission-meow-ros.py')
client = importlib.util.module_from_spec(spec)
spec.loader.exec_module(client)


def fixture():
    profile = yaml.safe_load((ROOT/'config/hardware/firefly_y6.meow_mit.example.yaml').read_text())
    profile.update(validated=True, calibrated=True)
    q = client.startup.FOLDED.copy()
    steps = []
    for label, target, duration in client.startup.startup_steps(profile, q, [0.]*6, align_folded=True):
        steps.append(dict(step=label, target=target, initial=q, duration_sec=duration, status=4, error_code=0))
        q = target
    prior = dict(passed=True, ready_hold={}, profile_sha256='exact', steps=steps)
    return profile, prior


def test_return_reverses_verified_startup_at_half_speed():
    profile, prior = fixture()
    steps = client.return_steps(profile, prior, 'exact')
    assert [s[0] for s in steps] == ['return_startup_j3', 'return_startup_j4', 'return_startup_j2']
    assert [s[2] for s in steps] == [12., 20., 16.]
    assert steps[-1][1] == client.startup.FOLDED


@pytest.mark.parametrize('fault', ['profile', 'failed', 'deactivated', 'no_hold', 'uncalibrated',
                                   'protocol', 'goal_failed', 'target', 'duration', 'entry', 'missing'])
def test_incomplete_or_changed_evidence_never_authorizes_folded_return(fault):
    profile, prior = fixture()
    prior = copy.deepcopy(prior)
    if fault == 'profile': prior['profile_sha256'] = 'different'
    if fault == 'failed': prior['passed'] = False
    if fault == 'deactivated': prior['deactivated'] = True
    if fault == 'no_hold': prior.pop('ready_hold')
    if fault == 'uncalibrated': profile['calibrated'] = False
    if fault == 'protocol': profile['bus']['protocol'] = 'cia402'
    if fault == 'goal_failed': prior['steps'][1]['status'] = 6
    if fault == 'target': prior['steps'][1]['target'][1] += .01
    if fault == 'duration': prior['steps'][1]['duration_sec'] = 1
    if fault == 'entry': prior['steps'][0]['initial'][1] += .1
    if fault == 'missing': prior['steps'].pop()
    with pytest.raises(RuntimeError):
        client.return_steps(profile, prior, 'exact')


def test_campaign_is_profile_bound_and_always_returns_ready():
    profile, _ = fixture()
    target = client.startup.READY.copy()
    target[0] = -.01
    campaign = dict(schema_version=1, profile_sha256='exact',
                    goals=[dict(label='base', position_rad=target, hold_sec=2.)])
    result = client.campaign_goals(campaign, profile, 'exact')
    assert result == [('base', target, 2.), ('return_ready', client.startup.READY, 1.)]
    for fault in ('hash', 'bounds', 'nan', 'short', 'hold', 'empty'):
        changed = copy.deepcopy(campaign)
        if fault == 'hash': changed['profile_sha256'] = 'another-arm'
        if fault == 'bounds': changed['goals'][0]['position_rad'][0] = 10.
        if fault == 'nan': changed['goals'][0]['position_rad'][0] = float('nan')
        if fault == 'short': changed['goals'][0]['position_rad'].pop()
        if fault == 'hold': changed['goals'][0]['hold_sec'] = 61.
        if fault == 'empty': changed['goals'] = []
        with pytest.raises(RuntimeError):
            client.campaign_goals(changed, profile, 'exact')


@pytest.mark.parametrize('value', ['nan', 'inf', '0', '-0.1', '1.01'])
def test_invalid_campaign_scaling_is_rejected(value):
    with pytest.raises(client.argparse.ArgumentTypeError):
        client.planning_scaling(value)


def test_campaign_scaling_accepts_si_limit_fractions():
    assert client.planning_scaling('1') == 1.0
    assert client.planning_scaling('0.5') == 0.5


def inactive_fixture():
    now = client.time.monotonic()
    node = SimpleNamespace(driver=SimpleNamespace(mode=1, session_owned=False, profile_valid=True,
        calibrated=True, all_motors_online=True, feedback_fresh=True, fault_latched=False),
        received_at=now, driver_at=now, positions={f'joint_{i}': 0. for i in range(1,7)},
        velocity={f'joint_{i}': 0. for i in range(1,7)}, q=lambda: [0.]*6)
    components = [SimpleNamespace(name='FireflyY6System', plugin_name='hex_arm_hardware/HexArmSystem',
                                 state=SimpleNamespace(id=2, label='inactive'))]
    return node, components


@pytest.mark.parametrize('fault', ['active', 'owner', 'fault', 'offline', 'joint_age', 'driver_age', 'nonfinite'])
def test_already_stopped_shortcut_requires_fresh_disabled_real_hardware(fault):
    node, components = inactive_fixture()
    assert client.already_inactive(node, components)
    if fault == 'active': components[0].state.id = 3
    if fault == 'owner': node.driver.session_owned = True
    if fault == 'fault': node.driver.fault_latched = True
    if fault == 'offline': node.driver.all_motors_online = False
    if fault == 'joint_age': node.received_at -= 1.
    if fault == 'driver_age': node.driver_at -= 1.
    if fault == 'nonfinite': node.q = lambda: [float('nan')]*6
    assert not client.already_inactive(node, components)


def test_normal_exit_after_campaign_does_not_move_or_enable_again(monkeypatch):
    node, components = inactive_fixture()
    monkeypatch.setattr(client, 'warm_connections', lambda *args, **kwargs: None)
    node.list_hardware_components = lambda: components
    report = {}
    client.run(node, [], False, report)
    assert report == dict(passed=True, deactivated=True, already_inactive=True)
    with pytest.raises(RuntimeError, match='require active'):
        client.run(node, [], True, {})
