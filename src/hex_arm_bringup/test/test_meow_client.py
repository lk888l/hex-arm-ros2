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


def fixture(deployment=False):
    profile = yaml.safe_load((ROOT/'config/hardware/firefly_y6.meow_mit.example.yaml').read_text())
    profile.update(validated=True, calibrated=True)
    if deployment:
        for joint in profile['joints']:
            joint['limits'].update(velocity_rad_s=1.2566370614359172,
                                   acceleration_rad_s2=1.2566370614359172)
    rates = client.startup.resolve_motion_limits(profile)
    if deployment:
        for limits in rates.values():
            limits['acceleration_rad_s2'] = .6
    q = client.startup.FOLDED.copy()
    steps = []
    for label, target, duration in client.startup.startup_steps(
            profile, q, [0.]*6, align_folded=True, motion_limits=rates):
        steps.append(dict(step=label, target=target, initial=q, duration_sec=duration, status=4, error_code=0))
        q = target
    prior = dict(passed=True, ready_hold={}, profile_sha256='exact', steps=steps, motion_limits=rates)
    return profile, prior


@pytest.mark.parametrize('deployment', [False, True])
def test_return_reverses_verified_startup_at_normal_motion_rates(deployment):
    profile, prior = fixture(deployment)
    steps = client.return_steps(profile, prior, 'exact')
    assert [s[0] for s in steps] == ['return_startup_j3', 'return_startup_j4', 'return_startup_j2']
    assert [s[2] for s in steps] == [s['duration_sec'] for s in reversed(prior['steps'][-3:])]
    assert steps[-1][1] == client.startup.FOLDED
    if deployment:
        assert sum(s[2] for s in steps) < 5.


@pytest.mark.parametrize('align', [False, True])
def test_boundary_entry_returns_to_bounded_command_reference(align):
    profile, prior = fixture(deployment=True)
    for joint in profile['joints']:
        joint['limits']['measured_position_margin_rad'] = .01
    initial = client.startup.FOLDED.copy()
    initial[1] = -1.570404052734375
    initial[2] = 1.579
    prior['steps'] = []
    reference = client.startup.folded_entry_command(profile, initial)
    for label, target, duration in client.startup.startup_steps(
            profile, initial, [0.] * 6, align_folded=align, motion_limits=prior['motion_limits']):
        prior['steps'].append(dict(step=label, initial=initial.copy(), target=target,
                                  command_reference=reference.copy(), duration_sec=duration,
                                  status=4, error_code=0))
        reference = target.copy()
        initial = target.copy()
    reverse = client.return_steps(profile, prior, 'exact')
    assert reverse[-1][1] == client.startup.FOLDED
    for _, target, _ in reverse:
        assert target == client.startup.folded_entry_command(profile, target)
    bad = copy.deepcopy(prior)
    bad['steps'][-3]['command_reference'][1] = -1.579
    with pytest.raises(RuntimeError, match='changed command reference'):
        client.return_steps(profile, bad, 'exact')


@pytest.mark.parametrize('fault', ['profile', 'failed', 'deactivated', 'no_hold', 'uncalibrated',
                                   'protocol', 'goal_failed', 'target', 'duration', 'entry', 'missing', 'no_rates'])
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
    if fault == 'no_rates': prior.pop('motion_limits')
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
