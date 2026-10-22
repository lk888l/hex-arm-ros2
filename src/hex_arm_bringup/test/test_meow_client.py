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


def measured_fixture():
    profile, prior = fixture(deployment=True)
    q = [-.12, -1.43, 1.43, .13, -.11, .217]
    prior.update(startup_mode='measured_hold', steps=[],
        measured_hold=dict(enable_position_q=q.copy(), reference_q=q.copy(), q=q.copy(),
                           seconds=client.startup.measured_hold_duration(profile), samples=20,
                           max_error_rad=[0.] * 6, max_velocity_rad_s=0., enable_transient_allowed=False),
        ready_hold=dict(target=q.copy(), q=q.copy(), seconds=client.startup.READY_HOLD_SECONDS,
                        samples=20, max_error_rad=[0.] * 6, max_velocity_rad_s=0.))
    return profile, prior


def test_measured_startup_preserves_fixed_ready_and_folded_shutdown_waypoints():
    profile, prior = measured_fixture()
    legacy_profile, legacy = fixture(deployment=True)
    assert client.return_steps(profile, prior, 'exact') == client.return_steps(legacy_profile, legacy, 'exact')
    assert prior['ready_hold']['target'] != client.startup.READY


@pytest.mark.parametrize('fault', ['profile', 'motion', 'partial', 'raw_reference', 'ready_reference',
                                   'failed_hold', 'moving_hold', 'no_samples', 'short_hold', 'rates'])
def test_changed_or_unverified_measured_startup_never_authorizes_folded_return(fault):
    profile, prior = measured_fixture()
    if fault == 'profile': prior['profile_sha256'] = 'changed'
    if fault == 'motion': prior['steps'] = [{}]
    if fault == 'partial': prior['measured_hold']['reference_q'].pop()
    if fault == 'raw_reference': prior['measured_hold']['enable_position_q'][0] += .02
    if fault == 'ready_reference': prior['ready_hold']['target'] = client.startup.READY.copy()
    if fault == 'failed_hold': prior['measured_hold']['max_error_rad'][0] = .0151
    if fault == 'moving_hold': prior['ready_hold']['max_velocity_rad_s'] = .0201
    if fault == 'no_samples': prior['ready_hold']['samples'] = 0
    if fault == 'short_hold': prior['measured_hold']['seconds'] = 0.01
    if fault == 'rates': prior['motion_limits']['joint_1']['velocity_rad_s'] = 100.
    with pytest.raises((RuntimeError, ValueError)):
        client.return_steps(profile, prior, 'exact')


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
    assert report == dict(passed=True, deactivated=True, already_inactive=True,
                          motion_completed=False, motion_skipped='already_inactive')
    with pytest.raises(RuntimeError, match='require active'):
        client.run(node, [], True, {})


@pytest.mark.parametrize('free_axes', [[0., 0., 0.], [.1, -.1, .217]])
def test_unmoved_folded_start_disables_without_an_unfolding_motion(monkeypatch, free_axes):
    profile, prior = measured_fixture()
    for joint in profile['joints']:
        joint['limits']['measured_position_margin_rad'] = .01
    q = client.startup.FOLDED.copy(); q[1] = -1.5704; q[2] = 1.579
    for axis, value in zip((0, 4, 5), free_axes): q[axis] = value
    events = []
    components = [SimpleNamespace(name='FireflyY6System', plugin_name='hex_arm_hardware/HexArmSystem',
                                 state=SimpleNamespace(id=3, label='active'))]
    node = SimpleNamespace(profile=profile, q=lambda: q.copy(), driver=None,
        list_hardware_components=lambda: components, check_live=lambda: events.append('healthy'),
        prepare_handoff=lambda: events.extend(['cancel_and_confirm', 'stationary']),
        is_valid=lambda reference, contacts: events.append(('collision', reference.copy(), contacts)),
        deactivate=lambda: events.append('disable'))
    monkeypatch.setattr(client, 'warm_connections', lambda *args, **kwargs: None)
    report = {'moveit': []}
    client.run(node, client.return_steps(profile, prior, 'exact'), False, report)
    assert events == ['healthy', 'cancel_and_confirm', 'stationary',
                      ('collision', q, client.startup.FOLDED_CONTACTS), 'disable']
    assert report['passed'] and report['deactivated'] and report['already_folded']
    assert report['folded_q'] == q and report['moveit'] == []


@pytest.mark.parametrize('campaign', [False, True])
def test_shutdown_tolerance_covers_ready_and_folding_without_changing_campaign(monkeypatch, campaign):
    profile, prior = fixture()
    reverse = client.return_steps(profile, prior, 'exact')
    q = client.startup.READY.copy(); q[0] += .02
    planned, folded = [], []
    components = [SimpleNamespace(name='FireflyY6System', plugin_name='hex_arm_hardware/HexArmSystem',
                                 state=SimpleNamespace(id=3, label='active'))]
    node = SimpleNamespace(profile=profile, q=lambda: q.copy(), driver=None, list_hardware_components=lambda: components,
                           check_live=lambda: None, wait_stationary=lambda *args: None,
                           prepare_handoff=lambda: None,
                           check_point=lambda q: None, is_valid=lambda *args: True,
                           sequence_path_checks=[], samples=[], deactivate=lambda: None,
                           spin_hold=lambda *args: ([0.]*6, 0.))
    def plan(target, velocity_scaling, acceleration_scaling):
        planned.append((node.shutdown_path_tolerance_rad, velocity_scaling, acceleration_scaling))
        q[:] = target
        return {}
    def fold(target, duration, label, **kwargs):
        folded.append(node.shutdown_path_tolerance_rad)
        q[:] = target
    node.plan_and_execute, node.direct_step = plan, fold
    monkeypatch.setattr(client, 'warm_connections', lambda *args, **kwargs: None)
    report = {'moveit': []}
    goals = [('return_ready', client.startup.READY, .5)] if campaign else None
    client.run(node, reverse, False, report, goals=goals, velocity_scaling=.8, acceleration_scaling=.6)
    assert planned == ([(None, .25, .25), (None, .8, .6)] if campaign else [(.11, .25, .25)])
    assert folded == [0.11]*3
    assert report['shutdown_path_tolerance_rad'] == 0.11 and report['passed']
