"""Commissioning entry cases, including nonzero J6 and motion limits."""
import importlib.util
import math
from pathlib import Path

import pytest
import yaml

WORKSPACE = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location("startup_client", WORKSPACE / "scripts/commission-startup-ros.py")
client = importlib.util.module_from_spec(spec)
spec.loader.exec_module(client)


def profile():
    return yaml.safe_load((WORKSPACE / "config/hardware/firefly_y6.meow_mit.example.yaml").read_text())


def test_measured_startup_retains_nearby_placement_and_nonzero_j6():
    q = [.02, -1.565, 1.565, .005, -.02, .217]
    assert client.validate_measured_startup(profile(), q, [0.] * 6) == q
    assert q != client.FOLDED and q != client.READY


@pytest.mark.parametrize("axis", range(5))
def test_measured_startup_placement_window_preserves_qualified_tolerance(axis):
    config = profile()
    for joint in config["joints"]:
        joint["limits"].update(position_lower_rad=-3., position_upper_rad=3.)
    q = client.FOLDED.copy(); q[axis] += client.FOLDED_TOLERANCE[axis]
    assert client.validate_measured_startup(config, q, [0.] * 6) == q
    q[axis] += .0001
    with pytest.raises(RuntimeError, match="default placement window"):
        client.validate_measured_startup(config, q, [0.] * 6)


def test_measured_startup_only_bounds_existing_encoder_margin():
    config = profile()
    q = client.FOLDED.copy(); q[1] = -1.575; q[5] = .217
    with pytest.raises(RuntimeError):
        client.validate_measured_startup(config, q, [0.] * 6)
    config["joints"][1]["limits"]["measured_position_margin_rad"] = .01
    reference = client.validate_measured_startup(config, q, [0.] * 6)
    assert reference[1] == -1.57 and reference[5] == q[5] and q[1] == -1.575
    q[1] = -1.5801
    with pytest.raises(RuntimeError, match="placement window"):
        client.validate_measured_startup(config, q, [0.] * 6)


@pytest.mark.parametrize("fault", ["moving", "nan", "partial", "j6_limit"])
def test_measured_startup_rejects_invalid_feedback_before_enable(fault):
    q, dq = client.FOLDED.copy(), [0.] * 6
    if fault == "moving": dq[4] = .0201
    if fault == "nan": q[0] = math.nan
    if fault == "partial": dq.pop()
    if fault == "j6_limit": q[5] = 2.8
    with pytest.raises(RuntimeError):
        client.validate_measured_startup(profile(), q, dq)


@pytest.mark.parametrize("protocol,hold_flag", [("meow", False), ("meow", True), ("cia402", True)])
def test_automatic_startup_verifies_hold_then_runs_protocol_sequence(tmp_path, monkeypatch, protocol, hold_flag):
    import hashlib
    import json
    import sys
    from types import SimpleNamespace as NS
    config = profile(); config.update(validated=True, calibrated=True)
    config["bus"]["protocol"] = protocol
    profile_path = tmp_path / "profile.yaml"; profile_path.write_text(yaml.safe_dump(config))
    output = tmp_path / "startup.json"
    argv = ["startup", "--profile", str(profile_path), "--allow-motion", "--activate-controllers",
            "--moveit-ready-token", "a" * 32, "--output", str(output)]
    if hold_flag: argv.append("--hold-current")
    monkeypatch.setattr(sys, "argv", argv)
    q = client.FOLDED.copy(); q[5] = .217
    entry = q.copy()
    events = []
    class FakeProbe:
        def __init__(self, profile, **_):
            self.profile = profile
            self.motion_limits = client.resolve_motion_limits(profile)
            self.steps, self.samples, self.commands, self.action_feedback = [], [], [], []
            self.service_timeouts, self.sequence_path_checks = [], []
            self.positions, self.velocity = dict(zip(client.JOINTS, q)), dict.fromkeys(client.JOINTS, 0.)
            self.hold_reference = self.hold_samples_start = self.active_goal = None
            self.hardware_activation_requested = False
            self.fjt, self.hardware = NS(wait_for_server=lambda **_: True), NS(wait_for_service=lambda **_: True)
        def q(self): return q.copy()
        def activate_controllers(self, *, startup_ready=False):
            assert startup_ready is not hold_flag
            events.append("enable_at_measured_position")
            self.enable_position_q = self.q()
            self.hold_reference = client.validate_measured_startup(self.profile, q, [0.] * 6)
            self.hold_samples_start = len(self.samples)
        def list_hardware_components(self):
            return [NS(name="FireflyY6System", plugin_name="hex_arm_hardware/HexArmSystem", state=NS(label="active"))]
        def spin_hold(self, reference, seconds, start, *args):
            assert reference == q
            events.append("verify_hold")
            self.samples.append({"q": self.q(), "dq": [0.] * 6})
            return client.measured_hold_metrics(reference, self.samples[start:])
        def activate_trajectory_controller(self): events.append("trajectory_controller")
        def check_point(self, target): pass
        def direct_step(self, target, duration, label, **_):
            events.append(label)
            q[:] = target
            self.steps.append({"label": label, "target": target.copy()})
        def unlock_moveit_after_hold(self, token):
            assert token == "a" * 32 and events[-1] == "verify_hold"
            events.append("unlock_moveit")
            return True
        def destroy_node(self): events.append("cleanup")
    monkeypatch.setattr(client, "Probe", FakeProbe)
    monkeypatch.setattr(client.rclpy, "init", lambda: None)
    monkeypatch.setattr(client.rclpy, "shutdown", lambda: None)
    client.main()
    report = json.loads(output.read_text())
    assert report["passed"]
    assert report["startup_mode"] == ("measured_hold" if hold_flag else "fixed_sequence")
    assert report["measured_hold"]["reference_q"] == entry
    assert report["ready_hold"]["target"] == (entry if hold_flag else client.READY)
    assert report["moveit_execution_unlocked"]
    if hold_flag:
        client.verified_measured_hold(config, report, hashlib.sha256(profile_path.read_bytes()).hexdigest())
    motion = [] if hold_flag else ["align_j6", "startup_j2", "startup_j4", "startup_j3"]
    assert [step["label"] for step in report["steps"]] == motion
    assert events == ["enable_at_measured_position", "verify_hold", "trajectory_controller",
                      "verify_hold", *motion, "verify_hold", "unlock_moveit", "cleanup"]


@pytest.mark.parametrize("j6", [-2.7, -0.219, 0.497, 2.7])
def test_j6_alignment_preserves_startup_order_and_quintic_limits(j6):
    config = profile()
    q = client.FOLDED.copy()
    q[5] = j6
    steps = client.startup_steps(config, q, [0.0] * 6)
    assert [label for label, _, _ in steps] == ["align_j6", "startup_j2", "startup_j4", "startup_j3"]
    _, target, duration = steps[0]
    assert target == client.FOLDED
    assert 1.875 * abs(j6) / duration <= config["joints"][5]["limits"]["velocity_rad_s"]
    assert 5.774 * abs(j6) / duration**2 <= config["joints"][5]["limits"]["acceleration_rad_s2"]
    assert steps[-1][1] == client.READY


def test_nearly_zero_j6_does_not_add_a_preparation_phase():
    q = client.FOLDED.copy()
    q[5] = 0.005
    assert len(client.startup_steps(profile(), q, [0.0] * 6)) == 3


def test_omitted_protocol_retains_the_meow_folded_startup():
    config = profile()
    del config["bus"]["protocol"]
    assert client.folded_entry_command(config, client.FOLDED) == client.FOLDED
    steps = client.startup_steps(config, client.FOLDED, [0.0] * 6, align_folded=True)
    assert [label for label, _, _ in steps] == ["align_folded", "startup_j2", "startup_j4", "startup_j3"]
    assert steps[-1][1] == client.READY


def test_direct_cold_start_waits_for_inactive_and_fresh_feedback(monkeypatch):
    from types import SimpleNamespace as NS
    clock = [100.0]
    calls = []
    component = NS(name="FireflyY6System", plugin_name="hex_arm_hardware/HexArmSystem", state=NS(id=2))
    future = NS(done=lambda: clock[0] >= 100.1, result=lambda: NS(component=[component]))
    fake = NS(positions={}, received_at=0.0,
        hardware=NS(service_is_ready=lambda: clock[0] >= 100.06,
                    call_async=lambda request: (calls.append("list") or future)))
    def spin(node, **kwargs):
        clock[0] += .02
        if clock[0] >= 100.2:
            fake.positions = dict.fromkeys(client.JOINTS, 0.0)
            fake.received_at = clock[0]
    monkeypatch.setattr(client.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(client.rclpy, "ok", lambda: True)
    monkeypatch.setattr(client.rclpy, "spin_once", spin)
    client.Probe.wait_for_inactive_hardware(fake, 1.)
    assert clock[0] >= 100.2
    assert calls and set(calls) == {"list"}  # no actuator or controller-load requests


def test_direct_cold_start_timeout_never_activates(monkeypatch):
    from types import SimpleNamespace as NS
    clock = [100.0]
    fake = NS(hardware=NS(service_is_ready=lambda: False), positions={})
    monkeypatch.setattr(client.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(client.rclpy, "ok", lambda: True)
    monkeypatch.setattr(client.rclpy, "spin_once", lambda *a, **k: clock.__setitem__(0, clock[0] + .02))
    with pytest.raises(RuntimeError, match="startup deadline"):
        client.Probe.wait_for_inactive_hardware(fake, .1)


@pytest.mark.parametrize("axis,value", [(0, 0.1), (1, -1.35), (5, math.nan), (5, 2.8)])
def test_j6_permission_does_not_accept_wrong_posture_or_invalid_feedback(axis, value):
    q = client.FOLDED.copy()
    q[axis] = value
    with pytest.raises(RuntimeError):
        client.startup_steps(profile(), q, [0.0] * 6)


def test_j6_permission_still_requires_stationary_feedback():
    with pytest.raises(RuntimeError):
        client.startup_steps(profile(), client.FOLDED, [0.0] * 5 + [0.03])


def test_controller_configuration_completes_before_any_enable():
    from types import SimpleNamespace as NS
    events = []
    class Endpoint:
        def __init__(self, name, result): self.name, self.result = name, result
        def wait_for_service(self, **_): return True
        def call_async(self, request):
            events.append((self.name, getattr(request, "name", None)))
            return self.result
    system = NS(name="FireflyY6System", plugin_name="hex_arm_hardware/HexArmSystem", state=NS(id=2))
    fake = NS(
        profile=profile(),
        hardware=Endpoint("list", NS(component=[system])),
        load_controller=Endpoint("load", NS(ok=True)),
        configure_controller=Endpoint("configure", NS(ok=True)),
        list_controllers=Endpoint("controllers", NS(controller=[])),
        set_hardware=Endpoint("enable", NS(ok=True, state=NS(id=3))),
        switch=Endpoint("activate_group", NS(ok=True)),
        fjt=NS(wait_for_server=lambda **_: True),
        positions=dict(zip(client.JOINTS, client.FOLDED)),
        velocity=dict.fromkeys(client.JOINTS, 0.0),
        q=lambda: client.FOLDED.copy(),
        check_point=lambda q: None,
        samples=[],
        wait=lambda future, _: future,
        spin=lambda _: events.append(("warm", None)),
        hardware_activation_requested=False,
    )
    fake.list_hardware_components = lambda **_: fake.wait(
        fake.hardware.call_async(client.ListHardwareComponents.Request()), 5.0).component
    client.Probe.activate_controllers(fake)
    assert events == [("list", None), ("controllers", None), ("load", "joint_state_broadcaster"),
                      ("configure", "joint_state_broadcaster"), ("load", "firefly_arm_controller"),
                      ("configure", "firefly_arm_controller"), ("warm", None),
                      ("enable", "FireflyY6System"), ("activate_group", None)]
    events.clear()
    fake.hardware_activation_requested = False
    fake.configure_controller.result = NS(ok=False)
    with pytest.raises(RuntimeError, match="before enable"):
        client.Probe.activate_controllers(fake)
    assert all(event[0] not in ("enable", "activate_group") for event in events)
    assert not fake.hardware_activation_requested

    # A shared controller manager already owns its broadcaster. This arm only
    # prepares its own inactive controller and never restarts the broadcaster.
    events.clear()
    fake.configure_controller.result = NS(ok=True)
    fake.list_controllers.result = NS(controller=[
        NS(name="joint_state_broadcaster", state="active"),
        NS(name="firefly_arm_controller", state="inactive"),
    ])
    client.Probe.activate_controllers(fake)
    assert events == [("list", None), ("controllers", None), ("warm", None),
                      ("enable", "FireflyY6System")]
    events.clear()
    fake.hardware_activation_requested = False
    fake.list_controllers.result.controller[1].state = "active"
    with pytest.raises(RuntimeError, match="must be unconfigured or inactive"):
        client.Probe.activate_controllers(fake)
    assert not fake.hardware_activation_requested
    fake.list_controllers.result = NS(controller=[])
    events.clear()
    fake.configure_controller.result = NS(ok=True)
    fake.profile = profile()
    bad = client.FOLDED.copy(); bad[1] += .151
    fake.q = lambda: bad
    fake.velocity = dict.fromkeys(client.JOINTS, 0.0)
    fake.check_point = lambda q: None
    with pytest.raises(RuntimeError, match="folded reference"):
        client.Probe.activate_controllers(fake, startup_ready=True)
    assert all(event[0] not in ("enable", "activate_group") for event in events)
    assert not fake.hardware_activation_requested


@pytest.mark.parametrize("field,value", [("q", math.nan), ("q", math.inf), ("dq", math.nan),
                                        ("q", 0.016), ("dq", 0.021)])
def test_measured_hold_rejects_bad_sample_even_after_valid_samples(field, value):
    samples = [{"q": [0.0] * 6, "dq": [0.0] * 6} for _ in range(2)]
    samples[1][field][4] = value
    with pytest.raises(RuntimeError):
        client.measured_hold_metrics([0.0] * 6, samples)


def test_measured_hold_checks_reference_empty_samples_and_missing_axes():
    sample = {"q": [0.001] * 6, "dq": [0.002] * 6}
    assert client.measured_hold_metrics([0.0] * 6, [sample]) == ([0.001] * 6, 0.002)
    for reference, samples in [([math.nan] * 6, [sample]), ([0.0] * 6, []),
                               ([0.0] * 6, [{"q": [0.0] * 5, "dq": [0.0] * 6}])]:
        with pytest.raises(RuntimeError):
            client.measured_hold_metrics(reference, samples)


def test_execution_handoff_requires_both_moveit_action_servers_and_preserves_plan_only():
    from types import SimpleNamespace as NS
    published = []
    calls = []
    fake = NS(
        create_publisher=lambda *args: NS(publish=lambda message: published.append(message.data)),
        move_group=NS(wait_for_server=lambda **_: calls.append("move_group") or True),
        trajectory_executor=NS(wait_for_server=lambda **_: calls.append("execute_trajectory") or True),
    )
    assert client.Probe.unlock_moveit_after_hold(fake, "") is False
    assert not published and not calls
    assert client.Probe.unlock_moveit_after_hold(fake, "a" * 32) is True
    assert published == ["a" * 32] and calls == ["move_group", "execute_trajectory"]
    fake.trajectory_executor.wait_for_server = lambda **_: False
    with pytest.raises(RuntimeError, match="did not acknowledge"):
        client.Probe.unlock_moveit_after_hold(fake, "a" * 32)


def test_measured_hold_waits_for_full_gravity_ramp_before_trajectory_activation():
    config = profile()
    config["controller"].pop("gravity_startup_slew_rate_nm_s", None)
    assert client.measured_hold_duration(config) == 3.0
    config["controller"]["gravity_startup_slew_rate_nm_s"] = 1.0
    for joint in config["joints"]:
        joint["limits"]["torque_nm"] = 6.0
    config["joints"][2]["limits"]["torque_nm"] = 7.5
    assert client.measured_hold_duration(config) == 8.5
    for rate in (0, -1, math.nan, math.inf, .01):
        config["controller"]["gravity_startup_slew_rate_nm_s"] = rate
        with pytest.raises(RuntimeError):
            client.measured_hold_duration(config)


@pytest.mark.parametrize("fault", ["partial", "duplicate", "missing_velocity", "nan"])
def test_invalid_state_cannot_refresh_feedback_freshness(fault):
    from types import SimpleNamespace as NS
    message = NS(name=client.JOINTS.copy(), position=[0.0] * 6, velocity=[0.0] * 6)
    if fault == "partial": message.name.pop(); message.position.pop(); message.velocity.pop()
    if fault == "duplicate": message.name[5] = message.name[0]
    if fault == "missing_velocity": message.velocity = []
    if fault == "nan": message.position[2] = math.nan
    fake = NS(positions={}, velocity={}, received_at=0.0, samples=[])
    with pytest.raises(RuntimeError, match="complete finite"):
        client.Probe.state(fake, message)
    assert fake.received_at == 0.0 and not fake.samples


def test_prefixed_feedback_maps_to_physical_profile_and_rejects_other_arm():
    from types import SimpleNamespace as NS
    instance = client.InstanceNames(namespace="left", joint_prefix="left_")
    fake = NS(instance=instance, positions={}, velocity={}, received_at=0., samples=[])
    fake.q = lambda: client.Probe.q(fake)
    message = NS(name=list(reversed(instance.joint_names)),
                 position=[float(index) for index in reversed(range(6))], velocity=[0.] * 6)
    client.Probe.state(fake, message)
    assert fake.q() == [float(index) for index in range(6)]
    assert set(fake.positions) == set(client.JOINTS)
    received = fake.received_at
    message.name = [name.replace("left_", "right_") for name in message.name]
    with pytest.raises(RuntimeError, match="complete finite"):
        client.Probe.state(fake, message)
    assert fake.received_at == received and len(fake.samples) == 1


def test_prefixed_collision_names_and_planning_group_are_scoped():
    from types import SimpleNamespace as NS
    requests = []
    result = NS(valid=False, contacts=[NS(contact_body_1="left_link_1", contact_body_2="left_link_5")])
    fake = NS(instance=client.InstanceNames(joint_prefix="left_", planning_group="left_arm"),
              validity=NS(call_async=lambda request: requests.append(request) or result),
              wait=lambda future, _: future)
    assert client.Probe.is_valid(fake, client.FOLDED, client.FOLDED_CONTACTS) is False
    assert requests[0].group_name == "left_arm"
    assert list(requests[0].robot_state.joint_state.name) == fake.instance.joint_names
    result.contacts[0].contact_body_1 = "right_link_1"
    with pytest.raises(RuntimeError, match="state rejected"):
        client.Probe.is_valid(fake, client.FOLDED, client.FOLDED_CONTACTS)


def test_single_arm_deactivation_keeps_shared_broadcaster_running():
    from types import SimpleNamespace as NS
    requests = []
    instance = client.InstanceNames(namespace="left", controller_manager="/manager",
                                   controller_name="left_arm", hardware_name="LeftArm")
    fake = NS(instance=instance,
              switch=NS(wait_for_service=lambda **_: True,
                        call_async=lambda request: requests.append(request) or NS(ok=True)),
              set_hardware=NS(call_async=lambda request: requests.append(request) or NS(ok=True, state=NS(id=2))),
              wait=lambda future, _: future)
    client.Probe.deactivate(fake)
    assert list(requests[0].deactivate_controllers) == ["left_arm"]
    assert requests[1].name == "LeftArm" and requests[1].target_state.id == 2


@pytest.mark.parametrize("recover", [True, False])
def test_lost_hardware_status_response_has_bounded_retry_and_cleanup(recover):
    from types import SimpleNamespace as NS
    from rclpy.task import Future
    requests, removed, warnings = [], [], []
    def call(_):
        future = Future()
        if recover and len(requests) == 1:
            future.set_result(NS(component=["inactive hardware"]))
        requests.append(future)
        return future
    def wait(future, _):
        if not future.done():
            raise TimeoutError("response reader not matched")
        return future.result()
    fake = NS(
        hardware=NS(call_async=call, remove_pending_request=removed.append),
        wait=wait, service_timeouts=[],
        get_logger=lambda: NS(warning=warnings.append),
        hardware_activation_requested=False,
    )
    if recover:
        assert client.Probe.list_hardware_components(fake, attempts=3) == ["inactive hardware"]
        assert len(requests) == 2 and len(removed) == 1
    else:
        with pytest.raises(TimeoutError, match="after 3 attempts"):
            client.Probe.list_hardware_components(fake, attempts=3)
        assert len(requests) == len(removed) == 3
    assert all(future.cancelled() for future in removed)
    assert len(fake.service_timeouts) == len(removed)
    assert not fake.hardware_activation_requested


def test_small_free_joint_placement_offsets_keep_original_reference_and_order():
    config = profile()
    config['joints'][1]['limits']['measured_position_margin_rad'] = .01
    q = [-0.0202, -1.5708, 1.5654, 0.00035, -0.02056, -0.15945]
    steps = client.startup_steps(config, q, [0.0] * 6)
    assert [s[0] for s in steps] == ["align_j6", "startup_j2", "startup_j4", "startup_j3"]
    assert [steps[0][1][i] for i in (1, 2, 3)] == [-1.57, q[2], q[3]]
    assert steps[1][1][2:4] == q[2:4]
    assert steps[2][1][2] == q[2]
    assert steps[-1][1] == client.READY
    for i in range(6):
        limits = config["joints"][i]["limits"]
        reference = client.folded_entry_command(config, q)
        assert 1.875 * abs(reference[i] - steps[0][1][i]) / steps[0][2] <= limits["velocity_rad_s"]
        assert 5.774 * abs(reference[i] - steps[0][1][i]) / steps[0][2]**2 <= limits["acceleration_rad_s2"]


@pytest.mark.parametrize('axis,measured', [(1, -1.570404052734375), (1, -1.58), (2, 1.58)])
@pytest.mark.parametrize('align', [False, True])
def test_folded_feedback_margin_produces_only_in_range_command_waypoints(axis, measured, align):
    from types import SimpleNamespace as NS
    config = profile()
    config['joints'][axis]['limits']['measured_position_margin_rad'] = .01
    q = client.FOLDED.copy(); q[axis] = measured; q[5] = .217
    limits = config['joints'][axis]['limits'].copy()
    steps = client.startup_steps(config, q, [0.] * 6, align_folded=align)
    fake = NS(profile=config)
    client.Probe.check_point(fake, client.folded_entry_command(config, q))
    for _, target, _ in steps:
        client.Probe.check_point(fake, target)
    assert steps[0][1][axis] == client.FOLDED[axis]
    assert steps[-1][1] == client.READY
    assert config['joints'][axis]['limits'] == limits
    assert q[axis] == measured
    with pytest.raises(RuntimeError, match='profile authority'):
        client.Probe.check_point(fake, q)
    config['joints'][axis]['limits']['measured_position_margin_rad'] = 0.
    with pytest.raises(RuntimeError, match='profile allowance'):
        client.startup_steps(config, q, [0.] * 6, align_folded=align)


@pytest.mark.parametrize('margin', [None, True, -.001, .0101, math.nan, math.inf])
def test_folded_entry_rejects_invalid_feedback_allowances(margin):
    config = profile()
    config['joints'][1]['limits']['measured_position_margin_rad'] = margin
    with pytest.raises(RuntimeError, match='invalid measured position margin'):
        client.folded_entry_command(config, client.FOLDED)


def test_folded_margin_does_not_accept_wrong_posture_or_moving_feedback():
    config = profile()
    for joint in config['joints']:
        joint['limits']['measured_position_margin_rad'] = .01
    for q, dq in [([0., -1.5801, 1.57, 0., 0., 0.], [0.] * 6),
                  ([0., -1.57, 1.5801, 0., 0., 0.], [0.] * 6),
                  (client.FOLDED, [0., .0201, 0., 0., 0., 0.])]:
        with pytest.raises(RuntimeError):
            client.startup_steps(config, q, dq, align_folded=True)


@pytest.mark.parametrize("axis,delta", [(0, 0.0301), (1, 0.0101), (2, -0.0101), (3, 0.0101), (4, -0.0301)])
def test_placement_tolerance_stays_bounded_per_joint(axis, delta):
    q = client.FOLDED.copy()
    q[axis] += delta
    with pytest.raises(RuntimeError, match="folded reference"):
        client.startup_steps(profile(), q, [0.0] * 6)


def test_ordered_startup_without_j6_preparation_holds_untouched_axes():
    q = [-.002, -1.569, 1.563, -.001, -.002, -.0006]
    steps = client.startup_steps(profile(), q, [0.0] * 6)
    assert [s[0] for s in steps] == ["startup_j2", "startup_j4", "startup_j3"]
    assert steps[0][1][2:4] == q[2:4]
    assert steps[1][1][2] == q[2]
    assert steps[2][1] == client.READY


def cia402_fixture():
    config = profile()
    config['bus']['protocol'] = 'cia402'
    recipe = dict(schema_version=1, profile_sha256='bound-profile',
                  entry_position_rad=client.FOLDED.copy(), steps=[
                      dict(joint='joint_3', delta_rad=-.06, duration_sec=20),
                      dict(joint='joint_4', delta_rad=-.06, duration_sec=20),
                      dict(joint='joint_2', delta_rad=.06, duration_sec=20)])
    return config, recipe


def test_cia402_sequence_preserves_other_axes_and_checks_arm_binding():
    config, recipe = cia402_fixture()
    q = client.FOLDED.copy(); q[0] = -.006; q[4] = -.007
    steps = client.cia402_steps(recipe, config, q, 'bound-profile')
    assert [s[0] for s in steps] == ['cia402_joint_3', 'cia402_joint_4', 'cia402_joint_2']
    assert steps[-1][1] == [q[0], q[1]+.06, q[2]-.06, q[3]-.06, q[4], q[5]]
    assert steps[0][1][1] == q[1] and steps[0][1][3] == q[3]
    with pytest.raises(RuntimeError, match='exact qualified'):
        client.cia402_steps(recipe, config, q, 'another-arm')


@pytest.mark.parametrize('fault', ['posture', 'nan', 'order', 'direction', 'travel', 'duration', 'limit', 'rate'])
def test_cia402_sequence_rejects_unqualified_paths_before_enable(fault):
    config, recipe = cia402_fixture()
    q = client.FOLDED.copy()
    if fault == 'posture': q[2] -= .02
    if fault == 'nan': q[5] = math.nan
    if fault == 'order': recipe['steps'].reverse()
    if fault == 'direction': recipe['steps'][0]['delta_rad'] = .06
    if fault == 'travel': recipe['steps'][0]['delta_rad'] = -.061
    if fault == 'duration': recipe['steps'][0]['duration_sec'] = 19
    if fault == 'limit': config['joints'][2]['limits']['position_lower_rad'] = 1.55
    if fault == 'rate': config['joints'][2]['limits']['velocity_rad_s'] = .001
    with pytest.raises(RuntimeError):
        client.cia402_steps(recipe, config, q, 'bound-profile')


def test_folded_exit_permission_does_not_accept_new_or_contactless_invalid_states():
    from types import SimpleNamespace as NS
    def contact(a,b): return NS(contact_body_1=a, contact_body_2=b)
    response = NS(valid=False, contacts=[contact('link_1','link_5')])
    fake = NS(validity=NS(call_async=lambda _: response), wait=lambda x,_: x)
    assert not client.Probe.is_valid(fake, client.FOLDED, client.FOLDED_CONTACTS)
    with pytest.raises(RuntimeError, match='strict MoveIt'):
        client.Probe.is_valid(fake, client.FOLDED)
    for contacts in [[], [contact('link_2','link_5')]]:
        response.contacts = contacts
        with pytest.raises(RuntimeError, match='strict MoveIt'):
            client.Probe.is_valid(fake, client.FOLDED, client.FOLDED_CONTACTS)


def test_sequence_collision_check_only_allows_fold_contacts_during_first_exit():
    from types import SimpleNamespace as NS
    config, recipe = cia402_fixture()
    steps = client.cia402_steps(recipe, config, client.FOLDED, 'bound-profile')
    permissions = []
    fake = NS(validity=NS(wait_for_service=lambda **_: True), check_point=lambda _: None,
              sequence_path_checks=[], is_valid=lambda q,allowed: permissions.append(allowed) or True)
    client.Probe.check_sequence_path(fake, client.FOLDED, steps)
    assert len(permissions) == 123
    assert all(p == client.FOLDED_CONTACTS for p in permissions[:40])
    assert all(not p for p in permissions[40:])


def test_explicit_meow_placement_alignment_preserves_fold_and_motion_rates():
    config = profile()
    q = [-.105, -1.57, 1.57, .0114, .008, -.418]
    with pytest.raises(RuntimeError, match='folded reference'):
        client.startup_steps(config, q, [0.0]*6)
    steps = client.startup_steps(config, q, [0.0]*6, align_folded=True)
    assert [s[0] for s in steps] == ['align_folded','startup_j2','startup_j4','startup_j3']
    assert steps[0][1] == client.FOLDED
    assert steps[-1][1] == client.READY
    for actual, target, joint in zip(q, steps[0][1], config["joints"]):
        assert 1.875*abs(target-actual)/steps[0][2] <= joint["limits"]["velocity_rad_s"]
        assert 5.774*abs(target-actual)/steps[0][2]**2 <= joint["limits"]["acceleration_rad_s2"]
    for axis, value in [(0, -.151), (1,-1.54), (2,1.54), (3,.031), (4,.051)]:
        bad = q.copy(); bad[axis]=value
        with pytest.raises(RuntimeError): client.startup_steps(config,bad,[0.0]*6,align_folded=True)
    config['bus']['protocol']='cia402'
    with pytest.raises(RuntimeError): client.startup_steps(config,q,[0.0]*6,align_folded=True)


def test_explicit_enable_transient_is_time_and_displacement_bounded():
    sample = {"t": 10.1, "q": [.0004] * 6, "dq": [.125] * 6}
    client.measured_hold_metrics([0.] * 6, [sample], enable_started_at=10.)
    with pytest.raises(RuntimeError):
        client.measured_hold_metrics([0.] * 6, [sample])
    for replacement in ({"t": 10.26}, {"t": 9.99}, {"dq": [.151] * 6}, {"q": [.0151] * 6}):
        with pytest.raises(RuntimeError):
            client.measured_hold_metrics([0.] * 6, [dict(sample, **replacement)], enable_started_at=10.)
    ramp = dict(sample, t=11., dq=[.04] * 6)
    client.measured_hold_metrics([0.] * 6, [ramp], enable_started_at=10.)
    with pytest.raises(RuntimeError):
        client.measured_hold_metrics([0.] * 6, [ramp])
    with pytest.raises(RuntimeError):
        client.measured_hold_metrics([0.] * 6, [dict(ramp, dq=[.051] * 6)], enable_started_at=10.)


@pytest.mark.parametrize("seconds", [8.0, 10.0, 6.0, 2, 2.125])
def test_yaml_trajectory_durations_serialize_as_ros_integers(seconds):
    from rclpy.serialization import serialize_message, deserialize_message
    duration = client.trajectory_duration(seconds)
    decoded = deserialize_message(serialize_message(duration), client.Duration)
    assert decoded.sec + decoded.nanosec / 1e9 == seconds


@pytest.mark.parametrize("seconds", [math.nan, math.inf, -1., 0., 121., True])
def test_invalid_trajectory_durations_are_rejected(seconds):
    with pytest.raises(RuntimeError):
        client.trajectory_duration(seconds)


def test_real_hold_acceptance_matches_real_controller_goal_override():
    base = yaml.safe_load((WORKSPACE/'src/hex_arm_bringup/config/controllers.yaml').read_text())
    real = yaml.safe_load((WORKSPACE/'src/hex_arm_bringup/config/controllers_real.yaml').read_text())
    for joint in client.JOINTS:
        assert real['firefly_arm_controller']['ros__parameters']['constraints'][joint]['goal'] == client.HOLD_POSITION_TOLERANCE_RAD
        assert real['firefly_arm_controller']['ros__parameters']['constraints'][joint]['trajectory'] == 2 * base['firefly_arm_controller']['ros__parameters']['constraints'][joint]['trajectory']
    client.measured_hold_metrics([0.]*6, [{'q':[.0149]*6,'dq':[0.]*6}])
    with pytest.raises(RuntimeError):
        client.measured_hold_metrics([0.]*6, [{'q':[.0151]*6,'dq':[0.]*6}])


def test_direct_trajectory_pins_start_derivatives_after_moveit():
    from rclpy.serialization import serialize_message, deserialize_message
    initial = client.READY.copy()
    target = initial.copy(); target[2] += .13
    points = client.rest_to_rest_points(initial, target, 12.)
    assert len(points) == 2
    for point in points:
        decoded = deserialize_message(serialize_message(point), client.JointTrajectoryPoint)
        assert list(decoded.velocities) == [0.] * 6
        assert list(decoded.accelerations) == [0.] * 6
    assert points[0].time_from_start.sec == points[0].time_from_start.nanosec == 0
    assert list(points[0].positions) == initial
    assert list(points[1].positions) == target
    assert points[0].positions[5] == points[1].positions[5]  # Held J6 cannot inherit acceleration.


def test_expanded_trajectory_deadline_tracks_duration_and_rejects_bad_timing():
    points = client.rest_to_rest_points(client.READY, client.READY, 45.)
    assert client.planned_execution_timeout(points) == 55.
    points[1].time_from_start.sec = 5
    assert client.planned_execution_timeout(points) == 30.
    for seconds in (0, -1, 121):
        points[1].time_from_start.sec = seconds
        with pytest.raises(RuntimeError):
            client.planned_execution_timeout(points)
    with pytest.raises(RuntimeError):
        client.planned_execution_timeout([])


@pytest.mark.parametrize("command_reference", [None, "settled"])
@pytest.mark.parametrize("path_tolerance", [None, 0.1])
@pytest.mark.parametrize("prefix", ["", "left_"])
def test_direct_motion_retimes_fresh_feedback_and_preserves_settled_reference(command_reference, path_tolerance, prefix):
    from types import SimpleNamespace as NS
    config = profile()
    for joint in config["joints"]:
        joint["limits"].update(velocity_rad_s=1.2566370614359172,
                               acceleration_rad_s2=1.2566370614359172)
    rates = {name: dict(velocity_rad_s=1.2566370614359172, acceleration_rad_s2=.6)
             for name in client.JOINTS}
    actual = client.READY.copy(); actual[2] += .01
    reference = client.READY.copy() if command_reference else None
    target = client.READY.copy(); target[2] = client.FOLDED[2]
    goals = []
    def send(goal, **_):
        goals.append(goal)
        def result():
            actual[:] = target
            return NS(status=4, result=NS(error_code=0, error_string=""))
        return NS(accepted=True, get_result_async=result)
    initial = actual.copy()
    fake = NS(profile=config, motion_limits=rates, last_commanded_target=reference,
              instance=client.InstanceNames(namespace="left", joint_prefix=prefix),
              q=lambda: actual.copy(), check_point=lambda _: None, action_feedback=[], steps=[],
              shutdown_path_tolerance_rad=path_tolerance,
              fjt=NS(send_goal_async=send), wait=lambda future, _: future, spin=lambda _: None)
    record = client.Probe.direct_step(fake, target, 48., "return_startup_j3", retime=True)
    start, end = goals[0].trajectory.points
    assert list(goals[0].trajectory.joint_names) == fake.instance.joint_names
    assert list(start.positions) == (reference or initial)
    assert list(end.positions) == target
    seconds = end.time_from_start.sec + end.time_from_start.nanosec / 1e9
    assert seconds == record["duration_sec"] < 2.
    for i, name in enumerate(client.JOINTS):
        distance = abs(end.positions[i] - start.positions[i])
        assert 1.875 * distance / seconds <= rates[name]["velocity_rad_s"]
        assert 5.774 * distance / seconds**2 <= rates[name]["acceleration_rad_s2"]
    assert fake.last_commanded_target == target
    assert [(t.name, t.position) for t in goals[0].path_tolerance] == (
        [(name, path_tolerance) for name in fake.instance.joint_names] if path_tolerance else [])
    assert not goals[0].goal_tolerance
    assert record.get("path_tolerance_rad") == path_tolerance


def test_direct_motion_rejects_rates_above_planning_caps_before_sending_goal():
    from types import SimpleNamespace as NS
    config = profile()
    for joint in config["joints"]:
        joint["limits"].update(velocity_rad_s=1.25, acceleration_rad_s2=1.25)
    fake = NS(profile=config, motion_limits={
        name: dict(velocity_rad_s=1.25, acceleration_rad_s2=.6) for name in client.JOINTS},
        last_commanded_target=None, q=lambda: client.READY.copy(), check_point=lambda _: None)
    target = client.READY.copy(); target[3] += .3
    with pytest.raises(RuntimeError, match="quintic motion rates"):
        client.Probe.direct_step(fake, target, 1.2, "too_fast")


def test_first_folded_goal_bounds_feedback_but_retains_actual_pose_in_report():
    from types import SimpleNamespace as NS
    config = profile()
    for joint in config['joints']:
        joint['limits']['measured_position_margin_rad'] = .01
    measured = [-.048891, -1.570404052734375, 1.579, .00154, .01108, .21766]
    actual = measured.copy()
    label, target, duration = client.startup_steps(
        config, measured, [0.] * 6, align_folded=True)[0]
    goals = []
    def send(goal, **_):
        goals.append(goal)
        actual[:] = target
        return NS(accepted=True, get_result_async=lambda: NS(
            status=4, result=NS(error_code=0, error_string='')))
    fake = NS(profile=config, motion_limits=client.resolve_motion_limits(config),
              last_commanded_target=None, q=lambda: actual.copy(),
              folded_entry_checked=False, action_feedback=[], steps=[],
              fjt=NS(send_goal_async=send), wait=lambda future, _: future, spin=lambda _: None)
    fake.check_point = lambda q: client.Probe.check_point(fake, q)
    with pytest.raises(RuntimeError, match='profile authority'):
        client.Probe.direct_step(fake, target, duration, label, retime=True)
    assert not goals
    fake.folded_entry_checked = True
    record = client.Probe.direct_step(fake, target, duration, label, retime=True)
    assert record['initial'] == measured
    assert record['command_reference'][1:3] == [-1.57, 1.57]
    assert list(goals[0].trajectory.points[0].positions) == record['command_reference']
    for point in goals[0].trajectory.points:
        client.Probe.check_point(fake, point.positions)
    outside_goal = target.copy(); outside_goal[1] = measured[1]
    with pytest.raises(RuntimeError, match='profile authority'):
        client.Probe.direct_step(fake, outside_goal, duration, 'invalid_goal', retime=True)
    assert len(goals) == 1


def test_return_to_ready_requests_full_normal_moveit_velocity_and_acceleration():
    from types import SimpleNamespace as NS
    goals = []
    fake = NS(profile=profile(), q=lambda: client.READY.copy(),
              planning_reference=lambda initial: initial,
              check_point=lambda _: None, is_valid=lambda _: True,
              move_group=NS(send_goal_async=lambda goal: goals.append(goal) or NS(accepted=False)),
              wait=lambda future, _: future)
    with pytest.raises(RuntimeError, match="planning goal rejected"):
        client.Probe.plan_and_execute(fake, client.READY)
    assert len(goals) == 1
    assert goals[0].request.max_velocity_scaling_factor == 1.0
    assert goals[0].request.max_acceleration_scaling_factor == 1.0
