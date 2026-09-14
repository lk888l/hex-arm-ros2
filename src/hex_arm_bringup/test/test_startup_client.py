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


@pytest.mark.parametrize("j6", [-2.7, -0.219, 0.497, 2.7])
def test_j6_alignment_preserves_startup_order_and_quintic_limits(j6):
    config = profile()
    q = client.FOLDED.copy()
    q[5] = j6
    steps = client.startup_steps(config, q, [0.0] * 6)
    assert [label for label, _, _ in steps] == ["align_j6", "startup_j2", "startup_j4", "startup_j3"]
    _, target, duration = steps[0]
    assert target == client.FOLDED
    assert 1.875 * abs(j6) / duration <= min(0.05, config["joints"][5]["limits"]["velocity_rad_s"] / 2)
    assert 5.774 * abs(j6) / duration**2 <= min(0.05, config["joints"][5]["limits"]["acceleration_rad_s2"] / 2)
    assert steps[-1][1] == client.READY


def test_nearly_zero_j6_does_not_add_a_preparation_phase():
    q = client.FOLDED.copy()
    q[5] = 0.005
    assert len(client.startup_steps(profile(), q, [0.0] * 6)) == 3


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
        hardware=Endpoint("list", NS(component=[system])),
        load_controller=Endpoint("load", NS(ok=True)),
        configure_controller=Endpoint("configure", NS(ok=True)),
        set_hardware=Endpoint("enable", NS(ok=True, state=NS(id=3))),
        switch=Endpoint("activate_group", NS(ok=True)),
        fjt=NS(wait_for_server=lambda **_: True),
        positions=dict(zip(client.JOINTS, client.FOLDED)),
        wait=lambda future, _: future,
        spin=lambda _: events.append(("warm", None)),
        hardware_activation_requested=False,
    )
    fake.list_hardware_components = lambda **_: fake.wait(
        fake.hardware.call_async(client.ListHardwareComponents.Request()), 5.0).component
    client.Probe.activate_controllers(fake)
    assert events == [("list", None), ("load", "joint_state_broadcaster"),
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
    events.clear()
    fake.configure_controller.result = NS(ok=True)
    fake.profile = profile()
    bad = client.FOLDED.copy(); bad[1] = -1.35
    fake.q = lambda: bad
    fake.velocity = dict.fromkeys(client.JOINTS, 0.0)
    fake.check_point = lambda q: None
    with pytest.raises(RuntimeError, match="folded reference"):
        client.Probe.activate_controllers(fake, startup_ready=True)
    assert all(event[0] not in ("enable", "activate_group") for event in events)
    assert not fake.hardware_activation_requested


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
    q = [-0.0202, -1.5708, 1.5654, 0.00035, -0.02056, -0.15945]
    steps = client.startup_steps(profile(), q, [0.0] * 6)
    assert [s[0] for s in steps] == ["align_j6", "startup_j2", "startup_j4", "startup_j3"]
    assert [steps[0][1][i] for i in (1, 2, 3)] == [q[i] for i in (1, 2, 3)]
    assert steps[1][1][2:4] == q[2:4]
    assert steps[2][1][2] == q[2]
    assert steps[-1][1] == client.READY
    for i in range(6):
        assert 1.875 * abs(q[i] - steps[0][1][i]) / steps[0][2] <= 0.05
        assert 5.774 * abs(q[i] - steps[0][1][i]) / steps[0][2]**2 <= 0.05


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
