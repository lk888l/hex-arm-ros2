import threading
from types import SimpleNamespace

import pytest

from hex_arm_tools.gravity_comp import GravityCompClient, validate_damping
from hex_arm_tools.pb import robot_api_pb2 as pb


class FakeSession:
    def __init__(self, responses):
        self.responses = list(responses)
        self.requests = []

    def get(self, key, payload, timeout):
        self.requests.append((key, payload))
        response = self.responses.pop(0)
        if response is None:
            return []
        return [SimpleNamespace(ok=SimpleNamespace(payload=SimpleNamespace(
            to_bytes=response.SerializeToString)))]


def description():
    return pb.RobotDescription(api_version=pb.ApiVersion(major=0, minor=4),
                               supported_modes=[pb.OPERATING_MODE_GRAVITY_COMP])


@pytest.mark.parametrize("values", [[], [1] * 5, [0] * 6, [-1] * 6,
                                    [float("nan")] * 6, [float("inf")] * 6])
def test_damping_rejects_bad_values(values):
    with pytest.raises(ValueError):
        validate_damping(values)


def test_client_owns_session_renews_sequence_and_confirms_release():
    session = FakeSession([description(), pb.AcquireSessionResponse(ok=True, session_id=42),
                           *[pb.GenericResponse(ok=True) for _ in range(4)]])
    client = GravityCompClient(session, "arm", [1.0] * 6)
    client.start(threading.Event())
    client.heartbeat()
    client.stop()
    client.stop()  # idempotent only after confirmed release
    assert client.session_id == 0
    start = pb.StartGravityCompRequest.FromString(session.requests[2][1])
    assert start.session_id == 42 and list(start.damping_nm_s_rad) == [1.0] * 6
    beats = [pb.GravityCompHeartbeatRequest.FromString(payload)
             for key, payload in session.requests if key.endswith("heartbeat")]
    assert [beat.sequence for beat in beats] == [1, 2]
    assert all(beat.session_id == 42 for beat in beats)
    assert session.requests[-1][0] == "arm/rpc/release_session"


def test_start_timeout_keeps_session_for_cleanup_and_never_retries_enable():
    session = FakeSession([description(), pb.AcquireSessionResponse(ok=True, session_id=42),
                           None, pb.GenericResponse(ok=True)])
    client = GravityCompClient(session, "arm", [1.0] * 6)
    with pytest.raises(TimeoutError):
        client.start(threading.Event())
    assert client.session_id == 42
    client.stop()
    assert sum(key.endswith("start_gravity_comp") for key, _ in session.requests) == 1


def test_release_timeout_does_not_claim_disabled_or_forget_ownership():
    client = GravityCompClient(FakeSession([None]), "arm", [1.0] * 6)
    client.session_id = 42
    with pytest.raises(TimeoutError):
        client.stop()
    assert client.session_id == 42


def test_busy_session_never_enables_or_releases_another_owner():
    session = FakeSession([description(), pb.AcquireSessionResponse(
        ok=False, current_holder=7, current_holder_name="ros2_control")])
    client = GravityCompClient(session, "arm", [1.0] * 6)
    with pytest.raises(RuntimeError, match="exclusive session unavailable"):
        client.start(threading.Event())
    client.stop()
    assert len(session.requests) == 2


def test_old_driver_is_rejected_before_session_acquisition():
    old = description()
    old.api_version.minor = 3
    session = FakeSession([old])
    client = GravityCompClient(session, "arm", [1.0] * 6)
    with pytest.raises(RuntimeError, match="does not support"):
        client.start(threading.Event())
    assert len(session.requests) == 1


def test_start_can_be_cancelled_before_discovery():
    cancelled = threading.Event()
    cancelled.set()
    client = GravityCompClient(FakeSession([]), "arm", [1.0] * 6)
    with pytest.raises(RuntimeError, match="cancelled"):
        client.start(cancelled)
