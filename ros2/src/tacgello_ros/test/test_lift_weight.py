import math
from types import SimpleNamespace

import pytest

from tacgello_ros.lift_weight import LiftWeight


def sample(cue, now, kg=0, speed=0, contact=True, fresh=True, stamp=None):
    return cue.update([0, 0, -(5 + kg * 9.80665), 0, 0, 0],
                      [0, 0, speed, 0, 0, 0], now if stamp is None else stamp,
                      now, contact, fresh)


def tared():
    cue = LiftWeight()
    cue.request_tare(0)
    for i in range(61):
        sample(cue, i * .02, contact=False)
    assert cue.zero_n == 5
    assert not cue.taring
    return cue


@pytest.mark.parametrize('kg', [.6, 1.2, 1.8, 2.4, 3.0])
def test_bottles_scale_to_capped_cue(kg):
    cue = tared()
    for i in range(61, 261):
        sample(cue, i * .02, kg=kg, speed=.02)
    assert cue.kg == pytest.approx(kg)
    assert cue.effort == pytest.approx(min(1, kg / 2.4))


def test_no_tare_no_cue():
    cue = LiftWeight()
    for i in range(200):
        assert sample(cue, i * .02, kg=2.4, speed=.02) == 0


@pytest.mark.parametrize('failure', ['contact', 'heartbeat', 'nan', 'fast', 'angular', 'stale', 'restart'])
def test_invalid_input_releases_cue(failure):
    cue = tared()
    for i in range(61, 200):
        sample(cue, i * .02, kg=1.2, speed=.02)
    assert cue.effort > 0
    if failure == 'nan':
        sample(cue, 4, kg=math.nan)
    elif failure == 'fast':
        sample(cue, 4, speed=.5)
    elif failure == 'angular':
        cue.update([0]*6, [0, 0, .02, 0, .2, 0], 4, 4, True, True)
    elif failure == 'stale':
        sample(cue, 4.2, kg=1.2, stamp=3.98)
    elif failure == 'restart':
        sample(cue, 4, kg=1.2, stamp=0)
        assert cue.zero_n is None
    else:
        sample(cue, 4, kg=1.2, contact=failure != 'contact', fresh=failure != 'heartbeat')
    assert cue.effort == 0
    assert cue.kg is None


@pytest.mark.parametrize('speed', [0, -.02])
def test_rest_descent_never_holds(speed):
    cue = tared()
    for i in range(61, 260):
        assert sample(cue, i * .02, kg=2.4, speed=speed) == 0
    assert cue.kg == pytest.approx(2.4)


def test_tare_contact_rejected_and_timeout():
    cue = LiftWeight()
    cue.request_tare(0)
    sample(cue, .02, contact=True)
    assert not cue.taring and cue.zero_n is None
    cue.request_tare(1)
    sample(cue, 12, contact=False)
    assert not cue.taring and cue.zero_n is None


def test_duplicate_samples_cannot_complete_tare():
    cue = LiftWeight()
    cue.request_tare(0)
    for i in range(100):
        sample(cue, i * .02, stamp=0, contact=False)
    assert cue.zero_n is None


def test_slow_fresh_tare_collects_all_25_samples_without_relaxing_stability():
    cue = LiftWeight()
    cue.request_tare(0)
    for i in range(24):
        sample(cue, i * .1, contact=False)
        assert cue.zero_n is None
    sample(cue, 2.4, contact=False)
    assert cue.zero_n == 5 and not cue.taring


def test_failure_reason_persists_until_explicit_retry():
    cue = LiftWeight()
    cue.request_tare(0)
    sample(cue, .1, contact=True)
    error = cue.status()['tare_error']
    for i in range(2, 20):
        sample(cue, i*.1, contact=False)
    assert cue.status()['state'] == error
    assert 'contact detected' in error
    cue.request_tare(2)
    assert cue.status()['tare_error'] is None


def test_timeout_records_blocker_even_without_rtde_updates():
    cue = LiftWeight()
    cue.request_tare(0)
    cue.invalidate('RTDE disconnected')
    cue.check_tare_timeout(11)
    cue.invalidate('another read failure')
    assert cue.status()['state'] == 'tare timed out: RTDE disconnected'
    assert not cue.taring


def test_unstable_force_not_weight_and_ramp_is_bounded():
    cue = tared()
    previous = 0
    for i in range(61, 250):
        effort = sample(cue, i * .02, kg=2.4, speed=.02)
        assert effort - previous <= .020001
        previous = effort
    for i in range(250, 350):
        sample(cue, i * .02, kg=i % 2, speed=.02)
    assert cue.effort == 0 and cue.kg is None


def test_ros_read_failure_and_tare_gates():
    from tacgello_ros.leader_publisher import TacGelloLeaderPublisher
    node = SimpleNamespace(lift_weight=tared(), body_contact_time=None,
                           body_contact=False, body_resistance=None,
                           rtde_receive_interface=None, last_weight_status=float('inf'))
    response = TacGelloLeaderPublisher.tare_lift_weight(node, None, SimpleNamespace())
    assert not response.success
    assert TacGelloLeaderPublisher.update_lift_weight(node) == 0
    assert node.lift_weight.kg is None


def test_ros_snapshot_boundary_retries_without_motion_command():
    import time
    from tacgello_ros.leader_publisher import TacGelloLeaderPublisher
    stamps = iter([1, 2, 2, 2])
    receive = SimpleNamespace(isConnected=lambda: True,
                              getTimestamp=lambda: next(stamps),
                              getActualTCPForce=lambda: [0]*6,
                              getActualTCPSpeed=lambda: [0]*6)
    node = SimpleNamespace(lift_weight=LiftWeight(), body_contact_time=time.monotonic(),
                           body_contact=False, body_resistance=None,
                           rtde_receive_interface=receive, last_weight_status=float('inf'))
    response = TacGelloLeaderPublisher.tare_lift_weight(node, None, SimpleNamespace())
    assert response.success and node.lift_weight.taring
    assert TacGelloLeaderPublisher.update_lift_weight(node) == 0
    assert len(node.lift_weight.samples) == 1
