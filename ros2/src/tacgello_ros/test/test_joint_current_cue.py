from types import SimpleNamespace
import time

import pytest

from tacgello_ros.joint_current_cue import JointCurrentCue
from tacgello_ros.joint_current_cue import JointCurrentPair


def test_pair_independent_current_and_shared_baseline_gate():
    cue = JointCurrentPair()
    cue.request_tare(0)
    for i in range(1, 62):
        sample(cue, i*.02)
    for i in range(62, 100):
        cue.update([1, 2, 1, 1, 1, 1], [0]*6, i*.02, i*.02, True, True)
    assert cue.efforts()[2] > 0
    assert cue.efforts()[3] == 0
    cue.invalidate('read failed')
    assert cue.efforts() == {2: 0, 3: 0}
    cue.request_tare(2)
    for i in range(101, 162):
        cue.update([1, 1, 1+(i%2), 1, 1, 1], [0]*6, i*.02, i*.02, False, True)
    assert cue.cues[2].baseline is not None
    assert cue.cues[3].baseline is None
    assert sample(cue, 3.24, 2, True) == 0
    assert cue.efforts() == {2: 0, 3: 0}


def sample(cue, t, current=1., contact=False, fresh=True, velocity=0.):
    return cue.update([current]*6, [velocity]*6, t, t, contact, fresh)


def ready(servo_id=2):
    cue = JointCurrentCue(servo_id)
    cue.request_tare(0)
    for i in range(1, 62):
        assert sample(cue, i*.02) == 0
    assert cue.baseline == [1.]*6
    return cue


def test_no_automatic_baseline_and_no_mass_claim():
    cue = JointCurrentCue()
    assert sample(cue, 1, 2, True) == 0
    assert cue.baseline is None
    assert cue.status()['estimated_kg'] is None
    assert not cue.status()['posture_compensated']


def test_startup_auto_waits_for_fresh_empty_stationary_data():
    cue = JointCurrentCue(auto_tare=True)
    for i in range(1, 30):
        sample(cue, i*.02, contact=True)
    for i in range(30, 60):
        sample(cue, i*.02, velocity=.1)
    for i in range(60, 90):
        sample(cue, i*.02, fresh=False)
    assert cue.auto_tare_pending and not cue.taring
    assert cue.baseline is None
    for i in range(90, 152):
        assert sample(cue, i*.02) == 0
    assert cue.baseline == [1.]*6
    assert not cue.auto_tare_pending
    for i in range(152, 220):
        sample(cue, i*.02, current=1.5)
    assert cue.baseline == [1.]*6


def test_auto_timeout_does_not_retry_or_enable_output():
    cue = JointCurrentCue(auto_tare=True)
    sample(cue, .02)
    for i in range(2, 600):
        assert sample(cue, i*.02, contact=True) == 0
    assert not cue.taring and not cue.auto_tare_pending
    for i in range(600, 680):
        sample(cue, i*.02)
    assert cue.baseline is None


def test_pair_auto_collects_both_once_and_manual_tare_still_works():
    cue = JointCurrentPair(auto_tare=True)
    for i in range(1, 63):
        assert sample(cue, i*.02) == 0
    assert all(c.baseline == [1.]*6 for c in cue.cues.values())
    assert not cue.status()['auto_tare_pending']
    cue.request_tare(1.26)
    assert all(c.baseline is None for c in cue.cues.values())
    for i in range(63, 125):
        sample(cue, i*.02, current=1.2)
    assert all(c.baseline == [1.2]*6 for c in cue.cues.values())


@pytest.mark.parametrize('kwargs', [dict(contact=True), dict(fresh=False), dict(velocity=.1)])
def test_baseline_requires_empty_fresh_stationary_tool(kwargs):
    cue = JointCurrentCue()
    cue.request_tare(0)
    for i in range(1, 80):
        sample(cue, i*.02, **kwargs)
    assert cue.baseline is None
    cue.check_tare_timeout(11)
    assert not cue.taring


def test_unstable_baseline_not_accepted():
    cue = JointCurrentCue()
    cue.request_tare(0)
    for i in range(1, 100):
        sample(cue, i*.02, 1 + (i % 2)*.2)
    assert cue.baseline is None


def test_bounded_ramp_and_immediate_contact_release():
    cue = ready()
    previous = 0
    for i in range(62, 200):
        result = sample(cue, i*.02, 2, True)
        assert 0 <= result <= 1
        assert result - previous <= .010001
        previous = result
    assert result > .9
    assert sample(cue, 4, 2, False) == 0


def test_stronger_current_mapping_keeps_fraction_cap():
    cue = ready()
    for i in range(62, 400):
        sample(cue, i*.02, 1.3, True)
    assert cue.effort == pytest.approx(.25/.35, abs=.001)


@pytest.mark.parametrize('increase,expected', [(.3, .25/.035), (.4, 10), (.575, 15), (.75, 20), (2., 20)])
def test_extended_ceiling_preserves_gain_and_raw_slew(increase, expected):
    cue = JointCurrentCue(output_limit=20)
    cue.request_tare(0)
    for i in range(1, 62):
        sample(cue, i*.02)
    previous = 0
    for i in range(62, 700):
        output = sample(cue, i*.02, 1+increase, True)*20
        assert output-previous <= .100001
        assert 0 <= output <= 20
        previous = output
    assert output == pytest.approx(expected, abs=.001)
    assert cue.status()['output_cap_raw'] == 20


def test_forty_trial_ramps_slowly_and_releases_on_contact_loss():
    cue = JointCurrentPair(output_limit=40, current_range_a=.35)
    cue.request_tare(0)
    for i in range(1, 62):
        sample(cue, i*.02)
    previous = 0
    for i in range(62, 700):
        output = sample(cue, i*.02, 1.4, True)*40
        assert 0 <= output <= 40
        assert output - previous <= .100001
        previous = output
    assert output == pytest.approx(40, abs=.001)
    assert sample(cue, 14, 1.4, False) == 0
    assert cue.efforts() == {2: 0, 3: 0}


def test_threshold_hysteresis_prevents_valid_current_on_off_chatter():
    cue = ready()
    for i in range(62, 100):
        sample(cue, i*.02, 1.1, True)
    for i in range(100, 250):
        assert sample(cue, i*.02, 1.04 if i % 2 else 1.06, True) > 0
    for i in range(250, 800):
        sample(cue, i*.02, 1, True)
    assert cue.effort == 0 and not cue.current_latched


def test_valid_current_output_falls_gradually_but_contact_loss_is_immediate():
    cue = ready()
    for i in range(62, 200):
        sample(cue, i*.02, 1.4, True)
    previous = cue.effort
    for i in range(200, 230):
        sample(cue, i*.02, 1, True)
        assert previous-cue.effort <= .010001
        previous = cue.effort
    assert sample(cue, 4.6, 1.4, False) == 0
    assert not cue.current_latched


@pytest.mark.parametrize('kind', ['duplicate', 'reset', 'gap', 'nan', 'heartbeat'])
def test_bad_data_removes_effort(kind):
    cue = ready()
    sample(cue, 1.24, 2, True)
    assert cue.effort > 0
    t = {'duplicate':1.24, 'reset':0., 'gap':2.}.get(kind, 1.26)
    assert sample(cue, t, float('nan') if kind == 'nan' else 2, True, kind != 'heartbeat') == 0
    if kind == 'reset':
        assert cue.baseline is None


def test_only_selected_joint_controls_cue():
    cue = ready(3)
    for i in range(62, 100):
        assert cue.update([1, 2, 1, 1, 1, 1], [0]*6, i*.02, i*.02, True, True) == 0
    assert cue.update([1, 1, 2, 1, 1, 1], [0]*6, 2, 2, True, True) > 0


def test_current_source_reads_current_not_tcp_force():
    from tacgello_ros.leader_publisher import TacGelloLeaderPublisher
    receive = SimpleNamespace(isConnected=lambda: True, getTimestamp=lambda: 1.,
                              getActualCurrent=lambda: [1.]*6, getActualQd=lambda: [0.]*6)
    node = SimpleNamespace(lift_weight=JointCurrentCue(), body_contact_time=time.monotonic(),
                           body_contact=False, rtde_receive_interface=receive)
    assert TacGelloLeaderPublisher.update_lift_weight(node) == 0
    assert node.lift_weight.current == [1.]*6


def contrast_ready(servo_id=3):
    cue = JointCurrentCue(servo_id, output_limit=40, contrast=True, pregrasp=True)
    cue.request_tare(-1.24)
    for i in range(-61, 1):
        cue.update([1.]*6, [0.]*6, i*.02, i*.02, False, True, [0.]*6)
    for i in range(1, 62):
        cue.update([1.]*6, [0.]*6, i*.02, i*.02, False, True, [0.]*6)
    return cue


@pytest.mark.parametrize('servo', [2, 3, 4])
@pytest.mark.parametrize('direction', [-1, 1])
def test_contrast_accepts_both_signed_changes_with_moderate_gain(servo, direction):
    cue = contrast_ready(servo)
    cue.pregrasp = False
    delta = direction * cue.contrast_mid
    for i in range(62, 260):
        cue.update([1 + delta]*6, [0]*6, i*.02, i*.02, True, True, [0]*6)
    assert cue.effort == pytest.approx(.24)
    status = cue.status()
    assert status['accepted_signed_delta_a'] == pytest.approx(delta)
    assert status['accepted_absolute_delta_a'] == pytest.approx(abs(delta))
    assert status['accepted_magnitude_increase_a'] is None
    assert not status['posture_compensated']
    assert cue.update([1 + delta]*6, [0]*6, 5.2, 5.2, False, True, [0]*6) == 0


def test_absolute_delta_does_not_hide_unstable_sign_changes():
    cue = contrast_ready()
    cue.pregrasp = False
    for i in range(62, 160):
        assert cue.update([1 + (-.3 if i % 2 else .3)]*6, [0]*6,
                          i*.02, i*.02, True, True, [0]*6) == 0
    assert cue.load_received is None
    assert cue.load_window_spread == pytest.approx(.6)


def test_negative_delta_while_moving_cannot_establish_cue():
    cue = contrast_ready()
    cue.pregrasp = False
    for i in range(62, 160):
        assert cue.update([.4]*6, [.1]*6, i*.02, i*.02, True, True, [0]*6) == 0
    assert cue.load_received is None


def test_three_joint_wrist_current_is_independent_and_bounded():
    cue = JointCurrentPair(output_limit=40, contrast=True, wrist=True, pregrasp=True)
    cue.request_tare(-1.24)
    for i in range(-61, 1):
        cue.update([1.]*6, [0.]*6, i*.02, i*.02, False, True, [0.]*6)
    for i in range(1, 62):
        cue.update([1.]*6, [0.]*6, i*.02, i*.02, False, True, [0.]*6)
    for i in range(62, 400):
        cue.update([1, 1, 1, 1.2, 1, 1], [0.]*6, i*.02, i*.02, True, True, [0.]*6)
    assert cue.efforts()[2] == cue.efforts()[3] == 0
    assert cue.efforts()[4] == pytest.approx(.36)
    assert cue.status()['joint_cues']['4']['output_cap_raw'] == 10
    assert cue.status()['cue_joints'] == [2, 3, 4]
    cue.invalidate('stale')
    assert cue.efforts() == {2:0, 3:0, 4:0}


def test_pregrasp_uses_recent_baseline_and_freezes_during_contact():
    cue = contrast_ready()
    for i in range(62, 160):
        cue.update([1.2]*6, [0.]*6, i*.02, i*.02, False, True, [0.]*6)
    for i in range(160, 400):
        cue.update([1.5]*6, [0.]*6, i*.02, i*.02, True, True, [0.]*6)
    assert cue.baseline == [1.2]*6
    assert cue.accepted_increase == pytest.approx(.3)
    assert cue.status()['baseline_source'] == 'pre_grasp_stationary'
    assert cue.status()['grasp_baseline_valid']
    cue.update([1.5]*6, [0.]*6, 8, 8, False, True, [0.]*6)
    assert cue.effort == 0
    cue.update([1.5]*6, [0.]*6, 8.02, 8.02, True, True, [0.]*6)
    assert not cue.grasp_baseline_valid
    assert cue.effort == 0


def test_old_pregrasp_baseline_not_used_after_moving_for_two_seconds():
    cue = contrast_ready()
    for i in range(62, 180):
        cue.update([1.]*6, [.1]*6, i*.02, i*.02, False, True, [0.]*6)
    for i in range(180, 300):
        assert cue.update([2.]*6, [0.]*6, i*.02, i*.02, True, True, [0.]*6) == 0
    assert not cue.grasp_baseline_valid


def test_fault_does_not_reuse_frozen_grasp_baseline():
    cue = contrast_ready()
    for i in range(62, 180):
        cue.update([1.3]*6, [0.]*6, i*.02, i*.02, True, True, [0.]*6)
    assert cue.effort > 0
    cue.invalidate('lost contact heartbeat')
    for i in range(180, 250):
        assert cue.update([1.3]*6, [0.]*6, i*.02, i*.02, True, True, [0.]*6) == 0
    assert not cue.grasp_baseline_valid


def test_startup_reference_needs_no_pregrasp_pause_and_excludes_motion():
    cue = JointCurrentPair(auto_tare=True, output_limit=40, contrast=True, wrist=True)
    for i in range(1, 63):
        cue.update([1.]*6, [0.]*6, i*.02, i*.02, False, True, [0.]*6)
    assert all(c.baseline == [1.]*6 for c in cue.cues.values())
    # Approach and grasp while moving: no new baseline or motion-derived load.
    for i in range(63, 200):
        assert cue.update([3.]*6, [.1]*6, i*.02, i*.02, i >= 100, True, [.2]*6) == 0
    assert all(c.baseline == [1.]*6 for c in cue.cues.values())
    # A different stationary pose is allowed but explicitly not compensated.
    for i in range(200, 500):
        cue.update([1.3]*6, [0.]*6, i*.02, i*.02, True, True, [.2]*6)
    assert all(e > 0 for e in cue.efforts().values())
    assert cue.status()['baseline_source'] == 'startup_or_manual'
    assert cue.status()['grasp_baseline_valid']
    assert not cue.status()['posture_compensated']
    targets = {i: c.load_target for i, c in cue.cues.items()}
    for i in range(500, 600):
        cue.update([4.]*6, [.1]*6, i*.02, i*.02, True, True, [.3]*6)
    assert {i: c.load_target for i, c in cue.cues.items()} == targets
    for i in range(600, 800):
        cue.update([4.]*6, [.1]*6, i*.02, i*.02, True, True, [.3]*6)
    assert all(c.effort == pytest.approx(targets[i]) for i, c in cue.cues.items())
    cue.update([1.3]*6, [0.]*6, 16, 16, False, True, [.2]*6)
    assert cue.efforts() == {2:0, 3:0, 4:0}
    assert all(c.baseline == [1.]*6 for c in cue.cues.values())


def test_startup_contrast_stale_contact_releases_all_joints():
    cue = JointCurrentPair(auto_tare=True, output_limit=40, contrast=True, wrist=True)
    for i in range(1, 400):
        cue.update([1. if i < 63 else 1.4]*6, [0.]*6, i*.02, i*.02,
                   i >= 63, True, [0.]*6)
    assert all(e > 0 for e in cue.efforts().values())
    assert cue.update([1.4]*6, [0.]*6, 8, 8, True, False, [0.]*6) == 0
    assert cue.efforts() == {2:0, 3:0, 4:0}


def test_half_second_load_window_and_specific_wait_reasons():
    cue = JointCurrentCue(auto_tare=True, output_limit=40, contrast=True)
    for i in range(1, 63):
        cue.update([1.]*6, [0.]*6, i*.02, i*.02, False, True, [0.]*6)
    for i in range(63, 88):
        assert cue.update([1.3]*6, [0.]*6, i*.02, i*.02, True, True, [0.]*6) == 0
    assert 'collecting' in cue.load_wait_reason
    cue.update([1.3]*6, [0.]*6, 1.76, 1.76, True, True, [0.]*6)
    assert cue.load_wait_reason == 'accepted'
    assert cue.effort > 0
    target = cue.load_target
    cue.update([3.]*6, [.1]*6, 1.78, 1.78, True, True, [0.]*6)
    assert 'speed' in cue.load_wait_reason
    assert cue.load_target == target
    cue.update([1.]*6, [0.]*6, 1.80, 1.80, False, True, [0.]*6)
    for i in range(91, 140):
        assert cue.update([1.3+(i%2)*.2]*6, [0.]*6, i*.02, i*.02, True, True, [0.]*6) == 0
    assert 'spread' in cue.load_wait_reason


@pytest.mark.parametrize('servo,increase,raw', [(3, .17, 0), (3, .30, 9.6),
                                               (3, .60, 24), (2, .09, 0), (2, .17, 9.6),
                                               (2, .34, 24)])
def test_contrast_curve_and_ramp(servo, increase, raw):
    cue = contrast_ready(servo)
    previous = 0
    for i in range(62, 700):
        result = cue.update([1+increase]*6, [0.]*6, i*.02, i*.02, True, True, [0.]*6)*40
        assert abs(result-previous) <= .100001
        previous = result
    assert result == pytest.approx(raw)
    assert cue.update([2.]*6, [0.]*6, 14, 14, False, True, [0.]*6) == 0


@pytest.mark.parametrize('servo,low,mid,high', [(2, .10, .17, .34), (3, .18, .30, .60),
                                              (4, .08, .15, .30)])
def test_current_mapping_preserves_headroom_and_is_continuous(servo, low, mid, high):
    cue = JointCurrentCue(servo, output_limit=10 if servo == 4 else 40, contrast=True)
    assert cue.contrast_fraction(low) == 0
    assert cue.contrast_fraction(mid) == pytest.approx(.24)
    assert cue.contrast_fraction(high) == pytest.approx(.6)
    assert cue.contrast_fraction(mid-1e-8) == pytest.approx(.24, abs=1e-6)
    assert cue.contrast_fraction(mid+1e-8) == pytest.approx(.24, abs=1e-6)
    outputs = [cue.contrast_fraction(i/1000) for i in range(1001)]
    assert outputs == sorted(outputs)
    assert all(0 <= v <= .6 for v in outputs)
    status = cue.status()
    assert status['contrast_mid_a'] == mid
    assert status['body_output_gain'] == .6
    assert status['current_mapping'] == 'piecewise_current_v5_absolute_delta_gain_0_6'
    assert status['estimated_kg'] is None


@pytest.mark.parametrize('pose,velocity', [([.1]*6, [0.]*6), ([0.]*6, [.1]*6)])
def test_contrast_does_not_learn_from_wrong_pose_or_motion(pose, velocity):
    cue = contrast_ready()
    for i in range(62, 200):
        assert cue.update([2.]*6, velocity, i*.02, i*.02, True, True, pose) == 0


def test_contrast_hold_persists_beyond_five_seconds_but_bad_data_releases():
    cue = contrast_ready()
    for i in range(62, 200):
        cue.update([1.3]*6, [0.]*6, i*.02, i*.02, True, True, [0.]*6)
    target = cue.load_target
    for i in range(200, 300):
        cue.update([4.]*6, [.1]*6, i*.02, i*.02, True, True, [.1]*6)
        assert cue.load_target == target
    for i in range(300, 500):
        cue.update([4.]*6, [.1]*6, i*.02, i*.02, True, True, [.1]*6)
    assert cue.effort == pytest.approx(target)
    assert cue.update([1.3]*6, [0.]*6, 10, 10, True, True, None) == 0
    assert cue.load_received is None
