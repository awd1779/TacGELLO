from tacgello_ros.lever_feedback_report import intervals
from tacgello_ros.tracking_report_recorder import make_gripper_svg


def frame(t, active, event='heartbeat', fault=''):
    return dict(component='gripper_lever_feedback', observed_pc_ns=int(t*1e9),
                enabled=True, hold_active=active, servo_id=7, hold_target_raw=3700,
                output_limit_raw=300, event=event, fault=fault, reason='')


def test_hold_interval_uses_feedback_not_contact():
    events = [frame(1, False), frame(1.1, True, 'hold_verified'),
              frame(1.2, True), frame(1.3, False, 'release_commanded')]
    rows = intervals(events, 1)
    assert len(rows) == 1
    assert abs(rows[0]['start_sec']-.1) < 1e-6
    assert abs(rows[0]['end_sec']-.3) < 1e-6
    assert rows[0]['output_limit_raw'] == 300
    svg = make_gripper_svg(2, [], [], {}, [(0, True)], origin_pc_s=1, slip_events=events)
    assert 'class="lever-hold-interval"' in svg
    assert 'output limit=300 raw' in svg


def test_failures_are_not_drawn_as_markers_or_hold_regions():
    events = [frame(1, False, 'hold_failed', 'register mismatch'),
              frame(1.1, True, 'release_failed', 'communication failed')]
    assert intervals(events, 0) == []
    svg = make_gripper_svg(2, [], [], {}, [(0, True)], slip_events=events)
    assert 'class="lever-hold-interval"' not in svg
    assert 'class="lever-feedback-fault"' not in svg
    assert 'Red: feedback failure' not in svg
    assert events[0]['fault'] == 'register mismatch'


def test_gaps_and_release_faults_never_claim_continuous_hold():
    events = [frame(1, True), frame(1.1, True), frame(2, True),
              frame(2.1, True, 'release_failed', 'communication failed')]
    rows = intervals(events, 0)
    assert len(rows) == 2
    assert rows[0]['end_sec'] == 1.1
    assert rows[0]['end_reason'] == 'telemetry gap; end unknown'
    assert rows[1]['end_sec'] == 2.1


def test_legacy_contact_does_not_invent_lever_feedback():
    svg = make_gripper_svg(2, [], [], {}, [(0, True)])
    assert 'class="lever-hold-interval"' not in svg
    assert 'Lever feedback telemetry unavailable' in svg
