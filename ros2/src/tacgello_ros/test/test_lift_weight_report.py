import csv
import json
import time
import pytest
from types import SimpleNamespace
from xml.etree import ElementTree

from tacgello_ros import lift_weight_report as report
from tacgello_ros.tracking_report_recorder import make_gripper_svg, make_joint_svg


def test_load_wait_diagnostics_saved_per_joint():
    frame = dict(observed_pc_ns=1_000_000_000,
                 joint_cues={'3': dict(load_wait_reason='current spread exceeds 0.08A',
                                       load_window_span_s=.6, load_window_spread_a=.11,
                                       load_max_joint_speed_rad_s=.001)})
    row = report.sample(frame, 0, 1)
    assert row['load_wait_reason_3'] == 'current spread exceeds 0.08A'
    assert row['load_window_spread_a_3'] == .11
    assert row['load_wait_reason_2'] is None
    assert set(row) <= set(report.FIELDS)


def test_absolute_change_report_preserves_negative_direction_and_mapping():
    frame = dict(observed_pc_ns=1_000_000_000,
                 current_mapping='piecewise_current_v4_absolute_delta_half_gain',
                 body_output_gain=.5,
                 joint_cues={'3': dict(accepted_signed_delta_a=-.3,
                                       accepted_absolute_delta_a=.3)})
    row = report.sample(frame, 0, 1)
    assert row['accepted_signed_delta_a_3'] == -.3
    assert row['accepted_absolute_delta_a_3'] == .3
    assert row['accepted_magnitude_increase_a_3'] is None
    assert row['body_output_gain'] == .5
    assert row['current_mapping'] == frame['current_mapping']
    assert set(row) <= set(report.FIELDS)


def test_controller_panels_show_only_corresponding_motor_and_not_robot():
    names = ['shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint']
    summary = {'per_joint': {n: dict(rmse_deg=0, mae_deg=0) for n in names}}
    samples = [current_row(1, 4, 0), current_row(1.2, 4, 7), current_row(1.4, 0, 0)]
    svg = make_joint_svg([0, 2], [[0]*3, [1]*3], [[0]*3, [1]*3], names, summary, samples)
    root = ElementTree.fromstring(svg)
    ns = {'s': 'http://www.w3.org/2000/svg'}
    groups = root.findall('s:g', ns)
    for group in groups:
        overlays = group.findall('s:g[@class="controller-resistance"]', ns)
        expected = group.attrib['data-series'] == 'Controller' and int(group.attrib['data-joint']) < 2
        assert bool(overlays) == expected
    assert 'Peak 4 raw' in svg and 'Peak 7 raw' in svg
    assert 'NOT measured torque/force' in svg
    assert 'controller-output-step' in svg


def test_controller_overlay_clips_times_and_breaks_gaps():
    samples = [current_row(1, 4, 0), current_row(1.2, 4, 0), current_row(2.5, 7, 0)]
    svg = '<svg>' + ''.join(report.controller_overlay(samples, 2, 1, 3, 100, 200, 50, 100)) + '</svg>'
    root = ElementTree.fromstring(svg)
    bands = root.findall('.//rect[@class="controller-resistance-band"]')
    assert float(bands[0].attrib['x']) == 100
    assert float(bands[0].attrib['width']) == pytest.approx(20)
    assert len(root.findall('.//path[@class="controller-output-step"]')) == 1
    unknown = ''.join(report.controller_overlay([], 3, 0, 3, 100, 200, 50, 100))
    assert 'activity unknown' in unknown


def test_controller_output_over_ten_is_not_clipped():
    samples = [current_row(1, 15, 20), current_row(1.2, 15, 20)]
    for row in samples:
        row['output_cap_raw'] = 20
    svg = '<svg>' + ''.join(report.controller_overlay(samples, 2, 0, 2, 100, 200, 50, 100)) + '</svg>'
    root = ElementTree.fromstring(svg)
    point = root.find('.//circle')
    assert float(point.attrib['cy']) == pytest.approx(50+100+42+55-15/20*55)
    assert '>20</text>' in svg and 'Peak 15 raw' in svg


def row(t, active=True, kg=.6):
    return report.sample(dict(observed_pc_ns=int((100+t)*1e9), contact=True,
        estimated_kg=kg, cue_fraction=.25, resistance_enabled=True,
        resistance_active=active, output_limit_raw=3, servo_id=2,
        state='lifting cue', resistance_fault=''), 100, t+.01)


def current_row(t, shoulder, elbow):
    return report.sample(dict(observed_pc_ns=int((100+t)*1e9), source='joint_current',
        contact=True, resistance_enabled=True, resistance_active=bool(shoulder or elbow),
        output_limit_raw=max(shoulder, elbow),
        body_servos=[dict(servo_id=i, resistance_active=bool(v), output_limit_raw=v)
                     for i, v in ((2, shoulder), (3, elbow))],
        joint_cues={str(i):dict(current_a=[1]*6, delta_current_a=[-.1]*6, cue_fraction=.5)
                    for i in (2, 3)}), 100, t+.01)


def test_current_report_separates_joints_without_fabricating_mass():
    samples = [current_row(1, 4, 0), current_row(1.2, 7, 3),
               current_row(1.4, 0, 6), current_row(1.6, 0, 0)]
    regions = report.joint_intervals(samples)
    assert len(regions) == 2
    for r, expected in zip(regions, [(2, 1, 1.4, 7), (3, 1.2, 1.6, 6)]):
        assert (r['servo_id'], r['start_sec'], r['end_sec'], r['peak_output_limit_raw']) == pytest.approx(expected)
    assert samples[0]['delta_a_2'] == -.1
    assert all(r['estimated_kg'] is None for r in samples)
    svg = make_gripper_svg(3, [], [], {}, [], lift_samples=samples)
    ElementTree.fromstring(svg)
    assert 'ID2 shoulder (raw)' in svg and 'ID3 elbow (raw)' in svg
    assert 'Estimated object (kg)' not in svg
    assert '1.000-1.400s; peak 7 raw' in svg
    assert 'NOT measured torque' in report.table([], samples)
    assert 'body_joint_resistance_intervals.csv' in report.table([], samples)


def test_missing_joint_telemetry_does_not_invent_continuation_or_release():
    rows = [current_row(1, 4, 0), current_row(1.2, 4, 0)]
    rows[1]['limit_raw_2'] = None
    rows[1]['active_2'] = False
    regions = report.joint_intervals(rows)
    assert regions[0]['end_sec'] == 1
    assert 'unknown' in regions[0]['end_reason']


def test_pair_output_and_individual_edges_trigger_immediate_telemetry():
    from tacgello_ros.body_resistance import BodyResistancePair
    from tacgello_ros.joint_current_cue import JointCurrentPair
    from tacgello_ros.leader_publisher import TacGelloLeaderPublisher
    pair = BodyResistancePair(None, None)
    pair.motors[2].active = True
    pair.motors[2].applied_limit = 3
    sent = []
    node = SimpleNamespace(body_resistance=pair, body_contact=True,
        get_parameter=lambda _: SimpleNamespace(value=True), lift_weight=JointCurrentPair(),
        last_weight_status=time.monotonic(),
        weight_status_publisher=SimpleNamespace(publish=lambda msg: sent.append(json.loads(msg.data))))
    TacGelloLeaderPublisher.publish_lift_weight(node)
    pair.motors[3].active = True
    pair.motors[3].applied_limit = 2
    TacGelloLeaderPublisher.publish_lift_weight(node)
    pair.motors[3].applied_limit = 4
    TacGelloLeaderPublisher.publish_lift_weight(node)
    pair.motors[2].active = False
    TacGelloLeaderPublisher.publish_lift_weight(node)
    assert len(sent) == 4
    assert all(s['resistance_active'] for s in sent)


def test_source_timestamp_not_receipt_and_malformed_samples():
    assert row(1)['time_sec'] == 1
    assert row(1)['received_sec'] == 1.01
    for frame in ([], {}, {'observed_pc_ns': float('nan')}, {'observed_pc_ns': 1}):
        assert report.sample(frame, 100, 1) is None
    assert row(1, kg=float('nan'))['estimated_kg'] is None


def test_contact_or_estimate_alone_does_not_make_resistance_interval():
    assert report.intervals([row(1, False), row(1.2, False)]) == []
    r = row(1)
    r['contact'] = False
    r['observed_pc_ns'] = 101000000000
    assert report.sample(r, 100, 1.01)['resistance_active'] is False


def test_edges_gaps_and_missing_end_never_extend_to_recording_end():
    rows = [row(1), row(1.2, kg=1.2), row(1.3, False), row(2), row(3)]
    regions = report.intervals(rows)
    assert len(regions) == 3
    assert regions[0]['end_sec'] == pytest.approx(1.3)
    assert abs(regions[0]['mean_estimated_kg'] - .9) < 1e-6
    assert regions[1]['end_sec'] == 2
    assert 'gap' in regions[1]['end_reason']
    assert regions[2]['end_sec'] == 3
    assert 'not observed' in regions[2]['end_reason']
    assert len(report.intervals([rows[0], rows[0]])) == 1


def test_svg_and_table_labels_do_not_claim_measured_force():
    rows = [row(1), row(1.2), row(1.4, False)]
    svg = make_gripper_svg(4, [], [], {}, [], lift_samples=rows)
    root = ElementTree.fromstring(svg)
    assert int(root.attrib['height']) >= 930
    assert 'body-resistance-band' in svg
    assert 'Estimated object (kg)' in svg and 'Shoulder limit (raw)' in svg
    assert '0.60 kg' in report.table(report.intervals(rows))
    assert 'NOT measured torque' in report.table([])


def test_publisher_emits_active_edges_without_waiting_for_period():
    from tacgello_ros.leader_publisher import TacGelloLeaderPublisher
    from tacgello_ros.lift_weight import LiftWeight
    sent = []
    resistance = SimpleNamespace(active=False, fault='', servo_id=2, applied_limit=3)
    node = SimpleNamespace(body_resistance=resistance, body_contact=True,
        get_parameter=lambda _: SimpleNamespace(value=True), lift_weight=LiftWeight(),
        last_weight_status=time.monotonic(),
        weight_status_publisher=SimpleNamespace(publish=lambda msg: sent.append(json.loads(msg.data))))
    TacGelloLeaderPublisher.publish_lift_weight(node)
    resistance.active = True
    TacGelloLeaderPublisher.publish_lift_weight(node)
    resistance.active = False
    TacGelloLeaderPublisher.publish_lift_weight(node)
    assert [s['resistance_active'] for s in sent] == [False, True, False]
    assert [s['output_limit_raw'] for s in sent] == [0, 3, 0]
    assert all(s['physical_resistance_verified'] is False for s in sent)


def test_full_report_saves_csv_summary_and_svg(tmp_path):
    import rclpy
    from std_msgs.msg import String
    from tacgello_ros.tracking_report_recorder import TrackingReportRecorder
    rclpy.init()
    node = TrackingReportRecorder()
    try:
        node.output_dir = tmp_path
        node.started_monotonic -= 4
        for t, active in ((1, True), (1.2, True), (1.4, False)):
            frame = dict(observed_pc_ns=int((node.started_monotonic+t)*1e9), contact=True,
                         estimated_kg=.6, resistance_enabled=True, resistance_active=active,
                         output_limit_raw=3)
            # Receipt time is separate and must be recent relative to source.
            normalized = report.sample(frame, node.started_monotonic, t+.01)
            node.lift_weight_samples.append(normalized)
        node.lift_weight_callback(String(data='not json'))
        directory = node.save_report()
        assert len(list(csv.DictReader((directory/'lift_weight.csv').open()))) == 3
        summary = json.loads((directory/'tracking_summary.json').read_text())
        assert len(summary['body_resistance_intervals']) == 1
        assert 'Body resistance during object lift' in (directory/'tracking_report.html').read_text()
        assert 'body-resistance-band' in (directory/'gripper_timeline.svg').read_text()
    finally:
        node.destroy_node()
        rclpy.shutdown()
