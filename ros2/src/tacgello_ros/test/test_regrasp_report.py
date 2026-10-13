import unittest
from tacgello_ros.regrasp_report import episodes, table, slip_intervals
from tacgello_ros.regrasp_report import model_high_intervals
from tacgello_ros.tracking_report_recorder import make_gripper_svg
from tacgello_ros.regrasp_report import direction_events, direction_table


class RegraspReportTest(unittest.TestCase):
    def test_persistent_reinforcement_not_labelled_new_slip(self):
        event = dict(component='gripper_regrasp_event', episode_id=1,
                     observed_pc_ns=1000000000, trigger='automatic_slip',
                     reinforcement_reason='persistent_slip',
                     previous_target_raw=[1700, 2300], closing_directions=[-1, 1])
        rows = episodes([event], [], 0)
        self.assertEqual(rows[0]['reinforcement_reason'], 'persistent_slip')
        self.assertIn('persistent slip recheck', table(rows))
        svg = make_gripper_svg(4, [], [], {}, [], reinforcement_events=rows)
        self.assertIn('Persistent slip recheck', svg)

    def test_direction_only_uses_confirmed_start_and_preserves_missing_sensor(self):
        event = dict(component='gripper_slip_observation', transition='started',
                     observed_pc_ns=2000000000, direction=dict(sensors=[dict(sensor=1,
                     magnetic_axis='-Y', mean_delta_xyz=[1, -5, 2], window_sec=.2,
                     coherence=.8, status='magnetic proxy only')]))
        rows = direction_events([event, dict(event, transition='ended')], 1)
        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[0]['time_sec'], 1)
        self.assertEqual(rows[0]['magnetic_axis'], '-Y')
        self.assertEqual(rows[0]['delta_y'], -5)
        self.assertEqual(rows[0]['physical_direction'], 'uncalibrated')
        self.assertIsNone(rows[1]['delta_y'])
        self.assertEqual(rows[1]['status'], 'unavailable')
        self.assertIn('NOT calibrated object-motion', direction_table(rows))
        self.assertIn('gripper_slip_directions.csv', direction_table(rows))

    def test_old_direction_records_are_unavailable_not_guessed(self):
        rows = direction_events([dict(component='gripper_slip_observation',
            transition='started', observed_pc_ns=1000000000)], 0)
        self.assertTrue(all(row['magnetic_axis'] is None for row in rows))
        self.assertEqual(direction_events([], 0), [])

    def frame(self, stamp, score, contact=True, epoch=1):
        return dict(observed_pc_ns=int(stamp*1e9), contact=contact,
                    slip_contact_mask=[True, True],
                    slip_model_prediction=[epoch, [stamp, stamp], [score, .01]])

    def test_interval_starts_at_model_high_not_grasp(self):
        frames = [self.frame(t, s) for t, s in
                  ((1, .01), (2, .01), (2.1, .9), (2.2, .95),
                   (2.3, .01), (2.4, .01), (2.5, .01), (2.6, .01))]
        rows = slip_intervals(frames, 0)
        self.assertEqual(len(rows), 1)
        self.assertAlmostEqual(rows[0]['start_sec'], 2.1)
        self.assertAlmostEqual(rows[0]['end_sec'], 2.3)
        self.assertEqual(rows[0]['end_reason'], 'model cleared')
        svg = make_gripper_svg(4, [], [], {}, [], sensor_events=frames)
        self.assertIn('class="slip-interval"', svg)

    def test_gap_release_and_epoch_never_join_intervals(self):
        frames = [self.frame(1, .9), self.frame(1.1, .9),
                  self.frame(2, .9), self.frame(2.1, .9, epoch=2),
                  self.frame(2.2, .9, contact=False, epoch=2)]
        rows = slip_intervals(frames, 0)
        self.assertEqual(len(rows), 3)
        self.assertEqual(rows[0]['end_sec'], 1.1)
        self.assertEqual(rows[-1]['end_reason'], 'contact released')

    def test_duplicate_low_samples_do_not_end_episode(self):
        high, low = self.frame(1, .9), self.frame(1.1, .01)
        rows = slip_intervals([high, low, low, low], 0)
        self.assertEqual(rows[0]['end_reason'], 'end not observed')

    def observed_frame(self, stamp, active, score=.95, contact=True):
        return dict(self.frame(stamp, score, contact=contact),
                    slip_observation=dict(active=active, observed_pc_s=stamp,
                                          count=1, last_confirmed_pc_s=1.1))

    def test_live_end_not_history_or_high_scores_controls_shading(self):
        frames = [self.observed_frame(t, active) for t, active in
                  ((1, False), (1.1, True), (1.2, True), (1.3, False), (2, False))]
        rows = slip_intervals(frames, 1)
        self.assertEqual(len(rows), 1)
        self.assertAlmostEqual(rows[0]['start_sec'], .1)
        self.assertAlmostEqual(rows[0]['end_sec'], .3)
        self.assertEqual(rows[0]['end_reason'], 'slip inactive')

    def test_live_high_before_confirmation_is_not_shaded(self):
        frames = [self.observed_frame(t, False) for t in (1, 1.1, 1.2)]
        self.assertEqual(slip_intervals(frames, 0), [])

    def test_transition_closes_between_heartbeats(self):
        frames = [self.observed_frame(1, True), self.observed_frame(1.2, False)]
        event = dict(component='gripper_slip_observation', transition='ended',
                     observed_pc_ns=1150000000, observed_pc_s=1.15)
        rows = slip_intervals(frames, 0, [event])
        self.assertEqual(rows[0]['end_sec'], 1.15)
        svg = make_gripper_svg(4, [], [], {}, [], sensor_events=frames, slip_events=[event])
        self.assertIn('Model-latched interval 1.000-1.150s', svg)

    def test_high_evidence_ends_without_erasing_latched_history(self):
        frames = [self.observed_frame(t, True) for t in (1, 1.1, 1.2, 1.3)]
        for frame, high in zip(frames, (True, True, False, False)):
            frame['slip_observation']['high_evidence'] = high
        self.assertEqual(slip_intervals(frames, 0)[0]['end_sec'], 1.3)
        high = model_high_intervals(frames, [], 0)
        self.assertEqual(high[0]['end_sec'], 1.2)
        self.assertIn('physical slip unverified', high[0]['end_reason'])
        svg = make_gripper_svg(4, [], [], {}, [], sensor_events=frames)
        self.assertIn('Model-latched interval 1.000-1.300s', svg)
        self.assertIn('Model-high evidence 1.000-1.200s', svg)
        self.assertIn('NOT verified slip', svg)

    def test_legacy_latched_data_is_not_fabricated_high_evidence(self):
        self.assertEqual(model_high_intervals([self.observed_frame(1, True)], [], 0), [])

    def test_raw_score_plot_keeps_scores_and_breaks_at_gaps(self):
        frames = [self.observed_frame(t, True) for t in (1, 1.1, 2, 2.1)]
        svg = make_gripper_svg(4, [], [], {}, [], sensor_events=frames)
        self.assertEqual(svg.count('class="model-score sensor-1"'), 2)
        self.assertEqual(svg.count('class="model-score sensor-2"'), 2)
        self.assertIn('Raw model scores (not slip ground truth)', svg)
        import xml.etree.ElementTree as ET
        ET.fromstring(svg)

    def test_ack_does_not_end_active_slip_and_gap_is_unknown(self):
        frames = [self.observed_frame(t, True) for t in (1, 1.1, 1.2, 2)]
        command = dict(component='gripper_command', observed_pc_ns=1050000000)
        rows = slip_intervals(frames, 0, [command])
        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[0]['end_sec'], 1.2)
        self.assertEqual(rows[0]['end_reason'], 'observation gap; end unknown')
        self.assertEqual(rows[1]['end_reason'], 'end not observed')

    def test_live_contact_release_and_invalid_timestamp(self):
        frames = [self.observed_frame(1, True), self.observed_frame(1.1, True, contact=False),
                  self.observed_frame(1.2, False)]
        frames[-1]['slip_observation']['observed_pc_s'] = None
        rows = slip_intervals(frames, 0)
        self.assertEqual(rows[0]['end_sec'], 1.1)
        self.assertEqual(rows[0]['end_reason'], 'contact released')

    def test_stages_are_one_episode_and_actual_is_not_command(self):
        event = dict(component='gripper_regrasp_event', episode_id=1,
                     observed_pc_ns=1000000000, trigger='automatic_slip',
                     previous_target_raw=[1757, 2241], closing_directions=[-1, 1])
        commands = [dict(regrasp=True, episode_id=1, send_start_ns=1100000000+i,
                         target_raw=target) for i, target in enumerate(
                             ([1754, 2244], [1745, 2253]))]
        observations = [dict(component='gripper_regrasp_position', episode_id=1,
                             observed_pc_ns=2000000000+i, servo_id=i,
                             closing_delta_raw=delta,
                             before_raw=1757 if i == 1 else 2241,
                             actual_raw=1751 if i == 1 else 2246)
                        for i, delta in ((1, 6), (2, 5))]
        rows = episodes([event] + observations, commands, 0)
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]['command_stages'], 2)
        self.assertEqual(rows[0]['id1_commanded_closing_raw'], 12)
        self.assertEqual(rows[0]['id1_observed_closing_raw'], 6)
        self.assertEqual(rows[0]['id2_observed_closing_raw'], 5)
        svg = make_gripper_svg(4, [], [], {}, [], reinforcement_events=rows)
        self.assertIn('Model event 1 @ 1.000s', svg)
        self.assertIn('ID1: 6 raw', svg)
        self.assertIn('Reinforcement command', svg)
        self.assertIn('ID1 target +12 raw / observed +6 raw', svg)
        self.assertIn('ID2 target +12 raw / observed +5 raw', svg)
        self.assertEqual(rows[0]['id1_before_raw'], 1757)
        self.assertEqual(rows[0]['id2_after_raw'], 2246)
        self.assertIn('1757 -&gt; 1751 (+6 raw)', table(rows))
        self.assertIn('2241 -&gt; 2246 (+5 raw)', table(rows))

    def test_cancelled_missing_and_manual_are_distinct(self):
        events = [dict(component='gripper_regrasp_event', episode_id=i,
                       observed_pc_ns=i*1000000000, trigger=trigger,
                       previous_target_raw=[1700, 2300], closing_directions=[-1, 1])
                  for i, trigger in ((1, 'automatic_slip'), (2, 'manual_probe'))]
        rows = episodes(events, [], 0)
        self.assertEqual(len(rows), 2)
        self.assertIsNone(rows[0]['id1_observed_closing_raw'])
        self.assertEqual(rows[0]['observation'], 'not commanded')
        self.assertEqual(rows[1]['trigger'], 'manual_probe')
        self.assertIn('Unavailable', table(rows))


if __name__ == '__main__':
    unittest.main()
