import ast
from pathlib import Path
from types import SimpleNamespace
import threading
import unittest
from unittest.mock import Mock

import loaded_regrasp as ramp
from gripper_regrasp import bounded_target_check, continuation_error
from gripper_position_tracking import PositionTracker
from learned_slip_regrasp import LearnedRegraspGuard
import math


def diagnostic(servo, goal, actual, stamp=10, load=32):
    return dict(id=servo, ok=True, status=0, servo_status_raw=0, observed_pc_ns=int(stamp*1e9),
                mode_raw=0, torque_enable_raw=1, torque_limit_raw=1000,
                goal_raw=goal, actual_raw=actual, load_raw=load, current_raw=1)


class DecisionSummaryTests(unittest.TestCase):
    def test_reports_goal_match_separately_from_movement(self):
        trace = dict(target=(1747, 2253), before=(1750, 2250), ack_ns=10_000_000_000,
                     observations=[2, 2], motion_counts=[0, 0], attempt=1)
        records = [diagnostic(1, 1747, 1750), diagnostic(2, 2250, 2250)]
        result = ramp.diagnostic_summary(records, trace, 10.5, 'goal mismatch')
        self.assertEqual(result['elapsed_ms'], 500)
        self.assertTrue(result['jaws'][0]['goal_matches'])
        self.assertFalse(result['jaws'][1]['goal_matches'])
        self.assertEqual(result['jaws'][0]['encoder_delta_raw'], 0)
        self.assertEqual(records[0]['goal_raw'], 1747)

    def test_missing_feedback_is_not_reported_as_zero_output(self):
        trace = dict(target=(1747, 2253), before=(1750, 2250), ack_ns=10_000_000_000)
        result = ramp.diagnostic_summary([None, None], trace, 11, 'missing')
        self.assertIsNone(result['jaws'][0]['actual_raw'])
        self.assertIsNone(result['jaws'][0]['pwm_raw'])
        self.assertFalse(result['jaws'][0]['control_settings_available'])
        self.assertIsNone(result['jaws'][0]['position_p_raw'])

    def test_control_settings_preserved_without_changing_motor_command(self):
        trace = dict(target=(1747, 2253), before=(1750, 2250), ack_ns=10_000_000_000)
        record = diagnostic(1, 1747, 1750)
        record.update(control_settings_available=True, position_p_raw=32,
                      position_i_raw=0, position_d_raw=0, minimum_start_output_raw=16,
                      deadband_cw_raw=1, deadband_ccw_raw=1)
        before = dict(record)
        jaw = ramp.diagnostic_summary([record, None], trace, 10.8, None)['jaws'][0]
        self.assertEqual(jaw['position_p_raw'], 32)
        self.assertEqual(jaw['minimum_start_output_raw'], 16)
        self.assertEqual(jaw['deadband_cw_raw'], 1)
        self.assertEqual(record, before)


class PlannerTests(unittest.TestCase):
    def test_normal_closing_lead_is_four_counts_but_opening_stays_eight(self):
        tracker = PositionTracker((2000, 2000), (1520, 2480), (1800, 2200), 2919)
        tracker.command(1, 10)
        self.assertEqual(tracker.tick(10, .02, [[0]*3]*2, [1, 1], True), (1796, 2204))
        tracker.command(0, 10.1)
        self.assertEqual(tracker.tick(10.1, .02, [[0]*3]*2, [2, 2], True), (1808, 2192))

    def test_logged_goal_ramps_beyond_unresolved_small_error(self):
        measured = (1751, 2244)
        previous = origin = (1751, 2246)
        goals = []
        for stage in (1, 2):
            target, error = ramp.next_target(measured, previous, origin, measured,
                                             (2000, 2000), (1520, 2480), stage=stage)
            self.assertIsNone(error)
            goals.append(target)
            previous = target
        self.assertEqual(goals, [(1738, 2259), (1734, 2262)])
        target, error = ramp.next_target(measured, previous, origin, measured,
                                         (2000, 2000), (1520, 2480), stage=3)
        self.assertIsNone(target)
        self.assertIn('budget exhausted', error)
        self.assertIsNone(ramp.next_target(measured, previous, origin, measured,
                                           (2000, 2000), (1520, 2480), stage=4)[0])

    def test_partial_response_still_plans_both_jaws(self):
        target, error = ramp.next_target((1748, 2244), (1748, 2249), (1751, 2246),
            (1751, 2244), (2000, 2000), (1520, 2480), stage=2)
        self.assertIsNone(error)
        self.assertEqual(target, (1744, 2253))

    def test_one_jaw_at_budget_cancels_paired_target(self):
        for previous in ((1733, 2251), (1744, 2262)):
            target, error = ramp.next_target((1751, 2244), previous,
                (1751, 2246), (1751, 2244), (2000, 2000), (1520, 2480), stage=2)
            self.assertIsNone(target)
            self.assertIn('budget exhausted', error)

    def test_endpoint_and_grasp_budget_never_exceeded(self):
        for origin in ((1530, 2470), (1750, 2250)):
            previous = origin
            for _ in range(20):
                target, _ = ramp.next_target(previous, previous, origin, previous,
                    (2000, 2000), (1520, 2480))
                if target is None:
                    break
                self.assertTrue(1520 <= target[0] <= origin[0])
                self.assertTrue(origin[1] <= target[1] <= 2480)
                self.assertTrue(all(abs(v-a) <= 50 for v, a in zip(target, origin)))
                previous = target
            else:
                self.fail('unbounded reinforcement')

    def test_larger_travel_budget_does_not_relax_stalled_goal_error(self):
        target, error = ramp.next_target((1750, 2250), (1731, 2269),
            (1750, 2250), (1750, 2250), (2000, 2000), (1520, 2480), stage=2)
        self.assertIsNone(target)
        self.assertIn('previous goal error', error)

    def test_cumulative_grasp_clamps_at_fifty_and_blocks_next_episode(self):
        origin = (1750, 2250)
        previous = (1701, 2299)
        target, error = ramp.next_target(previous, previous, origin, previous,
                                         (2000, 2000), (1520, 2480))
        self.assertIsNone(error)
        self.assertEqual(target, (1700, 2300))
        next_goal, error = ramp.next_target(target, target, origin, target,
                                            (2000, 2000), (1520, 2480))
        self.assertIsNone(next_goal)
        self.assertIn('budget exhausted', error)

    def test_responding_jaws_can_use_more_than_old_episode_budget(self):
        target, error = ramp.next_target((1738, 2262), (1733, 2267),
            (1750, 2250), (1750, 2250), (2000, 2000), (1520, 2480), stage=3)
        self.assertIsNone(error)
        self.assertEqual(target, (1729, 2271))

    def test_one_jaw_at_new_episode_budget_blocks_both(self):
        target, error = ramp.next_target((1730, 2262), (1725, 2267),
            (1750, 2250), (1750, 2250), (2000, 2000), (1520, 2480), stage=3)
        self.assertIsNone(target)
        self.assertIn('budget exhausted', error)

    def test_feedback_checks_goal_not_only_ack(self):
        records = [diagnostic(1, 1748, 1751), diagnostic(2, 2249, 2244, load=1072)]
        self.assertIsNone(ramp.feedback_error(records, (1748, 2249), 10.1, 9_000_000_000))
        self.assertEqual(ramp.signed_magnitude(1072, 10), -48)
        for field, value in (('goal_raw', 1751), ('torque_enable_raw', 0), ('mode_raw', 2),
                             ('servo_status_raw', 32), ('load_raw', 150), ('load_raw', 1174),
                             ('current_raw', 100), ('current_raw', 32868), ('observed_pc_ns', 0)):
            with self.subTest(field=field, value=value):
                bad = [dict(records[0], **{field: value}), records[1]]
                self.assertIsNotNone(ramp.feedback_error(bad, (1748, 2249), 10.1))
        self.assertIsNotNone(ramp.feedback_error([None, records[1]], (1748, 2249), 10.1))

    def test_episode_uses_confirmed_event_not_continuously_high_score(self):
        episode = dict(started=10, epoch=1)
        self.assertIsNone(ramp.episode_error(episode, (1, (11, 11), (.1, .1)), 11, 1))
        for now, epoch, prediction in ((14, 1, (1, (14, 14), (.9, .9))),
            (11, 2, (2, (11, 11), (.9, .9))), (11, 1, (1, (10, 10), (.9, .9)))):
            self.assertIsNotNone(ramp.episode_error(episode, prediction, now, epoch))


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        tree = ast.parse(Path(__file__).with_name('bluetooth_anyskin_gripper_shell.py').read_text(encoding='utf-8'))
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'RosPositionGripperController')
        names = ('_regrasp_target', '_regrasp_motion_ready', '_record_regrasp_position')
        methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
        self.scope = dict(time=SimpleNamespace(monotonic=lambda: self.now), math=math,
            loaded_regrasp=ramp, bounded_target_check=bounded_target_check,
            continuation_error=continuation_error,
            empty_baseline_magnet_strengths=lambda *args: [[0]*3]*2,
            grouped_contact_deltas=lambda values, expected: values, json=__import__('json'))
        exec(compile(ast.Module(body=methods, type_ignores=[]), '<loaded controller>', 'exec'), self.scope)
        p = PositionTracker((2000, 2000), (1520, 2480), (1751, 2246), 2919)
        p.contact, p.hold_input = True, .6
        p.command(.6, self.now)
        g = LearnedRegraspGuard()
        g.attempts = 1
        g.observe = Mock(return_value=True)
        g.event_preflight_error = Mock(return_value=None)
        self.o = SimpleNamespace(tracker=p, regrasp=g, regrasp_origin=p.target,
            condition=threading.Condition(), probe_armed=False, latest_ratio=.6, latest_received=self.now,
            web_target_active=False, manual_command=None, stop_event=threading.Event(), node=Mock(),
            args=SimpleNamespace(slip_regrasp='true', regrasp_diagnostics='true', config_data={},
                                 safe_empty_baseline_margin=20, speed=2919, acc=22),
            monitor=SimpleNamespace(ports=[1, 2], num_mags=3, raw_calibration_monotonic=1,
                streams=[SimpleNamespace(last_sample_time=self.now) for _ in range(2)],
                magnet_strengths=lambda: [[30]*3]*2),
            slip_scorer=SimpleNamespace(result=(1, (self.now, self.now), (.9, .1))),
            regrasp_motor_records={}, String=lambda: SimpleNamespace(), timing_publisher=Mock())
        self.actual = [1751, 2244]
        self.o._read_regrasp_position = self.read
        self.o._regrasp_motion_ready = lambda t: self.scope['_regrasp_motion_ready'](self.o, t)
        self.o._hold_contact_pose = Mock(side_effect=lambda: self.o.tracker.acknowledge(self.actual))

    def read(self, servo, phase, timeout):
        self.o.regrasp_motor_records[servo] = diagnostic(servo, self.o.tracker.target[servo-1],
                                                        self.actual[servo-1], self.now)
        return self.actual[servo-1]

    def advance(self, seconds):
        self.now += seconds
        self.o.latest_received = self.now
        self.o.tracker.command(self.o.latest_ratio, self.now)
        for stream in self.o.monitor.streams:
            stream.last_sample_time = self.now
        self.o.slip_scorer.result = (1, (self.now, self.now), (.1, .1))

    def target(self):
        return self.scope['_regrasp_target'](self.o, self.now, [[30]*3]*2, [1, 1], True)

    def test_missing_input_has_distinct_reason(self):
        self.o.tracker.received = None
        self.assertIsNone(self.target())
        self.assertEqual(self.o.last_regrasp_block['reason'], 'controller input missing')
        self.assertFalse(self.o.regrasp.inhibited)
        self.assertIsNotNone(self.o.regrasp.input_recovery)

    def test_brief_contact_dip_retains_event_without_motion_or_new_attempt(self):
        self.o.monitor.magnet_strengths = lambda: [[0]*3]*2
        self.assertIsNone(self.target())
        self.assertIsNotNone(self.o.regrasp_contact_pending)
        self.assertFalse(self.o.regrasp.inhibited)
        events = self.o.timing_publisher.publish.call_count
        self.advance(.1)
        self.o.monitor.magnet_strengths = lambda: [[30]*3]*2
        self.assertIsNotNone(self.target())
        self.assertIsNone(self.o.regrasp_contact_pending)
        self.assertEqual(self.o.regrasp.observe.call_count, 1)
        self.assertEqual(self.o.timing_publisher.publish.call_count, events)
        self.assertEqual(self.o.regrasp_step, 1)
        self.o._hold_contact_pose.assert_not_called()

    def test_contact_recheck_does_not_renew_deadline(self):
        self.o.monitor.magnet_strengths = lambda: [[0]*3]*2
        self.assertIsNone(self.target())
        started = self.o.regrasp_contact_pending['started']
        self.advance(.15)
        self.assertIsNone(self.target())
        self.assertEqual(self.o.regrasp_contact_pending['started'], started)
        self.advance(.11)
        self.o.monitor.magnet_strengths = lambda: [[30]*3]*2
        self.assertIsNone(self.target())
        self.assertTrue(self.o.regrasp.inhibited)
        self.assertIsNone(self.o.regrasp_contact_pending)

    def test_contact_pending_does_not_bypass_hard_guards(self):
        for case in ('opening', 'stale', 'rise', 'epoch'):
            with self.subTest(case=case):
                self.setUp()
                self.o.monitor.magnet_strengths = lambda: [[0]*3]*2
                self.assertIsNone(self.target())
                self.advance(.05)
                self.o.monitor.magnet_strengths = lambda: [[30]*3]*2
                if case == 'opening':
                    self.o.latest_ratio = 0
                elif case == 'stale':
                    self.o.monitor.streams[0].last_sample_time = 0
                elif case == 'rise':
                    self.o.regrasp.response_reference = [-501]*6
                else:
                    self.o.monitor.raw_calibration_monotonic = 2
                self.assertIsNone(self.target())
                self.assertIsNone(self.o.regrasp_contact_pending)
                self.assertTrue(self.o.regrasp.inhibited)

    def test_pending_continuation_retains_step_and_episode_budget(self):
        self.o.regrasp_continuation = dict(started=self.now, epoch=1,
            before=tuple(self.actual), step=1)
        self.o.monitor.magnet_strengths = lambda: [[0]*3]*2
        self.assertIsNone(self.target())
        self.advance(.1)
        self.o.monitor.magnet_strengths = lambda: [[30]*3]*2
        self.assertIsNotNone(self.target())
        self.assertEqual(self.o.regrasp_step, 2)
        self.assertEqual(self.o.regrasp_episode_started, 10)
        self.o.regrasp.observe.assert_not_called()

    def test_timeout_is_not_reported_as_baseline_error(self):
        self.o.tracker.received = self.now-1
        self.assertIsNone(self.target())
        self.assertEqual(self.o.last_regrasp_block['reason'], 'controller input timeout')
        self.assertEqual(self.o.last_regrasp_block['controller_age_sec'], 1)

    def test_invalid_baseline_has_distinct_reason(self):
        self.assertIsNone(self.scope['_regrasp_target'](
            self.o, self.now, [[30]*3]*2, [1, 1], False))
        self.assertEqual(self.o.last_regrasp_block['reason'], 'invalid empty-close baseline')
        self.assertTrue(self.o.regrasp.inhibited)

    def test_timeout_discards_pending_work_without_resetting_budgets(self):
        self.o.probe_pending = True
        self.o.regrasp_continuation = {'step': 2}
        self.o.regrasp.response_reference = [25]*6
        origin = self.o.regrasp_origin
        self.o.tracker.received = self.now - 1
        self.assertIsNone(self.target())
        self.assertFalse(self.o.probe_pending)
        self.assertIsNone(self.o.regrasp_continuation)
        self.assertEqual(self.o.regrasp.attempts, 1)
        self.assertEqual(self.o.regrasp.response_reference, [25]*6)
        self.assertEqual(self.o.regrasp_origin, origin)
        self.advance(.01)
        self.assertIsNone(self.target())
        self.o.regrasp.observe.assert_not_called()

    def test_timeout_with_unverified_motion_does_not_auto_recover(self):
        self.acknowledge(self.target())
        self.o.tracker.received = self.now - 1
        self.assertIsNone(self.target())
        self.assertTrue(self.o.regrasp.inhibited)

    def acknowledge(self, target):
        o = self.o
        o.tracker.acknowledge_reinforcement(target)
        o.regrasp_position_trace = dict(before=o.regrasp_before_positions, target=target,
            ack_ns=int(self.now*1e9), attempt=1, loaded=True, trigger='automatic_slip',
            step=o.regrasp_step, started=o.regrasp_episode_started, epoch=1,
            motion_counts=[2 if s else 0 for s in o.regrasp_settled])

    def feedback(self):
        for servo in (1, 2):
            raw = self.read(servo, 'after', .1)
            self.scope['_record_regrasp_position'](self.o, servo, raw, int(self.now*1e9))

    def test_thirteen_count_step_keeps_output_and_grasp_caps(self):
        self.assertEqual(ramp.STEP_RAW, 13)
        self.assertEqual(ramp.MAX_CLOSING_ERROR_RAW, 18)
        self.assertEqual(ramp.EPISODE_TRAVEL_RAW, 25)
        self.assertEqual(ramp.CONFIRM_CLOSING_RAW, 12)
        self.assertEqual(ramp.GRASP_TRAVEL_RAW, 50)
        self.assertEqual(ramp.MAX_STAGES, 3)
        self.assertEqual(ramp.SECOND_STEP_RAW, 4)
        self.assertEqual(ramp.OBSERVE_SEC, .8)
        self.assertEqual(ramp.MAX_PWM_RAW, 150)
        self.assertEqual(ramp.MAX_CURRENT_RAW, 100)

    def test_three_stage_episode_advances_both_jaws_after_partial_response(self):
        self.actual[1] = 2246
        first = self.target()
        self.acknowledge(first)
        self.actual[0] -= 2
        for _ in range(2):
            self.advance(.41)
            self.feedback()
        self.assertTrue(self.o._regrasp_motion_ready(self.now))
        second = self.target()
        self.assertEqual(second[0], first[0] - 4)
        self.assertEqual(second[1], min(first[1] + 4, self.actual[1] + 18))
        self.assertEqual(self.o.regrasp_step, 2)
        self.assertEqual(self.o.regrasp.observe.call_count, 1)
        self.acknowledge(second)
        for _ in range(2):
            self.advance(.41)
            self.feedback()
        self.assertTrue(self.o._regrasp_motion_ready(self.now))
        third = self.target()
        self.assertEqual(third[0], max(second[0] - 4, 1751 - ramp.EPISODE_TRAVEL_RAW,
                                      self.actual[0] - ramp.MAX_CLOSING_ERROR_RAW))
        self.assertEqual(third[1], min(second[1] + 4, self.actual[1] + 18))
        self.assertEqual(self.o.regrasp_step, 3)
        self.assertEqual(self.o.regrasp.observe.call_count, 1)
        self.acknowledge(third)
        for _ in range(2):
            self.advance(.41)
            self.feedback()
        self.assertFalse(self.o._regrasp_motion_ready(self.now))
        self.o._hold_contact_pose.assert_called_once()
        self.assertTrue(self.o.regrasp.inhibited)
        self.assertIsNone(self.target())

    def test_single_step_stops_on_encoder_response(self):
        target = self.target()
        self.assertEqual(target, (1738, 2259))
        self.acknowledge(target)
        self.actual = [1739, 2256]
        self.advance(.01)
        self.feedback()
        self.advance(.01)
        self.feedback()
        self.assertTrue(self.o._regrasp_motion_ready(self.now))
        self.assertIn('displacement observed', self.o.regrasp_motion_result)
        self.assertFalse(self.o.regrasp.inhibited)
        self.assertEqual(self.o.regrasp.observe.call_count, 1)
        self.assertTrue(self.o.tracker.contact)
        self.assertEqual(self.o.tracker.hold_input, .6)

    def test_two_counts_on_both_jaws_is_not_completion(self):
        self.acknowledge(self.target())
        self.actual = [1749, 2246]
        for _ in range(2):
            self.advance(.1)
            self.feedback()
        self.assertFalse(self.o._regrasp_motion_ready(self.now))
        self.assertFalse(self.o.regrasp.motion_confirmed)

    def test_previous_eight_count_threshold_no_longer_finishes_episode(self):
        self.acknowledge(self.target())
        self.actual = [1743, 2252]
        for _ in range(2):
            self.advance(.1)
            self.feedback()
        self.assertFalse(self.o._regrasp_motion_ready(self.now))
        self.assertFalse(self.o.regrasp.motion_confirmed)

    def test_ramp_continuation_rechecks_opening_sensor_and_output(self):
        for case in ('opening', 'sensor_lost', 'contact_lost', 'nan', 'rise', 'overwritten', 'pwm', 'stale', 'expired'):
            with self.subTest(case=case):
                self.setUp()
                self.acknowledge(self.target())
                self.advance(.21)
                self.feedback()
                self.advance(.21)
                self.feedback()
                # Preflight also rejects a stale continuation supplied by an older caller.
                self.o.regrasp_position_trace = None
                self.o.regrasp_continuation = dict(started=self.now, epoch=1,
                    before=tuple(self.actual), step=1, settled=(False, False))
                if case == 'opening':
                    self.o.latest_ratio = 0
                elif case == 'sensor_lost':
                    self.o.monitor.streams[0].last_sample_time = 0
                elif case in ('contact_lost', 'nan'):
                    value = 0 if case == 'contact_lost' else float('nan')
                    self.o.monitor.magnet_strengths = lambda: [[value]*3]*2
                elif case == 'rise':
                    self.o.regrasp.response_reference = [-501]*6
                elif case == 'expired':
                    self.advance(4)
                else:
                    read = self.read
                    def altered(servo, phase, timeout):
                        result = read(servo, phase, timeout)
                        key, value = {'overwritten': ('goal_raw', 2000), 'pwm': ('load_raw', 200),
                                      'stale': ('observed_pc_ns', 0)}[case]
                        self.o.regrasp_motor_records[servo][key] = value
                        return result
                    self.o._read_regrasp_position = altered
                self.assertIsNone(self.target())

    def test_unresponsive_motor_does_not_cause_unbounded_commands(self):
        self.assertEqual(ramp.MAX_STAGES, 3)
        self.acknowledge(self.target())
        self.o.regrasp_position_trace['step'] = ramp.MAX_STAGES
        self.advance(.21)
        self.feedback()
        self.advance(.21)
        self.feedback()
        self.assertFalse(self.o._regrasp_motion_ready(self.now))
        self.assertFalse(self.o.regrasp.inhibited)
        self.o._hold_contact_pose.assert_not_called()
        self.advance(.4)
        self.feedback()
        self.assertFalse(self.o._regrasp_motion_ready(self.now))
        self.assertTrue(self.o.regrasp.inhibited)
        self.assertIn('limit reached', self.o.regrasp_motion_result)
        self.o._hold_contact_pose.assert_called_once()
        self.assertEqual(tuple(self.o.tracker.target), tuple(self.actual))
        self.assertEqual(self.o.tracker.hold_input, .6)
        self.assertIsNone(self.target())

    def test_no_advance_from_only_one_feedback_sample(self):
        self.acknowledge(self.target())
        self.advance(.42)
        self.feedback()
        self.assertFalse(self.o._regrasp_motion_ready(self.now))
        self.assertFalse(hasattr(self.o, 'regrasp_continuation') and self.o.regrasp_continuation)

    def test_partial_response_also_ends_with_measured_hold(self):
        self.acknowledge(self.target())
        self.o.regrasp_position_trace['step'] = ramp.MAX_STAGES
        self.actual[0] -= 2
        for _ in range(2):
            self.advance(.41)
            self.feedback()
        self.assertFalse(self.o._regrasp_motion_ready(self.now))
        self.o._hold_contact_pose.assert_called_once()
        self.assertTrue(self.o.regrasp.inhibited)

    def test_failed_hold_ack_is_not_reported_as_success_or_retried(self):
        self.acknowledge(self.target())
        self.o.regrasp_position_trace['step'] = ramp.MAX_STAGES
        for _ in range(2):
            self.advance(.41)
            self.feedback()
        self.o._hold_contact_pose.side_effect = TimeoutError('ACK missing')
        with self.assertRaises(TimeoutError):
            self.o._regrasp_motion_ready(self.now)
        self.o._hold_contact_pose.assert_called_once()

    def test_encoder_movement_does_not_hide_goal_overwrite(self):
        self.acknowledge(self.target())
        self.actual = [1739, 2256]
        for _ in range(2):
            self.advance(.21)
            self.feedback()
        self.o.regrasp_motor_records[1]['goal_raw'] = 2000
        self.assertFalse(self.o._regrasp_motion_ready(self.now))
        self.assertTrue(self.o.regrasp.inhibited)
        self.assertIn('goal readback differs', self.o.regrasp_motion_result)
        self.o._hold_contact_pose.assert_not_called()

    def test_stale_feedback_waits_for_read_without_new_command(self):
        self.acknowledge(self.target())
        self.o.regrasp_position_trace['step'] = ramp.MAX_STAGES
        self.advance(.21)
        self.feedback()
        self.advance(.4)
        self.assertFalse(self.o._regrasp_motion_ready(self.now))
        self.assertFalse(self.o.regrasp.inhibited)
        self.assertIsNotNone(self.o.regrasp_position_trace)
        self.assertFalse(getattr(self.o, 'regrasp_continuation', None))
        self.feedback()
        self.assertFalse(self.o._regrasp_motion_ready(self.now))
        self.o._hold_contact_pose.assert_not_called()
        self.advance(.21)
        self.feedback()
        self.assertFalse(self.o._regrasp_motion_ready(self.now))
        self.o._hold_contact_pose.assert_called_once()

    def test_output_limit_still_blocks_during_extended_observation(self):
        self.acknowledge(self.target())
        for _ in range(2):
            self.advance(.21)
            self.feedback()
        self.o.regrasp_motor_records[1]['load_raw'] = 200
        self.assertFalse(self.o._regrasp_motion_ready(self.now))
        self.assertTrue(self.o.regrasp.inhibited)
        self.assertIn('PWM', self.o.regrasp_motion_result)
        self.o._hold_contact_pose.assert_not_called()

    def test_delayed_movement_succeeds_without_more_closing_commands(self):
        self.acknowledge(self.target())
        for _ in range(2):
            self.advance(.21)
            self.feedback()
        self.assertFalse(self.o._regrasp_motion_ready(self.now))
        self.actual = [1739, 2256]
        for _ in range(2):
            self.advance(.12)
            self.feedback()
        self.assertTrue(self.o._regrasp_motion_ready(self.now))
        self.o._hold_contact_pose.assert_not_called()
        self.assertFalse(getattr(self.o, 'regrasp_continuation', None))

    def test_grace_observes_late_motion_but_never_advances_goal(self):
        self.acknowledge(self.target())
        self.advance(ramp.DEADLINE_SEC + .1)
        self.feedback()
        self.feedback()
        self.assertFalse(self.o._regrasp_motion_ready(self.now))
        self.assertFalse(getattr(self.o, 'regrasp_continuation', None))
        self.actual = [1739, 2256]
        self.advance(.1)
        self.feedback()
        self.advance(.1)
        self.feedback()
        self.assertTrue(self.o._regrasp_motion_ready(self.now))
        self.assertIn('displacement observed', self.o.regrasp_motion_result)

    def test_missing_feedback_still_expires(self):
        self.acknowledge(self.target())
        self.advance(ramp.DEADLINE_SEC + ramp.OBSERVATION_GRACE_SEC + .1)
        self.assertFalse(self.o._regrasp_motion_ready(self.now))
        self.assertTrue(self.o.regrasp.inhibited)


if __name__ == '__main__':
    unittest.main()
