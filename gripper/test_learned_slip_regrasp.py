import unittest
import math
from learned_slip_regrasp import LearnedRegraspGuard
from pretrained_anyskin_slip import PollenSlipModel


class LearnedTests(unittest.TestCase):
    def test_learned_rise_limit_500_without_rebasing(self):
        from gripper_regrasp import RegraspGuard
        g = self.make()
        g.note_move([40]*6)
        self.assertEqual(g.RESPONSE_RISE_LIMIT, 500)
        self.assertEqual(RegraspGuard.RESPONSE_RISE_LIMIT, 60)
        self.assertFalse(g.response_exceeded([101]*6))
        self.assertFalse(g.response_exceeded([160]*6))
        self.assertFalse(g.response_exceeded([222.9]*6))
        self.assertFalse(g.response_exceeded([340]*6))
        self.assertFalse(g.response_exceeded([440]*6))
        self.assertFalse(g.response_exceeded([540]*6))
        for channel in range(6):
            values = [40]*6
            values[channel] = 540.01
            self.assertTrue(g.response_exceeded(values))
        g.note_move([150]*6)
        self.assertEqual(g.response_reference, [40]*6)
        self.assertTrue(g.response_exceeded([541]*6))

    def test_persistent_high_never_authorizes_repeat_even_after_verified_motion(self):
        g = self.make()
        for t in (.05, .1, .15):
            self.score(g, t)
        g.note_move([40]*3 + [0]*3)
        for t in (1, 1.1, 1.2):
            self.assertFalse(self.score(g, t))
        self.assertEqual(g.attempts, 1)
        g.confirm_motion(1.2)
        self.assertFalse(self.score(g, 1.5))
        self.assertFalse(self.score(g, 1.6))
        self.assertFalse(self.score(g, 1.61, stamps=(1.6, 1.6)))
        self.assertFalse(self.score(g, 1.65))
        self.assertFalse(self.score(g, 1.7))
        self.assertEqual(g.attempts, 1)
        self.assertIn('repeat withheld', g.state)
        self.assertTrue(g.motion_confirmed)
        for t in (2.5, 2.6, 2.7):
            self.assertFalse(self.score(g, t))

    def test_persistent_slip_stops_on_clear_and_retains_budget(self):
        g = self.make()
        for t in (.05, .1, .15):
            self.score(g, t)
        g.confirm_motion(.5)
        for t in (1, 1.05, 1.1):
            self.assertFalse(self.score(g, t, (.1, .1)))
        self.assertIsNone(g.rearm_sensor)
        self.assertEqual(g.attempts, 1)
        self.assertFalse(self.score(g, 1.15))
        self.assertFalse(self.score(g, 1.2))
        self.assertTrue(self.score(g, 1.25))
        self.assertEqual(g.trigger_reason, 'new_slip')

    def test_persistent_slip_never_exceeds_six_attempts_or_rise_cap(self):
        g = self.make()
        for t in (.05, .1, .15):
            self.score(g, t)
        g.note_move([40]*3 + [0]*3)
        original = list(g.response_reference)
        g.confirm_motion(.5)
        for t in (1, 1.05, 1.1):
            self.assertFalse(self.score(g, t, rows=[[541]*3, [0]*3]))
        self.assertEqual(g.attempts, 1)
        for cycle in range(5):
            t = 2 + cycle * 2
            for clear in (t, t+.05, t+.1):
                self.assertFalse(self.score(g, clear, (.1, .1)))
            self.assertFalse(self.score(g, t+.2))
            self.assertFalse(self.score(g, t+.25))
            self.assertTrue(self.score(g, t+.3))
            g.confirm_motion(t+.4)
        self.assertFalse(self.score(g, 13))
        self.assertEqual(g.attempts, 6)
        self.assertTrue(g.inhibited)
        self.assertEqual(g.response_reference, original)

    def test_faster_poll_keeps_three_distinct_samples(self):
        from learned_slip_regrasp import SlipScorer
        self.assertEqual(SlipScorer.POLL_SEC, .025)
        g = self.make()
        self.assertFalse(self.score(g, .05))
        self.assertFalse(self.score(g, .075))
        self.assertTrue(self.score(g, .1))

    def test_completed_motion_rise_pause_requires_original_cap_clear_and_new_slip(self):
        g = self.make()
        for t in (.1, .2):
            self.assertFalse(self.score(g, t))
        self.assertTrue(self.score(g, .3))
        g.note_move([40]*3 + [0]*3)
        g.confirm_motion()
        original = list(g.response_reference)
        for t in (1.1, 1.2, 1.3):
            self.assertFalse(self.score(g, t, (.1, .1), rows=[[541]*3, [0]*3]))
        self.assertFalse(g.inhibited)
        self.assertEqual(g.clear_count, 0)
        for t in (1.4, 1.5, 1.6):
            self.assertFalse(self.score(g, t, (.1, .1)))
        for t in (1.7, 1.8):
            self.assertFalse(self.score(g, t))
        self.assertTrue(self.score(g, 1.9))
        self.assertEqual(g.attempts, 2)
        self.assertEqual(g.response_reference, original)
        g.note_move([40]*3 + [0]*3)
        self.assertFalse(g.motion_confirmed)

    def test_unverified_motion_rise_and_other_faults_stay_inhibited(self):
        g = self.make()
        g.note_move([40]*3 + [0]*3)
        self.assertFalse(self.score(g, 1, rows=[[541]*3, [0]*3]))
        self.assertTrue(g.inhibited)
        g.confirm_motion()
        self.assertFalse(self.score(g, 2, (.1, .1)))
        self.assertTrue(g.inhibited)

    def test_input_recovery_requires_distinct_input_then_low_then_new_slip(self):
        g = self.make()
        g.attempts = 1
        g.response_reference = [40]*3 + [0]*3
        self.score(g, .05)
        self.score(g, .1)
        g.pause_input('controller input timeout')
        for _ in range(5):
            self.assertFalse(g.recover_input(1, 1.1))
        self.assertFalse(g.recover_input(1.1, 1.1))
        self.assertTrue(g.recover_input(1.21, 1.21))
        self.assertFalse(self.score(g, 1.22, (.1, .1), stamps=(1.20, 1.20)))
        self.assertEqual(g.clear_count, 0)
        for t in (1.22, 1.27, 1.32):
            self.assertFalse(self.score(g, t))
        self.assertEqual(g.attempts, 1)
        for t in (1.4, 1.45, 1.5):
            self.assertFalse(self.score(g, t, (.1, .1)))
        self.assertFalse(self.score(g, 1.55))
        self.assertFalse(self.score(g, 1.6))
        self.assertTrue(self.score(g, 1.65))
        self.assertEqual(g.attempts, 2)
        self.assertEqual(g.response_reference, [40]*3 + [0]*3)

    def test_input_recovery_never_clears_hard_inhibit(self):
        g = self.make()
        g.inhibit('motor goal mismatch')
        g.pause_input('timeout')
        self.assertTrue(g.inhibited)
        self.assertEqual(g.state, 'motor goal mismatch')
        self.assertFalse(self.score(g, 1))

    def test_another_timeout_restarts_recovery(self):
        g = self.make()
        g.pause_input('timeout')
        self.assertFalse(g.recover_input(1, 1))
        self.assertFalse(g.recover_input(1.1, 1.1))
        g.pause_input('timeout')
        self.assertFalse(g.recover_input(2, 2))

    def test_real_model_load_and_finite_score(self):
        score = PollenSlipModel().score([0.0] * 15)
        self.assertTrue(math.isfinite(score))
        self.assertTrue(0 <= score <= 1)

    def make(self):
        guard = LearnedRegraspGuard()
        guard.expected_epoch = 1
        guard.observe(-1, [[40]*3, [0]*3], [-1, -1], 20)
        for t in (-.3, -.2, -.1):
            self.score(guard, t, (.1, .1))
        return guard

    def test_initial_high_triggers_only_after_settle_and_three_new_samples(self):
        g = LearnedRegraspGuard()
        g.expected_epoch = 1
        for t in (0, .1, .2, .3):
            self.assertFalse(self.score(g, t))
            self.assertEqual(g.high_counts, [0, 0])
        self.assertEqual(g.attempts, 0)
        self.assertFalse(self.score(g, .31))
        self.assertEqual(g.high_counts, [1, 0])
        self.assertFalse(self.score(g, .36))
        self.assertTrue(self.score(g, .41))
        for t in (.5, .6, 1, 2):
            self.assertFalse(self.score(g, t))
        self.assertEqual(g.attempts, 1)
        g.reset()
        self.assertFalse(g.initial_clear)

    def test_pre_settle_and_repeated_samples_cannot_confirm_initial_slip(self):
        g = LearnedRegraspGuard()
        g.expected_epoch = 1
        self.score(g, 0)
        self.assertFalse(self.score(g, .4, stamps=(.25, .25)))
        self.assertFalse(g.initial_clear)
        self.assertFalse(self.score(g, .41))
        self.assertFalse(self.score(g, .45, stamps=(.41, .41)))
        self.assertEqual(g.high_counts, [1, 0])
        self.assertFalse(self.score(g, .46))
        self.assertTrue(self.score(g, .51))

    def score(self, g, t, scores=(.95, .1), stamps=None, rows=None):
        g.prediction = (g.expected_epoch, stamps or (t, t), scores)
        return g.observe(t, rows or [[40]*3, [0]*3], [t, t], 20)

    def test_three_fresh_samples_and_no_repeat_while_high(self):
        g = self.make()
        self.assertFalse(self.score(g, .05))
        self.assertFalse(self.score(g, .1, stamps=(.05, .05)))
        self.assertFalse(self.score(g, .15))
        self.assertTrue(self.score(g, .2))
        for i in range(10, 30):
            self.assertFalse(self.score(g, i * .1))
        self.assertEqual(g.attempts, 1)

    def test_strict_threshold_still_requires_three_fresh_samples(self):
        g = self.make()
        self.assertFalse(self.score(g, .05, (.90, .1)))
        self.assertFalse(self.score(g, .1, (.90, .1), stamps=(.05, .05)))
        self.assertFalse(self.score(g, .15, (.90, .1)))
        self.assertTrue(self.score(g, .2, (.90, .1)))
        self.assertEqual(g.attempts, 1)
        self.assertFalse(self.score(g, 1, (.85, .1)))

    def test_below_sensitive_threshold_breaks_consecutive_count(self):
        g = self.make()
        for t, score in ((.05, .95), (.1, .89), (.15, .95), (.2, .95)):
            self.assertFalse(self.score(g, t, (score, .1)))
        self.assertTrue(self.score(g, .25, (.95, .1)))

    def test_moderate_scores_never_trigger_or_repeat(self):
        g = self.make()
        for t in (.05, .1, .15, .2, .25):
            self.assertFalse(self.score(g, t, (.89, .89)))
        self.assertEqual(g.attempts, 0)
        for t in (.3, .35, .4):
            self.score(g, t)
        g.confirm_motion(.5)
        for t in (1.2, 1.25, 1.3, 1.35):
            self.assertFalse(self.score(g, t, (.89, .89)))
        self.assertEqual(g.attempts, 1)

    def test_low_rearms_and_six_attempt_cap(self):
        g = self.make()
        for cycle in range(6):
            t = cycle * 2
            self.assertFalse(self.score(g, t + .05))
            self.assertFalse(self.score(g, t + .1))
            self.assertTrue(self.score(g, t + .15))
            for clear_t in (t + 1, t + 1.05, t + 1.1):
                self.assertFalse(self.score(g, clear_t, (.1, .1)))
        self.assertFalse(self.score(g, 13))
        self.assertEqual(g.attempts, 6)

    def test_inactive_sensor_cannot_trigger(self):
        g = self.make()
        for i in range(1, 10):
            self.assertFalse(self.score(g, i * .05, (.1, .99)))

    def test_single_clear_glitch_does_not_rearm(self):
        g = self.make()
        for t in (.05, .1, .15):
            self.score(g, t)
        self.score(g, 1, (.1, .1))
        for t in (1.05, 1.1, 1.15, 1.2):
            self.assertFalse(self.score(g, t))
        self.assertEqual(g.attempts, 1)

    def test_other_contacting_sensor_must_also_clear(self):
        g = self.make()
        for t in (.05, .1, .15):
            self.score(g, t)
        for t in (1, 1.05, 1.1):
            self.assertFalse(self.score(g, t, (.1, .99), rows=[[40]*3]*2))
        self.assertIsNotNone(g.rearm_sensor)

    def test_stale_and_invalid_predictions(self):
        for scores, stamps in [((float('nan'), .1), (.5, .5)), ((.99, .1), (.01, .01))]:
            g = self.make()
            self.assertFalse(self.score(g, .5, scores, stamps))
            self.assertEqual(g.high_counts, [0, 0])

    def test_epoch_change_blocks(self):
        g = self.make()
        self.score(g, .05)
        g.expected_epoch = 2
        self.assertFalse(self.score(g, .1))
        self.assertTrue(g.inhibited)

    def test_contact_loss_and_rise_cap(self):
        g = self.make()
        self.assertFalse(self.score(g, .05, rows=[[0]*3]*2))
        self.assertFalse(g.inhibited)
        self.assertFalse(self.score(g, .3, rows=[[0]*3]*2))
        self.assertTrue(g.inhibited)
        g = self.make()
        g.note_move([40]*3 + [0]*3)
        self.assertFalse(self.score(g, .05, rows=[[541]*3, [0]*3]))
        self.assertTrue(g.inhibited)

    def test_transient_contact_dip_requires_new_confirmation(self):
        g = self.make()
        self.score(g, .05)
        self.score(g, .1)
        self.assertFalse(self.score(g, .15, rows=[[0]*3]*2))
        self.assertFalse(self.score(g, .2))
        self.assertFalse(self.score(g, .25))
        self.assertTrue(self.score(g, .3))

    def test_reset_clears_model_episode(self):
        g = self.make()
        for t in (.05, .1, .15):
            self.score(g, t)
        g.reset()
        self.assertEqual(g.attempts, 0)
        self.assertIsNone(g.rearm_sensor)

    def test_contact_jitter_does_not_erase_confirmed_contact(self):
        g = self.make()
        self.assertFalse(self.score(g, .05, rows=[[21]*3, [0]*3]))
        self.assertFalse(self.score(g, .1, rows=[[18]*3, [0]*3]))
        self.assertTrue(self.score(g, .15, rows=[[19]*3, [0]*3]))

    def test_low_signal_cannot_establish_contact(self):
        g = self.make()
        g.contact_present = [False, False]
        for t in (.05, .1, .15):
            self.assertFalse(self.score(g, t, rows=[[18]*3, [0]*3]))

    def test_confirmed_event_does_not_need_fourth_high_sample(self):
        g = self.make()
        for t in (.05, .1, .15):
            self.score(g, t)
        self.assertIsNone(g.event_preflight_error((1, (.18, .18), (.4, .1)), .2, 1))
        self.assertIn('older', g.event_preflight_error((1, (.5, .5), (.99, .1)), .5, 1))
        self.assertIsNotNone(g.event_preflight_error((2, (.18, .18), (.99, .1)), .2, 2))
        self.assertIsNotNone(g.event_preflight_error(None, .2, 1))

    def test_cancelled_event_waits_for_clear_before_next_event(self):
        g = self.make()
        for t in (.05, .1, .15):
            self.score(g, t)
        self.assertIsNotNone(g.event_preflight_error(None, .2, 1))
        for t in (.3, .4, 1):
            self.assertFalse(self.score(g, t))
        for t in (1.1, 1.15, 1.2):
            self.assertFalse(self.score(g, t, (.1, .1)))
        self.assertFalse(self.score(g, 1.25))
        self.assertFalse(self.score(g, 1.3))
        self.assertTrue(self.score(g, 1.35))


if __name__ == '__main__':
    unittest.main()
