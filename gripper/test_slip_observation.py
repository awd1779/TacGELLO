import unittest
from slip_observation import SlipObservation


class ObservationTests(unittest.TestCase):
    def test_intermediate_scores_are_latched_not_current_high_evidence(self):
        observer = SlipObservation()
        def sample(t, score):
            return observer.update(t, (1, (t, t), (score, score)), 1, True, [[30]*3]*2, 20)
        sample(0, .95)
        for t in (.31, .36, .41):
            value = sample(t, .95)
        self.assertTrue(value['high_evidence'])
        value = sample(.46, .6)
        self.assertTrue(value['active'])
        self.assertFalse(value['high_evidence'])
        self.assertFalse(value['physical_slip_verified'])
        self.assertIn('LATCHED', value['state'])
        value = sample(.51, .95)
        self.assertTrue(value['high_evidence'])
        self.assertEqual(value['total_count'], 1)
        self.assertIn('UNVERIFIED', value['state'])

    def test_monitor_uses_same_stricter_threshold_as_motor_guard(self):
        from learned_slip_regrasp import LearnedRegraspGuard
        observer = SlipObservation()
        self.assertEqual(observer.SLIP_THRESHOLD, LearnedRegraspGuard.SLIP_THRESHOLD)
        self.assertEqual(observer.SLIP_THRESHOLD, .90)
        for t in (0, .31, .36, .41, .46):
            value = observer.update(t, (1, (t, t), (.89, .89)), 1, True, [[30]*3]*2, 20)
            self.assertFalse(value['active'])
        self.assertEqual(observer.total, 0)

    def test_two_episodes_without_motor_guard(self):
        observer = SlipObservation()
        def sample(t, score, contact=True):
            return observer.update(t, (1, (t, t), (score, score)), 1,
                                   contact, [[30]*3]*2, 20)
        sample(0, .9)
        for t in (.31, .36, .41):
            value = sample(t, .9)
        self.assertTrue(value['active'])
        self.assertEqual(value['total_count'], 1)
        for t in (.46, .51, .56):
            self.assertEqual(sample(t, .99)['total_count'], 1)
        for t in (.61, .66, .71):
            value = sample(t, .01)
        self.assertFalse(value['active'])
        self.assertEqual(value['transitions'], ['ended'])
        for t in (.76, .81, .86):
            value = sample(t, .95)
        self.assertEqual(value['total_count'], 2)
        self.assertEqual(value['count'], 2)
        self.assertEqual(sample(.9, .01, False)['count'], 0)
        self.assertEqual(observer.total, 2)

    def test_duplicate_and_stale_do_not_confirm(self):
        observer = SlipObservation()
        observer.update(0, (1, (0, 0), (.9, .9)), 1, True, [[30]*3]*2, 20)
        for now in (.31, .32, .33):
            observer.update(now, (1, (.31, .31), (.9, .9)), 1, True, [[30]*3]*2, 20)
        self.assertEqual(observer.total, 0)
        value = observer.update(1, (1, (.31, .31), (.9, .9)), 1, True, [[30]*3]*2, 20)
        self.assertEqual(value['state'], 'MODEL UNAVAILABLE')
        self.assertEqual(value['high_counts'], [0, 0])


if __name__ == '__main__':
    unittest.main()
