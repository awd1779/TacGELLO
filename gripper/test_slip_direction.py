import unittest

from slip_direction import SlipDirection


class DirectionTests(unittest.TestCase):
    def snapshot(self, stamp, xyz=None, epoch=1):
        return dict(calibration_epoch=epoch, sensors=[dict(
            received_pc_monotonic_s=stamp, xyz=list(xyz or [0]*15)) for _ in range(2)])

    def test_axis_vectors_and_sensor_separation(self):
        estimator = SlipDirection()
        estimator.update(self.snapshot(1), 1, True)
        snapshot = self.snapshot(1.2, [0, -5, 1]*5)
        snapshot['sensors'][1]['xyz'] = [8, 0, 0]*5
        results = estimator.update(snapshot, 1.2, True)['sensors']
        self.assertEqual(results[0]['magnetic_axis'], '-Y')
        self.assertEqual(results[1]['magnetic_axis'], '+X')
        self.assertEqual(results[0]['mean_delta_xyz'], [0, -5, 1])
        self.assertEqual(len(results[0]['delta_xyz_by_magnet']), 5)
        self.assertEqual(results[0]['physical_direction'], 'uncalibrated')
        self.assertAlmostEqual(results[0]['window_sec'], .2)

    def test_opposing_magnets_are_mixed_not_a_physical_direction(self):
        estimator = SlipDirection()
        estimator.update(self.snapshot(1), 1, True)
        result = estimator.update(self.snapshot(1.2, [5, 0, 0, -5, 0, 0] * 2 + [0]*3),
                                  1.2, True)['sensors'][0]
        self.assertEqual(result['magnetic_axis'], 'mixed')
        self.assertEqual(result['coherence'], 0)

    def test_epoch_release_and_missing_data_reset_window(self):
        for case in ('epoch', 'release', 'stale', 'nan', 'missing', 'gap'):
            with self.subTest(case=case):
                estimator = SlipDirection()
                estimator.update(self.snapshot(1), 1, True)
                snapshot = self.snapshot(1.1)
                if case == 'epoch':
                    snapshot['calibration_epoch'] = 2
                if case == 'nan':
                    snapshot['sensors'][0]['xyz'][0] = float('nan')
                if case == 'missing':
                    snapshot['sensors'] = []
                if case == 'gap':
                    snapshot = self.snapshot(1.5)
                estimator.update(snapshot, 1.5 if case in ('stale', 'gap') else 1.1,
                                 case != 'release')
                result = estimator.update(self.snapshot(1.6, [1]*15,
                    epoch=2 if case == 'epoch' else 1), 1.6, True)['sensors'][0]
                if case == 'gap':
                    self.assertEqual(result['start_pc_s'], 1.5)
                else:
                    self.assertEqual(result['status'], 'warming up')

    def test_duplicate_samples_do_not_fill_window(self):
        estimator = SlipDirection()
        for _ in range(50):
            result = estimator.update(self.snapshot(1), 1.1, True)
        self.assertEqual(result['sensors'][0]['status'], 'warming up')
        self.assertEqual(len(estimator.history[0]), 1)

    def test_no_change_is_not_a_direction(self):
        estimator = SlipDirection()
        estimator.update(self.snapshot(1), 1, True)
        result = estimator.update(self.snapshot(1.2), 1.2, True)
        self.assertEqual(result['sensors'][0]['magnetic_axis'], 'no change')


if __name__ == '__main__':
    unittest.main()
