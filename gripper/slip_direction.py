"""Magnetic change telemetry, NOT a calibrated object-motion direction estimator."""
from collections import deque
import math


class SlipDirection:
    def __init__(self):
        self.boundary = None
        self.history = [deque(maxlen=64), deque(maxlen=64)]

    def update(self, snapshot, now, contact):
        boundary = (snapshot.get('calibration_epoch'), bool(contact))
        if boundary != self.boundary:
            for history in self.history:
                history.clear()
            self.boundary = boundary
        sensors = snapshot.get('sensors', [])
        results = []
        for index, history in enumerate(self.history):
            result = dict(sensor=index + 1, status='unavailable',
                          magnetic_axis=None, physical_direction='uncalibrated')
            results.append(result)
            sensor = sensors[index] if index < len(sensors) else {}
            stamp, xyz = sensor.get('received_pc_monotonic_s'), sensor.get('xyz')
            valid = (isinstance(stamp, (int, float)) and math.isfinite(stamp)
                     and 0 <= now - stamp <= .25 and isinstance(xyz, (list, tuple))
                     and len(xyz) == 15 and all(isinstance(v, (int, float))
                     and math.isfinite(v) for v in xyz))
            if not valid or not contact:
                history.clear()
                continue
            if history and (stamp < history[-1][0] or stamp - history[-1][0] > .25):
                history.clear()
            if not history or stamp > history[-1][0]:
                history.append((stamp, list(xyz)))
            candidates = [(t, values) for t, values in history if .1 <= stamp - t <= .3]
            if not candidates:
                result['status'] = 'warming up'
                continue
            before, values = min(candidates, key=lambda entry: abs(stamp - entry[0] - .2))
            deltas = [[xyz[3*m+a] - values[3*m+a] for a in range(3)] for m in range(5)]
            mean = [sum(row[a] for row in deltas) / 5 for a in range(3)]
            magnitude = math.sqrt(sum(v*v for v in mean))
            average_magnitude = sum(math.sqrt(sum(v*v for v in row)) for row in deltas) / 5
            coherence = magnitude / average_magnitude if average_magnitude else 0.0
            axis = max(range(3), key=lambda a: abs(mean[a]))
            label = ('no change' if average_magnitude <= 1e-9 else 'mixed'
                     if coherence < .5 else ('+' if mean[axis] >= 0 else '-') + 'XYZ'[axis])
            result.update(status='magnetic proxy only', magnetic_axis=label,
                          start_pc_s=before, end_pc_s=stamp, window_sec=stamp-before,
                          delta_xyz_by_magnet=deltas, mean_delta_xyz=mean,
                          coherence=coherence)
        return dict(schema='magnetic_direction_proxy_v1', sensors=results)
