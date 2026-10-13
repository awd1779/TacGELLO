"""Conservative base-Z apparent-load estimate; not a scale or a safety sensor."""

from collections import deque
import math
import statistics


class LiftWeight:
    MAX_KG = 2.4  # Cue saturation, NOT permission to lift this payload.

    def __init__(self):
        self.zero_n = None
        self.taring = False
        self.tare_started = None
        self.tare_error = None
        self.tare_wait_reason = None
        self.motion_diagnostics = {}
        self.samples = deque()
        self.last_stamp = None
        self.last_fresh = None
        self.last_velocity = None
        self.kg = None
        self.state = 'tare required: empty tool, upright robot'
        self.effort = 0.0

    def request_tare(self, now):
        self.zero_n = None
        self.taring = True
        self.tare_started = now
        self.tare_error = None
        self.tare_wait_reason = None
        self.invalidate('collecting empty-tool zero')

    def fail_tare(self, reason):
        self.taring = False
        self.tare_error = reason
        self.invalidate(reason)

    def check_tare_timeout(self, now):
        if self.taring and now - self.tare_started > 10.0:
            self.fail_tare('tare timed out: ' + (self.tare_wait_reason or 'no valid samples'))

    def invalidate(self, reason):
        self.samples.clear()
        self.kg = None
        self.effort = 0.0
        self.last_velocity = None
        if self.taring:
            self.tare_wait_reason = reason
        self.state = self.tare_error if self.zero_n is None and self.tare_error else reason

    def update(self, force, speed, stamp, now, contact, contact_fresh):
        self.check_tare_timeout(now)
        if self.tare_error and self.zero_n is None:
            return 0.0
        if (len(force) != 6 or len(speed) != 6
                or not all(math.isfinite(v) for v in [*force, *speed, stamp, now])):
            self.invalidate('invalid RTDE sample')
            return 0.0
        if self.last_stamp is not None and stamp <= self.last_stamp:
            if stamp < self.last_stamp:
                self.zero_n = None
                self.fail_tare('robot clock reset; tare required')
                self.last_stamp = stamp
            elif self.last_fresh is None or now - self.last_fresh > 0.15:
                self.invalidate('stale RTDE sample')
            return self.effort
        dt = None if self.last_fresh is None else now - self.last_fresh
        self.last_stamp, self.last_fresh = stamp, now
        previous_velocity = self.last_velocity
        self.last_velocity = list(speed[:3])
        if not contact_fresh:
            self.invalidate('contact heartbeat unavailable')
            return 0.0
        if self.taring and contact:
            self.fail_tare('tare cancelled: contact detected; tool must be empty')
            return 0.0
        linear = math.sqrt(sum(v*v for v in speed[:3]))
        angular = math.sqrt(sum(v*v for v in speed[3:]))
        acceleration = (0.0 if previous_velocity is None or dt is None or dt <= 0
                        else math.sqrt(sum((a-b)**2 for a, b in
                                           zip(speed[:3], previous_velocity))) / dt)
        self.motion_diagnostics = dict(linear_speed_m_s=linear, angular_speed_rad_s=angular,
                                       acceleration_m_s2=acceleration, sample_gap_s=dt)
        if ((dt is not None and not 0 < dt <= 0.15) or angular > 0.05
                or linear > (0.003 if self.taring else 0.06)
                or acceleration > 0.15):
            self.invalidate('motion unsettled; no weight cue')
            return 0.0
        if not self.taring and (self.zero_n is None or not contact):
            self.invalidate('tare required' if self.zero_n is None else 'no contact')
            return 0.0
        # Upright base frame only: downward external force is negative Z.
        self.samples.append((now, -float(force[2])))
        # Keep all 25 fresh samples at slower controller polling rates. Stability,
        # motion and per-sample freshness limits remain unchanged.
        window = 4.0 if self.taring else 1.2
        while self.samples and now - self.samples[0][0] > window:
            self.samples.popleft()
        values = [sample[1] for sample in self.samples]
        if len(values) < 25 or now - self.samples[0][0] < 1.0:
            self.kg, self.effort = None, 0.0
            self.state = 'collecting empty-tool zero' if self.taring else 'load settling'
            if self.taring:
                self.tare_wait_reason = f'collecting samples: {len(values)}/25; span {now-self.samples[0][0]:.2f}/1.00s'
            return 0.0
        if max(values) - min(values) > 2.0:
            self.kg, self.effort = None, 0.0
            self.state = 'force unstable; no weight cue'
            if self.taring:
                self.tare_wait_reason = f'force spread {max(values)-min(values):.3f}N exceeds 2.000N'
            return 0.0
        measured = statistics.median(values)
        if self.taring:
            self.zero_n = measured
            self.taring = False
            self.tare_wait_reason = None
            self.invalidate('zero ready; lift one bottle and verify estimate')
            return 0.0
        self.kg = (measured - self.zero_n) / 9.80665
        if self.kg < -0.2:
            self.effort = 0.0
            self.state = 'negative load: check mounting, tare and payload compensation'
            return 0.0
        self.kg = max(0.0, self.kg)
        # Release on descent/rest; no constant position hold or return-to-home.
        target = min(1.0, self.kg / self.MAX_KG) if speed[2] > 0.003 and self.kg >= 0.2 else 0.0
        self.effort = min(target, self.effort + min(dt or 0.0, 0.05))
        self.state = 'lifting cue' if target else 'estimated load; cue off at rest/descent'
        return self.effort

    def status(self):
        return dict(state=self.state, estimated_kg=self.kg, zero_force_n=self.zero_n,
                    tare_in_progress=self.taring, tare_error=self.tare_error,
                    tare_wait_reason=self.tare_wait_reason, sample_count=len(self.samples),
                    sample_span_s=self.samples[-1][0]-self.samples[0][0] if self.samples else 0.0,
                    motion_diagnostics=self.motion_diagnostics,
                    cue_fraction=self.effort, cue_max_kg=self.MAX_KG,
                    sample_received_monotonic_s=self.last_fresh, robot_stamp_s=self.last_stamp,
                    estimate_only=True, axis='base -Z force / +Z lift')
