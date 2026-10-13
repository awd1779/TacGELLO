"""Bounded experimental current cue, not external torque or payload estimation."""
import math
import statistics


CONTRAST_OUTPUT_GAIN = .6


class JointCurrentCue:
    def __init__(self, servo_id=2, auto_tare=False, output_limit=10, current_range_a=None, contrast=False,
                 pregrasp=False):
        if servo_id not in (2, 3, 4):
            raise ValueError('Only body IDs 2, 3, 4 supported')
        self.index = servo_id - 1
        self.contrast = contrast
        self.pregrasp = pregrasp
        # Midpoints approximate the first 600g trial's current changes. Upper
        # points are experimental headroom, NOT a fitted 1.2kg calibration.
        self.contrast_low, self.contrast_mid, self.contrast_high = {
            2: (.10, .17, .34), 3: (.18, .30, .60), 4: (.08, .15, .30)}[servo_id]
        self.baseline_pose = None
        self.load_samples = []
        self.load_target = 0.0
        self.load_received = None
        self.pregrasp_samples = []
        self.pregrasp_candidate = None
        self.was_contact = False
        self.grasp_baseline_valid = False
        self.grasp_baseline_time = None
        self.accepted_increase = None
        self.load_wait_reason = 'not sampled'
        self.load_window_span = 0.0
        self.load_window_spread = None
        self.load_max_speed = None
        if isinstance(output_limit, bool) or not isinstance(output_limit, int) or not 1 <= output_limit <= 40:
            raise ValueError('Current cue output limit must be an integer 1..40')
        self.output_limit = output_limit
        self.current_range_a = .035 * output_limit if current_range_a is None else current_range_a
        if not math.isfinite(self.current_range_a) or self.current_range_a <= 0:
            raise ValueError('Current range must be positive and finite')
        self.baseline = None
        self.current = None
        self.delta = None
        self.effort = 0.0
        self.filtered = 0.0
        self.stamp = self.received = None
        self.taring = False
        self.samples = []
        self.state = 'current baseline required'
        self.tare_start = None
        self.auto_tare_pending = bool(auto_tare)
        self.current_latched = False
        self.low_current_since = None

    def request_tare(self, now):
        self.baseline_pose = None
        self.auto_tare_pending = False
        self.baseline = None
        self.invalidate('collecting empty stationary current')
        self.taring = True
        self.tare_start = now

    def invalidate(self, reason):
        self.load_wait_reason = reason
        self.load_window_span = 0.0
        self.load_window_spread = self.load_max_speed = None
        self.pregrasp_samples.clear()
        self.pregrasp_candidate = None
        self.grasp_baseline_valid = False
        self.grasp_baseline_time = None
        self.accepted_increase = None
        self.load_samples.clear()
        self.load_target = 0.0
        self.load_received = None
        self.effort = self.filtered = 0.0
        self.delta = None
        self.samples.clear()
        self.state = reason
        self.current_latched = False
        self.low_current_since = None

    def check_tare_timeout(self, now):
        if self.taring and now - self.tare_start >= 10:
            self.taring = False
            self.invalidate('current baseline timed out; request again')

    def update(self, current, velocity, stamp, now, contact, fresh, pose=None):
        self.check_tare_timeout(now)
        if (len(current) != 6 or len(velocity) != 6 or
                not all(math.isfinite(v) for v in [*current, *velocity, stamp, now])):
            self.invalidate('invalid current snapshot')
            return 0.0
        if self.stamp is not None and stamp <= self.stamp:
            if stamp < self.stamp:
                self.baseline = None
                self.taring = False
            self.invalidate('stale or reset robot sample')
            return 0.0
        dt = .008 if self.received is None else now - self.received
        gap = self.stamp is not None and stamp - self.stamp > .15
        self.stamp, self.received = stamp, now
        self.current = list(current)
        if gap or not 0 < dt <= .15 or not fresh:
            self.invalidate('sample gap or contact heartbeat unavailable')
            return 0.0
        if self.contrast and (pose is None or len(pose) != 6 or
                              not all(math.isfinite(v) for v in pose)):
            self.invalidate('joint pose unavailable; cue off')
            return 0.0
        if self.auto_tare_pending:
            if contact or max(abs(v) for v in velocity) > .03:
                self.invalidate('startup baseline waiting for empty stationary tool')
                return 0.0
            self.request_tare(now)
        if self.taring:
            if contact or max(abs(v) for v in velocity) > .03:
                self.invalidate('tare waiting for empty stationary tool')
                return 0.0
            self.samples.append((now, list(current)))
            self.samples = [s for s in self.samples if now - s[0] <= 1.2]
            values = [s[1][self.index] for s in self.samples]
            if (len(values) >= 25 and now - self.samples[0][0] >= 1.0 and
                    max(values) - min(values) <= .15):
                self.baseline = [statistics.median(s[1][i] for s in self.samples) for i in range(6)]
                self.baseline_pose = list(pose) if pose is not None else None
                self.taring = False
                self.state = 'current baseline ready; no contact'
            return 0.0
        if self.contrast and self.pregrasp and not self._prepare_grasp_baseline(current, velocity, pose, now, contact):
            return 0.0
        if self.baseline is None:
            self.invalidate('current baseline required')
            return 0.0
        self.delta = [a - b for a, b in zip(current, self.baseline)]
        if not contact:
            self.accepted_increase = None
            self.load_wait_reason = 'no contact'
            self.load_window_span = 0.0
            self.load_window_spread = self.load_max_speed = None
            self.load_samples.clear()
            self.load_target = 0.0
            self.load_received = None
            self.effort = self.filtered = 0.0
            self.current_latched = False
            self.low_current_since = None
            self.state = 'no contact; current cue off'
            return 0.0
        if self.contrast:
            return self._contrast_update(self.delta[self.index], velocity, pose, now, dt)
        increase = max(0.0, abs(current[self.index]) - abs(self.baseline[self.index]))
        self.filtered += (increase - self.filtered) * (1 - math.exp(-dt / .2))
        # Hysteresis applies only to valid current, never to contact/data faults.
        if self.filtered >= .05:
            self.current_latched = True
            self.low_current_since = None
        elif self.current_latched and self.filtered <= .025:
            if self.low_current_since is None:
                self.low_current_since = now
            elif now - self.low_current_since >= .25:
                self.current_latched = False
        else:
            self.low_current_since = None
        # Output gain is explicit; raw-output upward slew stays bounded.
        target = min(1.0, max(0.0, (self.filtered - .05) / self.current_range_a))
        if self.current_latched:
            target = max(1.0 / self.output_limit, target)
        step = 5.0 / self.output_limit * dt
        self.effort = min(target, self.effort + step) if target >= self.effort else max(target, self.effort - step)
        self.state = 'relative current cue; posture uncorrected'
        return self.effort

    def _prepare_grasp_baseline(self, current, velocity, pose, now, contact):
        if not contact:
            if self.was_contact:
                self.pregrasp_samples.clear()
                self.pregrasp_candidate = None
            self.was_contact = False
            self.grasp_baseline_valid = False
            self.grasp_baseline_time = None
            self.accepted_increase = None
            self.load_samples.clear()
            self.load_target = self.effort = self.filtered = 0.0
            self.load_received = None
            self.delta = None
            if max(abs(v) for v in velocity) <= .01:
                self.pregrasp_samples.append((now, list(current), list(pose)))
                self.pregrasp_samples = [s for s in self.pregrasp_samples if now-s[0] <= 1.2]
                samples = self.pregrasp_samples
                values = [s[1][self.index] for s in samples]
                stable_pose = all(max(s[2][i] for s in samples)-min(s[2][i] for s in samples) <= .01
                                  for i in range(6))
                if (len(samples) >= 25 and now-samples[0][0] >= 1 and
                        max(values)-min(values) <= .08 and stable_pose):
                    self.pregrasp_candidate = (now,
                        [statistics.median(s[1][i] for s in samples) for i in range(6)],
                        [statistics.median(s[2][i] for s in samples) for i in range(6)])
            else:
                self.pregrasp_samples.clear()
            self.state = 'collecting pre-grasp stationary current; cue off'
            if self.pregrasp_candidate and now-self.pregrasp_candidate[0] <= 2:
                self.state = 'pre-grasp baseline ready; cue off'
            return False
        if not self.was_contact:
            candidate = self.pregrasp_candidate
            self.grasp_baseline_valid = bool(candidate and now-candidate[0] <= 2 and
                max(abs(a-b) for a, b in zip(pose, candidate[2])) <= .035)
            if self.grasp_baseline_valid:
                self.grasp_baseline_time, self.baseline, self.baseline_pose = candidate
            self.pregrasp_candidate = None
            self.pregrasp_samples.clear()
        self.was_contact = True
        if not self.grasp_baseline_valid:
            self.effort = self.load_target = 0.0
            self.load_received = None
            self.delta = None
            self.state = 'no fresh pre-grasp baseline; release and reacquire'
            return False
        return True

    def _contrast_update(self, signed_delta, velocity, pose, now, dt):
        # Reject moving samples rather than inventing a dynamic compensation
        # model. Startup-reference mode remains sensitive to static posture.
        pose_matches = (self.baseline_pose is not None and
                        max(abs(a-b) for a, b in zip(pose, self.baseline_pose)) <= .035)
        self.load_max_speed = max(abs(v) for v in velocity)
        eligible = (self.load_max_speed <= .01 and
                    (not self.pregrasp or pose_matches))
        if eligible:
            # Check stability before taking abs: alternating signs must not
            # masquerade as a steady load. Posture effects remain uncompensated.
            self.load_samples.append((now, signed_delta))
            self.load_samples = [s for s in self.load_samples if now-s[0] <= .7]
            values = [s[1] for s in self.load_samples]
            self.load_window_span = now-self.load_samples[0][0]
            self.load_window_spread = max(values)-min(values)
            if self.load_window_spread > .08:
                self.load_wait_reason = 'current spread exceeds 0.08A'
            elif len(values) < 25 or self.load_window_span < .5:
                self.load_wait_reason = 'collecting 0.5s and 25 fresh stationary samples'
            else:
                self.load_wait_reason = 'accepted'
                self.accepted_increase = statistics.median(values)
                self.load_target = self.contrast_fraction(abs(self.accepted_increase))
                self.load_received = now
        else:
            self.load_samples.clear()
            self.load_window_span = 0.0
            self.load_window_spread = None
            self.load_wait_reason = ('joint speed exceeds 0.01rad/s' if self.load_max_speed > .01
                                     else 'outside pre-grasp pose tolerance')
        # Retain the last load target during this grasp, not its input sample.
        # Every tick still requires fresh RTDE/contact and valid joint feedback.
        if self.load_received is None:
            self.effort = 0.0
            self.load_target = 0.0
            self.state = ('waiting for stable current at baseline pose; cue off' if self.pregrasp else
                          'waiting for stationary loaded current; motion samples excluded; cue off')
            return 0.0
        step = 5.0/self.output_limit*dt
        self.effort += max(-step, min(step, self.load_target-self.effort))
        self.state = 'stationary contrast cue; not mass calibrated' if eligible else 'holding last load cue during grasp; live data required'
        return self.effort

    def contrast_fraction(self, increase):
        low, mid, high = self.contrast_low, self.contrast_mid, self.contrast_high
        if increase <= mid:
            return CONTRAST_OUTPUT_GAIN * max(0.0, .4 * (increase-low)/(mid-low))
        return CONTRAST_OUTPUT_GAIN * min(1.0, .4 + .6 * (increase-mid)/(high-mid))

    def status(self):
        return dict(state=self.state, source='joint_current', estimated_kg=None,
                    zero_force_n=None, cue_max_kg=None, tare_in_progress=self.taring,
                    auto_tare_pending=self.auto_tare_pending,
                    output_cap_raw=self.output_limit,
                    current_profile='stationary_contrast' if self.contrast else 'linear',
                    baseline_source='pre_grasp_stationary' if self.contrast and self.pregrasp else 'startup_or_manual',
                    grasp_baseline_valid=(self.grasp_baseline_valid if self.pregrasp else
                                          self.baseline is not None and not self.taring),
                    motion_current_handling='exclude_moving_samples_hold_during_valid_grasp' if self.contrast else 'uncompensated',
                    grasp_baseline_pc_s=self.grasp_baseline_time,
                    accepted_magnitude_increase_a=None,
                    accepted_signed_delta_a=self.accepted_increase,
                    accepted_absolute_delta_a=(abs(self.accepted_increase)
                                               if self.accepted_increase is not None else None),
                    load_wait_reason=self.load_wait_reason,
                    load_window_span_s=self.load_window_span,
                    load_window_spread_a=self.load_window_spread,
                    load_max_joint_speed_rad_s=self.load_max_speed,
                    support_clearance_verified=False,
                    contrast_low_a=self.contrast_low if self.contrast else None,
                    contrast_mid_a=self.contrast_mid if self.contrast else None,
                    contrast_mid_fraction=.4 * CONTRAST_OUTPUT_GAIN if self.contrast else None,
                    body_output_gain=CONTRAST_OUTPUT_GAIN if self.contrast else 1.0,
                    current_mapping='piecewise_current_v5_absolute_delta_gain_0_6' if self.contrast else 'linear',
                    contrast_high_a=self.contrast_high if self.contrast else None,
                    load_target_fraction=self.load_target if self.contrast else None,
                    load_age_sec=(self.received-self.load_received if self.load_received is not None else None),
                    current_range_a=(self.contrast_high-self.contrast_low if self.contrast else self.current_range_a),
                    current_deadband_a=(self.contrast_low if self.contrast else .05),
                    current_a=self.current, baseline_current_a=self.baseline,
                    delta_current_a=self.delta, cue_joint=self.index + 1,
                    cue_fraction=self.effort, posture_compensated=False,
                    external_load_verified=False, sample_received_monotonic_s=self.received,
                    robot_stamp_s=self.stamp)


class JointCurrentPair(JointCurrentCue):
    def __init__(self, auto_tare=False, output_limit=10, current_range_a=None, contrast=False, wrist=False,
                 pregrasp=False):
        self.contrast = contrast
        self.pregrasp = pregrasp
        self.cues = {i: JointCurrentCue(i, auto_tare=auto_tare, output_limit=min(output_limit, 10) if i == 4 else output_limit,
                                      current_range_a=current_range_a, contrast=contrast, pregrasp=pregrasp)
                     for i in ((2, 3, 4) if wrist else (2, 3))}

    def request_tare(self, now):
        for cue in self.cues.values():
            cue.request_tare(now)

    def invalidate(self, reason):
        for cue in self.cues.values():
            cue.invalidate(reason)

    def check_tare_timeout(self, now):
        for cue in self.cues.values():
            cue.check_tare_timeout(now)

    def update(self, *args):
        results = {i: cue.update(*args) for i, cue in self.cues.items()}
        if any(cue.baseline is None or cue.taring or
               (self.contrast and self.pregrasp and not cue.grasp_baseline_valid) for cue in self.cues.values()):
            for cue in self.cues.values():
                cue.effort = cue.filtered = 0.0
            return 0.0
        return max(results.values())

    def efforts(self):
        return {i: cue.effort for i, cue in self.cues.items()}

    def status(self):
        result = self.cues[2].status()
        result.update(cue_joint=None, cue_joints=list(self.cues),
                      joint_cues={str(i): cue.status() for i, cue in self.cues.items()},
                      tare_in_progress=any(c.taring for c in self.cues.values()),
                      auto_tare_pending=any(c.auto_tare_pending for c in self.cues.values()),
                      cue_fraction=max(self.efforts().values()))
        if any(c.baseline is None for c in self.cues.values()):
            result['state'] = 'both current baselines required'
        elif self.contrast and self.pregrasp and any(not c.grasp_baseline_valid for c in self.cues.values()):
            result['state'] = 'all selected pre-grasp baselines required; cue off'
        return result
