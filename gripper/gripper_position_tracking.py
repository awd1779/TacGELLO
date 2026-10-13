"""Hardware-independent position tracking and contact gating."""

import math


class PositionTracker:
    def __init__(self, open_raws, close_raws, initial_raws, speed,
                 margin=45.0, confirm_sec=0.10, confirm_samples=4,
                 required_sensors=2, command_timeout=0.5, release_delta=0.01):
        self.open = tuple(open_raws)
        self.closed = tuple(close_raws)
        self.target = tuple(initial_raws)
        self.speed = speed
        self.margin = margin
        self.confirm_sec = confirm_sec
        self.confirm_samples = confirm_samples
        self.required = required_sensors
        self.command_timeout = command_timeout
        if not math.isfinite(release_delta) or not 0 < release_delta <= 0.1:
            raise ValueError('release_delta must be finite and in (0, 0.1]')
        self.release_delta = release_delta
        self.desired = None
        self.received = None
        self.contact = False
        self.blocked = False
        self.state = "waiting for controller"
        self.timers = {}
        self.last_tokens = {}
        self.hold_input = None
        self.offset = 0.0
        self.opening_release = False
        self.stop_requested = False
        self.open_return_input = None
        self.reinforcement_active = False
        self.candidate_hold = False
        self._reset_candidate_recovery()

    def _reset_candidate_recovery(self):
        self._clear_candidate_evidence()
        self.candidate_resume_count = 0
        self.candidate_resume_origin = None

    def _clear_candidate_evidence(self):
        self.candidate_clear_start = self.candidate_clear_last = None
        self.candidate_clear_count = 0
        self.candidate_clear_tokens = None

    def _candidate_cleared(self, now, channels, tokens):
        if (len(channels) < 2 or len(tokens) != len(channels) or
                any(v >= self.margin * .6 for row in channels for v in row)):
            self._clear_candidate_evidence()
            return False
        if self.candidate_clear_last is not None and now - self.candidate_clear_last > .1:
            self._clear_candidate_evidence()
        if (self.candidate_clear_tokens is not None and
                any(a == b for a, b in zip(tokens, self.candidate_clear_tokens))):
            return False
        if self.candidate_clear_start is None:
            self.candidate_clear_start = now
        self.candidate_clear_last = now
        self.candidate_clear_tokens = tuple(tokens)
        self.candidate_clear_count += 1
        return now - self.candidate_clear_start >= .15 and self.candidate_clear_count >= 3

    @property
    def motion_phase(self):
        if self.contact:
            return 'slip reinforcement' if self.reinforcement_active else 'contact paused'
        return self.state

    def can_issue_motion(self, source, now):
        if self.received is None or now - self.received > self.command_timeout:
            return False
        if source == 'reinforcement':
            return (self.contact and not self.blocked and not self.stop_requested
                    and self.desired is not None and self.desired > 0.003
                    and self.hold_input is not None
                    and not self.opening_requested(self.desired))
        # The tracking loop must process an opening release before it owns
        # motion again. Contact pause never writes the old target repeatedly.
        return (source == 'tracking' and not self.contact and not self.stop_requested
                and not self.candidate_hold)

    def acknowledge_reinforcement(self, target):
        self.target = tuple(target)
        self.reinforcement_active = True

    def finish_reinforcement(self):
        # Keep the new target AND the original lever lock anchor. No HOLD write.
        self.reinforcement_active = False

    def opening_requested(self, desired):
        return desired is not None and (desired <= 0.003 or
            (self.hold_input is not None and self.hold_input - desired >= self.release_delta))

    def command(self, ratio, now):
        if not math.isfinite(ratio):
            return
        self.desired = min(1.0, max(0.0, ratio))
        self.received = now

    def ratio(self):
        ratios = [(v - a) / (b - a) for v, a, b in
                  zip(self.target, self.open, self.closed) if b != a]
        return min(1.0, max(0.0, sum(ratios) / len(ratios)))

    def wants_full_open(self, desired):
        return desired is not None and (desired <= 0.003 or
            (self.opening_release and max(0.0, desired - self.offset) <= 0.003))

    def release_at_measured_open(self, positions, now, tolerance):
        if (not self.wants_full_open(self.desired) or self.received is None
                or now - self.received > self.command_timeout
                or len(positions) != len(self.open)
                or any(v is None or not math.isfinite(v) or abs(v - goal) > tolerance
                       for v, goal in zip(positions, self.open))):
            return False
        self.contact = self.blocked = False
        self.candidate_hold = False
        self._reset_candidate_recovery()
        self.reinforcement_active = False
        self.hold_input = None
        self.stop_requested = False
        self.timers.clear()
        self.last_tokens.clear()
        self.opening_release = True
        self.state = "opening"
        return True

    def tick(self, now, dt, channels, sample_tokens, valid_baseline):
        self.stop_requested = False
        was_pending = self.candidate_hold
        if self.desired is None:
            return None
        current = self.ratio()
        released = self.hold_input is not None and self.opening_requested(self.desired)
        # A measured-pose update must not look like an operator opening command.
        if self.contact and not released and self.desired > 0.003:
            # Keep the contact anchor fixed: recoil after pulling harder is not OPEN.
            self.state = "contact hold"
            return None
        if released:
            # Release on a lever reversal even if it was pulled beyond contact.
            self.offset = max(0.0, self.desired - max(0.0, current - 0.01))
            self.hold_input = None
        if self.desired <= 0.003:
            self.offset = 0.0
            self.open_return_input = None
        # Release offsets must not create a lasting dead zone after returning open.
        # Wait for a deliberate closing reversal; never close just by clearing an offset.
        if current <= 0.003 and self.open_return_input is not None:
            if self.desired > self.open_return_input + 0.01:
                self.offset = 0.0
                self.open_return_input = None
            else:
                self.open_return_input = min(self.open_return_input, self.desired)
        requested = max(0.0, self.desired - self.offset)
        if current <= 0.003 and requested <= 0.003 and self.offset > 0:
            if self.open_return_input is None:
                self.open_return_input = self.desired
        elif current > 0.003:
            self.open_return_input = None
        if now - self.received > self.command_timeout:
            self._clear_candidate_evidence()
            self.stop_requested = self.candidate_resume_origin is not None and not was_pending
            self.blocked = True
            self.hold_input = self.desired
            self.timers.clear()
            self.state = "controller timeout; open to rearm"
            return None
        # Opening intent clears the latch even when no travel remains.
        # Keep it clear at the opening target until a new closing request.
        if released or self.desired <= 0.003 or requested < current - 0.003:
            self.opening_release = True
        elif requested > current + 0.003:
            self.opening_release = False
        opening = requested < current - 0.003 or self.opening_release
        if was_pending and not released and self.desired > 0.003:
            opening = False
        if opening:
            self.candidate_hold = False
            self._reset_candidate_recovery()
            self.contact = self.blocked = False
            self.reinforcement_active = False
            self.hold_input = None
            self.timers.clear()
            self.state = "opening"
        else:
            if self.contact or self.blocked:
                self.hold_input = max(self.desired, self.hold_input or 0.0)
                self.state = "contact hold" if self.contact else "blocked; open to rearm"
                return None
            if not valid_baseline:
                self._clear_candidate_evidence()
                if self.candidate_resume_origin is not None:
                    self.stop_requested = not was_pending
                    self.candidate_hold = True
                    self.hold_input = self.desired
                self.state = "empty-close calibration required"
                return None
            if (not channels or len(channels) < self.required or len(sample_tokens) != len(channels)
                    or any(not row or any(not math.isfinite(v) for v in row)
                           for row in channels)):
                self._clear_candidate_evidence()
                self.stop_requested = self.candidate_resume_origin is not None and not was_pending
                self.blocked = True
                self.hold_input = self.desired
                self.state = "sensor lost; open to rearm"
                return None
            pending_sensors = set()
            mature = set()
            fresh_pair = all(self.last_tokens.get(i) != token for i, token in enumerate(sample_tokens))
            for sensor, row in enumerate(channels):
                fresh = self.last_tokens.get(sensor) != sample_tokens[sensor]
                self.last_tokens[sensor] = sample_tokens[sensor]
                for channel, delta in enumerate(row):
                    key = (sensor, channel)
                    if delta < self.margin:
                        self.timers.pop(key, None)
                        continue
                    pending_sensors.add(sensor)
                    start, count = self.timers.get(key, (now, 0))
                    count += int(fresh)
                    self.timers[key] = (start, count)
                    if fresh and now - start >= self.confirm_sec and count >= self.confirm_samples:
                        mature.add(sensor)
            if len(mature) >= self.required:
                self._clear_candidate_evidence()
                self.stop_requested = not was_pending
                self.contact = True
                if not was_pending:
                    self.hold_input = self.desired
                self.state = "contact hold"
                return None
            if (len(pending_sensors) >= self.required or
                    (self.candidate_resume_origin is not None and
                     any(v >= self.margin * .6 for row in channels for v in row))):
                self._clear_candidate_evidence()
                # Cancel outstanding travel before waiting for contact confirmation.
                self.stop_requested = not was_pending
                self.candidate_hold = True
                if not was_pending:
                    self.hold_input = self.desired
                self.state = "confirming contact; target held"
                return None
            if was_pending:
                if not self._candidate_cleared(now, channels, sample_tokens):
                    self.state = "contact candidate held; waiting for both sensors clear"
                    return None
                if self.candidate_resume_count >= 2:
                    self.state = "candidate retry limit; open lever to resume"
                    return None
                if requested <= current + .003:
                    self.state = "contact candidate clear; waiting for closing request"
                    return None
                self.candidate_resume_count += 1
                if self.candidate_resume_origin is None:
                    self.candidate_resume_origin = self.target
                self.candidate_hold = False
                self._clear_candidate_evidence()
            self.state = "tracking"
            self.hold_input = None
            if self.candidate_resume_origin is not None and not fresh_pair:
                self.state = "candidate recovery waiting for fresh sensor pair"
                return None
            if self.candidate_resume_origin is not None:
                self.state = "candidate cleared; bounded slow closing"
        if self.desired <= 0.003 and current <= 0.003:
            self.offset = 0.0
            self.blocked = self.contact = False
        desired = tuple(round(a + requested * (b - a))
                        for a, b in zip(self.open, self.closed))
        distance = max(abs(b - a) for a, b in zip(self.target, desired))
        if not opening and self.candidate_resume_origin is not None:
            # Bound all recovery attempts from the first held pose, not per retry.
            directions = [1 if b > a else -1 for a, b in zip(self.open, self.closed)]
            limits = [start + direction * 8 for start, direction in
                      zip(self.candidate_resume_origin, directions)]
            if any(direction * (limit - value) <= 0 for direction, limit, value in
                   zip(directions, limits, self.target)):
                self.stop_requested = True
                self.blocked = True
                self.hold_input = self.desired
                self.state = "candidate recovery travel limit; open lever to resume"
                return None
            desired = tuple(direction * min(direction * goal, direction * limit)
                            for direction, goal, limit in zip(directions, desired, limits))
            distance = max(abs(b - a) for a, b in zip(self.target, desired))
        if not distance:
            return None
        # Bound target lead even when Bluetooth ACKs or scheduling are delayed.
        step = max(1, min(8 if opening else 4, self.speed * min(dt, 0.02)))
        if not opening and self.candidate_resume_origin is not None:
            step = min(step, 2)
        fraction = min(1.0, step / distance)
        target = tuple(round(a + fraction * (b - a))
                       for a, b in zip(self.target, desired))
        return target if target != self.target else None

    def acknowledge(self, target):
        self.target = tuple(target)
