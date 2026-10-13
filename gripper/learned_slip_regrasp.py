"""Opt-in model scoring and bounded trigger; motor I/O stays in the motion worker."""

import math
import threading
import time

from gripper_regrasp import RegraspGuard
from slip_observation import SlipObservation
from pretrained_anyskin_slip import PollenSlipModel, corrected_input


class SlipScorer:
    POLL_SEC = 0.025
    def __init__(self, monitor):
        self.monitor = monitor
        self.result = None
        self.error = 'model loading'
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        try:
            model = PollenSlipModel()
            while not self.stop.is_set():
                try:
                    snapshot = self.monitor.slip_model_snapshot()
                    now = time.monotonic()
                    sensors = snapshot['sensors']
                    if len(sensors) != 2:
                        raise ValueError('two healthy sensors required')
                    scores = tuple(model.score(corrected_input(s, now)) for s in sensors)
                    self.result = (snapshot['calibration_epoch'],
                                   tuple(s['received_pc_monotonic_s'] for s in sensors), scores)
                    self.error = None
                except Exception as exc:
                    self.result = None
                    self.error = str(exc)
                self.stop.wait(self.POLL_SEC)
        except Exception as exc:
            self.result = None
            self.error = str(exc)


class LearnedRegraspGuard(RegraspGuard):
    SLIP_THRESHOLD = SlipObservation.SLIP_THRESHOLD
    MAX_ATTEMPTS = 6
    RESPONSE_RISE_LIMIT = 500
    INITIAL_SETTLE_SEC = 0.30
    POST_MOTION_SETTLE_SEC = 0.35

    def reset(self):
        super().reset()
        self.prediction = None
        self.expected_epoch = None
        self.model_tokens = None
        self.high_counts = [0, 0]
        self.rearm_sensor = None
        self.clear_count = 0
        self.contact_since = None
        self.contact_epoch = None
        self.contact_lost_since = None
        self.contact_present = [False, False]
        self.confirmed_event = None
        self.initial_clear = False
        self.input_recovery = None
        self.recovery_clear_required = False
        self.input_recovered_at = None
        self.motion_confirmed = False
        self.motion_confirmed_at = None
        self.trigger_reason = 'new_slip'

    def note_move(self, values):
        self.motion_confirmed = False
        self.motion_confirmed_at = None
        super().note_move(values)

    def confirm_motion(self, now=None):
        self.motion_confirmed = True
        self.motion_confirmed_at = time.monotonic() if now is None else now
        self.high_counts = [0, 0]
        self.clear_count = 0

    def request_reinforcement(self, now, prediction, sensor, persistent=False):
        self.attempts += 1
        self.cooldown = now + 0.7
        self.rearm_sensor = sensor
        self.high_counts = [0, 0]
        self.confirmed_event = prediction
        self.trigger_reason = 'persistent_slip' if persistent else 'new_slip'
        # Only a NEW verified motor response can authorize another persistent request.
        self.motion_confirmed = False
        self.motion_confirmed_at = None
        self.state = f'{self.trigger_reason}: sensor {sensor + 1}, score={prediction[2][sensor]:.4f}'
        return True

    def pause_input(self, reason):
        if self.inhibited:
            return
        self.input_recovery = {'first': None, 'last': None, 'count': 0}
        self.recovery_clear_required = True
        self.confirmed_event = None
        self.high_counts = [0, 0]
        self.clear_count = 0
        self.model_tokens = None
        self.state = reason + '; waiting for stable fresh controller input'

    def recover_input(self, received, now):
        recovery = self.input_recovery
        if recovery is None:
            return True
        if self.inhibited:
            return False
        if recovery['last'] is None or received > recovery['last']:
            if recovery['first'] is None:
                recovery['first'] = received
            recovery['last'] = received
            recovery['count'] += 1
        if recovery['count'] < 3 or received - recovery['first'] < 0.2:
            return False
        self.input_recovery = None
        self.input_recovered_at = now
        self.model_tokens = None
        self.high_counts = [0, 0]
        self.clear_count = 0
        self.state = 'input recovered; waiting for clear scores then a new slip'
        return True

    def contact_mask(self, rows, margin):
        if (len(rows) != 2 or any(len(row) != 3 for row in rows)
                or not all(math.isfinite(v) for row in rows for v in row)):
            self.contact_present = [False, False]
            return self.contact_present
        # Only retain a contact that first crossed the full entry threshold.
        self.contact_present = [max(row) >= (margin * 0.5 if held else margin)
                                for row, held in zip(rows, self.contact_present)]
        return self.contact_present

    def event_preflight_error(self, prediction, now, epoch):
        event = self.confirmed_event
        if event is None or event[0] != epoch:
            return 'confirmed slip event missing or calibration changed'
        if any(not 0 <= now - stamp <= 0.25 for stamp in event[1]):
            return 'confirmed slip event older than 250ms; not replayed'
        if (prediction is None or prediction[0] != epoch or len(prediction[1]) != 2
                or len(prediction[2]) != 2
                or any(not math.isfinite(t) or not 0 <= now - t <= 0.25 for t in prediction[1])
                or any(not math.isfinite(s) or not 0 <= s <= 1 for s in prediction[2])):
            return 'model stream invalid or stale during position check'
        # Three high samples already confirmed this event. Do not require an
        # accidental fourth high sample after blocking motor-position reads.
        return None

    def observe(self, now, rows, tokens, margin):
        if self.inhibited or self.input_recovery is not None:
            return False
        if self.contact_since is None:
            self.contact_since = now
        if (len(rows) != 2 or any(len(row) != 3 for row in rows)
                or not all(math.isfinite(v) for row in rows for v in row)):
            return self.inhibit('invalid contact sensor input')
        contacts = self.contact_mask(rows, margin)
        if not any(contacts):
            self.high_counts = [0, 0]
            self.clear_count = 0
            if self.contact_lost_since is None:
                self.contact_lost_since = now
            if now - self.contact_lost_since >= 0.2:
                return self.inhibit('contact lost for 0.2s; no automatic chase')
            self.state = 'contact below release threshold; reinforcement paused'
            return False
        self.contact_lost_since = None
        if self.response_exceeded([v for row in rows for v in row]):
            if self.motion_confirmed:
                self.high_counts = [0, 0]
                self.clear_count = 0
                self.state = 'post-regrasp rise cap; waiting below original cap, then clear and new slip'
                return False
            return self.inhibit('post-regrasp rise cap reached')
        prediction = self.prediction
        if prediction is None:
            self.high_counts = [0, 0]
            self.clear_count = 0
            self.state = 'waiting for valid model scores'
            return False
        epoch, stamps, scores = prediction
        if (epoch != self.expected_epoch or len(stamps) != 2 or len(scores) != 2
                or any(not math.isfinite(t) or not 0 <= now - t <= 0.25 for t in stamps)
                or any(not math.isfinite(s) or not 0 <= s <= 1 for s in scores)):
            self.high_counts = [0, 0]
            self.clear_count = 0
            self.state = 'stale or invalid model scores; no reinforcement'
            return False
        if self.contact_epoch is None:
            self.contact_epoch = epoch
        elif epoch != self.contact_epoch:
            return self.inhibit('zero reference changed during grasp')
        if self.attempts >= self.MAX_ATTEMPTS:
            return self.inhibit(f'{self.MAX_ATTEMPTS}-attempt regrasp limit reached')
        previous = self.model_tokens
        self.model_tokens = stamps
        fresh = [previous is None or t > previous[i] for i, t in enumerate(stamps)]
        if self.recovery_clear_required:
            # Discard the pre-timeout event without replenishing any motion budget.
            self.high_counts = [0, 0]
            if any(t <= self.input_recovered_at for t in stamps):
                self.clear_count = 0
                return False
            if now >= self.cooldown and all(fresh):
                self.clear_count = self.clear_count + 1 if all(s <= 0.3 for s in scores) else 0
            if self.clear_count >= 3:
                self.recovery_clear_required = False
                self.rearm_sensor = None
                self.clear_count = 0
            self.state = 'input recovered; waiting for clear scores then a new slip'
            return False
        if not self.initial_clear:
            # Ignore contact transients, but do not require a low model score
            # before the first slip. Only samples acquired AFTER settling count.
            self.high_counts = [0, 0]
            if any(t <= self.contact_since + self.INITIAL_SETTLE_SEC for t in stamps):
                self.state = 'initial contact settling (0.30s); waiting for post-settle samples'
                return False
            self.initial_clear = True
            self.clear_count = 0
        if any(t <= self.contact_since for t in stamps):
            return False
        if self.rearm_sensor is not None:
            i = self.rearm_sensor
            # One episode spans both contacting sensors. A single low glitch or
            # the other sensor staying high must not cause repeated tightening.
            eligible = [j for j in range(2) if j == i or contacts[j]]
            if now >= self.cooldown and all(fresh[j] for j in eligible):
                self.clear_count = self.clear_count + 1 if all(scores[j] <= 0.3 for j in eligible) else 0
                if self.clear_count >= 3:
                    self.rearm_sensor = None
                    self.clear_count = 0
            # Persistent slip is not a new physical slip episode. Recheck only
            # after verified bilateral movement and post-motion sensor settling.
            ready = (self.rearm_sensor is not None and self.motion_confirmed
                     and self.motion_confirmed_at is not None and now >= self.cooldown
                     and all(t > self.motion_confirmed_at + self.POST_MOTION_SETTLE_SEC for t in stamps))
            if ready and all(fresh):
                for j in range(2):
                    self.high_counts[j] = (self.high_counts[j] + 1
                        if contacts[j] and scores[j] >= self.SLIP_THRESHOLD else 0)
                    if self.high_counts[j] >= 3:
                        # Static post-grasp deformation can keep the model high.
                        # Verified jaw motion is not independent evidence of slip.
                        self.state = 'persistent model high; repeat withheld until clear then new high event'
                        return False
            elif not ready:
                self.high_counts = [0, 0]
            self.state = ('slip cleared; waiting for a new slip episode' if self.rearm_sensor is None
                          else 'post-motion recheck: waiting for clear then new high event'
                          if self.motion_confirmed else 'waiting for verified motion or 3 fresh clear samples')
            return False
        self.state = f'watching model scores >= {self.SLIP_THRESHOLD:.2f} (3 fresh samples)'
        for i in range(2):
            if not fresh[i]:
                continue
            self.high_counts[i] = self.high_counts[i] + 1 if scores[i] >= self.SLIP_THRESHOLD and contacts[i] else 0
            if self.high_counts[i] >= 3:
                return self.request_reinforcement(now, prediction, i)
        return False
