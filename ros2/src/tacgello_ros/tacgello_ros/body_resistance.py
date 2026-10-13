"""Opt-in, output-limited position-lag cue. Not calibrated torque control.

The host cannot guarantee release after cable loss or process termination.
Leave the low SRAM limit in place on exit; never restore full power here.
"""

import math


class BodyResistance:
    def __init__(self, packet, port, servo_id=2, output_limit=10, continuous=False):
        if servo_id not in (2, 3, 4):
            raise ValueError('Only body IDs 2, 3, 4 may be selected')
        if servo_id == 4 and output_limit > 10:
            raise ValueError('Wrist trial output limit must not exceed 10')
        self.packet, self.port, self.servo_id = packet, port, servo_id
        if isinstance(output_limit, bool) or not isinstance(output_limit, int) or not 1 <= output_limit <= 40:
            raise ValueError('Experimental body output limit must be an integer 1..40')
        self.limit = output_limit  # Register setting, NOT measured torque percentage.
        self.ready = self.active = False
        self.fault = ''
        self.previous = None
        self.previous_time = None
        self.contact = False
        self.contact_time = None
        self.last_release_retry = None
        self.last_off_check = None
        self.continuous = continuous
        self.filtered_velocity = 0.0
        self.output_envelope = 0.0

    def ensure_off(self, now):
        if self.active or self.last_off_check is None or now - self.last_off_check >= .5:
            self.release(verify=True)
            self.last_off_check = now

    def write(self, address, values):
        params = [self.servo_id, *values]
        result = self.packet.syncWriteTxOnly(
            self.port, address, len(values), params, len(params))
        if result != 0:
            raise RuntimeError(f'write register {address}: communication={result}')

    def read(self, address, width=1):
        method = self.packet.read1ByteTxRx if width == 1 else self.packet.read2ByteTxRx
        value, result, error = method(self.port, self.servo_id, address)
        if result or error:
            raise RuntimeError(f'read register {address}: communication={result}, error={error}')
        return value

    def initialize(self):
        if self.read(33) != 0:
            raise RuntimeError('Position mode required; mode was NOT changed')
        if self.read(40) != 0:
            raise RuntimeError('Servo must start with torque OFF')
        self.write(48, [self.limit, 0])
        if self.read(48, 2) != self.limit:
            raise RuntimeError('Output-limit readback mismatch')
        self.ready = True

    def release(self, verify=False):
        self.write(40, [0])
        if verify and self.read(40) != 0:
            raise RuntimeError('Torque-OFF readback mismatch')
        self.active = False
        self.filtered_velocity = 0.0
        self.output_envelope = 0.0

    def feedback(self, contact, now):
        self.contact = bool(contact)
        self.contact_time = now

    def fail(self, detail):
        self.fault = detail
        try:
            self.release(verify=True)
        except Exception as exc:
            self.fault += f'; RELEASE NOT CONFIRMED: {exc}'

    def update(self, raw, now, enabled=True, effort=1.0):
        if self.fault:
            if self.last_release_retry is None or now - self.last_release_retry >= 0.5:
                self.last_release_retry = now
                try:
                    self.release(verify=True)
                except Exception:
                    pass
            return
        try:
            if not math.isfinite(effort) or not 0 <= effort <= 1:
                raise RuntimeError('Invalid cue effort')
            if not enabled or effort == 0:
                self.ensure_off(now)
                self.previous = None
                self.previous_time = None
                return
            if not self.ready:
                self.initialize()
            if raw is None or not math.isfinite(now) or not 0 <= raw <= 4095:
                raise RuntimeError('Missing or invalid fresh position')
            previous, previous_time = self.previous, self.previous_time
            self.previous, self.previous_time = raw, now
            fresh_contact = (self.contact and self.contact_time is not None
                             and 0 <= now - self.contact_time <= 0.35)
            if previous_time is not None and not 0 < now - previous_time <= 0.15:
                raise RuntimeError('Controller update gap; restart resistance feature')
            delta = 0 if previous is None else raw - previous
            if abs(delta) > 100:
                raise RuntimeError('Position jump or encoder wrap; resistance inhibited')
            if not fresh_contact or not 4 <= raw <= 4091:
                self.ensure_off(now)
                return
            # At most two counts of lag, recomputed from every fresh observation.
            # This limits return travel, but is not guaranteed passive damping.
            if self.continuous:
                if previous_time is None:
                    self.ensure_off(now)
                    return
                dt = now - previous_time
                velocity = delta / dt
                if delta == 0 or velocity * self.filtered_velocity < 0:
                    # Never retain the old lag through rest or direction reversal.
                    self.filtered_velocity = 0.0
                else:
                    self.filtered_velocity += (velocity - self.filtered_velocity) * (1 - math.exp(-dt / .08))
                lag = round(2 * math.tanh(self.filtered_velocity / 50))
                target = round(raw) - lag
                # Ramp the hardware output too, even if the upstream cue was
                # already high while this motor was inactive.
                self.output_envelope = min(self.limit * effort, self.output_envelope + 5 * dt)
                output_limit = max(1, min(self.limit, math.ceil(self.output_envelope)))
            else:
                if abs(delta) < 2:
                    self.ensure_off(now)
                    return
                target = raw - (2 if delta > 0 else -2)
                output_limit = max(1, round(self.limit * effort))
            if not self.active or output_limit != getattr(self, 'applied_limit', None):
                self.write(48, [output_limit, 0])
                if self.read(48, 2) != output_limit:
                    raise RuntimeError('Output-limit readback mismatch')
                self.applied_limit = output_limit
            self.write(41, [20, target & 255, target >> 8, 0, 0, 100, 0,
                            output_limit, 0])
            if not self.active:
                self.write(40, [1])
                self.active = True
                if self.read(40) != 1:
                    raise RuntimeError('Torque-ON readback mismatch')
        except Exception as exc:
            self.fail(str(exc))

    def close(self):
        if self.ready or self.active:
            try:
                self.release(verify=True)
            except Exception as exc:
                self.fault = f'RELEASE NOT CONFIRMED: {exc}'


class BodyResistancePair:
    """Independent shoulder/elbow cues with a shared fail-closed boundary."""
    servo_id = 2  # Legacy summary; per-servo details are available via status().

    def __init__(self, packet, port, output_limit=10, continuous=False, wrist=False):
        self.motors = {i: BodyResistance(packet, port, i, min(output_limit, 10) if i == 4 else output_limit,
                                        continuous) for i in ((2, 3, 4) if wrist else (2, 3))}

    @property
    def fault(self):
        return '; '.join(f'ID {i}: {m.fault}' for i, m in self.motors.items() if m.fault)

    @property
    def active(self):
        return any(m.active for m in self.motors.values())

    @property
    def applied_limit(self):
        return max(getattr(m, 'applied_limit', 0) if m.active else 0 for m in self.motors.values())

    def feedback(self, contact, now):
        for m in self.motors.values():
            m.feedback(contact, now)

    def fail(self, detail):
        for m in self.motors.values():
            m.fail(detail)

    def update(self, raw, now, enabled=True, effort=0):
        if self.fault:
            enabled = False
        for i, m in self.motors.items():
            value = effort.get(i, 0) if isinstance(effort, dict) else effort
            m.update(raw.get(i) if isinstance(raw, dict) else None, now, enabled, value)
            if m.fault:
                # Do not enable the next joint after an earlier joint failed.
                enabled = False
        if self.fault:
            for m in self.motors.values():
                m.update(None, now, enabled=False, effort=0)

    def status(self):
        return [dict(servo_id=i, resistance_active=m.active, resistance_fault=m.fault,
                     resistance_mode='continuous_position_lag' if m.continuous else 'motion_gated',
                     output_limit_raw=getattr(m, 'applied_limit', 0) if m.active else 0)
                for i, m in self.motors.items()]

    def close(self):
        for m in self.motors.values():
            m.close()
