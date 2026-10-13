"""Bounded position-error ramp for loaded servos, not a force controller.

Advance only after reading back the previous goal. Output thresholds below
block further advances; they are NOT hardware current/PWM limits.
"""

import math

STEP_RAW = 13
SECOND_STEP_RAW = 4
MAX_STAGES = 3
MAX_CLOSING_ERROR_RAW = 18
EPISODE_TRAVEL_RAW = 25
GRASP_TRAVEL_RAW = 50
SETTLE_SEC = 0.4
OBSERVE_SEC = 0.8
DEADLINE_SEC = 3.5
OBSERVATION_GRACE_SEC = 0.75
CONTACT_RECHECK_SEC = 0.25
CONFIRM_CLOSING_RAW = 12
MAX_PWM_RAW = 150
MAX_CURRENT_RAW = 100


def feedback_pending(records, now, after_ns):
    """Missing/old replies need another read, not another movement."""
    return any(r is None or not math.isfinite(r.get('observed_pc_ns', 0))
               or r.get('observed_pc_ns', 0) <= after_ns
               or not 0 <= now - r.get('observed_pc_ns', 0) / 1e9 <= .35
               for r in records)


def signed_magnitude(value, sign_bit):
    sign = 1 << sign_bit
    return -(value & ~sign) if value & sign else value


def diagnostic_summary(records, trace, now, error):
    """Snapshot before any measured-pose hold replaces the reinforcement goal."""
    jaws = []
    for i in range(2):
        record = records[i] or {}
        target = trace['target'][i]
        actual = record.get('actual_raw')
        load, current = record.get('load_raw'), record.get('current_raw')
        jaws.append(dict(
            id=i + 1, before_raw=trace['before'][i], target_raw=target,
            goal_raw=record.get('goal_raw'), actual_raw=actual,
            goal_matches=record.get('goal_raw') == target,
            encoder_delta_raw=actual - trace['before'][i] if actual is not None else None,
            remaining_error_raw=target - actual if actual is not None else None,
            pwm_raw=signed_magnitude(load, 10) if type(load) is int else None,
            current_raw=signed_magnitude(current, 15) if type(current) is int else None,
            voltage_raw=record.get('voltage_raw'), torque_limit_raw=record.get('torque_limit_raw'),
            control_settings_available=record.get('control_settings_available', False),
            position_p_raw=record.get('position_p_raw'),
            position_i_raw=record.get('position_i_raw'),
            position_d_raw=record.get('position_d_raw'),
            minimum_start_output_raw=record.get('minimum_start_output_raw'),
            deadband_cw_raw=record.get('deadband_cw_raw'),
            deadband_ccw_raw=record.get('deadband_ccw_raw'),
            goal_speed_raw=record.get('goal_speed_raw'), moving_raw=record.get('moving_raw'),
            observations=trace.get('observations', [0, 0])[i],
            motion_confirmations=trace.get('motion_counts', [0, 0])[i],
            observed_pc_ns=record.get('observed_pc_ns')))
    return dict(attempt=trace.get('attempt'), elapsed_ms=round((now - trace['ack_ns'] / 1e9) * 1000, 1),
                feedback_error=error, jaws=jaws)


def feedback_error(records, previous, now, after_ns=0):
    if len(records) != 2:
        return 'both motor diagnostic replies required'
    for index, (record, goal) in enumerate(zip(records, previous), 1):
        prefix = f'jaw {index}: '
        if record is None or record.get('id') != index or record.get('ok') is not True:
            return prefix + 'motor diagnostic unavailable'
        stamp = record.get('observed_pc_ns', 0)
        if not math.isfinite(stamp) or not 0 <= now - stamp / 1e9 <= .35 or stamp <= after_ns:
            return prefix + 'motor diagnostic stale or predates command'
        if record.get('status') != 0 or record.get('servo_status_raw', 0) != 0:
            return prefix + 'motor fault reported'
        if record.get('torque_enable_raw') != 1 or record.get('mode_raw') != 0:
            return prefix + 'position control not enabled'
        if record.get('goal_raw') != goal:
            return prefix + 'goal readback differs from acknowledged target'
        load, current = record.get('load_raw'), record.get('current_raw')
        if (type(load) is not int or not 0 <= load <= 2047 or
                type(current) is not int or not 0 <= current <= 65535):
            return prefix + 'invalid motor output feedback'
        if abs(signed_magnitude(load, 10)) >= MAX_PWM_RAW:
            return prefix + 'PWM observation ceiling reached; no further advance'
        if abs(signed_magnitude(current, 15)) >= MAX_CURRENT_RAW:
            return prefix + 'current observation ceiling reached; no further advance'
        if record.get('torque_limit_raw', 0) <= 0:
            return prefix + 'zero torque limit'
    return None


def next_target(measured, previous, origin, episode_origin, opened, closed,
                stage=1):
    if type(stage) is not int or not 1 <= stage <= MAX_STAGES:
        return None, 'loaded reinforcement stage limit reached'
    step = STEP_RAW if stage == 1 else SECOND_STEP_RAW
    rows = (measured, previous, origin, episode_origin, opened, closed)
    if any(len(row) != 2 for row in rows):
        return None, 'two jaw values required'
    result = []
    for i, (value, old, start, episode, a, b) in enumerate(zip(*rows), 1):
        if not all(type(v) is int for v in (value, old, start, episode, a, b)):
            return None, f'jaw {i}: invalid position'
        if a == b or not all(min(a, b) <= v <= max(a, b) for v in (value, old, start, episode)):
            return None, f'jaw {i}: position outside configured limits'
        direction = 1 if b > a else -1
        if abs(value - old) > MAX_CLOSING_ERROR_RAW:
            return None, f'jaw {i}: previous goal error exceeds loaded ramp limit'
        # Plan both jaws atomically: a blocked jaw cancels the whole paired step.
        base = max(direction * old, direction * value)
        goal = direction * min(base + step, direction * episode + EPISODE_TRAVEL_RAW,
                               direction * start + GRASP_TRAVEL_RAW, direction * b,
                               direction * value + MAX_CLOSING_ERROR_RAW)
        if direction * (goal - old) <= 0 or direction * (goal - value) <= 0:
            return None, f'jaw {i}: loaded reinforcement travel budget exhausted'
        result.append(goal)
    target = tuple(result)
    return (target, None) if target != tuple(previous) else (None, 'both jaws already responded')


def episode_error(episode, prediction, now, epoch):
    if epoch != episode['epoch'] or not 0 <= now - episode['started'] <= DEADLINE_SEC:
        return 'loaded episode expired or calibration changed'
    if prediction is None:
        return 'loaded episode requires fresh model input'
    model_epoch, stamps, scores = prediction
    if (model_epoch != epoch or len(stamps) != 2 or len(scores) != 2 or
            any(not math.isfinite(t) or not 0 <= now - t <= .25 for t in stamps) or
            any(not math.isfinite(s) or not 0 <= s <= 1 for s in scores)):
        return 'loaded episode model input invalid or stale'
    # The confirmed event owns this short episode. A fourth high prediction is
    # not required; live contact, sensor rise and opening are rechecked by caller.
    return None
