"""Lever hold state reported by the controller, not measured resistance force."""
import math


FIELDS = ['start_sec', 'end_sec', 'servo_id', 'hold_target_raw', 'output_limit_raw', 'end_reason']


def intervals(events, origin):
    frames = sorted((e for e in events if e.get('component') == 'gripper_lever_feedback'
                     and isinstance(e.get('observed_pc_ns'), (int, float))
                     and math.isfinite(e['observed_pc_ns'])), key=lambda e: e['observed_pc_ns'])
    rows, current, last = [], None, None
    for frame in frames:
        stamp = frame['observed_pc_ns']/1e9-origin
        signature = tuple(frame.get(k) for k in ('servo_id', 'hold_target_raw', 'output_limit_raw'))
        active = frame.get('enabled') is True and frame.get('hold_active') is True and not frame.get('fault')
        gap = last is not None and stamp-last > .5
        if current is not None and (gap or not active or signature != current[1]):
            start, old_signature = current
            rows.append(dict(zip(FIELDS, [start, last if gap else stamp, *old_signature,
                'telemetry gap; end unknown' if gap else frame.get('reason') or frame.get('event', 'state changed')])))
            current = None
        if active and current is None:
            current = (stamp, signature)
        last = stamp
    if current is not None:
        rows.append(dict(zip(FIELDS, [current[0], last, *current[1], 'end not observed'])))
    return rows
