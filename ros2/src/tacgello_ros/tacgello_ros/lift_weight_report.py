"""Read-only weight/cue report helpers. Register settings are not measured force."""
import html
import math
import statistics


FIELDS = ['time_sec', 'received_sec', 'contact', 'estimated_kg', 'cue_fraction',
          'resistance_enabled', 'resistance_active', 'output_limit_raw', 'servo_id',
          'state', 'resistance_fault', 'physical_resistance_verified']
FIELDS += ['source', 'resistance_mode', 'output_cap_raw', 'current_range_a', 'current_deadband_a', 'current_mapping', 'body_output_gain'] + [f'{field}_{i}' for i in (2, 3, 4) for field in
                       ('active', 'limit_raw', 'current_a', 'delta_a', 'cue_fraction')]
LOAD_FIELDS = ('load_wait_reason', 'load_window_span_s', 'load_window_spread_a',
               'load_max_joint_speed_rad_s', 'accepted_magnitude_increase_a', 'load_target_fraction',
               'accepted_signed_delta_a', 'accepted_absolute_delta_a')
FIELDS += [f'{field}_{i}' for i in (2, 3, 4) for field in LOAD_FIELDS]


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def sample(frame, origin, received):
    if not isinstance(frame, dict) or not finite(frame.get('observed_pc_ns')):
        return None
    stamp = frame['observed_pc_ns'] / 1e9 - origin
    if not 0 <= stamp <= received + .05 or received - stamp > .5:
        return None
    row = {field: frame.get(field) for field in FIELDS}
    row.update(time_sec=stamp, received_sec=received, physical_resistance_verified=False)
    for field in ('estimated_kg', 'cue_fraction', 'output_limit_raw', 'output_cap_raw'):
        if not finite(row[field]) or row[field] < 0:
            row[field] = None
    row['resistance_active'] = (frame.get('resistance_active') is True
        and frame.get('resistance_enabled') is True and frame.get('contact') is True
        and not frame.get('resistance_fault') and row['output_limit_raw'] is not None
        and row['output_limit_raw'] > 0)
    if not row['resistance_active']:
        row['output_limit_raw'] = 0
    servos = frame.get('body_servos', [])
    servos = servos if isinstance(servos, list) else []
    cues = frame.get('joint_cues', {})
    cues = cues if isinstance(cues, dict) else {}
    for i in (2, 3, 4):
        motor = next((m for m in servos if isinstance(m, dict) and m.get('servo_id') == i), None)
        if motor is None and not servos and frame.get('servo_id') == i:
            motor = frame
        cue = cues.get(str(i), frame if frame.get('cue_joint') == i else {})
        cue = cue if isinstance(cue, dict) else {}
        for field in LOAD_FIELDS:
            value = cue.get(field)
            row[f'{field}_{i}'] = value if (isinstance(value, str) if field == 'load_wait_reason'
                                          else finite(value)) else None
        row[f'active_{i}'] = False
        row[f'limit_raw_{i}'] = None
        if motor is not None:
            limit = motor.get('output_limit_raw')
            on = (row['resistance_active'] and motor.get('resistance_active') is True
                  and not motor.get('resistance_fault') and finite(limit) and limit > 0)
            row[f'active_{i}'] = bool(on)
            row[f'limit_raw_{i}'] = limit if on else 0
        for source, field in [('current_a', 'current_a'), ('delta_current_a', 'delta_a')]:
            values = cue.get(source)
            v = values[i-1] if isinstance(values, list) and len(values) == 6 else None
            row[f'{field}_{i}'] = v if finite(v) else None
        v = cue.get('cue_fraction')
        row[f'cue_fraction_{i}'] = v if finite(v) and 0 <= v <= 1 else None
    return row


def joint_rows(rows, servo_id):
    return [dict(r, resistance_active=r.get(f'active_{servo_id}', False),
                 output_limit_raw=r.get(f'limit_raw_{servo_id}'), estimated_kg=None)
            for r in rows]


def joint_intervals(rows):
    return [dict(region, servo_id=i) for i in (2, 3, 4)
            for region in intervals(joint_rows(rows, i))]


def controller_overlay(rows, servo_id, start, end, left, width, top, height):
    """Controller-only activity shading and a separately scaled output strip."""
    color = '#0f766e' if servo_id == 2 else '#b45309'
    duration = max(.001, end-start)
    x_at = lambda t: left + (t-start) / duration * width
    strip = top + height + 42
    maximum = max([10] + [r.get(f'limit_raw_{servo_id}') or 0 for r in rows]
                  + [r.get('output_cap_raw') or 0 for r in rows])
    parts = [f'<g class="controller-resistance" data-servo-id="{servo_id}">',
             f'<text x="18" y="{strip+18}" class="tick">ID{servo_id} output</text>',
             f'<text x="18" y="{strip+35}" class="tick">limit (raw)</text>',
             f'<rect x="{left}" y="{strip}" width="{width}" height="55" fill="#f8fafc" stroke="#cbd5e1"/>']
    for value in (0, maximum/2, maximum):
        y = strip + 55 - value / maximum * 55
        parts.append(f'<text x="{left-8}" y="{y+4}" text-anchor="end" class="tick">{value:g}</text>')
    for region in intervals(joint_rows(rows, servo_id)):
        a, b = max(start, region['start_sec']), min(end, region['end_sec'])
        if a > b:
            continue
        title = html.escape(f'ID{servo_id} reported resistance: {region["start_sec"]:.3f}-{region["end_sec"]:.3f}s; '
                            f'peak output {region["peak_output_limit_raw"]:g} raw; {region["end_reason"]}')
        parts.append(f'<rect class="controller-resistance-band" x="{x_at(a):.2f}" y="{top}" '
                     f'width="{max(1, x_at(b)-x_at(a)):.2f}" height="{height}" fill="{color}" opacity=".16"><title>{title}</title></rect>')
    visible = [r for r in rows if start <= r['time_sec'] <= end]
    previous = None
    for row in visible:
        value = row.get(f'limit_raw_{servo_id}')
        if value is None:
            previous = None
            continue
        x, y = x_at(row['time_sec']), strip + 55 - value / maximum * 55
        if previous and 0 < row['time_sec'] - previous[0] <= .5:
            parts.append(f'<path class="controller-output-step" d="M {previous[1]:.2f} {previous[2]:.2f} H {x:.2f} V {y:.2f}" '
                         f'fill="none" stroke="{color}" stroke-width="2"/>')
        parts.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="2" fill="{color}"><title>'
                     f'{row["time_sec"]:.3f}s: ID{servo_id} output {value:g} raw</title></circle>')
        previous = row['time_sec'], x, y
    peak = max((r.get(f'limit_raw_{servo_id}') or 0 for r in visible), default=0)
    note = (f'Shading: reported resistance ON | Peak {peak:g} raw | Same time axis | NOT measured torque/force'
            if any(r.get(f'limit_raw_{servo_id}') is not None for r in visible)
            else 'No per-joint resistance telemetry; activity unknown')
    parts.append(f'<text x="{left}" y="{strip+73}" class="tick">{note}</text></g>')
    return parts


def intervals(rows):
    result, run = [], []

    def finish(end, reason):
        weights = [r['estimated_kg'] for r in run if r['estimated_kg'] is not None]
        result.append(dict(start_sec=run[0]['time_sec'], end_sec=end, end_reason=reason,
                           mean_estimated_kg=statistics.mean(weights) if weights else None,
                           peak_output_limit_raw=max(r['output_limit_raw'] for r in run)))
        run.clear()

    last = None
    for row in sorted(rows, key=lambda r: r['time_sec']):
        stamp = row['time_sec']
        if last is not None and stamp <= last:
            continue
        if run and stamp - last > .5:
            finish(last, 'telemetry gap; physical release unknown')
        if run and row['output_limit_raw'] is None:
            finish(last, 'joint telemetry missing; physical release unknown')
        if row['resistance_active']:
            run.append(row)
        elif run:
            finish(stamp, 'cue inactive')
        last = stamp
    if run:
        finish(last, 'end not observed; physical release unknown')
    return result


def graph(rows, regions, duration, left, plot_width, top):
    """Two aligned panels; never connect missing estimates or telemetry gaps."""
    parts = []
    current_mode = any(r.get('source') == 'joint_current' for r in rows)
    panels = [
            ('estimated_kg', 'Estimated object (kg)', '#b45309', 2.4),
            ('output_limit_raw', 'Shoulder limit (raw)', '#0f766e', 10)]
    if current_mode:
        panels = [('limit_raw_2', 'ID2 shoulder (raw)', '#0f766e', 10),
                  ('limit_raw_3', 'ID3 elbow (raw)', '#b45309', 10)]
        if any(r.get('current_a_4') is not None for r in rows):
            panels.append(('limit_raw_4', 'ID4 wrist (raw)', '#a21caf', 10))
    for index, (field, title, color, floor) in enumerate(panels):
        panel_regions = intervals(joint_rows(rows, index+2)) if current_mode else regions
        y = top + index * 155
        maximum = max([floor] + [r.get(field) for r in rows if r.get(field) is not None]) * 1.1
        parts.append(f'<rect x="{left}" y="{y}" width="{plot_width}" height="110" fill="#f8fafc" stroke="#cbd5e1"/>')
        parts.append(f'<text x="18" y="{y+25}" class="label">{title}</text>')
        parts.append(f'<text x="18" y="{y+46}" class="tick">0 - {maximum:.2f}</text>')
        for region in panel_regions:
            x = left + region['start_sec'] / max(duration, .001) * plot_width
            span = max(1, (region['end_sec'] - region['start_sec']) / max(duration, .001) * plot_width)
            label = f'{region["start_sec"]:.3f}-{region["end_sec"]:.3f}s; peak {region["peak_output_limit_raw"]:g} raw; {region["end_reason"]}'
            parts.append(f'<rect class="body-resistance-interval" x="{x:.2f}" y="{y}" width="{span:.2f}" height="110" fill="{color}" opacity=".15"><title>{html.escape(label)}</title></rect>')
        previous = None
        for row in rows:
            value = row.get(field)
            if value is None:
                previous = None
                continue
            x = left + row['time_sec'] / max(duration, .001) * plot_width
            yy = y + 110 - value / maximum * 110
            if previous and 0 < row['time_sec'] - previous[0] <= .5:
                if field == 'output_limit_raw' or field.startswith('limit_raw_'):
                    parts.append(f'<path d="M {previous[1]:.2f} {previous[2]:.2f} H {x:.2f} V {yy:.2f}" fill="none" stroke="{color}" stroke-width="2"/>')
                else:
                    parts.append(f'<line x1="{previous[1]:.2f}" y1="{previous[2]:.2f}" x2="{x:.2f}" y2="{yy:.2f}" stroke="{color}" stroke-width="2"/>')
            detail = f'; current={row.get(f"current_a_{index+2}")} A; signed delta={row.get(f"delta_a_{index+2}")} A' if current_mode else ''
            parts.append(f'<circle cx="{x:.2f}" cy="{yy:.2f}" r="2" fill="{color}"><title>{row["time_sec"]:.3f}s: {value:.3f}{detail}</title></circle>')
            previous = row['time_sec'], x, yy
        if not rows:
            parts.append(f'<text x="{left+15}" y="{y+35}" class="note">No lift-weight telemetry recorded</text>')
    return parts


def table(regions, samples=()):
    if any(r.get('source') == 'joint_current' for r in samples):
        entries = ''.join(f'<tr><td>{r["servo_id"]}</td><td>{r["start_sec"]:.3f} - {r["end_sec"]:.3f} s</td>'
                          f'<td>{r["peak_output_limit_raw"]:g} raw</td><td>{html.escape(r["end_reason"])}</td></tr>'
                          for r in joint_intervals(samples))
        return ('<section><h2>Leader body current feedback</h2><p>Shading shows reported activation per joint. '
                'Output limit is NOT measured torque, force, or torque percentage. No posture compensation or mass estimate. '
                'Telemetry gaps are unknown. Configured output caps are saved in the sample CSV.</p>'
                '<table><tr><th>Motor ID</th><th>Active interval</th><th>Peak output setting</th><th>End</th></tr>'
                + entries + '</table><p><a href="body_joint_resistance_intervals.csv">Per-joint intervals</a> | '
                '<a href="lift_weight.csv">Current and output samples</a></p></section>')
    rows = []
    for i, r in enumerate(regions, 1):
        kg = 'Unavailable' if r['mean_estimated_kg'] is None else f'{r["mean_estimated_kg"]:.2f} kg'
        rows.append(f'<tr><td>{i}</td><td>{r["start_sec"]:.3f} - {r["end_sec"]:.3f} s</td>'
                    f'<td>{kg}</td><td>{r["peak_output_limit_raw"]:g} raw</td>'
                    f'<td>{html.escape(r["end_reason"])}</td></tr>')
    return ('<section><h2>Body resistance during object lift</h2>'
            '<p>Teal regions show reported shoulder cue activity, not contact alone. '
            'Weight is an approximate RTDE load estimate. Output limit is a servo register '
            'setting, NOT measured torque, force, or torque percentage. Physical resistance '
            'is unverified. Gaps are unknown, not continued resistance.</p>'
            '<table><tr><th>Cue</th><th>Interval</th><th>Mean estimated object weight</th>'
            '<th>Peak output limit</th><th>End</th></tr>' + ''.join(rows) + '</table>'
            + ('<p>No active body resistance observed.</p>' if not rows else '')
            + '<p><a href="lift_weight.csv">Weight and resistance samples</a> | '
            '<a href="body_resistance_intervals.csv">Resistance intervals</a></p></section>')
