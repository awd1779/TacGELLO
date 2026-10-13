"""Read-only episode summaries. Encoder displacement is not grip force."""
import html
import math


DIRECTION_FIELDS = ['event', 'time_sec', 'sensor', 'magnetic_axis', 'status',
                    'physical_direction', 'window_sec', 'delta_x', 'delta_y',
                    'delta_z', 'coherence']


def direction_events(diagnostics, origin):
    rows = []
    events = sorted((f for f in diagnostics if f.get('component') == 'gripper_slip_observation'
                     and f.get('transition') == 'started'), key=lambda f: f['observed_pc_ns'])
    for index, event in enumerate(events, 1):
        sensors = event.get('direction', {}).get('sensors', [])
        by_id = {s.get('sensor'): s for s in sensors}
        for sensor in (1, 2):
            data = by_id.get(sensor, {})
            xyz = data.get('mean_delta_xyz', [None]*3)
            rows.append(dict(event=index, time_sec=event['observed_pc_ns']/1e9-origin,
                             sensor=sensor, magnetic_axis=data.get('magnetic_axis'),
                             status=data.get('status', 'unavailable'),
                             physical_direction='uncalibrated', window_sec=data.get('window_sec'),
                             delta_x=xyz[0], delta_y=xyz[1], delta_z=xyz[2],
                             coherence=data.get('coherence')))
    return rows


def direction_table(rows):
    def display(value):
        return 'Unavailable' if value is None else html.escape(
            f'{value:.3f}' if isinstance(value, float) else str(value))
    fields = ['event', 'time_sec', 'sensor', 'magnetic_axis', 'delta_x', 'delta_y', 'delta_z', 'status']
    body = ''.join('<tr>' + ''.join(f'<td>{display(row[f])}</td>' for f in fields) + '</tr>' for row in rows)
    return ('<section><h2>Magnetic change at confirmed model events</h2>'
            '<p>Sensor-local magnetic XYZ changes over approximately 0.2 seconds at confirmation. '
            'These are NOT calibrated object-motion directions, distances or velocities. '
            'Opposing sensor orientations are not combined. Mixed means magnet changes disagree. '
            'Small changes can be noise; coherence is not model confidence.</p>'
            '<p><a href="gripper_slip_directions.csv">Download direction observations</a></p>'
            '<table><tr><th>Event</th><th>Time (s)</th><th>Sensor</th><th>Magnetic axis</th>'
            '<th>Delta X</th><th>Delta Y</th><th>Delta Z</th><th>Status</th></tr>' + body + '</table>'
            + ('<p>No confirmed slip direction events recorded.</p>' if not rows else '') + '</section>')


def observed_slip_intervals(frames, observations, origin, activity_key='active'):
    points = []
    for frame in frames:
        snapshot = frame.get('slip_observation')
        if isinstance(snapshot, dict) and type(snapshot.get(activity_key)) is bool:
            points.append((snapshot.get('observed_pc_s', frame['observed_pc_ns']/1e9),
                           snapshot[activity_key] and bool(frame.get('contact')),
                           'slip inactive' if frame.get('contact') else 'contact released'))
    for event in observations:
        if event.get('component') != 'gripper_slip_observation':
            continue
        transition = event.get('transition')
        if transition in ('started', 'ended', 'interrupted'):
            if activity_key != 'active' and type(event.get(activity_key)) is not bool:
                continue
            points.append((event.get('observed_pc_s', event['observed_pc_ns']/1e9),
                           event.get(activity_key, transition == 'started'),
                           'slip ended' if transition == 'ended' else 'observation interrupted'))
    intervals, start, last = [], None, None
    points = [p for p in points if isinstance(p[0], (int, float)) and math.isfinite(p[0])]
    for now, active, reason in sorted(points, key=lambda p: p[0]):
        if start is not None and now-last > .35:
            intervals.append(dict(start_sec=start-origin, end_sec=last-origin,
                                  end_reason='observation gap; end unknown'))
            start = None
        if active and start is None:
            start = now
        elif not active and start is not None:
            intervals.append(dict(start_sec=start-origin, end_sec=now-origin, end_reason=reason))
            start = None
        last = now
    if start is not None:
        intervals.append(dict(start_sec=start-origin, end_sec=last-origin, end_reason='end not observed'))
    return intervals


def model_high_intervals(frames, observations, origin):
    """Only explicit v2 high-score evidence; old records are not relabelled."""
    rows = observed_slip_intervals(frames, observations, origin, 'high_evidence')
    for row in rows:
        if row['end_reason'] in ('slip inactive', 'slip ended'):
            row['end_reason'] = 'high-score evidence ended; physical slip unverified'
    return rows


def slip_intervals(frames, origin, observations=()):
    """Observed model episodes, not ground-truth slip or motor permission.

    Use live activity transitions when available. Legacy score-only records
    enter at >= .8 and end after three distinct <= .3 samples. Never bridge
    missing telemetry or a released contact.
    """
    # Prefer the live observer, including every start/end transition. Its history
    # counter and reinforcement attempts are not current slip activity.
    if (any(isinstance(f.get('slip_observation'), dict) for f in frames)
            or any(f.get('component') == 'gripper_slip_observation' for f in observations)):
        return observed_slip_intervals(frames, observations, origin)
    intervals = []
    active = None
    last = None
    epoch = None
    tokens = None
    low = 0
    low_start = None
    for frame in sorted(frames, key=lambda f: f['observed_pc_ns']):
        now = frame['observed_pc_ns'] / 1e9
        prediction = frame.get('slip_model_prediction')
        valid = (isinstance(prediction, (list, tuple)) and len(prediction) == 3
                 and len(prediction[1]) == len(prediction[2]) == 2
                 and all(isinstance(t, (int, float)) and math.isfinite(t)
                         and 0 <= now - t <= .25 for t in prediction[1])
                 and all(isinstance(s, (int, float)) and math.isfinite(s)
                         and 0 <= s <= 1 for s in prediction[2]))
        gap = last is not None and now - last > .35
        changed = valid and epoch is not None and epoch != prediction[0]
        if active is not None and (not valid or gap or changed or not frame.get('contact')):
            intervals.append(dict(start_sec=active-origin, end_sec=last-origin,
                                  end_reason='contact released' if not frame.get('contact') else 'observation interrupted'))
            active = None
            low = 0
        if not valid or not frame.get('contact'):
            tokens = None
            continue
        if changed or gap:
            tokens = None
        epoch, stamps, scores = prediction
        fresh = tokens is None or all(t > old for t, old in zip(stamps, tokens))
        if not fresh:
            continue
        tokens = stamps
        last = now
        contacts = frame.get('slip_contact_mask', [True, True])
        if active is None and any(s >= .8 and c for s, c in zip(scores, contacts)):
            active = now
            low = 0
        if active is not None:
            if all(s <= .3 for s in scores):
                if low == 0:
                    low_start = now
                low += 1
                if low >= 3:
                    intervals.append(dict(start_sec=active-origin, end_sec=low_start-origin,
                                          end_reason='model cleared'))
                    active = None
                    low = 0
            else:
                low = 0
    if active is not None:
        intervals.append(dict(start_sec=active-origin, end_sec=last-origin,
                              end_reason='end not observed'))
    return intervals


def episodes(diagnostics, commands, origin):
    result = []
    for event in sorted((f for f in diagnostics
                         if f.get('component') == 'gripper_regrasp_event'),
                        key=lambda f: f['observed_pc_ns']):
        key = event['episode_id']
        moves = sorted((f for f in commands if f.get('regrasp')
                        and f.get('episode_id') == key), key=lambda f: f['send_start_ns'])
        positions = sorted((f for f in diagnostics
                            if f.get('component') == 'gripper_regrasp_position'
                            and f.get('episode_id') == key),
                           key=lambda f: f['observed_pc_ns'])
        row = dict(episode_id=key, trigger=event['trigger'],
                   reinforcement_reason=event.get('reinforcement_reason', 'new_slip'),
                   time_sec=event['observed_pc_ns'] / 1e9 - origin,
                   command_times_sec=[f['send_start_ns'] / 1e9 - origin for f in moves],
                   command_stages=len(moves))
        for index in range(2):
            motor = index + 1
            samples = [f for f in positions if f['servo_id'] == motor]
            row[f'id{motor}_commanded_closing_raw'] = (
                event['closing_directions'][index] *
                (moves[-1]['target_raw'][index] - event['previous_target_raw'][index])
                if moves else None)
            row[f'id{motor}_observed_closing_raw'] = (
                samples[-1]['closing_delta_raw'] if samples else None)
            row[f'id{motor}_observation_time_sec'] = (
                samples[-1]['observed_pc_ns'] / 1e9 - origin if samples else None)
            row[f'id{motor}_before_raw'] = samples[-1].get('before_raw') if samples else None
            row[f'id{motor}_after_raw'] = samples[-1].get('actual_raw') if samples else None
        row['observation'] = ('not commanded' if not moves else
                              'encoder observations available' if positions else
                              'ACK recorded; displacement unavailable')
        result.append(row)
    return result


def table(rows):
    def value(v):
        return 'Unavailable' if v is None else html.escape(str(v))
    body = []
    def measured(row, motor):
        delta = row[f'id{motor}_observed_closing_raw']
        if delta is None:
            return 'Unavailable'
        before, after = row.get(f'id{motor}_before_raw'), row.get(f'id{motor}_after_raw')
        return (f'{before} -> {after} ({delta:+g} raw)' if before is not None and after is not None
                else f'{delta:+g} raw')
    for index, row in enumerate(rows, 1):
        trigger_label = ('persistent slip recheck' if row.get('reinforcement_reason') == 'persistent_slip'
                         else row['trigger'])
        cells = [index, f"{row['time_sec']:.3f}", trigger_label, row['command_stages'],
                 row['id1_commanded_closing_raw'], measured(row, 1),
                 row['id2_commanded_closing_raw'], measured(row, 2),
                 row['observation']]
        body.append('<tr>' + ''.join(f'<td>{value(v)}</td>' for v in cells) + '</tr>')
    return ('<section><h2>Slip and reinforcement episodes</h2>'
            '<p>Model detections are not independently verified object slip. Blue: reinforcement command. Each episode can contain multiple command stages. '
            'Commanded amounts are final goal minus pre-episode goal. Observed amounts are last sampled encoder '
            'position minus pre-episode measured position, positive toward closing. Units: raw counts, not mm or force. '
            'Missing feedback is not zero movement. Encoder observations alone do not certify successful reinforcement.</p>'
            '<p><a href="gripper_reinforcement.csv">Download episode measurements</a></p>'
            '<table><tr><th>Event</th><th>Time (s)</th><th>Trigger</th><th>Stages</th>'
            '<th>ID1 commanded</th><th>ID1 observed</th><th>ID2 commanded</th><th>ID2 observed</th>'
            '<th>Evidence</th></tr>' + ''.join(body) + '</table>'
            + ('<p>No recorded episode events. Older recordings may not include this telemetry.</p>' if not rows else '')
            + '</section>')
