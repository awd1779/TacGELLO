#!/usr/bin/env python3
"""Read-only RTDE force recording. Never commands motion, tare or motor output."""
import argparse
import csv
from datetime import datetime
import html
import json
import math
from pathlib import Path
import statistics
import time


def percentile(values, fraction):
    values = sorted(values)
    position = (len(values) - 1) * fraction
    lo = int(position)
    hi = min(lo + 1, len(values) - 1)
    return values[lo] + (values[hi] - values[lo]) * (position - lo)


def stats(values):
    if not values:
        return None
    median = statistics.median(values)
    return dict(count=len(values), mean=statistics.mean(values), median=median,
                minimum=min(values), maximum=max(values), spread=max(values)-min(values),
                std=statistics.pstdev(values), p05=percentile(values, .05),
                p95=percentile(values, .95), mad=statistics.median(abs(v-median) for v in values))


def summarize(rows):
    stable = [r for r in rows if r['stationary']]
    buckets = {}
    for row in rows:
        buckets.setdefault(int(row['time_sec']), []).append(row)
    blocks = []
    for second, group in sorted(buckets.items()):
        gaps = [b['time_sec']-a['time_sec'] for a, b in zip(group, group[1:])]
        valid = (len(group) >= 10 and group[-1]['time_sec']-group[0]['time_sec'] >= .7
                 and all(r['stationary'] for r in group) and max(gaps, default=1) <= .15)
        blocks.append(dict(start_sec=second, valid=valid, samples=len(group),
                           mean_n=statistics.mean(r['down_force_n'] for r in group) if valid else None))
    valid = [b for b in blocks if b['valid']]
    means = [b['mean_n'] for b in valid]
    slope = None
    if len(valid) >= 2:
        xs = [b['start_sec'] for b in valid]
        xm, ym = statistics.mean(xs), statistics.mean(means)
        slope = sum((x-xm)*(y-ym) for x, y in zip(xs, means)) / sum((x-xm)**2 for x in xs)
    representative = lambda key: [statistics.median(r[f'{key}{i}'] for r in stable) for i in range(6)] if stable else None
    return dict(samples=len(rows), stationary_samples=len(stable),
                raw_down_force_n=stats([r['down_force_n'] for r in rows]),
                stationary_down_force_n=stats([r['down_force_n'] for r in stable]),
                block_means_n=stats(means), blocks=blocks, drift_n_per_sec=slope,
                stationary_fraction=len(stable)/len(rows) if rows else 0,
                sufficient_data=len(valid) >= 10,
                joint_current_a=[stats([r[f'current{i}'] for r in stable if f'current{i}' in r]) for i in range(6)],
                median_tcp_pose=representative('pose'), median_joint_q=representative('q'))


def compare(empty, loaded, expected_kg):
    warnings = []
    for label, report in [('empty', empty), ('loaded', loaded)]:
        if not report['analysis']['sufficient_data'] or not report.get('complete'):
            warnings.append(label + ': incomplete capture or fewer than ten valid 1s blocks')
    if empty.get('phase') != 'empty' or loaded.get('phase') != 'bottle':
        warnings.append('Need an empty recording followed by a bottle recording')
    if empty.get('host') != loaded.get('host'):
        warnings.append('Robot hosts differ')
    a, b = empty['analysis'], loaded['analysis']
    delta = None
    if a['block_means_n'] and b['block_means_n']:
        delta = b['block_means_n']['mean'] - a['block_means_n']['mean']
        if delta <= 0:
            warnings.append('Nonpositive load change: check force sign, support and payload settings')
        variability = max(a['block_means_n']['spread'], b['block_means_n']['spread'])
        if abs(delta) <= variability:
            warnings.append('Load difference does not exceed within-recording block-mean spread')
        if abs(delta - expected_kg*9.80665) > expected_kg*9.80665*.5:
            warnings.append('Measured change differs from nominal bottle force by more than 50%')
    if a['median_tcp_pose'] and b['median_tcp_pose']:
        position_gap = math.dist(a['median_tcp_pose'][:3], b['median_tcp_pose'][:3])
        joints_gap = max(abs(math.atan2(math.sin(x-y), math.cos(x-y)))
                         for x, y in zip(a['median_joint_q'], b['median_joint_q']))
        if position_gap > .05 or joints_gap > .1:
            warnings.append('Poses differ: bias may be posture-dependent; repeat near the same pose')
    return dict(delta_force_n=delta, apparent_mass_delta_kg=delta/9.80665 if delta is not None else None,
                expected_kg=expected_kg, expected_force_n=expected_kg*9.80665, warnings=warnings,
                note='Diagnostic comparison only. Not approval to enable resistance or a calibrated scale.')


def disabled_status(status, received, now, phase):
    if status is None or received is None or not 0 <= now-received <= .75:
        return 'No fresh /tacgello/lift_weight heartbeat'
    source_ns = status.get('observed_pc_ns')
    if not isinstance(source_ns, (int, float)) or not math.isfinite(source_ns) or not 0 <= now-source_ns/1e9 <= .75:
        return 'Stale or invalid source timestamp on /tacgello/lift_weight'
    if status.get('resistance_enabled') is not False or status.get('resistance_active') is not False:
        return 'Body resistance must be disabled; this recorder will NOT disable it for you'
    if status.get('resistance_fault'):
        return 'Resistance fault; physical release must be checked'
    if status.get('contact') is not (phase == 'bottle'):
        return 'Contact state does not match the requested phase'
    return None


def make_row(force, speed, pose, joints, stamp, now, origin, current=None, joint_speed=None, target_moment=None):
    vectors = [force, speed, pose, joints]
    if any(len(v) != 6 for v in vectors) or not all(math.isfinite(x) for v in vectors for x in v):
        raise ValueError('Invalid RTDE vectors')
    if not all(math.isfinite(x) for x in (stamp, now, origin)):
        raise ValueError('Invalid timestamp')
    linear, angular = math.sqrt(sum(x*x for x in speed[:3])), math.sqrt(sum(x*x for x in speed[3:]))
    row = dict(time_sec=now-origin, pc_monotonic_s=now, robot_stamp_s=stamp,
               down_force_n=-float(force[2]), linear_speed_m_s=linear,
               angular_speed_rad_s=angular, stationary=linear <= .003 and angular <= .05)
    for name, values in zip(['force', 'speed', 'pose', 'q'], vectors):
        row.update({f'{name}{i}': float(value) for i, value in enumerate(values)})
    for name, values in [('current', current), ('qd', joint_speed), ('target_moment', target_moment)]:
        if values is None:
            continue
        if len(values) != 6 or not all(math.isfinite(x) for x in values):
            raise ValueError('Invalid RTDE ' + name)
        row.update({f'{name}{i}': float(value) for i, value in enumerate(values)})
    return row


def save(directory, report, rows):
    directory.mkdir(parents=True, exist_ok=False)
    (directory/'summary.json').write_text(json.dumps(report, indent=2, allow_nan=False), encoding='utf-8')
    with (directory/'raw_force.csv').open('w', newline='', encoding='utf-8') as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]) if rows else ['time_sec', 'down_force_n'])
        writer.writeheader()
        writer.writerows(rows)
    parts = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1000 330">']
    if rows:
        low, high = min(r['down_force_n'] for r in rows)-1, max(r['down_force_n'] for r in rows)+1
        duration = max(1, rows[-1]['time_sec'])
        xy = lambda t, v: (70+t/duration*900, 280-(v-low)/(high-low)*220)
        parts.append(f'<text x="15" y="30">Downward force (-Fz), N: {low:.2f} to {high:.2f}; time 0 to {duration:.1f}s</text>')
        for a, b in zip(rows, rows[1:]):
            if b['time_sec']-a['time_sec'] > .15:
                continue
            x1, y1 = xy(a['time_sec'], a['down_force_n'])
            x2, y2 = xy(b['time_sec'], b['down_force_n'])
            color = '#94a3b8' if a['stationary'] and b['stationary'] else '#dc2626'
            parts.append(f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" stroke="{color}"/>')
        for block in report['analysis']['blocks']:
            if block['valid']:
                x, y = xy(block['start_sec']+.5, block['mean_n'])
                parts.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="4" fill="#0f766e"/>')
    parts.append('</svg>')
    body = ('<!doctype html><html><meta charset="utf-8"><title>Lift force diagnostic</title>'
            '<style>body{font:16px sans-serif;margin:24px}svg{width:100%;max-width:1100px}pre{white-space:pre-wrap}</style>'
            '<h1>Read-only lift force diagnostic</h1><p>Grey: raw stationary samples. Red: moving samples. '
            'Teal: valid 1s means. Not measured object mass.</p>' + ''.join(parts)
            + '<pre>' + html.escape(json.dumps(report, indent=2)) + '</pre></html>')
    (directory/'report.html').write_text(body, encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='192.168.50.14')
    parser.add_argument('--phase', choices=['empty', 'bottle'], required=True)
    parser.add_argument('--seconds', type=float, default=20)
    parser.add_argument('--empty-report', type=Path)
    parser.add_argument('--expected-kg', type=float, default=.6)
    parser.add_argument('--session', default='', help='Shared identifier for one unchanged robot/tool setup')
    parser.add_argument('--role', choices=['calibration', 'validation'], default='calibration')
    parser.add_argument('--output', type=Path, default=Path(__file__).parent/'lift_force_diagnostics')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 12 <= args.seconds <= 120:
        parser.error('--seconds must be between 12 and 120')
    if not math.isfinite(args.expected_kg) or not 0 < args.expected_kg <= 3.0:
        parser.error('--expected-kg must be positive and <= 3.0 (measurement label, not payload approval)')
    empty = None
    if args.empty_report:
        try:
            empty = json.loads(args.empty_report.read_text())
            if empty.get('schema') != 'lift_force_diagnostic_v1' or empty.get('phase') != 'empty':
                raise ValueError('Use summary.json from an empty-tool diagnostic')
        except (OSError, ValueError, AttributeError) as exc:
            parser.error(str(exc))
    import rclpy
    import rtde_receive
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node('lift_force_readonly_recorder')
    status, received = None, None

    def feedback(msg):
        nonlocal status, received
        try:
            value = json.loads(msg.data)
            status = value if isinstance(value, dict) else None
            received = time.monotonic()
        except ValueError:
            status = None

    node.create_subscription(String, '/tacgello/lift_weight', feedback, 10)
    rows, error, complete, receive = [], None, False, None
    directory = args.output / (datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'_'+args.phase)
    try:
        print('Read-only: no robot/servo commands. Keep tool stationary and clear of supports. Body resistance must be OFF.', flush=True)
        deadline = time.monotonic()+5
        while received is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        reason = disabled_status(status, received, time.monotonic(), args.phase)
        if reason:
            raise RuntimeError(reason)
        receive = rtde_receive.RTDEReceiveInterface(args.host)
        origin, last_stamp, last_fresh, next_progress = time.monotonic(), None, time.monotonic(), 0
        print(f'Recording {args.phase}: {args.seconds:g}s', flush=True)
        while time.monotonic()-origin < args.seconds:
            rclpy.spin_once(node, timeout_sec=0)
            now = time.monotonic()
            reason = disabled_status(status, received, now, args.phase)
            if reason:
                raise RuntimeError(reason)
            if not receive.isConnected():
                raise RuntimeError('RTDE disconnected')
            stamp = receive.getTimestamp()
            force, speed = receive.getActualTCPForce(), receive.getActualTCPSpeed()
            pose, joints = receive.getActualTCPPose(), receive.getActualQ()
            current, joint_speed = receive.getActualCurrent(), receive.getActualQd()
            target_moment = receive.getTargetMoment()
            after = receive.getTimestamp()
            if last_stamp is not None and stamp < last_stamp:
                raise RuntimeError('Robot timestamp reset')
            if stamp == after and (last_stamp is None or stamp > last_stamp):
                row = make_row(force, speed, pose, joints, stamp, now, origin,
                               current, joint_speed, target_moment)
                row.update(contact=status['contact'], status_age_sec=now-received,
                           resistance_enabled=False, resistance_active=False)
                rows.append(row)
                last_stamp, last_fresh = stamp, now
            if now-last_fresh > .5:
                raise RuntimeError('No fresh consistent RTDE samples for 0.5s')
            if now-origin >= next_progress:
                print(f'{now-origin:.1f}s: {len(rows)} unique samples', flush=True)
                next_progress += 5
            time.sleep(.008)
        complete = True
    except KeyboardInterrupt:
        error = 'Interrupted; partial recording only'
    except Exception as exc:
        error = str(exc)
    finally:
        if receive is not None:
            try:
                receive.disconnect()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()
    report = dict(schema='lift_force_diagnostic_v1', phase=args.phase, host=args.host,
                  session=args.session, role=args.role,
                  expected_kg=args.expected_kg if args.phase == 'bottle' else 0.0,
                  complete=complete, error=error, requested_seconds=args.seconds,
                  analysis=summarize(rows), last_status=status,
                  note='No motion, tare, payload, safety or output parameters changed.')
    if empty:
        report['comparison'] = compare(empty, report, args.expected_kg)
        report['empty_report'] = str(args.empty_report.resolve())
    save(directory, report, rows)
    print(json.dumps({k: v for k, v in report['analysis'].items()
                      if k not in ('blocks', 'median_tcp_pose', 'median_joint_q')}, indent=2))
    if 'comparison' in report:
        print(json.dumps(report['comparison'], indent=2))
    print(f'Report: {directory.resolve() / "report.html"}', flush=True)
    if error:
        print('Capture failed: '+error, flush=True)
    return 0 if complete and report['analysis']['sufficient_data'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
