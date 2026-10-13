#!/usr/bin/env python3
"""Offline paired-load calibration. Never loads a model into robot control."""
import argparse
from collections import Counter
from datetime import datetime
import json
import math
from pathlib import Path


def finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


def point(path):
    report = json.loads(path.read_text(encoding='utf-8'))
    if report.get('schema') != 'lift_force_diagnostic_v1' or report.get('phase') != 'bottle':
        raise ValueError(f'{path}: expected bottle diagnostic')
    reference = Path(report['empty_report'])
    if not reference.is_absolute():
        reference = path.parent / reference
    empty = json.loads(reference.read_text(encoding='utf-8'))
    if empty.get('schema') != 'lift_force_diagnostic_v1' or empty.get('phase') != 'empty':
        raise ValueError('Invalid empty reference')
    if not report.get('session') or empty.get('session') != report['session'] or report.get('host') != empty.get('host'):
        raise ValueError('Loaded and empty captures must share an explicit session and robot host')
    for sample in (empty, report):
        a = sample['analysis']
        blocks = a.get('block_means_n')
        if not sample.get('complete') or not a.get('sufficient_data') or not blocks:
            raise ValueError('Incomplete recording or insufficient stable blocks')
        # Qualification criteria, not a replacement for controller protections.
        if (not finite(a.get('drift_n_per_sec')) or abs(a['drift_n_per_sec']) > .1
                or not finite(blocks.get('spread')) or blocks['spread'] > 2
                or not finite(blocks.get('mean'))):
            raise ValueError('Force drift/spread too large for calibration')
    a, b = empty['analysis'], report['analysis']
    vectors = [s.get(k) for s in (a, b) for k in ('median_tcp_pose', 'median_joint_q')]
    if any(not isinstance(v, list) or len(v) != 6 or not all(finite(x) for x in v) for v in vectors):
        raise ValueError('Pose and joint records required')
    if math.dist(a['median_tcp_pose'][:3], b['median_tcp_pose'][:3]) > .02 or any(
            abs(math.atan2(math.sin(x-y), math.cos(x-y))) > .05
            for x, y in zip(a['median_joint_q'], b['median_joint_q'])):
        raise ValueError('Empty and loaded poses differ; repeat at the reference pose')
    kg = report.get('expected_kg')
    delta = b['block_means_n']['mean']-a['block_means_n']['mean']
    if not finite(kg) or not 0 < kg <= 3.0 or delta <= max(a['block_means_n']['spread'], b['block_means_n']['spread']):
        raise ValueError('Known mass missing/out of range or load change below observed variability')
    role = report.get('role')
    if role not in ('calibration', 'validation'):
        raise ValueError('Explicit calibration/validation role required')
    return dict(path=str(path.resolve()), empty_path=str(reference.resolve()),
                session=report['session'], host=report['host'], role=role,
                known_kg=kg, delta_n=delta, reference_pose=a['median_tcp_pose'],
                reference_q=a['median_joint_q'])


def fit(points):
    if len({p['path'] for p in points}) != len(points):
        raise ValueError('Duplicate recordings are not independent repeats')
    if len({(p['session'], p['host']) for p in points}) != 1:
        raise ValueError('Use one unchanged robot/tool session')
    train = [p for p in points if p['role'] == 'calibration']
    test = [p for p in points if p['role'] == 'validation']
    counts = Counter(p['known_kg'] for p in train)
    if len(counts) < 3 or min(counts.values()) < 2:
        raise ValueError('Need at least 3 distinct masses with 2 calibration recordings each')
    anchor = train[0]
    for p in points:
        if math.dist(p['reference_pose'][:3], anchor['reference_pose'][:3]) > .02 or any(
                abs(math.atan2(math.sin(x-y), math.cos(x-y))) > .05
                for x, y in zip(p['reference_q'], anchor['reference_q'])):
            raise ValueError('Reference pose changed across trials')
    means = [(kg, sum(p['delta_n'] for p in train if p['known_kg'] == kg)/count)
             for kg, count in sorted(counts.items())]
    if any(b[1] <= a[1] for a, b in zip(means, means[1:])):
        raise ValueError('Force response is not increasing with known mass')
    # Paired tare removes the intercept. Independent trials, not individual
    # correlated force samples, are the regression observations.
    gain = sum(p['delta_n']*p['known_kg'] for p in train)/sum(p['delta_n']**2 for p in train)
    rows = [dict(p, predicted_kg=gain*p['delta_n'], error_kg=gain*p['delta_n']-p['known_kg']) for p in points]
    validation = [p for p in rows if p['role'] == 'validation']
    coverage = set(counts) <= {p['known_kg'] for p in validation}
    errors = [abs(p['error_kg']) for p in validation]
    return dict(schema='paired_load_candidate_v1', session=anchor['session'], host=anchor['host'],
                kg_per_n=gain, calibrated_range_kg=[min(counts), max(counts)],
                reference_pose=anchor['reference_pose'], reference_q=anchor['reference_q'],
                validation_covers_training_masses=coverage,
                validation_mae_kg=sum(errors)/len(errors) if errors else None,
                validation_max_error_kg=max(errors) if errors else None,
                eligible_for_review=bool(coverage and errors and max(errors) <= .2),
                motor_control_approved=False, trials=rows,
                note='Offline candidate only. No extrapolation, actuator change, payload approval or automatic controller activation. '
                     'Validation error is measured on held-out recordings, not a confidence interval. '
                     'Requires new-session/pose validation and live stable-force qualification.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reports', nargs='+', required=True, type=Path)
    parser.add_argument('--output', type=Path, default=Path(__file__).parent/'lift_force_diagnostics'/
                        (datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'_calibration'))
    args = parser.parse_args()
    try:
        result = fit([point(path) for path in args.reports])
    except (ValueError, KeyError, TypeError, OSError) as exc:
        parser.exit(1, f'Calibration rejected: {exc}\n')
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'candidate.json').write_text(json.dumps(result, indent=2, allow_nan=False), encoding='utf-8')
    print(json.dumps({k: v for k, v in result.items() if k != 'trials'}, indent=2))
    print(f'Report: {args.output.resolve() / "candidate.json"}')


if __name__ == '__main__':
    main()
