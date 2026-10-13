import json
import pytest
from fit_lift_weight import fit, point


def trial(kg, repeat=0, role='calibration'):
    return dict(path=f'{role}-{kg}-{repeat}', session='test', host='robot', role=role,
                known_kg=kg, delta_n=kg*10, reference_pose=[0]*6, reference_q=[0]*6)


def trials():
    return [trial(kg, repeat) for kg in (.6, 1.2, 1.8) for repeat in (0, 1)]


def test_offline_fit_without_validation_never_approves_control():
    result = fit(trials())
    assert result['kg_per_n'] == pytest.approx(.1)
    assert result['validation_mae_kg'] is None
    assert not result['eligible_for_review']
    assert not result['motor_control_approved']


def test_held_out_validation_reports_error_not_training_accuracy():
    validation = [trial(kg, role='validation') for kg in (.6, 1.2, 1.8)]
    validation[0]['delta_n'] += 1
    result = fit(trials()+validation)
    assert result['validation_max_error_kg'] == pytest.approx(.1)
    assert result['eligible_for_review']
    assert not result['motor_control_approved']


def test_bad_validation_does_not_qualify():
    validation = [trial(kg, role='validation') for kg in (.6, 1.2, 1.8)]
    validation[0]['delta_n'] += 4
    assert not fit(trials()+validation)['eligible_for_review']


def test_duplicate_or_mixed_session_rejected():
    rows = trials()
    with pytest.raises(ValueError, match='Duplicate'):
        fit(rows+[rows[0]])
    rows[0]['session'] = 'another'
    with pytest.raises(ValueError, match='session'):
        fit(rows)


def test_pose_change_or_nonmonotonic_load_rejected():
    rows = trials()
    rows[0]['reference_pose'] = [.1, 0, 0, 0, 0, 0]
    with pytest.raises(ValueError, match='pose'):
        fit(rows)
    rows = trials()
    for row in rows:
        if row['known_kg'] == 1.8:
            row['delta_n'] = 1
    with pytest.raises(ValueError, match='increasing'):
        fit(rows)


def test_diagnostic_pair_requires_stable_complete_samples(tmp_path):
    analysis = dict(sufficient_data=True, drift_n_per_sec=0,
                    block_means_n=dict(mean=10, spread=.5),
                    median_tcp_pose=[0]*6, median_joint_q=[0]*6)
    empty = dict(schema='lift_force_diagnostic_v1', phase='empty', complete=True,
                 session='test', host='robot', analysis=analysis)
    path = tmp_path/'empty.json'
    path.write_text(json.dumps(empty))
    loaded = dict(empty, phase='bottle', expected_kg=.6, role='calibration',
                  empty_report=str(path), analysis=dict(analysis, block_means_n=dict(mean=16, spread=.5)))
    target = tmp_path/'loaded.json'
    target.write_text(json.dumps(loaded))
    assert point(target)['delta_n'] == 6
    loaded['analysis']['drift_n_per_sec'] = .3
    target.write_text(json.dumps(loaded))
    with pytest.raises(ValueError, match='drift'):
        point(target)
