import json
import math
from xml.etree import ElementTree

import pytest

from record_lift_force import compare, disabled_status, make_row, save, summarize


def rows(noise=None, offset=0):
    result = []
    for i in range(2000):
        t = i*.01
        f = offset + (noise(t) if noise else 0)
        result.append(make_row([0, 0, -f, 0, 0, 0], [0]*6, [0]*6, [0]*6, t, t, 0))
    return result


def test_raw_noise_vs_block_drift():
    signal = rows(lambda t: 11*math.sin(2*math.pi*5*t))
    result = summarize(signal)
    assert result['raw_down_force_n']['spread'] == pytest.approx(22)
    assert result['block_means_n']['spread'] < .01
    assert result['sufficient_data']
    drift = summarize(rows(lambda t: t))
    assert drift['drift_n_per_sec'] == pytest.approx(1, abs=.01)
    assert drift['block_means_n']['spread'] > 18


def test_joint_channels_preserve_signed_current_and_target_moment():
    row = make_row([0]*6, [0]*6, [0]*6, [0]*6, 1, 1, 0,
                   [-.2]*6, [.01]*6, [2]*6)
    assert row['current0'] == -.2
    assert row['qd5'] == .01
    assert row['target_moment3'] == 2
    assert summarize([row])['joint_current_a'][0]['mean'] == -.2
    assert summarize(rows())['joint_current_a'] == [None]*6
    with pytest.raises(ValueError, match='Invalid RTDE current'):
        make_row([0]*6, [0]*6, [0]*6, [0]*6, 1, 1, 0, [float('nan')]*6)


def test_spike_remains_in_raw_and_moving_blocks_rejected():
    signal = rows()
    signal[50]['down_force_n'] = 100
    assert summarize(signal)['raw_down_force_n']['maximum'] == 100
    signal[55]['stationary'] = False
    result = summarize(signal)
    assert not result['blocks'][0]['valid']
    assert result['stationary_samples'] == 1999


def test_gaps_and_short_clips_not_sufficient():
    signal = rows()
    assert not summarize(signal[:50])['sufficient_data']
    gapped = [r for r in signal if r['time_sec'] % 1 < .2]
    assert not summarize(gapped)['sufficient_data']
    assert summarize([])['raw_down_force_n'] is None


def test_comparison_is_diagnostic_not_permission():
    a = dict(analysis=summarize(rows()), phase='empty', complete=True, host='robot')
    b = dict(analysis=summarize(rows(offset=.6*9.80665)), phase='bottle', complete=True, host='robot')
    result = compare(a, b, .6)
    assert result['apparent_mass_delta_kg'] == pytest.approx(.6)
    assert 'Not approval' in result['note']
    b['analysis']['median_joint_q'][0] = .3
    b['complete'] = False
    result = compare(a, b, .6)
    assert any('Poses differ' in x for x in result['warnings'])
    assert any('incomplete' in x for x in result['warnings'])


def test_capture_gate_requires_off_fresh_matching_contact():
    status = dict(resistance_enabled=False, resistance_active=False, contact=False,
                  resistance_fault='', observed_pc_ns=1000000000)
    assert disabled_status(status, 1, 1.1, 'empty') is None
    assert disabled_status(status, 1, 1.1, 'bottle')
    assert disabled_status(status, 1, 2, 'empty')
    assert disabled_status(dict(status, observed_pc_ns=0), 1, 1.1, 'empty')
    for key, value in [('resistance_enabled', True), ('resistance_active', True), ('resistance_fault', 'fault')]:
        assert disabled_status(dict(status, **{key: value}), 1, 1.1, 'empty')


def test_invalid_vectors_rejected_and_saved_files(tmp_path):
    with pytest.raises(ValueError):
        make_row([0]*5, [0]*6, [0]*6, [0]*6, 0, 0, 0)
    with pytest.raises(ValueError):
        make_row([float('nan')]*6, [0]*6, [0]*6, [0]*6, 0, 0, 0)
    data = rows()
    report = dict(analysis=summarize(data), complete=True)
    directory = tmp_path/'capture'
    save(directory, report, data)
    assert json.loads((directory/'summary.json').read_text())['complete']
    assert len((directory/'raw_force.csv').read_text().splitlines()) == 2001
    page = (directory/'report.html').read_text()
    ElementTree.fromstring(page[page.index('<svg'):page.index('</svg>')+6])
    assert 'Read-only' in page
