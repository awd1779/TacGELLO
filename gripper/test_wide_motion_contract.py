"""Firmware/PC range contract without motor I/O."""
import ast
import json
from pathlib import Path
import re
from unittest.mock import Mock

import pytest
import bluetooth_anyskin_gripper_shell as control
import manual_gripper_shell as manual
from gripper_position_tracking import PositionTracker


ROOT = Path(__file__).parent
OPEN = (2080, 1920)
CLOSE = (1520, 2480)


def test_firmware_config_and_fallbacks_agree():
    config = json.loads((ROOT/'gripper_motion_ranges.json').read_text())
    firmware = (ROOT/'esp32_bluetooth/esp32_bluetooth.ino').read_text()
    assert control.open_raws(config) == control.open_raws({}) == OPEN
    assert control.close_raws(config) == CLOSE
    assert manual.preset_raws('open', [1, 2], {}) == list(OPEN)
    for i in (1, 2):
        assert int(re.search(rf'kServo{i}OpenRaw = (\d+)', firmware)[1]) == OPEN[i-1]
        assert int(re.search(rf'kServo{i}ClosedRaw = (\d+)', firmware)[1]) == CLOSE[i-1]


def test_full_travel_mapping_stays_inside_firmware_limits():
    config = dict(open=list(OPEN), close=list(CLOSE))
    for step in range(501):
        ratio = step/500
        target = control.interpolate_raws(config, ratio)
        assert CLOSE[0] <= target[0] <= OPEN[0]
        assert OPEN[1] <= target[1] <= CLOSE[1]
        assert manual.close_ratio_from_raws(target, OPEN, CLOSE) == pytest.approx(ratio, abs=1/560)
        tracker = PositionTracker(OPEN, CLOSE, target, 2919)
        assert tracker.ratio() == pytest.approx(ratio, abs=1/560)
    assert control.interpolate_raws(config, 0) == OPEN
    assert control.interpolate_raws(config, 1) == CLOSE


def test_sync_move_encodes_new_open_without_reversing_jaws():
    client = object.__new__(control.BluetoothGripperClient)
    client.trace_commands = False
    client.drain_lines = Mock()
    client.send = Mock()
    client.wait_for_command_ack = Mock()
    client.sync_move(*OPEN, 100, 10)
    client.send.assert_called_once_with('SYNC_MOVE 1 33 2 -127 100 10')
    assert client.last_raws == OPEN


def test_old_envelope_is_rejected_not_relabelled():
    config = dict(open=list(OPEN), close=list(CLOSE),
                  empty_close_baseline=dict(open=[2000, 2000], close=list(CLOSE)))
    assert not control.baseline_matches_motion(config)
    config['empty_close_baseline']['open'] = list(OPEN)
    assert control.baseline_matches_motion(config)


def test_old_open_cannot_be_accepted_as_measured_full_open():
    tracker = PositionTracker(OPEN, CLOSE, (2000, 2000), 2919)
    tracker.contact = True
    tracker.command(0, 1)
    assert not tracker.release_at_measured_open((2000, 2000), 1, 8)
    assert tracker.release_at_measured_open(OPEN, 1, 8)


def test_no_sensor_launch_retains_eight_raw_step_budget():
    tree = ast.parse((ROOT.parent/'ros2/src/tacgello_ros/launch/'
                     'ur3_tacgello_no_anyskin.launch.py').read_text())
    expressions = [value for node in ast.walk(tree) if isinstance(node, ast.Dict)
                   for key, value in zip(node.keys, node.values)
                   if isinstance(key, ast.Constant) and key.value == 'max_step_per_update']
    assert len(expressions) == 1
    expression = expressions[0]
    assert isinstance(expression, ast.BinOp) and isinstance(expression.op, ast.Div)
    assert ast.literal_eval(expression.right) == max(abs(a-b) for a, b in zip(OPEN, CLOSE))
    assert ast.literal_eval(expression.left.left) == 8
    assert ast.literal_eval(expression.left.right) == 500
