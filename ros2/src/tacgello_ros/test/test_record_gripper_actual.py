from pathlib import Path
import json
import sys
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import pyarrow.parquet as pq

from tacgello_ros.lerobot_demo_recorder import LeRobotDemoRecorder


def recorder_stub():
    recorder = LeRobotDemoRecorder.__new__(LeRobotDemoRecorder)
    recorder.latest_gripper_actual = {}
    recorder.recording_started = True
    recorder.manual_save_pending = False
    recorder.image_sequence = 0
    recorder.recorded_image_sequence = 0
    recorder.episode_capture_times = []
    recorder.episode_source_stamps = []
    recorder.latest_image_received = 10.0
    recorder.latest_image_stamp = 10000000000
    recorder.next_record_time = None
    return recorder


def test_different_fps_starts_new_dataset(tmp_path):
    recorder = recorder_stub()
    recorder.robot_type = 'ur3'
    recorder.fps = 30
    (tmp_path / 'meta').mkdir()
    (tmp_path / 'meta' / 'info.json').write_text(json.dumps(dict(robot_type='ur3', fps=10)))
    recorder.next_dataset_root = Mock(return_value=tmp_path / 'new')
    assert recorder.resolve_dataset_root(tmp_path) == tmp_path / 'new'


def test_camera_rate_reports_slow_input_not_video_setting():
    for rate in (5, 30):
        recorder = recorder_stub()
        recorder.fps = 30
        recorder.image_height = recorder.image_width = 2
        recorder.image_msg_to_rgb = Mock(return_value=np.zeros((2, 2, 3)))
        recorder.get_logger = Mock(return_value=Mock())
        recorder.record_frame = Mock()
        msg = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)))
        with patch('tacgello_ros.lerobot_demo_recorder.time.monotonic',
                   side_effect=[i / rate for i in range(5 * rate + 1)]):
            for _ in range(5 * rate + 1):
                recorder.image_callback(msg)
        logger = recorder.get_logger.return_value
        if rate == 5:
            assert '5.0 FPS' in logger.warn.call_args.args[0]
            assert 'timing report' in logger.warn.call_args.args[0]
        else:
            assert '30.0 FPS' in logger.info.call_args.args[0]
            logger.warn.assert_not_called()


def test_initial_wait_then_start_and_subsequent_save():
    recorder = recorder_stub()
    recorder.recording_started = False
    # Paused frames must not even access the not-yet-initialized inputs.
    recorder.record_frame()
    recorder.latest_robot_joints = np.zeros(6)
    recorder.latest_leader_joints = np.zeros(6)
    recorder.latest_image = None
    response = SimpleNamespace()
    recorder.start_or_save_episode_service(None, response)
    assert not response.success
    assert not recorder.recording_started
    recorder.latest_image = np.zeros((2, 2, 3))
    recorder.clear_episode_buffers = Mock()
    recorder.record_frame = Mock()
    recorder.get_logger = Mock(return_value=Mock())
    recorder.episode_states = [None] * 5
    recorder.min_episode_frames = 5
    recorder.save_episode = Mock(return_value=True)
    recorder.start_or_save_episode_service(None, response)
    assert response.success
    assert recorder.recording_started
    recorder.clear_episode_buffers.assert_called_once()
    recorder.record_frame.assert_not_called()
    recorder.save_episode.assert_not_called()
    recorder.start_or_save_episode_service(None, response)
    recorder.save_episode.assert_called_once_with()
    assert not recorder.recording_started
    recorder.start_or_save_episode_service(None, response)
    assert recorder.recording_started
    assert recorder.record_frame.call_count == 0


def test_failed_save_retained_and_retried_while_paused():
    recorder = recorder_stub()
    recorder.episode_states = [None] * 5
    recorder.min_episode_frames = 5
    recorder.get_logger = Mock(return_value=Mock())
    recorder.clear_episode_buffers = Mock()
    recorder.save_episode = Mock(side_effect=[False, True])
    response = SimpleNamespace()
    recorder.start_or_save_episode_service(None, response)
    assert not response.success and recorder.manual_save_pending
    assert not recorder.recording_started
    recorder.record_frame()  # No image or state accesses while paused.
    recorder.start_or_save_episode_service(None, response)
    assert response.success and not recorder.manual_save_pending
    assert not recorder.recording_started
    recorder.clear_episode_buffers.assert_not_called()


def test_short_episode_stops_with_explicit_discard():
    recorder = recorder_stub()
    recorder.episode_states = [None]
    recorder.min_episode_frames = 5
    recorder.clear_episode_buffers = Mock()
    recorder.get_logger = Mock(return_value=Mock())
    response = SimpleNamespace()
    recorder.start_or_save_episode_service(None, response)
    assert not recorder.recording_started
    assert not response.success
    assert "too short" in response.message
    recorder.clear_episode_buffers.assert_called_once()


def test_actual_position_pair_and_expiry():
    recorder = recorder_stub()
    with patch("time.monotonic", return_value=10.0):
        recorder.gripper_actual_callback(SimpleNamespace(data=[2, 2243]))
        recorder.gripper_actual_callback(SimpleNamespace(data=[1, 1755]))
        assert recorder.gripper_actual_snapshot() == ([1755, 2243], [0, 0], [1, 1])
    with patch("time.monotonic", return_value=11.0):
        assert recorder.gripper_actual_snapshot() == ([1755, 2243], [1, 1], [0, 0])


def test_invalid_and_missing_measurements():
    recorder = recorder_stub()
    for data in ([1], [1, float("nan")], [3, 2000], [1.5, 2000], [1, -1], [1, 4096], [1, 2.5]):
        recorder.gripper_actual_callback(SimpleNamespace(data=data))
    assert recorder.gripper_actual_snapshot() == ([-1, -1], [-1, -1], [0, 0])
    recorder.gripper_actual_callback(SimpleNamespace(data=[2, 2100]))
    assert recorder.gripper_actual_snapshot()[2] == [0, 1]


def test_record_and_parquet_round_trip(tmp_path, monkeypatch):
    # Other tests add the Windows venv for pure-Python helpers. Arrow must use
    # native Linux pandas when its optional pandas integration loads lazily.
    monkeypatch.setattr(sys, "path", [p for p in sys.path if "/Lib/site-packages" not in p])
    recorder = recorder_stub()
    recorder.latest_robot_joints = np.zeros(6)
    recorder.latest_leader_joints = np.ones(6)
    recorder.latest_gripper_width = 0.07
    recorder.latest_image = np.zeros((2, 2, 3), dtype=np.uint8)
    recorder.episode_states = []
    recorder.episode_actions = []
    recorder.episode_gripper_actual = []
    recorder.episode_images = []
    recorder.episode_timestamps = []
    recorder.fps = 10
    recorder.root = Path(tmp_path)
    recorder.image_feature_key = "observation.images.main"
    with patch("time.monotonic", return_value=10):
        recorder.gripper_actual_callback(SimpleNamespace(data=[1, 1730]))
        recorder.gripper_actual_callback(SimpleNamespace(data=[2, 2250]))
        recorder.image_sequence += 1
        recorder.record_frame()
    with patch("time.monotonic", return_value=11):
        recorder.latest_image_received = 11.0
        recorder.image_sequence += 1
        recorder.record_frame()
    table = recorder.build_episode_table(0, 0)
    path = tmp_path / "test.parquet"
    pq.write_table(table, path)
    saved = pq.read_table(path)
    assert saved["observation.gripper_actual_raw"].to_pylist() == [[1730, 2250]] * 2
    assert saved["observation.gripper_actual_valid"].to_pylist() == [[1, 1], [0, 0]]
    assert saved["observation.gripper_actual_age_s"].to_pylist() == [[0, 0], [1, 1]]
    assert np.isclose(saved["action"][0].as_py()[-1], 0.07)
    recorder.clear_episode_buffers()
    assert not recorder.episode_gripper_actual
