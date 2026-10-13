from unittest.mock import Mock

import numpy as np

from tacgello_ros.lerobot_demo_recorder import LeRobotDemoRecorder


def timing(rate=30, count=300):
    r = LeRobotDemoRecorder.__new__(LeRobotDemoRecorder)
    r.fps = 30
    r.episode_capture_times = (10 + np.arange(count) / rate).tolist()
    r.episode_source_stamps = (np.asarray(r.episode_capture_times) * 1e9).astype(np.int64).tolist()
    return r


def test_normal_rate_accepted():
    assert timing().camera_timing_quality()['accepted']


def test_low_rate_rejected_not_compressed_to_30fps():
    for rate in (5, 15, 25):
        assert not timing(rate).camera_timing_quality()['accepted']


def test_gap_or_repeated_source_stamp_rejected():
    r = timing()
    r.episode_capture_times[50] += .2
    assert not r.camera_timing_quality()['accepted']
    r = timing()
    r.episode_source_stamps[50] = r.episode_source_stamps[49]
    assert not r.camera_timing_quality()['accepted']


def test_timer_cannot_record_duplicates():
    r = timing()
    r.record_frame = Mock()
    r.record_timer_callback()
    r.record_frame.assert_not_called()


def test_consumed_image_cannot_record_again():
    r = timing()
    r.recording_started = True
    r.image_sequence = r.recorded_image_sequence = 2
    r.record_frame()  # Must return before touching robot or image buffers.


def test_faster_camera_selects_30_new_frames_per_second():
    r = timing()
    r.recording_started = True
    r.next_record_time = None
    r.recorded_image_sequence = 0
    r.latest_robot_joints = r.latest_leader_joints = np.zeros(6)
    r.latest_gripper_width = .1
    r.latest_image = np.zeros((2, 2, 3), dtype=np.uint8)
    r.episode_states = []
    r.episode_actions = []
    r.episode_gripper_actual = []
    r.episode_images = []
    r.episode_timestamps = []
    r.episode_capture_times = []
    r.episode_source_stamps = []
    r.gripper_actual_snapshot = Mock(return_value=([], [], []))
    for i in range(330):
        r.image_sequence = i + 1
        r.latest_image_received = 10 + i / 33
        r.latest_image_stamp = int(r.latest_image_received * 1e9)
        r.record_frame()
    assert len(r.episode_images) == 300
    assert len(set(r.episode_source_stamps)) == 300
    assert r.camera_timing_quality()['accepted']


def test_bad_episode_quarantined_without_training_commit(tmp_path):
    r = timing(15, 10)
    r.root = tmp_path
    r.strict_camera_timing = True
    r.saving_episode = False
    r.min_episode_frames = 2
    r.episode_states = [np.zeros(7)] * 10
    r.episode_actions = [np.zeros(7)] * 10
    r.episode_images = [np.zeros((2, 2, 3), dtype=np.uint8)] * 10
    r.episode_gripper_actual = []
    r.episode_timestamps = []
    r.get_logger = Mock(return_value=Mock())
    assert not r.save_episode()
    assert r.episode_quarantined
    assert list(tmp_path.glob('quarantine/*/capture.npz'))
    assert not (tmp_path / 'data').exists()
    assert not r.episode_images
