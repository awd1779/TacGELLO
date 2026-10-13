import argparse
from pathlib import Path
import signal
import unittest
from unittest.mock import Mock, patch

import record_camera


class RecordingSessionTests(unittest.TestCase):
    @patch('record_camera.subprocess.run')
    def test_50hz_and_10ms_exposure(self, run):
        run.side_effect = [Mock(returncode=0, stdout='auto_exposure exposure_time_absolute power_line_frequency'),
                           Mock(returncode=0), Mock(returncode=0, stdout='readback')]
        record_camera.configure_camera_timing('/dev/video0', 100, 50)
        settings = run.call_args_list[1].args[0][-1]
        self.assertIn('power_line_frequency=1', settings)
        self.assertIn('exposure_time_absolute=100', settings)

    @patch('record_camera.subprocess.run')
    def test_dynamic_framerate_disabled_when_supported(self, run):
        run.side_effect = [Mock(returncode=0, stdout='exposure_dynamic_framerate'),
                           Mock(returncode=0), Mock(returncode=0, stdout='exposure_dynamic_framerate: 0')]
        record_camera.configure_camera_timing('/dev/video0')
        self.assertIn('--set-ctrl=exposure_dynamic_framerate=0', run.call_args_list[1].args[0])
        self.assertEqual(run.call_count, 3)

    @patch('record_camera.subprocess.run')
    def test_c960_modern_exposure_names(self, run):
        run.side_effect = [Mock(returncode=0, stdout='auto_exposure exposure_time_absolute power_line_frequency exposure_dynamic_framerate'),
                           Mock(returncode=0), Mock(returncode=0, stdout='readback')]
        record_camera.configure_camera_timing('/dev/video0')
        settings = run.call_args_list[1].args[0][-1]
        self.assertIn('auto_exposure=1', settings)
        self.assertIn('exposure_time_absolute=100', settings)
        self.assertIn('power_line_frequency=1', settings)

    @patch('record_camera.subprocess.Popen')
    @patch.dict('record_camera.os.environ', {}, clear=True)
    def test_child_middleware_without_global_change(self, popen):
        record_camera.start(['test'])
        self.assertEqual(popen.call_args.kwargs['env']['RMW_IMPLEMENTATION'], 'rmw_cyclonedds_cpp')
        self.assertNotIn('RMW_IMPLEMENTATION', record_camera.os.environ)

    @patch('record_camera.subprocess.run')
    def test_unknown_camera_controls_are_not_written(self, run):
        run.return_value = Mock(returncode=0, stdout='brightness')
        record_camera.configure_camera_timing('/dev/video0')
        self.assertEqual(run.call_count, 1)

    def test_commands_keep_robot_control_separate_and_save_private(self):
        args = argparse.Namespace(device="/dev/video0", root=Path("/tmp/my dataset"), task="pick object")
        camera, recorder = record_camera.commands(args, "/test_session/save_episode")
        self.assertIn("video_device:=/dev/video0", camera)
        self.assertIn("framerate:=30.0", camera)
        self.assertIn("exposure:=100", camera)
        self.assertIn("autoexposure:=false", camera)
        self.assertIn("fps:=30", recorder)
        self.assertIn("root:=/tmp/my dataset", recorder)
        self.assertIn("save_episode:=/test_session/save_episode", recorder)
        self.assertIn("start_paused:=true", recorder)
        self.assertIn("start_or_save_episode:=/test_session/save_episode_start_or_save", recorder)
        self.assertIn("save_partial_episode_on_shutdown:=false", recorder)
        self.assertNotIn("leader_publisher", recorder)
        self.assertNotIn("bluetooth_gripper_bridge", recorder)

    @patch("record_camera.os.killpg")
    def test_stop_only_owned_live_process_group(self, killpg):
        process = Mock(pid=123)
        process.poll.return_value = None
        record_camera.stop(process)
        killpg.assert_called_once_with(123, signal.SIGINT)
        process.wait.assert_called_once_with(timeout=10)

    @patch("record_camera.os.killpg")
    def test_finished_process_not_signalled(self, killpg):
        process = Mock()
        process.poll.return_value = 0
        record_camera.stop(process)
        record_camera.stop(None)
        killpg.assert_not_called()


if __name__ == "__main__":
    unittest.main()
