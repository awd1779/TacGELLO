#!/usr/bin/env python3
"""Isolated camera/recorder benchmark. No robot command publishers."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time

import numpy as np
import rclpy

from record_camera import commands, configure_camera_timing, stop
from tacgello_ros.lerobot_demo_recorder import LeRobotDemoRecorder


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=15)
    parser.add_argument('--exposure', type=int, default=100)
    parser.add_argument('--power-line-frequency', type=int, choices=(0, 50, 60), default=50)
    parser.add_argument('--trace', action='store_true')
    parser.add_argument('--check-state-link', action='store_true')
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = '87'
    os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    if subprocess.run(['fuser', '/dev/video0'], capture_output=True).returncode != 1:
        parser.error('Camera is in use or ownership could not be checked')
    configure_camera_timing('/dev/video0', args.exposure, args.power_line_frequency)
    root = Path(__file__).resolve().parent / 'camera_diagnostics' / str(time.time_ns())
    root.mkdir(parents=True)
    camera_cmd, _ = commands(argparse.Namespace(device='/dev/video0', root=root, task='diagnostic', exposure=args.exposure), '/probe')
    if args.trace:
        camera_cmd = ['strace', '-f', '-tt', '-e', 'trace=ioctl', '-o', str(root / 'ioctl.log')] + camera_cmd
    camera = None
    recorder = None
    state_publisher = None
    try:
        with (root / 'camera.log').open('w') as log:
            camera = subprocess.Popen(camera_cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            rclpy.init(args=['--ros-args', '-p', f'root:={root / "dataset"}',
                             '-p', 'fps:=30', '-p', 'start_paused:=true'])
            recorder = LeRobotDemoRecorder()
            state_count = [0]
            if args.check_state_link:
                from sensor_msgs.msg import JointState
                def state_callback(message):
                    state_count[0] += 1
                recorder.create_subscription(JointState, '/camera_diagnostic_joint_states', state_callback, 10)
                state_env = os.environ.copy()
                state_env['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
                state_publisher = subprocess.Popen(['ros2', 'topic', 'pub', '--rate', '30',
                    '/camera_diagnostic_joint_states', 'sensor_msgs/msg/JointState',
                    '{name: [test_joint], position: [0.0]}'], env=state_env,
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            end = time.monotonic() + 40
            while recorder.latest_image is None and time.monotonic() < end:
                rclpy.spin_once(recorder, timeout_sec=.1)
            if recorder.latest_image is None:
                raise RuntimeError('No camera image within 40 seconds')
            end = time.monotonic() + 3
            while time.monotonic() < end:
                rclpy.spin_once(recorder, timeout_sec=.1)
            # Fake state exists only inside this diagnostic recorder, never on ROS topics.
            recorder.latest_robot_joints = np.zeros(6, dtype=np.float32)
            recorder.latest_leader_joints = np.zeros(6, dtype=np.float32)
            recorder.recorded_image_sequence = recorder.image_sequence
            recorder.recording_started = True
            end = time.monotonic() + args.seconds
            while time.monotonic() < end:
                rclpy.spin_once(recorder, timeout_sec=.1)
            recorder.recording_started = False
            result = recorder.camera_timing_quality()
            result['diagnostic_only_synthetic_robot_state'] = True
            result['exposure_100us'] = args.exposure
            result['power_line_frequency_hz'] = args.power_line_frequency
            if args.check_state_link:
                result['fastdds_state_messages_received'] = state_count[0]
            result['saved'] = recorder.save_episode()
            (root / 'result.json').write_text(json.dumps(result, indent=2))
            print(json.dumps(result, indent=2), flush=True)
            print(root, flush=True)
    finally:
        stop(camera)
        stop(state_publisher)
        if recorder is not None:
            recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
