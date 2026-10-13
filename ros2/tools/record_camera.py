#!/usr/bin/env python3
"""WSL recording: s starts, s stops/saves, then waits for s. q exits."""

import argparse
from datetime import datetime
import os
import re
from pathlib import Path
import select
import signal
import subprocess
import sys
import termios
import tty
import uuid


def commands(args, service):
    camera = [
        "ros2", "run", "usb_cam", "usb_cam_node_exe", "--ros-args",
        "-r", "__node:=base_rgb_camera", "-p", f"video_device:={args.device}",
        "-p", "image_width:=640", "-p", "image_height:=480",
        "-p", "pixel_format:=mjpeg2rgb", "-p", "framerate:=30.0",
        "-p", "autoexposure:=false", "-p", f"exposure:={getattr(args, 'exposure', 100)}",
        "-p", "brightness:=-1",
        "-r", "/image_raw:=/camera/color/image_raw",
    ]
    recorder = [
        "ros2", "run", "tacgello_ros", "lerobot_demo_recorder",
        "--ros-args", "-p", f"root:={args.root}",
        "-p", "repo_id:=local/ur3_anyskin", "-p", "robot_type:=ur3_tacgello",
        "-p", f"task:={args.task}", "-p", "fps:=30",
        "-p", "camera_topic:=/camera/color/image_raw",
        "-p", "image_width:=640", "-p", "image_height:=480",
        "-p", "robot_joint_topic:=/joint_states", "-p", "leader_joint_topic:=/tacgello/joint_states",
        "-p", "gripper_command_topic:=/tacgello/gripper_width_command",
        "-p", "auto_save_enabled:=false", "-p", "save_partial_episode_on_shutdown:=false",
        "-p", "start_paused:=true",
        "-r", f"save_episode:={service}", "-r", f"save_demo:={service}_topic",
        "-r", f"start_or_save_episode:={service}_start_or_save",
    ]
    return camera, recorder


def start(command):
    environment = os.environ.copy()
    environment.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    return subprocess.Popen(command, stdin=subprocess.DEVNULL, start_new_session=True,
                            env=environment)


def configure_camera_timing(device, exposure=100, power_line_frequency=50):
    # usb_cam 0.8.1 uses legacy control names that this C960 does not expose.
    try:
        controls = subprocess.run(['v4l2-ctl', '-d', device, '--list-ctrls'],
                                  capture_output=True, text=True, timeout=3)
        if controls.returncode != 0:
            print('Camera controls unavailable; settings unchanged.', flush=True)
            return
        settings = []
        for aliases, value in ((('exposure_dynamic_framerate', 'exposure_auto_priority'), 0),
                               (('auto_exposure', 'exposure_auto'), 1),
                               (('exposure_time_absolute', 'exposure_absolute'), exposure),
                               (('power_line_frequency',), {0: 0, 50: 1, 60: 2}[power_line_frequency])):
            name = next((name for name in aliases if re.search(r'\b' + name + r'\b', controls.stdout)), None)
            if name:
                settings.append(f'{name}={value}')
        if not settings:
            print('Camera timing controls unavailable; settings unchanged.', flush=True)
            return
        result = subprocess.run(['v4l2-ctl', '-d', device,
                                 '--set-ctrl=' + ','.join(settings)],
                                capture_output=True, text=True, timeout=3)
        if result.returncode:
            raise RuntimeError('Could not configure camera timing: ' + result.stderr)
        else:
            print('Camera timing: ' + ', '.join(settings) +
                  '. Fixed short exposure needs adequate lighting.', flush=True)
            readback = subprocess.run(['v4l2-ctl', '-d', device,
                '--get-ctrl=' + ','.join(s.split('=')[0] for s in settings)],
                capture_output=True, text=True, timeout=3)
            print(readback.stdout or readback.stderr, flush=True)
    except (OSError, subprocess.TimeoutExpired) as exc:
        print(f'Camera timing configuration unavailable: {exc}', flush=True)


def stop(process):
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument('--exposure', type=int, default=100,
                        help='Manual exposure in 100 us units (default 100 = 10 ms)')
    parser.add_argument('--power-line-frequency', type=int, choices=(0, 50, 60), default=50,
                        help='Anti-flicker frequency in Hz; 0 disables compensation')
    parser.add_argument("--external-camera", action="store_true",
                        help="Use an existing image publisher instead of starting usb_cam")
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parent /
                        "lerobot_demos" / datetime.now().strftime("ur3_anyskin_%Y%m%d_%H%M%S_%f"))
    parser.add_argument("--task", default="grasp the object and test slip recovery")
    args = parser.parse_args()
    if not 1 <= args.exposure <= 300:
        parser.error('--exposure must be 1..300 to stay below the 30 FPS frame period')
    middleware = os.environ.get('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    if subprocess.run(['ros2', 'pkg', 'prefix', middleware], capture_output=True).returncode:
        parser.error(f'Missing ROS middleware {middleware}; install ros-humble-rmw-cyclonedds-cpp')
    if not sys.stdin.isatty():
        parser.error("Run in an interactive WSL terminal.")
    if not args.external_camera:
        if not os.access(args.device, os.R_OK | os.W_OK):
            parser.error(f"Camera unavailable or permission denied: {args.device}")
        check = subprocess.run(["fuser", args.device], capture_output=True)
        if check.returncode == 0:
            parser.error("Camera already in use. Stop its node or use --external-camera.")
        if check.returncode != 1:
            parser.error("Could not check camera ownership with fuser.")
    service = "/record_session_" + uuid.uuid4().hex + "/save_episode"
    camera_cmd, recorder_cmd = commands(args, service)
    camera = recorder = saving = None
    old_terminal = termios.tcgetattr(sys.stdin)
    exit_requested = False
    # Keep SIGINT as an orderly request so an active save can finish first.
    old_handler = signal.getsignal(signal.SIGINT)

    def request_exit(signum, frame):
        nonlocal exit_requested
        exit_requested = True

    signal.signal(signal.SIGINT, request_exit)
    try:
        if not args.external_camera:
            configure_camera_timing(args.device, args.exposure, args.power_line_frequency)
            camera = start(camera_cmd)
        recorder = start(recorder_cmd)
        tty.setcbreak(sys.stdin.fileno())
        print(f"\nDataset: {args.root}\nWAITING: s starts; s again stops/saves. Then wait for s to start again.\nq / Ctrl+C: exit (unsaved frames discarded)", flush=True)
        while True:
            if recorder.poll() is not None or (camera is not None and camera.poll() is not None):
                print("\nCamera or recorder exited. Ending recording session.", flush=True)
                break
            if saving is not None and saving.poll() is not None:
                print("\nRequest finished. Check RECORDING / WAITING / SAVE FAILED response above.", flush=True)
                saving = None
            if exit_requested and saving is None:
                break
            if not select.select([sys.stdin], [], [], 0.1)[0]:
                continue
            key = sys.stdin.read(1).lower()
            if key == "q":
                exit_requested = True
                if saving is not None:
                    print("\nWaiting for the current save before exit.", flush=True)
            elif key == "s" and not exit_requested:
                if saving is not None:
                    print("\nSave already pending; no duplicate sent.", flush=True)
                else:
                    print("\nStart/save requested. Wait for confirmation; keep robot/leader still while saving.", flush=True)
                    saving = start(["ros2", "service", "call", service + "_start_or_save", "std_srvs/srv/Trigger", "{}"])
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_terminal)
        stop(saving)
        stop(recorder)
        stop(camera)
        signal.signal(signal.SIGINT, old_handler)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
