# TacGELLO

**Tactile Contact Feedback for a 3D-Printed Teleoperation Leader**

Younsoo Kim, Sangmin Song, Sarath Kodagoda, Marc Carmichael, Karthick Thiyagarajan

[![arXiv](https://img.shields.io/badge/arXiv-2610.05583-b31b1b.svg)](https://arxiv.org/abs/2610.05583)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

**Paper:** [https://arxiv.org/abs/2610.05583](https://arxiv.org/abs/2610.05583)

TacGELLO adds two feedback channels to a [GELLO](https://wuphilipp.github.io/gello_site/)-style
3D-printed leader arm. It uses only the leader's existing STS3215 servos, with no extra
actuators or force/torque sensors.

- **Fingertip channel (contact → trigger).** [AnySkin](https://any-skin.github.io/) skins on both
  gripper fingertips detect contact against an empty-gripper baseline. When contact is detected,
  the leader's trigger servo holds its measured position with a capped output limit (150 raw),
  so the operator feels contact onset.
- **Arm channel (load → joints).** The UR3 shoulder, elbow and wrist-1 joint currents are compared
  with an empty-arm baseline. Changes in current adjust the output limits of the matching leader
  servos to give resistance while lifting.

This repository contains the software and firmware used in the paper: a ROS 2 teleoperation package
for a UR3 follower, the host-side controller for the custom two-finger gripper, the ESP32 gripper
firmware, the AnySkin QT Py firmware, and the logging and report tools.

> CAD files for the leader and gripper will be released separately.

## System overview

```
 Leader arm (6 joint + 1 trigger STS3215) ──USB──► Host PC (ROS 2) ──RTDE servoJ──► UR3
        ▲   trigger hold / joint output limits        │   ▲                          │
        └─────────────────────────────────────────────┘   └── joint angles, currents ┘
                                                      │
                     width commands (TCP ↔ Bluetooth) ▼        AnySkin ×2 (USB, QT Py M0)
                                          ESP32 gripper (2× STS3215) ──────► Host PC
```

| Component | Qty |
| --- | --- |
| Leader joint servos (Feetech STS3215) | 6 |
| Leader trigger servo (STS3215) | 1 |
| Gripper servos (STS3215) + ESP32 servo driver (Waveshare 21593) | 2 + 1 |
| AnySkin skins + Adafruit QT Py M0 boards | 2 + 2 |
| UR3 follower (RTDE) | 1 |

## Repository layout

```
TacGELLO/
├── ros2/src/tacgello_ros/     ROS 2 package: leader publisher, feedback cues, bridges, recorders
│   ├── launch/                ur3_tacgello.launch.py (TacGELLO), ur3_tacgello_no_anyskin.launch.py
│   │                          (baseline without tactile/arm feedback), ur3_tacgello_record.launch.py
│   └── config/ur3_tacgello.yaml
├── ros2/tools/                lift-force recording, lift-weight fitting, camera recording
├── gripper/                   host-side gripper controller (AnySkin contact detection, trigger
│   │                          hold signalling, position tracking, diagnostics) + setup tools
│   ├── esp32_bluetooth/       ESP32 gripper firmware and the Windows Bluetooth↔TCP bridge
│   ├── anyskin_qtpy_checked/  AnySkin QT Py M0 firmware (MLX90393, checked reads)
│   └── firmware_tests/        host-side unit test for the MLX90393 driver
├── configs/servo_offsets.json leader servo zero offsets
├── tools/save_leader_offsets.py
├── docs/                      arm-channel (joint current) and recording notes
└── experimental/              slip/regrasp probes not covered by the paper
```

## Installation

Used with ROS 2 Humble under WSL2 on a Windows host. The gripper's Bluetooth link runs through
a small bridge on the Windows side.

```bash
git clone https://github.com/awd1779/TacGELLO.git
cd TacGELLO
pip install -r requirements.txt

cd ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select tacgello_ros
source install/setup.bash
```

Keep the colcon workspace at `TacGELLO/ros2`. The launch files find `gripper/` and
`configs/servo_offsets.json` relative to the repository.

### Firmware

- **Gripper ESP32:** flash `gripper/esp32_bluetooth/esp32_bluetooth.ino` (Arduino IDE,
  *ESP32 Dev Module*). See [gripper/esp32_bluetooth/README.md](gripper/esp32_bluetooth/README.md)
  for commands and servo ranges.
- **AnySkin QT Py M0 (×2):** flash `gripper/anyskin_qtpy_checked/anyskin_qtpy_checked.ino`. See
  [gripper/anyskin_qtpy_checked/README.md](gripper/anyskin_qtpy_checked/README.md).

### Servo IDs and leader calibration

1. Give every STS3215 a unique ID. Leader: 1–6 for the joints (base to wrist), 7 for the trigger.
   Use `gripper/scan_ids.py` and `gripper/assign_id.py` (see [gripper/README.md](gripper/README.md)).
2. Put the leader in the reference pose that matches the UR3's zero configuration and record the
   offsets:

   ```bash
   python tools/save_leader_offsets.py --port /dev/ttyACM0
   ```

   This writes `configs/servo_offsets.json`.
3. Edit `ros2/src/tacgello_ros/config/ur3_tacgello.yaml` for your setup: `robot_ip`, the trigger
   raw range `gripper_min_raw`/`gripper_max_raw`, and the gripper width range.

## Running

**1. Gripper Bluetooth bridge** (Windows host, leave it running):

```powershell
python gripper\esp32_bluetooth\windows_bluetooth_tcp_bridge.py --port COM11 --host 127.0.0.1 --tcp-port 15555
```

**2. TacGELLO teleoperation with the fingertip channel** (WSL/Ubuntu, ROS sourced):

```bash
ros2 launch tacgello_ros ur3_tacgello.launch.py \
  leader_port:=/dev/serial/by-id/<leader-adapter> robot_ip:=<ur3-ip> \
  anyskin_ports:=/dev/serial/by-id/<anyskin-1>,/dev/serial/by-id/<anyskin-2> \
  bluetooth_host:=127.0.0.1 bluetooth_port:=15555
```

At startup the gripper controller calibrates the AnySkin empty-gripper baseline. Keep the gripper
empty and open until it reports that calibration is done. The leader publisher starts about
14 s later and aligns its first command to the robot's current joint state.

**3. Add the arm channel** (joint-current load cue on leader IDs 2, 3 and 4):

```bash
ros2 launch tacgello_ros ur3_tacgello.launch.py <arguments as above> \
  body_resistance:=true body_resistance_source:=current \
  body_resistance_both:=true body_resistance_wrist:=true \
  body_resistance_continuous:=true body_weight_feedback:=true
```

Keep the tool empty and still at startup while the current baseline is collected. Monitor the cue
with `ros2 topic echo /tacgello/lift_weight`. Settings, safety behaviour and limitations are in
[docs/JOINT_CURRENT_FEEDBACK.md](docs/JOINT_CURRENT_FEEDBACK.md) and
[docs/BODY_WEIGHT_FEEDBACK.md](docs/BODY_WEIGHT_FEEDBACK.md).

**Baseline interface** (same leader and gripper, no AnySkin and no leader feedback), used as the
"original interface" comparison:

```bash
ros2 launch tacgello_ros ur3_tacgello_no_anyskin.launch.py leader_port:=<leader-adapter> robot_ip:=<ur3-ip>
```

### Logs and reports

With `tracking_report:=true` (the default), each session writes `gripper/tracking_reports/<timestamp>/`.
It contains joint command/actual tracking, gripper events and latency, AnySkin contact events,
`lift_weight.csv` for the arm channel, and an HTML summary. See
[docs/GRIPPER_RECORDING.md](docs/GRIPPER_RECORDING.md) and
[gripper/ANYSKIN_RAW_RECORDING.md](gripper/ANYSKIN_RAW_RECORDING.md).

## Tests

The control logic has hardware-free unit tests:

```bash
cd gripper && python -m pytest
cd ros2/src/tacgello_ros && python -m pytest   # needs a sourced ROS 2 environment
cd ros2/tools && python -m pytest
```

## Safety and limitations

Leader-servo output limits aren't calibrated as joint torque or as manual-override force, and the
arm cue isn't a weight estimate: posture and motion also change joint current. Cue release after
host failure or communication loss hasn't been tested, so keep power isolation for the leader and
the robot within reach. AnySkin is magnetic; ferromagnetic objects can affect contact detection.
See the paper's *Operating limits* section for details.

## Experimental features

Slip detection and automatic regrasp (heuristic, and learned with the
[Pollen Robotics AnySkin slip model](https://huggingface.co/pollen-robotics/anyskin-slip-detection))
are in the gripper controller but **off by default** (`slip_regrasp:=false`). The paper doesn't
evaluate them. Their probes and notes are in [experimental/](experimental/).

## Acknowledgements

TacGELLO started as a fork of [GELLO](https://github.com/wuphilipp/gello_software) by Philipp Wu
et al., and its leader design follows GELLO's kinematically matched, low-cost approach. Tactile
sensing uses [AnySkin](https://github.com/raunaqbhirangi/anyskin) by Raunaq Bhirangi et al.; the QT
Py firmware is adapted from AnySkin's Arduino sketch. Licenses for third-party components are listed
in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

## Citation

If you use this work, please cite TacGELLO together with GELLO and AnySkin:

```bibtex
@article{kim2026tacgello,
  title   = {{TacGELLO}: Tactile Contact Feedback for a 3D-Printed Teleoperation Leader},
  author  = {Kim, Younsoo and Song, Sangmin and Kodagoda, Sarath and Carmichael, Marc and Thiyagarajan, Karthick},
  journal = {arXiv preprint arXiv:2610.05583},
  year    = {2026}
}

@inproceedings{wu2024gello,
  title     = {{GELLO}: A General, Low-Cost, and Intuitive Teleoperation Framework for Robot Manipulators},
  author    = {Wu, Philipp and Shentu, Yide and Yi, Zhongke and Lin, Xingyu and Abbeel, Pieter},
  booktitle = {IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year      = {2024}
}

@inproceedings{bhirangi2025anyskin,
  title     = {{AnySkin}: Plug-and-play Skin Sensing for Robotic Touch},
  author    = {Bhirangi, Raunaq and Pattabiraman, Venkatesh and Erciyes, Enes and Cao, Yifeng and Hellebrekers, Tess and Pinto, Lerrel},
  booktitle = {IEEE International Conference on Robotics and Automation (ICRA)},
  year      = {2025}
}
```

## License

TacGELLO is released under the [MIT License](LICENSE). This covers everything we designed, including
the leader software, the custom two-finger gripper (controller code and ESP32 firmware) and, when
released, its CAD files. You can use, modify and redistribute them freely, including commercially,
as long as you keep the copyright notice.

TacGELLO builds on [GELLO](https://github.com/wuphilipp/gello_software) and
[AnySkin](https://github.com/raunaqbhirangi/anyskin), which are both MIT-licensed. The pretrained
slip model in `gripper/models/` is Apache-2.0. See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
