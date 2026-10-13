# tacgello_ros

ROS 2 (Humble) package for TacGELLO. It reads the STS3215 leader arm, streams joint targets to a
UR3 over RTDE `servoJ`, maps the leader trigger to gripper width, and renders the two feedback cues
on the leader's own servos.

## Nodes

| Executable | Role |
| --- | --- |
| `leader_publisher` | Reads leader servos 1–7, applies `configs/servo_offsets.json` and a startup alignment to the robot's joint state, and sends RTDE `servoJ` targets. Publishes the leader joint states and the trigger-mapped gripper width. Holds the trigger servo on AnySkin contact (fingertip channel), and optionally adjusts leader joint output limits from UR joint current (arm channel). |
| `tracking_report_recorder` | Records leader-vs-UR3 tracking, gripper commands and positions, contact events, timing and arm-cue telemetry, and writes CSV/SVG/HTML reports. |
| `bluetooth_gripper_bridge` | Drives the Bluetooth gripper directly from the trigger, without AnySkin. Used by the baseline launch. |
| `lerobot_demo_recorder` | Optional camera + joint dataset recorder (LeRobot v3 format; needs `opencv-python`, `pyarrow`). |

Supporting modules: `joint_current_cue.py` (current-to-output mapping), `body_resistance.py`
(output-limited position-lag cue), `lift_weight.py`, `timing_measurements.py` and the `*_report.py`
helpers.

## Launch files

| Launch | Description |
| --- | --- |
| `ur3_tacgello.launch.py` | TacGELLO: gripper controller with AnySkin contact detection + leader publisher with trigger hold. Add the `body_resistance*` arguments for the arm channel (see the top-level README). |
| `ur3_tacgello_no_anyskin.launch.py` | Baseline interface: same leader and gripper, no AnySkin and no leader feedback. |
| `ur3_tacgello_record.launch.py` | Teleoperation + camera dataset recording. |

## Main topics

| Topic | Type | Notes |
| --- | --- | --- |
| `/tacgello/joint_states` | `sensor_msgs/JointState` | leader joint angles (UR joint names) |
| `/tacgello/gripper_raw` | `std_msgs/Float32` | raw trigger position |
| `/tacgello/gripper_width_command` | `std_msgs/Float64MultiArray` | commanded gripper width (m) |
| `/tacgello/gripper_actual_raw` | `std_msgs/Float64MultiArray` | finger positions reported by the gripper |
| `/tacgello/gripper_contact_hold` | `std_msgs/Bool` | AnySkin contact flag that drives the trigger hold |
| `/tacgello/lift_weight` | `std_msgs/String` (JSON) | arm-channel status: baselines, current deltas, output limits |
| `/tacgello/control_timing` | `std_msgs/String` (JSON) | host timing events |

Services: `/tacgello/tare_body_current` and `/tacgello/tare_lift_weight` (`std_srvs/Trigger`) to
re-collect the empty baseline.

## Build and test

```bash
cd TacGELLO/ros2
colcon build --packages-select tacgello_ros
source install/setup.bash
cd src/tacgello_ros && python -m pytest
```
