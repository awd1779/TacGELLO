# Measured gripper position recording

From `ros2/`, run `python3 tools/record_camera.py` after sourcing ROS and the workspace. Recording
initially waits: the first `s` starts the first episode once robot, leader and
camera inputs are available. Later `s` presses save the current episode and
pause recording until the next `s` starts a new episode. Both image frames and
robot/gripper data are excluded while waiting; the camera connection stays on.
Failed saves retain the paused episode: `s` retries saving, without overwriting
it. Episodes shorter than five frames are discarded with a message. The existing
robot/gripper controller must run separately.

Each recorded frame now includes these length-two float32 features, ordered by
servo ID 1, then ID 2:

- `observation.gripper_actual_raw`: last measured encoder counts, not commands
  or millimetres. A never-received position is -1.
- `observation.gripper_actual_age_s`: monotonic time since the recorder received
  that motor's measurement; -1 means never received.
- `observation.gripper_actual_valid`: 1 if received within 0.5 seconds, otherwise
  0. Stale raw values remain available but must not be treated as current.

The source is `/tacgello/gripper_actual_raw` (`Float64MultiArray`, one
`[servo_id, raw]` pair per message). This recorder only subscribes; it does not
send additional hardware reads or movement commands. Measurements are latest
available values at frame capture, not hardware-synchronized measurements.

The original seven-element `observation.state`, `action`, and `gripper` fields
retain their existing command-width semantics. Actual position is stored in the
new separate features, so existing consumers are not silently given new units.
Raw-position statistics exclude invalid measurements; zero statistics when no
valid measurements exist are placeholders, not measured zero positions.

Older datasets without these features are not appended to: requesting an old
root selects a new numbered sibling. Existing recordings cannot be retroactively
filled with measurements that were not recorded.
