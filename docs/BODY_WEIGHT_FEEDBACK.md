# Experimental vertical-lift feedback

This is a bounded leader shoulder cue (servo ID 2), separate from the gripper
lever hold and slip reinforcement. It is NOT whole-arm force reflection, a
scale, a safety stop, or calibrated torque control. No UR payload setting is
changed. The robot must be mounted upright, with base +Z pointing upward.

## Load and effort

An explicit empty-tool tare subtracts the median base-Z force. Subsequent
apparent mass is `(-Fz - zero_force_n) / 9.80665`, using a roughly one-second
stable window of fresh RTDE samples. Four 600ml bottles contain about 2.4kg of
water; bottle/container mass is additional. **2.4kg is only cue saturation.**
UR3's nominal 3kg payload includes the gripper, sensor, mount and object, with
additional centre-of-gravity constraints. Do not test four bottles until total
mass and centre of gravity have been checked against the robot manual.

Nominal mappings: 0.6/1.2/1.8/2.4kg -> 25/50/75/100% of the EXPERIMENTAL cue.
This percentage is not motor torque or human-perceived force. The existing
maximum SRAM output limit stays at 10/1000; target lag stays at two encoder
counts. Rising effort ramps at at most one full scale per second. The cue may
be subtle; its perceptual strength has not been measured on hardware.

Cue requires contact heartbeat, stable force, fresh robot/controller samples,
slow upward TCP movement, and shoulder movement. It releases at rest, on
descent, missing data, contact loss or excessive motion. It does not immobilize
the controller. Position lag is not guaranteed passive; cable loss or a killed
process can prevent torque release. Do not rely on this feature for safety.

CB3 force estimates can be noisy and pose-dependent. Accelerations, table
contact, side loads and changes to payload compensation can look like changes
in weight. Use unsupported objects, near the tare pose/orientation, and verify
against one known bottle first. A wrong sign or value is a calibration issue,
not a reason to increase resistance. A newly acquired load needs about one
second to settle before a cue; rotation/high speed intentionally invalidates it.

## Procedure

### Paired multi-load session (2026-09-30)

Default gripper opening is now ID1=2080, ID2=1920, 80 raw farther open per jaw;
the user confirmed mechanical clearance. The old empty-close envelope does NOT
match this range. Recalibrate empty close before use. Closing endpoints are unchanged.

For calibration, keep body resistance and slip reinforcement OFF to avoid changing
the grip during force recording. Keep other tool/payload settings and reference
pose unchanged. Weigh each taped bottle assembly, including bottles and tape;
600/1200/1800/2400 g water content is not the measured assembly mass. Verify total
payload and centre of gravity separately. The approximate 150 g tool mass is not
a measured payload certificate.

Use `record_lift_force.py --phase empty --seconds 20 --session load_20260930`
then pair each loaded capture with that fresh empty `summary.json`:
`--phase bottle --seconds 20 --expected-kg MEASURED_KG --session load_20260930
--role calibration --empty-report PATH`. Capture near the same unsupported,
stationary pose. Recheck empty zero between loads, not yesterday's zero.
Record at least two independent calibration captures at each allowed load and
separate captures using `--role validation`; do not split one capture into both.

`python3 tools/fit_lift_weight.py --reports PATH1 PATH2 ...` reads the loaded summaries
and their paired empty reports. It requires at least three mass levels with two
captures each, rejects incomplete/noisy/drifting/pose-mismatched data, and saves
an offline `candidate.json`. The fitted gain uses paired mean force differences;
validation errors use held-out recordings. Reported errors are not confidence
intervals. The provisional 0.2 kg review criterion is not a validated accuracy claim.
No candidate is automatically loaded into control. A qualified fit still needs
live stability and new-session verification before connecting it to the cue.
Existing tare limits and servo protection are unchanged. The kg label accepts
up to 3 kg only to describe measurements; this does not authorize such a load.

The display saturation at 2.4 kg is an experimental cue reference, not a safety
limit or an estimate of remaining robot payload capacity. Restore slip control
only for the subsequent functional trial after calibration capture is complete.

Build once from the ROS workspace with the existing ROS environment sourced:

```bash
colcon build --symlink-install --packages-select tacgello_ros
source install/setup.bash
```

Keep the existing Bluetooth TCP bridge running once on port 15555. In WSL,
start read-only weight estimation (this still starts normal robot teleoperation,
so clear the workspace and use the usual robot startup checks):

```bash
cd ~/TacGELLO/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ls -l /dev/serial/by-id/
sudo chmod a+rw /dev/ttyACM*
LEADER=/dev/serial/by-id/usb-1a86_USB_Single_Serial_<LEADER_SERIAL>-if00
S1=/dev/serial/by-id/usb-Adafruit_QT_Py_M0_<ANYSKIN_SERIAL>-if00
S2=/dev/serial/by-id/usb-Adafruit_QT_Py_M0_<ANYSKIN_SERIAL>-if00
ros2 launch tacgello_ros ur3_tacgello.launch.py \
  leader_port:="$LEADER" robot_ip:=192.168.50.14 anyskin_ports:="$S1,$S2" \
  bluetooth_host:=127.0.0.1 bluetooth_port:=15555 firmware_hold:=false \
  close_speed:=2919 close_acc:=22 contact_margin:=20 haptic_release_delta:=0.03 \
  empty_close_baseline_percentile:=75 slip_regrasp:=false \
  regrasp_diagnostics:=true record_anyskin_raw:=false tracking_report:=true \
  body_weight_feedback:=true body_resistance:=false body_resistance_bench:=false
```

Finish the usual AnySkin calibration, fully open the gripper, leave it EMPTY
and stationary. In another sourced ROS terminal:

```bash
ros2 service call /tacgello/tare_lift_weight std_srvs/srv/Trigger '{}'
ros2 topic echo /tacgello/lift_weight
```

The service queues tare, not completion. Tare requires at least 25 distinct,
fresh samples spanning at least one second, retained for up to four seconds
to support slower polling. The existing motion, 2N spread, 150ms sample-gap
and 10s overall timeout limits remain in effect. `tare_in_progress`,
`tare_wait_reason`, `tare_error`, `sample_count`, `sample_span_s` and
`motion_diagnostics` expose progress and blockers. Failure remains visible
until a new explicit tare request; it is not replaced by `tare required`.
Wait for `zero_force_n` to become
non-null. Lift one bottle slowly, briefly pause to settle, and verify
`estimated_kg` is approximately the measured bottle mass. Null means unknown,
not zero. This is a JSON String topic including state, estimate, cue fraction,
resistance active/fault and output-limit readback value.

After verifying the estimate and bench cue, restart the same launch with
`body_resistance:=true` (bench false). Tare again. There is no automatic motion
test, maximum-output change, or automatic full-power restoration on shutdown.
To disable the cue on a running node:

```bash
ros2 param set /leader_publisher body_resistance false
```

Disabling sends torque-off on the next controller update; physical release
still needs confirmation. Stop work on readback faults or unexpected motion.
The read-only force estimate can remain available while resistance is disabled.

## Tracking report

With `tracking_report:=true`, `/tacgello/lift_weight` is recorded on the source PC
monotonic time axis used by the gripper graph. The graph adds a teal strip on
the measured-gripper panel only when the shoulder cue is reported active.
Two aligned panels show estimated object kg and shoulder output-limit raw.
The HTML includes each cue interval's mean estimated mass and peak limit.
Contact without a cue is not shaded as resistance. Missing mass is unknown,
not zero; telemetry gaps over 0.5s stop intervals at the last observation.
An unobserved end does NOT imply a confirmed physical torque release.

The publisher records state edges immediately after the servo update plus
periodic 5Hz samples. Limits are settings, not measured human resistance in N
or motor torque in Nm. A failed torque-off readback remains a fault, not proof
of release. No extra robot commands are issued by the recorder.

Ctrl+C on the launch saves `tracking_report.html`, `gripper_timeline.svg`,
`lift_weight.csv`, `body_resistance_intervals.csv` and `tracking_summary.json`
under the existing `gripper/tracking_reports` session directory.
Old reports lack the weight topic; the new information cannot be reconstructed
from them. LeRobot episode recording and its `s` key flow are unchanged.

## Read-only force diagnostic before changing tare criteria

Keep the existing control launch running with `body_weight_feedback:=true`,
`body_resistance:=false`, `body_resistance_bench:=false`, and `slip_regrasp:=false`.
No successful force tare is needed. The diagnostic creates only an RTDE receive
connection and a ROS subscriber; it never issues motion, payload, tare or output
commands. It requires a fresh source-timestamped status confirming resistance
OFF with no resistance fault. Empty phase requires no contact; bottle phase
requires contact. Software status does not independently measure motor torque.

From the ROS workspace in a separately sourced WSL terminal:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 tools/record_lift_force.py --phase empty --seconds 20
```

Keep the unloaded tool stationary and clear of supports for the whole capture.
Read the printed report path. Next, manually grasp ONE known bottle and hold it
unsupported near the SAME robot pose and orientation. Do not change payload
settings between captures. After motion settles, run:

```bash
python3 tools/record_lift_force.py --phase bottle --seconds 20 \
  --expected-kg 0.6 --empty-report /path/to/empty/summary.json
```

0.6kg is nominal water mass; use measured bottle-plus-water mass when known.
`raw_force.csv` retains raw force0..2 (N), force3..5 (Nm), speed, TCP pose, joint
angles, robot and host timestamps, and stationary flags. `summary.json` and
`report.html` show raw spread, standard deviation, percentiles, MAD, valid 1s
block means, drift, and optional loaded-minus-empty comparison. They are saved
under `ros2/lift_force_diagnostics/<time>_<phase>/`. Source timestamps must
advance; duplicate packets are not counted. Interrupted captures remain marked
incomplete. Gaps and moving samples invalidate blocks; ten valid blocks are
required for sufficient data. Comparison warnings are diagnostic heuristics,
not permission to activate resistance, proof of mass accuracy, or automatic
updates to the tare criteria. No data is silently discarded from the raw CSV.
