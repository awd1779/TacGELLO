# Loaded reinforcement: evidence and bounded correction

## Learned-mode sensor-rise trial (2026-10-01)

Learned mode now permits up to +500 sensor units relative to the original
pre-reinforcement reference, previously +400. Any channel above +500 still
blocks advancement. Repeated moves do not rebase this reference. Legacy
heuristic mode remains +60. This changes a sensor-change threshold, not grip
force, torque, or a validated safe load. No hardware validation has been run.
The cumulative grasp target budget is now 50 raw per jaw (previously 40),
and each episode permits up to 25 raw (previously 22). These are ceilings,
not a command to execute exactly two 25-raw moves. The 13/4/4 step schedule,
PWM/current observations, 18-raw goal error, three
stages and six attempts remain unchanged. Persistent high model scores still
require clear then a new high event for another attempt. Restart learned-mode
control to apply; do not flash firmware. Use a low-height catch-supported trial.
Rapid sensor growth is not verified safe compression or a calibrated force
curve. Stop testing on power resets, abnormal heat, or visible deformation;
the remaining software observations do not guarantee hardware protection.

## Bounded travel increase (2026-10-01)

Episode and cumulative grasp budgets are now 25/50 raw.
Bilateral completion now requires 12 raw observed closing instead of 10, in
two fresh observations per jaw. Initial/continuation increments remain 13/4.
Both previous-goal error and proposed closing error remain limited to 18 raw;
travel permission does not authorize increasing error against a stalled motor.
PWM/current observation ceilings, sensor-rise guard, freshness, contact and
opening release are unchanged. No firmware or torque-register change is needed.
Restart the controller. These are displacement bounds, not verified grip force
or a guarantee of holding 1.2 kg. Recent sensor-rise stops can still block motion.

## Persistent-slip recheck

The learned scorer's wait between inferences is now 25ms (was 50ms). Inference
and transport time are additional; this is not a measured end-to-end latency
guarantee. The 0.90 threshold, three distinct fresh high samples, and initial
0.30s contact settling are unchanged.

Persistent high scores no longer authorize another automatic request, even
after verified bilateral jaw movement. Static post-grasp deformation can produce
high model output; motor movement is not independent confirmation of object slip.
Three fresh low samples must first rearm detection, followed by a new three-sample
high event after the existing cooldown. Attempts and cumulative travel are not
reset by rearming. No new temporal tactile-change threshold has been validated.

No unlimited tightening: six learned-mode attempts per grasp, 25 raw per episode, 50 raw
total per grasp, existing position-error, contact, sensor-rise, PWM/current
observation ceilings and lever-opening release remain unchanged. Budgets are
not reset between repeated requests. The existing short bounded motor ramp may
finish its already-authorized step; this is not a force controller or a promise
to prevent every slip. No automatic extra request occurs on low model scores.

Status exposes `regrasp_trigger_reason`, and ACK logs include `reason=new_slip`
or `reason=persistent_slip`. Reports label repeated requests as "Persistent slip
recheck"; the independent slip observation counter is not incremented by a
repeated reinforcement request. No extra launch arguments are needed: learned
slip reinforcement and motor diagnostics must remain enabled. Restart the
controller after this update; never run two gripper controllers concurrently.

## Normal grasp contact stop

The first above-margin contact candidate requests a measured-pose stop before
feedback publication, slip scoring telemetry or direction recording. The live
launch requires both sensors. An unconfirmed candidate can now recover only
after all channels on both sensors stay below 60% of the contact margin for
0.15 s and at least three new paired samples, with no clear-sample gap over 0.1 s.
With the current margin of 20, the release threshold is 12. Invalid calibration,
missing/stale sensors or stale controller commands cannot authorize recovery.

Recovery requires a continuing closing request, takes at most 2 raw per command
and 8 raw per jaw from the first recovery pose, across at most two resumptions.
Each closing step needs fresh samples from both sensors. Recontact on either
sensor at the release threshold pauses recovery. Exhaustion holds until the
operator opens; budgets reset on an opening release, not on a new candidate.
Already confirmed contact never auto-releases this way. Confirmation and fresh
slip evidence are still required for reinforcement; recovery is not reinforcement.
Status reports bounded slow closing, waiting for clear/fresh data, or exhausted
retry/travel budgets. Restart the controller to apply; no firmware change.
A hold ACK is not proof of physical stop or current limiting, and a missing hold
ACK remains a fault with no replay. Validate with low-height physical trials.

## Softer leader lever hold

The AnySkin launch defaults `haptic_hold_output_limit` to 300 (register range
1..1000). This affects only the leader lever servo ID 7, not the remote gripper
motors or UR3. It is a servo output ceiling, not calibrated hand force or an
ampere limit; a lighter hold may yield under hand pressure.

Before enabling hold, the reader disables torque, checks registers 40..49,
writes the measured-position target and SRAM output limit (48..49), and verifies
both by readback. It uses the lower of the requested and existing limits and
never restores full output on release. EEPROM is not changed. A readback failure
prevents hold activation. Opening release and heartbeat timeout remain active.

Restart the existing launch to use the new default; no extra argument is needed.
The lowered SRAM limit remains until changed or servo power is cycled; restarting
the node with a higher requested limit does not automatically raise it.

## Controller input recovery

In learned mode, a missing/timed-out controller input before transmission now
pauses reinforcement instead of permanently inhibiting that grasp. Pending
manual probes and ramp continuations are discarded. Recovery requires at least
three distinct fresh input timestamps spanning 0.2 seconds, then three fresh
model samples <=0.30 on both sensors, then a new confirmed slip. No queued slip
is replayed. Attempts, grasp travel origin, cooldown and pressure reference are
retained. Invalid baselines, motor/ACK faults and unverified in-flight motion
remain inhibited; this recovery never clears them.

Fresh lever commands are published before UR3 RTDE read/send calls, independently
of body alignment. Missing servo reads do not renew the command heartbeat from
cached positions. Verbose pre-send state logs now use DEBUG; ACK and physical
motion results remain available. This reduces avoidable delays but does not make
the shared publisher timer independent of a blocked RTDE call.

No new launch arguments are required. Rebuild the ROS publisher and restart the
existing launch after stopping the prior session. Hardware validation is still
required; software tests do not prove loaded motor movement.

## Diagnostic timing

Missing or stale motor diagnostics pause further commands while fresh replies
are collected. The command window is 3.5 seconds; up to 0.75 additional
seconds are allowed for observation only, never another closing step. Missing
feedback still ends the episode without success. Travel, output, contact and
sensor-rise limits are unchanged. Full motor diagnostic JSON remains on the
recording topic; console copies now use DEBUG to reduce synchronous log output.

## Evidence

The 38babb79 trial registered goals (1748, 2249), speed 2919, acceleration 22,
position mode 0, torque enabled, torque limit 1000. Both goal readbacks stayed
correct but encoder positions stayed (1751, 2244) for two seconds. No persistent
old HOLD/lever target overwrite is present in those observations.

Waveshare's [ST3215 register map V3.7](https://files.waveshare.com/upload/2/27/ST3215%20memory%20register%20map-EN.xls)
identifies decimal 21/22/23 as position P/D/I, 24 as minimum starting output,
and 60 as output PWM duty (not measured torque). The installed settings read
P=32, D=32, I=0, minimum output=16. The vendored SMS_STS.cpp ReadLoad decodes
bit 10 as direction: 1072 means -48, not an output of 107.2%.

After the command, jaw 1 duty was 3.2%; jaw 2 duty was about 4.8-5.6%.
This supports insufficient small-error drive under load as the working
hypothesis, rather than a missing motion instruction. It does not distinguish
static friction from object compliance/blockage, or rule out unsampled power
transients. Speed 2919 and torque ceiling 1000 do not demand full torque.

The older 20260916 tracker paused *new commands* while confirming contact but
did not replace outstanding travel with a measured-position hold until contact
was confirmed. That could produce the earlier unwanted extra squeeze. Do not
restore that behavior on first contact.

## Changed automatic actuation

After the 20260923 power-cycle report, automatic loaded reinforcement is limited
to ONE 3-count stage, with at most 6 counts of closing goal error relative to the
measured pose. It no longer ramps a stationary jaw through four stages. With
fresh fault-free motor feedback and no confirmed two-jaw response, it reads the
current pose again and requests a measured-pose hold, then inhibits reinforcement
until release. This reduces outstanding position error; it is not torque-off,
current regulation or proof of lower force. Missing/faulted feedback never causes
a blind corrective move. Normal closing target increments are reduced from 8 to
4 counts per cycle; opening increments and configured speed 2919 are unchanged.

Initial arming now uses 0.30 seconds of contact settling instead of requiring
three low model scores. Three fresh contacting-sensor scores >=0.90, acquired
after the settling interval, confirm the first event. Repeated timestamps and
samples acquired during settling do not count. The monitor shows CONTACT
SETTLING during this interval. After reinforcement, the existing cooldown and
three fresh scores <=0.30 are still required before another episode. A
sustained false high can therefore cause the first episode, but cannot by
itself rearm repeated reinforcement. Sensor validity and live-contact checks
are unchanged.

For learned automatic reinforcement with `regrasp_diagnostics:=true`:

- Keep first-contact stop, lever lock, speed 2919 and acceleration 22 unchanged.
- One confirmed slip starts one bounded episode, not repeated new slip events.
- First advance goals by 13 counts. After 0.8 seconds without confirmed movement,
  permit up to two additional paired 4-count steps, after rechecking
  contact, opening input, goal readback, sensor validity and observed output.
  All stages are capped at 18 counts of error from measured position; existing
  error can reduce either increment. No gain or torque-register changes.
- Require two fresh observations per jaw. Log a midpoint after 0.4 seconds and
  observe the unchanged goal until 0.8 seconds before judging no response.
- A partial response does not exclude one jaw from the next paired command.
  Every continuation advances both goals with one SYNC_MOVE; if either jaw has
  no remaining budget, neither goal is sent. Physical start times may differ.
  Once both jaws have at least 12 counts of closing displacement in two fresh
  observations, the episode finishes. This is movement confirmation, not proof
  of slip suppression. Output, contact and sensor-rise guards remain unchanged.
- Maximum 3 stages, 25 counts from episode starting measured position,
  50 counts from original contact target per grasp, and 3.5 seconds for progression.
- Fresh sensor/model data, live contact, the existing sensor-rise guard and
  opening priority remain required. A confirmed slip need not stay high in
  every later model prediction while this short episode executes.
- If live contact alone briefly falls below threshold, retain the confirmed
  automatic event for at most 0.25 seconds. No new closing command is sent while
  waiting (an already active motor goal is not cancelled by this wait).
  Retry with fresh motor and sensor feedback, without another slip trigger or
  resetting the attempt, stage, travel budget or continuation deadline.
  Expiry, calibration changes and hard guards cancel the pending request.
  Sensor-rise limits are not relaxed by this contact-only grace period.
- After verified loaded motion only, a later signal rise above the original
  response cap pauses rearming rather than permanently inhibiting the grasp.
  No new motion is allowed above that cap. The reference is never rebased:
  values must return below it, then three fresh clear predictions and a new
  three-sample slip event are required. Unverified motion, active-step rise
  rejection and motor faults remain inhibited. The six-attempt and cumulative
  travel limits are unchanged. Persistent high scores do not trigger a chase.
- No advance on goal mismatch, motor status fault, disabled torque, non-position
  mode, stale feedback, observed absolute duty >=15%, or current raw >=100.
  These are **observed-output checks, not hardware current/force limits**.
- No blind replay after ACK loss. No EEPROM, PID, torque-limit or firmware changes.
- Manual probe and non-diagnostic/heuristic paths retain their previous bounds.

The larger bounded goal error is intentional: the old fixed 3-count command
could remain stationary under load. This may increase pressure. A rigid object
can prevent encoder motion even when pressure rises; limits terminate escalation,
and success means encoder displacement only, never verified holding force.
On a no-response result with valid feedback, a measured-pose hold replaces the
closing target. Other fault paths can leave the last acknowledged goal active;
communications failure means physical stop cannot be confirmed.

## Run

Stop the previous ROS launch before restarting. Do not use `probe_arm` for this
automatic test. Firmware re-upload and ROS rebuild are not required for this
Python change. Keep the first test object supported on the table, keep hands
clear, and use a non-fragile object. Do not lift until holding is verified.

Windows bridge (only restart if not already running):

```powershell
cd "C:\path\to\TacGELLO\gripper\esp32_bluetooth"
python windows_bluetooth_tcp_bridge.py --port COM11 --host 127.0.0.1 --tcp-port 15555 --trace
```

WSL:

```bash
cd ~/TacGELLO/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
sudo chmod a+rw /dev/ttyACM*
LEADER=/dev/serial/by-id/usb-1a86_USB_Single_Serial_<LEADER_SERIAL>-if00
S1=/dev/serial/by-id/usb-Adafruit_QT_Py_M0_<ANYSKIN_SERIAL>-if00
S2=/dev/serial/by-id/usb-Adafruit_QT_Py_M0_<ANYSKIN_SERIAL>-if00
ros2 launch tacgello_ros ur3_tacgello.launch.py \
  leader_port:="$LEADER" robot_ip:=192.168.50.14 anyskin_ports:="$S1,$S2" \
  bluetooth_host:=127.0.0.1 bluetooth_port:=15555 firmware_hold:=false \
  close_speed:=2919 close_acc:=22 contact_margin:=20 haptic_release_delta:=0.03 \
  empty_close_baseline_percentile:=75 slip_regrasp:=true slip_regrasp_mode:=learned \
  regrasp_diagnostics:=true record_anyskin_raw:=false \
  body_resistance:=false body_resistance_bench:=false tracking_report:=true
```

Complete startup empty-close calibration and open the lever to enable control.
On automatic slip, check `Regrasp ACK` with `bounded_step=1/3` and, only if
needed and permitted, `bounded_step=2/3` and `bounded_step=3/3`.
There is no fourth step in an episode. This profile still needs hardware validation;
encoder response does not establish grip force or protection against brownouts.
Per-position console telemetry is DEBUG; full observations remain in the report.
`closing displacement observed on both jaws; force not verified` is the motion
result. An ACK alone is not success. Ctrl+C saves the tracking report.

Hardware-independent tests exercise the production target/observation methods
with stationary and responding encoder samples. They do not validate motor
breakaway or grip force on the physical gripper.

### Reported slip intervals and reinforcement amounts

New reports distinguish amber model-latched state from purple current high-score
evidence. Purple requires an active model event plus a currently contacting
sensor scoring >=0.90. Amber may persist until both scores clear <=0.30 for
three fresh samples. Neither color is verified object slip. No timeout or ACK
is treated as physical recovery. Raw score traces and original sensor JSONL
are retained, including suspected false positives. Missing telemetry is unknown.
`gripper_slip_intervals.csv` retains latched history for compatibility;
`gripper_model_high_intervals.csv` separately stores high-evidence intervals.
Legacy records without v2 evidence do not invent high-evidence intervals.
Do not report these durations as ground-truth slip or recovery success.

Each reinforcement episode shows commanded and observed closing displacement
separately for ID 1 and ID 2. The table also shows measured before/after encoder
positions, and these positions are exported in `gripper_reinforcement.csv`.
Positive raw counts mean closing; unavailable feedback is not zero movement.
Commanded displacement uses the pre-episode goal, whereas observed displacement
uses the pre-episode measured pose. Neither measures grip force or certifies that
physical slip stopped. Existing saved HTML/SVG files are not automatically rewritten.

### leader lever feedback recording

The controller publishes `gripper_lever_feedback` events and 10 Hz heartbeats on
`/tacgello/control_timing`. Cyan bands on the controller-lever panel represent a
reported hold following successful target/output-limit and torque-enable
readback. Hold/release failures remain in diagnostic logs but are not drawn as
graph markers or successful feedback regions.
Hover for the target and output limit; `lever_feedback_intervals.csv` exports
the intervals. Gaps over 0.5 seconds terminate evidence at the last observation.
This is not continuous register verification or measured physical resistance.
Old reports without this telemetry must not infer lever resistance from contact.

A failed target/output-limit verification now gets one additional read-only
check after 3 ms before failing. No target is replayed and torque is not enabled
without matching registers. Final failures include expected/read register bytes,
communication status and servo error. A failed grasp stays blocked until release;
this does not repeatedly try to lock against the operator.
