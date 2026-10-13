# Experimental leader body current cue

## Moderate output increase (current default)

`piecewise_current_v5_absolute_delta_gain_0_6` uses gain 0.6 instead of 0.5,
increasing targets by 20% for the same accepted change. Maximum requested
outputs are 24/24/6 raw for IDs 2/3/4; hardware ceilings remain 40/40/10.
Signed-window stability, absolute-delta mapping, freshness, release and output
slew checks are unchanged. Integer output rounding means instantaneous values
need not scale exactly. This is not a measured physical-force increase.
The observed ID3 position-jump fault remains unresolved and its protective
shutdown is NOT relaxed. Restart to apply; no hardware actuation was run here.

## Absolute signed-current change (previous 0.5 gain)

`piecewise_current_v4_absolute_delta_half_gain` uses
`abs(median(current - empty_baseline))` for the live contrast-mode curve.
Stable negative and positive changes now give equal targets. Stability is
checked on SIGNED changes before abs, so alternating signs cannot appear stable.
The 0.5 s / 25-sample stationary acquisition, contact/freshness gates, deadbands,
half gain, output slew and hardware caps remain unchanged. Legacy linear mode
retains its previous magnitude-increase calculation.

This is an experimental change cue, NOT posture compensation or a weight model.
A stable posture-induced change or sign reversal can also produce resistance.
Status/CSV record `accepted_signed_delta_a` and `accepted_absolute_delta_a` per
joint; the historical `accepted_magnitude_increase_a` field is left empty for
new data rather than mislabeling the new quantity. Old reports are not rewritten.
Restart with an empty stationary tool. Validate at low height before heavier
loads; the agent has not verified physical resistance.

## Softer body cue (retained gain)

Following the operator's report of excessive resistance with one bottle,
`piecewise_current_v3_half_gain` multiplies the v2 target fraction by 0.5.
Current thresholds, motion hold and safety release are unchanged. Midpoint
targets are now 8/8/2 raw for ID2/3/4 and maximum requested settings are 20/20/5.
Configured motor ceilings remain 40/40/10; the input mapping is softer. Integer
rounding and slew mean instantaneous settings need not be exactly halved.
This is NOT a claim of half the physical force. Lever output (150) and gripper
reinforcement are unchanged. Status and CSV include `body_output_gain=0.5`
and the mapping name. Restart the controller to apply. Hardware feel was not
tested by the agent. The previous v2 curve below describes the unscaled mapping.

## Loaded-current acquisition update

Loaded acquisition now requires 0.5 s (previously 1 s) and at least 25 fresh
samples in a 0.7 s rolling window. The maximum joint speed remains 0.01 rad/s
and current-increase spread remains <= 0.08 A. Startup tare and the optional
pre-grasp baseline collection are unchanged. Motion still clears the loaded
window but does not discard an already accepted target in a valid grasp.

The 20261001_163017 report contains six contact intervals; three intervals of
about 8-9 s have zero output on all selected joints and remain in loaded-current
waiting state. It contains no reported motor faults. This confirms acquisition
blocking, but old telemetry cannot distinguish speed from current-spread or
window-duration rejection. Shortening the window is a bounded trial, not proof
that every failed grasp will now produce output or that currents identify mass.
Per-joint status and `lift_weight.csv` now retain `load_wait_reason`, window span,
current spread, maximum joint speed, accepted increase and target fraction.
Low accepted load can still correctly yield zero; empty-baseline drift and
static posture effects remain unresolved. No hardware activation was run here.

## Current mapping v2 (2026-10-01)

This mapping supersedes the older squared curve documented below. The live
contrast mode uses continuous piecewise-linear interpolation between three
points per joint, expressed as magnitude increase over the empty baseline:

| Joint | Zero-output boundary | 40% of output cap | Full-cap boundary |
| --- | --- | --- | --- |
| J2 | 0.10 A | 0.17 A | 0.34 A |
| J3 | 0.18 A | 0.30 A | 0.60 A |
| J4 | 0.08 A | 0.15 A | 0.30 A |

Outputs at the midpoint are 16/16/4 raw, with unchanged caps 40/40/10.
Interpolate from zero to 0.4 fraction below the midpoint, and from 0.4 to 1
above it. This preserves headroom beyond the first bottle-scale reference.
Midpoints approximate the FIRST 600g trial relative to the preceding empty
capture. Full-cap boundaries are provisional twice-midpoint current settings,
NOT measured or calibrated 1.2kg thresholds. Repeated trials had inconsistent
baselines: the second 600g J2/J3 changes remain below the deadbands. No label
override forces a nominal bottle mass to produce a particular output.

Status identifies `current_mapping=piecewise_current_v2`, per-joint low/mid/high
points and `contrast_mid_fraction=0.4`. Startup baseline, stationary-load
acquisition, 5 raw/s slew, held target during valid motion, and immediate
contact/data-fault release are unchanged. No actuator commands were issued
while implementing this mapping. Physical perceptibility remains unverified.

## Live startup-reference update

The live publisher now uses the automatically acquired STARTUP empty-current
baseline, not a per-grasp baseline. No pause immediately before closing and no
return to the initial pose is required. The baseline is not silently updated
during approach, grasp or release. Manual tare still replaces it deliberately.

Movement handling is exclusion, NOT numerical compensation: only samples with
all six joint speeds <= 0.01 rad/s enter the 1-second stable-load window. Moving
samples clear that window and cannot raise the accepted load target. A previous
target is held throughout the same valid grasp, without a 5-second expiry.
Fresh RTDE and contact heartbeat remain mandatory on every tick: holding a
target never permits stale sensor input. Before the first loaded window,
output is zero. Contact loss and
invalid/stale input still release immediately. Thus the operator can grasp
without a pre-grasp pause, but must briefly settle AFTER lifting to establish
a cue. Motion and accelerations are not converted into estimated kilograms.

The fixed startup reference does NOT compensate for static gravity changes
with pose, friction or baseline drift. A heavier-object guarantee cannot be
made across arbitrary poses. All thresholds, caps and slew rates are unchanged.
Status reports `baseline_source=startup_or_manual` and
`motion_current_handling=exclude_moving_samples_hold_during_valid_grasp`.
`load_age_sec` is the age of the accepted load estimate, not RTDE freshness.
Load changes during motion are not re-estimated until a stable window.
Sustained real resistance and heating remain unverified; position mode still
limits lag to two raw counts and is not calibrated constant torque.
The former pre-grasp behavior remains only as an internal opt-in helper mode
(`pregrasp=True`) covered by tests; it is not selected by the live publisher.

## Optional ID4 trial

Add `body_resistance_wrist:=true` with `body_resistance_both:=true` and
`body_resistance_source:=current` to include leader ID4 driven by UR J4 current.
Default is OFF; restart is required. ID4 has its own empty baseline and a
10-raw maximum (ID2/3 retain 40), with contrast boundaries 0.08/0.20 A.
The recent 600g trials increased J4 current by 0.157/0.152 A relative to
preceding empty captures; light-object changes were 0.032/-0.007 A. This is
only two trials, not calibrated mass or verified wrist force. The predicted
steady wrist setting is roughly 4 raw for those bottle differences.
All selected baselines must finish; any selected motor fault releases all.
Contact loss and data faults still clear cues. The same stability and valid-grasp
gates apply. Per-joint telemetry and report overlays include ID4.
No physical wrist activation was performed; start with a brief supported trial
and stop on vibration, unexpected motion, or heating.

## Stationary contrast trial (current live default)

The optional helper profile (`pregrasp=True`, not the live default) freezes a fresh PRE-GRASP baseline for each
contact episode instead of using startup current as the load reference.
While contact is false, a rolling 1.2 s window requires at least 25 samples
spanning 1 s, all joint speeds <= 0.01 rad/s, selected-current spread <= 0.08 A,
and each joint's pose spread <= 0.01 rad. Median current and pose are retained.
At contact onset, the candidate must be <= 2 s old and within 0.035 rad on
every joint. It is then frozen until release; loaded samples never update it.
All selected joints need a valid pre-grasp baseline before any cue is allowed.
Missing/expired baseline or data failure requires release and reacquisition;
there is no fallback to the startup baseline. Startup/manual tare remains a
readiness step, but is no longer the load comparison reference.

Hold the actually empty gripper still near the intended grasp pose before
closing. After lifting clear of supports, pause near that pose to acquire the
loaded current. No-contact is not independent proof of empty load; a falsely
released contact can contaminate a new candidate. The code DOES NOT verify
support clearance or determine actual mass. Compare only supported test
conditions. Status includes `baseline_source`, `grasp_baseline_valid`,
`grasp_baseline_pc_s`, and `accepted_magnitude_increase_a` per joint.

The current-mode publisher now selects `stationary_contrast`. The linear
mapping described below remains the legacy helper default, not the live curve.
The experimental curve is `40 * clamp((increase-low)/(high-low), 0, 1)^2`:
J2 low/high = 0.10/0.30 A; J3 = 0.18/0.35 A. Increase is current magnitude
minus the empty-baseline magnitude. No kg is inferred. Cap stays 40 raw;
valid output changes remain bounded to 5 raw/s. Lever feedback stays 150.

These are trial thresholds, not fitted or validated mass boundaries. In the
20260930 12:59-13:05 captures, light-object J3 increases relative to preceding
empty captures were about 0.12 and 0.07 A (about 0.17 A relative to the empty
capture following the first light object). The first 600g increase was 0.30 A,
but the second was only 0.047 A. Empty baseline variation overlapped loaded
values. This curve intentionally does NOT force the second bottle trial to
produce a strong cue. J2 thresholds likewise only suppress small changes;
they do not distinguish mass reliably across these recordings.

In that optional pre-grasp mode, the pre-grasp median joint pose is retained. Load updates require all six
joints within 0.035 rad of that pose, all joint speeds <= 0.01 rad/s, at least
25 samples spanning 1 s, and selected-current-increase spread <= 0.08 A in
the rolling 1.2 s window. Stay near the pre-grasp pose for each measurement. This
is a pose eligibility gate, NOT posture compensation. Baseline drift remains
unresolved. Invalid pose/data/contact clears the cue immediately.

Outside the eligible measurement window, the last accepted target is retained
throughout the valid grasp; motion current cannot increase that target.
Contact loss resets the held load. Acquisition and target age are reported in
status. This is a held estimate, not live weight measurement during motion.
No live motor trial was run during editing.

## Continuous position-lag trial

Add `body_resistance_continuous:=true` to the current-mode launch to test
continuous torque enable while contact and positive valid cue effort persist.
Default is false, preserving the original motion-gated mode. Restart to change
this option. No EEPROM, firmware or servo operating mode is changed.

The continuous mode removes the per-tick two-count movement gate. Velocity is
filtered over 0.08 s; target lag is bounded to two counts. Rest places the target
at the current measured position, and reversal clears previous lag before
building opposite resistance. Torque may remain ON at rest, but that does not
mean constant physical torque or weight. Output starts at a low setting and
increases at 5 raw/s (integer quantization can add one raw), up to the current
40-raw ceiling. Disable, zero effort, contact loss, stale contact, missing joint
position and timing faults still request torque OFF; either-joint fault releases
the pair. Mode is included as `resistance_mode` in status and report CSV.

This is not direct torque control or guaranteed passive damping. Gear friction,
position quantization, communication timing and current-input dropouts can still
cause ripple. Hardware smoothness and sustained temperature are unverified.
Begin with brief low-height trials and accessible power isolation. Stop for
unexpected motion, vibration or heating. The host cannot guarantee release on
process/cable failure. Motor activation was not performed during implementation.

This mode changes only the selected leader body servo, not gripper commands or
gripper-lever feedback. It is not calibrated torque, object mass, external load,
or a robot payload-limit indicator. Posture and motion affect motor current.
There is no posture compensation in this first mode.

Add these options to the existing AnySkin control launch, with control stopped
before restarting. Do not run a second publisher against the leader port:

```bash
body_resistance_source:=current body_resistance_servo_id:=2 \
body_resistance:=true body_resistance_bench:=false body_weight_feedback:=true
```

By default one body servo is controlled: 2 = shoulder, 3 = elbow.
Add `body_resistance_both:=true` to control BOTH 2 and 3, each using its own
UR joint current and baseline. This option requires `body_resistance_source:=current`.
Both baselines must finish before either cue can act. Each servo retains its
own current-mode limit of 40 raw for the requested trial; the stronger combined feel has not been physically validated.
Either motor fault inhibits both. `body_servos` reports the two output states,
and `joint_cues` reports each baseline, current difference and cue fraction.
All six currents and signed baseline differences are exposed in status. Only
the selected joint's increase in current magnitude determines its cue.
At startup, current mode automatically waits for fresh RTDE and no-contact
heartbeat with stationary joints, then collects the empty baseline once. Keep
the gripper EMPTY during startup: no-contact telemetry is not independent proof
of an unloaded tool. No motor cue is permitted before acquisition completes for
all selected joints. No manual command is normally necessary. Check status with:

```bash
ros2 topic echo /tacgello/lift_weight --once --full-length
```

`auto_tare_pending` indicates waiting to start; `tare_in_progress` indicates
collection. Baseline values appear when successful. The baseline is never
automatically replaced after a grasp, a timeout, or a robot timestamp reset.
After a failed collection, or to deliberately replace the baseline, use:

```bash
source /opt/ros/humble/setup.bash
ros2 service call /tacgello/tare_body_current std_srvs/srv/Trigger '{}'
ros2 topic echo /tacgello/lift_weight --once --full-length
```

Keep the EMPTY tool stationary and clear of supports. The manual request needs a fresh
no-contact heartbeat. Acquisition requires at least 25 unique robot samples
over one second, joint speeds <= 0.03 rad/s, and selected-current spread <= 0.15 A.
Collection times out after 10 seconds (startup waiting does not consume that
window). `current baseline ready` means acquisition, not
verified resistance. Baseline is not silently updated while grasping.

For subsequent contact, the positive magnitude increase has a 0.05 A deadband,
0.35 A additional range to saturation, 0.2 s low-pass filter and maximum valid-current
output ramp of 5 raw per second in both directions. Activation latches at 0.05 A;
release requires current increase <= 0.025 A for 0.25 s, then ramps down.
While latched, the requested output floor is one raw. Contact loss, invalid or
stale input and faults still release immediately, without this ramp or delay.
These are experimental mapping settings,
not motor ratings or a weight calibration. The existing output-limit register
ceiling is 40 in current mode (legacy weight/bench remains 10). The two-count
position lag is unchanged. This trial increases gain: steady current magnitude
increases of 0.225 and 0.40 A map to 20 and 40 raw. At the retained 5 raw/s slew,
rising from zero to 40 takes at least eight seconds of eligible samples.
The higher setting needs physical validation; it is not a measured torque ratio.
It does not guarantee 40 raw for a 600g bottle. No constant-output lock is enabled.
In the default motion-gated mode, cue is applied only when the operator moves
the selected controller joint. Continuous mode retains torque enable at rest
as described above; it does not provide calibrated constant resistance.

The separate gripper-lever feedback launch default is now 150 raw instead of
300. Use `haptic_hold_output_limit:=150` explicitly if reusing an old command.
This does not change the body-servo ceiling of 40 raw.

Status remains on `/tacgello/lift_weight` for existing telemetry consumers, but
`source=joint_current`, `estimated_kg=null`, `cue_max_kg=null`,
`posture_compensated=false`, and `external_load_verified=false` explicitly
distinguish it from weight estimation. `delta_current_a` is signed; cue uses
increase in magnitude, not absolute signed difference. Do not plot it as kg.

Disable during a trial:

```bash
ros2 param set /leader_publisher body_resistance false
```

Check `resistance_active=false` and any `resistance_fault`. Host-side release
depends on an operating process and serial connection; cable loss or process
termination cannot guarantee release. Have physical power isolation available.
Faults latch and require inspection/restart. No automatic motor activation was
performed during implementation. First physical validation should use the low
output limit, accessible stop, and a supported test area.

Changing source or servo ID requires restarting the node. Do not alter these
parameters at runtime. A validated posture-dependent baseline is still needed
before attributing the cue specifically to object load across arbitrary poses.

When effort is zero, control is disabled, or contact/rest requires release,
torque OFF is now sent and read back at least every 0.5 seconds, even when the
host believes the servo is already inactive. This handles leftover torque from
a previous run without treating the internal flag as hardware proof. No contact
must produce no intentional body cue. A false latched contact can still allow
posture-driven current to create resistance; this mode does not solve contact
classification or gravity compensation. If an empty arm feels heavy, compare
`contact`, `joint_cues`, and `body_servos` with body resistance disabled. Do not
interpret ordinary mechanical drag as verified motor output.

Reports now retain per-joint currents, signed current changes, cue fractions,
activation and output-limit settings in `lift_weight.csv`. Current-mode graphs
show separate ID2/ID3 step traces and activation shading (not kg). The HTML table
and `body_joint_resistance_intervals.csv` give start/end times and peak settings.
Each joint's activation and output changes trigger telemetry immediately rather
than only the combined active flag. Missing telemetry is never extended as a
known active interval. Historical recordings without joint telemetry cannot be
reconstructed into verified per-joint results.

The joint tracking SVG also shades the J2/J3 Controller angle panels directly.
Each gets a separately scaled output strip on the same time axis, a visible peak
setting, and hover timestamps/settings. UR3 actual panels are not shaded. Body
telemetry uses recording-relative timestamps; joint plots use their own PC
origin, so the renderer converts between those origins before drawing. New
summaries retain `recording_origin_pc_s` alongside `plot_origin_pc_s` for replay.
`output_cap_raw`, `current_range_a` and `current_deadband_a` are saved in status
and CSV. The strip scale includes the recorded
cap and observed maximum, so values above 10 are not clipped.
