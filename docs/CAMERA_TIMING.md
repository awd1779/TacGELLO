# Camera timing checks

The recorder selects new received images at up to the dataset FPS, using the most recently received
robot state and action at that callback. This is receipt-time pairing, not
hardware-synchronized exposure-time interpolation. Press `s` to arm recording;
the next image starts the episode. Press `s` again to save and pause.

The timing report checks:

- Received rate within 5% of the configured dataset rate.
- No receive gap longer than 2.5 frame periods.
- Receive and camera-header times within two frame periods of the nominal grid
  (a constant startup offset is removed from camera-header timing).
- Positive, strictly increasing camera-header timestamps.

These are timing checks, not proof of unique camera exposures or image quality.
Accepted episodes keep source and receive timestamps in
`meta/capture_timing/episode_NNNNNN.json`. Normal LeRobot timestamps remain on the
constant-rate grid for compatibility; timing error is bounded, not eliminated.

The normal `s` save workflow is unchanged: timing warnings are reported and the
episode is saved with its timing report. Only with the optional ROS parameter
`strict_camera_timing:=true` are failed episodes excluded. Their images,
states, actions, gripper feedback and timestamps are preserved in
`quarantine/<time>/capture.npz`, with a `timing.json` report. The terminal reports
QUARANTINED and the next `s` starts a new episode. These archives can be large.
Do not import them into training without reviewing timing and synchronizing data.

## C960 capture settings and transport

`record_camera.py` still runs WSL `usb_cam`, the same ROS topics and the same
LeRobot recorder, with `s` start/save. No Windows capture service is used.

- Child camera, recorder and keyboard-service processes default to Cyclone DDS.
  An explicit `RMW_IMPLEMENTATION` environment override is respected. Install
  `ros-humble-rmw-cyclonedds-cpp` first; the launcher checks this dependency.
- C960's current V4L2 control names are used directly because usb_cam 0.8.1
  attempts legacy exposure names that this device rejects.
- As of 2026-09-28, dynamic frame-rate reduction is disabled, exposure defaults
  to 100 x 100 us (10 ms), and power-line frequency compensation defaults to
  50 Hz. Both the recording launcher and diagnostic probe use these defaults.
  `--exposure` can adjust exposure within 1..300 units; `--power-line-frequency`
  accepts 0 (disabled), 50 or 60 Hz. These settings do not guarantee 30 FPS.
- Camera subscription depth is eight to absorb short scheduling delays.
- Camera rates above 30 are downsampled using only new arrivals; missing frames
  are never padded by copying the last image.

## Measured Results (2026-09-25)

In this machine's WSL environment, direct input with dynamic rate enabled was
about 19 FPS. Disabling it increased rate to about 27 FPS. The original Fast DDS
ROS test also showed no-data runs and one 3.56 FPS run; a larger SHM segment did
not reliably resolve it. Cyclone DDS with the original 10 ms exposure delivered
26 FPS. With 5 ms exposure and frequency compensation disabled, camera input
was approximately 31-33 FPS and the recorder selected approximately 30 FPS.
These are sequential tests, not an exhaustive factorial isolation of settings.

Diagnostic `camera_diagnostics/1790262336217195108/result.json` recorded 899
frames over approximately 30 seconds (29.986 FPS). The saved MP4 decoded to 899
frames with zero identical adjacent decoded frames, matching the Parquet rows.
Maximum receive gap was 87.6 ms, so strict timing still failed: this is not proof
of perfectly uniform 33.3 ms capture. No frame interpolation or duplication was
used. Previous datasets are not repaired.

`probe_camera_pipeline.py` repeats camera/recorder/save verification in isolated
ROS domain 87 and writes only under `camera_diagnostics`. Its robot states are
synthetic and must not be used for training. It does not control the robot.
Full robot-load recording remains a separate validation step.
