# Magnetic direction observations at slip

This read-only telemetry does not change slip thresholds, counters, contact
hold, lever feedback, reinforcement permission, or motor commands.

Each confirmed `gripper_slip_observation` start includes a `direction` object.
For each AnySkin pad it compares the latest 15 raw XYZ components with a sample
approximately 0.2 seconds earlier (accepted window 0.1..0.3 seconds). It records
five magnet delta vectors, their mean XYZ vector, actual sample timestamps and
the dominant signed axis. Opposing changes with coherence below 0.5 are marked
`mixed`. Coherence is mean-vector magnitude divided by average magnet-vector
magnitude, NOT confidence in physical slip. Tiny changes may be sensor noise.

Coordinates are magnetic sensor axes, not robot/tool/world axes. A `-Y` entry
means a negative mean magnetic Y change, NOT that the object moved downward.
Compression, release, gripper motion and magnet deformation can all affect it.
Physical direction stays `uncalibrated`; known-direction trials for each mounted
pad are needed before training/validating a physical direction estimator.

History resets at contact/calibration boundaries, sensor loss and sample gaps.
Missing/invalid/warming-up data is unavailable, not zero. Pads are kept separate
because their mounting orientations may differ. The slip observer's confirmation
time is not an exact physical slip onset timestamp.

The HTML report includes a magnetic-change table, and
`gripper_slip_directions.csv` and the summary JSON contain one row per pad per
confirmed start. Full magnet vectors and sample times remain in diagnostic JSONL;
10 Hz contact sensor records also contain changing direction observations during
the grasp. Full raw recording is not required. Old reports cannot recover missing
direction observations. Existing slip interval plots are unchanged.

Restart the existing tracking-report launch after rebuilding the publisher.
No extra command-line options are needed.
