# TacGELLO gripper

Host-side controller and tools for the TacGELLO two-finger gripper: two STS3215 servos driven by an
ESP32 over Bluetooth, with an [AnySkin](https://github.com/raunaqbhirangi/anyskin) sensor on each fingertip.

The ROS 2 launch file `ur3_tacgello.launch.py` starts `bluetooth_anyskin_gripper_shell.py`
automatically. You only need to run the tools here directly for setup and debugging.

## Contents

| Path | Purpose |
| --- | --- |
| `bluetooth_anyskin_gripper_shell.py` | Main gripper controller. It follows leader trigger width commands from ROS, detects AnySkin contact against an empty-gripper baseline, stops closing on contact, and publishes the contact flag that drives the leader trigger hold. |
| `manual_gripper_shell.py`, `sts3215_bus.py` | AnySkin serial stream decoding, contact detection and STS3215 protocol helpers (also usable as an interactive shell) |
| `gripper_position_tracking.py`, `gripper_motion_timing.py` | Position tracking, contact gating and motion-timing measurement |
| `anyskin_web_viz.py`, `anyskin_raw_recorder.py` | Live browser view of AnySkin contact (default port 8765) and raw XYZ recording |
| `gripper_diagnostics.py`, `audit_gripper_log.py` | Read-only servo register diagnostics and offline audit |
| `gripper_motion_ranges.json` | Gripper servo ranges and saved calibration state (written by the controller; recalibrate for your hardware) |
| `esp32_bluetooth/` | ESP32 gripper firmware and the Windows Bluetooth↔TCP bridge |
| `anyskin_qtpy_checked/` | AnySkin firmware for the Adafruit QT Py M0 boards |
| `firmware_tests/` | Host-side test for the MLX90393 driver |
| `list_ports.py`, `scan_ids.py`, `assign_id.py` | Serial port listing and STS3215 ID setup |
| `anyskin_probe.py`, `anyskin_raw_watch.py`, `anyskin_contact_viz.py`, `bluetooth_position_probe.py` | Sensor and link debugging |
| `gripper_regrasp.py`, `learned_slip_regrasp.py`, `loaded_regrasp.py`, `slip_*.py`, `pretrained_anyskin_slip.py`, `models/` | **Experimental** slip detection / regrasp. Off by default (`slip_regrasp:=false`) and not part of the paper; see [../experimental](../experimental/) |

Docs: [ANYSKIN_RAW_RECORDING.md](ANYSKIN_RAW_RECORDING.md), [MOTOR_DIAGNOSTICS.md](MOTOR_DIAGNOSTICS.md),
[BODY_RESISTANCE.md](BODY_RESISTANCE.md), [esp32_bluetooth/README.md](esp32_bluetooth/README.md),
[anyskin_qtpy_checked/README.md](anyskin_qtpy_checked/README.md).

## Assigning STS3215 IDs

Use this for both the leader servos (IDs 1–7) and the gripper servos (IDs 1–2 on the ESP32 bus).

1. Connect **only one** STS3215 to the USB serial adapter (e.g. URT-2). Check that motor power and
   the adapter's GND are connected. With two servos sharing an ID on the bus, replies collide and
   settings can be written to the wrong servo.
2. Find the port and the current ID (factory default is usually 1; the default baud rate is 1 Mbps):

   ```bash
   python list_ports.py
   python scan_ids.py --port COM3 --start 0 --end 10
   ```

   If nothing is found, scan `--end 253`, or try `--baudrate 115200`.
3. Change the ID, e.g. from 1 to 7:

   ```bash
   python assign_id.py --port COM3 --old-id 1 --new-id 7
   ```

   The script checks that the old ID replies and the new ID is free. To write the EEPROM you must
   type the confirmation exactly as shown (`CHANGE 1 TO 7`). Power-cycle and scan again to check
   that the new ID stuck.

If something fails, check the port name, power, TX/RX direction and baud rate first.

## Tests

```bash
cd gripper
python -m pytest
```

The two tests that load the pretrained slip model also need `torch` and `safetensors`.
