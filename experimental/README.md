# Experimental (not part of the paper)

These scripts and notes cover follow-up work that the TacGELLO paper does **not** evaluate:
slip detection, automatic regrasp/reinforcement and an earlier TCP-force load probe. They're
kept for reference and may change or be removed.

| Path | Purpose |
| --- | --- |
| `REGRASP.md`, `REGRASP_CONTROL_DIAGNOSTICS.md`, `LOADED_REGRASP.md`, `MANUAL_REGRASP_PROBE.md` | Heuristic slip-triggered regrasp and its diagnostics |
| `LEARNED_REGRASP.md`, `PRETRAINED_SLIP.md` | Learned slip scoring with the [Pollen Robotics AnySkin slip model](https://huggingface.co/pollen-robotics/anyskin-slip-detection) (Apache-2.0) |
| `SLIP_DIRECTION_RECORDING.md` | Slip-direction logging |
| `label_anyskin_slip.py` | Label slip events in raw AnySkin recordings |
| `probe_regrasp_step.py` | Single, operator-confirmed unloaded regrasp step |
| `probe_controller_endpoints.py` | Read leader trigger endpoints (no motion commands) |
| `ur3_load_probe.py`, `ur3_load_feedback_preview.py`, `load_feedback_signal.py` | Earlier TCP-force load probe, superseded by the joint-current arm channel in `tacgello_ros` |

The runtime parts of slip/regrasp live in `gripper/`, because the gripper controller imports them.
They're disabled unless you launch with `slip_regrasp:=true`.

Run the tests from this folder:

```bash
cd experimental
python -m pytest
```
