# Third-party notices

TacGELLO builds on the following work. Each component keeps its original license.

| Component | Where | Origin | License |
| --- | --- | --- | --- |
| GELLO | whole project (forked from, then reduced to TacGELLO only) | [wuphilipp/gello_software](https://github.com/wuphilipp/gello_software), © 2023 Philipp Wu | MIT |
| AnySkin QT Py firmware | `gripper/anyskin_qtpy_checked/anyskin_qtpy_checked.ino` | Adapted from the AnySkin `arduino/5X_binary_burst_stream` sketch, [raunaqbhirangi/anyskin](https://github.com/raunaqbhirangi/anyskin) | MIT |
| MLX90393 Arduino driver | `gripper/anyskin_qtpy_checked/MLX90393.{h,cpp}` | `arduino-MLX90393`, © 2016-2018 Theodore C. Yapo (distributed with AnySkin) | MIT, see `gripper/anyskin_qtpy_checked/LICENSE` |
| Pretrained AnySkin slip model | `gripper/models/pollen_anyskin_slip/` | [pollen-robotics/anyskin-slip-detection](https://huggingface.co/pollen-robotics/anyskin-slip-detection), revision `24a3728c35985def720ff292a8a45ef7e8662a64` | Apache-2.0, see `gripper/models/pollen_anyskin_slip/LICENSE` |

Python dependencies installed from PyPI or ROS (`ur_rtde`, `feetech-servo-sdk`,
`pyserial`, `rclpy`, ...) aren't vendored here and are covered by their own licenses.

## Citations

If you use TacGELLO, please also cite GELLO and AnySkin; the BibTeX is in the
[README](README.md#citation).
