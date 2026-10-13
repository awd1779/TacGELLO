import json
import math
import sys
import time
import threading
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Float64MultiArray, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from .body_resistance import BodyResistance, BodyResistancePair
from .lift_weight import LiftWeight
from .joint_current_cue import JointCurrentCue, JointCurrentPair


def _add_local_scservo_sdk_path() -> None:
    roots = [Path.cwd(), Path(__file__).resolve()]
    for root in list(roots):
        roots.extend(root.parents)

    for root in roots:
        site_packages = root / ".venv" / "Lib" / "site-packages"
        if (site_packages / "scservo_sdk").exists():
            site_packages_text = str(site_packages)
            if site_packages_text not in sys.path:
                sys.path.insert(0, site_packages_text)
            return


_add_local_scservo_sdk_path()

try:
    from scservo_sdk import GroupSyncRead, PacketHandler, PortHandler
except ImportError:
    GroupSyncRead = None
    PacketHandler = None
    PortHandler = None

try:
    import serial
except ImportError as exc:
    raise ImportError(
        "pyserial is required. Install it with `pip install pyserial` before "
        "running tacgello_ros."
    ) from exc

try:
    import rtde_control
    import rtde_receive
except ImportError:
    rtde_control = None
    rtde_receive = None


class STS3215LeaderReader:
    """Reads STS3215 positions and converts them to UR joint radians."""

    def __init__(
        self,
        port: str,
        baudrate: int,
        offset_file: str,
        read_servo_ids: Optional[List[int]] = None,
        gripper_servo_id: int = 7,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.offset_file = self._resolve_offset_file(offset_file)

        self.gripper_servo_id = int(gripper_servo_id)
        self.servo_ids = (
            [int(servo_id) for servo_id in read_servo_ids]
            if read_servo_ids
            else [1, 2, 3, 4, 5, 6, self.gripper_servo_id]
        )
        self.servo_ids = list(dict.fromkeys(self.servo_ids))
        tracked_servo_ids = list(dict.fromkeys([1, 2, 3, 4, 5, 6, self.gripper_servo_id]))
        self.present_position_addr = 56
        self.read_length = 2
        self.torque_enable_addr = 40
        self.acceleration_addr = 41

        self.signs = {
            1: -1,
            2: -1,
            3: 1,
            4: -1,
            5: -1,
            6: -1,
            7: -1,
        }

        self.count_to_rad = 2 * math.pi / 4096
        self.joint_limits = {
            1: (-2 * math.pi, 2 * math.pi),
            2: (-2 * math.pi, 2 * math.pi),
            3: (-2 * math.pi, 2 * math.pi),
            4: (-math.pi, math.pi),
            5: (-math.pi, math.pi),
            6: (-2 * math.pi, 2 * math.pi),
            7: (-1.5, 1.5),
        }
        self.max_step_rad = 1.50

        with open(self.offset_file, "r", encoding="utf-8") as file:
            self.offsets = json.load(file)

        self.use_scservo_sdk = (
            GroupSyncRead is not None
            and PacketHandler is not None
            and PortHandler is not None
        )
        if not self.use_scservo_sdk:
            raise ImportError(
                "scservo_sdk is required for synchronized STS3215 reads. "
                "Install it with `python3 -m pip install scservo_sdk`."
            )

        self.port_handler = None
        self.packet_handler = None
        self.group_sync_read = None
        self.serial_port = None

        self.port_handler = PortHandler(self.port)
        self.packet_handler = PacketHandler(0)

        if not self.port_handler.openPort():
            raise RuntimeError(f"Failed to open STS3215 port: {self.port}")

        if not self.port_handler.setBaudRate(self.baudrate):
            self.port_handler.closePort()
            raise RuntimeError(f"Failed to set STS3215 baudrate: {self.baudrate}")

        self.group_sync_read = GroupSyncRead(
            self.port_handler,
            self.packet_handler,
            self.present_position_addr,
            self.read_length,
        )

        for servo_id in self.servo_ids:
            if not self.group_sync_read.addParam(servo_id):
                raise RuntimeError(f"Failed to add servo ID {servo_id} to sync read")

        time.sleep(0.2)

        self.previous_radians: Dict[int, Optional[float]] = {
            servo_id: None for servo_id in tracked_servo_ids
        }
        self.unclamped_radians: Dict[int, Optional[float]] = {
            servo_id: None for servo_id in tracked_servo_ids
        }
        self.previous_raw_positions: Dict[int, Optional[int]] = {
            servo_id: None for servo_id in tracked_servo_ids
        }
        self.latest_gripper_raw: Optional[int] = None

    def _resolve_offset_file(self, offset_file: str) -> Path:
        offset_path = Path(offset_file).expanduser()
        if offset_path.is_absolute() and offset_path.exists():
            return offset_path

        candidates = [
            Path.cwd() / offset_path,
            Path.cwd().parent / offset_path,
            Path.cwd().parent.parent / offset_path,
            Path.cwd() / "configs" / offset_path,
            Path.cwd().parent / "configs" / offset_path,
        ]

        for candidate in candidates:
            if candidate.exists():
                return candidate

        searched = "\n".join(str(candidate) for candidate in candidates)
        raise FileNotFoundError(
            f"Could not find offset file '{offset_file}'. Searched:\n{searched}"
        )

    def sync_read_positions(self) -> Dict[int, Optional[int]]:
        self.group_sync_read.txRxPacket()

        positions: Dict[int, Optional[int]] = {}
        for servo_id in self.servo_ids:
            available = self.group_sync_read.isAvailable(
                servo_id,
                self.present_position_addr,
                self.read_length,
            )
            if available:
                positions[servo_id] = self.group_sync_read.getData(
                    servo_id,
                    self.present_position_addr,
                    self.read_length,
                )
            else:
                positions[servo_id] = None

        return positions

    def checksum(self, packet_body: List[int]) -> int:
        return (~sum(packet_body)) & 0xFF

    def make_read_packet(self, servo_id: int, address: int, read_length: int) -> bytes:
        instruction = 0x02
        length = 0x04
        body = [servo_id, length, instruction, address, read_length]
        return bytes([0xFF, 0xFF] + body + [self.checksum(body)])

    def read_position_with_serial(self, servo_id: int) -> Optional[int]:
        packet = self.make_read_packet(
            servo_id,
            self.present_position_addr,
            self.read_length,
        )

        self.serial_port.reset_input_buffer()
        self.serial_port.write(packet)
        time.sleep(0.002)

        response = self.serial_port.read(20)
        if len(response) >= 7 and response[0] == 0xFF and response[1] == 0xFF:
            low = response[5]
            high = response[6]
            return low + (high << 8)

        return None

    def wrapped_delta(self, current: int, offset: int) -> int:
        return (current - offset + 2048) % 4096 - 2048

    def clamp(self, value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(value, max_value))

    def limit_step(self, servo_id: int, new_value: float) -> float:
        previous = self.previous_radians.get(servo_id)
        if previous is None:
            return new_value

        difference = new_value - previous
        if difference > self.max_step_rad:
            return previous + self.max_step_rad
        if difference < -self.max_step_rad:
            return previous - self.max_step_rad

        return new_value

    def position_to_radian(self, servo_id: int, current_pos: int) -> float:
        offset = self.offsets.get(str(servo_id))
        if offset is None:
            previous = self.previous_radians.get(servo_id)
            return 0.0 if previous is None else previous

        previous_raw = self.previous_raw_positions.get(servo_id)
        previous_unclamped_rad = self.unclamped_radians.get(servo_id)

        if previous_raw is None or previous_unclamped_rad is None:
            raw_delta = self.wrapped_delta(current_pos, offset)
            unclamped_radian = raw_delta * self.signs[servo_id] * self.count_to_rad
        else:
            raw_step = self.wrapped_delta(current_pos, previous_raw)
            unclamped_radian = (
                previous_unclamped_rad
                + raw_step * self.signs[servo_id] * self.count_to_rad
            )

        min_limit, max_limit = self.joint_limits[servo_id]
        radian = self.clamp(unclamped_radian, min_limit, max_limit)
        radian = self.limit_step(servo_id, radian)

        self.previous_raw_positions[servo_id] = current_pos
        self.unclamped_radians[servo_id] = unclamped_radian
        self.previous_radians[servo_id] = radian
        return radian

    def read_joints_and_gripper(self) -> Tuple[List[float], float]:
        joints: List[float] = []
        self.fresh_raw_positions = {}
        positions = self.sync_read_positions()
        self.fresh_raw_positions = positions

        for servo_id in range(1, 7):
            current_pos = positions.get(servo_id)
            if current_pos is None:
                previous = self.previous_radians.get(servo_id)
                radian = 0.0 if previous is None else previous
            else:
                radian = self.position_to_radian(servo_id, current_pos)

            joints.append(radian)

        gripper_pos = positions.get(self.gripper_servo_id)
        if gripper_pos is None:
            previous = self.previous_radians.get(self.gripper_servo_id)
            gripper = 0.0 if previous is None else previous
        else:
            self.latest_gripper_raw = gripper_pos
            gripper = self.position_to_radian(self.gripper_servo_id, gripper_pos)

        return joints, gripper

    def _write_gripper_registers(self, address, values):
        # A single-ID sync write produces no write ACK, regardless of return level.
        parameters = [self.gripper_servo_id] + list(values)
        return self.packet_handler.syncWriteTxOnly(
            self.port_handler, address, len(values), parameters, len(parameters))

    def hold_gripper_at_raw(
        self,
        raw_position: int,
        speed: int,
        acceleration: int,
        output_limit: int = 300,
    ) -> Tuple[bool, str]:
        raw_position = max(0, min(4095, int(raw_position)))
        speed = max(1, min(4095, int(speed)))
        acceleration = max(0, min(255, int(acceleration)))
        output_limit = int(output_limit)
        if not 1 <= output_limit <= 1000:
            return False, 'Hold output limit must be 1..1000'

        torque_result = self._write_gripper_registers(self.torque_enable_addr, [0])
        if torque_result != 0:
            return False, f"Torque disable failed: communication={torque_result}"

        registers, result, error = self._read_gripper_registers(40, 10)
        if result != 0 or error != 0 or registers is None or registers[0] != 0:
            self.release_gripper_hold()
            return False, 'Torque-off/output-limit readback failed; hold not enabled'
        existing_limit = registers[8] | (registers[9] << 8)
        if not 1 <= existing_limit <= 1000:
            return False, 'Invalid existing output limit; hold not enabled'
        output_limit = min(existing_limit, output_limit)

        command = [
            acceleration,
            raw_position & 0xFF,
            (raw_position >> 8) & 0xFF,
            0,
            0,
            speed & 0xFF,
            (speed >> 8) & 0xFF,
            output_limit & 0xFF,
            (output_limit >> 8) & 0xFF,
        ]
        command_result = self._write_gripper_registers(self.acceleration_addr, command)
        if command_result != 0:
            return False, f"Hold target failed: communication={command_result}"

        # Sync write has no ACK. Allow one read-only recheck for device write
        # processing; never resend the target or enable torque on a mismatch.
        expected = [0] + command
        for read_attempt in range(2):
            if read_attempt:
                time.sleep(.003)
            registers, result, error = self._read_gripper_registers(40, 10)
            if error != 0 or (result == 0 and registers == expected):
                break
        # Some observed replies have the exact goal/limit but torque already on.
        # Restore and verify torque-off; do not accept an unexplained state or
        # replay the position command. All register checks still apply below.
        if result == 0 and error == 0 and registers == [1] + command:
            disabled = self._write_gripper_registers(self.torque_enable_addr, [0])
            if disabled != 0:
                self.release_gripper_hold()
                return False, 'Torque-off restore failed; hold not enabled'
            registers, result, error = self._read_gripper_registers(40, 10)
        if result != 0 or error != 0 or registers != expected:
            self.release_gripper_hold()
            return False, (f'Hold target/output-limit readback failed; hold not enabled; '
                           f'expected={expected}, received={registers}, communication={result}, servo_error={error}')
        self.last_gripper_hold_output_limit = output_limit

        enable_result = self._write_gripper_registers(self.torque_enable_addr, [1])
        if enable_result != 0:
            self.release_gripper_hold()
            return False, f"Torque enable failed: communication={enable_result}"
        enabled, result, error = self._read_gripper_torque_enabled()
        if result != 0 or error != 0 or enabled != 1:
            released, release_detail = self.release_gripper_hold()
            return False, (
                f"Torque enable readback failed: value={enabled}, "
                f"communication={result}, servo_error={error}; "
                f"release={released} ({release_detail})"
            )
        return True, "ok"

    def _read_gripper_torque_enabled(self):
        values, result, error = self._read_gripper_registers(self.torque_enable_addr, 1)
        return (values[0] if values is not None else None), result, error

    def _read_gripper_registers(self, address, length):
        # SDK read1ByteTxRx accepts the first parameter even from a longer
        # response. Reject delayed position replies instead of treating them
        # as a torque-enable byte. No motion command is retried here.
        port = self.port_handler
        port.ser.reset_input_buffer()
        result = self.packet_handler.readTx(port, self.gripper_servo_id,
                                            address, length)
        if result != 0:
            return None, result, 0
        port.setPacketTimeoutMillis(30)
        deadline = time.monotonic() + .03
        try:
            while time.monotonic() < deadline:
                packet, result = self.packet_handler.rxPacket(port)
                if result != 0:
                    return None, result, 0
                if (len(packet) == length + 6 and packet[2] == self.gripper_servo_id
                        and packet[3] == length + 2):
                    return packet[5:5 + length], 0, packet[4]
            return None, -3001, 0
        finally:
            port.is_using = False

    def release_gripper_hold(self) -> Tuple[bool, str]:
        result = self._write_gripper_registers(self.torque_enable_addr, [0])
        if result != 0:
            return False, f"Torque release failed: communication={result}"
        return True, "ok"

    def close(self) -> None:
        if self.group_sync_read is not None:
            self.group_sync_read.clearParam()
        if self.port_handler is not None:
            self.port_handler.closePort()
        if self.serial_port is not None:
            self.serial_port.close()


class TacGelloLeaderPublisher(Node):
    def __init__(self) -> None:
        super().__init__("leader_publisher")

        self.declare_parameter("port", "COM3")
        self.declare_parameter("body_resistance", False)
        self.declare_parameter("body_resistance_bench", False)
        self.declare_parameter("body_weight_feedback", False)
        self.declare_parameter("body_resistance_source", "weight")
        self.declare_parameter("body_resistance_servo_id", 2)
        self.declare_parameter("body_resistance_both", False)
        self.declare_parameter("body_resistance_wrist", False)
        self.declare_parameter("body_resistance_continuous", False)
        self.declare_parameter("baudrate", 1000000)
        self.declare_parameter("offset_file", "servo_offsets.json")
        self.declare_parameter("read_servo_ids", [1, 2, 3, 4, 5, 6, 7])
        self.declare_parameter("gripper_servo_id", 7)
        self.declare_parameter("control_mode", "ros2_position")
        self.declare_parameter("robot_ip", "192.168.0.119")
        self.declare_parameter("rtde_velocity", 0.5)
        self.declare_parameter("rtde_acceleration", 0.5)
        self.declare_parameter("rtde_dt", 1.0 / 500.0)
        self.declare_parameter("rtde_lookahead_time", 0.2)
        self.declare_parameter("rtde_gain", 100)
        self.declare_parameter("publish_rate_hz", 25.0)
        self.declare_parameter("frame_id", "base")
        self.declare_parameter("joint_state_topic", "tacgello/joint_states")
        self.declare_parameter(
            "trajectory_topic",
            "scaled_joint_trajectory_controller/joint_trajectory",
        )
        self.declare_parameter(
            "position_command_topic",
            "forward_position_controller/commands",
        )
        self.declare_parameter("gripper_topic", "tacgello/gripper_position")
        self.declare_parameter("gripper_raw_topic", "tacgello/gripper_raw")
        self.declare_parameter(
            "gripper_command_topic",
            "tacgello/gripper_width_command",
        )
        self.declare_parameter(
            "gripper_command_topic_alt",
            "finger_width_controller/commands",
        )
        self.declare_parameter(
            "gripper_trajectory_topic",
            "finger_width_trajectory_controller/joint_trajectory",
        )
        self.declare_parameter("publish_trajectory", True)
        self.declare_parameter("publish_position_command", False)
        self.declare_parameter("publish_gripper_command", True)
        self.declare_parameter("publish_gripper_trajectory", True)
        self.declare_parameter("trajectory_time_from_start_sec", 0.08)
        self.declare_parameter("command_smoothing_alpha", 0.25)
        self.declare_parameter("max_joint_speed_rad_per_sec", 0.5)
        self.declare_parameter("max_joint_delta_rad_per_cycle", 0.0)
        self.declare_parameter("command_deadband_rad", 0.002)
        self.declare_parameter("gripper_joint_name", "finger_width")
        self.declare_parameter("gripper_min_rad", -1.5)
        self.declare_parameter("gripper_max_rad", 1.5)
        self.declare_parameter("gripper_min_width", 0.0)
        self.declare_parameter("gripper_max_width", 0.1)
        self.declare_parameter("gripper_min_raw", 3400)
        self.declare_parameter("gripper_max_raw", 3800)
        self.declare_parameter("gripper_encoder_ticks", 4096)
        self.declare_parameter("gripper_wrap_open_raw", -1)
        self.declare_parameter("gripper_control_mode", "continuous")
        self.declare_parameter("gripper_discrete_start_open", True)
        self.declare_parameter("gripper_discrete_close_threshold", 0.35)
        self.declare_parameter("gripper_discrete_open_threshold", 0.65)
        self.declare_parameter("gripper_discrete_step_m", 0.0)
        self.declare_parameter("gripper_command_deadband_m", 0.004)
        self.declare_parameter("gripper_command_publish_rate_hz", 5.0)
        self.declare_parameter("gripper_command_publish_deadband_m", 0.006)
        self.declare_parameter("gripper_smoothing_alpha", 0.45)
        self.declare_parameter("gripper_reversal_deadband_m", 0.008)
        self.declare_parameter("gripper_close_latch_release_m", 0.025)
        self.declare_parameter("gripper_open_confirm_cycles", 5)
        self.declare_parameter("enable_gripper_haptic_feedback", False)
        self.declare_parameter(
            "gripper_haptic_feedback_topic", "tacgello/gripper_contact_hold"
        )
        self.declare_parameter("gripper_haptic_release_delta", 0.015)
        self.declare_parameter("gripper_haptic_hold_speed", 120)
        self.declare_parameter("gripper_haptic_hold_acceleration", 20)
        self.declare_parameter("gripper_haptic_hold_output_limit", 150)
        self.declare_parameter("gripper_haptic_hold_duration_sec", 3.0)
        self.declare_parameter("gripper_haptic_feedback_timeout_sec", 1.0)
        self.declare_parameter("invert_gripper", False)
        self.declare_parameter("align_to_robot_on_start", True)
        self.declare_parameter("robot_joint_state_topic", "joint_states")
        self.declare_parameter(
            "joint_names",
            [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
        )
        self.declare_parameter("joint_position_offsets", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.joint_names = list(self.get_parameter("joint_names").value)
        if len(self.joint_names) != 6:
            raise ValueError("joint_names must contain exactly 6 UR joint names")

        self.joint_position_offsets = [
            float(value) for value in self.get_parameter("joint_position_offsets").value
        ]
        if len(self.joint_position_offsets) != 6:
            raise ValueError("joint_position_offsets must contain exactly 6 values")

        self.control_mode = str(self.get_parameter("control_mode").value)
        self.body_resistance_bench = bool(self.get_parameter("body_resistance_bench").value)
        if self.body_resistance_bench:
            self.control_mode = "disabled"
        self.robot_ip = str(self.get_parameter("robot_ip").value)
        self.rtde_velocity = float(self.get_parameter("rtde_velocity").value)
        self.rtde_acceleration = float(self.get_parameter("rtde_acceleration").value)
        self.rtde_dt = float(self.get_parameter("rtde_dt").value)
        self.rtde_lookahead_time = float(
            self.get_parameter("rtde_lookahead_time").value
        )
        self.rtde_gain = int(self.get_parameter("rtde_gain").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.publish_trajectory = bool(self.get_parameter("publish_trajectory").value)
        self.publish_position_command = bool(
            self.get_parameter("publish_position_command").value
        )
        self.publish_gripper_command = bool(
            self.get_parameter("publish_gripper_command").value
        )
        self.publish_gripper_trajectory = bool(
            self.get_parameter("publish_gripper_trajectory").value
        )
        self.trajectory_time_from_start_sec = float(
            self.get_parameter("trajectory_time_from_start_sec").value
        )
        self.command_smoothing_alpha = self.clamp(
            float(self.get_parameter("command_smoothing_alpha").value),
            0.0,
            1.0,
        )
        self.max_joint_speed_rad_per_sec = max(
            0.0,
            float(self.get_parameter("max_joint_speed_rad_per_sec").value),
        )
        self.max_joint_delta_rad_per_cycle = max(
            0.0,
            float(self.get_parameter("max_joint_delta_rad_per_cycle").value),
        )
        self.command_deadband_rad = max(
            0.0,
            float(self.get_parameter("command_deadband_rad").value),
        )
        self.gripper_joint_name = str(self.get_parameter("gripper_joint_name").value)
        self.gripper_min_rad = float(self.get_parameter("gripper_min_rad").value)
        self.gripper_max_rad = float(self.get_parameter("gripper_max_rad").value)
        self.gripper_min_width = float(self.get_parameter("gripper_min_width").value)
        self.gripper_max_width = float(self.get_parameter("gripper_max_width").value)
        self.gripper_min_raw = int(self.get_parameter("gripper_min_raw").value)
        self.gripper_max_raw = int(self.get_parameter("gripper_max_raw").value)
        self.gripper_encoder_ticks = max(
            1,
            int(self.get_parameter("gripper_encoder_ticks").value),
        )
        self.gripper_wrap_open_raw = int(
            self.get_parameter("gripper_wrap_open_raw").value
        )
        self.gripper_control_mode = str(
            self.get_parameter("gripper_control_mode").value
        ).lower()
        self.gripper_discrete_start_open = bool(
            self.get_parameter("gripper_discrete_start_open").value
        )
        self.gripper_discrete_close_threshold = self.clamp(
            float(self.get_parameter("gripper_discrete_close_threshold").value),
            0.0,
            1.0,
        )
        self.gripper_discrete_open_threshold = self.clamp(
            float(self.get_parameter("gripper_discrete_open_threshold").value),
            0.0,
            1.0,
        )
        self.gripper_discrete_step_m = max(
            0.0,
            float(self.get_parameter("gripper_discrete_step_m").value),
        )
        self.gripper_command_deadband_m = max(
            0.0,
            float(self.get_parameter("gripper_command_deadband_m").value),
        )
        self.gripper_command_publish_rate_hz = max(
            0.0,
            float(self.get_parameter("gripper_command_publish_rate_hz").value),
        )
        self.gripper_command_publish_deadband_m = max(
            0.0,
            float(self.get_parameter("gripper_command_publish_deadband_m").value),
        )
        self.gripper_smoothing_alpha = self.clamp(
            float(self.get_parameter("gripper_smoothing_alpha").value),
            0.0,
            1.0,
        )
        self.gripper_reversal_deadband_m = max(
            0.0,
            float(self.get_parameter("gripper_reversal_deadband_m").value),
        )
        self.gripper_close_latch_release_m = max(
            0.0,
            float(self.get_parameter("gripper_close_latch_release_m").value),
        )
        self.gripper_open_confirm_cycles = max(
            1,
            int(self.get_parameter("gripper_open_confirm_cycles").value),
        )
        self.enable_gripper_haptic_feedback = bool(
            self.get_parameter("enable_gripper_haptic_feedback").value
        )
        self.gripper_haptic_feedback_topic = str(
            self.get_parameter("gripper_haptic_feedback_topic").value
        )
        self.gripper_haptic_release_delta = max(
            0.001,
            float(self.get_parameter("gripper_haptic_release_delta").value),
        )
        self.gripper_haptic_hold_speed = max(
            1,
            int(self.get_parameter("gripper_haptic_hold_speed").value),
        )
        self.gripper_haptic_hold_acceleration = max(
            0,
            int(self.get_parameter("gripper_haptic_hold_acceleration").value),
        )
        self.gripper_haptic_hold_output_limit = int(
            self.get_parameter("gripper_haptic_hold_output_limit").value)
        if not 1 <= self.gripper_haptic_hold_output_limit <= 1000:
            raise ValueError('gripper_haptic_hold_output_limit must be 1..1000')
        self.gripper_haptic_hold_duration_sec = max(
            0.0,
            float(self.get_parameter("gripper_haptic_hold_duration_sec").value),
        )
        self.gripper_haptic_feedback_timeout_sec = max(
            0.2,
            float(
                self.get_parameter("gripper_haptic_feedback_timeout_sec").value
            ),
        )
        self.gripper_haptic_contact_signal = False
        self.gripper_haptic_hold_active = False
        self.gripper_haptic_rearm_blocked = False
        self.gripper_haptic_hold_raw: Optional[int] = None
        self.gripper_haptic_hold_normalized: Optional[float] = None
        self.gripper_haptic_hold_started_time = None
        self.gripper_haptic_last_feedback_time = self.get_clock().now()
        self.invert_gripper = bool(self.get_parameter("invert_gripper").value)
        self.align_to_robot_on_start = bool(
            self.get_parameter("align_to_robot_on_start").value
        )
        self.alignment_offsets = [0.0] * 6
        self.alignment_ready = not self.align_to_robot_on_start
        self.latest_robot_joints: Optional[List[float]] = None
        self.smoothed_target_joints: Optional[List[float]] = None
        self.filtered_commanded_joints: Optional[List[float]] = None
        self.filtered_gripper_width: Optional[float] = None
        self.last_gripper_command_width: Optional[float] = None
        self.last_published_gripper_command_width: Optional[float] = None
        self.last_gripper_command_publish_time = self.get_clock().now()
        self.gripper_close_latch_width: Optional[float] = None
        self.gripper_open_confirm_count = 0
        self.last_gripper_direction = 0
        self.discrete_gripper_state: Optional[int] = None
        self.discrete_gripper_width: Optional[float] = None
        self.discrete_gripper_start_open_pending = self.gripper_discrete_start_open
        self.last_command_time = self.get_clock().now()
        self.rtde_control_interface = None
        self.rtde_receive_interface = None

        if self.control_mode == "rtde_servoj":
            if rtde_control is None or rtde_receive is None:
                raise ImportError(
                    "ur_rtde is required for control_mode=rtde_servoj. "
                    "Install it with `python3 -m pip install ur-rtde`."
                )

            self.get_logger().info(
                f"Connecting directly to UR RTDE at {self.robot_ip} for servoJ control."
            )
            self.rtde_control_interface = rtde_control.RTDEControlInterface(
                self.robot_ip
            )
            self.rtde_receive_interface = rtde_receive.RTDEReceiveInterface(
                self.robot_ip
            )
            try:
                self.rtde_control_interface.endFreedriveMode()
            except Exception as exc:
                self.get_logger().warn(f"endFreedriveMode skipped: {exc}")

        self.reader = STS3215LeaderReader(
            port=str(self.get_parameter("port").value),
            baudrate=int(self.get_parameter("baudrate").value),
            offset_file=str(self.get_parameter("offset_file").value),
            read_servo_ids=[
                int(value) for value in self.get_parameter("read_servo_ids").value
            ],
            gripper_servo_id=int(self.get_parameter("gripper_servo_id").value),
        )
        if self.enable_gripper_haptic_feedback:
            released, detail = self.reader.release_gripper_hold()
            if not released:
                self.get_logger().warn(
                    "Could not release a previous leader gripper lever hold at "
                    f"startup: {detail}"
                )

        self.body_resistance = None
        self.body_resistance_subscription = None
        self.body_contact = False
        self.body_contact_time = None
        self.lift_weight = None
        self.last_weight_status = 0.0
        cue_source = str(self.get_parameter('body_resistance_source').value)
        cue_servo_id = int(self.get_parameter('body_resistance_servo_id').value)
        cue_limit = 40 if cue_source == 'current' else 10
        cue_both = bool(self.get_parameter('body_resistance_both').value)
        cue_wrist = bool(self.get_parameter('body_resistance_wrist').value)
        if cue_wrist and (not cue_both or cue_source != 'current'):
            raise ValueError('Wrist trial requires both body joints and current source')
        cue_continuous = bool(self.get_parameter('body_resistance_continuous').value)
        if cue_both and cue_source != 'current':
            raise ValueError('Dual body resistance requires current source')
        if cue_source not in ('weight', 'current') or cue_servo_id not in (2, 3):
            raise ValueError('Body source must be weight/current and servo ID 2/3')
        if self.body_resistance_bench and cue_source == 'current':
            raise ValueError('Current cue requires live RTDE, not bench mode')
        if bool(self.get_parameter("body_resistance").value):
            self.body_resistance = BodyResistance(
                self.reader.packet_handler, self.reader.port_handler, servo_id=cue_servo_id,
                output_limit=cue_limit, continuous=cue_continuous)
            if cue_both:
                self.body_resistance = BodyResistancePair(self.reader.packet_handler, self.reader.port_handler,
                                                         output_limit=cue_limit, continuous=cue_continuous, wrist=cue_wrist)
            self.get_logger().warn(
                f"Experimental body resistance enabled: ID {'2,3' if cue_both else cue_servo_id}, limit register {cue_limit}, "
                "2-count position lag. Not weight measurement or passive damping. "
                "Cable loss/process kill can prevent torque release.")

        if not self.body_resistance_bench and (
                self.body_resistance is not None
                or bool(self.get_parameter("body_weight_feedback").value)):
            self.lift_weight = JointCurrentCue(cue_servo_id, auto_tare=True, output_limit=cue_limit,
                                               current_range_a=.35, contrast=True) if cue_source == 'current' else LiftWeight()
            if cue_both:
                self.lift_weight = JointCurrentPair(auto_tare=True, output_limit=cue_limit, current_range_a=.35, contrast=True, wrist=cue_wrist)
            self.weight_status_publisher = self.create_publisher(String, "/tacgello/lift_weight", 1)
            self.weight_tare_service = self.create_service(
                Trigger, "/tacgello/tare_body_current" if cue_source == 'current' else "/tacgello/tare_lift_weight",
                self.tare_lift_weight)
            self.get_logger().warn(
                "Current cue: startup baseline is automatic; keep EMPTY tool stationary. "
                "No resistance until ready; no posture compensation or mass estimate."
                if cue_source == 'current' else
                "Apparent-load estimate enabled: upright base Z, 2.4kg cue saturation. "
                "Tare empty tool first; this is not calibrated mass or a payload safety limit.")
        if self.body_resistance is not None or self.lift_weight is not None:
            self.body_resistance_subscription = self.create_subscription(
                Bool, self.gripper_haptic_feedback_topic, self.body_contact_feedback, 1)

        self.joint_state_publisher = self.create_publisher(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            10,
        )
        self.robot_joint_state_publisher = self.create_publisher(
            JointState,
            str(self.get_parameter("robot_joint_state_topic").value),
            10,
        )
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            str(self.get_parameter("trajectory_topic").value),
            10,
        )
        self.position_command_publisher = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("position_command_topic").value),
            10,
        )
        self.gripper_publisher = self.create_publisher(
            Float32,
            str(self.get_parameter("gripper_topic").value),
            10,
        )
        self.gripper_raw_publisher = self.create_publisher(
            Float32,
            str(self.get_parameter("gripper_raw_topic").value),
            10,
        )
        self.gripper_command_publisher = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("gripper_command_topic").value),
            10,
        )
        self.gripper_command_alt_publisher = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("gripper_command_topic_alt").value),
            10,
        )
        self.gripper_trajectory_publisher = self.create_publisher(
            JointTrajectory,
            str(self.get_parameter("gripper_trajectory_topic").value),
            10,
        )
        self.robot_joint_state_subscription = self.create_subscription(
            JointState,
            str(self.get_parameter("robot_joint_state_topic").value),
            self.robot_joint_state_callback,
            10,
        )
        self.gripper_haptic_feedback_subscription = None
        self.haptic_mailbox_lock = threading.Lock()
        self.haptic_mailbox = None
        self.haptic_pending_clear = False
        self.haptic_feedback_group = MutuallyExclusiveCallbackGroup()
        if self.enable_gripper_haptic_feedback:
            self.gripper_haptic_feedback_subscription = self.create_subscription(
                Bool,
                self.gripper_haptic_feedback_topic,
                self.gripper_haptic_feedback_callback,
                10,
                callback_group=self.haptic_feedback_group,
            )

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_state)

        self.get_logger().info(
            "Publishing STS3215 leader as UR3 joint states"
            f" at {publish_rate_hz:.1f} Hz."
        )
        self.get_logger().info(
            "STS3215 GroupSyncRead enabled for servo IDs "
            f"{self.reader.servo_ids} at {self.reader.baudrate} baud."
        )
        if self.control_mode == "rtde_servoj":
            self.get_logger().info(
                "Using RTDE servoJ control "
                f"(dt={self.rtde_dt:.4f}, lookahead={self.rtde_lookahead_time:.3f}, "
                f"gain={self.rtde_gain})."
            )
        if self.enable_gripper_haptic_feedback:
            self.get_logger().info(
                "Leader gripper contact feedback enabled on "
                f"{self.gripper_haptic_feedback_topic}. Servo ID "
                f"{self.reader.gripper_servo_id} will hold at confirmed contact and "
                f"release after {self.gripper_haptic_hold_duration_sec:.1f}s or "
                "when the lever moves toward open."
            )

    def clamp(self, value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(value, max_value))

    def robot_joint_state_callback(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        if all(joint_name in positions for joint_name in self.joint_names):
            self.latest_robot_joints = [
                float(positions[joint_name]) for joint_name in self.joint_names
            ]

    def gripper_haptic_feedback_callback(self, msg: Bool) -> None:
        # This callback owns no serial operations. It can run while the
        # default callback group is servicing the robot/servo bus.
        with self.haptic_mailbox_lock:
            self.haptic_mailbox = (bool(msg.data), self.get_clock().now())
            if not msg.data:
                self.haptic_pending_clear = True

    def process_gripper_haptic_feedback(self) -> None:
        with self.haptic_mailbox_lock:
            pending = self.haptic_mailbox
            clear = self.haptic_pending_clear
            self.haptic_pending_clear = False
        if pending is None:
            return
        contact, received = pending
        self.gripper_haptic_last_feedback_time = received
        age = (self.get_clock().now() - received).nanoseconds * 1e-9
        if clear or age > self.gripper_haptic_feedback_timeout_sec:
            self._apply_gripper_haptic_contact(False)
        if age <= self.gripper_haptic_feedback_timeout_sec:
            self._apply_gripper_haptic_contact(contact)

    def _apply_gripper_haptic_contact(self, contact) -> None:
        previous_contact = self.gripper_haptic_contact_signal
        self.gripper_haptic_contact_signal = contact

        if not contact:
            self.gripper_haptic_rearm_blocked = False
            self.release_gripper_haptic_hold("AnySkin contact cleared")
            return
        if (
            previous_contact
            or self.gripper_haptic_hold_active
            or self.gripper_haptic_rearm_blocked
        ):
            return

        raw_position = self.reader.latest_gripper_raw
        if raw_position is None:
            self.gripper_haptic_contact_signal = False
            self.get_logger().warn(
                "AnySkin contact arrived before leader gripper servo ID "
                f"{self.reader.gripper_servo_id} had a valid position; waiting for "
                "the next feedback message."
            )
            return

        ok, detail = self.reader.hold_gripper_at_raw(
            raw_position,
            self.gripper_haptic_hold_speed,
            self.gripper_haptic_hold_acceleration,
            self.gripper_haptic_hold_output_limit,
        )
        if not ok:
            self.gripper_haptic_rearm_blocked = True
            self.gripper_haptic_fault = detail
            self.publish_gripper_haptic_status('hold_failed')
            self.get_logger().error(f"Failed to lock leader gripper lever: {detail}")
            return

        self.gripper_haptic_hold_active = True
        self.gripper_haptic_hold_raw = raw_position
        self.gripper_haptic_hold_normalized = self.gripper_raw_to_normalized(
            raw_position
        )
        self.gripper_haptic_hold_started_time = self.get_clock().now()
        self.gripper_haptic_fault = ''
        self.publish_gripper_haptic_status('hold_verified')
        self.get_logger().info(
            "AnySkin contact locked leader gripper servo ID "
            f"{self.reader.gripper_servo_id} at raw={raw_position}. Push the lever "
            f"toward open to release it. Hold duration: "
            f"{self.gripper_haptic_hold_duration_sec:.1f}s (0 = while contact remains). "
            f"Output limit raw={self.reader.last_gripper_hold_output_limit}/1000 (not measured force)."
        )

    def release_gripper_haptic_hold(
        self,
        reason: str,
        block_rearm: bool = False,
    ) -> None:
        if not self.gripper_haptic_hold_active:
            return
        ok, detail = self.reader.release_gripper_hold()
        if not ok:
            self.gripper_haptic_fault = detail
            self.publish_gripper_haptic_status('release_failed')
            self.get_logger().warn(
                f"Failed to release leader gripper lever hold: {detail}",
                throttle_duration_sec=1.0,
            )
            return

        self.gripper_haptic_hold_active = False
        self.gripper_haptic_rearm_blocked = block_rearm
        self.gripper_haptic_hold_raw = None
        self.gripper_haptic_hold_normalized = None
        self.gripper_haptic_hold_started_time = None
        self.gripper_haptic_fault = ''
        self.publish_gripper_haptic_status('release_commanded', reason)
        self.get_logger().info(f"Leader gripper lever hold released: {reason}.")

    def publish_gripper_haptic_status(self, event='heartbeat', reason='') -> None:
        now = time.monotonic_ns()
        if event == 'heartbeat' and now-getattr(self, 'haptic_status_ns', 0) < 100_000_000:
            return
        self.haptic_status_ns = now
        try:
            if not hasattr(self, 'timing_publisher'):
                self.timing_publisher = self.create_publisher(String, '/tacgello/control_timing', 100)
            message = String()
            message.data = json.dumps(dict(component='gripper_lever_feedback', observed_pc_ns=now,
                event=event, reason=reason, enabled=self.enable_gripper_haptic_feedback,
                contact_requested=self.gripper_haptic_contact_signal,
                hold_active=self.gripper_haptic_hold_active,
                hold_target_raw=self.gripper_haptic_hold_raw,
                servo_id=self.reader.gripper_servo_id,
                output_limit_raw=getattr(self.reader, 'last_gripper_hold_output_limit', None),
                fault=getattr(self, 'gripper_haptic_fault', ''),
                physical_resistance_verified=False), allow_nan=False)
            self.timing_publisher.publish(message)
        except Exception as exc:
            self.get_logger().warn(f'Lever feedback telemetry unavailable: {exc}', throttle_duration_sec=5.0)

    def update_gripper_haptic_hold(self) -> None:
        if not self.gripper_haptic_hold_active:
            return

        feedback_age = (
            self.get_clock().now() - self.gripper_haptic_last_feedback_time
        ).nanoseconds * 1e-9
        if feedback_age > self.gripper_haptic_feedback_timeout_sec:
            self.release_gripper_haptic_hold(
                "AnySkin feedback heartbeat timed out",
                block_rearm=True,
            )
            return

        hold_started_time = self.gripper_haptic_hold_started_time
        if hold_started_time is not None:
            hold_age = (
                self.get_clock().now() - hold_started_time
            ).nanoseconds * 1e-9
            if 0 < self.gripper_haptic_hold_duration_sec <= hold_age:
                self.release_gripper_haptic_hold(
                    f"{self.gripper_haptic_hold_duration_sec:.1f}s feedback pulse completed",
                    block_rearm=True,
                )
                return

        raw_position = self.reader.latest_gripper_raw
        held_normalized = self.gripper_haptic_hold_normalized
        if raw_position is None or held_normalized is None:
            return
        current_normalized = self.gripper_raw_to_normalized(raw_position)
        if (current_normalized >= 0.997 or
                current_normalized >= held_normalized + self.gripper_haptic_release_delta):
            self.release_gripper_haptic_hold(
                "operator moved the lever toward open",
                block_rearm=True,
            )

    def publish_state(self) -> None:
        try:
            read_start_ns = time.monotonic_ns()
            joints, gripper = self.reader.read_joints_and_gripper()
            read_end_ns = time.monotonic_ns()
        except (serial.SerialException, OSError) as exc:
            resistance = getattr(self, "body_resistance", None)
            if resistance is not None:
                resistance.fail(f"Controller serial read failed: {exc}")
            if getattr(self, 'lift_weight', None) is not None:
                self.lift_weight.invalidate('controller serial read failed')
                self.publish_lift_weight()
            self.get_logger().warn(
                "Leader serial read failed; keeping the robot at the last command. "
                f"Check that {self.reader.port} is still attached and not opened by "
                f"another process. Error: {exc}",
                throttle_duration_sec=2.0,
            )
            return

        resistance = getattr(self, "body_resistance", None)
        effort = self.update_lift_weight() if getattr(self, "lift_weight", None) is not None else 1.0
        if resistance is not None:
            raw = self.reader.fresh_raw_positions.get(resistance.servo_id)
            if isinstance(resistance, BodyResistancePair):
                raw = self.reader.fresh_raw_positions
                effort = self.lift_weight.efforts()
            resistance.update(
                raw,
                time.monotonic(), bool(self.get_parameter("body_resistance").value), effort=effort)
            if resistance.fault:
                self.get_logger().error(
                    f"Body resistance disabled: {resistance.fault}", throttle_duration_sec=2.0)
        if getattr(self, 'lift_weight', None) is not None:
            self.publish_lift_weight()

        if getattr(self, "body_resistance_bench", False):
            return  # No UR3/ROS motion or gripper commands from this publisher.

        # A robot round-trip must not delay delivery of the fresh lever sample.
        gripper_width = self.publish_gripper_input(gripper)
        if self.control_mode == "rtde_servoj":
            self.update_robot_joints_from_rtde()

        base_joints = [
            joint + offset
            for joint, offset in zip(joints, self.joint_position_offsets)
        ]

        if not self.alignment_ready:
            if self.latest_robot_joints is None:
                self.get_logger().warn(
                    "Waiting for robot /joint_states before sending leader commands.",
                    throttle_duration_sec=2.0,
                )
                return

            self.alignment_offsets = [
                robot_joint - leader_joint
                for robot_joint, leader_joint in zip(self.latest_robot_joints, base_joints)
            ]
            self.alignment_ready = True
            self.smoothed_target_joints = list(self.latest_robot_joints)
            self.filtered_commanded_joints = list(self.latest_robot_joints)
            self.last_command_time = self.get_clock().now()
            self.get_logger().info(
                "Aligned leader startup command to current robot joint state. "
                f"Alignment offsets: {[round(value, 4) for value in self.alignment_offsets]}"
            )

        target_joints = [
            joint + offset
            for joint, offset in zip(base_joints, self.alignment_offsets)
        ]
        commanded_joints = self.smooth_and_limit_command(target_joints)

        now = self.get_clock().now().to_msg()

        if self.latest_robot_joints is not None:
            robot_joint_state = JointState()
            robot_joint_state.header.stamp = now
            robot_joint_state.header.frame_id = self.frame_id
            robot_joint_state.name = self.joint_names
            robot_joint_state.position = self.latest_robot_joints
            self.robot_joint_state_publisher.publish(robot_joint_state)

        joint_state = JointState()
        joint_state.header.stamp = now
        joint_state.header.frame_id = self.frame_id
        joint_state.name = self.joint_names
        joint_state.position = commanded_joints
        self.joint_state_publisher.publish(joint_state)

        if self.control_mode == "rtde_servoj":
            self.send_rtde_servoj(commanded_joints)
            try:
                timing = dict(self.last_servoj_timing)
                timing.update(self.last_robot_observation)
                timing.update(read_start_ns=read_start_ns, read_end_ns=read_end_ns,
                              controller_q=list(joints), controller_target_q=list(target_joints),
                              command_q=list(commanded_joints), seq=getattr(self, "timing_seq", 0))
                self.timing_seq = timing["seq"] + 1
                if not hasattr(self, "timing_publisher"):
                    self.timing_publisher = self.create_publisher(String, "/tacgello/control_timing", 100)
                message = String()
                message.data = json.dumps(timing, allow_nan=False)
                self.timing_publisher.publish(message)
            except Exception as exc:
                self.get_logger().warn(f"Timing telemetry unavailable: {exc}", throttle_duration_sec=5.0)

        if self.publish_position_command:
            position_command = Float64MultiArray()
            position_command.data = commanded_joints
            self.position_command_publisher.publish(position_command)

        if self.publish_trajectory:
            trajectory = JointTrajectory()
            trajectory.joint_names = self.joint_names

            point = JointTrajectoryPoint()
            point.positions = commanded_joints
            seconds = int(self.trajectory_time_from_start_sec)
            nanoseconds = int((self.trajectory_time_from_start_sec - seconds) * 1e9)
            point.time_from_start.sec = seconds
            point.time_from_start.nanosec = nanoseconds
            trajectory.points = [point]

            self.trajectory_publisher.publish(trajectory)

        if self.publish_gripper_trajectory:
            gripper_trajectory = JointTrajectory()
            gripper_trajectory.joint_names = [self.gripper_joint_name]

            point = JointTrajectoryPoint()
            point.positions = [gripper_width]
            seconds = int(self.trajectory_time_from_start_sec)
            nanoseconds = int((self.trajectory_time_from_start_sec - seconds) * 1e9)
            point.time_from_start.sec = seconds
            point.time_from_start.nanosec = nanoseconds
            gripper_trajectory.points = [point]

            self.gripper_trajectory_publisher.publish(gripper_trajectory)

        if self.enable_gripper_haptic_feedback:
            self.process_gripper_haptic_feedback()
            self.update_gripper_haptic_hold()
        self.publish_gripper_haptic_status()

    def smooth_and_limit_command(self, target_joints: List[float]) -> List[float]:
        now = self.get_clock().now()
        dt = max((now - self.last_command_time).nanoseconds * 1e-9, 1e-3)
        self.last_command_time = now

        if self.filtered_commanded_joints is None or self.smoothed_target_joints is None:
            self.smoothed_target_joints = list(target_joints)
            self.filtered_commanded_joints = list(target_joints)
            return list(target_joints)

        if self.max_joint_delta_rad_per_cycle > 0.0:
            max_step = self.max_joint_delta_rad_per_cycle
        else:
            max_step = self.max_joint_speed_rad_per_sec * dt

        reference_joints = (
            self.latest_robot_joints
            if self.latest_robot_joints is not None
            else self.filtered_commanded_joints
        )
        next_smoothed_targets: List[float] = []
        deltas: List[float] = []

        for reference, smoothed_target, target in zip(
            reference_joints,
            self.smoothed_target_joints,
            target_joints,
        ):
            target = self.nearest_equivalent_angle(target, reference)
            if abs(target - reference) < self.command_deadband_rad:
                target = reference

            smoothed_target = (
                (1.0 - self.command_smoothing_alpha) * smoothed_target
                + self.command_smoothing_alpha * target
            )
            delta = smoothed_target - reference
            if abs(delta) < self.command_deadband_rad:
                delta = 0.0

            next_smoothed_targets.append(smoothed_target)
            deltas.append(delta)

        max_delta = max((abs(delta) for delta in deltas), default=0.0)
        if max_step > 0.0 and max_delta > max_step:
            scale = max_step / max_delta
            deltas = [delta * scale for delta in deltas]

        next_joints = [
            reference + delta
            for reference, delta in zip(reference_joints, deltas)
        ]

        self.smoothed_target_joints = next_smoothed_targets
        self.filtered_commanded_joints = next_joints
        return next_joints

    def nearest_equivalent_angle(self, target: float, reference: float) -> float:
        """Return the 2*pi-equivalent target closest to the current joint angle."""
        delta = math.atan2(math.sin(target - reference), math.cos(target - reference))
        return reference + delta

    def publish_gripper_input(self, gripper):
        gripper_msg = Float32()
        gripper_msg.data = float(gripper)
        self.gripper_publisher.publish(gripper_msg)
        raw = self.reader.fresh_raw_positions.get(self.reader.gripper_servo_id)
        gripper_raw_msg = Float32()
        gripper_raw_msg.data = float(raw) if raw is not None else -1.0
        self.gripper_raw_publisher.publish(gripper_raw_msg)
        width = self.gripper_to_width(gripper)
        if raw is None:
            return width  # Cached positions must not renew the command heartbeat.
        if self.gripper_control_mode not in ("discrete", "binary", "open_close"):
            width = self.filter_gripper_width(width)
        if self.publish_gripper_command and self.should_publish_gripper_command(width):
            command = Float64MultiArray()
            command.data = [width]
            self.gripper_command_publisher.publish(command)
            self.gripper_command_alt_publisher.publish(command)
            self.last_published_gripper_command_width = width
            self.last_gripper_command_publish_time = self.get_clock().now()
        return width

    def body_contact_feedback(self, msg):
        now = time.monotonic()
        self.body_contact, self.body_contact_time = bool(msg.data), now
        if self.body_resistance is not None:
            self.body_resistance.feedback(msg.data, now)

    def tare_lift_weight(self, request, response):
        now = time.monotonic()
        fresh = self.body_contact_time is not None and 0 <= now - self.body_contact_time <= 0.35
        response.success = fresh and not self.body_contact
        if response.success:
            self.lift_weight.request_tare(now)
            if self.body_resistance is not None:
                self.body_resistance.update(None, now, enabled=False)
            response.message = ('Current baseline queued' if isinstance(self.lift_weight, JointCurrentCue)
                                else 'Tare queued') + ': keep EMPTY tool stationary; watch /tacgello/lift_weight'
        else:
            response.message = 'Tare refused: need fresh no-contact heartbeat and empty tool'
        return response

    def update_lift_weight(self):
        now = time.monotonic()
        fresh = self.body_contact_time is not None and 0 <= now - self.body_contact_time <= 0.35
        try:
            receive = self.rtde_receive_interface
            if receive is None or not receive.isConnected():
                raise RuntimeError('RTDE disconnected')
            # These getters read a receive-thread cache. Retry a boundary crossing
            # once so a single RTDE tick does not discard the settling window.
            for _ in range(2):
                before = receive.getTimestamp()
                pose = None
                if isinstance(self.lift_weight, JointCurrentCue):
                    force = receive.getActualCurrent()
                    speed = receive.getActualQd()
                    if self.lift_weight.contrast:
                        pose = receive.getActualQ()
                else:
                    force = receive.getActualTCPForce()
                    speed = receive.getActualTCPSpeed()
                after = receive.getTimestamp()
                if before == after:
                    break
            else:
                raise RuntimeError('inconsistent RTDE snapshot')
            args = (force, speed, after, now, self.body_contact, fresh)
            effort = self.lift_weight.update(*args, pose) if pose is not None else self.lift_weight.update(*args)
        except Exception as exc:
            self.lift_weight.invalidate(f'weight unavailable: {exc}')
            effort = 0.0
        self.lift_weight.check_tare_timeout(now)
        return effort

    def publish_lift_weight(self):
        # Publish AFTER the servo update, including edges between 5Hz samples.
        now = time.monotonic()
        resistance = self.body_resistance
        enabled = resistance is not None and bool(self.get_parameter('body_resistance').value)
        active = bool(resistance and resistance.active and not resistance.fault)
        fault = resistance.fault if resistance else ''
        key = (self.body_contact, enabled, active, fault)
        if isinstance(resistance, BodyResistancePair):
            key += tuple((m['servo_id'], m['resistance_active'], m['output_limit_raw'],
                          m['resistance_fault']) for m in resistance.status())
        elif resistance is not None:
            key += (getattr(resistance, 'applied_limit', 0) if active else 0,)
        if key != getattr(self, 'last_weight_key', None) or now - self.last_weight_status >= 0.2:
            self.last_weight_key = key
            self.last_weight_status = now
            status = self.lift_weight.status()
            status.update(observed_pc_ns=time.monotonic_ns(), contact=self.body_contact,
                resistance_enabled=enabled, resistance_active=active, resistance_fault=fault,
                servo_id=resistance.servo_id if resistance else 2,
                output_limit_raw=getattr(resistance, 'applied_limit', 0) if active else 0,
                physical_resistance_verified=False)
            status['resistance_mode'] = ('continuous_position_lag' if
                (any(m.continuous for m in resistance.motors.values()) if isinstance(resistance, BodyResistancePair)
                 else getattr(resistance, 'continuous', False)) else 'motion_gated')
            if isinstance(resistance, BodyResistancePair):
                status['body_servos'] = resistance.status()
                status['servo_ids'] = list(resistance.motors)
            self.weight_status_publisher.publish(String(data=json.dumps(status, allow_nan=False)))

    def update_robot_joints_from_rtde(self) -> None:
        self.last_robot_observation = {"robot_sample_consistent": False}
        if self.rtde_receive_interface is None:
            return

        try:
            stamp_before = self.rtde_receive_interface.getTimestamp()
            joints = self.rtde_receive_interface.getActualQ()
            stamp_after = self.rtde_receive_interface.getTimestamp()
            observed_ns = time.monotonic_ns()
        except Exception as exc:
            self.get_logger().warn(
                f"Failed to read RTDE joint state: {exc}",
                throttle_duration_sec=2.0,
            )
            return

        if joints is not None and len(joints) >= 6:
            self.latest_robot_joints = [float(value) for value in joints[:6]]
            self.last_robot_observation = {
                "robot_sample_consistent": stamp_before == stamp_after,
                "robot_stamp": stamp_after, "observed_ns": observed_ns,
                "robot_q": list(self.latest_robot_joints),
            }

    def send_rtde_servoj(self, commanded_joints: List[float]) -> None:
        self.last_servoj_timing = {"send_ok": False}
        if self.rtde_control_interface is None:
            return

        try:
            period_start = self.rtde_control_interface.initPeriod()
            send_start_ns = time.monotonic_ns()
            result = self.rtde_control_interface.servoJ(
                commanded_joints[:6],
                self.rtde_velocity,
                self.rtde_acceleration,
                self.rtde_dt,
                self.rtde_lookahead_time,
                self.rtde_gain,
            )
            send_end_ns = time.monotonic_ns()
            self.last_servoj_timing = {"send_start_ns": send_start_ns,
                                       "send_end_ns": send_end_ns, "send_ok": bool(result)}
            self.rtde_control_interface.waitPeriod(period_start)
        except Exception as exc:
            self.get_logger().error(
                f"Failed to send RTDE servoJ command: {exc}",
                throttle_duration_sec=1.0,
            )

    def gripper_to_width(self, gripper_rad: float) -> float:
        gripper_raw = self.reader.latest_gripper_raw
        if gripper_raw is not None and self.gripper_max_raw != self.gripper_min_raw:
            normalized = self.gripper_raw_to_normalized(gripper_raw)
        elif self.gripper_max_rad == self.gripper_min_rad:
            normalized = 0.0
        else:
            normalized = (gripper_rad - self.gripper_min_rad) / (
                self.gripper_max_rad - self.gripper_min_rad
            )

        normalized = max(0.0, min(normalized, 1.0))
        if self.invert_gripper:
            normalized = 1.0 - normalized

        if self.gripper_control_mode in ("discrete", "binary", "open_close"):
            return self.discrete_gripper_to_width(normalized)

        return self.gripper_min_width + normalized * (
            self.gripper_max_width - self.gripper_min_width
        )

    def discrete_gripper_to_width(self, normalized: float) -> float:
        previous_state = self.discrete_gripper_state
        if self.discrete_gripper_start_open_pending:
            self.discrete_gripper_start_open_pending = False
            self.discrete_gripper_state = 1
            self.discrete_gripper_width = self.gripper_max_width
            self.get_logger().info("Discrete gripper starts with an open command.")
            return self.gripper_max_width

        if self.discrete_gripper_state is None:
            self.discrete_gripper_state = 1 if normalized >= 0.5 else -1

        if normalized <= self.gripper_discrete_close_threshold:
            self.discrete_gripper_state = -1
        elif normalized >= self.gripper_discrete_open_threshold:
            self.discrete_gripper_state = 1

        if previous_state != self.discrete_gripper_state:
            state_name = "open" if self.discrete_gripper_state > 0 else "close"
            self.get_logger().info(
                f"Discrete gripper command state changed to {state_name} "
                f"(normalized={normalized:.3f})."
            )

        target_width = (
            self.gripper_min_width
            if self.discrete_gripper_state < 0
            else self.gripper_max_width
        )
        if self.gripper_discrete_step_m <= 0.0:
            self.discrete_gripper_width = target_width
            return target_width

        if self.discrete_gripper_width is None:
            self.discrete_gripper_width = target_width
            return self.discrete_gripper_width

        delta = target_width - self.discrete_gripper_width
        if abs(delta) <= self.gripper_discrete_step_m:
            self.discrete_gripper_width = target_width
        else:
            step = self.gripper_discrete_step_m if delta > 0.0 else -self.gripper_discrete_step_m
            self.discrete_gripper_width += step
        return self.discrete_gripper_width

    def gripper_raw_to_normalized(self, raw: int) -> float:
        raw = int(raw)
        min_raw = self.gripper_min_raw
        max_raw = self.gripper_max_raw

        if self.gripper_wrap_open_raw >= 0:
            if raw <= self.gripper_wrap_open_raw:
                raw += self.gripper_encoder_ticks
            if max_raw <= min_raw:
                max_raw += self.gripper_encoder_ticks

        lower = min(min_raw, max_raw)
        upper = max(min_raw, max_raw)
        raw = max(lower, min(upper, raw))
        return (raw - min_raw) / (max_raw - min_raw)

    def should_publish_gripper_command(self, width: float) -> bool:
        if self.last_published_gripper_command_width is None:
            return True

        if (
            abs(width - self.last_published_gripper_command_width)
            < self.gripper_command_publish_deadband_m
        ):
            return False

        if self.gripper_command_publish_rate_hz <= 0.0:
            return True

        elapsed = (
            self.get_clock().now() - self.last_gripper_command_publish_time
        ).nanoseconds / 1e9
        return elapsed >= 1.0 / self.gripper_command_publish_rate_hz

    def clamp_gripper_raw(self, raw: int) -> int:
        lower = min(self.gripper_min_raw, self.gripper_max_raw)
        upper = max(self.gripper_min_raw, self.gripper_max_raw)
        return int(max(lower, min(upper, raw)))

    def filter_gripper_width(self, width: float) -> float:
        width = self.clamp(width, self.gripper_min_width, self.gripper_max_width)
        if self.filtered_gripper_width is None:
            self.filtered_gripper_width = width
            self.last_gripper_command_width = width
            return width

        reference_width = (
            self.last_gripper_command_width
            if self.last_gripper_command_width is not None
            else self.filtered_gripper_width
        )
        if width < reference_width - self.gripper_command_deadband_m:
            self.gripper_open_confirm_count = 0
            if self.gripper_close_latch_width is None:
                self.gripper_close_latch_width = width
            else:
                self.gripper_close_latch_width = min(
                    self.gripper_close_latch_width,
                    width,
                )
        elif (
            self.gripper_close_latch_width is not None
            and width < self.gripper_close_latch_width + self.gripper_close_latch_release_m
        ):
            self.gripper_open_confirm_count = 0
            width = self.gripper_close_latch_width
        elif (
            self.gripper_close_latch_width is not None
            and width >= self.gripper_close_latch_width + self.gripper_close_latch_release_m
        ):
            self.gripper_open_confirm_count += 1
            if self.gripper_open_confirm_count >= self.gripper_open_confirm_cycles:
                self.gripper_close_latch_width = None
                self.gripper_open_confirm_count = 0
            else:
                width = self.gripper_close_latch_width

        self.filtered_gripper_width = (
            (1.0 - self.gripper_smoothing_alpha) * self.filtered_gripper_width
            + self.gripper_smoothing_alpha * width
        )

        if self.last_gripper_command_width is None:
            self.last_gripper_command_width = self.filtered_gripper_width
            return self.filtered_gripper_width

        delta = self.filtered_gripper_width - self.last_gripper_command_width
        if abs(delta) < self.gripper_command_deadband_m:
            return self.last_gripper_command_width

        direction = 1 if delta > 0.0 else -1
        if (
            self.last_gripper_direction != 0
            and direction != self.last_gripper_direction
            and abs(delta) < self.gripper_reversal_deadband_m
        ):
            return self.last_gripper_command_width

        self.last_gripper_direction = direction
        self.last_gripper_command_width = self.filtered_gripper_width
        return self.filtered_gripper_width

    def destroy_node(self) -> None:
        resistance = getattr(self, "body_resistance", None)
        if resistance is not None:
            resistance.close()
            if resistance.fault:
                self.get_logger().error(resistance.fault)
        if self.gripper_haptic_hold_active:
            self.release_gripper_haptic_hold("node shutdown", block_rearm=True)
        if self.rtde_control_interface is not None:
            try:
                self.rtde_control_interface.servoStop()
            except Exception:
                pass
        self.reader.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None

    try:
        node = TacGelloLeaderPublisher()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
