from types import SimpleNamespace
import threading

from tacgello_ros.leader_publisher import (
    STS3215LeaderReader,
    TacGelloLeaderPublisher,
)


class FakePacketHandler:
    def __init__(self):
        self.calls = []
        self.registers = [0] * 10
        self.registers[8:10] = [232, 3]
        self.read_length = 1

    def syncWriteTxOnly(self, port, address, length, values, param_length):
        assert param_length == length + 1
        assert values[0] == 7
        for offset, value in enumerate(values[1:]):
            self.registers[address - 40 + offset] = value
        if length == 1:
            self.calls.append(("write1", port, values[0], address, values[1]))
        else:
            self.calls.append(("write", port, values[0], address, length, list(values[1:])))
        return 0

    def write1ByteTxRx(self, port, servo_id, address, value):
        self.calls.append(("write1", port, servo_id, address, value))
        return 0, 0

    def writeTxRx(self, port, servo_id, address, length, values):
        self.calls.append(
            ("write", port, servo_id, address, length, list(values))
        )
        return 0, 0

    def getTxRxResult(self, result):
        return f"result={result}"

    def read1ByteTxRx(self, port, servo_id, address):
        self.calls.append(("read1", port, servo_id, address))
        return getattr(self, "readback", (1, 0, 0))

    def readTx(self, port, servo_id, address, length):
        self.read_length = length
        self.calls.append(('read1' if length == 1 else 'read10', port, servo_id, address))
        return 0

    def rxPacket(self, port):
        if getattr(self, 'packets', None):
            return self.packets.pop(0), 0
        if self.read_length == 10:
            registers = list(self.registers)
            if getattr(self, 'bad_limit', False):
                registers[8:10] = [232, 3]
            return [255, 255, 7, 12, 0] + registers + [0], 0
        value, result, error = getattr(self, 'readback', (1, 0, 0))
        return [255, 255, 7, 3, error, value, 0], result


def make_reader():
    reader = object.__new__(STS3215LeaderReader)
    reader.port_handler = SimpleNamespace(
        ser=SimpleNamespace(reset_input_buffer=lambda: None),
        setPacketTimeoutMillis=lambda ms: None, is_using=False)
    reader.packet_handler = FakePacketHandler()
    reader.gripper_servo_id = 7
    reader.torque_enable_addr = 40
    reader.acceleration_addr = 41
    return reader


def test_hold_targets_only_gripper_servo_at_current_position():
    reader = make_reader()

    ok, detail = reader.hold_gripper_at_raw(0x0ABC, 0x0123, 20)

    assert ok is True
    assert detail == "ok"
    assert reader.packet_handler.calls == [
        ("write1", reader.port_handler, 7, 40, 0),
        ("read10", reader.port_handler, 7, 40),
        (
            "write",
            reader.port_handler,
            7,
            41,
            9,
            [20, 0xBC, 0x0A, 0, 0, 0x23, 0x01, 44, 1],
        ),
        ("read10", reader.port_handler, 7, 40),
        ("write1", reader.port_handler, 7, 40, 1),
        ("read1", reader.port_handler, 7, 40),
    ]


def test_hold_readback_failure_releases_torque_and_reports_failure():
    for readback in ((0, 0, 0), (1, -1, 0), (1, 0, 4)):
        reader = make_reader()
        reader.packet_handler.readback = readback
        ok, detail = reader.hold_gripper_at_raw(3700, 120, 20)
        assert not ok
        assert "readback failed" in detail
        assert reader.packet_handler.calls[-1] == (
            "write1", reader.port_handler, 7, 40, 0)


def test_hold_never_increases_existing_lower_output_limit():
    reader = make_reader()
    reader.packet_handler.registers[8:10] = [100, 0]
    assert reader.hold_gripper_at_raw(3700, 120, 20, 300)[0]
    assert reader.last_gripper_hold_output_limit == 100
    assert reader.packet_handler.registers[8:10] == [100, 0]


def test_limit_readback_mismatch_never_enables_hold():
    reader = make_reader()
    reader.packet_handler.bad_limit = True
    ok, detail = reader.hold_gripper_at_raw(3700, 120, 20)
    assert not ok
    assert 'output-limit readback failed' in detail
    assert ('write1', reader.port_handler, 7, 40, 1) not in reader.packet_handler.calls


def test_delayed_hold_registers_get_read_only_recheck_without_target_replay():
    reader = make_reader()
    original = reader.packet_handler.rxPacket
    reads = []
    def delayed(port):
        packet, result = original(port)
        if reader.packet_handler.read_length == 10:
            reads.append(1)
            if len(reads) == 2:
                packet[7] = 0
        return packet, result
    reader.packet_handler.rxPacket = delayed
    assert reader.hold_gripper_at_raw(3700, 120, 20)[0]
    assert len(reads) == 3
    assert sum(c[0] == 'write' for c in reader.packet_handler.calls) == 1


def test_readback_failure_detail_includes_expected_and_received_registers():
    reader = make_reader()
    reader.packet_handler.bad_limit = True
    ok, detail = reader.hold_gripper_at_raw(3700, 120, 20)
    assert not ok
    assert 'expected=' in detail and 'received=' in detail


def test_unexpected_torque_on_requires_verified_off_without_goal_replay():
    reader = make_reader()
    write = reader.packet_handler.syncWriteTxOnly
    def auto_enable(port, address, length, values, param_length):
        result = write(port, address, length, values, param_length)
        if address == 41:
            reader.packet_handler.registers[0] = 1
        return result
    reader.packet_handler.syncWriteTxOnly = auto_enable
    assert reader.hold_gripper_at_raw(3700, 120, 20)[0]
    calls = reader.packet_handler.calls
    assert sum(c[0] == 'write' for c in calls) == 1
    assert sum(c[0] == 'write1' and c[-1] == 0 for c in calls) == 2


def test_persistent_unexpected_torque_on_fails_closed():
    reader = make_reader()
    write = reader.packet_handler.syncWriteTxOnly
    def stuck_on(port, address, length, values, param_length):
        result = write(port, address, length, values, param_length)
        if any(c[0] == 'write' for c in reader.packet_handler.calls):
            reader.packet_handler.registers[0] = 1
        return result
    reader.packet_handler.syncWriteTxOnly = stuck_on
    ok, detail = reader.hold_gripper_at_raw(3700, 120, 20)
    assert not ok
    assert 'readback failed' in detail
    assert ('write1', reader.port_handler, 7, 40, 1) not in reader.packet_handler.calls


def test_haptic_telemetry_reports_state_not_physical_force():
    import json
    messages = []
    state = SimpleNamespace(timing_publisher=SimpleNamespace(publish=messages.append),
        enable_gripper_haptic_feedback=True, gripper_haptic_contact_signal=True,
        gripper_haptic_hold_active=True, gripper_haptic_hold_raw=3700,
        reader=SimpleNamespace(gripper_servo_id=7, last_gripper_hold_output_limit=300))
    TacGelloLeaderPublisher.publish_gripper_haptic_status(state, 'hold_verified')
    value = json.loads(messages[-1].data)
    assert value['hold_active']
    assert value['output_limit_raw'] == 300
    assert value['physical_resistance_verified'] is False
    state.gripper_haptic_fault = 'release readback failed'
    TacGelloLeaderPublisher.publish_gripper_haptic_status(state, 'release_failed')
    assert json.loads(messages[-1].data)['fault']


def test_failed_hold_does_not_claim_feedback_or_retry_same_contact():
    events, calls = [], []
    def hold(*args):
        calls.append(args)
        return False, 'readback mismatch'
    state = SimpleNamespace(gripper_haptic_contact_signal=False, gripper_haptic_hold_active=False,
        gripper_haptic_rearm_blocked=False, gripper_haptic_hold_speed=120,
        gripper_haptic_hold_acceleration=20, gripper_haptic_hold_output_limit=300,
        reader=SimpleNamespace(latest_gripper_raw=3700, hold_gripper_at_raw=hold),
        publish_gripper_haptic_status=lambda event: events.append(event),
        get_logger=lambda: SimpleNamespace(error=lambda message: None))
    TacGelloLeaderPublisher._apply_gripper_haptic_contact(state, True)
    assert events == ['hold_failed']
    assert not state.gripper_haptic_hold_active
    assert state.gripper_haptic_rearm_blocked
    TacGelloLeaderPublisher._apply_gripper_haptic_contact(state, True)
    assert len(calls) == 1


def test_invalid_limit_does_not_write_anything():
    for value in (0, -1, 1001):
        reader = make_reader()
        assert not reader.hold_gripper_at_raw(3700, 120, 20, value)[0]
        assert reader.packet_handler.calls == []


def test_block_read_rejects_delayed_position_reply():
    reader = make_reader()
    reader.packet_handler.packets = [[255, 255, 7, 4, 0, 73, 14, 0]]
    values, result, error = reader._read_gripper_registers(40, 10)
    assert values == reader.packet_handler.registers
    assert result == error == 0


def test_release_disables_only_gripper_servo_torque():
    reader = make_reader()

    ok, detail = reader.release_gripper_hold()

    assert ok is True
    assert detail == "ok"
    assert reader.packet_handler.calls == [
        ("write1", reader.port_handler, 7, 40, 0)
    ]


def test_torque_read_rejects_delayed_position_packet():
    reader = make_reader()
    reader.packet_handler.packets = [[255, 255, 7, 4, 0, 73, 14, 0],
                                     [255, 255, 6, 3, 0, 1, 0]]
    assert reader._read_gripper_torque_enabled() == (1, 0, 0)
    assert not reader.port_handler.is_using


def test_feedback_callback_never_accesses_servo_bus():
    state = SimpleNamespace(haptic_mailbox_lock=threading.Lock(),
                            haptic_pending_clear=False,
                            get_clock=lambda: SimpleNamespace(now=lambda: FakeTime(10)))
    TacGelloLeaderPublisher.gripper_haptic_feedback_callback(state, SimpleNamespace(data=True))
    assert state.haptic_mailbox[0] is True
    TacGelloLeaderPublisher.gripper_haptic_feedback_callback(state, SimpleNamespace(data=False))
    assert state.haptic_pending_clear


def test_mailbox_does_not_replay_stale_contact():
    applied = []
    state = SimpleNamespace(haptic_mailbox_lock=threading.Lock(),
                            haptic_mailbox=(True, FakeTime(8)), haptic_pending_clear=False,
                            gripper_haptic_feedback_timeout_sec=1,
                            get_clock=lambda: SimpleNamespace(now=lambda: FakeTime(10)),
                            _apply_gripper_haptic_contact=applied.append)
    TacGelloLeaderPublisher.process_gripper_haptic_feedback(state)
    assert applied == [False]
    assert state.gripper_haptic_last_feedback_time.nanoseconds == 8_000_000_000


def test_mailbox_preserves_clear_before_new_contact():
    applied = []
    state = SimpleNamespace(haptic_mailbox_lock=threading.Lock(),
                            haptic_mailbox=(True, FakeTime(9.9)), haptic_pending_clear=True,
                            gripper_haptic_feedback_timeout_sec=1,
                            get_clock=lambda: SimpleNamespace(now=lambda: FakeTime(10)),
                            _apply_gripper_haptic_contact=applied.append)
    TacGelloLeaderPublisher.process_gripper_haptic_feedback(state)
    assert applied == [False, True]
    assert not state.haptic_pending_clear


class FakeTime:
    def __init__(self, seconds):
        self.nanoseconds = int(seconds * 1e9)

    def __sub__(self, other):
        return FakeTime((self.nanoseconds - other.nanoseconds) * 1e-9)


def test_contact_hold_persists_but_opening_and_stale_feedback_release():
    releases = []
    state = SimpleNamespace(
        gripper_haptic_hold_active=True,
        gripper_haptic_last_feedback_time=FakeTime(9.9),
        gripper_haptic_feedback_timeout_sec=1.0,
        gripper_haptic_hold_started_time=FakeTime(0),
        gripper_haptic_hold_duration_sec=0,
        gripper_haptic_hold_normalized=0.5,
        gripper_haptic_release_delta=0.015,
        reader=SimpleNamespace(latest_gripper_raw=0.5),
        gripper_raw_to_normalized=lambda raw: raw,
        get_clock=lambda: SimpleNamespace(now=lambda: FakeTime(10)),
        release_gripper_haptic_hold=lambda reason, block_rearm=False: releases.append(reason))
    TacGelloLeaderPublisher.update_gripper_haptic_hold(state)
    assert releases == []
    state.reader.latest_gripper_raw = 0.52
    TacGelloLeaderPublisher.update_gripper_haptic_hold(state)
    assert 'toward open' in releases.pop()
    state.reader.latest_gripper_raw = 0.5
    state.gripper_haptic_last_feedback_time = FakeTime(8)
    TacGelloLeaderPublisher.update_gripper_haptic_hold(state)
    assert 'heartbeat timed out' in releases.pop()


def test_measured_controller_endpoints_cover_full_range():
    state = SimpleNamespace(gripper_min_raw=3425, gripper_max_raw=3924,
                            gripper_wrap_open_raw=700, gripper_encoder_ticks=4096)
    normalize = lambda raw: TacGelloLeaderPublisher.gripper_raw_to_normalized(state, raw)
    assert normalize(3425) == 0.0
    assert normalize(3924) == 1.0
    assert normalize(3400) == 0.0
    assert normalize(3950) == 1.0
    assert 0.49 < normalize(3674) < 0.51


def test_haptic_hold_releases_after_feedback_pulse_duration():
    releases = []
    now = FakeTime(3.0)
    state = SimpleNamespace(
        gripper_haptic_hold_active=True,
        gripper_haptic_last_feedback_time=FakeTime(2.8),
        gripper_haptic_feedback_timeout_sec=1.0,
        gripper_haptic_hold_started_time=FakeTime(0.0),
        gripper_haptic_hold_duration_sec=3.0,
        get_clock=lambda: SimpleNamespace(now=lambda: now),
        release_gripper_haptic_hold=lambda reason, block_rearm=False: releases.append(
            (reason, block_rearm)
        ),
    )

    TacGelloLeaderPublisher.update_gripper_haptic_hold(state)

    assert releases == [("3.0s feedback pulse completed", True)]
