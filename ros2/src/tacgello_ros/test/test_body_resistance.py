from tacgello_ros.body_resistance import BodyResistance
from tacgello_ros.body_resistance import BodyResistancePair
import pytest


def continuous_cue():
    packet = Packet()
    motor = BodyResistance(packet, None, output_limit=40, continuous=True)
    motor.feedback(True, 1)
    motor.update(2000, 1)
    return packet, motor


def test_continuous_small_motion_and_rest_keep_torque_on():
    packet, motor = continuous_cue()
    for i, raw in enumerate([2001, 2002, 2002, 2002, 2003], 1):
        motor.feedback(True, 1+i*.02)
        motor.update(raw, 1+i*.02)
        assert motor.active
        target = packet.registers[42] + 256*packet.registers[43]
        assert abs(target-raw) <= 2
        if i in (3, 4):
            assert target == raw
    assert packet.writes.count((40, [1])) == 1


def test_continuous_reversal_neutralizes_previous_lag():
    packet, motor = continuous_cue()
    motor.update(2010, 1.02)
    motor.update(2020, 1.04)
    motor.update(2019, 1.06)
    assert packet.registers[42] + 256*packet.registers[43] == 2019
    assert motor.active


def test_continuous_twelve_second_motion_keeps_enable_and_bounded_lag():
    packet, motor = continuous_cue()
    for i in range(1, 601):
        now = 1+i*.02
        raw = 2000 + (i % 100 if i % 200 < 100 else 100-i % 100)
        motor.feedback(True, now)
        motor.update(raw, now)
        assert motor.active and not motor.fault
        target = packet.registers[42] + 256*packet.registers[43]
        assert abs(raw-target) <= 2
    assert packet.writes.count((40, [1])) == 1
    assert motor.applied_limit == 40
    motor.feedback(False, 13.02)
    motor.update(2000, 13.02)
    assert not motor.active


@pytest.mark.parametrize('reason', ['contact', 'stale', 'disabled', 'zero', 'gap', 'missing'])
def test_continuous_release_conditions_remain_immediate(reason):
    packet, motor = continuous_cue()
    motor.update(2001, 1.02)
    if reason == 'contact':
        motor.feedback(False, 1.03)
    if reason == 'stale':
        motor.feedback(True, .1)
    motor.update(None if reason == 'missing' else 2002,
                 2 if reason == 'gap' else 1.04,
                 enabled=reason != 'disabled', effort=0 if reason == 'zero' else 1)
    assert not motor.active and packet.registers[40] == 0


def test_continuous_hardware_limit_ramps_even_with_full_requested_effort():
    packet, motor = continuous_cue()
    for i in range(1, 101):
        now = 1 + i*.02
        motor.feedback(True, now)
        motor.update(2000+i, now)
        assert motor.applied_limit <= 1 + 5*(now-1)
    assert motor.applied_limit <= 11


def test_off_clears_leftover_torque_even_when_not_internally_active():
    packet = Packet()
    packet.registers[40] = 1
    resistance = BodyResistance(packet, None)
    resistance.update(None, 1, effort=0)
    assert packet.registers[40] == 0
    packet.registers[40] = 1
    resistance.update(None, 1.6, effort=0)
    assert packet.registers[40] == 0


class PairPacket:
    def __init__(self):
        self.devices = {i: Packet() for i in (2, 3)}

    def syncWriteTxOnly(self, port, address, width, params, count):
        return self.devices[params[0]].syncWriteTxOnly(port, address, width, [2, *params[1:]], count)

    def read1ByteTxRx(self, port, servo_id, address):
        return self.devices[servo_id].read1ByteTxRx(port, servo_id, address)

    def read2ByteTxRx(self, port, servo_id, address):
        return self.devices[servo_id].read2ByteTxRx(port, servo_id, address)


def test_wrist_trial_cap_and_shared_fault_release():
    packet = PairPacket()
    packet.devices[4] = Packet()
    pair = BodyResistancePair(packet, None, output_limit=40, wrist=True)
    pair.feedback(True, 1)
    pair.update({2:2000, 3:2000, 4:2000}, 1, effort={2:1, 3:1, 4:1})
    pair.update({2:2005, 3:2005, 4:2005}, 1.02, effort={2:1, 3:1, 4:1})
    assert packet.devices[4].registers[48] == 10
    assert [s['servo_id'] for s in pair.status()] == [2, 3, 4]
    pair.motors[4].fail('wrist test fault')
    pair.update({2:2010, 3:2010, 4:2010}, 1.04, effort={2:1, 3:1, 4:1})
    assert all(p.registers[40] == 0 for p in packet.devices.values())


def test_pair_independent_limits_and_release_both():
    packet = PairPacket()
    pair = BodyResistancePair(packet, None)
    pair.feedback(True, 1)
    pair.update({2:2000, 3:2000}, 1, effort={2:.2, 3:.7})
    pair.update({2:2005, 3:2005}, 1.02, effort={2:.2, 3:.7})
    assert packet.devices[2].registers[48] == 2
    assert packet.devices[3].registers[48] == 7
    assert all(m.active for m in pair.motors.values())
    pair.feedback(False, 1.04)
    pair.update({2:2010, 3:2010}, 1.04, effort={2:.2, 3:.7})
    assert all(p.registers[40] == 0 for p in packet.devices.values())


def test_pair_fault_on_elbow_releases_shoulder():
    packet = PairPacket()
    pair = BodyResistancePair(packet, None)
    pair.feedback(True, 1)
    pair.update({2:2000, 3:2000}, 1, effort={2:1., 3:1.})
    packet.devices[3].bad_limit = True
    pair.update({2:2005, 3:2005}, 1.02, effort={2:1., 3:.5})
    assert pair.fault
    assert not pair.active
    assert all(p.registers[40] == 0 for p in packet.devices.values())


def test_extended_limit_verified_before_enable_and_released():
    packet = PairPacket()
    pair = BodyResistancePair(packet, None, output_limit=20)
    pair.feedback(True, 1)
    pair.update({2:2000, 3:2000}, 1, effort={2:1., 3:.75})
    pair.update({2:2005, 3:2005}, 1.02, effort={2:1., 3:.75})
    assert packet.devices[2].registers[48] == 20
    assert packet.devices[3].registers[48] == 15
    pair.update(None, 1.04, enabled=False)
    assert all(p.registers[40] == 0 for p in packet.devices.values())


def test_forty_trial_output_limit_and_off():
    packet = PairPacket()
    pair = BodyResistancePair(packet, None, output_limit=40)
    pair.feedback(True, 1)
    pair.update({2:2000, 3:2000}, 1, effort={2:1., 3:1.})
    pair.update({2:2005, 3:2005}, 1.02, effort={2:1., 3:1.})
    assert all(p.registers[48] == 40 for p in packet.devices.values())
    pair.update(None, 1.04, enabled=False)
    assert all(p.registers[40] == 0 for p in packet.devices.values())


class Packet:
    def __init__(self):
        self.registers = {33: 0, 40: 0, 48: 1000, 49: 3}
        self.writes = []
        self.bad_limit = False

    def syncWriteTxOnly(self, port, address, width, params, count):
        assert params[0] == 2
        self.writes.append((address, params[1:]))
        self.registers.update({address + i: v for i, v in enumerate(params[1:])})
        return 0

    def read1ByteTxRx(self, port, servo_id, address):
        return self.registers.get(address, 0), 0, 0

    def read2ByteTxRx(self, port, servo_id, address):
        if self.bad_limit:
            return 1000, 0, 0
        return self.registers[address] + 256 * self.registers[address + 1], 0, 0


def cue():
    packet = Packet()
    resistance = BodyResistance(packet, None)
    resistance.feedback(True, 1)
    resistance.update(2000, 1)
    return packet, resistance


def test_small_lag_and_cap_before_enable():
    packet, resistance = cue()
    resistance.update(2005, 1.02)
    assert resistance.active
    assert packet.registers[42] + 256 * packet.registers[43] == 2003
    assert packet.registers[48] == 10
    enable_index = packet.writes.index((40, [1]))
    assert any(address == 48 for address, _ in packet.writes[:enable_index])


def test_rest_releases_and_reversal_tracks_current():
    packet, resistance = cue()
    resistance.update(2005, 1.02)
    resistance.update(2005, 1.04)
    assert not resistance.active
    assert packet.registers[40] == 0
    resistance.update(2000, 1.06)
    assert packet.registers[42] + 256 * packet.registers[43] == 2002


def test_contact_release_and_stale():
    for contact_age in (0, 1):
        packet, resistance = cue()
        resistance.update(2005, 1.02)
        resistance.feedback(bool(contact_age), 1.02 - contact_age)
        resistance.update(2010, 1.04)
        assert packet.registers[40] == 0


def test_bad_limit_never_enables():
    packet = Packet()
    packet.bad_limit = True
    resistance = BodyResistance(packet, None)
    resistance.feedback(True, 1)
    resistance.update(2000, 1)
    resistance.update(2005, 1.02)
    assert resistance.fault
    assert (40, [1]) not in packet.writes


def test_missing_position_and_gap_and_wrap_latch_fault():
    for raw, now in ((None, 1.04), (2010, 2), (4095, 1.04)):
        packet, resistance = cue()
        resistance.update(2005, 1.02)
        resistance.update(raw, now)
        assert resistance.fault
        assert packet.registers[40] == 0


def test_mode_not_changed():
    packet = Packet()
    packet.registers[33] = 1
    resistance = BodyResistance(packet, None)
    resistance.update(2000, 1)
    assert resistance.fault
    assert all(address != 33 for address, _ in packet.writes)


def test_disable_and_close_keep_low_limit():
    packet, resistance = cue()
    resistance.update(2005, 1.02)
    resistance.update(2010, 1.04, enabled=False)
    resistance.close()
    assert packet.registers[40] == 0
    assert packet.registers[48] == 10


def test_no_contact_no_enable():
    packet = Packet()
    resistance = BodyResistance(packet, None)
    resistance.update(2000, 1)
    resistance.update(2010, 1.02)
    assert (40, [1]) not in packet.writes


def test_weight_fraction_rewrites_limit_and_zero_releases():
    packet, resistance = cue()
    resistance.update(2005, 1.02, effort=.25)
    assert resistance.active and packet.registers[48] == 2
    resistance.update(2010, 1.04, effort=.75)
    assert packet.registers[48] == 8
    resistance.update(2015, 1.06, effort=0)
    assert not resistance.active and packet.registers[40] == 0


def test_invalid_weight_fraction_never_enables():
    for effort in (-1, 1.1, float('nan')):
        packet, resistance = cue()
        resistance.update(2005, 1.02, effort=effort)
        assert resistance.fault and (40, [1]) not in packet.writes


def test_bench_returns_before_any_motion_publish():
    from types import SimpleNamespace
    from tacgello_ros.leader_publisher import TacGelloLeaderPublisher

    node = SimpleNamespace(
        control_mode='disabled', body_resistance=None,
        enable_gripper_haptic_feedback=False, body_resistance_bench=True,
        reader=SimpleNamespace(read_joints_and_gripper=lambda: ([0] * 6, 0)))
    # No motion publishers/RTDE interface exist on this stub: touching one fails.
    TacGelloLeaderPublisher.publish_state(node)
