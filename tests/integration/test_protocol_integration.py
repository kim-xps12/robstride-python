"""
Integration tests ensuring RobStrideMotor emits RS02-compliant CAN frames
"""

import struct
import pytest

from robstride.models import ParameterIndex, MotorState
from robstride.protocol.can_utils import (
    build_extended_can_id,
    encode_angle_16bit,
    encode_kd_16bit,
    encode_kp_16bit,
    encode_speed_16bit,
    encode_torque_16bit,
    encode_uint16,
)


def _last_message(mock_can_bus):
    assert mock_can_bus.send.call_count > 0, "No CAN frame was emitted"
    return mock_can_bus.send.call_args_list[-1][0][0]


def _extract_id_fields(msg):
    ext_id = msg.arbitration_id
    comm_type = (ext_id >> 24) & 0x1F
    data_field = (ext_id >> 8) & 0xFFFF
    motor_id = ext_id & 0xFF
    return comm_type, data_field, motor_id


@pytest.mark.integration
def test_enable_disable_sequence(mock_motor, mock_can_bus):
    motor = mock_motor
    motor.enable_motor()
    enable_msg = _last_message(mock_can_bus)
    ct, df, mid = _extract_id_fields(enable_msg)
    assert ct == 0x03
    assert (df >> 8) & 0xFF == motor.master_id
    assert (df & 0xFF) == 0x00
    assert mid == motor.motor_id
    assert enable_msg.data == bytes(8)

    mock_can_bus.send.reset_mock()
    motor.disable_motor(clear_error=True)
    disable_msg = _last_message(mock_can_bus)
    ct, df, mid = _extract_id_fields(disable_msg)
    assert ct == 0x04
    assert (df >> 8) & 0xFF == motor.master_id
    assert (df & 0xFF) == 0x00
    assert mid == motor.motor_id
    assert disable_msg.data[0] == 0x01
    assert list(disable_msg.data[1:]) == [0x00] * 7


@pytest.mark.integration
def test_set_zero_emits_correct_frame(mock_motor, mock_can_bus):
    motor = mock_motor
    motor.state = MotorState.DISABLED
    mock_can_bus.send.reset_mock()
    motor.set_zero_position()
    msg = _last_message(mock_can_bus)
    ct, df, mid = _extract_id_fields(msg)
    assert ct == 0x06
    assert (df >> 8) & 0xFF == motor.master_id
    assert (df & 0xFF) == 0x00
    assert mid == motor.motor_id
    assert msg.data[0] == 0x01
    assert list(msg.data[1:]) == [0x00] * 7


@pytest.mark.integration
def test_set_parameter_float_payload(mock_motor, mock_can_bus):
    motor = mock_motor
    mock_can_bus.send.reset_mock()
    motor.set_parameter(ParameterIndex.LIMIT_CUR, 10.0, value_mode='p')
    msg = _last_message(mock_can_bus)
    ct, df, mid = _extract_id_fields(msg)
    assert ct == 0x12
    assert (df >> 8) & 0xFF == motor.master_id
    assert (df & 0xFF) == 0x00
    assert mid == motor.motor_id
    assert msg.data[0] == ParameterIndex.LIMIT_CUR & 0xFF
    assert msg.data[1] == (ParameterIndex.LIMIT_CUR >> 8) & 0xFF
    assert msg.data[2:4] == b"\x00\x00"
    assert msg.data[4:8] == struct.pack("<f", 10.0)


@pytest.mark.integration
def test_set_parameter_mode_payload(mock_motor, mock_can_bus):
    motor = mock_motor
    mock_can_bus.send.reset_mock()
    motor.set_parameter(ParameterIndex.RUN_MODE, 2, value_mode='j')
    msg = _last_message(mock_can_bus)
    ct, df, mid = _extract_id_fields(msg)
    assert ct == 0x12
    assert (df >> 8) & 0xFF == motor.master_id
    assert (df & 0xFF) == 0x00
    assert mid == motor.motor_id
    assert msg.data[0] == ParameterIndex.RUN_MODE & 0xFF
    assert msg.data[1] == (ParameterIndex.RUN_MODE >> 8) & 0xFF
    assert msg.data[2:4] == b"\x00\x00"
    assert msg.data[4] == 0x02
    assert msg.data[5:] == b"\x00\x00\x00"


@pytest.mark.integration
def test_get_parameter_frame(mock_motor, mock_can_bus):
    motor = mock_motor
    mock_can_bus.send.reset_mock()
    motor.get_parameter(ParameterIndex.MECH_POS)
    msg = _last_message(mock_can_bus)
    ct, df, mid = _extract_id_fields(msg)
    assert ct == 0x11
    assert (df >> 8) & 0xFF == motor.master_id
    assert (df & 0xFF) == 0x00
    assert mid == motor.motor_id
    assert msg.data[0] == ParameterIndex.MECH_POS & 0xFF
    assert msg.data[1] == (ParameterIndex.MECH_POS >> 8) & 0xFF


@pytest.mark.integration
def test_save_parameters_frame(mock_motor, mock_can_bus):
    motor = mock_motor
    mock_can_bus.send.reset_mock()
    motor.save_parameters()
    msg = _last_message(mock_can_bus)
    ct, df, _ = _extract_id_fields(msg)
    assert ct == 0x16
    assert (df >> 8) & 0xFF == motor.master_id
    assert msg.data == bytes([0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08])


@pytest.mark.integration
def test_set_can_id_through_handler(mock_motor, mock_can_bus):
    motor = mock_motor
    mock_can_bus.send.reset_mock()
    motor.private_handler.send_set_can_id(0x55)
    msg = _last_message(mock_can_bus)
    ct, df, mid = _extract_id_fields(msg)
    assert ct == 0x07
    assert (df >> 8) & 0xFF == 0x55
    assert (df & 0xFF) == motor.master_id
    assert mid == motor.motor_id


@pytest.mark.integration
def test_auto_report_enable_disable(mock_motor, mock_can_bus):
    motor = mock_motor
    mock_can_bus.send.reset_mock()
    motor.private_handler.send_set_auto_report(True)
    msg = _last_message(mock_can_bus)
    ct, df, mid = _extract_id_fields(msg)
    assert ct == 0x18
    assert (df >> 8) & 0xFF == motor.master_id
    assert (df & 0xFF) == 0x00
    assert mid == motor.motor_id
    assert list(msg.data[0:6]) == [0x01, 0x02, 0x03, 0x04, 0x05, 0x06]
    assert msg.data[6] == 0x01
    assert msg.data[7] == 0x00

    motor.private_handler.send_set_auto_report(False)
    msg = _last_message(mock_can_bus)
    ct, df, mid = _extract_id_fields(msg)
    assert ct == 0x18
    assert (df >> 8) & 0xFF == motor.master_id
    assert (df & 0xFF) == 0x00
    assert mid == motor.motor_id
    assert list(msg.data[0:6]) == [0x01, 0x02, 0x03, 0x04, 0x05, 0x06]
    assert msg.data[6] == 0x00
    assert msg.data[7] == 0x00


@pytest.mark.integration
def test_motion_control_frame(mock_motor, mock_can_bus):
    motor = mock_motor
    mock_can_bus.send.reset_mock()
    motor.send_motion_control(torque=0.5, angle=1.0, speed=5.0, kp=50.0, kd=0.5)
    msg = _last_message(mock_can_bus)
    ct, df, mid = _extract_id_fields(msg)
    assert ct == 0x01
    assert mid == motor.motor_id
    assert df == encode_torque_16bit(0.5)

    expected_ext_id = build_extended_can_id(0x01, encode_torque_16bit(0.5), motor.master_id, motor.motor_id)
    assert msg.arbitration_id == expected_ext_id
    expected_payload = (
        encode_uint16(encode_angle_16bit(1.0))
        + encode_uint16(encode_speed_16bit(5.0))
        + encode_uint16(encode_kp_16bit(50.0))
        + encode_uint16(encode_kd_16bit(0.5))
    )
    assert msg.data == expected_payload
