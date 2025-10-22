"""
Unit tests for MITProtocolHandler command formatting and feedback decoding.
"""

import struct
import can
import pytest

from robstride.models import MITCommand, MITMotorType, MotorStatus
from robstride.protocol.mit import MITProtocolHandler


@pytest.fixture
def mit_handler(mock_can_bus):
    """MITProtocolHandler with mock CAN bus."""
    return MITProtocolHandler(motor_id=0x21, can_bus=mock_can_bus)


def _last_sent(mock_can_bus: can.Bus) -> can.Message:
    assert mock_can_bus.send.call_count > 0, "No CAN frame sent"
    return mock_can_bus.send.call_args[0][0]


def test_send_enable_disable_and_zero(mit_handler, mock_can_bus):
    mit_handler.send_enable()
    enable_msg = _last_sent(mock_can_bus)
    assert enable_msg.arbitration_id == mit_handler.motor_id
    assert enable_msg.is_extended_id is False
    assert enable_msg.data == bytes([0xFF] * 7 + [0xFC])

    mock_can_bus.send.reset_mock()
    mit_handler.send_disable()
    disable_msg = _last_sent(mock_can_bus)
    assert disable_msg.data == bytes([0xFF] * 7 + [0xFD])

    mock_can_bus.send.reset_mock()
    mit_handler.send_set_zero()
    zero_msg = _last_sent(mock_can_bus)
    assert zero_msg.data == bytes([0xFF] * 7 + [0xFE])


def test_send_composite_control_payload(mit_handler, mock_can_bus):
    cmd = MITCommand(position=1.2, velocity=-10.0, kp=100.0, kd=0.5, torque=5.0)
    mit_handler.send_composite_control(cmd)
    msg = _last_sent(mock_can_bus)

    # Unpack fields following MIT bit layout
    data = msg.data
    p_int = (data[0] << 8) | data[1]
    v_int = (data[2] << 4) | (data[3] >> 4)
    kp_int = ((data[3] & 0x0F) << 8) | data[4]
    kd_int = (data[5] << 4) | (data[6] >> 4)
    t_int = ((data[6] & 0x0F) << 8) | data[7]

    expected_p = mit_handler._float_to_uint(cmd.position, -12.57, 12.57, 16)
    expected_v = mit_handler._float_to_uint(cmd.velocity, -44.0, 44.0, 12)
    expected_kp = mit_handler._float_to_uint(cmd.kp, 0.0, 500.0, 12)
    expected_kd = mit_handler._float_to_uint(cmd.kd, 0.0, 5.0, 12)
    expected_t = mit_handler._float_to_uint(cmd.torque, -17.0, 17.0, 12)

    assert p_int == expected_p
    assert v_int == expected_v
    assert kp_int == expected_kp
    assert kd_int == expected_kd
    assert t_int == expected_t

    expected_bytes = bytearray(8)
    expected_bytes[0] = (expected_p >> 8) & 0xFF
    expected_bytes[1] = expected_p & 0xFF
    expected_bytes[2] = (expected_v >> 4) & 0xFF
    expected_bytes[3] = ((expected_v & 0x0F) << 4) | ((expected_kp >> 8) & 0x0F)
    expected_bytes[4] = expected_kp & 0xFF
    expected_bytes[5] = (expected_kd >> 4) & 0xFF
    expected_bytes[6] = ((expected_kd & 0x0F) << 4) | ((expected_t >> 8) & 0x0F)
    expected_bytes[7] = expected_t & 0xFF
    assert msg.data == bytes(expected_bytes)


def test_send_position_and_speed_control_frames(mit_handler, mock_can_bus):
    mit_handler.send_position_control(position=2.5, speed=3.0)
    pos_msg = _last_sent(mock_can_bus)
    assert pos_msg.arbitration_id == ((1 << 8) | mit_handler.motor_id)
    assert pos_msg.data == struct.pack('<ff', 2.5, 3.0)

    mock_can_bus.send.reset_mock()
    mit_handler.send_speed_control(speed=4.5, current_limit=6.0)
    spd_msg = _last_sent(mock_can_bus)
    assert spd_msg.arbitration_id == ((2 << 8) | mit_handler.motor_id)
    assert spd_msg.data == struct.pack('<ff', 4.5, 6.0)


def test_send_clear_error_and_set_motor_type(mit_handler, mock_can_bus):
    """Test MIT Command 5 (clear error) and Command 6 (set motor type)
    
    Per RS02 specification (rs02_ja.md:468):
    Command 5: FF FF FF FF FF FF F_CMD FB
    - F_CMD = 0xFF: Clear current errors
    - F_CMD = other: Check error status
    """
    mit_handler.send_clear_error(clear=True)
    clear_msg = _last_sent(mock_can_bus)
    assert clear_msg.data == bytes([0xFF] * 6 + [0xFF, 0xFB])  # Fixed: 0xFF for clear

    mock_can_bus.send.reset_mock()
    mit_handler.send_set_motor_type(MITMotorType.POSITION_CONTROL)
    type_msg = _last_sent(mock_can_bus)
    assert type_msg.data == bytes([0xFF] * 6 + [MITMotorType.POSITION_CONTROL, 0xFC])


def test_process_feedback_updates_status(mit_handler):
    status = MotorStatus()
    # Craft feedback frame following spec
    # NOTE: RS02応答コマンド1（MITプロトコル）では角度レンジが「-12.57〜12.57rad」と明記。
    #       全プロトコルで統一された±12.57radを使用（2025-10-16仕様書修正により確定）。
    data = bytes([
        mit_handler.motor_id,  # Byte0: motor id
        0x80, 0x00,            # Byte1-2: mid-scale position
        0x08,                  # Byte3: high 8 bits of velocity
        0x10,                  # Byte4: upper 4 bits velocity | upper 4 bits torque
        0x20,                  # Byte5: lower 8 bits torque
        0x00, 0x64             # Byte6-7: temperature = 0x0064 -> 100°C
    ])
    msg = can.Message(arbitration_id=0x200, data=data, is_extended_id=False, dlc=8)

    assert mit_handler.process_message(msg, status) is True
    p_int = (data[1] << 8) | data[2]
    v_int = (data[3] << 4) | ((data[4] >> 4) & 0x0F)
    t_int = ((data[4] & 0x0F) << 8) | data[5]
    expected_angle = mit_handler._uint_to_float(p_int, -12.57, 12.57, 16)
    expected_speed = mit_handler._uint_to_float(v_int, -44.0, 44.0, 12)
    expected_torque = mit_handler._uint_to_float(t_int, -17.0, 17.0, 12)
    assert status.angle == pytest.approx(expected_angle, rel=0.0, abs=1e-6)
    assert status.speed == pytest.approx(expected_speed, rel=0.0, abs=1e-6)
    assert status.torque == pytest.approx(expected_torque, rel=0.0, abs=1e-6)
    assert pytest.approx(status.temperature, rel=0.0, abs=1e-6) == 100.0
    assert status.pattern == 2
    assert status.error_code == 0


def test_mit_feedback_boundary_values(mit_handler):
    """Test MIT feedback decoding at 12-bit boundary values (0, 2047, 4095)"""
    status = MotorStatus()
    
    # Test velocity at boundary: 0 (minimum in 12-bit)
    # Velocity: bits [Byte3:8][Byte4:4(high)]
    data_v_min = bytes([
        mit_handler.motor_id,
        0x00, 0x00,  # position = 0
        0x00,        # velocity high 8 bits = 0
        0x00,        # velocity low 4 bits = 0, torque high 4 bits = 0
        0x00,        # torque low 8 bits = 0
        0x00, 0x01   # temperature = 1°C (non-zero to avoid error detection)
    ])
    msg = can.Message(arbitration_id=0x200, data=data_v_min, is_extended_id=False, dlc=8)
    assert mit_handler.process_message(msg, status) is True
    assert status.speed == pytest.approx(-44.0, abs=0.1)  # 0 maps to minimum
    
    # Test velocity at mid-point: 2047 (middle of 12-bit range)
    data_v_mid = bytes([
        mit_handler.motor_id,
        0x00, 0x00,
        0x7F,        # velocity high 8 bits = 0x7F
        0xF0,        # velocity low 4 bits = 0xF (0x7FF = 2047)
        0x00,
        0x00, 0x01
    ])
    msg = can.Message(arbitration_id=0x200, data=data_v_mid, is_extended_id=False, dlc=8)
    assert mit_handler.process_message(msg, status) is True
    assert status.speed == pytest.approx(0.0, abs=0.5)  # 2047 should map close to 0
    
    # Test velocity at maximum: 4095 (maximum 12-bit value)
    data_v_max = bytes([
        mit_handler.motor_id,
        0x00, 0x00,
        0xFF,        # velocity high 8 bits = 0xFF
        0xF0,        # velocity low 4 bits = 0xF (0xFFF = 4095)
        0x00,
        0x00, 0x01
    ])
    msg = can.Message(arbitration_id=0x200, data=data_v_max, is_extended_id=False, dlc=8)
    assert mit_handler.process_message(msg, status) is True
    assert status.speed == pytest.approx(44.0, abs=0.1)  # 4095 maps to maximum
    
    # Test torque at boundary: 0 (minimum in 12-bit)
    # Torque: bits [Byte4:4(low)][Byte5:8]
    data_t_min = bytes([
        mit_handler.motor_id,
        0x00, 0x00,
        0x00,
        0x00,        # torque high 4 bits = 0
        0x00,        # torque low 8 bits = 0
        0x00, 0x01
    ])
    msg = can.Message(arbitration_id=0x200, data=data_t_min, is_extended_id=False, dlc=8)
    assert mit_handler.process_message(msg, status) is True
    assert status.torque == pytest.approx(-17.0, abs=0.1)
    
    # Test torque at mid-point: 2047
    data_t_mid = bytes([
        mit_handler.motor_id,
        0x00, 0x00,
        0x00,
        0x07,        # torque high 4 bits = 0x7
        0xFF,        # torque low 8 bits = 0xFF (0x7FF = 2047)
        0x00, 0x01
    ])
    msg = can.Message(arbitration_id=0x200, data=data_t_mid, is_extended_id=False, dlc=8)
    assert mit_handler.process_message(msg, status) is True
    assert status.torque == pytest.approx(0.0, abs=0.5)
    
    # Test torque at maximum: 4095
    data_t_max = bytes([
        mit_handler.motor_id,
        0x00, 0x00,
        0x00,
        0x0F,        # torque high 4 bits = 0xF
        0xFF,        # torque low 8 bits = 0xFF (0xFFF = 4095)
        0x00, 0x01
    ])
    msg = can.Message(arbitration_id=0x200, data=data_t_max, is_extended_id=False, dlc=8)
    assert mit_handler.process_message(msg, status) is True
    assert status.torque == pytest.approx(17.0, abs=0.1)


def test_mit_feedback_position_boundary_values(mit_handler):
    """Test MIT feedback position decoding at 16-bit boundary values"""
    status = MotorStatus()
    
    # Test position at minimum: 0
    data_p_min = bytes([
        mit_handler.motor_id,
        0x00, 0x00,  # position = 0
        0x00, 0x00, 0x00,
        0x00, 0x01   # temperature = 1°C (non-zero to avoid error detection)
    ])
    msg = can.Message(arbitration_id=0x200, data=data_p_min, is_extended_id=False, dlc=8)
    assert mit_handler.process_message(msg, status) is True
    assert status.angle == pytest.approx(-12.57, abs=0.1)
    
    # Test position at mid-point: 32767
    data_p_mid = bytes([
        mit_handler.motor_id,
        0x7F, 0xFF,  # position = 32767
        0x00, 0x00, 0x00,
        0x00, 0x01
    ])
    msg = can.Message(arbitration_id=0x200, data=data_p_mid, is_extended_id=False, dlc=8)
    assert mit_handler.process_message(msg, status) is True
    assert status.angle == pytest.approx(0.0, abs=0.1)
    
    # Test position at maximum: 65535
    data_p_max = bytes([
        mit_handler.motor_id,
        0xFF, 0xFF,  # position = 65535
        0x00, 0x00, 0x00,
        0x00, 0x01
    ])
    msg = can.Message(arbitration_id=0x200, data=data_p_max, is_extended_id=False, dlc=8)
    assert mit_handler.process_message(msg, status) is True
    assert status.angle == pytest.approx(12.57, abs=0.1)


def test_clear_error_response_byte1_parsing(mit_handler):
    """
    Test MIT Command 5 (clear error) response parsing.
    
    Per RS02 specification (rs02_ja.md:468): 
    "いずれの値でも、応答の BYTE1 にエラー値が返されます"
    (In any case, the error value is returned in BYTE1 of the response)
    
    This test verifies that error codes in Byte1 are correctly extracted
    and stored in status.error_code.
    """
    status = MotorStatus()
    
    # Test case 1: No error (Byte1 = 0x00)
    data_no_error = bytes([
        mit_handler.motor_id,  # Byte0: motor id
        0x00,                  # Byte1: error code = 0 (no error)
        0x00, 0x00,            # Byte2-3: reserved (zeros indicate error response)
        0x00, 0x00, 0x00, 0x00 # Byte4-7: other data (zeros)
    ])
    msg = can.Message(arbitration_id=mit_handler.motor_id, data=data_no_error, is_extended_id=False, dlc=8)
    assert mit_handler.process_message(msg, status) is True
    assert status.error_code == 0x00, "Error code should be 0x00 (no error)"
    
    # Test case 2: Under-voltage error (Byte1 = 0x02, bit1)
    data_under_voltage = bytes([
        mit_handler.motor_id,
        0x02,                  # Byte1: error code = 0x02 (under-voltage, bit1)
        0x00, 0x00,
        0x00, 0x00, 0x00, 0x00
    ])
    msg = can.Message(arbitration_id=mit_handler.motor_id, data=data_under_voltage, is_extended_id=False, dlc=8)
    assert mit_handler.process_message(msg, status) is True
    assert status.error_code == 0x02, "Error code should be 0x02 (under-voltage)"
    
    # Test case 3: Multiple errors (Byte1 = 0x0A, bit1 + bit3)
    data_multiple_errors = bytes([
        mit_handler.motor_id,
        0x0A,                  # Byte1: error code = 0x0A (under-voltage + over-pressure)
        0x00, 0x00,
        0x00, 0x00, 0x00, 0x00
    ])
    msg = can.Message(arbitration_id=mit_handler.motor_id, data=data_multiple_errors, is_extended_id=False, dlc=8)
    assert mit_handler.process_message(msg, status) is True
    assert status.error_code == 0x0A, "Error code should be 0x0A (multiple errors)"
    
    # Test case 4: Motor over-temperature (Byte1 = 0x01, bit0)
    data_over_temp = bytes([
        mit_handler.motor_id,
        0x01,                  # Byte1: error code = 0x01 (motor over-temp, bit0)
        0x00, 0x00,
        0x00, 0x00, 0x00, 0x00
    ])
    msg = can.Message(arbitration_id=mit_handler.motor_id, data=data_over_temp, is_extended_id=False, dlc=8)
    assert mit_handler.process_message(msg, status) is True
    assert status.error_code == 0x01, "Error code should be 0x01 (motor over-temperature)"
    
    # Test case 5: High error code (Byte1 = 0x80, bit7)
    data_high_error = bytes([
        mit_handler.motor_id,
        0x80,                  # Byte1: error code = 0x80 (encoder uncalibrated, bit7)
        0x00, 0x00,
        0x00, 0x00, 0x00, 0x00
    ])
    msg = can.Message(arbitration_id=mit_handler.motor_id, data=data_high_error, is_extended_id=False, dlc=8)
    assert mit_handler.process_message(msg, status) is True
    assert status.error_code == 0x80, "Error code should be 0x80 (encoder uncalibrated)"


