"""
Unit tests for PrivateProtocolHandler command formatting
"""

import struct
import pytest

from robstride.models import CommunicationType, MotionControlCommand, ParameterIndex, get_parameter_spec
from robstride.protocol.private import PrivateProtocolHandler
from robstride.protocol.can_utils import (
    encode_torque_16bit,
    encode_angle_16bit,
    encode_speed_16bit,
    encode_speed_auto_report_16bit,
    encode_torque_auto_report_16bit,
    encode_angle_auto_report_16bit,
    encode_kp_16bit,
    encode_kd_16bit,
    encode_uint16,
)


@pytest.fixture
def handler(mock_can_bus):
    """PrivateProtocolHandler with mock CAN bus"""
    return PrivateProtocolHandler(motor_id=0x7F, can_bus=mock_can_bus, master_id=0xFD)


def _extract_fields(msg):
    ext_id = msg.arbitration_id
    comm_type = (ext_id >> 24) & 0x1F
    data_field = (ext_id >> 8) & 0xFFFF
    motor_id = ext_id & 0xFF
    return comm_type, data_field, motor_id


def test_send_enable_message(handler, mock_can_bus):
    handler.send_enable()
    mock_can_bus.send.assert_called_once()
    msg = mock_can_bus.send.call_args[0][0]
    comm_type, data_field, motor_id = _extract_fields(msg)

    assert comm_type == CommunicationType.MOTOR_ENABLE
    assert (data_field >> 8) & 0xFF == handler.master_id
    assert (data_field & 0xFF) == 0x00
    assert motor_id == handler.motor_id
    assert msg.data == bytes(8)


@pytest.mark.parametrize("clear_error,expected_byte", [(False, 0x00), (True, 0x01)])
def test_send_disable_message(handler, mock_can_bus, clear_error, expected_byte):
    handler.send_disable(clear_error=clear_error)
    msg = mock_can_bus.send.call_args[0][0]
    comm_type, data_field, motor_id = _extract_fields(msg)

    assert comm_type == CommunicationType.MOTOR_STOP
    assert (data_field >> 8) & 0xFF == handler.master_id
    assert (data_field & 0xFF) == 0x00
    assert motor_id == handler.motor_id
    assert msg.data[0] == expected_byte
    assert list(msg.data[1:]) == [0x00] * 7


def test_send_set_zero_message(handler, mock_can_bus):
    handler.send_set_zero()
    msg = mock_can_bus.send.call_args[0][0]
    comm_type, data_field, _ = _extract_fields(msg)

    assert comm_type == CommunicationType.SET_POS_ZERO
    assert (data_field >> 8) & 0xFF == handler.master_id
    assert (data_field & 0xFF) == 0x00
    assert msg.data[0] == 0x01
    assert list(msg.data[1:]) == [0x00] * 7


def test_send_set_can_id_message(handler, mock_can_bus):
    new_id = 0x22
    handler.send_set_can_id(new_id)
    msg = mock_can_bus.send.call_args[0][0]
    comm_type, data_field, motor_id = _extract_fields(msg)

    assert comm_type == CommunicationType.CAN_ID
    assert (data_field >> 8) & 0xFF == new_id  # new CAN ID in upper byte
    assert (data_field & 0xFF) == handler.master_id  # master ID in lower byte
    assert motor_id == handler.motor_id
    assert msg.data == bytes(8)


def test_send_get_parameter_payload(handler, mock_can_bus):
    index = ParameterIndex.LOC_REF
    handler.send_get_parameter(index)
    msg = mock_can_bus.send.call_args[0][0]
    comm_type, data_field, motor_id = _extract_fields(msg)

    assert comm_type == CommunicationType.GET_SINGLE_PARAMETER
    assert (data_field >> 8) & 0xFF == handler.master_id
    assert (data_field & 0xFF) == 0x00
    assert motor_id == handler.motor_id
    assert msg.data[0] == index & 0xFF
    assert msg.data[1] == (index >> 8) & 0xFF
    assert msg.data[2:4] == b"\x00\x00"
    assert msg.data[4:] == b"\x00\x00\x00\x00"


def test_send_set_parameter_float(handler, mock_can_bus):
    index = ParameterIndex.LIMIT_SPD
    value = 12.5
    handler.send_set_parameter(index, value)
    msg = mock_can_bus.send.call_args[0][0]

    assert msg.data[0] == index & 0xFF
    assert msg.data[1] == (index >> 8) & 0xFF
    assert msg.data[2:4] == b"\x00\x00"
    assert msg.data[4:8] == struct.pack("<f", value)


def test_send_set_parameter_mode(handler, mock_can_bus):
    index = ParameterIndex.RUN_MODE
    handler.send_set_parameter(index, 2, value_mode='j')
    msg = mock_can_bus.send.call_args[0][0]

    assert msg.data[0] == index & 0xFF
    assert msg.data[1] == (index >> 8) & 0xFF
    assert msg.data[2:4] == b"\x00\x00"
    assert msg.data[4] == 0x02
    assert msg.data[5:] == b"\x00\x00\x00"


@pytest.mark.parametrize("enable,expected_cmd", [(True, 0x01), (False, 0x00)])
def test_send_set_auto_report(handler, mock_can_bus, enable, expected_cmd):
    handler.send_set_auto_report(enable)
    msg = mock_can_bus.send.call_args[0][0]
    comm_type, data_field, _ = _extract_fields(msg)

    assert comm_type == CommunicationType.PROACTIVE_ESCALATION_SET
    assert (data_field >> 8) & 0xFF == handler.master_id
    assert (data_field & 0xFF) == 0x00
    assert list(msg.data[0:6]) == [0x01, 0x02, 0x03, 0x04, 0x05, 0x06]
    assert msg.data[6] == expected_cmd
    assert msg.data[7] == 0x00


@pytest.mark.parametrize("mode", [0x00, 0x01, 0x02])
def test_send_set_protocol_mode(handler, mock_can_bus, mode):
    handler.send_set_protocol_mode(mode)
    msg = mock_can_bus.send.call_args[0][0]
    comm_type, data_field, _ = _extract_fields(msg)

    assert comm_type == CommunicationType.MOTOR_MODE_SET
    assert (data_field >> 8) & 0xFF == handler.master_id
    assert (data_field & 0xFF) == 0x00
    assert list(msg.data[0:6]) == [0x01, 0x02, 0x03, 0x04, 0x05, 0x06]
    assert msg.data[6] == mode
    assert msg.data[7] == 0x00


@pytest.mark.parametrize("baud_code", [0x01, 0x02, 0x03, 0x04])
def test_send_change_baud_rate(handler, mock_can_bus, baud_code):
    handler.send_change_baud_rate(baud_code)
    msg = mock_can_bus.send.call_args[0][0]
    comm_type, data_field, _ = _extract_fields(msg)

    assert comm_type == CommunicationType.BAUD_RATE_CHANGE
    assert (data_field >> 8) & 0xFF == handler.master_id
    assert (data_field & 0xFF) == 0x00
    assert list(msg.data[0:6]) == [0x01, 0x02, 0x03, 0x04, 0x05, 0x06]
    assert msg.data[6] == baud_code
    assert msg.data[7] == 0x00


def test_baud_rate_change_response_processing(handler, motor_status, mock_can_message):
    """
    Test BAUD_RATE_CHANGE (Type 0x17) response processing.
    
    Per RS02 specification (rs02_ja.md:381):
    "応答フレーム：モータブロードキャスト応答フレーム（通信タイプ0を参照）"
    (Response frame: Motor broadcast response frame, refer to Type 0x00)
    "再通電後に有効"
    (Effective after power cycle)
    
    Note: Requires hardware verification for actual response behavior.
    """
    # Simulate Type 0x00 (GET_ID) broadcast response after baud rate change
    unique_id = 0x1234567890ABCDEF
    data_field = (handler.motor_id << 8) | 0x00
    ext_id = ((CommunicationType.GET_ID & 0x1F) << 24) | (data_field << 8) | 0xFE
    payload = list(struct.pack(">Q", unique_id))
    msg = mock_can_message(arbitration_id=ext_id, data=payload)
    
    assert handler.process_message(msg, motor_status) is True
    assert motor_status.device_id == handler.motor_id
    assert motor_status.device_uid == unique_id


def test_protocol_mode_change_response_processing(handler, motor_status, mock_can_message):
    """
    Test MOTOR_MODE_SET (Type 0x19) response processing.
    
    Per RS02 specification (rs02_ja.md:393):
    "応答フレーム：モータフィードバック応答フレーム（通信タイプ0を参照）"
    (Response frame: Motor feedback response frame, refer to Type 0x00)
    "再通電後に有効になります"
    (Effective after power cycle)
    
    Note: Requires hardware verification for actual response behavior.
    """
    # Simulate Type 0x00 (GET_ID) response after protocol mode change
    unique_id = 0xFEDCBA0987654321
    data_field = (handler.motor_id << 8) | 0x00
    ext_id = ((CommunicationType.GET_ID & 0x1F) << 24) | (data_field << 8) | 0xFE
    payload = list(struct.pack(">Q", unique_id))
    msg = mock_can_message(arbitration_id=ext_id, data=payload)
    
    assert handler.process_message(msg, motor_status) is True
    assert motor_status.device_id == handler.motor_id
    assert motor_status.device_uid == unique_id
    assert handler.last_device_id == handler.motor_id
    assert handler.last_device_uid == unique_id


def test_send_save_parameters(handler, mock_can_bus):
    handler.send_save_parameters()
    msg = mock_can_bus.send.call_args[0][0]
    comm_type, data_field, _ = _extract_fields(msg)

    assert comm_type == CommunicationType.MOTOR_DATA_SAVE
    assert (data_field >> 8) & 0xFF == handler.master_id
    assert (data_field & 0xFF) == 0x00
    assert list(msg.data) == [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08]


def test_motor_data_save_response_processing(handler, motor_status, mock_can_message):
    """
    Test MOTOR_DATA_SAVE (Type 0x16) response processing.
    
    Per RS02 specification (rs02_ja.md:373):
    "応答フレーム：モータフィードバック応答フレーム（通信タイプ2を参照）"
    (Response frame: Motor feedback response frame, refer to Type 0x02)
    
    Note: Actual hardware response behavior requires verification.
    This test documents the expected behavior per specification.
    """
    # Simulate Type 0x02 response after save command
    pattern = 0x02
    error_code = 0x00  # No error after successful save
    data_field = (pattern << 14) | (error_code << 8) | handler.motor_id
    ext_id = ((CommunicationType.MOTOR_REQUEST & 0x1F) << 24) | (data_field << 8) | handler.master_id
    
    from robstride.protocol.can_utils import encode_uint16, encode_angle_16bit, encode_speed_16bit, encode_torque_16bit
    
    payload = (
        encode_uint16(encode_angle_16bit(0.5))
        + encode_uint16(encode_speed_16bit(1.0))
        + encode_uint16(encode_torque_16bit(0.2))
        + encode_uint16(300)  # 30.0°C
    )
    msg = mock_can_message(arbitration_id=ext_id, data=list(payload))
    
    assert handler.process_message(msg, motor_status) is True
    assert motor_status.pattern == pattern
    assert motor_status.error_code == error_code
    assert motor_status.angle == pytest.approx(0.5, rel=0.05)
    assert motor_status.temperature == pytest.approx(30.0, rel=0.01)


def test_send_motion_control(handler, mock_can_bus):
    cmd = MotionControlCommand(torque=1.0, angle=0.5, speed=2.0, kp=100.0, kd=1.0)
    handler.send_motion_control(cmd)
    msg = mock_can_bus.send.call_args[0][0]
    comm_type, data_field, motor_id = _extract_fields(msg)

    assert comm_type == CommunicationType.MOTION_CONTROL
    assert motor_id == handler.motor_id
    assert data_field == encode_torque_16bit(cmd.torque)

    # Payload bytes are big-endian pairs for angle, speed, kp, kd
    expected_payload = (
        encode_uint16(encode_angle_16bit(cmd.angle))
        + encode_uint16(encode_speed_16bit(cmd.speed))
        + encode_uint16(encode_kp_16bit(cmd.kp))
        + encode_uint16(encode_kd_16bit(cmd.kd))
    )
    assert msg.data == expected_payload


def test_process_motor_status(handler, motor_status, mock_can_message):
    pattern = 0x02
    error_code = 0x15
    data_field = (pattern << 14) | (error_code << 8) | handler.motor_id
    ext_id = ((CommunicationType.MOTOR_REQUEST & 0x1F) << 24) | (data_field << 8) | handler.master_id
    payload = (
        encode_uint16(encode_angle_16bit(1.0))
        + encode_uint16(encode_speed_16bit(-5.0))
        + encode_uint16(encode_torque_16bit(1.5))
        + encode_uint16(320)  # 32.0°C feedback (value * 10)
    )
    msg = mock_can_message(arbitration_id=ext_id, data=list(payload))

    assert handler.process_message(msg, motor_status) is True
    assert motor_status.pattern == pattern
    assert motor_status.error_code == error_code
    assert motor_status.angle == pytest.approx(1.0, rel=0.05)
    assert motor_status.speed == pytest.approx(-5.0, rel=0.05)
    assert motor_status.torque == pytest.approx(1.5, rel=0.05)
    assert motor_status.temperature == pytest.approx(32.0, rel=0.01)


def test_process_auto_report(handler, motor_status, mock_can_message):
    """
    Test auto-report feedback processing with corrected RS02 specification.
    
    Per RS02 spec (corrected 2025-10-16), Type 0x18 uses:
    - Angle: −12.57f〜12.57f (same as Type 0x02, ±12.57 rad)
    - Speed: −33rad/s〜33rad/s (different from Type 0x02's ±44 rad/s)
    - Torque: −14Nm〜14Nm (different from Type 0x02's ±17 Nm)
    """
    pattern = 0x01
    error_code = 0x08
    data_field = (pattern << 14) | (error_code << 8) | handler.motor_id
    ext_id = ((CommunicationType.PROACTIVE_ESCALATION_SET & 0x1F) << 24) | (data_field << 8) | handler.master_id
    payload = (
        encode_uint16(encode_angle_auto_report_16bit(-0.8))  # Now uses standard ±12.57 rad range
        + encode_uint16(encode_speed_auto_report_16bit(3.2))  # Uses ±33 rad/s range
        + encode_uint16(encode_torque_auto_report_16bit(0.7))  # Uses ±14 Nm range
        + encode_uint16(410)  # 41.0°C
    )
    msg = mock_can_message(arbitration_id=ext_id, data=list(payload))

    assert handler.process_message(msg, motor_status) is True
    assert motor_status.pattern == pattern
    assert motor_status.error_code == error_code
    assert motor_status.temperature == pytest.approx(41.0, rel=0.01)
    assert motor_status.angle == pytest.approx(-0.8, rel=0.05)
    assert motor_status.speed == pytest.approx(3.2, rel=0.05)
    assert motor_status.torque == pytest.approx(0.7, rel=0.05)


def test_auto_report_velocity_torque_boundary_values(handler, motor_status, mock_can_message):
    """
    Test Type 0x18 auto-report speed and torque at boundary values.
    
    Per RS02 specification (corrected 2025-10-16):
    - Angle: −12.57f〜12.57f (±12.57 rad, same as Type 0x02)
    - Speed: −33rad/s〜33rad/s (different from Type 0x02's ±44 rad/s)
    - Torque: −14Nm〜14Nm (different from Type 0x02's ±17 Nm)
    
    This test verifies encoding/decoding accuracy at boundary values.
    """
    pattern = 0x02
    error_code = 0x00
    data_field = (pattern << 14) | (error_code << 8) | handler.motor_id
    ext_id = ((CommunicationType.PROACTIVE_ESCALATION_SET & 0x1F) << 24) | (data_field << 8) | handler.master_id
    
    # Test case 1: Maximum speed (+33 rad/s)
    payload_speed_max = (
        encode_uint16(encode_angle_auto_report_16bit(0.0))
        + encode_uint16(encode_speed_auto_report_16bit(33.0))  # Maximum speed
        + encode_uint16(encode_torque_auto_report_16bit(0.0))
        + encode_uint16(250)  # 25.0°C
    )
    msg = mock_can_message(arbitration_id=ext_id, data=list(payload_speed_max))
    assert handler.process_message(msg, motor_status) is True
    assert motor_status.speed == pytest.approx(33.0, abs=0.1), "Speed should be ~33.0 rad/s at maximum"
    
    # Test case 2: Minimum speed (-33 rad/s)
    payload_speed_min = (
        encode_uint16(encode_angle_auto_report_16bit(0.0))
        + encode_uint16(encode_speed_auto_report_16bit(-33.0))  # Minimum speed
        + encode_uint16(encode_torque_auto_report_16bit(0.0))
        + encode_uint16(250)
    )
    msg = mock_can_message(arbitration_id=ext_id, data=list(payload_speed_min))
    assert handler.process_message(msg, motor_status) is True
    assert motor_status.speed == pytest.approx(-33.0, abs=0.1), "Speed should be ~-33.0 rad/s at minimum"
    
    # Test case 3: Zero speed
    payload_speed_zero = (
        encode_uint16(encode_angle_auto_report_16bit(0.0))
        + encode_uint16(encode_speed_auto_report_16bit(0.0))  # Zero speed
        + encode_uint16(encode_torque_auto_report_16bit(0.0))
        + encode_uint16(250)
    )
    msg = mock_can_message(arbitration_id=ext_id, data=list(payload_speed_zero))
    assert handler.process_message(msg, motor_status) is True
    assert motor_status.speed == pytest.approx(0.0, abs=0.1), "Speed should be ~0.0 rad/s"
    
    # Test case 4: Maximum torque (+14 Nm)
    payload_torque_max = (
        encode_uint16(encode_angle_auto_report_16bit(0.0))
        + encode_uint16(encode_speed_auto_report_16bit(0.0))
        + encode_uint16(encode_torque_auto_report_16bit(14.0))  # Maximum torque
        + encode_uint16(250)
    )
    msg = mock_can_message(arbitration_id=ext_id, data=list(payload_torque_max))
    assert handler.process_message(msg, motor_status) is True
    assert motor_status.torque == pytest.approx(14.0, abs=0.1), "Torque should be ~14.0 Nm at maximum"
    
    # Test case 5: Minimum torque (-14 Nm)
    payload_torque_min = (
        encode_uint16(encode_angle_auto_report_16bit(0.0))
        + encode_uint16(encode_speed_auto_report_16bit(0.0))
        + encode_uint16(encode_torque_auto_report_16bit(-14.0))  # Minimum torque
        + encode_uint16(250)
    )
    msg = mock_can_message(arbitration_id=ext_id, data=list(payload_torque_min))
    assert handler.process_message(msg, motor_status) is True
    assert motor_status.torque == pytest.approx(-14.0, abs=0.1), "Torque should be ~-14.0 Nm at minimum"
    
    # Test case 6: Zero torque
    payload_torque_zero = (
        encode_uint16(encode_angle_auto_report_16bit(0.0))
        + encode_uint16(encode_speed_auto_report_16bit(0.0))
        + encode_uint16(encode_torque_auto_report_16bit(0.0))  # Zero torque
        + encode_uint16(250)
    )
    msg = mock_can_message(arbitration_id=ext_id, data=list(payload_torque_zero))
    assert handler.process_message(msg, motor_status) is True
    assert motor_status.torque == pytest.approx(0.0, abs=0.1), "Torque should be ~0.0 Nm"
    
    # Test case 7: Combined boundary values (max speed + max torque)
    payload_combined_max = (
        encode_uint16(encode_angle_auto_report_16bit(12.57))  # Max angle
        + encode_uint16(encode_speed_auto_report_16bit(33.0))  # Max speed
        + encode_uint16(encode_torque_auto_report_16bit(14.0))  # Max torque
        + encode_uint16(1350)  # 135.0°C (over-temp threshold)
    )
    msg = mock_can_message(arbitration_id=ext_id, data=list(payload_combined_max))
    assert handler.process_message(msg, motor_status) is True
    assert motor_status.angle == pytest.approx(12.57, abs=0.1)
    assert motor_status.speed == pytest.approx(33.0, abs=0.1)
    assert motor_status.torque == pytest.approx(14.0, abs=0.1)
    assert motor_status.temperature == pytest.approx(135.0, abs=0.1)
    
    # Test case 8: Combined boundary values (min speed + min torque)
    payload_combined_min = (
        encode_uint16(encode_angle_auto_report_16bit(-12.57))  # Min angle
        + encode_uint16(encode_speed_auto_report_16bit(-33.0))  # Min speed
        + encode_uint16(encode_torque_auto_report_16bit(-14.0))  # Min torque
        + encode_uint16(0)  # 0.0°C
    )
    msg = mock_can_message(arbitration_id=ext_id, data=list(payload_combined_min))
    assert handler.process_message(msg, motor_status) is True
    assert motor_status.angle == pytest.approx(-12.57, abs=0.1)
    assert motor_status.speed == pytest.approx(-33.0, abs=0.1)
    assert motor_status.torque == pytest.approx(-14.0, abs=0.1)
    assert motor_status.temperature == pytest.approx(0.0, abs=0.1)


def test_process_parameter_response(handler, motor_status, parameter_data, mock_can_message):
    index = ParameterIndex.LIMIT_CUR
    value = 12.34
    data_field = (handler.master_id << 8) | 0x00
    ext_id = ((CommunicationType.GET_SINGLE_PARAMETER & 0x1F) << 24) | (data_field << 8) | handler.master_id
    payload = [
        index & 0xFF,
        (index >> 8) & 0xFF,
        0x00,
        0x00,
        *struct.pack("<f", value)
    ]
    msg = mock_can_message(arbitration_id=ext_id, data=payload)

    assert handler.process_message(msg, motor_status, parameter_data) is True
    assert parameter_data.limit_cur == pytest.approx(value, rel=0.0, abs=1e-6)


def test_process_get_id_response(handler, motor_status, mock_can_message):
    unique_id = 0x0123456789ABCDEF
    data_field = (handler.motor_id << 8) | 0x00
    ext_id = ((CommunicationType.GET_ID & 0x1F) << 24) | (data_field << 8) | 0xFE
    payload = list(struct.pack(">Q", unique_id))
    msg = mock_can_message(arbitration_id=ext_id, data=payload)

    assert handler.process_message(msg, motor_status) is True
    assert handler.last_device_uid == unique_id
    assert handler.last_device_id == handler.motor_id
    assert motor_status.device_id == handler.motor_id
    assert motor_status.device_uid == unique_id


def test_get_id_64bit_endianness_boundary_values(handler, motor_status, mock_can_message):
    """
    Test GET_ID (Type 0x00) 64-bit UID encoding with boundary values.
    
    Per current implementation: Big-endian (">Q")
    Note: RS02 specification doesn't explicitly state endianness.
    This test documents the current implementation assumption.
    Hardware verification recommended.
    """
    data_field = (handler.motor_id << 8) | 0x00
    ext_id = ((CommunicationType.GET_ID & 0x1F) << 24) | (data_field << 8) | 0xFE
    
    # Test case 1: Minimum value (0x0000000000000000)
    uid_min = 0x0000000000000000
    payload = list(struct.pack(">Q", uid_min))
    msg = mock_can_message(arbitration_id=ext_id, data=payload)
    assert handler.process_message(msg, motor_status) is True
    assert motor_status.device_uid == uid_min
    
    # Test case 2: Maximum value (0xFFFFFFFFFFFFFFFF)
    uid_max = 0xFFFFFFFFFFFFFFFF
    payload = list(struct.pack(">Q", uid_max))
    msg = mock_can_message(arbitration_id=ext_id, data=payload)
    assert handler.process_message(msg, motor_status) is True
    assert motor_status.device_uid == uid_max
    
    # Test case 3: Pattern with clear byte boundaries (0x0102030405060708)
    uid_pattern = 0x0102030405060708
    payload = list(struct.pack(">Q", uid_pattern))
    msg = mock_can_message(arbitration_id=ext_id, data=payload)
    assert handler.process_message(msg, motor_status) is True
    assert motor_status.device_uid == uid_pattern
    # Verify byte order (big-endian: high byte first)
    assert payload[0] == 0x01  # MSB first in big-endian
    assert payload[7] == 0x08  # LSB last in big-endian
    
    # Test case 4: Alternating bits pattern (0xAAAAAAAAAAAAAAAA)
    uid_alt = 0xAAAAAAAAAAAAAAAA
    payload = list(struct.pack(">Q", uid_alt))
    msg = mock_can_message(arbitration_id=ext_id, data=payload)
    assert handler.process_message(msg, motor_status) is True
    assert motor_status.device_uid == uid_alt


def test_process_error_feedback(handler, motor_status, mock_can_message):
    fault = 0x12345678
    warning = 0x9ABCDEF0
    data_field = (handler.motor_id << 8) | 0x00
    ext_id = ((CommunicationType.ERROR_FEEDBACK & 0x1F) << 24) | (data_field << 8) | handler.master_id
    payload = list(struct.pack(">I", fault) + struct.pack(">I", warning))
    msg = mock_can_message(arbitration_id=ext_id, data=payload)

    assert handler.process_message(msg, motor_status) is True
    assert motor_status.fault_code == fault
    assert motor_status.warning_code == warning


def test_error_feedback_bit_field_parsing(handler, motor_status, mock_can_message):
    """
    Test ERROR_FEEDBACK (Type 0x15) individual bit field parsing.
    
    Per RS02 specification, fault field contains:
    - bit10: モータ過温障害 (Motor over-temperature, 135°C)
    - bit11: ドライバチップ障害 (Driver chip fault)
    - bit12: 欠圧障害 (Under-voltage fault)
    - bit13: 過温障害 (Controller over-temperature fault)
    - bit14: 積分過大障害 (Over-integration fault)
    - bit17: 磁気エンコーダ未校定 (Magnetic encoder uncalibrated)
    
    Warning field contains:
    - bit0: モータ過温警告 (Motor over-temperature warning, 125°C)
    """
    data_field = (handler.motor_id << 8) | 0x00
    ext_id = ((CommunicationType.ERROR_FEEDBACK & 0x1F) << 24) | (data_field << 8) | handler.master_id
    
    # Test case 1: bit14 (over-integration fault) only
    fault_bit14 = 1 << 14  # 0x00004000
    payload = list(struct.pack(">I", fault_bit14) + struct.pack(">I", 0))
    msg = mock_can_message(arbitration_id=ext_id, data=payload)
    
    assert handler.process_message(msg, motor_status) is True
    assert motor_status.fault_code & (1 << 14) != 0, "bit14 (over-integration) should be set"
    assert motor_status.fault_code & (1 << 13) == 0, "bit13 should not be set"
    
    # Test case 2: bit17 (encoder uncalibrated)
    fault_bit17 = 1 << 17  # 0x00020000
    payload = list(struct.pack(">I", fault_bit17) + struct.pack(">I", 0))
    msg = mock_can_message(arbitration_id=ext_id, data=payload)
    
    assert handler.process_message(msg, motor_status) is True
    assert motor_status.fault_code & (1 << 17) != 0, "bit17 (encoder uncalibrated) should be set"
    
    # Test case 3: bit13 (over-temperature fault)
    fault_bit13 = 1 << 13  # 0x00002000
    payload = list(struct.pack(">I", fault_bit13) + struct.pack(">I", 0))
    msg = mock_can_message(arbitration_id=ext_id, data=payload)
    
    assert handler.process_message(msg, motor_status) is True
    assert motor_status.fault_code & (1 << 13) != 0, "bit13 (over-temperature) should be set"
    
    # Test case 4: bit12 (under-voltage fault)
    fault_bit12 = 1 << 12  # 0x00001000
    payload = list(struct.pack(">I", fault_bit12) + struct.pack(">I", 0))
    msg = mock_can_message(arbitration_id=ext_id, data=payload)
    
    assert handler.process_message(msg, motor_status) is True
    assert motor_status.fault_code & (1 << 12) != 0, "bit12 (under-voltage) should be set"
    
    # Test case 5: bit11 (driver chip fault)
    fault_bit11 = 1 << 11  # 0x00000800
    payload = list(struct.pack(">I", fault_bit11) + struct.pack(">I", 0))
    msg = mock_can_message(arbitration_id=ext_id, data=payload)
    
    assert handler.process_message(msg, motor_status) is True
    assert motor_status.fault_code & (1 << 11) != 0, "bit11 (driver chip fault) should be set"
    
    # Test case 6: bit10 (motor over-temperature, 135°C)
    fault_bit10 = 1 << 10  # 0x00000400
    payload = list(struct.pack(">I", fault_bit10) + struct.pack(">I", 0))
    msg = mock_can_message(arbitration_id=ext_id, data=payload)
    
    assert handler.process_message(msg, motor_status) is True
    assert motor_status.fault_code & (1 << 10) != 0, "bit10 (motor over-temp) should be set"
    
    # Test case 7: Multiple faults (bit14 + bit13 + bit12)
    fault_multiple = (1 << 14) | (1 << 13) | (1 << 12)  # 0x00007000
    payload = list(struct.pack(">I", fault_multiple) + struct.pack(">I", 0))
    msg = mock_can_message(arbitration_id=ext_id, data=payload)
    
    assert handler.process_message(msg, motor_status) is True
    assert motor_status.fault_code & (1 << 14) != 0, "bit14 should be set"
    assert motor_status.fault_code & (1 << 13) != 0, "bit13 should be set"
    assert motor_status.fault_code & (1 << 12) != 0, "bit12 should be set"
    assert motor_status.fault_code & (1 << 11) == 0, "bit11 should not be set"
    
    # Test case 8: Warning bit0 (motor over-temperature warning, 125°C)
    warning_bit0 = 1 << 0  # 0x00000001
    payload = list(struct.pack(">I", 0) + struct.pack(">I", warning_bit0))
    msg = mock_can_message(arbitration_id=ext_id, data=payload)
    
    assert handler.process_message(msg, motor_status) is True
    assert motor_status.warning_code & (1 << 0) != 0, "warning bit0 (motor over-temp warning) should be set"
    assert motor_status.fault_code == 0, "fault_code should be 0 when only warning is set"


def test_parameter_response_spec_example_loc_kp(handler, motor_status, parameter_data, mock_can_message):
    """
    Test parameter read response with RS02 specification example.
    
    Per RS02 specification example:
    Read loc_kp (0x701E):
    Command:  1E 70 00 00 00 00 00 00
    Response: 1E 70 00 00 00 00 F0 41
    
    Where 0x41F00000 (little-endian: 00 00 F0 41) = 30.0 in IEEE-754 float32
    """
    index = ParameterIndex.LOC_KP  # 0x701E
    expected_value = 30.0
    
    # Build response frame according to spec
    data_field = (handler.master_id << 8) | 0x00  # Success flag = 0x00
    ext_id = ((CommunicationType.GET_SINGLE_PARAMETER & 0x1F) << 24) | (data_field << 8) | handler.master_id
    
    # Payload: index (little-endian) + 0x0000 + value (little-endian IEEE-754)
    payload = [
        index & 0xFF,           # Byte0: 0x1E
        (index >> 8) & 0xFF,    # Byte1: 0x70
        0x00,                   # Byte2
        0x00,                   # Byte3
        0x00,                   # Byte4: LSB of float
        0x00,                   # Byte5
        0xF0,                   # Byte6
        0x41,                   # Byte7: MSB of float (0x41F00000 = 30.0)
    ]
    msg = mock_can_message(arbitration_id=ext_id, data=payload)

    assert handler.process_message(msg, motor_status, parameter_data) is True
    assert parameter_data.loc_kp == pytest.approx(expected_value, rel=0.0, abs=1e-6)


def test_all_parameter_indices_mapping(handler, motor_status, parameter_data, mock_can_message):
    """Test that all RS02 parameter indices are properly mapped"""
    # Test a sample of each parameter type from RS02 specification table
    test_params = [
        (ParameterIndex.RUN_MODE, 2.0, 'run_mode'),
        (ParameterIndex.IQ_REF, 5.0, 'iq_ref'),
        (ParameterIndex.LIMIT_SPD, 30.0, 'limit_spd'),
        (ParameterIndex.SPD_REF, 10.0, 'spd_ref'),
        (ParameterIndex.LIMIT_TORQUE, 10.0, 'limit_torque'),
        (ParameterIndex.CUR_KP, 0.2, 'cur_kp'),
        (ParameterIndex.CUR_KI, 0.015, 'cur_ki'),
        (ParameterIndex.LOC_REF, 3.14, 'loc_ref'),
        (ParameterIndex.LIMIT_SPD_CSP, 20.0, 'limit_spd_csp'),
        (ParameterIndex.LIMIT_CUR, 15.0, 'limit_cur'),
        (ParameterIndex.LOC_KP, 50.0, 'loc_kp'),
        (ParameterIndex.SPD_KP, 20.0, 'spd_kp'),
        (ParameterIndex.SPD_KI, 0.05, 'spd_ki'),
        (ParameterIndex.SPD_FILT_GAIN, 0.2, 'spd_filt_gain'),
        (ParameterIndex.ACC_RAD, 25.0, 'acc_rad'),
        (ParameterIndex.VEL_MAX, 15.0, 'limit_spd_pp'),
        (ParameterIndex.ACC_SET, 12.0, 'acceleration'),
    ]
    
    for param_index, test_value, attr_name in test_params:
        data_field = (handler.master_id << 8) | 0x00
        ext_id = ((CommunicationType.GET_SINGLE_PARAMETER & 0x1F) << 24) | (data_field << 8) | handler.master_id
        
        # Get parameter spec to determine data type
        spec = get_parameter_spec(param_index)
        
        # Encode value based on data type
        if spec and spec.data_type in ('uint8', 'uint16', 'uint32'):
            int_value = int(test_value)
            if spec.data_type == 'uint8':
                value_bytes = [int_value, 0x00, 0x00, 0x00]
            elif spec.data_type == 'uint16':
                value_bytes = list(struct.pack("<H", int_value)) + [0x00, 0x00]
            else:  # uint32
                value_bytes = list(struct.pack("<I", int_value))
        else:
            # Default to float32
            value_bytes = list(struct.pack("<f", test_value))
        
        payload = [
            param_index & 0xFF,
            (param_index >> 8) & 0xFF,
            0x00,
            0x00,
            *value_bytes
        ]
        msg = mock_can_message(arbitration_id=ext_id, data=payload)
        
        # Reset parameter_data
        setattr(parameter_data, attr_name, 0.0)
        
        assert handler.process_message(msg, motor_status, parameter_data) is True
        actual_value = getattr(parameter_data, attr_name)
        assert actual_value == pytest.approx(test_value, rel=0.0, abs=1e-5), \
            f"Parameter {attr_name} (0x{param_index:04X}) mismatch"

