"""
Unit tests for CAN utility functions (encode/decode)
"""

import pytest
import struct

from robstride.protocol.can_utils import (
    build_extended_can_id,
    parse_extended_can_id,
    encode_int16,
    decode_int16,
    encode_uint16,
    decode_uint16,
    encode_float32,
    decode_float32,
    encode_angle_16bit,
    decode_angle_16bit,
    encode_angle_auto_report_16bit,
    decode_angle_auto_report_16bit,
    encode_speed_16bit,
    decode_speed_16bit,
    encode_torque_16bit,
    decode_torque_16bit_motion_control,
    encode_kp_16bit,
    decode_kp_16bit,
    encode_kd_16bit,
    decode_kd_16bit,
)


@pytest.mark.unit
class TestExtendedCANID:
    """Tests for Extended CAN ID building and parsing"""
    
    def test_build_extended_id_basic(self):
        """TC-U-CU-001: build_extended_can_id creates correct 29-bit ID"""
        # Type=0x01, Data=0x0000, Master=0xFD, Motor=0x05
        ext_id = build_extended_can_id(0x01, 0x0000, 0xFD, 0x05)
        
        # Verify ID is valid 29-bit value
        assert isinstance(ext_id, int)
        assert ext_id >= 0
        assert ext_id < (1 << 29)  # 29-bit max
    
    def test_parse_extended_id_roundtrip(self):
        """TC-U-CU-002: parse_extended_can_id reverses build correctly"""
        comm_type = 0x12
        data_byte = 0x34  # 8-bit data
        master_id = 0xFD
        motor_id = 0x10
        
        ext_id = build_extended_can_id(comm_type, data_byte, master_id, motor_id)
        parsed_type, parsed_data, parsed_master, parsed_motor = parse_extended_can_id(ext_id)
        
        assert parsed_type == comm_type
        # data_field will be 16-bit: [data_byte:8][master_id:8]
        expected_data_field = ((master_id & 0xFF) << 8) | (data_byte & 0xFF)
        assert parsed_data == expected_data_field
        assert parsed_master == master_id
        assert parsed_motor == motor_id
        assert (parsed_data & 0xFF) == data_byte
    
    @pytest.mark.parametrize("comm_type,data,master,motor,expected_data", [
        (0x00, 0x00, 0xFD, 0x01, 0xFD00),      # Master in upper byte, data byte 0x00
        (0x12, 0x34, 0x7E, 0x05, 0x7E34),      # Master embedded with non-zero data byte
        (0x1F, 0xFFFF, 0xFF, 0x7F, 0xFFFF),    # Raw 16-bit data (e.g., torque/status)
    ])
    def test_build_parse_parametrized(self, comm_type, data, master, motor, expected_data):
        """TC-U-CU-003: build and parse work for various inputs"""
        ext_id = build_extended_can_id(comm_type, data, master, motor)
        p_type, p_data, p_master, p_motor = parse_extended_can_id(ext_id)
        
        assert p_type == comm_type
        assert p_data == expected_data
        if data <= 0xFF:
            assert p_master == master
        else:
            # When the 16-bit data field is fully consumed (e.g. motion control torque),
            # master ID is not embedded per RS02 specification.
            assert p_master == ((expected_data >> 8) & 0xFF)
        assert p_motor == motor


@pytest.mark.unit
class TestIntegerEncoding:
    """Tests for integer encode/decode functions"""
    
    def test_encode_decode_int16(self):
        """TC-U-IE-001: int16 encoding/decoding roundtrip"""
        test_values = [-32768, -1000, 0, 1000, 32767]
        
        for value in test_values:
            encoded = encode_int16(value)
            assert len(encoded) == 2
            
            decoded = decode_int16(encoded, 0)
            assert decoded == value
    
    def test_encode_decode_uint16(self):
        """TC-U-IE-002: uint16 encoding/decoding roundtrip"""
        test_values = [0, 1000, 32768, 65535]
        
        for value in test_values:
            encoded = encode_uint16(value)
            assert len(encoded) == 2
            
            decoded = decode_uint16(encoded, 0)
            assert decoded == value
    
    def test_encode_decode_uint16_with_offset(self):
        """TC-U-IE-003: uint16 decode with offset works correctly"""
        data = bytes([0x00, 0x12, 0x34, 0x56, 0x78])
        
        # Decode from offset 1
        value = decode_uint16(data, 1)
        assert value == 0x1234
        
        # Decode from offset 3
        value = decode_uint16(data, 3)
        assert value == 0x5678


@pytest.mark.unit
class TestFloatEncoding:
    """Tests for float encode/decode functions"""
    
    def test_encode_decode_float32(self):
        """TC-U-FE-001: float32 encoding/decoding roundtrip"""
        test_values = [-123.456, 0.0, 1.57, 100.0, -17.5]
        
        for value in test_values:
            encoded = encode_float32(value)
            assert len(encoded) == 4
            
            decoded = decode_float32(encoded, 0)
            # float32 has precision ~7 decimal digits, tolerance 1e-5
            assert abs(decoded - value) < 1e-5
    
    def test_encode_float32_byte_order(self):
        """TC-U-FE-002: float32 uses little-endian byte order"""
        value = 1.57
        encoded = encode_float32(value)
        
        # Verify little-endian
        expected = struct.pack('<f', value)
        assert encoded == expected


@pytest.mark.unit
class TestPhysicalValueEncoding:
    """Tests for physical value (angle, speed, torque) encoding"""
    
    def test_encode_angle_16bit_range(self):
        """TC-U-PV-001: angle encoding maps full range correctly (unsigned 0-65535)"""
        # Per RS02 specification (corrected 2025-10-16):
        # Type 0x01/0x02: "−4π〜4π" ≈ ±12.57 rad
        # Type 0x18: "−12.57f〜12.57f" (f=float, ±12.57 rad)
        # All use the same range: [0〜65535] 対応（±12.57 rad）
        
        # Min angle -> 0
        min_val = encode_angle_16bit(-12.57)
        assert min_val == 0
        
        # Max angle -> 65535
        max_val = encode_angle_16bit(12.57)
        assert max_val == 65535
        
        # Zero (should be around 32767 for unsigned encoding)
        zero_val = encode_angle_16bit(0.0)
        assert 32700 < zero_val < 32800  # Approximately 32767
    
    def test_decode_angle_16bit_range(self):
        """TC-U-PV-002: angle decoding maps full range correctly (unsigned 0-65535)"""
        # Per RS02 specification (corrected 2025-10-16):
        # All protocol types use unified range: [0〜65535] 対応（±12.57 rad）
        
        # Min (0 -> -12.57)
        min_angle = decode_angle_16bit(0)
        assert abs(min_angle - (-12.57)) < 0.01
        
        # Max (65535 -> 12.57)
        max_angle = decode_angle_16bit(65535)
        assert abs(max_angle - 12.57) < 0.01
        
        # Mid (32767 -> 0.0)
        mid_angle = decode_angle_16bit(32767)
        assert abs(mid_angle - 0.0) < 0.01
    
    def test_encode_decode_speed_16bit(self):
        """TC-U-PV-003: speed encoding/decoding roundtrip"""
        test_speeds = [-44.0, -20.0, 0.0, 20.0, 44.0]
        
        for speed in test_speeds:
            encoded = encode_speed_16bit(speed)
            decoded = decode_speed_16bit(encoded)
            
            # Allow small error due to quantization
            assert abs(decoded - speed) < 0.1
    
    def test_encode_decode_torque_16bit(self):
        """TC-U-PV-004: torque encoding/decoding roundtrip"""
        test_torques = [-17.0, -10.0, 0.0, 10.0, 17.0]
        
        for torque in test_torques:
            encoded = encode_torque_16bit(torque)
            decoded = decode_torque_16bit_motion_control(encoded)
            
            assert abs(decoded - torque) < 0.1
    
    def test_encode_kp_kd_16bit(self):
        """TC-U-PV-005: Kp/Kd encoding in valid range"""
        # Kp: 0-500
        kp_values = [0.0, 50.0, 250.0, 500.0]
        for kp in kp_values:
            encoded = encode_kp_16bit(kp)
            decoded = decode_kp_16bit(encoded)
            assert abs(decoded - kp) < 1.0
        
        # Kd: 0-5
        kd_values = [0.0, 1.0, 2.5, 5.0]
        for kd in kd_values:
            encoded = encode_kd_16bit(kd)
            decoded = decode_kd_16bit(encoded)
            assert abs(decoded - kd) < 0.01
    
    def test_encode_clamps_out_of_range(self):
        """TC-U-PV-006: Encoding clamps values to valid range (unsigned 0-65535)"""
        # Per specification: all encodings use unsigned (0-65535)
        
        # Angle beyond range (should clamp)
        over_angle = encode_angle_16bit(20.0)  # > 12.57
        assert over_angle == 65535  # Should clamp to max (unsigned)
        
        under_angle = encode_angle_16bit(-20.0)  # < -12.57
        assert under_angle == 0  # Should clamp to min (unsigned)
        
        # Speed beyond range
        over_speed = encode_speed_16bit(50.0)  # > 44
        assert over_speed == 65535  # Should clamp to max (unsigned)


@pytest.mark.unit
class TestAngleRangeVariations:
    """Tests for angle encoding/decoding - unified range across all protocols"""
    
    def test_angle_unified_range_all_types(self):
        """TC-U-AR-001: All protocol types use the same angle range (±12.57 rad)"""
        # Per corrected RS02 specification (2025-10-16):
        # - Type 0x01/0x02: "−4π〜4π" ≈ ±12.566 rad
        # - Type 0x18: "−12.57f〜12.57f" where f denotes float type
        # - MIT Protocol: "−12.57rad〜+12.57rad"
        # All are equivalent: ±12.57 rad
        test_angles = [-12.57, -6.28, 0.0, 6.28, 12.57]
        
        for angle in test_angles:
            encoded = encode_angle_16bit(angle)
            decoded = decode_angle_16bit(encoded)
            assert abs(decoded - angle) < 0.01
    
    def test_angle_auto_report_deprecated_compatibility(self):
        """TC-U-AR-002: Deprecated auto_report functions use same range"""
        # The auto_report functions are deprecated but kept for backward compatibility
        # They should produce the same results as the standard functions
        test_angles = [-12.57, -6.0, 0.0, 6.0, 12.57]
        
        for angle in test_angles:
            std_encoded = encode_angle_16bit(angle)
            auto_encoded = encode_angle_auto_report_16bit(angle)
            assert std_encoded == auto_encoded, f"Angle {angle}: standard and auto_report encoding differ"
            
            std_decoded = decode_angle_16bit(std_encoded)
            auto_decoded = decode_angle_auto_report_16bit(auto_encoded)
            assert abs(std_decoded - auto_decoded) < 0.01
    
    def test_angle_range_boundaries(self):
        """TC-U-AR-003: Test boundary values for angle range"""
        # All protocols use ±12.57 rad
        assert encode_angle_16bit(-12.57) == 0
        assert encode_angle_16bit(0.0) == pytest.approx(32767, abs=100)
        assert encode_angle_16bit(12.57) == 65535
    
    def test_angle_documentation_consistency(self):
        """TC-U-AR-004: Verify angle functions match corrected specification"""
        # After RS02 spec correction, all types use ±12.57 rad
        encoded = encode_angle_16bit(1.0)
        decoded = decode_angle_16bit(encoded)
        assert abs(decoded - 1.0) < 0.01
        
        # Deprecated function should produce same result
        encoded_auto = encode_angle_auto_report_16bit(1.0)
        decoded_auto = decode_angle_auto_report_16bit(encoded_auto)
        assert abs(decoded_auto - 1.0) < 0.01
        assert encoded == encoded_auto


@pytest.mark.unit
class TestAutoReportBoundaryValues:
    """Tests for Type 0x18 auto-report specific encoding/decoding boundary values"""
    
    def test_auto_report_speed_boundary_values(self):
        """TC-U-ARB-001: Type 0x18 speed range (±33 rad/s) boundary value encoding"""
        from robstride.protocol.can_utils import (
            encode_speed_auto_report_16bit,
            decode_speed_auto_report_16bit
        )
        
        # Maximum speed: +33 rad/s
        encoded_max = encode_speed_auto_report_16bit(33.0)
        decoded_max = decode_speed_auto_report_16bit(encoded_max)
        assert decoded_max == pytest.approx(33.0, abs=0.1), "Max speed should be ~33.0 rad/s"
        assert encoded_max == 65535, "Max speed should encode to 65535 (unsigned max)"
        
        # Minimum speed: -33 rad/s
        encoded_min = encode_speed_auto_report_16bit(-33.0)
        decoded_min = decode_speed_auto_report_16bit(encoded_min)
        assert decoded_min == pytest.approx(-33.0, abs=0.1), "Min speed should be ~-33.0 rad/s"
        assert encoded_min == 0, "Min speed should encode to 0 (unsigned min)"
        
        # Zero speed
        encoded_zero = encode_speed_auto_report_16bit(0.0)
        decoded_zero = decode_speed_auto_report_16bit(encoded_zero)
        assert decoded_zero == pytest.approx(0.0, abs=0.1), "Zero speed should be ~0.0 rad/s"
        assert encoded_zero == pytest.approx(32767, abs=100), "Zero speed should encode to ~32767 (mid-point)"
        
        # Intermediate values
        test_speeds = [-20.0, -10.0, 5.0, 15.0, 25.0]
        for speed in test_speeds:
            encoded = encode_speed_auto_report_16bit(speed)
            decoded = decode_speed_auto_report_16bit(encoded)
            assert decoded == pytest.approx(speed, abs=0.2), f"Speed {speed} roundtrip failed"
    
    def test_auto_report_torque_boundary_values(self):
        """TC-U-ARB-002: Type 0x18 torque range (±14 Nm) boundary value encoding"""
        from robstride.protocol.can_utils import (
            encode_torque_auto_report_16bit,
            decode_torque_auto_report_16bit
        )
        
        # Maximum torque: +14 Nm
        encoded_max = encode_torque_auto_report_16bit(14.0)
        decoded_max = decode_torque_auto_report_16bit(encoded_max)
        assert decoded_max == pytest.approx(14.0, abs=0.1), "Max torque should be ~14.0 Nm"
        assert encoded_max == 65535, "Max torque should encode to 65535 (unsigned max)"
        
        # Minimum torque: -14 Nm
        encoded_min = encode_torque_auto_report_16bit(-14.0)
        decoded_min = decode_torque_auto_report_16bit(encoded_min)
        assert decoded_min == pytest.approx(-14.0, abs=0.1), "Min torque should be ~-14.0 Nm"
        assert encoded_min == 0, "Min torque should encode to 0 (unsigned min)"
        
        # Zero torque
        encoded_zero = encode_torque_auto_report_16bit(0.0)
        decoded_zero = decode_torque_auto_report_16bit(encoded_zero)
        assert decoded_zero == pytest.approx(0.0, abs=0.1), "Zero torque should be ~0.0 Nm"
        assert encoded_zero == pytest.approx(32767, abs=100), "Zero torque should encode to ~32767 (mid-point)"
        
        # Intermediate values
        test_torques = [-10.0, -5.0, 2.5, 7.0, 12.0]
        for torque in test_torques:
            encoded = encode_torque_auto_report_16bit(torque)
            decoded = decode_torque_auto_report_16bit(encoded)
            assert decoded == pytest.approx(torque, abs=0.15), f"Torque {torque} roundtrip failed"
    
    def test_auto_report_clamping_behavior(self):
        """TC-U-ARB-003: Verify clamping behavior for out-of-range values"""
        from robstride.protocol.can_utils import (
            encode_speed_auto_report_16bit,
            decode_speed_auto_report_16bit,
            encode_torque_auto_report_16bit,
            decode_torque_auto_report_16bit
        )
        
        # Speed beyond range (should clamp)
        over_speed = encode_speed_auto_report_16bit(50.0)  # > 33
        assert over_speed == 65535, "Over-range speed should clamp to max"
        decoded_over_speed = decode_speed_auto_report_16bit(over_speed)
        assert decoded_over_speed == pytest.approx(33.0, abs=0.1)
        
        under_speed = encode_speed_auto_report_16bit(-50.0)  # < -33
        assert under_speed == 0, "Under-range speed should clamp to min"
        decoded_under_speed = decode_speed_auto_report_16bit(under_speed)
        assert decoded_under_speed == pytest.approx(-33.0, abs=0.1)
        
        # Torque beyond range (should clamp)
        over_torque = encode_torque_auto_report_16bit(20.0)  # > 14
        assert over_torque == 65535, "Over-range torque should clamp to max"
        decoded_over_torque = decode_torque_auto_report_16bit(over_torque)
        assert decoded_over_torque == pytest.approx(14.0, abs=0.1)
        
        under_torque = encode_torque_auto_report_16bit(-20.0)  # < -14
        assert under_torque == 0, "Under-range torque should clamp to min"
        decoded_under_torque = decode_torque_auto_report_16bit(under_torque)
        assert decoded_under_torque == pytest.approx(-14.0, abs=0.1)


