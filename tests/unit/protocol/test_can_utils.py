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
    encode_speed_16bit,
    decode_speed_16bit,
    encode_torque_16bit,
    decode_torque_16bit,
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
        data_field = 0x1234
        master_id = 0xFD
        motor_id = 0x10
        
        ext_id = build_extended_can_id(comm_type, data_field, master_id, motor_id)
        parsed_type, parsed_data, parsed_master, parsed_motor = parse_extended_can_id(ext_id)
        
        assert parsed_type == comm_type
        assert parsed_motor == motor_id
    
    @pytest.mark.parametrize("comm_type,data,master,motor", [
        (0x00, 0x0000, 0xFD, 0x01),
        (0x1F, 0xFFFF, 0xFF, 0x7F),
        (0x12, 0x7016, 0xFD, 0x05),
    ])
    def test_build_parse_parametrized(self, comm_type, data, master, motor):
        """TC-U-CU-003: build and parse work for various inputs"""
        ext_id = build_extended_can_id(comm_type, data, master, motor)
        p_type, p_data, p_master, p_motor = parse_extended_can_id(ext_id)
        
        assert p_type == comm_type
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
            assert abs(decoded - value) < 1e-6
    
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
        """TC-U-PV-001: angle encoding maps full range correctly"""
        # Min angle
        min_val = encode_angle_16bit(-12.57)
        assert min_val == 0
        
        # Max angle
        max_val = encode_angle_16bit(12.57)
        assert max_val == 65535
        
        # Zero (should be around middle)
        zero_val = encode_angle_16bit(0.0)
        assert 32000 < zero_val < 33000  # Approximately 32768
    
    def test_decode_angle_16bit_range(self):
        """TC-U-PV-002: angle decoding maps full range correctly"""
        # Min
        min_angle = decode_angle_16bit(0)
        assert abs(min_angle - (-12.57)) < 0.01
        
        # Max
        max_angle = decode_angle_16bit(65535)
        assert abs(max_angle - 12.57) < 0.01
        
        # Middle
        mid_angle = decode_angle_16bit(32768)
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
            decoded = decode_torque_16bit(encoded)
            
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
        """TC-U-PV-006: Encoding clamps values to valid range"""
        # Angle beyond range
        over_angle = encode_angle_16bit(20.0)  # > 12.57
        assert over_angle == 65535  # Should clamp to max
        
        under_angle = encode_angle_16bit(-20.0)  # < -12.57
        assert under_angle == 0  # Should clamp to min
        
        # Speed beyond range
        over_speed = encode_speed_16bit(50.0)  # > 44
        assert over_speed == 65535
