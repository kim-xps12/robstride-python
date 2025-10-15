"""
CAN protocol utilities for RobStride motor control

Provides functions for CAN message construction, parsing, encoding, and decoding.
"""

import struct
from typing import Tuple


def build_extended_can_id(
    comm_type: int,
    data_byte: int,
    master_id: int,
    motor_id: int
) -> int:
    """
    Build 29-bit extended CAN ID for Private protocol
    
    Format: [CommType:8][Data/Error:8][MasterID:8][MotorID:8]
    
    Args:
        comm_type: Communication type (0x00-0x19)
        data_byte: Additional data or error code
        master_id: Master/host CAN ID (default: 0xFD)
        motor_id: Motor CAN ID (0x00-0x7F)
        
    Returns:
        29-bit extended CAN ID
    """
    ext_id = ((comm_type & 0xFF) << 24) | \
             ((data_byte & 0xFF) << 16) | \
             ((master_id & 0xFF) << 8) | \
             (motor_id & 0xFF)
    return ext_id & 0x1FFFFFFF  # Mask to 29 bits


def parse_extended_can_id(ext_id: int) -> Tuple[int, int, int, int]:
    """
    Parse 29-bit extended CAN ID
    
    Args:
        ext_id: 29-bit extended CAN ID
        
    Returns:
        Tuple of (comm_type, data_byte, master_id, motor_id)
    """
    comm_type = (ext_id >> 24) & 0xFF
    data_byte = (ext_id >> 16) & 0xFF
    master_id = (ext_id >> 8) & 0xFF
    motor_id = ext_id & 0xFF
    return comm_type, data_byte, master_id, motor_id


def encode_int16(value: int) -> bytes:
    """
    Encode 16-bit signed integer to little-endian bytes
    
    Args:
        value: Integer value (-32768 to 32767)
        
    Returns:
        2 bytes in little-endian format
    """
    return struct.pack('<h', value)


def decode_int16(data: bytes, offset: int = 0) -> int:
    """
    Decode 16-bit signed integer from little-endian bytes
    
    Args:
        data: Byte array
        offset: Starting offset in data
        
    Returns:
        Decoded integer value
    """
    return struct.unpack_from('<h', data, offset)[0]


def encode_uint16(value: int) -> bytes:
    """
    Encode 16-bit unsigned integer to little-endian bytes
    
    Args:
        value: Integer value (0 to 65535)
        
    Returns:
        2 bytes in little-endian format
    """
    return struct.pack('<H', value)


def decode_uint16(data: bytes, offset: int = 0) -> int:
    """
    Decode 16-bit unsigned integer from little-endian bytes
    
    Args:
        data: Byte array
        offset: Starting offset in data
        
    Returns:
        Decoded integer value
    """
    return struct.unpack_from('<H', data, offset)[0]


def encode_float32(value: float) -> bytes:
    """
    Encode 32-bit float to little-endian bytes
    
    Args:
        value: Float value
        
    Returns:
        4 bytes in little-endian format
    """
    return struct.pack('<f', value)


def decode_float32(data: bytes, offset: int = 0) -> float:
    """
    Decode 32-bit float from little-endian bytes
    
    Args:
        data: Byte array
        offset: Starting offset in data
        
    Returns:
        Decoded float value
    """
    return struct.unpack_from('<f', data, offset)[0]


def encode_uint64(value: int) -> bytes:
    """
    Encode 64-bit unsigned integer to little-endian bytes
    
    Args:
        value: Integer value
        
    Returns:
        8 bytes in little-endian format
    """
    return struct.pack('<Q', value)


def decode_uint64(data: bytes, offset: int = 0) -> int:
    """
    Decode 64-bit unsigned integer from little-endian bytes
    
    Args:
        data: Byte array
        offset: Starting offset in data
        
    Returns:
        Decoded integer value
    """
    return struct.unpack_from('<Q', data, offset)[0]


def scale_value(value: float, min_val: float, max_val: float, bits: int = 16) -> int:
    """
    Scale float value to integer for CAN transmission
    
    Args:
        value: Float value to scale
        min_val: Minimum value in range
        max_val: Maximum value in range
        bits: Number of bits for scaling (default: 16)
        
    Returns:
        Scaled integer value
    """
    # Clamp value to range
    value = max(min_val, min(max_val, value))
    
    # Calculate scaling
    max_int = (1 << bits) - 1  # 2^bits - 1
    scaled = int((value - min_val) / (max_val - min_val) * max_int)
    
    return scaled


def unscale_value(scaled: int, min_val: float, max_val: float, bits: int = 16) -> float:
    """
    Unscale integer to float value from CAN reception
    
    Args:
        scaled: Scaled integer value
        min_val: Minimum value in range
        max_val: Maximum value in range
        bits: Number of bits for scaling (default: 16)
        
    Returns:
        Unscaled float value
    """
    max_int = (1 << bits) - 1  # 2^bits - 1
    value = min_val + (scaled / max_int) * (max_val - min_val)
    return value


def scale_int16(value: float, min_val: float, max_val: float) -> int:
    """
    Scale float to signed 16-bit integer
    
    Args:
        value: Float value
        min_val: Minimum value
        max_val: Maximum value
        
    Returns:
        Signed 16-bit integer
    """
    # Clamp value
    value = max(min_val, min(max_val, value))
    
    # Scale to -32768 to 32767
    scaled = int(((value - min_val) / (max_val - min_val) * 65535) - 32768)
    return max(-32768, min(32767, scaled))


def unscale_int16(scaled: int, min_val: float, max_val: float) -> float:
    """
    Unscale signed 16-bit integer to float
    
    Args:
        scaled: Signed 16-bit integer
        min_val: Minimum value
        max_val: Maximum value
        
    Returns:
        Unscaled float value
    """
    # Unscale from -32768 to 32767
    value = min_val + ((scaled + 32768) / 65535) * (max_val - min_val)
    return value


def scale_uint16(value: float, min_val: float, max_val: float) -> int:
    """
    Scale float to unsigned 16-bit integer
    
    Args:
        value: Float value
        min_val: Minimum value
        max_val: Maximum value
        
    Returns:
        Unsigned 16-bit integer
    """
    # Clamp value
    value = max(min_val, min(max_val, value))
    
    # Scale to 0 to 65535
    scaled = int(((value - min_val) / (max_val - min_val)) * 65535)
    return max(0, min(65535, scaled))


def unscale_uint16(scaled: int, min_val: float, max_val: float) -> float:
    """
    Unscale unsigned 16-bit integer to float
    
    Args:
        scaled: Unsigned 16-bit integer
        min_val: Minimum value
        max_val: Maximum value
        
    Returns:
        Unscaled float value
    """
    # Unscale from 0 to 65535
    value = min_val + (scaled / 65535) * (max_val - min_val)
    return value


# Specific scaling functions for motor control parameters

def encode_torque_8bit(torque: float) -> int:
    """
    Encode torque (-4 to 4 Nm) to 8-bit for ExtID
    
    Args:
        torque: Torque value in Nm
        
    Returns:
        8-bit scaled torque
    """
    return scale_value(torque, -4.0, 4.0, bits=8)


def encode_angle_16bit(angle: float) -> int:
    """
    Encode angle (-12.5 to 12.5 rad) to 16-bit
    
    Args:
        angle: Angle in radians
        
    Returns:
        16-bit scaled angle
    """
    return scale_int16(angle, -12.5, 12.5)


def decode_angle_16bit(scaled: int) -> float:
    """
    Decode 16-bit angle to float (-12.5 to 12.5 rad)
    
    Args:
        scaled: 16-bit scaled angle
        
    Returns:
        Angle in radians
    """
    return unscale_int16(scaled, -12.5, 12.5)


def encode_speed_16bit(speed: float) -> int:
    """
    Encode speed (-30 to 30 rad/s) to 16-bit
    
    Args:
        speed: Speed in rad/s
        
    Returns:
        16-bit scaled speed
    """
    return scale_int16(speed, -30.0, 30.0)


def decode_speed_16bit(scaled: int) -> float:
    """
    Decode 16-bit speed to float (-30 to 30 rad/s)
    
    Args:
        scaled: 16-bit scaled speed
        
    Returns:
        Speed in rad/s
    """
    return unscale_int16(scaled, -30.0, 30.0)


def decode_speed_feedback_16bit(scaled: int) -> float:
    """
    Decode 16-bit speed feedback to float (-44 to 44 rad/s)
    
    Args:
        scaled: 16-bit scaled speed
        
    Returns:
        Speed in rad/s
    """
    return unscale_int16(scaled, -44.0, 44.0)


def encode_kp_16bit(kp: float) -> int:
    """
    Encode Kp gain (0 to 500) to 16-bit
    
    Args:
        kp: Kp gain value
        
    Returns:
        16-bit scaled Kp
    """
    return scale_uint16(kp, 0.0, 500.0)


def decode_kp_16bit(scaled: int) -> float:
    """
    Decode 16-bit Kp to float (0 to 500)
    
    Args:
        scaled: 16-bit scaled Kp
        
    Returns:
        Kp gain value
    """
    return unscale_uint16(scaled, 0.0, 500.0)


def encode_kd_16bit(kd: float) -> int:
    """
    Encode Kd gain (0 to 5) to 16-bit
    
    Args:
        kd: Kd gain value
        
    Returns:
        16-bit scaled Kd
    """
    return scale_uint16(kd, 0.0, 5.0)


def decode_kd_16bit(scaled: int) -> float:
    """
    Decode 16-bit Kd to float (0 to 5)
    
    Args:
        scaled: 16-bit scaled Kd
        
    Returns:
        Kd gain value
    """
    return unscale_uint16(scaled, 0.0, 5.0)


def decode_torque_16bit(scaled: int) -> float:
    """
    Decode 16-bit torque feedback to float (-17 to 17 Nm)
    
    Args:
        scaled: 16-bit scaled torque
        
    Returns:
        Torque in Nm
    """
    return unscale_int16(scaled, -17.0, 17.0)


def decode_temperature_16bit(scaled: int) -> float:
    """
    Decode 16-bit temperature to float (0.1°C resolution)
    
    Args:
        scaled: 16-bit temperature value
        
    Returns:
        Temperature in °C
    """
    return scaled * 0.1
