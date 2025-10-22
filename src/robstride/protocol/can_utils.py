"""
CAN protocol utilities for RobStride motor control

Provides functions for CAN message construction, parsing, encoding, and decoding.
"""

import struct
from typing import Tuple


def build_extended_can_id(
    comm_type: int,
    data_field: int,
    master_id: int,
    motor_id: int
) -> int:
    """
    Build 29-bit extended CAN ID for Private protocol
    
    Based on RS02 specification (rs02_ja.md):
    Bit layout: [CommType:5][DataField:16][MotorID:8]
    - Bit 28-24: Communication Type
    - Bit 23-8: Data Field (usage depends on comm_type)
      - Bit 15-8: Host/Master CAN ID for command frames that do not need the
        entire 16-bit range (e.g., enable/disable, zeroing, etc.)
      - Bit 7-0 : Command-specific data byte
      - Commands that require the full 16-bit range (e.g., motion-control torque
        or error/status flags) place their value directly in Bit 23-8 without
        embedding the master ID.
    
    Args:
        comm_type: Communication type (0x00-0x19)
        data_field: 16-bit data field or 8-bit data
                   If <= 0xFF: treated as upper byte, combined with master_id
                   If > 0xFF: used as-is (16-bit)
        master_id: Master/host CAN ID (0x00-0xFF, default: 0xFD)
        motor_id: Motor CAN ID (0x00-0x7F)
        
    Returns:
        29-bit extended CAN ID
    """
    # Build 16-bit data field: [upper_byte:8][master_id:8]
    if data_field > 0xFF:
        # data_field consumes entire 16-bit region (e.g., torque, status flags)
        data_16bit = data_field & 0xFFFF
    else:
        # Embed host/master ID in Bit 15-8 as mandated by RS02 spec
        data_16bit = ((master_id & 0xFF) << 8) | (data_field & 0xFF)
    
    # Build 29-bit Extended CAN ID
    ext_id = ((comm_type & 0x1F) << 24) | \
             ((data_16bit & 0xFFFF) << 8) | \
             (motor_id & 0xFF)
    
    return ext_id & 0x1FFFFFFF  # Mask to 29 bits


def parse_extended_can_id(ext_id: int) -> Tuple[int, int, int, int]:
    """
    Parse 29-bit extended CAN ID
    
    Based on RS02 specification:
    Bit layout: [CommType:5][DataField:16][MotorID:8]
    - Bit 28-24: Communication Type
    - Bit 23-16: Upper data byte
    - Bit 15-8: Master CAN_ID (for most types)
    - Bit 7-0: Motor CAN_ID
    
    Args:
        ext_id: 29-bit extended CAN ID
        
    Returns:
        Tuple of (comm_type, data_field, master_id, motor_id)
        - data_field: Full 16-bit field as stored in the identifier
        - master_id: Extracted from bits 15-8 (upper byte of the 16-bit field)
    """
    comm_type = (ext_id >> 24) & 0x1F
    data_16bit = (ext_id >> 8) & 0xFFFF
    motor_id = ext_id & 0xFF
    
    # By default treat Bit 15-8 as host/master ID when present
    master_id = (data_16bit >> 8) & 0xFF
    data_field = data_16bit
    
    return comm_type, data_field, master_id, motor_id


def encode_int16(value: int) -> bytes:
    """
    Encode 16-bit signed integer to big-endian bytes (high byte first)
    
    Args:
        value: Integer value (-32768 to 32767)
        
    Returns:
        2 bytes in big-endian format
    """
    return struct.pack('>h', value)


def decode_int16(data: bytes, offset: int = 0) -> int:
    """
    Decode 16-bit signed integer from big-endian bytes
    
    Args:
        data: Byte array
        offset: Starting offset in data
        
    Returns:
        Decoded integer value
    """
    return struct.unpack_from('>h', data, offset)[0]


def encode_uint16(value: int) -> bytes:
    """
    Encode 16-bit unsigned integer to big-endian bytes (high byte first)
    
    Used for Motion Control messages (Type 0x01, 0x02) where spec says:
    「高バイトが前、低バイトが後に配置される」
    
    Args:
        value: Integer value (0 to 65535)
        
    Returns:
        2 bytes in big-endian format
    """
    return struct.pack('>H', value)


def encode_uint16_le(value: int) -> bytes:
    """
    Encode 16-bit unsigned integer to little-endian bytes (low byte first)
    
    Used for Parameter Read/Write messages (Type 0x11, 0x12) where spec says:
    「低バイトが前、高バイトが後」
    
    Args:
        value: Integer value (0 to 65535)
        
    Returns:
        2 bytes in little-endian format
    """
    return struct.pack('<H', value)


def decode_uint16(data: bytes, offset: int = 0) -> int:
    """
    Decode 16-bit unsigned integer from big-endian bytes
    
    Used for Motion Control feedback (Type 0x02) where spec says:
    「高バイトが前、低バイトが後に配置」
    
    Args:
        data: Byte array
        offset: Starting offset in data
        
    Returns:
        Decoded integer value
    """
    return struct.unpack_from('>H', data, offset)[0]


def decode_uint16_le(data: bytes, offset: int = 0) -> int:
    """
    Decode 16-bit unsigned integer from little-endian bytes
    
    Used for Parameter Read response (Type 0x11) where spec says:
    「低バイトが前、高バイトが後」
    
    Args:
        data: Byte array
        offset: Starting offset in data
        
    Returns:
        Decoded integer value
    """
    return struct.unpack_from('<H', data, offset)[0]


def encode_float32(value: float) -> bytes:
    """
    Encode 32-bit float to little-endian bytes (low byte first)
    
    Per specification: Parameters and MIT Protocol use little-endian.
    「低バイトが前、高バイトが後」for Parameter R/W (Type 0x11/0x12)
    Little-endian for MIT Position/Speed Control
    
    Args:
        value: Float value
        
    Returns:
        4 bytes in little-endian format
    """
    return struct.pack('<f', value)


def encode_float32_le(value: float) -> bytes:
    """
    Encode 32-bit float to little-endian bytes (low byte first)
    
    Alias for encode_float32() for backward compatibility.
    
    Args:
        value: Float value
        
    Returns:
        4 bytes in little-endian format
    """
    return encode_float32(value)


def decode_float32(data: bytes, offset: int = 0) -> float:
    """
    Decode 32-bit float from little-endian bytes
    
    Per specification: Parameters and MIT Protocol use little-endian.
    
    Args:
        data: Byte array
        offset: Starting offset in data
        
    Returns:
        Decoded float value
    """
    return struct.unpack_from('<f', data, offset)[0]


def decode_float32_le(data: bytes, offset: int = 0) -> float:
    """
    Decode 32-bit float from little-endian bytes
    
    Alias for decode_float32() for backward compatibility.
    
    Args:
        data: Byte array
        offset: Starting offset in data
        
    Returns:
        Decoded float value
    """
    return decode_float32(data, offset)


def encode_uint64(value: int) -> bytes:
    """
    Encode 64-bit unsigned integer to big-endian bytes (high byte first)
    
    Args:
        value: Integer value
        
    Returns:
        8 bytes in big-endian format
    """
    return struct.pack('>Q', value)


def decode_uint64(data: bytes, offset: int = 0) -> int:
    """
    Decode 64-bit unsigned integer from big-endian bytes
    
    Args:
        data: Byte array
        offset: Starting offset in data
        
    Returns:
        Decoded integer value
    """
    return struct.unpack_from('>Q', data, offset)[0]


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

def encode_torque_16bit(torque: float) -> int:
    """
    Encode torque (-17 to 17 Nm) to 16-bit unsigned integer (0-65535)
    
    Per specification: "トルク（0〜65535）対応（−17Nm〜17Nm）"
    
    Args:
        torque: Torque value in Nm (-17 to 17)
        
    Returns:
        16-bit unsigned integer (0-65535)
    """
    return scale_uint16(torque, -17.0, 17.0)


def decode_torque_16bit_motion_control(scaled: int) -> float:
    """
    Decode 16-bit torque from motion control command (0-65535 to -17~17 Nm)
    
    Args:
        scaled: 16-bit unsigned integer (0-65535)
        
    Returns:
        Torque in Nm (-17 to 17)
    """
    return unscale_uint16(scaled, -17.0, 17.0)


def encode_torque_8bit(torque: float) -> int:
    """
    Encode torque (-17 to 17 Nm) to 8-bit for ExtID (legacy compatibility)
    
    Args:
        torque: Torque value in Nm
        
    Returns:
        8-bit scaled torque
    """
    return scale_value(torque, -17.0, 17.0, bits=8)


def encode_angle_16bit(angle: float) -> int:
    """
    Encode angle (-12.57 to 12.57 rad) to 16-bit unsigned integer (0-65535).
    
    Usage:
        - Private Protocol Type 0x01 (Motion Control) - command payload
        - Private Protocol Type 0x02 (Motor Feedback) - status feedback
        - Private Protocol Type 0x18 (Auto Report) - auto report feedback
        - MIT Protocol composite control and feedback
    
    Per RS02 specification (corrected 2025-10-16):
        - Type 0x01: "目標角度 [0〜65535] 対応（−4π〜4π）"
        - Type 0x02: "現在角度 [0〜65535] 対応（−4π〜4π）"
        - Type 0x18: "現在角度 [0〜65535] 対応（−12.57f〜12.57f）" where f denotes float
        - MIT Protocol: "−12.57rad〜+12.57rad"
        
    All ranges are equivalent: ±4π ≈ ±12.566 rad ≈ ±12.57 rad
    
    Args:
        angle: Angle in radians (-12.57 to 12.57)
        
    Returns:
        16-bit unsigned integer (0-65535)
    """
    return scale_uint16(angle, -12.57, 12.57)


def decode_angle_16bit(scaled: int) -> float:
    """
    Decode 16-bit unsigned integer to angle (-12.57 to 12.57 rad).
    
    Usage:
        - Private Protocol Type 0x02 (Motor Feedback)
        - Private Protocol Type 0x18 (Auto Report)
        - MIT Protocol feedback
    
    See encode_angle_16bit for specification details and range rationale.
    """
    return unscale_uint16(scaled, -12.57, 12.57)


# Legacy functions - deprecated, kept for backward compatibility
# Type 0x18 uses the same range as Type 0x02, no separate encoding needed
def encode_angle_auto_report_16bit(angle: float) -> int:
    """
    DEPRECATED: Use encode_angle_16bit() instead.
    
    Per corrected RS02 specification (2025-10-16), Type 0x18 uses the same
    angle range as Type 0x02: "−12.57f〜12.57f" (±12.57 rad), not "−12.57π〜12.57π".
    
    This function is kept for backward compatibility only.
    """
    return encode_angle_16bit(angle)


def decode_angle_auto_report_16bit(scaled: int) -> float:
    """
    DEPRECATED: Use decode_angle_16bit() instead.
    
    Per corrected RS02 specification (2025-10-16), Type 0x18 uses the same
    angle range as Type 0x02: "−12.57f〜12.57f" (±12.57 rad), not "−12.57π〜12.57π".
    
    This function is kept for backward compatibility only.
    """
    return decode_angle_16bit(scaled)


def encode_speed_16bit(speed: float) -> int:
    """
    Encode speed (-44 to 44 rad/s) to 16-bit unsigned integer (0-65535)
    
    Per specification: "[0〜65535] 対応（−44rad/s〜44rad/s）"
    
    Args:
        speed: Speed in rad/s (-44 to 44)
        
    Returns:
        16-bit unsigned integer (0-65535)
    """
    return scale_uint16(speed, -44.0, 44.0)


def decode_speed_16bit(scaled: int) -> float:
    """
    Decode 16-bit unsigned integer to speed (-44 to 44 rad/s)
    
    Per specification: "[0〜65535] 対応（−44rad/s〜44rad/s）"
    
    Args:
        scaled: 16-bit unsigned integer (0-65535)
        
    Returns:
        Speed in rad/s (-44 to 44)
    """
    return unscale_uint16(scaled, -44.0, 44.0)


def decode_speed_feedback_16bit(scaled: int) -> float:
    """
    Decode 16-bit unsigned integer speed feedback to float (-44 to 44 rad/s)
    
    Per specification: "[0〜65535] 対応（−44rad/s〜44rad/s）"
    
    Args:
        scaled: 16-bit unsigned integer (0-65535)
        
    Returns:
        Speed in rad/s (-44 to 44)
    """
    return unscale_uint16(scaled, -44.0, 44.0)


def encode_speed_auto_report_16bit(speed: float) -> int:
    """
    Encode speed (-33 to 33 rad/s) for auto-report responses (Type 0x18)
    
    Per specification: "[0〜65535] 対応（−33rad/s〜33rad/s）"
    """
    return scale_uint16(speed, -33.0, 33.0)


def decode_speed_auto_report_16bit(scaled: int) -> float:
    """
    Decode auto-report speed (-33 to 33 rad/s) from 16-bit unsigned integer
    """
    return unscale_uint16(scaled, -33.0, 33.0)


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
    Decode 16-bit unsigned integer torque feedback to float (-17 to 17 Nm)
    
    Per specification: "[0〜65535] 対応（−17Nm〜17Nm）"
    
    Args:
        scaled: 16-bit unsigned integer (0-65535)
        
    Returns:
        Torque in Nm (-17 to 17)
    """
    return unscale_uint16(scaled, -17.0, 17.0)


def encode_torque_auto_report_16bit(torque: float) -> int:
    """
    Encode torque (-14 to 14 Nm) for auto-report responses (Type 0x18)
    """
    return scale_uint16(torque, -14.0, 14.0)


def decode_torque_auto_report_16bit(scaled: int) -> float:
    """
    Decode auto-report torque (-14 to 14 Nm) from 16-bit unsigned integer
    """
    return unscale_uint16(scaled, -14.0, 14.0)


def decode_temperature_16bit(scaled: int) -> float:
    """
    Decode 16-bit temperature to float (0.1°C resolution)
    
    Motor sends temperature as (°C × 10), so divide by 10 to get actual temperature.
    
    Args:
        scaled: 16-bit temperature value (temperature × 10)
        
    Returns:
        Temperature in °C
    """
    return scaled / 10.0
