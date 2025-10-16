"""
MIT protocol handler for RobStride motor control

Implements MIT Cheetah protocol for simple position/speed/torque control.
"""

import can
import struct
from typing import Optional
from ..models import MotorStatus, MITCommand, MITMotorType
from .can_utils import (
    decode_angle_16bit,
    decode_speed_feedback_16bit,
    decode_torque_16bit,
    decode_temperature_16bit
)


class MITProtocolHandler:
    """Handler for MIT protocol communication"""
    
    def __init__(self, motor_id: int, can_bus: can.Bus):
        """
        Initialize MIT protocol handler
        
        Args:
            motor_id: Motor CAN ID (0x00-0x7F)
            can_bus: python-can Bus instance
        """
        self.motor_id = motor_id
        self.can_bus = can_bus
        self.timeout = 0.1  # 100ms timeout
        
    def _send_message(self, can_id: int, data: bytes, is_extended: bool = False) -> bool:
        """
        Send CAN message
        
        Args:
            can_id: CAN ID (11-bit standard or 29-bit extended)
            data: 8-byte data payload
            is_extended: Use extended ID
            
        Returns:
            True if sent successfully
        """
        try:
            msg = can.Message(
                arbitration_id=can_id,
                data=data,
                is_extended_id=is_extended,
                dlc=8
            )
            self.can_bus.send(msg)
            return True
        except Exception as e:
            print(f"Error sending MIT message: {e}")
            return False
    
    # === Control commands ===
    
    def send_enable(self) -> bool:
        """
        Send MIT enable command
        
        Returns:
            True if sent successfully
        """
        data = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
        return self._send_message(self.motor_id, data)
    
    def send_disable(self) -> bool:
        """
        Send MIT disable command
        
        Returns:
            True if sent successfully
        """
        data = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])
        return self._send_message(self.motor_id, data)
    
    def send_set_zero(self) -> bool:
        """
        Send MIT set zero position command
        
        Returns:
            True if sent successfully
        """
        data = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE])
        return self._send_message(self.motor_id, data)
    
    def send_composite_control(self, cmd: MITCommand) -> bool:
        """
        Send MIT composite control command
        
        Packs position, velocity, Kp, Kd, torque into 8 bytes.
        
        Args:
            cmd: MIT command with control parameters
            
        Returns:
            True if sent successfully
        """
        # Scale parameters to integers
        p_int = self._float_to_uint(cmd.position, -12.57, 12.57, 16)
        v_int = self._float_to_uint(cmd.velocity, -44.0, 44.0, 12)
        kp_int = self._float_to_uint(cmd.kp, 0.0, 500.0, 12)
        kd_int = self._float_to_uint(cmd.kd, 0.0, 5.0, 12)
        t_int = self._float_to_uint(cmd.torque, -17.0, 17.0, 12)
        
        # Pack into 8 bytes
        data = bytearray(8)
        data[0] = (p_int >> 8) & 0xFF
        data[1] = p_int & 0xFF
        data[2] = (v_int >> 4) & 0xFF
        data[3] = ((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F)
        data[4] = kp_int & 0xFF
        data[5] = (kd_int >> 4) & 0xFF
        data[6] = ((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F)
        data[7] = t_int & 0xFF
        
        return self._send_message(self.motor_id, bytes(data))
    
    def send_position_control(self, position: float, speed: float) -> bool:
        """
        Send MIT position control command (Command 10)
        
        Per specification:
        CAN ID: [Mode:3][MotorID:8] = (1 << 8) | motor_id
        Data: [Position:32bit float][Speed:32bit float] (little-endian)
        
        Args:
            position: Target position [rad]
            speed: Target speed [rad/s]
            
        Returns:
            True if sent successfully
        """
        # CAN ID with command type (bits 10:8 = 1, bits 7:0 = motor_id)
        can_id = (1 << 8) | self.motor_id
        
        # Pack position and speed as little-endian float32
        data = struct.pack('<ff', position, speed)
        
        return self._send_message(can_id, data)
    
    def send_speed_control(self, speed: float, current_limit: float) -> bool:
        """
        Send MIT speed control command (Command 11)
        
        Per specification:
        CAN ID: [Mode:3][MotorID:8] = (2 << 8) | motor_id  
        Data: [Speed:32bit float][CurrentLimit:32bit float] (little-endian)
        
        Args:
            speed: Target speed [rad/s]
            current_limit: Current limit [A]
            
        Returns:
            True if sent successfully
        """
        # CAN ID with command type (bits 10:8 = 2, bits 7:0 = motor_id)
        can_id = (2 << 8) | self.motor_id
        
        # Pack speed and current limit as little-endian float32
        data = struct.pack('<ff', speed, current_limit)
        
        return self._send_message(can_id, data)
    
    def send_clear_error(self, clear: bool = True) -> bool:
        """
        Send MIT clear/check error command (Command 5)
        
        Per RS02 specification (rs02_ja.md:468):
        "F_CMD バイトが 0xFF の場合は「現在の異常をクリア」を意味し、
        その他の値は別の状態を示す。"
        
        Args:
            clear: True to clear error (F_CMD=0xFF), False to check (F_CMD=0x00)
            
        Returns:
            True if sent successfully
        """
        cmd = 0xFF if clear else 0x00  # Fixed: 0xFF for clear per RS02 spec
        data = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd, 0xFB])
        return self._send_message(self.motor_id, data)
    
    def send_set_motor_type(self, motor_type: MITMotorType) -> bool:
        """
        Send MIT set motor type command
        
        Args:
            motor_type: Motor control type
            
        Returns:
            True if sent successfully
        """
        data = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, motor_type, 0xFC])
        return self._send_message(self.motor_id, data)
    
    # === Message processing ===
    
    def process_message(self, msg: can.Message, status: MotorStatus) -> bool:
        """
        Process received MIT feedback message (Response Command 1)
        
        Per specification:
        Standard feedback (Response Command 1):
        - Byte0: Motor CAN ID
        - Byte1-2: Target angle [0-65535] → (-12.57 ~ 12.57 rad)
        - Byte3 (high 8) + Byte4 (low 4): Target speed [0-4096] → (-44 ~ 44 rad/s)
        - Byte4 (low 4) + Byte5 (high 8): Target torque [0-4096] → (-17 ~ 17 Nm)
        - Byte6-7: Winding temperature (°C)
        
        Error response (Command 5 response, RS02 rs02_ja.md:468):
        - Byte0: Motor CAN ID
        - Byte1: Error code (fault value)
        - "いずれの値でも、応答の BYTE1 にエラー値が返されます"
        
        Args:
            msg: Received CAN message
            status: MotorStatus object to update
            
        Returns:
            True if message was processed
        """
        # MIT feedback uses standard 11-bit ID
        if msg.is_extended_id:
            return False
        
        # For feedback, check if this is the host ID receiving from motor
        # Motor sends to host ID, not motor ID
        # Accept any message in MIT mode for now
        
        try:
            data = msg.data
            
            # Byte0: Motor CAN ID (verification)
            motor_can_id = data[0]
            
            # Verify this is from the expected motor
            if motor_can_id != self.motor_id:
                # Not from this motor
                return False
            
            # Check if this is an error response (Command 5 response)
            # Error responses have Byte1 as error code.
            # Per RS02 spec: "応答の BYTE1 にエラー値が返されます"
            # 
            # Heuristic detection: Error responses typically have:
            # - Byte1: small error code value (0x00-0xFF)
            # - Byte2-7: mostly zeros or don't form valid sensor data
            # We check if Byte6-7 (temperature) are zero, which is unlikely
            # in normal operation (motor would have some temperature)
            is_likely_error_response = (
                data[6] == 0x00 and data[7] == 0x00 and
                data[2] == 0x00 and data[3] == 0x00 and 
                data[4] == 0x00 and data[5] == 0x00
            )
            
            if is_likely_error_response:
                # Error response: Byte1 contains error code
                error_code = data[1]
                status.error_code = error_code
                # Keep other fields unchanged or set to safe values
                status.pattern = 2  # Motor mode
                return True
            
            # Normal feedback processing
            # Byte1-2: Target angle (16-bit)
            p_int = (data[1] << 8) | data[2]
            
            # Byte3 (high 8) + Byte4 (low 4): Target speed (12-bit)
            v_int = (data[3] << 4) | ((data[4] >> 4) & 0x0F)
            
            # Byte4 (low 4) + Byte5 (high 8): Target torque (12-bit)
            t_int = ((data[4] & 0x0F) << 8) | data[5]
            
            # Byte6-7: Winding temperature (16-bit, unit: °C)
            temp_raw = (data[6] << 8) | data[7]
            
            # Convert to float values
            status.angle = self._uint_to_float(p_int, -12.57, 12.57, 16)
            status.speed = self._uint_to_float(v_int, -44.0, 44.0, 12)
            status.torque = self._uint_to_float(t_int, -17.0, 17.0, 12)
            
            # Temperature is directly in °C (no scaling needed for MIT protocol)
            status.temperature = float(temp_raw)
            
            # MIT protocol doesn't provide pattern/error in standard feedback
            # Set to default values
            status.pattern = 2  # Assume motor mode
            status.error_code = 0
            
            return True
        except Exception as e:
            print(f"Error processing MIT feedback: {e}")
            return False
    
    # === Utility functions ===
    
    def _float_to_uint(self, value: float, min_val: float, max_val: float, bits: int) -> int:
        """
        Convert float to unsigned integer with scaling
        
        Args:
            value: Float value
            min_val: Minimum value
            max_val: Maximum value
            bits: Number of bits
            
        Returns:
            Scaled unsigned integer
        """
        # Clamp value
        value = max(min_val, min(max_val, value))
        
        # Scale to integer
        max_int = (1 << bits) - 1
        scaled = int((value - min_val) / (max_val - min_val) * max_int)
        
        return max(0, min(max_int, scaled))
    
    def _uint_to_float(self, value: int, min_val: float, max_val: float, bits: int) -> float:
        """
        Convert unsigned integer to float with unscaling
        
        Args:
            value: Unsigned integer
            min_val: Minimum value
            max_val: Maximum value
            bits: Number of bits
            
        Returns:
            Unscaled float value
        """
        max_int = (1 << bits) - 1
        return min_val + (value / max_int) * (max_val - min_val)
