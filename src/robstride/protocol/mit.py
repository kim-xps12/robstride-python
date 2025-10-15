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
        p_int = self._float_to_uint(cmd.position, -12.5, 12.5, 16)
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
        Send MIT position control command
        
        Args:
            position: Target position [rad]
            speed: Target speed [rad/s]
            
        Returns:
            True if sent successfully
        """
        # CAN ID with command type
        can_id = (1 << 8) | self.motor_id
        
        # Pack position and speed as float32
        data = struct.pack('<ff', position, speed)
        
        return self._send_message(can_id, data)
    
    def send_speed_control(self, speed: float, current_limit: float) -> bool:
        """
        Send MIT speed control command
        
        Args:
            speed: Target speed [rad/s]
            current_limit: Current limit [A]
            
        Returns:
            True if sent successfully
        """
        # CAN ID with command type
        can_id = (2 << 8) | self.motor_id
        
        # Pack speed and current limit as float32
        data = struct.pack('<ff', speed, current_limit)
        
        return self._send_message(can_id, data)
    
    def send_clear_error(self, clear: bool = True) -> bool:
        """
        Send MIT clear/check error command
        
        Args:
            clear: True to clear error, False to check
            
        Returns:
            True if sent successfully
        """
        cmd = 0x01 if clear else 0x00
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
        Process received MIT feedback message
        
        Args:
            msg: Received CAN message
            status: MotorStatus object to update
            
        Returns:
            True if message was processed
        """
        # MIT feedback uses standard 11-bit ID
        if msg.is_extended_id:
            return False
        
        # Check if message is from this motor
        if msg.arbitration_id != self.motor_id:
            return False
        
        try:
            data = msg.data
            
            # Unpack MIT feedback (similar to composite control format)
            p_int = (data[0] << 8) | data[1]
            v_int = (data[2] << 4) | ((data[3] >> 4) & 0x0F)
            t_int = ((data[6] & 0x0F) << 8) | data[7]
            
            # Convert to float
            status.angle = self._uint_to_float(p_int, -12.5, 12.5, 16)
            status.speed = self._uint_to_float(v_int, -44.0, 44.0, 12)
            status.torque = self._uint_to_float(t_int, -17.0, 17.0, 12)
            
            # Temperature and pattern (if available in data)
            # Note: MIT protocol feedback format may vary
            
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
