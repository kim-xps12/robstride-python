"""
Private protocol handler for RobStride motor control

Implements all communication types (0x00-0x19) for the Private protocol.
"""

import can
import time
import struct
from typing import Optional, Callable
from ..models import CommunicationType, MotorStatus, ParameterData, MotionControlCommand
from .can_utils import (
    build_extended_can_id,
    parse_extended_can_id,
    encode_int16,
    decode_int16,
    encode_uint16,
    decode_uint16,
    encode_float32,
    decode_float32,
    encode_uint64,
    decode_uint64,
    encode_angle_16bit,
    decode_angle_16bit,
    encode_speed_16bit,
    decode_speed_16bit,
    decode_speed_feedback_16bit,
    encode_kp_16bit,
    decode_kp_16bit,
    encode_kd_16bit,
    decode_kd_16bit,
    decode_torque_16bit,
    decode_temperature_16bit,
    encode_torque_8bit
)


class PrivateProtocolHandler:
    """Handler for RobStride Private protocol communication"""
    
    def __init__(self, motor_id: int, can_bus: can.Bus, master_id: int = 0xFD):
        """
        Initialize Private protocol handler
        
        Args:
            motor_id: Motor CAN ID (0x00-0x7F)
            can_bus: python-can Bus instance
            master_id: Master/host ID (default: 0xFD)
        """
        self.motor_id = motor_id
        self.can_bus = can_bus
        self.master_id = master_id
        self.timeout = 0.1  # 100ms timeout for responses
        
    def _send_message(self, comm_type: int, data: bytes, data_byte: int = 0x00) -> bool:
        """
        Send CAN message with Private protocol format
        
        Args:
            comm_type: Communication type
            data: 8-byte data payload
            data_byte: Additional data byte in extended ID
            
        Returns:
            True if sent successfully
        """
        try:
            ext_id = build_extended_can_id(comm_type, data_byte, self.master_id, self.motor_id)
            msg = can.Message(
                arbitration_id=ext_id,
                data=data,
                is_extended_id=True,
                dlc=8
            )
            self.can_bus.send(msg)
            return True
        except Exception as e:
            print(f"Error sending message: {e}")
            return False
    
    def _receive_message(self, expected_comm_type: Optional[int] = None) -> Optional[can.Message]:
        """
        Receive CAN message with optional filtering
        
        Args:
            expected_comm_type: Expected communication type (None for any)
            
        Returns:
            Received CAN message or None
        """
        try:
            msg = self.can_bus.recv(timeout=self.timeout)
            if msg is None:
                return None
                
            # Parse extended ID
            if msg.is_extended_id:
                comm_type, data_byte, master_id, motor_id = parse_extended_can_id(msg.arbitration_id)
                
                # Check if message is for this motor
                if motor_id != self.master_id:  # Response has master_id in motor_id field
                    return None
                    
                # Check communication type if specified
                if expected_comm_type is not None and comm_type != expected_comm_type:
                    return None
                    
                return msg
            return None
        except Exception as e:
            print(f"Error receiving message: {e}")
            return None
    
    # === Core control commands ===
    
    def send_enable(self) -> bool:
        """
        Send motor enable command (Type 0x03)
        
        Returns:
            True if sent successfully
        """
        data = bytes([0x00] * 8)
        return self._send_message(CommunicationType.MOTOR_ENABLE, data)
    
    def send_disable(self, clear_error: bool = False) -> bool:
        """
        Send motor disable command (Type 0x04)
        
        Args:
            clear_error: If True, also clear error flags
            
        Returns:
            True if sent successfully
        """
        data = bytes([0x01 if clear_error else 0x00] + [0x00] * 7)
        return self._send_message(CommunicationType.MOTOR_STOP, data)
    
    def send_set_zero(self) -> bool:
        """
        Send set zero position command (Type 0x06)
        
        Returns:
            True if sent successfully
        """
        data = bytes([0x01] + [0x00] * 7)
        return self._send_message(CommunicationType.SET_POS_ZERO, data)
    
    def send_motion_control(self, cmd: MotionControlCommand) -> bool:
        """
        Send motion control command (Type 0x01)
        
        Args:
            cmd: Motion control command with torque, angle, speed, Kp, Kd
            
        Returns:
            True if sent successfully
        """
        # Encode parameters
        torque_8bit = encode_torque_8bit(cmd.torque)
        angle_bytes = encode_int16(encode_angle_16bit(cmd.angle))
        speed_bytes = encode_int16(encode_speed_16bit(cmd.speed))
        kp_bytes = encode_uint16(encode_kp_16bit(cmd.kp))
        kd_bytes = encode_uint16(encode_kd_16bit(cmd.kd))
        
        # Build data payload
        data = angle_bytes + speed_bytes + kp_bytes + kd_bytes
        
        return self._send_message(CommunicationType.MOTION_CONTROL, data, data_byte=torque_8bit)
    
    # === Parameter access ===
    
    def send_get_parameter(self, param_index: int) -> bool:
        """
        Send get parameter command (Type 0x11)
        
        Args:
            param_index: Parameter index (e.g., 0x7005)
            
        Returns:
            True if sent successfully
        """
        data = encode_uint16(param_index) + bytes([0x00] * 6)
        return self._send_message(CommunicationType.GET_SINGLE_PARAMETER, data)
    
    def send_set_parameter(self, param_index: int, value: float, value_mode: str = 'p') -> bool:
        """
        Send set parameter command (Type 0x12)
        
        Args:
            param_index: Parameter index (e.g., 0x7005)
            value: Parameter value
            value_mode: 'p' for parameter (float), 'j' for mode (uint8)
            
        Returns:
            True if sent successfully
        """
        index_bytes = encode_uint16(param_index)
        
        if value_mode == 'p':
            # Float parameter
            value_bytes = encode_float32(value)
        elif value_mode == 'j':
            # Mode parameter (uint8)
            value_bytes = bytes([int(value), 0x00, 0x00, 0x00])
        else:
            raise ValueError(f"Invalid value_mode: {value_mode}")
        
        data = index_bytes + bytes([0x00, 0x00]) + value_bytes
        return self._send_message(CommunicationType.SET_SINGLE_PARAMETER, data)
    
    def send_save_parameters(self) -> bool:
        """
        Send save parameters to FLASH command (Type 0x16)
        
        Returns:
            True if sent successfully
        """
        # Magic sequence for confirmation
        data = bytes([0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08])
        return self._send_message(CommunicationType.MOTOR_DATA_SAVE, data)
    
    # === Configuration commands ===
    
    def send_get_id(self) -> bool:
        """
        Send get CAN ID command (Type 0x00)
        
        Returns:
            True if sent successfully
        """
        data = bytes([0x00] * 8)
        return self._send_message(CommunicationType.GET_ID, data)
    
    def send_set_can_id(self, new_id: int) -> bool:
        """
        Send set CAN ID command (Type 0x07)
        
        Args:
            new_id: New motor CAN ID (0x00-0x7F)
            
        Returns:
            True if sent successfully
        """
        data = bytes([0x00] * 8)
        return self._send_message(CommunicationType.CAN_ID, data, data_byte=new_id)
    
    def send_change_baud_rate(self, baud_rate_code: int) -> bool:
        """
        Send change baud rate command (Type 0x17)
        
        Args:
            baud_rate_code: 0x01=1M, 0x02=500K, 0x03=250K, 0x04=125K
            
        Returns:
            True if sent successfully
        """
        data = bytes([0x01, 0x02, 0x03, 0x04, 0x05, 0x06, baud_rate_code, 0x08])
        return self._send_message(CommunicationType.BAUD_RATE_CHANGE, data)
    
    def send_set_auto_report(self, enable: bool) -> bool:
        """
        Send proactive reporting enable/disable command (Type 0x18)
        
        Args:
            enable: True to enable auto-reporting
            
        Returns:
            True if sent successfully
        """
        mode = 0x01 if enable else 0x00
        data = bytes([0x01, 0x02, 0x03, 0x04, 0x05, 0x06, mode, 0x08])
        return self._send_message(CommunicationType.PROACTIVE_ESCALATION_SET, data)
    
    def send_set_protocol_mode(self, protocol_mode: int) -> bool:
        """
        Send protocol mode change command (Type 0x19)
        
        Args:
            protocol_mode: 0x00=Private, 0x01=CANopen, 0x02=MIT
            
        Returns:
            True if sent successfully
        """
        data = bytes([0x01, 0x02, 0x03, 0x04, 0x05, 0x06, protocol_mode, 0x08])
        return self._send_message(CommunicationType.MOTOR_MODE_SET, data)
    
    # === Message processing ===
    
    def process_message(self, msg: can.Message, status: MotorStatus, param_data: Optional[ParameterData] = None) -> bool:
        """
        Process received CAN message and update status/parameters
        
        Args:
            msg: Received CAN message
            status: MotorStatus object to update
            param_data: Optional ParameterData object to update
            
        Returns:
            True if message was processed
        """
        if not msg.is_extended_id:
            return False
        
        comm_type, data_byte, master_id, motor_id = parse_extended_can_id(msg.arbitration_id)
        
        # Check if message is from this motor
        if motor_id != self.master_id:
            return False
        
        # Process based on communication type
        if comm_type == CommunicationType.MOTOR_REQUEST:
            # Motor status feedback (Type 0x02)
            return self._process_motor_status(msg, data_byte, status)
        
        elif comm_type == CommunicationType.GET_SINGLE_PARAMETER and param_data is not None:
            # Parameter read response (Type 0x11)
            return self._process_parameter_response(msg, param_data)
        
        elif comm_type == CommunicationType.ERROR_FEEDBACK:
            # Error feedback (Type 0x15)
            status.error_code = data_byte
            return True
        
        return False
    
    def _process_motor_status(self, msg: can.Message, data_byte: int, status: MotorStatus) -> bool:
        """Process motor status feedback message"""
        try:
            data = msg.data
            
            # Extract pattern and error code from data_byte
            # Pattern is in bits [23:22] of ExtID
            pattern = (data_byte >> 6) & 0x03
            error_code = data_byte & 0x3F
            
            # Decode feedback values
            angle_scaled = decode_int16(data, 0)
            speed_scaled = decode_int16(data, 2)
            torque_scaled = decode_int16(data, 4)
            temp_raw = decode_uint16(data, 6)
            
            # Unscale values
            status.angle = decode_angle_16bit(angle_scaled)
            status.speed = decode_speed_feedback_16bit(speed_scaled)
            status.torque = decode_torque_16bit(torque_scaled)
            status.temperature = decode_temperature_16bit(temp_raw)
            status.pattern = pattern
            status.error_code = error_code
            
            return True
        except Exception as e:
            print(f"Error processing motor status: {e}")
            return False
    
    def _process_parameter_response(self, msg: can.Message, param_data: ParameterData) -> bool:
        """Process parameter read response"""
        try:
            data = msg.data
            
            # Extract parameter index
            param_index = decode_uint16(data, 0)
            
            # Extract value (float32 or uint8)
            # Try float32 first
            try:
                value = decode_float32(data, 4)
            except:
                value = float(data[4])
            
            # Update ParameterData based on index
            param_name_map = {
                0x7005: 'run_mode',
                0x7006: 'iq_ref',
                0x700A: 'spd_ref',
                0x700B: 'limit_torque',
                0x7010: 'cur_kp',
                0x7011: 'cur_ki',
                0x7014: 'cur_filt_gain',
                0x7016: 'loc_ref',
                0x7017: 'limit_spd',
                0x7018: 'limit_cur',
                0x7019: 'mech_pos',
                0x701A: 'iqf',
                0x701B: 'mech_vel',
                0x701C: 'vbus',
                0x701D: 'rotation',
                0x7022: 'accel_spd',
                0x7024: 'limit_spd_pp',
                0x7025: 'acceleration',
            }
            
            if param_index in param_name_map:
                param_name = param_name_map[param_index]
                setattr(param_data, param_name, value)
                return True
            
            return False
        except Exception as e:
            print(f"Error processing parameter response: {e}")
            return False
