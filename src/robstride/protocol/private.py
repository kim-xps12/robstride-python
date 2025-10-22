"""
Private protocol handler for RobStride motor control

Implements all communication types (0x00-0x19) for the Private protocol.
"""

import can
import time
import struct
import logging
from typing import Optional, Callable
from ..models import CommunicationType, MotorStatus, ParameterData, MotionControlCommand
from ..models import get_parameter_spec
from .can_utils import (
    build_extended_can_id,
    parse_extended_can_id,
    encode_int16,
    decode_int16,
    encode_uint16,
    decode_uint16,
    encode_uint16_le,
    decode_uint16_le,
    encode_float32,
    decode_float32,
    encode_float32_le,
    decode_float32_le,
    encode_uint64,
    decode_uint64,
    encode_angle_16bit,
    decode_angle_16bit,
    encode_angle_auto_report_16bit,
    decode_angle_auto_report_16bit,
    encode_speed_16bit,
    decode_speed_16bit,
    decode_speed_feedback_16bit,
    encode_kp_16bit,
    decode_kp_16bit,
    encode_kd_16bit,
    decode_kd_16bit,
    decode_torque_16bit,
    decode_torque_auto_report_16bit,
    decode_speed_auto_report_16bit,
    decode_temperature_16bit,
    encode_torque_8bit,
    encode_torque_16bit
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
        self.last_device_id: Optional[int] = None
        self.last_device_uid: Optional[int] = None
    
    def _send_message(self, comm_type: int, data: bytes, data_field: int = 0x00) -> bool:
        """
        Send CAN message with Private protocol format
        
        Args:
            comm_type: Communication type
            data: 8-byte data payload
            data_field: Additional data field (8-bit for most commands)
            
        Returns:
            True if sent successfully
        """
        try:
            ext_id = build_extended_can_id(comm_type, data_field, self.master_id, self.motor_id)
            # Show human-readable frame for debugging parameter writes
            try:
                hex_data = ' '.join(f"{b:02X}" for b in data)
            except Exception:
                hex_data = str(data)
            logger = logging.getLogger(__name__)
            if comm_type == CommunicationType.SET_SINGLE_PARAMETER:
                logger.debug(f"[PARAM TX] EXTID=0x{ext_id:08X} DATA={hex_data}")

            msg = can.Message(
                arbitration_id=ext_id,
                data=data,
                is_extended_id=True,
                dlc=8
            )
            self.can_bus.send(msg)
            return True
        except Exception as e:
            logging.error(f"Error sending message: {e}")
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
            if not msg.is_extended_id:
                return None

            comm_type, data_field, master_id, motor_id = parse_extended_can_id(msg.arbitration_id)

            # Message addressing varies across RS02 variants: some devices
            # place the host/master ID in the low byte (motor_id field),
            # others place it in the high byte of the 16-bit data field.
            # Accept messages when either the extracted master_id or motor_id
            # matches the expected host/master ID or this motor's ID.
            if not (
                master_id == self.master_id or
                motor_id == self.master_id or
                master_id == self.motor_id or
                motor_id == self.motor_id
            ):
                return None

            # Check communication type if specified
            if expected_comm_type is not None and comm_type != expected_comm_type:
                return None

            return msg
        except Exception as e:
            logging.error(f"Error receiving message: {e}")
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
        
        Per specification:
        - ExtID: [Type:5][Torque:16][MotorID:8]  
        - Data: [Angle:16][Speed:16][Kp:16][Kd:16] (big-endian)
        
        Args:
            cmd: Motion control command with torque, angle, speed, Kp, Kd
            
        Returns:
            True if sent successfully
        """
        # Encode parameters according to specification
        torque_16bit = encode_torque_16bit(cmd.torque)
        angle_16bit = encode_angle_16bit(cmd.angle)
        speed_16bit = encode_speed_16bit(cmd.speed) 
        kp_16bit = encode_kp_16bit(cmd.kp)
        kd_16bit = encode_kd_16bit(cmd.kd)
        
        # Build data payload (big-endian, high byte first)
        angle_bytes = encode_uint16(angle_16bit)
        speed_bytes = encode_uint16(speed_16bit)
        kp_bytes = encode_uint16(kp_16bit)
        kd_bytes = encode_uint16(kd_16bit)
        
        data = angle_bytes + speed_bytes + kp_bytes + kd_bytes
        
        # Build extended CAN ID for motion control
        try:
            ext_id = build_extended_can_id(0x01, torque_16bit, self.master_id, self.motor_id)
            msg = can.Message(
                arbitration_id=ext_id,
                data=data,
                is_extended_id=True,
                dlc=8
            )
            # Debug: show what is being sent
            try:
                hex_data = ' '.join(f"{b:02X}" for b in data)
                logging.debug(f"[DEBUG TX] EXTID=0x{ext_id:08X} DATA={hex_data}")
            except Exception:
                pass

            self.can_bus.send(msg)
            return True
        except Exception as e:
            logging.error(f"Error sending motion control message: {e}")
            return False
    
    # === Parameter access ===
    
    def send_get_parameter(self, param_index: int) -> bool:
        """
        Send get parameter command (Type 0x11)
        
        Per specification: 「低バイトが前、高バイトが後」(little-endian)
        
        Args:
            param_index: Parameter index (e.g., 0x7005)
            
        Returns:
            True if sent successfully
        """
        data = encode_uint16_le(param_index) + bytes([0x00] * 6)
        return self._send_message(CommunicationType.GET_SINGLE_PARAMETER, data)
    
    def send_set_parameter(self, param_index: int, value: float, value_mode: str = 'p') -> bool:
        """
        Send set parameter command (Type 0x12)
        
        Per specification: 「低バイトが前、高バイトが後」(little-endian)
        
        Args:
            param_index: Parameter index (e.g., 0x7005)
            value: Parameter value
            value_mode: 'p' for parameter (float), 'j' for mode (uint8)
            
        Returns:
            True if sent successfully
        """
        index_bytes = encode_uint16_le(param_index)
        
        if value_mode == 'p':
            # Float parameter (little-endian)
            value_bytes = encode_float32_le(value)
        elif value_mode == 'j':
            # Mode parameter (uint8)
            value_bytes = bytes([int(value), 0x00, 0x00, 0x00])
        else:
            raise ValueError(f"Invalid value_mode: {value_mode}")
        
        data = index_bytes + bytes([0x00, 0x00]) + value_bytes
        # Debug: show parameter set frame
        try:
            ext_id = build_extended_can_id(CommunicationType.SET_SINGLE_PARAMETER, 0x00, self.master_id, self.motor_id)
            hex_data = ' '.join(f"{b:02X}" for b in data)
            logging.debug(f"[DEBUG PARAM TX] EXTID=0x{ext_id:08X} DATA={hex_data} (index=0x{param_index:04X}, mode={value_mode})")
        except Exception:
            pass
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
        sent = self._send_message(CommunicationType.GET_ID, data)

        # Some RS02 variants/examples show different byte-ordering in the
        # 16-bit data field. As a robustness measure, also send an alternate
        # extended ID encoding where the master ID is placed in the other
        # byte of the 16-bit field. This increases chance of discovery on
        # devices that expect the opposite ordering.
        try:
            # Alternate ext id: build the 16-bit field with master in low byte
            alt_data_field = (0x00 << 8) | (self.master_id & 0xFF)
            alt_ext_id = build_extended_can_id(CommunicationType.GET_ID, alt_data_field, self.master_id, self.motor_id)
            alt_msg = can.Message(arbitration_id=alt_ext_id, data=data, is_extended_id=True, dlc=8)
            self.can_bus.send(alt_msg)
            sent = sent or True
        except Exception:
            # Ignore secondary send failures
            pass

        return bool(sent)
    
    def send_set_can_id(self, new_id: int) -> bool:
        """
        Send set CAN ID command (Type 0x07)
        
        Args:
            new_id: New motor CAN ID (0x00-0x7F)
            
        Returns:
            True if sent successfully
        """
        if not (0x00 <= new_id <= 0x7F):
            raise ValueError(f"Invalid CAN ID: {new_id}")
        
        data = bytes([0x00] * 8)
        # Per RS02 spec: Bit23-16 = new CAN ID, Bit15-8 = host/master ID
        data_field = ((new_id & 0xFF) << 8) | (self.master_id & 0xFF)
        return self._send_message(CommunicationType.CAN_ID, data, data_field=data_field)
    
    def send_change_baud_rate(self, baud_rate_code: int) -> bool:
        """
        Send change baud rate command (Type 0x17)
        
        Args:
            baud_rate_code: 0x01=1M, 0x02=500K, 0x03=250K, 0x04=125K
            
        Returns:
            True if sent successfully
        """
        data = bytes([0x01, 0x02, 0x03, 0x04, 0x05, 0x06, baud_rate_code, 0x00])
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
        data = bytes([0x01, 0x02, 0x03, 0x04, 0x05, 0x06, mode, 0x00])
        return self._send_message(CommunicationType.PROACTIVE_ESCALATION_SET, data)
    
    def send_set_protocol_mode(self, protocol_mode: int) -> bool:
        """
        Send protocol mode change command (Type 0x19)
        
        Args:
            protocol_mode: 0x00=Private, 0x01=CANopen, 0x02=MIT
            
        Returns:
            True if sent successfully
        """
        data = bytes([0x01, 0x02, 0x03, 0x04, 0x05, 0x06, protocol_mode, 0x00])
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

        comm_type, data_field, master_id, motor_id = parse_extended_can_id(msg.arbitration_id)

        # Log the incoming message for debugging
        logging.getLogger(__name__).debug(
            f"[PROC MSG] EXTID=0x{msg.arbitration_id:08X} COMM=0x{comm_type:02X} "
            f"DATAFIELD=0x{data_field:04X} MASTER=0x{master_id:02X} MOTOR=0x{motor_id:02X}"
        )

        # Handle GET_ID immediately
        if comm_type == CommunicationType.GET_ID:
            return self._process_get_id(msg, data_field, master_id, status)

        # Auto-report (Type 0x18) has motor_id field = 0x00, handle separately
        if comm_type == CommunicationType.PROACTIVE_ESCALATION_SET:
            return self._process_auto_report(msg, data_field, status)

        # Addressing: accept messages when any of the candidate ID locations
        # (extid motor_id, data_field high byte, data_field low byte) match
        # either the host/master ID or the expected motor ID. Some devices
        # place IDs in different bytes, so check all of them.
        data_high = (data_field >> 8) & 0xFF
        data_low = data_field & 0xFF
        candidate_ids = {motor_id, data_high, data_low}
        if not (self.master_id in candidate_ids or self.motor_id in candidate_ids):
            return False

        # Dispatch based on communication type
        if comm_type == CommunicationType.MOTOR_REQUEST:
            return self._process_motor_status(msg, data_field, status)

        if comm_type == CommunicationType.GET_SINGLE_PARAMETER and param_data is not None:
            logging.getLogger(__name__).debug(f"[PARAM HANDLER] Received GET_SINGLE_PARAMETER extid=0x{msg.arbitration_id:08X}")
            processed = self._process_parameter_response(msg, param_data)
            logging.getLogger(__name__).debug(
                f"[PARAM HANDLER] process result={processed} param.limit_cur={getattr(param_data,'limit_cur',None)}"
            )
            return processed

        if comm_type == CommunicationType.ERROR_FEEDBACK:
            return self._process_error_feedback(msg, data_field, status)

        return False
    
    def _process_get_id(self, msg: can.Message, data_field: int, master_id: int, status: MotorStatus) -> bool:
        """
        Process GET_ID response (Type 0x00)
        
        Response layout:
        - Bit23-16: Motor CAN ID
        - Bit7-0: 0xFE (broadcast indicator)
        - Data bytes: 64-bit MCU unique identifier
        """
        try:
            # Some devices may place the responding motor ID in either the
            # high or low byte of the 16-bit data field. Accept both.
            responding_motor_id_high = (data_field >> 8) & 0xFF
            responding_motor_id_low = data_field & 0xFF

            if responding_motor_id_high == self.motor_id:
                responding_motor_id = responding_motor_id_high
            elif responding_motor_id_low == self.motor_id:
                responding_motor_id = responding_motor_id_low
            else:
                return False
            
            unique_id = decode_uint64(msg.data, 0)
            status.device_id = responding_motor_id
            status.device_uid = unique_id
            self.last_device_id = responding_motor_id
            self.last_device_uid = unique_id
            return True
        except Exception as e:
            logging.error(f"Error processing GET_ID response: {e}")
            return False
    
    def _process_motor_status(self, msg: can.Message, data_field: int, status: MotorStatus) -> bool:
        """
        Process motor status feedback message (Type 0x02)
        
        Per specification:
        ExtID Bit23~8: [Pattern:2][Error:6][MotorID:8]
        Data: [Angle:16][Speed:16][Torque:16][Temperature:16] (big-endian)
        """
        try:
            data = msg.data
            # Debug: show received raw frame for motor status
            try:
                hex_data = ' '.join(f"{b:02X}" for b in data)
                logging.debug(f"[DEBUG RX] EXTID=0x{msg.arbitration_id:08X} DATA={hex_data}")
            except Exception:
                pass
            
            # Extract pattern and error code from data_field (16-bit from ExtID)
            pattern = (data_field >> 14) & 0x03    # Bits 15:14
            error_code = (data_field >> 8) & 0x3F  # Bits 13:8
            current_motor_id = data_field & 0xFF   # Bits 7:0
            
            # Verify this is from the correct motor
            if current_motor_id != self.motor_id:
                return False
            
            # Decode feedback values (big-endian, unsigned 16-bit)
            angle_raw = decode_uint16(data, 0)
            speed_raw = decode_uint16(data, 2)
            torque_raw = decode_uint16(data, 4)
            temp_raw = decode_uint16(data, 6)
            
            # Convert to physical values using dedicated decode functions
            status.angle = decode_angle_16bit(angle_raw)             # -4π ~ 4π (-12.57 ~ 12.57 rad)
            status.speed = decode_speed_16bit(speed_raw)             # -44 ~ 44 rad/s
            status.torque = decode_torque_16bit(torque_raw)          # -17 ~ 17 Nm
            status.temperature = decode_temperature_16bit(temp_raw)  # Temperature × 10, decoded to °C
            status.pattern = pattern
            status.error_code = error_code
            
            return True
        except Exception as e:
            logging.error(f"Error processing motor status: {e}")
            return False
    
    def _process_auto_report(self, msg: can.Message, data_field: int, status: MotorStatus) -> bool:
        """
        Process auto-report feedback message (Type 0x18 response)
        
        Per RS02 specification (corrected 2025-10-16):
        - Angle: -12.57f ~ +12.57f (±12.57 rad, same as Type 0x02)
        - Speed: -33 ~ +33 rad/s (different from Type 0x02's ±44 rad/s)
        - Torque: -14 ~ +14 Nm (different from Type 0x02's ±17 Nm)
        """
        try:
            data = msg.data
            # Debug: show received raw frame for auto-report
            try:
                hex_data = ' '.join(f"{b:02X}" for b in data)
                logging.debug(f"[DEBUG RX] EXTID=0x{msg.arbitration_id:08X} DATA={hex_data}")
            except Exception:
                pass
            
            pattern = (data_field >> 14) & 0x03
            error_code = (data_field >> 8) & 0x3F
            current_motor_id = data_field & 0xFF
            
            if current_motor_id != self.motor_id:
                return False
            
            angle_raw = decode_uint16(data, 0)
            speed_raw = decode_uint16(data, 2)
            torque_raw = decode_uint16(data, 4)
            temp_raw = decode_uint16(data, 6)
            
            status.angle = decode_angle_auto_report_16bit(angle_raw)
            status.speed = decode_speed_auto_report_16bit(speed_raw)
            status.torque = decode_torque_auto_report_16bit(torque_raw)
            status.temperature = decode_temperature_16bit(temp_raw)
            status.pattern = pattern
            status.error_code = error_code
            return True
        except Exception as e:
            logging.error(f"Error processing auto report: {e}")
            return False
    
    def _process_error_feedback(self, msg: can.Message, data_field: int, status: MotorStatus) -> bool:
        """
        Process error feedback frame (Type 0x15)
        """
        try:
            motor_id = (data_field >> 8) & 0xFF
            if motor_id != self.motor_id:
                return False
            
            fault = struct.unpack('>I', msg.data[0:4])[0]
            warning = struct.unpack('>I', msg.data[4:8])[0]
            
            status.fault_code = fault
            status.warning_code = warning
            return True
        except Exception as e:
            logging.error(f"Error processing error feedback: {e}")
            return False
    
    def _process_parameter_response(self, msg: can.Message, param_data: ParameterData) -> bool:
        """
        Process parameter read response (Type 0x11 response)
        
        Per specification: 「低バイトが前、高バイトが後」(little-endian)
        """
        try:
            data = msg.data
            
            # Extract parameter index (little-endian)
            param_index = decode_uint16_le(data, 0)

            # Determine parameter spec if available
            spec = get_parameter_spec(param_index)

            # Extract value. Prefer float32 (little-endian), but fall back
            # to integer decoding (uint32/uint16/uint8) based on spec or
            # observed payload when float decode fails.
            value = None
            if spec and spec.data_type == 'float32':
                try:
                    value = decode_float32_le(data, 4)
                except Exception:
                    value = None
            elif spec and spec.data_type in ('uint32', 'uint16', 'uint8'):
                try:
                    if spec.data_type == 'uint32':
                        import struct as _struct
                        value = _struct.unpack_from('<I', data, 4)[0]
                    elif spec.data_type == 'uint16':
                        value = decode_uint16_le(data, 4)
                    else:
                        # uint8
                        value = int(data[4])
                except Exception:
                    value = None

            # Generic fallback: try float32, then uint32/uint16/uint8
            if value is None:
                try:
                    value = decode_float32_le(data, 4)
                except Exception:
                    try:
                        import struct as _struct
                        value = _struct.unpack_from('<I', data, 4)[0]
                    except Exception:
                        try:
                            value = decode_uint16_le(data, 4)
                        except Exception:
                            # Last resort: take single byte as integer
                            value = float(data[4])

            # Log raw response for debugging
            try:
                hex_data = ' '.join(f"{b:02X}" for b in data)
            except Exception:
                hex_data = str(data)
            logging.getLogger(__name__).debug(f"[PARAM RX] IDX=0x{param_index:04X} RAW={hex_data} DECODED={value}")

            # Update ParameterData based on index - Complete RS02 parameter table
            param_name_map = {
                0x7005: 'run_mode',
                0x7006: 'iq_ref',
                0x7008: 'limit_spd',           # Speed mode speed limit
                0x700A: 'spd_ref',
                0x700B: 'limit_torque',
                0x700F: 'limit_torque',        # Alternative torque limit register
                0x7010: 'cur_kp',
                0x7011: 'cur_ki',
                0x7014: 'cur_filt_gain',
                0x7016: 'loc_ref',
                0x7017: 'limit_spd_csp',       # CSP mode speed limit
                0x7018: 'limit_cur',
                0x7019: 'mech_pos',
                0x701A: 'iqf',
                0x701B: 'mech_vel',
                0x701C: 'vbus',
                0x701D: 'rotation',
                0x701E: 'loc_kp',              # Position control Kp
                0x701F: 'spd_kp',              # Speed control Kp
                0x7020: 'spd_ki',              # Speed control Ki
                0x7021: 'spd_filt_gain',       # Speed loop filter gain
                0x7022: 'acc_rad',             # Position mode acceleration
                0x7024: 'limit_spd_pp',        # PP mode max velocity
                0x7025: 'acceleration',        # PP mode acceleration setting
                0x7026: 'epscan_time',         # Auto-report interval
                0x7028: 'can_timeout',         # CAN timeout
                0x7029: 'zero_sta',            # Zero point status
            }
            
            if param_index in param_name_map:
                param_name = param_name_map[param_index]
                setattr(param_data, param_name, value)
                return True
            
            return False
        except Exception as e:
            logging.error(f"Error processing parameter response: {e}")
            return False
