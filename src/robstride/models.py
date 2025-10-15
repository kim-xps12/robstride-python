"""
Data models for RobStride motor control

Consolidates:
- Enumerations (ControlMode, ProtocolMode, ErrorFlag, etc.)
- Data structures (MotorStatus, ParameterData, etc.)  
- Parameter mapping and validation

Based on specifications:
- doc/architecture/04_data_structures_specification.md
- doc/architecture/05_parameter_mapping.md
"""

from dataclasses import dataclass
from enum import IntEnum, IntFlag
from typing import Dict, Tuple, Optional


# ============================================================================
# ENUMERATIONS
# ============================================================================

class ControlMode(IntEnum):
    """Control mode enumeration"""
    MOTION_CONTROL = 0  # Composite motion control (torque/position/speed)
    POSITION_PP = 1     # PP position control
    SPEED = 2           # Speed control
    CURRENT = 3         # Current control
    SET_ZERO = 4        # Zero position setting mode
    POSITION_CSP = 5    # CSP position control


class ProtocolMode(IntEnum):
    """Protocol mode enumeration"""
    PRIVATE = 0x00   # RobStride proprietary protocol
    CANOPEN = 0x01   # CANopen protocol
    MIT = 0x02       # MIT Cheetah protocol


class CommunicationType(IntEnum):
    """Communication type for Private protocol"""
    GET_ID = 0x00
    MOTION_CONTROL = 0x01
    MOTOR_REQUEST = 0x02
    MOTOR_ENABLE = 0x03
    MOTOR_STOP = 0x04
    SET_POS_ZERO = 0x06
    CAN_ID = 0x07
    GET_SINGLE_PARAMETER = 0x11
    SET_SINGLE_PARAMETER = 0x12
    ERROR_FEEDBACK = 0x15
    MOTOR_DATA_SAVE = 0x16
    BAUD_RATE_CHANGE = 0x17
    PROACTIVE_ESCALATION_SET = 0x18
    MOTOR_MODE_SET = 0x19


class BaudRate(IntEnum):
    """CAN baud rate enumeration"""
    RATE_1M = 0x01    # 1 Mbps
    RATE_500K = 0x02  # 500 Kbps
    RATE_250K = 0x03  # 250 Kbps
    RATE_125K = 0x04  # 125 Kbps


class ErrorFlag(IntFlag):
    """Error flag bitmap (8-bit)"""
    NONE = 0x00
    OVER_TEMPERATURE = 0x01  # Bit 0: Over temperature
    OVER_CURRENT = 0x02      # Bit 1: Over current
    OVER_VOLTAGE = 0x04      # Bit 2: Over voltage
    UNDER_VOLTAGE = 0x08     # Bit 3: Under voltage
    ENCODER_ERROR = 0x10     # Bit 4: Encoder error
    PHASE_ERROR = 0x20       # Bit 5: Phase current unbalance
    RESERVED = 0x40          # Bit 6: Reserved
    CAN_TIMEOUT = 0x80       # Bit 7: CAN timeout


class MotorPattern(IntEnum):
    """Motor operation pattern"""
    TORQUE = 0    # Torque control mode
    POSITION = 1  # Position control mode
    SPEED = 2     # Speed control mode
    RUNNING = 3   # Running state


class MITMotorType(IntEnum):
    """MIT motor control type"""
    OPERATION_CONTROL = 0x01  # Composite control
    POSITION_CONTROL = 0x02   # Position control
    SPEED_CONTROL = 0x03      # Speed control


class MotorState(IntEnum):
    """Motor state machine states"""
    UNINITIALIZED = 0  # Initial state, CAN not connected
    DISABLED = 1       # CAN connected, motor disabled
    ENABLED = 2        # Motor enabled, ready for control
    RUNNING = 3        # Motor running, control active
    FAULT = 4          # Error state, manual recovery required


# ============================================================================
# DATA STRUCTURES
# ============================================================================

@dataclass
class MotorStatus:
    """Motor status information"""
    angle: float = 0.0          # Angle [rad], range: -12.5 ~ 12.5
    speed: float = 0.0          # Speed [rad/s], range: -44 ~ 44
    torque: float = 0.0         # Torque [Nm], range: -17 ~ 17
    temperature: float = 0.0    # Temperature [°C], range: 0 ~ 200
    pattern: int = 0            # Control pattern (0-3)
    error_code: int = 0         # Error code (8-bit bitmap)
    
    @property
    def has_error(self) -> bool:
        """Check if any error flag is set"""
        return self.error_code != 0
    
    @property
    def is_running(self) -> bool:
        """Check if motor is running (pattern > 0)"""
        return self.pattern > 0
    
    def get_error_names(self) -> list[str]:
        """Get list of error flag names"""
        errors = []
        if self.error_code & ErrorFlag.OVER_TEMPERATURE:
            errors.append("OVER_TEMPERATURE")
        if self.error_code & ErrorFlag.OVER_CURRENT:
            errors.append("OVER_CURRENT")
        if self.error_code & ErrorFlag.OVER_VOLTAGE:
            errors.append("OVER_VOLTAGE")
        if self.error_code & ErrorFlag.UNDER_VOLTAGE:
            errors.append("UNDER_VOLTAGE")
        if self.error_code & ErrorFlag.ENCODER_ERROR:
            errors.append("ENCODER_ERROR")
        if self.error_code & ErrorFlag.PHASE_ERROR:
            errors.append("PHASE_ERROR")
        if self.error_code & ErrorFlag.CAN_TIMEOUT:
            errors.append("CAN_TIMEOUT")
        return errors
    
    def __str__(self) -> str:
        """String representation"""
        return (f"MotorStatus(angle={self.angle:.3f} rad, "
                f"speed={self.speed:.3f} rad/s, "
                f"torque={self.torque:.3f} Nm, "
                f"temp={self.temperature:.1f}°C, "
                f"pattern={self.pattern}, "
                f"errors={self.get_error_names()})")


@dataclass
class ParameterData:
    """Motor parameter data"""
    run_mode: float = 0.0
    iq_ref: float = 0.0
    spd_ref: float = 0.0
    limit_torque: float = 12.0
    cur_kp: float = 0.0
    cur_ki: float = 0.0
    cur_filt_gain: float = 0.0
    loc_ref: float = 0.0
    limit_spd: float = 44.0
    limit_cur: float = 23.0
    mech_pos: float = 0.0
    iqf: float = 0.0
    mech_vel: float = 0.0
    vbus: float = 0.0
    rotation: int = 0
    accel_spd: float = 0.0
    limit_spd_pp: float = 30.0
    acceleration: float = 0.0
    
    def __str__(self) -> str:
        return (f"ParameterData(mode={self.run_mode}, "
                f"pos={self.loc_ref:.3f} rad, "
                f"spd={self.spd_ref:.3f} rad/s, "
                f"cur={self.iq_ref:.3f} A, "
                f"vbus={self.vbus:.1f} V)")


@dataclass
class MotionControlCommand:
    """Motion control command for Private protocol Type 0x01"""
    torque: float = 0.0    # Torque [Nm], range: -4 ~ 4
    angle: float = 0.0     # Angle [rad], range: -12.5 ~ 12.5
    speed: float = 0.0     # Speed [rad/s], range: -30 ~ 30
    kp: float = 0.0        # Position gain, range: 0 ~ 500
    kd: float = 0.0        # Damping gain, range: 0 ~ 5


@dataclass
class MITCommand:
    """MIT protocol command"""
    position: float = 0.0   # Position [rad], range: -12.5 ~ 12.5
    velocity: float = 0.0   # Velocity [rad/s], range: -30 ~ 30
    kp: float = 0.0         # Position gain, range: 0 ~ 500
    kd: float = 0.0         # Damping gain, range: 0 ~ 5
    torque: float = 0.0     # Feedforward torque [Nm], range: -18 ~ 18


@dataclass
class CANMessage:
    """CAN message structure"""
    can_id: int
    data: bytes
    is_extended: bool = False
    dlc: int = 8
    
    def to_dict(self) -> dict:
        return {
            'can_id': self.can_id,
            'data': list(self.data),
            'is_extended': self.is_extended,
            'dlc': self.dlc
        }


# ============================================================================
# PARAMETER MAPPING
# ============================================================================

@dataclass
class ParameterSpec:
    """Parameter specification"""
    index: int
    name: str
    data_type: str
    access: str
    min_value: float
    max_value: float
    unit: str
    description: str
    default: Optional[float] = None


PARAMETER_MAP: Dict[int, ParameterSpec] = {
    0x7005: ParameterSpec(0x7005, 'run_mode', 'uint8', 'RW', 0, 5, '', 
                         'Control mode (0=motion, 1=position_pp, 2=speed, 3=current, 4=set_zero, 5=csp)', 0),
    0x7006: ParameterSpec(0x7006, 'iq_ref', 'float32', 'RW', -23.0, 23.0, 'A', 
                         'Current reference (current mode)', 0.0),
    0x700A: ParameterSpec(0x700A, 'spd_ref', 'float32', 'RW', -30.0, 30.0, 'rad/s', 
                         'Speed reference (speed mode)', 0.0),
    0x700B: ParameterSpec(0x700B, 'limit_torque', 'float32', 'RW', 0.0, 12.0, 'Nm', 
                         'Torque limit', 12.0),
    0x7010: ParameterSpec(0x7010, 'cur_kp', 'float32', 'RW', 0.0, 10.0, '', 
                         'Current control Kp gain', 0.0),
    0x7011: ParameterSpec(0x7011, 'cur_ki', 'float32', 'RW', 0.0, 1.0, '', 
                         'Current control Ki gain', 0.0),
    0x7014: ParameterSpec(0x7014, 'cur_filt_gain', 'float32', 'RW', 0.0, 1.0, '', 
                         'Current filter gain', 0.0),
    0x7016: ParameterSpec(0x7016, 'loc_ref', 'float32', 'RW', float('-inf'), float('inf'), 'rad', 
                         'Position reference (position mode)', 0.0),
    0x7017: ParameterSpec(0x7017, 'limit_spd', 'float32', 'RW', 0.0, 44.0, 'rad/s', 
                         'Speed limit (CSP mode)', 44.0),
    0x7018: ParameterSpec(0x7018, 'limit_cur', 'float32', 'RW', 0.0, 23.0, 'A', 
                         'Current limit', 23.0),
    0x7019: ParameterSpec(0x7019, 'mech_pos', 'float32', 'R', float('-inf'), float('inf'), 'rad', 
                         'Mechanical position (cumulative)', None),
    0x701A: ParameterSpec(0x701A, 'iqf', 'float32', 'R', -23.0, 23.0, 'A', 
                         'Filtered current (measured)', None),
    0x701B: ParameterSpec(0x701B, 'mech_vel', 'float32', 'R', -30.0, 30.0, 'rad/s', 
                         'Mechanical velocity (measured)', None),
    0x701C: ParameterSpec(0x701C, 'vbus', 'float32', 'R', 0.0, 60.0, 'V', 
                         'Bus voltage (measured)', None),
    0x701D: ParameterSpec(0x701D, 'rotation', 'int16', 'R', -32768, 32767, 'rounds', 
                         'Rotation count', None),
    0x7022: ParameterSpec(0x7022, 'accel_spd', 'float32', 'RW', 0.0, 100.0, 'rad/s²', 
                         'Acceleration (speed mode)', 0.0),
    0x7024: ParameterSpec(0x7024, 'limit_spd_pp', 'float32', 'RW', 0.0, 30.0, 'rad/s', 
                         'Speed limit (PP position mode)', 30.0),
    0x7025: ParameterSpec(0x7025, 'acceleration', 'float32', 'RW', 0.0, 100.0, 'rad/s²', 
                         'Acceleration (position mode)', 0.0),
}


def validate_parameter(index: int, value: float) -> Tuple[bool, str]:
    """Validate parameter value"""
    if index not in PARAMETER_MAP:
        return False, f"Unknown parameter index: 0x{index:04X}"
    
    spec = PARAMETER_MAP[index]
    
    if 'W' not in spec.access:
        return False, f"Parameter {spec.name} (0x{index:04X}) is read-only"
    
    if not (spec.min_value <= value <= spec.max_value):
        return False, (f"Parameter {spec.name} value {value} out of range "
                      f"[{spec.min_value}, {spec.max_value}]")
    
    return True, ""


def get_parameter_name(index: int) -> str:
    """Get parameter name from index"""
    return PARAMETER_MAP.get(index, ParameterSpec(index, f"UNKNOWN_0x{index:04X}", '', '', 0, 0, '', '', None)).name


def get_parameter_spec(index: int) -> Optional[ParameterSpec]:
    """Get parameter specification"""
    return PARAMETER_MAP.get(index)


def is_readable(index: int) -> bool:
    """Check if parameter is readable"""
    return index in PARAMETER_MAP and 'R' in PARAMETER_MAP[index].access


def is_writable(index: int) -> bool:
    """Check if parameter is writable"""
    return index in PARAMETER_MAP and 'W' in PARAMETER_MAP[index].access


class ParameterIndex:
    """Parameter index constants"""
    RUN_MODE = 0x7005
    IQ_REF = 0x7006
    SPD_REF = 0x700A
    LIMIT_TORQUE = 0x700B
    CUR_KP = 0x7010
    CUR_KI = 0x7011
    CUR_FILT_GAIN = 0x7014
    LOC_REF = 0x7016
    LIMIT_SPD = 0x7017
    LIMIT_CUR = 0x7018
    MECH_POS = 0x7019
    IQF = 0x701A
    MECH_VEL = 0x701B
    VBUS = 0x701C
    ROTATION = 0x701D
    ACCEL_SPD = 0x7022
    LIMIT_SPD_PP = 0x7024
    ACCELERATION = 0x7025


__all__ = [
    # Enums
    'ControlMode', 'ProtocolMode', 'CommunicationType', 'BaudRate',
    'ErrorFlag', 'MotorPattern', 'MITMotorType', 'MotorState',
    # Data structures
    'MotorStatus', 'ParameterData', 'MotionControlCommand', 'MITCommand', 'CANMessage',
    # Parameter mapping
    'ParameterSpec', 'PARAMETER_MAP', 'ParameterIndex',
    'validate_parameter', 'get_parameter_name', 'get_parameter_spec',
    'is_readable', 'is_writable',
]
