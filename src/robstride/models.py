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
    """
    Control mode enumeration (RS02 run_mode 0x7005)
    
    Per RS02 specification:
    0: モーション制御モード (Motion Control Mode)
    1: 位置制御モード（PP） (Position Control Mode - PP)
    2: 速度制御モード (Speed Control Mode)
    3: 電流制御モード (Current Control Mode)
    5: 位置モード（CSP） (Position Mode - CSP)
    """
    MOTION_CONTROL = 0        # Motion control mode (composite control with torque/position/speed/kp/kd)
    POSITION_PP = 1           # Position control mode (PP - Point to Point)
    SPEED = 2                 # Speed control mode
    CURRENT = 3               # Current control mode
    SET_ZERO = 4              # Zero position setting mode (vendor extension, not in RS02 spec)
    POSITION_CSP = 5          # Position mode (CSP - Cyclic Synchronous Position)


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
    """
    Error flags extracted from Communication Type 0x02 (bits 21-16 of the ExtID data field).
    
    Each flag is normalised to a compact 6-bit bitmap, where bit0 corresponds to RS02 bit16.
    """
    NONE = 0x00
    UNDER_VOLTAGE = 1 << 0      # RS02 bit16
    OVER_CURRENT = 1 << 1       # RS02 bit17
    OVER_TEMPERATURE = 1 << 2   # RS02 bit18
    ENCODER_FAULT = 1 << 3      # RS02 bit19
    OVER_INTEGRATION = 1 << 4   # RS02 bit20
    UNCALIBRATED = 1 << 5       # RS02 bit21


class FaultFlag(IntFlag):
    """
    Fault flags reported by Communication Type 0x15 payload (32-bit fault field).
    """
    NONE = 0x00000000
    MOTOR_OVER_TEMP = 1 << 10         # Motor over-temperature (135°C)
    DRIVER_CHIP_FAULT = 1 << 11       # Driver chip fault
    UNDER_VOLTAGE = 1 << 12           # Under-voltage fault
    OVER_TEMPERATURE = 1 << 13        # Controller over-temperature fault
    OVER_INTEGRATION = 1 << 14        # Over-integration fault
    ENCODER_UNCALIBRATED = 1 << 17    # Magnetic encoder uncalibrated


class WarningFlag(IntFlag):
    """
    Warning flags reported by Communication Type 0x15 payload (32-bit warning field).
    """
    NONE = 0x00000000
    MOTOR_OVER_TEMP = 1 << 0          # Motor over-temperature warning (125°C)


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
    device_id: Optional[int] = None       # Motor CAN ID returned by GET_ID
    device_uid: Optional[int] = None      # 64-bit MCU unique identifier
    angle: float = 0.0          # Angle [rad], range: -12.57 ~ 12.57
    speed: float = 0.0          # Speed [rad/s], range: -44 ~ 44
    torque: float = 0.0         # Torque [Nm], range: -17 ~ 17
    temperature: float = 0.0    # Temperature [°C], range: 0 ~ 200
    pattern: int = 0            # Control pattern (0-3)
    error_code: int = 0         # Type 0x02 error flags (normalised 6-bit bitmap)
    fault_code: int = 0         # Type 0x15 fault bitmap
    warning_code: int = 0       # Type 0x15 warning bitmap
    
    @property
    def has_error(self) -> bool:
        """Check if any error flag is set"""
        return (self.error_code != 0) or (self.fault_code != 0)
    
    @property
    def is_running(self) -> bool:
        """Check if motor is running (pattern > 0)"""
        return self.pattern > 0
    
    def get_error_names(self) -> list[str]:
        """Get list of error flag names across Type0x02 and Type0x15 feedback"""
        errors = []
        
        error_flags = ErrorFlag(self.error_code)
        for flag in ErrorFlag:
            if flag is ErrorFlag.NONE:
                continue
            if error_flags & flag and flag.name not in errors:
                errors.append(flag.name)
        
        fault_flags = FaultFlag(self.fault_code)
        for flag in FaultFlag:
            if flag is FaultFlag.NONE:
                continue
            if fault_flags & flag and flag.name not in errors:
                errors.append(flag.name)
        
        warning_flags = WarningFlag(self.warning_code)
        for flag in WarningFlag:
            if flag is WarningFlag.NONE:
                continue
            name = f"WARNING_{flag.name}"
            if warning_flags & flag and name not in errors:
                errors.append(name)
        
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
    limit_spd: float = 44.0          # Speed mode speed limit (0x7008)
    limit_torque: float = 12.0       # Torque limit (0x700F)
    cur_kp: float = 0.0
    cur_ki: float = 0.0
    cur_filt_gain: float = 0.0
    loc_ref: float = 0.0
    limit_spd_csp: float = 44.0      # CSP mode speed limit (0x7017) - alias for limit_spd
    limit_cur: float = 23.0
    mech_pos: float = 0.0
    iqf: float = 0.0
    mech_vel: float = 0.0
    vbus: float = 0.0
    loc_kp: float = 0.0              # Position control Kp (0x701E)
    spd_kp: float = 0.0              # Speed control Kp (0x701F)
    spd_ki: float = 0.0              # Speed control Ki (0x7020)
    spd_filt_gain: float = 0.0       # Speed loop filter gain (0x7021)
    acc_rad: float = 0.0             # Position mode acceleration (0x7022)
    rotation: int = 0
    accel_spd: float = 0.0           # Legacy field, may overlap with acc_rad
    limit_spd_pp: float = 30.0       # PP mode max speed (0x7024)
    acceleration: float = 0.0        # PP mode acceleration (0x7025)
    epscan_time: int = 1             # Auto-report time setting (0x7026)
    can_timeout: int = 30            # CAN timeout in ms (0x7028)
    zero_sta: int = 0                # Zero point status (0x7029)
    
    def __str__(self) -> str:
        return (f"ParameterData(mode={self.run_mode}, "
                f"pos={self.loc_ref:.3f} rad, "
                f"spd={self.spd_ref:.3f} rad/s, "
                f"cur={self.iq_ref:.3f} A, "
                f"vbus={self.vbus:.1f} V)")


@dataclass
class MotionControlCommand:
    """Motion control command for Private protocol Type 0x01"""
    torque: float = 0.0    # Torque [Nm], range: -17 ~ 17
    angle: float = 0.0     # Angle [rad], range: -12.57 ~ 12.57
    speed: float = 0.0     # Speed [rad/s], range: -44 ~ 44
    kp: float = 0.0        # Position gain, range: 0 ~ 500
    kd: float = 0.0        # Damping gain, range: 0 ~ 5


@dataclass
class MITCommand:
    """MIT protocol command"""
    position: float = 0.0   # Position [rad], range: -12.57 ~ 12.57
    velocity: float = 0.0   # Velocity [rad/s], range: -44 ~ 44
    kp: float = 0.0         # Position gain, range: 0 ~ 500
    kd: float = 0.0         # Damping gain, range: 0 ~ 5
    torque: float = 0.0     # Feedforward torque [Nm], range: -17 ~ 17


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
    0x7008: ParameterSpec(0x7008, 'limit_spd', 'float32', 'RW', -44.0, 44.0, 'rad/s', 
                         'Speed limit (speed mode)', 44.0),
    0x700A: ParameterSpec(0x700A, 'spd_ref', 'float32', 'RW', -44.0, 44.0, 'rad/s', 
                         'Speed reference (speed mode)', 0.0),
    0x700B: ParameterSpec(0x700B, 'limit_torque', 'float32', 'RW', 0.0, 12.0, 'Nm', 
                         'Torque limit', 12.0),
    0x700F: ParameterSpec(0x700F, 'limit_torque_alt', 'float32', 'RW', 0.0, 17.0, 'Nm', 
                         'Torque limit (alternative register)', 17.0),
    0x7010: ParameterSpec(0x7010, 'cur_kp', 'float32', 'RW', 0.0, 10.0, '', 
                         'Current control Kp gain', 0.17),
    0x7011: ParameterSpec(0x7011, 'cur_ki', 'float32', 'RW', 0.0, 1.0, '', 
                         'Current control Ki gain', 0.012),
    0x7014: ParameterSpec(0x7014, 'cur_filt_gain', 'float32', 'RW', 0.0, 1.0, '', 
                         'Current filter gain', 0.1),
    0x7016: ParameterSpec(0x7016, 'loc_ref', 'float32', 'RW', float('-inf'), float('inf'), 'rad', 
                         'Position reference (position mode)', 0.0),
    0x7017: ParameterSpec(0x7017, 'limit_spd_csp', 'float32', 'RW', -44.0, 44.0, 'rad/s', 
                         'Speed limit (CSP mode)', 44.0),
    0x7018: ParameterSpec(0x7018, 'limit_cur', 'float32', 'RW', 0.0, 23.0, 'A', 
                         'Current limit', 23.0),
    0x7019: ParameterSpec(0x7019, 'mech_pos', 'float32', 'R', float('-inf'), float('inf'), 'rad', 
                         'Mechanical position (cumulative)', None),
    0x701A: ParameterSpec(0x701A, 'iqf', 'float32', 'R', -23.0, 23.0, 'A', 
                         'Filtered current (measured)', None),
    0x701B: ParameterSpec(0x701B, 'mech_vel', 'float32', 'R', -44.0, 44.0, 'rad/s', 
                         'Mechanical velocity (measured)', None),
    0x701C: ParameterSpec(0x701C, 'vbus', 'float32', 'R', 0.0, 60.0, 'V', 
                         'Bus voltage (measured)', None),
    0x701D: ParameterSpec(0x701D, 'rotation', 'int16', 'R', -32768, 32767, 'rounds', 
                         'Rotation count', None),
    0x701E: ParameterSpec(0x701E, 'loc_kp', 'float32', 'RW', 0.0, 500.0, '', 
                         'Position control Kp gain', 40.0),
    0x701F: ParameterSpec(0x701F, 'spd_kp', 'float32', 'RW', 0.0, 100.0, '', 
                         'Speed control Kp gain', 16.0),
    0x7020: ParameterSpec(0x7020, 'spd_ki', 'float32', 'RW', 0.0, 10.0, '', 
                         'Speed control Ki gain', 0.02),
    0x7021: ParameterSpec(0x7021, 'spd_filt_gain', 'float32', 'RW', 0.0, 1.0, '', 
                         'Speed filter gain', 0.1),
    0x7022: ParameterSpec(0x7022, 'acc_rad', 'float32', 'RW', 0.0, 100.0, 'rad/s²', 
                         'Acceleration (position mode)', 20.0),
    0x7024: ParameterSpec(0x7024, 'vel_max', 'float32', 'RW', 0.0, 44.0, 'rad/s', 
                         'Maximum speed (PP position mode)', 10.0),
    0x7025: ParameterSpec(0x7025, 'acc_set', 'float32', 'RW', 0.0, 100.0, 'rad/s²', 
                         'Acceleration setting (PP position mode)', 10.0),
    0x7026: ParameterSpec(0x7026, 'EPScan_time', 'uint16', 'RW', 1, 1500, '', 
                         'Auto-report interval (1 unit = 10ms, max +15ms)', 1),
    0x7028: ParameterSpec(0x7028, 'canTimeout', 'uint32', 'RW', 0, 20000, 'ms', 
                         'CAN communication timeout', 30),
    0x7029: ParameterSpec(0x7029, 'zero_sta', 'uint8', 'RW', 0, 1, '', 
                         'Zero position status (0=-2π, 1=+π)', 0),
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
    """Parameter index constants for all supported parameters"""
    # Control mode and basic settings
    RUN_MODE = 0x7005
    IQ_REF = 0x7006
    LIMIT_SPD = 0x7008           # Speed mode speed limit
    SPD_REF = 0x700A
    LIMIT_TORQUE = 0x700B
    LIMIT_TORQUE_ALT = 0x700F    # Alternative torque limit register
    
    # Current control parameters
    CUR_KP = 0x7010
    CUR_KI = 0x7011
    CUR_FILT_GAIN = 0x7014
    
    # Position and speed control
    LOC_REF = 0x7016
    LIMIT_SPD_CSP = 0x7017       # CSP mode speed limit
    LIMIT_CUR = 0x7018
    
    # Measured values (read-only)
    MECH_POS = 0x7019
    IQF = 0x701A
    MECH_VEL = 0x701B
    VBUS = 0x701C
    ROTATION = 0x701D
    
    # Position and speed loop gains
    LOC_KP = 0x701E
    SPD_KP = 0x701F
    SPD_KI = 0x7020
    SPD_FILT_GAIN = 0x7021
    
    # Motion profile parameters
    ACC_RAD = 0x7022             # Position mode acceleration
    VEL_MAX = 0x7024             # PP mode maximum speed
    ACC_SET = 0x7025             # PP mode acceleration setting
    
    # System parameters
    EPSCAN_TIME = 0x7026         # Auto-report interval
    CAN_TIMEOUT = 0x7028         # CAN communication timeout
    ZERO_STA = 0x7029            # Zero position status


__all__ = [
    # Enums
    'ControlMode', 'ProtocolMode', 'CommunicationType', 'BaudRate',
    'ErrorFlag', 'FaultFlag', 'WarningFlag',
    'MotorPattern', 'MITMotorType', 'MotorState',
    # Data structures
    'MotorStatus', 'ParameterData', 'MotionControlCommand', 'MITCommand', 'CANMessage',
    # Parameter mapping
    'ParameterSpec', 'PARAMETER_MAP', 'ParameterIndex',
    'validate_parameter', 'get_parameter_name', 'get_parameter_spec',
    'is_readable', 'is_writable',
]
