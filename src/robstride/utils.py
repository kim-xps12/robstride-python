"""
Utility functions for RobStride motor control

Consolidates:
- Error handling (ErrorHandler with recovery strategies)
- Error logging (ErrorLogger for structured logging)
- Input validation (parameter range checking)
- Unit conversions (deg/rad, rpm/rad/s, torque/current)

Based on specifications:
- doc/architecture/07_error_handling_specification.md
"""

import logging
import math
import time
from typing import Optional
from pathlib import Path
from .models import ErrorFlag, FaultFlag, WarningFlag


# ============================================================================
# EXCEPTION CLASSES
# ============================================================================

class MotorException(Exception):
    """Base exception for motor control errors"""
    pass


class CANException(MotorException):
    """CAN communication error"""
    pass


class ParameterException(MotorException):
    """Parameter validation or access error"""
    pass


class ProtocolException(MotorException):
    """Protocol-related error"""
    pass


# ============================================================================
# ERROR HANDLER
# ============================================================================

class ErrorHandler:
    """
    Error handler for motor control
    
    Provides error detection, classification, and recovery strategies.
    """
    
    def __init__(self, motor):
        """
        Initialize error handler
        
        Args:
            motor: RobStrideMotor instance
        """
        self.motor = motor
        self.logger = logging.getLogger(__name__)
        
    def check_errors(self) -> bool:
        """
        Check if motor has any errors
        
        Returns:
            True if errors present
        """
        return self.motor.status.has_error
    
    def get_error_description(self, error_code: int, fault_code: int = 0, warning_code: int = 0) -> str:
        """
        Get human-readable error description
        
        Args:
            error_code: Error code bitmap
            fault_code: Fault bitmap from Type 0x15 payload
            warning_code: Warning bitmap from Type 0x15 payload
            
        Returns:
            Error description string
        """
        errors = []
        
        error_flags = ErrorFlag(error_code)
        if error_flags & ErrorFlag.OVER_TEMPERATURE and "Over-temperature (>80°C)" not in errors:
            errors.append("Over-temperature (>80°C)")
        if error_flags & ErrorFlag.OVER_CURRENT and "Over-current (>23A)" not in errors:
            errors.append("Over-current (>23A)")
        if error_flags & ErrorFlag.UNDER_VOLTAGE and "Under-voltage (<12V)" not in errors:
            errors.append("Under-voltage (<12V)")
        if error_flags & ErrorFlag.ENCODER_FAULT and "Encoder fault" not in errors:
            errors.append("Encoder fault")
        if error_flags & ErrorFlag.OVER_INTEGRATION and "Over-integration fault" not in errors:
            errors.append("Over-integration fault")
        if error_flags & ErrorFlag.UNCALIBRATED and "Uncalibrated encoder" not in errors:
            errors.append("Uncalibrated encoder")
        
        fault_flags = FaultFlag(fault_code)
        if fault_flags & FaultFlag.MOTOR_OVER_TEMP and "Motor over-temperature fault (>135°C)" not in errors:
            errors.append("Motor over-temperature fault (>135°C)")
        if fault_flags & FaultFlag.DRIVER_CHIP_FAULT and "Driver chip fault" not in errors:
            errors.append("Driver chip fault")
        if fault_flags & FaultFlag.UNDER_VOLTAGE and "Under-voltage fault (<12V)" not in errors:
            errors.append("Under-voltage fault (<12V)")
        if fault_flags & FaultFlag.OVER_TEMPERATURE and "Controller over-temperature fault" not in errors:
            errors.append("Controller over-temperature fault")
        if fault_flags & FaultFlag.OVER_INTEGRATION and "Over-integration fault" not in errors:
            errors.append("Over-integration fault")
        if fault_flags & FaultFlag.ENCODER_UNCALIBRATED and "Encoder uncalibrated" not in errors:
            errors.append("Encoder uncalibrated")
        
        warning_flags = WarningFlag(warning_code)
        if warning_flags & WarningFlag.MOTOR_OVER_TEMP and "Warning: Motor over-temperature (>125°C)" not in errors:
            errors.append("Warning: Motor over-temperature (>125°C)")
        
        if not errors:
            return "No errors"
        
        return "; ".join(errors)
    
    def handle_error(self, error_code: int) -> bool:
        """
        Handle motor error with appropriate recovery strategy
        
        Args:
            error_code: Error code bitmap
            
        Returns:
            True if recovery successful
        """
        fault_flags = FaultFlag(self.motor.status.fault_code)
        warning_flags = WarningFlag(self.motor.status.warning_code)
        
        if error_code == 0 and fault_flags == FaultFlag.NONE:
            if warning_flags != WarningFlag.NONE:
                self.logger.warning("Motor warning detected: %s",
                                     self.get_error_description(0, 0, warning_flags.value))
            return True
        
        description = self.get_error_description(error_code, fault_flags.value, warning_flags.value)
        self.logger.error(f"Motor error detected: {description}")
        
        error_flags = ErrorFlag(error_code)
        
        # Check critical errors
        if (error_flags & ErrorFlag.OVER_TEMPERATURE) or \
           (fault_flags & (FaultFlag.OVER_TEMPERATURE | FaultFlag.MOTOR_OVER_TEMP)):
            return self._recover_over_temperature()
        
        if error_flags & ErrorFlag.OVER_CURRENT:
            return self._recover_over_current()
        
        if (error_flags & ErrorFlag.UNDER_VOLTAGE) or (fault_flags & FaultFlag.UNDER_VOLTAGE):
            return self._recover_under_voltage()
        
        if (error_flags & ErrorFlag.ENCODER_FAULT) or (fault_flags & FaultFlag.ENCODER_UNCALIBRATED):
            return self._recover_encoder_error()
        
        # Default recovery: clear error and restart
        return self._default_recovery()
    
    def _recover_over_temperature(self) -> bool:
        """Recovery strategy for over-temperature"""
        self.logger.warning("Over-temperature detected. Motor should cool down before restart.")
        # Motor automatically stops on over-temperature
        # User must wait for cooling
        return False
    
    def _recover_over_current(self) -> bool:
        """Recovery strategy for over-current"""
        self.logger.warning("Over-current detected. Reducing current limit.")
        
        try:
            # Disable motor
            self.motor.disable_motor(clear_error=True)
            
            # Reduce current limit
            from .models import ParameterIndex
            current_limit = 5.0  # Reduced limit
            self.motor.set_parameter(ParameterIndex.LIMIT_CUR, current_limit)
            
            # Re-enable
            self.motor.enable_motor()
            
            self.logger.info("Current limit reduced to 5A. Recovery successful.")
            return True
        except Exception as e:
            self.logger.error(f"Over-current recovery failed: {e}")
            return False
    
    def _recover_under_voltage(self) -> bool:
        """Recovery strategy for under-voltage"""
        self.logger.error("Under-voltage detected. Check power supply!")
        # Motor automatically stops
        # User must fix power supply issue
        return False
    
    def _recover_encoder_error(self) -> bool:
        """Recovery strategy for encoder error"""
        self.logger.warning("Encoder fault detected. Attempting restart.")
        
        try:
            # Disable and re-enable motor
            self.motor.disable_motor(clear_error=True)
            time.sleep(0.5)  # Wait before restart
            self.motor.enable_motor()
            
            self.logger.info("Motor restarted. Check encoder connection.")
            return True
        except Exception as e:
            self.logger.error(f"Encoder error recovery failed: {e}")
            return False
    
    def _default_recovery(self) -> bool:
        """Default recovery strategy"""
        try:
            # Clear error and restart
            self.motor.disable_motor(clear_error=True)
            time.sleep(0.1)
            self.motor.enable_motor()
            
            self.logger.info("Error cleared and motor restarted.")
            return True
        except Exception as e:
            self.logger.error(f"Default recovery failed: {e}")
            return False


# ============================================================================
# ERROR LOGGER
# ============================================================================

class ErrorLogger:
    """
    Error logger for motor control
    
    Logs errors, warnings, and events to file and console.
    """
    
    def __init__(self, log_file: Optional[str] = None, console_level=logging.INFO):
        """
        Initialize error logger
        
        Args:
            log_file: Path to log file (optional)
            console_level: Console logging level
        """
        self.logger = logging.getLogger("RobStride")
        self.logger.setLevel(logging.DEBUG)
        
        # Remove existing handlers
        self.logger.handlers = []
        
        # Console handler
        console_handler = logging.StreamHandler()
        console_handler.setLevel(console_level)
        console_format = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            datefmt='%H:%M:%S'
        )
        console_handler.setFormatter(console_format)
        self.logger.addHandler(console_handler)
        
        # File handler (if specified)
        if log_file:
            file_handler = logging.FileHandler(log_file)
            file_handler.setLevel(logging.DEBUG)
            file_format = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
                datefmt='%Y-%m-%d %H:%M:%S'
            )
            file_handler.setFormatter(file_format)
            self.logger.addHandler(file_handler)
    
    def log_error(self, motor_id: int, error_code: int, description: str):
        """Log motor error"""
        self.logger.error(f"Motor {motor_id}: Error 0x{error_code:02X} - {description}")
    
    def log_warning(self, motor_id: int, message: str):
        """Log warning"""
        self.logger.warning(f"Motor {motor_id}: {message}")
    
    def log_info(self, motor_id: int, message: str):
        """Log info"""
        self.logger.info(f"Motor {motor_id}: {message}")
    
    def log_debug(self, motor_id: int, message: str):
        """Log debug"""
        self.logger.debug(f"Motor {motor_id}: {message}")
    
    def log_status(self, motor_id: int, status):
        """Log motor status"""
        self.logger.info(f"Motor {motor_id}: {status}")


# ============================================================================
# VALIDATION FUNCTIONS
# ============================================================================

def validate_can_id(can_id: int) -> bool:
    """
    Validate CAN ID
    
    Args:
        can_id: CAN ID to validate
        
    Returns:
        True if valid
        
    Raises:
        TypeError: If can_id is not an integer
        ValueError: If CAN ID is out of range
    """
    if not isinstance(can_id, int):
        raise TypeError(f"CAN ID must be an integer, got {type(can_id).__name__}")
    if not (0x00 <= can_id <= 0x7F):
        raise ValueError(f"CAN ID must be 0x00-0x7F, got {can_id}")
    return True


def validate_angle(angle: float, min_val: float = -12.57, max_val: float = 12.57) -> bool:
    """
    Validate angle value
    
    Args:
        angle: Angle in radians (±4π rad range)
        min_val: Minimum allowed value
        max_val: Maximum allowed value
        
    Returns:
        True if valid
        
    Raises:
        ValueError: If angle is out of range
        
    Warnings:
        Issues warning if angle is within 5% of limits
    """
    logger = logging.getLogger(__name__)
    
    # Warning threshold: within 5% of limit
    threshold = 0.05
    range_size = max_val - min_val
    warning_margin = range_size * threshold
    
    if angle > max_val - warning_margin or angle < min_val + warning_margin:
        logger.warning(f"Angle {angle:.3f} rad is near limit ({min_val}~{max_val})")
    
    if not (min_val <= angle <= max_val):
        raise ValueError(f"Angle must be {min_val}~{max_val} rad, got {angle}")
    return True


def validate_speed(speed: float, min_val: float = -44.0, max_val: float = 44.0) -> bool:
    """
    Validate speed value
    
    Args:
        speed: Speed in rad/s (~420 RPM max)
        min_val: Minimum allowed value
        max_val: Maximum allowed value
        
    Returns:
        True if valid
        
    Raises:
        ValueError: If speed is out of range
    """
    if not (min_val <= speed <= max_val):
        raise ValueError(f"Speed must be {min_val}~{max_val} rad/s, got {speed}")
    return True


def validate_torque(torque: float, min_val: float = -17.0, max_val: float = 17.0) -> bool:
    """
    Validate torque value
    
    Args:
        torque: Torque in Nm (continuous rating)
        min_val: Minimum allowed value
        max_val: Maximum allowed value
        
    Returns:
        True if valid
        
    Raises:
        ValueError: If torque is out of range
    """
    if not (min_val <= torque <= max_val):
        raise ValueError(f"Torque must be {min_val}~{max_val} Nm, got {torque}")
    return True


def validate_current(current: float, min_val: float = -23.0, max_val: float = 23.0) -> bool:
    """
    Validate current value
    
    Args:
        current: Current in A
        min_val: Minimum allowed value
        max_val: Maximum allowed value
        
    Returns:
        True if valid
        
    Raises:
        ValueError: If current is out of range
    """
    if not (min_val <= current <= max_val):
        raise ValueError(f"Current must be {min_val}~{max_val} A, got {current}")
    return True


def validate_kp(kp: float, min_val: float = 0.0, max_val: float = 500.0) -> bool:
    """
    Validate Kp gain value
    
    Args:
        kp: Kp gain
        min_val: Minimum allowed value
        max_val: Maximum allowed value
        
    Returns:
        True if valid
        
    Raises:
        ValueError: If Kp is out of range
    """
    if not (min_val <= kp <= max_val):
        raise ValueError(f"Kp must be {min_val}~{max_val}, got {kp}")
    return True


def validate_kd(kd: float, min_val: float = 0.0, max_val: float = 5.0) -> bool:
    """
    Validate Kd gain value
    
    Args:
        kd: Kd gain
        min_val: Minimum allowed value
        max_val: Maximum allowed value
        
    Returns:
        True if valid
        
    Raises:
        ValueError: If Kd is out of range
    """
    if not (min_val <= kd <= max_val):
        raise ValueError(f"Kd must be {min_val}~{max_val}, got {kd}")
    return True


def clamp(value: float, min_val: float, max_val: float) -> float:
    """
    Clamp value to range
    
    Args:
        value: Value to clamp
        min_val: Minimum value
        max_val: Maximum value
        
    Returns:
        Clamped value
    """
    return max(min_val, min(max_val, value))


# ============================================================================
# CONVERSION FUNCTIONS
# ============================================================================

def deg_to_rad(degrees: float) -> float:
    """
    Convert degrees to radians
    
    Args:
        degrees: Angle in degrees
        
    Returns:
        Angle in radians
    """
    return degrees * math.pi / 180.0


def rad_to_deg(radians: float) -> float:
    """
    Convert radians to degrees
    
    Args:
        radians: Angle in radians
        
    Returns:
        Angle in degrees
    """
    return radians * 180.0 / math.pi


def rpm_to_rad_s(rpm: float) -> float:
    """
    Convert RPM to rad/s
    
    Args:
        rpm: Speed in revolutions per minute
        
    Returns:
        Speed in rad/s
    """
    return rpm * 2.0 * math.pi / 60.0


def rad_s_to_rpm(rad_s: float) -> float:
    """
    Convert rad/s to RPM
    
    Args:
        rad_s: Speed in rad/s
        
    Returns:
        Speed in RPM
    """
    return rad_s * 60.0 / (2.0 * math.pi)


def nm_to_a(torque_nm: float, torque_constant: float = 0.335) -> float:
    """
    Convert torque (Nm) to current (A)
    
    Args:
        torque_nm: Torque in Nm
        torque_constant: Motor torque constant in Nm/A (default: 0.335 for RS02)
        
    Returns:
        Current in A
    """
    return torque_nm / torque_constant


def a_to_nm(current_a: float, torque_constant: float = 0.335) -> float:
    """
    Convert current (A) to torque (Nm)
    
    Args:
        current_a: Current in A
        torque_constant: Motor torque constant in Nm/A (default: 0.335 for RS02)
        
    Returns:
        Torque in Nm
    """
    return current_a * torque_constant


__all__ = [
    # Exceptions
    'MotorException', 'CANException', 'ParameterException', 'ProtocolException',
    # Error handling
    'ErrorHandler', 'ErrorLogger',
    # Validation
    'validate_can_id', 'validate_angle', 'validate_speed', 'validate_torque',
    'validate_current', 'validate_kp', 'validate_kd', 'clamp',
    # Conversion
    'deg_to_rad', 'rad_to_deg', 'rpm_to_rad_s', 'rad_s_to_rpm',
    'nm_to_a', 'a_to_nm',
]
