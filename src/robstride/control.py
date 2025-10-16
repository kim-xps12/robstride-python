"""
Control strategies for RobStride motor

Consolidates:
- Position control (PP and CSP modes)
- Speed control
- Current/Torque control

Based on specifications:
- doc/architecture/02_api_specification.md
- doc/architecture/06_state_machine_design.md
"""

from typing import Optional
from .models import ControlMode, ParameterIndex


class PositionController:
    """Position control strategies"""
    
    def __init__(self, motor):
        """
        Initialize position controller
        
        Args:
            motor: RobStrideMotor instance
        """
        self.motor = motor
    
    def set_pp_position(self, target_angle: float, target_speed: float = 5.0):
        """
        Set position using PP (Point-to-Point) mode
        
        Per RS02 specification: run_mode=1 is Position Control Mode (PP)
        
        Args:
            target_angle: Target angle in radians
            target_speed: Target speed in rad/s
        """
        # Set control mode to PP (run_mode=1)
        self.motor.set_parameter(ParameterIndex.RUN_MODE, ControlMode.POSITION_PP, value_mode='j')
        
        # Set speed limit (VEL_MAX for PP mode)
        self.motor.set_parameter(ParameterIndex.VEL_MAX, target_speed)
        
        # Set target position
        self.motor.set_parameter(ParameterIndex.LOC_REF, target_angle)
    
    def set_csp_position(self, target_angle: float, speed_limit: float = 10.0):
        """
        Set position using CSP (Cyclic Synchronous Position) mode
        
        Per RS02 specification: run_mode=5 is Position Mode (CSP)
        
        Args:
            target_angle: Target angle in radians
            speed_limit: Speed limit in rad/s
        """
        # Set control mode to CSP (run_mode=5)
        self.motor.set_parameter(ParameterIndex.RUN_MODE, ControlMode.POSITION_CSP, value_mode='j')
        
        # Set speed limit
        self.motor.set_parameter(ParameterIndex.LIMIT_SPD, speed_limit)
        
        # Set target position
        self.motor.set_parameter(ParameterIndex.LOC_REF, target_angle)


class SpeedController:
    """Speed control strategies"""
    
    def __init__(self, motor):
        """
        Initialize speed controller
        
        Args:
            motor: RobStrideMotor instance
        """
        self.motor = motor
    
    def set_speed(self, target_speed: float, current_limit: float = 10.0):
        """
        Set target speed
        
        Per RS02 specification: run_mode=2 is Speed Control Mode
        
        Args:
            target_speed: Target speed in rad/s (-44 to 44)
            current_limit: Current limit in A (0 to 23)
        """
        # Set control mode to speed (run_mode=2)
        self.motor.set_parameter(ParameterIndex.RUN_MODE, ControlMode.SPEED, value_mode='j')
        
        # Set current limit
        self.motor.set_parameter(ParameterIndex.LIMIT_CUR, current_limit)
        
        # Set target speed
        self.motor.set_parameter(ParameterIndex.SPD_REF, target_speed)


class CurrentController:
    """Current (torque) control strategies"""
    
    def __init__(self, motor):
        """
        Initialize current controller
        
        Args:
            motor: RobStrideMotor instance
        """
        self.motor = motor
    
    def set_current(self, target_current: float):
        """
        Set target current (torque)
        
        Per RS02 specification: run_mode=3 is Current Control Mode
        
        Args:
            target_current: Target current in A (-23 to 23)
        """
        # Set control mode to current (run_mode=3)
        self.motor.set_parameter(ParameterIndex.RUN_MODE, ControlMode.CURRENT, value_mode='j')
        
        # Set target current
        self.motor.set_parameter(ParameterIndex.IQ_REF, target_current)
    
    def set_torque(self, target_torque: float, torque_constant: float = 0.335):
        """
        Set target torque (converts to current)
        
        Args:
            target_torque: Target torque in Nm
            torque_constant: Motor torque constant (Nm/A)
        """
        # Convert torque to current
        target_current = target_torque / torque_constant
        
        # Set current
        self.set_current(target_current)


__all__ = ['PositionController', 'SpeedController', 'CurrentController']
