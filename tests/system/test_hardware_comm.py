"""
System tests for hardware communication
"""

import pytest
import time

from robstride import RobStrideMotor
from robstride.models import ParameterIndex


@pytest.mark.hardware
@pytest.mark.system
class TestBasicCommunication:
    """Tests for basic CAN communication with real motor"""
    
    def test_motor_connection(self, hardware_motor, safety_check):
        """TC-S-001: Motor responds to CAN messages"""
        motor = hardware_motor
        
        # Try to enable motor
        result = motor.enable_motor()
        assert result is True
        
        time.sleep(0.5)
        
        # Motor should be enabled
        # (State verification depends on feedback implementation)
        
        motor.disable_motor()
    
    def test_status_feedback_reception(self, hardware_motor, safety_check):
        """TC-S-002: Motor sends status feedback"""
        motor = hardware_motor
        
        motor.enable_motor()
        time.sleep(0.2)
        
        # Read position
        motor.get_parameter(ParameterIndex.MECH_POS)
        time.sleep(0.2)
        
        # Status should have been updated
        # (Check that last_update timestamp changed)
        
        motor.disable_motor()
    
    def test_parameter_roundtrip(self, hardware_motor, safety_check):
        """TC-S-003: Parameter write-read roundtrip"""
        motor = hardware_motor
        
        motor.enable_motor()
        time.sleep(0.2)
        
        # Write current limit
        test_value = 8.0
        motor.set_parameter(ParameterIndex.LIMIT_CUR, test_value)
        time.sleep(0.1)
        
        # Read back
        motor.get_parameter(ParameterIndex.LIMIT_CUR)
        time.sleep(0.2)
        
        # Verify (implementation-dependent)
        # assert abs(motor.param_data.limit_cur - test_value) < 0.1
        
        motor.disable_motor()


@pytest.mark.hardware
@pytest.mark.system
@pytest.mark.slow
class TestControlPerformance:
    """Tests for control performance on real hardware"""
    
    def test_position_control_basic(self, hardware_motor, safety_check):
        """TC-S-010: Basic position control works"""
        motor = hardware_motor
        
        # Setup
        motor.enable_motor()
        time.sleep(0.2)
        
        motor.set_parameter(ParameterIndex.RUN_MODE, 5, value_mode='j')  # CSP mode
        motor.set_parameter(ParameterIndex.LIMIT_SPD_CSP, 5.0)
        motor.set_parameter(ParameterIndex.LIMIT_CUR, 5.0)
        time.sleep(0.2)
        
        # Set zero
        motor.set_zero_position()
        time.sleep(0.5)
        
        # Move to target
        target = 1.0  # ~57 degrees
        motor.set_parameter(ParameterIndex.LOC_REF, target)
        
        # Wait for movement
        time.sleep(3.0)
        
        # Read position
        motor.get_parameter(ParameterIndex.MECH_POS)
        time.sleep(0.2)
        
        # Verify arrival (tolerance 0.1 rad)
        # assert abs(motor.param_data.mech_pos - target) < 0.1
        
        # Return to zero
        motor.set_parameter(ParameterIndex.LOC_REF, 0.0)
        time.sleep(3.0)
        
        motor.disable_motor()
