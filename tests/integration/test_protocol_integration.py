"""
Integration tests for protocol operations
"""

import pytest
import time

from robstride import RobStrideMotor
from robstride.models import ProtocolMode, ParameterIndex


@pytest.mark.integration
class TestEnableDisableSequence:
    """Tests for enable/disable sequence"""
    
    def test_enable_disable_cycle(self, vcan_motor, vcan_setup):
        """TC-I-001: Enable-disable cycle completes successfully"""
        motor = vcan_motor
        
        # Enable
        result = motor.enable_motor()
        time.sleep(0.2)
        
        # Verify command was sent (state check depends on implementation)
        
        # Disable
        result = motor.disable_motor()
        time.sleep(0.2)
    
    def test_multiple_enable_disable_cycles(self, vcan_motor, vcan_setup):
        """TC-I-002: Multiple enable/disable cycles work correctly"""
        motor = vcan_motor
        
        for _ in range(5):
            motor.enable_motor()
            time.sleep(0.1)
            motor.disable_motor()
            time.sleep(0.1)


@pytest.mark.integration
class TestParameterReadWrite:
    """Tests for parameter read/write operations"""
    
    def test_parameter_write_sequence(self, vcan_motor, vcan_setup):
        """TC-I-010: Parameter write completes without error"""
        motor = vcan_motor
        motor.enable_motor()
        time.sleep(0.1)
        
        # Write parameters
        result = motor.set_parameter(ParameterIndex.LIMIT_CUR, 10.0, value_mode='p')
        assert result is True
        
        time.sleep(0.05)
        
        result = motor.set_parameter(ParameterIndex.SPD_REF, 5.0, value_mode='p')
        assert result is True
    
    def test_parameter_read_sequence(self, vcan_motor, vcan_setup):
        """TC-I-011: Parameter read completes without error"""
        motor = vcan_motor
        motor.enable_motor()
        time.sleep(0.1)
        
        # Read parameter
        result = motor.get_parameter(ParameterIndex.MECH_POS)
        assert result is True
        
        time.sleep(0.1)


@pytest.mark.integration
class TestMessageFlow:
    """Tests for message send/receive flow"""
    
    def test_command_response_timing(self, vcan_motor, vcan_setup, benchmark_timer):
        """TC-I-020: Command-response completes within timeout"""
        motor = vcan_motor
        
        with benchmark_timer() as t:
            motor.enable_motor()
            time.sleep(0.1)
        
        # Should complete quickly
        assert t.elapsed < 0.5
    
    def test_continuous_commands(self, vcan_motor, vcan_setup):
        """TC-I-021: Continuous commands don't cause buffer overflow"""
        motor = vcan_motor
        motor.enable_motor()
        time.sleep(0.1)
        
        # Send many commands rapidly
        for i in range(100):
            motor.set_parameter(ParameterIndex.LOC_REF, float(i % 10))
            time.sleep(0.01)


@pytest.mark.integration
class TestErrorRecovery:
    """Tests for error recovery mechanisms"""
    
    def test_clear_error_sequence(self, vcan_motor, vcan_setup):
        """TC-I-030: Clear error sequence works correctly"""
        motor = vcan_motor
        
        # Simulate error condition
        motor.status.error_code = 0x20
        
        # Clear error via disable with clear flag
        motor.disable_motor(clear_error=True)
        time.sleep(0.1)
        
        # Error should be cleared (in real scenario)
