"""
Unit tests for validation functions in utils.py
"""

import pytest
import math

from robstride.utils import (
    validate_can_id,
    validate_angle,
    validate_speed,
    validate_torque,
    validate_current,
    validate_kp,
    validate_kd,
    clamp
)


@pytest.mark.unit
class TestCANIDValidation:
    """Tests for CAN ID validation"""
    
    @pytest.mark.parametrize("can_id", [0x00, 0x01, 0x10, 0x7F])
    def test_valid_can_ids(self, can_id):
        """TC-U-V-001: Valid CAN IDs pass validation"""
        assert validate_can_id(can_id) is True
    
    @pytest.mark.parametrize("can_id", [-1, 0x80, 0xFF, 256])
    def test_invalid_can_id_range(self, can_id):
        """TC-U-V-002: Out-of-range CAN IDs raise ValueError"""
        with pytest.raises(ValueError, match="CAN ID"):
            validate_can_id(can_id)
    
    def test_invalid_can_id_type(self):
        """TC-U-V-003: Non-integer CAN ID raises TypeError"""
        with pytest.raises(TypeError):
            validate_can_id("not_an_int")  # type: ignore
        
        with pytest.raises(TypeError):
            validate_can_id(1.5)  # type: ignore


@pytest.mark.unit
class TestAngleValidation:
    """Tests for angle validation"""
    
    @pytest.mark.parametrize("angle", [-12.57, -6.28, 0.0, 3.14, 12.57])
    def test_valid_angles(self, angle):
        """TC-U-V-010: Valid angles pass validation"""
        assert validate_angle(angle) is True
    
    @pytest.mark.parametrize("angle", [-12.58, -20.0, 12.58, 20.0])
    def test_invalid_angle_range(self, angle):
        """TC-U-V-011: Out-of-range angles raise ValueError"""
        with pytest.raises(ValueError):
            validate_angle(angle)
    
    def test_angle_nan(self):
        """TC-U-V-012: NaN angle raises ValueError"""
        with pytest.raises(ValueError):
            validate_angle(float('nan'))
    
    def test_angle_inf(self):
        """TC-U-V-013: Infinite angle raises ValueError"""
        with pytest.raises(ValueError):
            validate_angle(float('inf'))
        
        with pytest.raises(ValueError):
            validate_angle(float('-inf'))
    
    def test_angle_type_error(self):
        """TC-U-V-014: Non-numeric angle raises TypeError"""
        with pytest.raises(TypeError):
            validate_angle("not_a_number")
    
    def test_angle_warning_threshold(self, caplog):
        """TC-U-V-015: Near-limit angles generate warnings"""
        import logging
        caplog.set_level(logging.WARNING)
        
        # Angle at 95% of limit should warn
        validate_angle(11.9)  # ~0.95 * 12.57
        
        # Check if warning was logged
        assert any("near limit" in record.message.lower() or "warning" in record.message.lower() 
                   for record in caplog.records)


@pytest.mark.unit
class TestSpeedValidation:
    """Tests for speed validation"""
    
    @pytest.mark.parametrize("speed", [-44.0, -20.0, 0.0, 20.0, 44.0])
    def test_valid_speeds(self, speed):
        """TC-U-V-020: Valid speeds pass validation"""
        assert validate_speed(speed) is True
    
    @pytest.mark.parametrize("speed", [-45.0, -100.0, 45.0, 100.0])
    def test_invalid_speed_range(self, speed):
        """TC-U-V-021: Out-of-range speeds raise ValueError"""
        with pytest.raises(ValueError):
            validate_speed(speed)
    
    def test_speed_nan_inf(self):
        """TC-U-V-022: NaN/Inf speed raises ValueError"""
        with pytest.raises(ValueError):
            validate_speed(float('nan'))
        
        with pytest.raises(ValueError):
            validate_speed(float('inf'))


@pytest.mark.unit
class TestTorqueValidation:
    """Tests for torque validation"""
    
    @pytest.mark.parametrize("torque", [-17.0, -10.0, 0.0, 10.0, 17.0])
    def test_valid_torques(self, torque):
        """TC-U-V-030: Valid torques pass validation"""
        assert validate_torque(torque) is True
    
    @pytest.mark.parametrize("torque", [-18.0, -100.0, 18.0, 100.0])
    def test_invalid_torque_range(self, torque):
        """TC-U-V-031: Out-of-range torques raise ValueError"""
        with pytest.raises(ValueError):
            validate_torque(torque)


@pytest.mark.unit
class TestCurrentValidation:
    """Tests for current validation"""
    
    @pytest.mark.parametrize("current", [-23.0, -10.0, 0.0, 10.0, 23.0])
    def test_valid_currents(self, current):
        """TC-U-V-040: Valid currents pass validation"""
        assert validate_current(current) is True
    
    @pytest.mark.parametrize("current", [-24.0, -100.0, 24.0, 100.0])
    def test_invalid_current_range(self, current):
        """TC-U-V-041: Out-of-range currents raise ValueError"""
        with pytest.raises(ValueError):
            validate_current(current)


@pytest.mark.unit
class TestGainValidation:
    """Tests for Kp/Kd gain validation"""
    
    @pytest.mark.parametrize("kp", [0.0, 50.0, 250.0, 500.0])
    def test_valid_kp(self, kp):
        """TC-U-V-050: Valid Kp values pass validation"""
        assert validate_kp(kp) is True
    
    def test_invalid_kp(self):
        """TC-U-V-051: Out-of-range Kp raises ValueError"""
        with pytest.raises(ValueError):
            validate_kp(-1.0)
        
        with pytest.raises(ValueError):
            validate_kp(501.0)
    
    @pytest.mark.parametrize("kd", [0.0, 1.0, 2.5, 5.0])
    def test_valid_kd(self, kd):
        """TC-U-V-052: Valid Kd values pass validation"""
        assert validate_kd(kd) is True
    
    def test_invalid_kd(self):
        """TC-U-V-053: Out-of-range Kd raises ValueError"""
        with pytest.raises(ValueError):
            validate_kd(-0.1)
        
        with pytest.raises(ValueError):
            validate_kd(5.1)


@pytest.mark.unit
class TestClampFunction:
    """Tests for clamp utility function"""
    
    def test_clamp_within_range(self):
        """TC-U-V-060: Clamp returns value if within range"""
        assert clamp(5.0, 0.0, 10.0) == 5.0
        assert clamp(0.0, -10.0, 10.0) == 0.0
    
    def test_clamp_below_min(self):
        """TC-U-V-061: Clamp returns min if value below"""
        assert clamp(-5.0, 0.0, 10.0) == 0.0
        assert clamp(-100.0, -10.0, 10.0) == -10.0
    
    def test_clamp_above_max(self):
        """TC-U-V-062: Clamp returns max if value above"""
        assert clamp(15.0, 0.0, 10.0) == 10.0
        assert clamp(100.0, -10.0, 10.0) == 10.0
