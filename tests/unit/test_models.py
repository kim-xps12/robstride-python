"""
Unit tests for data models (MotorStatus, ParameterData, enums, etc.)
"""

import pytest
from robstride.models import (
    MotorStatus, ParameterData, ControlMode, ProtocolMode, ErrorFlag,
    MotorState, ParameterIndex, validate_parameter, get_parameter_name,
    is_readable, is_writable, MotorPattern, CommunicationType
)


@pytest.mark.unit
class TestMotorStatus:
    """Tests for MotorStatus data class"""
    
    def test_motor_status_initialization(self):
        """TC-U-M-001: MotorStatus initializes with default values"""
        status = MotorStatus()
        
        assert status.device_id is None
        assert status.device_uid is None
        assert status.angle == 0.0
        assert status.speed == 0.0
        assert status.torque == 0.0
        assert status.temperature == 0.0
        assert status.pattern == 0
        assert status.error_code == 0
        assert status.warning_code == 0
    
    def test_motor_status_has_error_property(self):
        """TC-U-M-002: has_error property correctly detects errors"""
        status = MotorStatus()
        
        # No error
        status.error_code = 0
        assert status.has_error is False
        
        # With error
        status.error_code = ErrorFlag.OVER_TEMPERATURE
        assert status.has_error is True
    
    def test_motor_status_is_running_property(self):
        """TC-U-M-003: is_running property based on pattern"""
        status = MotorStatus()
        
        # Not running
        status.pattern = 0
        assert status.is_running is False
        
        # Running
        status.pattern = 1
        assert status.is_running is True
        
        status.pattern = 3
        assert status.is_running is True
    
    def test_motor_status_get_error_names(self):
        """TC-U-M-004: get_error_names returns correct error list"""
        status = MotorStatus()
        
        # No errors
        status.error_code = 0
        error_names = status.get_error_names()
        assert error_names == []
        
        # Single error
        status.error_code = ErrorFlag.OVER_TEMPERATURE
        error_names = status.get_error_names()
        assert len(error_names) == 1
        assert any('TEMPERATURE' in name.upper() or 'TEMP' in name.upper() for name in error_names)
        
        # Multiple errors
        status.error_code = ErrorFlag.OVER_TEMPERATURE | ErrorFlag.OVER_CURRENT
        error_names = status.get_error_names()
        assert len(error_names) == 2
    
    def test_motor_status_str_representation(self):
        """TC-U-M-005: String representation is informative"""
        status = MotorStatus()
        status.angle = 1.57
        status.speed = 10.0
        status.torque = 5.0
        status.temperature = 45.0
        
        status_str = str(status)
        
        assert '1.57' in status_str or '1.570' in status_str
        assert '10.0' in status_str or '10.000' in status_str
        assert '5.0' in status_str or '5.000' in status_str
        assert '45.0' in status_str or '45' in status_str


@pytest.mark.unit
class TestParameterData:
    """Tests for ParameterData data class"""
    
    def test_parameter_data_initialization(self):
        """TC-U-P-001: ParameterData initializes with default values"""
        data = ParameterData()
        
        assert data.run_mode == 0.0
        assert data.iq_ref == 0.0
        assert data.spd_ref == 0.0
        assert data.limit_torque == 12.0
        assert data.limit_spd == 44.0
        assert data.limit_cur == 23.0
    
    def test_parameter_data_attribute_assignment(self):
        """TC-U-P-002: ParameterData allows attribute modification"""
        data = ParameterData()
        
        data.iq_ref = 5.0
        data.spd_ref = 20.0
        data.mech_pos = 3.14
        
        assert data.iq_ref == 5.0
        assert data.spd_ref == 20.0
        assert data.mech_pos == 3.14


@pytest.mark.unit
class TestEnumerations:
    """Tests for enum classes"""
    
    def test_control_mode_values(self):
        """TC-U-E-001: ControlMode enum has correct values per RS02 specification"""
        assert ControlMode.MOTION_CONTROL == 0
        assert ControlMode.POSITION_PP == 1
        assert ControlMode.SPEED == 2
        assert ControlMode.CURRENT == 3
        assert ControlMode.SET_ZERO == 4
        assert ControlMode.POSITION_CSP == 5
    
    def test_protocol_mode_values(self):
        """TC-U-E-002: ProtocolMode enum has correct values"""
        assert ProtocolMode.PRIVATE == 0x00
        assert ProtocolMode.CANOPEN == 0x01
        assert ProtocolMode.MIT == 0x02
    
    def test_error_flag_combinations(self):
        """TC-U-E-003: ErrorFlag bitwise operations work correctly"""
        # Single flags
        assert ErrorFlag.OVER_TEMPERATURE != 0
        assert ErrorFlag.OVER_CURRENT != 0
        
        # Combination
        combined = ErrorFlag.OVER_TEMPERATURE | ErrorFlag.OVER_CURRENT
        
        assert combined & ErrorFlag.OVER_TEMPERATURE
        assert combined & ErrorFlag.OVER_CURRENT
        assert not (combined & ErrorFlag.UNDER_VOLTAGE)
    
    def test_motor_state_values(self):
        """TC-U-E-004: MotorState enum has correct values"""
        assert MotorState.UNINITIALIZED == 0
        assert MotorState.DISABLED == 1
        assert MotorState.ENABLED == 2
        assert MotorState.RUNNING == 3
        assert MotorState.FAULT == 4
    
    def test_motor_pattern_values(self):
        """TC-U-E-005: MotorPattern enum has correct values"""
        assert MotorPattern.TORQUE == 0
        assert MotorPattern.POSITION == 1
        assert MotorPattern.SPEED == 2
        assert MotorPattern.RUNNING == 3


@pytest.mark.unit
class TestParameterMapping:
    """Tests for parameter validation and mapping"""
    
    def test_validate_parameter_valid_values(self):
        """TC-U-PM-001: validate_parameter accepts valid values"""
        # Position reference (no strict limit)
        valid, msg = validate_parameter(ParameterIndex.LOC_REF, 10.0)
        assert valid is True
        
        # Current reference (limited)
        valid, msg = validate_parameter(ParameterIndex.IQ_REF, 10.0)
        assert valid is True
        
        # Speed reference
        valid, msg = validate_parameter(ParameterIndex.SPD_REF, 20.0)
        assert valid is True
    
    def test_validate_parameter_out_of_range(self):
        """TC-U-PM-002: validate_parameter rejects out-of-range values"""
        # Current too high
        valid, msg = validate_parameter(ParameterIndex.IQ_REF, 25.0)
        assert valid is False
        assert "out of range" in msg.lower()
        
        # Speed too high
        valid, msg = validate_parameter(ParameterIndex.SPD_REF, 50.0)
        assert valid is False
    
    def test_validate_parameter_read_only(self):
        """TC-U-PM-003: validate_parameter rejects read-only parameters"""
        # Mechanical position is read-only
        valid, msg = validate_parameter(ParameterIndex.MECH_POS, 10.0)
        assert valid is False
        assert "read-only" in msg.lower() or "read only" in msg.lower()
    
    def test_validate_parameter_unknown_index(self):
        """TC-U-PM-004: validate_parameter handles unknown parameters"""
        valid, msg = validate_parameter(0x9999, 10.0)
        assert valid is False
        assert "unknown" in msg.lower()
    
    def test_get_parameter_name(self):
        """TC-U-PM-005: get_parameter_name returns correct names"""
        assert get_parameter_name(ParameterIndex.RUN_MODE) == 'run_mode'
        assert get_parameter_name(ParameterIndex.IQ_REF) == 'iq_ref'
        assert get_parameter_name(ParameterIndex.MECH_POS) == 'mech_pos'
        
        # Unknown parameter
        name = get_parameter_name(0x9999)
        assert 'UNKNOWN' in name or '9999' in name
    
    def test_is_readable_writable(self):
        """TC-U-PM-006: Parameter access flags are correct"""
        # Read-write parameter
        assert is_readable(ParameterIndex.IQ_REF) is True
        assert is_writable(ParameterIndex.IQ_REF) is True
        
        # Read-only parameter
        assert is_readable(ParameterIndex.MECH_POS) is True
        assert is_writable(ParameterIndex.MECH_POS) is False
