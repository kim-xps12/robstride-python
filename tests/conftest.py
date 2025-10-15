"""
Common test fixtures for RobStride motor control library tests
"""

import pytest
import can
from unittest.mock import Mock, MagicMock, patch
from typing import Generator

from robstride import RobStrideMotor
from robstride.models import MotorStatus, ParameterData, ControlMode, ProtocolMode
from robstride.protocol.private import PrivateProtocolHandler
from robstride.protocol.mit import MITProtocolHandler


# ============================================================================
# MOCK CAN BUS FIXTURES
# ============================================================================

@pytest.fixture
def mock_can_bus() -> Mock:
    """
    Mock CAN bus for unit tests
    
    Returns:
        Mock can.Bus instance with send/recv capabilities
    """
    mock_bus = Mock(spec=can.Bus)
    mock_bus.send = Mock(return_value=None)
    mock_bus.recv = Mock(return_value=None)
    mock_bus.shutdown = Mock()
    return mock_bus


@pytest.fixture
def mock_can_message():
    """
    Factory for creating mock CAN messages
    
    Usage:
        msg = mock_can_message(arbitration_id=0x123, data=[1,2,3,4,5,6,7,8])
    """
    def _create_message(
        arbitration_id: int = 0x01000100,
        data: list = None,
        is_extended_id: bool = True,
        dlc: int = 8
    ) -> can.Message:
        if data is None:
            data = [0] * 8
        return can.Message(
            arbitration_id=arbitration_id,
            data=bytes(data),
            is_extended_id=is_extended_id,
            dlc=dlc
        )
    return _create_message


# ============================================================================
# VIRTUAL CAN BUS FIXTURES (for integration tests)
# ============================================================================

@pytest.fixture(scope="session")
def vcan_interface() -> str:
    """
    Virtual CAN interface name for integration tests
    
    Returns:
        Interface name (e.g., 'vcan0')
    """
    return 'vcan0'


@pytest.fixture
def vcan_bus(vcan_interface: str) -> Generator[can.Bus, None, None]:
    """
    Real virtual CAN bus for integration tests
    
    Yields:
        can.Bus instance connected to vcan0
    """
    try:
        bus = can.interface.Bus(
            channel=vcan_interface,
            bustype='socketcan',
            bitrate=1000000
        )
        yield bus
    except Exception as e:
        pytest.skip(f"Virtual CAN not available: {e}")
    finally:
        try:
            bus.shutdown()
        except:
            pass


# ============================================================================
# MOTOR INSTANCE FIXTURES
# ============================================================================

@pytest.fixture
def mock_motor(mock_can_bus: Mock) -> RobStrideMotor:
    """
    Mock motor instance for unit tests
    
    Uses mocked CAN bus to avoid actual hardware communication.
    Auto-enable is disabled to control initialization in tests.
    """
    with patch('robstride.motor.can.interface.Bus', return_value=mock_can_bus):
        motor = RobStrideMotor(
            can_id=1,
            can_interface='test_can',
            protocol=ProtocolMode.PRIVATE,
            auto_enable=False
        )
        # Stop background thread for testing
        motor._running = False
        yield motor


@pytest.fixture
def vcan_motor(vcan_interface: str) -> Generator[RobStrideMotor, None, None]:
    """
    Motor instance connected to virtual CAN for integration tests
    """
    try:
        motor = RobStrideMotor(
            can_id=1,
            can_interface=vcan_interface,
            protocol=ProtocolMode.PRIVATE,
            auto_enable=False
        )
        yield motor
    finally:
        motor._running = False
        if hasattr(motor, 'can_bus'):
            try:
                motor.can_bus.shutdown()
            except:
                pass


# ============================================================================
# HARDWARE MOTOR FIXTURE (for system tests)
# ============================================================================

@pytest.fixture(scope="module")
def hardware_motor() -> Generator[RobStrideMotor, None, None]:
    """
    Real motor instance for hardware tests
    
    Requires:
        - Real motor connected to can0
        - Motor powered and ready
    
    Usage:
        @pytest.mark.hardware
        def test_with_real_motor(hardware_motor):
            hardware_motor.enable_motor()
            ...
    """
    motor = RobStrideMotor(
        can_id=1,
        can_interface='can0',
        protocol=ProtocolMode.PRIVATE,
        auto_enable=False
    )
    
    try:
        yield motor
    finally:
        # Safe cleanup
        try:
            motor.disable_motor(clear_error=True)
        except:
            pass
        motor._running = False
        if hasattr(motor, 'can_bus'):
            try:
                motor.can_bus.shutdown()
            except:
                pass


# ============================================================================
# DATA MODEL FIXTURES
# ============================================================================

@pytest.fixture
def motor_status() -> MotorStatus:
    """Empty MotorStatus instance"""
    return MotorStatus()


@pytest.fixture
def parameter_data() -> ParameterData:
    """Empty ParameterData instance"""
    return ParameterData()


@pytest.fixture
def sample_status() -> MotorStatus:
    """MotorStatus with sample data"""
    status = MotorStatus()
    status.angle = 1.57  # 90 degrees
    status.speed = 10.0
    status.torque = 5.0
    status.temperature = 45.0
    status.pattern = 1
    status.error_code = 0
    return status


# ============================================================================
# PROTOCOL HANDLER FIXTURES
# ============================================================================

@pytest.fixture
def private_handler(mock_can_bus: Mock) -> PrivateProtocolHandler:
    """Private protocol handler with mock bus"""
    return PrivateProtocolHandler(motor_id=1, can_bus=mock_can_bus, master_id=0xFD)


@pytest.fixture
def mit_handler(mock_can_bus: Mock) -> MITProtocolHandler:
    """MIT protocol handler with mock bus"""
    return MITProtocolHandler(motor_id=1, can_bus=mock_can_bus)


# ============================================================================
# PARAMETRIZE HELPERS
# ============================================================================

@pytest.fixture
def valid_can_ids():
    """Valid CAN ID values for parametrized tests"""
    return [0x01, 0x10, 0x20, 0x7F]


@pytest.fixture
def invalid_can_ids():
    """Invalid CAN ID values for parametrized tests"""
    return [-1, 0x80, 0xFF, 256, "not_an_int"]


@pytest.fixture
def angle_test_values():
    """Angle test values: (value, expected_valid)"""
    return [
        (-12.57, True),   # Min
        (0.0, True),      # Zero
        (12.57, True),    # Max
        (1.57, True),     # π/2
        (-12.58, False),  # Below min
        (12.58, False),   # Above max
        (float('nan'), False),  # NaN
        (float('inf'), False),  # Inf
    ]


# ============================================================================
# TIMEOUT AND PERFORMANCE FIXTURES
# ============================================================================

@pytest.fixture
def benchmark_timer():
    """
    Simple timer for performance measurements
    
    Usage:
        with benchmark_timer() as t:
            # do something
        assert t.elapsed < 0.01  # 10ms
    """
    import time
    from contextlib import contextmanager
    
    @contextmanager
    def timer():
        class Timer:
            def __init__(self):
                self.start = time.perf_counter()
                self.elapsed = 0
        
        t = Timer()
        yield t
        t.elapsed = time.perf_counter() - t.start
    
    return timer
