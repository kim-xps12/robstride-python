"""
Integration test fixtures
"""

import pytest
import can
import time
import threading


@pytest.fixture(scope="module")
def vcan_setup():
    """
    Ensure vcan0 is set up
    
    Note: This assumes vcan0 is already configured.
    In CI, this should be done in workflow setup.
    """
    # Verification only
    try:
        bus = can.interface.Bus(channel='vcan0', bustype='socketcan')
        bus.shutdown()
        return True
    except Exception as e:
        pytest.skip(f"vcan0 not available: {e}")


@pytest.fixture
def mock_motor_response_handler(vcan_bus):
    """
    Mock motor that responds to commands on vcan
    
    Simulates motor responses for integration testing.
    """
    running = threading.Event()
    running.set()
    
    def responder():
        while running.is_set():
            try:
                msg = vcan_bus.recv(timeout=0.1)
                if msg and msg.is_extended_id:
                    # Parse and respond based on command type
                    # (Implementation would generate appropriate responses)
                    pass
            except:
                pass
    
    thread = threading.Thread(target=responder, daemon=True)
    thread.start()
    
    yield
    
    running.clear()
    thread.join(timeout=1.0)
