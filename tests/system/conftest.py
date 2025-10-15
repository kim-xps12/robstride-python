"""
System test fixtures (hardware required)
"""

import pytest


def pytest_configure(config):
    """Add custom markers"""
    config.addinivalue_line(
        "markers", "hardware: tests requiring real motor hardware"
    )


@pytest.fixture(scope="module")
def safety_check():
    """
    Safety check before hardware tests
    
    Ensures:
    - Emergency stop is accessible
    - Power supply is within limits
    - Motor is properly mounted
    """
    print("\n" + "="*60)
    print("HARDWARE TEST SAFETY CHECK")
    print("="*60)
    print("Before proceeding, verify:")
    print("  1. Emergency stop button is within reach")
    print("  2. Motor is securely mounted")
    print("  3. Power supply is 24V (12-50V range)")
    print("  4. Area is clear of obstacles")
    print("  5. You are ready to stop test if needed")
    print("="*60)
    
    response = input("Continue with hardware tests? (yes/no): ")
    
    if response.lower() != 'yes':
        pytest.skip("Hardware tests skipped by user")
    
    yield
    
    print("\nHardware tests completed. Safe to power down.")
