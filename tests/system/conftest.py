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
    
    # Allow automation via env var: set RS_HARDWARE_AUTO_CONFIRM=yes to auto-accept
    import os

    auto = os.environ.get("RS_HARDWARE_AUTO_CONFIRM", "").lower()
    if auto == "yes":
        response = "yes"
    else:
        # If stdin is not available (pytest captures output) avoid raising OSError
        try:
            response = input("Continue with hardware tests? (yes/no): ")
        except Exception:
            # Default to skipping hardware tests in non-interactive environments
            pytest.skip("Hardware tests skipped (non-interactive environment)")

    if response.lower() != "yes":
        pytest.skip("Hardware tests skipped by user")
    
    yield
    
    print("\nHardware tests completed. Safe to power down.")
