#!/usr/bin/env python3
"""
Move motor to 0 rad using robstride library (Private protocol, PP mode)

Usage: run from repository root:
    uv run python src/examples/motion_control_zero.py

This script will:
 - create RobStrideMotor(can_id=0x7F)
 - enable motor
 - enable auto-report
 - set zero position and send PP position=0.0
 - monitor a few auto-reports
 - always disable motor and close CAN on exit
"""

import time
import math
from robstride import RobStrideMotor, ProtocolMode


def main():
    motor = None
    try:
        motor = RobStrideMotor(can_id=0x7F, can_interface='can0', protocol=ProtocolMode.PRIVATE, auto_enable=False)
        print(f"Motor initialized: {motor}")

        # Enable motor
        print("Enabling motor...")
        motor.enable_motor()
        time.sleep(0.1)

        # Enable auto-report for status updates
        print("Enabling auto-report...")
        motor.set_auto_report(True)
        time.sleep(0.1)

        # NOTE: do NOT call set_zero_position() here — it overwrites the encoder zero to the
        # current physical position. For motion-control to absolute 0 rad, do not reset zero.
        # If you explicitly want to redefine the zero, call motor.set_zero_position() manually.

        # Use direct motion control command (Communication Type 1)
        target_angle = 0  # rad (example)
        print(f"Sending motion-control command (Type 1) to target {target_angle:.2f} rad...")

        # Send repeatedly until within tolerance or timeout
        timeout = 5.0  # seconds
        deadline = time.time() + timeout
        tol_rad = 0.05  # ~2.8 degrees

        while time.time() < deadline:
            # send a small non-zero torque to provoke motion (experiment A)
            motor.send_motion_control(torque=0.5, angle=target_angle, speed=0.0, kp=50.0, kd=1.0)
            # Wait a short time for motor to act and for auto-report to update
            time.sleep(0.1)
            current_angle = motor.status.angle
            err = abs(current_angle - target_angle)
            if err <= tol_rad:
                print(f"Reached target within tolerance: err={err:.3f} rad")
                break

        # Wait and print a few status updates
        for i in range(50):
            time.sleep(0.1)
            status = motor.status
            angle_deg = math.degrees(status.angle)
            print(f"Status #{i+1}: angle={angle_deg:+.2f}°, speed={status.speed:.3f} rad/s, torque={status.torque:.3f} Nm, temp={status.temperature:.1f}°C")

        print("Done")

    except Exception as e:
        print(f"Error: {e}")

    finally:
        if motor is not None:
            try:
                motor.disable_motor()
                print("Motor disabled")
            except Exception:
                pass


if __name__ == '__main__':
    main()
