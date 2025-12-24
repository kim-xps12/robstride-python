"""Sample script demonstrating RobStride motor control.

This script shows how to use the RobStride motor library to control a motor
in various modes including motion control, velocity, position, and current modes.

Usage:
    uv run examples/basic_control.py
    uv run examples/basic_control.py --motor-id 1
"""

import time
import argparse

from robstride_motor import ActuatorType, RobStrideMotor


def main() -> None:
    """Main control loop demonstration."""
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='Demonstrate basic motor control in various modes'
    )
    parser.add_argument('--interface', default='can0', help='CAN interface (default: can0)')
    parser.add_argument(
        '--motor-id',
        type=lambda x: int(x, 0),
        default=1,
        help='Motor ID (default: 1)',
    )
    parser.add_argument(
        '--master-id',
        type=lambda x: int(x, 0),
        default=255,
        help='Master ID (default: 255)',
    )
    parser.add_argument(
        '--actuator-type',
        type=int,
        default=0,
        choices=range(7),
        help='Actuator type 0-6 (default: 0 for RS00)',
    )
    args = parser.parse_args()
    
    # Initialize motor controller
    # Parameters: CAN interface, master ID, motor ID, actuator type
    motor = RobStrideMotor(
        can_interface=args.interface,
        master_id=args.master_id,
        motor_id=args.motor_id,
        actuator_type=ActuatorType(args.actuator_type),
    )

    try:
        # Enable the motor
        print("Enabling motor...")
        feedback = motor.enable_motor()
        print(f"Motor enabled: {feedback}")
        time.sleep(0.001)

        # Example 1: Motion control mode
        print("\n=== Motion Control Mode ===")
        position = 1.57  # rad (~90 degrees)
        velocity = 0.1  # rad/s
        for _ in range(10):
            feedback = motor.send_motion_command(
                torque=0.0,
                position=position,
                velocity=velocity,
                kp=0.1,
                kd=0.1,
            )
            print(f"Position: {feedback.position:.3f} rad, Velocity: {feedback.velocity:.3f} rad/s")
            time.sleep(0.001)

        # Example 2: Position CSP mode
        print("\n=== Position CSP Mode ===")
        for _ in range(10):
            feedback = motor.send_position_csp_command(angle=position, speed=velocity)
            print(f"Position: {feedback.position:.3f} rad, Temp: {feedback.temperature:.1f}°C")
            time.sleep(0.001)

        # Example 3: Velocity mode
        print("\n=== Velocity Mode ===")
        feedback = motor.send_velocity_command(velocity=5.0, acceleration=10.0)
        print(f"Velocity command sent: {feedback}")
        time.sleep(0.1)

        # Example 4: Current mode
        print("\n=== Current Mode ===")
        feedback = motor.send_current_command(iq=1.0, id_val=0.0)
        print(f"Current command sent: {feedback}")
        time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        # Always disable motor on exit
        print("\nDisabling motor...")
        motor.disable_motor()
        print("Motor disabled.")


if __name__ == "__main__":
    main()
