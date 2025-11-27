"""Simple example showing basic motor initialization and enable/disable."""

from robstride_motor import ActuatorType, RobStrideMotor


def main() -> None:
    """Minimal example: initialize and enable motor."""
    print("Initializing motor...")
    motor = RobStrideMotor(
        can_interface="can0",
        master_id=0xFF,
        motor_id=0x01,
        actuator_type=ActuatorType.ROBSTRIDE_00,
    )

    print("Enabling motor...")
    feedback = motor.enable_motor()
    print(f"Motor enabled successfully: {feedback}")

    print("Disabling motor...")
    motor.disable_motor()
    print("Motor disabled successfully.")


if __name__ == "__main__":
    main()
