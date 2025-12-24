"""
Example: Set current position as zero point on RobStride motor using Private protocol.

Usage:
    python src/examples/set_zero_position.py --interface can0 --motor 0x7F

The script will:
 - open the CAN bus
 - instantiate RobStrideMotor/PrivateProtocolHandler
 - enable the motor
 - send set-zero command
 - disable the motor and exit
"""

import argparse
import logging
import time

from robstride_motor import ActuatorType, RobStrideMotor


def main():
    parser = argparse.ArgumentParser(description="Set current motor position to zero (CSP/encoder zero)")
    parser.add_argument("--interface", default="can0", help="CAN interface (default: can0)")
    parser.add_argument(
        "--motor-id",
        type=lambda x: int(x, 0),
        default=127,
        help="Motor ID (default: 127)",
    )
    parser.add_argument(
        "--master-id",
        type=lambda x: int(x, 0),
        default=255,
        help="Master ID (default: 255)",
    )
    parser.add_argument(
        "--actuator-type",
        type=int,
        default=2,
        choices=range(7),
        help="Actuator type 0-6 (default: 2 for RS02)",
    )
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")
    logger = logging.getLogger(__name__)

    logger.info(f"Opening CAN interface {args.interface} for motor ID {args.motor_id}")

    # Instantiate high-level motor wrapper (RobStrideMotor handles protocol internals)
    motor = RobStrideMotor(
        can_interface=args.interface,
        master_id=args.master_id,
        motor_id=args.motor_id,
        actuator_type=ActuatorType(args.actuator_type),
    )

    try:
        logger.info("Enabling motor...")
        try:
            motor.enable_motor()
        except Exception as e:
            logger.error(f"Enable failed: {e}")
            return 1

        # Small delay to let motor actually enable
        time.sleep(0.2)

        logger.info("Sending set-zero command (set current position as zero)")
        try:
            motor.set_zero_position()
        except Exception as e:
            logger.error(f"Set-zero failed: {e}")
            motor.disable_motor()
            return 1

        # Allow device to process set-zero
        time.sleep(0.2)

        logger.info("Disabling motor (clean exit)")
        motor.disable_motor()

        logger.info("Set-zero complete")
        return 0

    finally:
        # Ensure bus closed if RobStrideMotor exposes close
        try:
            motor.__del__()
        except Exception:
            pass


if __name__ == '__main__':
    raise SystemExit(main())
