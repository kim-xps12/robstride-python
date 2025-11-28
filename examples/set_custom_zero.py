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
    parser.add_argument("--interface", default="can0", help="SocketCAN interface")
    parser.add_argument("--motor", type=lambda x: int(x, 0), default=0x7F, help="Motor CAN ID (hex or int)")
    parser.add_argument("--master", type=lambda x: int(x, 0), default=0xFD, help="Master/host CAN ID (hex or int)")
    parser.add_argument("--actuator", type=int, default=2, help="Actuator type (0-6)")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")
    logger = logging.getLogger(__name__)

    logger.info(f"Opening CAN interface {args.interface} for motor 0x{args.motor:02X}")

    # Instantiate high-level motor wrapper (RobStrideMotor handles protocol internals)
    motor = RobStrideMotor(
        can_interface=args.interface,
        master_id=args.master,
        motor_id=args.motor,
        actuator_type=ActuatorType(args.actuator),
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
