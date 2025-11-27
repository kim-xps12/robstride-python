"""Pure Python library for RobStride BLDC motor control via CAN."""

from robstride_motor.motor import RobStrideMotor
from robstride_motor.types import (
    ActuatorType,
    CommunicationType,
    ControlMode,
    MotorFeedback,
)

__version__ = "0.1.0"

__all__ = [
    "RobStrideMotor",
    "ActuatorType",
    "ControlMode",
    "CommunicationType",
    "MotorFeedback",
]
