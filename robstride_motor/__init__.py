"""Pure Python library for RobStride BLDC motor control via CAN."""

from robstride_motor.bus import create_can_bus
from robstride_motor.motor import RobStrideMotor
from robstride_motor.types import (
    ActuatorType,
    CommunicationType,
    ControlMode,
    FirmwareInfo,
    MotorFeedback,
    ParameterIndex,
)

__version__ = "0.1.0"

__all__ = [
    "RobStrideMotor",
    "create_can_bus",
    "ActuatorType",
    "ControlMode",
    "CommunicationType",
    "FirmwareInfo",
    "MotorFeedback",
    "ParameterIndex",
]
