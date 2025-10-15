"""
RobStride Motor Control Library for Python

Python implementation of RobStride RS02 motor control via CAN bus.
Supports both Private and MIT protocols.

Version: 1.0.0
"""

from .motor import RobStrideMotor
from .models import (
    MotorStatus, 
    ParameterData,
    ControlMode,
    ProtocolMode,
    ErrorFlag,
    CommunicationType,
    BaudRate,
    MITMotorType,
    MotorPattern
)

__version__ = "1.0.0"
__all__ = [
    "RobStrideMotor",
    "MotorStatus",
    "ParameterData",
    "ControlMode",
    "ProtocolMode",
    "ErrorFlag",
    "CommunicationType",
    "BaudRate",
    "MITMotorType",
    "MotorPattern"
]
