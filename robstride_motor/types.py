"""Type definitions and data structures for RobStride motor control."""

import math
from dataclasses import dataclass
from enum import IntEnum
from typing import NamedTuple


class ActuatorType(IntEnum):
    """Actuator type enumeration."""

    ROBSTRIDE_00 = 0
    ROBSTRIDE_01 = 1
    ROBSTRIDE_02 = 2
    ROBSTRIDE_03 = 3
    ROBSTRIDE_04 = 4
    ROBSTRIDE_05 = 5
    ROBSTRIDE_06 = 6


class ControlMode(IntEnum):
    """Control mode enumeration."""

    MOTION_CONTROL = 0  # 運控モード
    POSITION_PP = 1  # 位置モード（PP）
    VELOCITY = 2  # 速度モード
    CURRENT = 3  # 電流モード
    SET_ZERO = 4  # 零点モード
    POSITION_CSP = 5  # 位置モード（CSP）


class CommunicationType(IntEnum):
    """CAN communication type enumeration."""

    GET_ID = 0x00
    MOTION_CONTROL = 0x01
    MOTOR_REQUEST = 0x02
    MOTOR_ENABLE = 0x03
    MOTOR_STOP = 0x04
    SET_POS_ZERO = 0x06
    CAN_ID = 0x07
    GET_SINGLE_PARAMETER = 0x11
    SET_SINGLE_PARAMETER = 0x12
    ERROR_FEEDBACK = 0x15


class ActuatorOperation(NamedTuple):
    """Actuator operation parameters."""

    position: float  # rad
    velocity: float  # rad/s
    torque: float  # Nm
    kp: float
    kd: float


ACTUATOR_OPERATION_MAPPING: dict[ActuatorType, ActuatorOperation] = {
    ActuatorType.ROBSTRIDE_00: ActuatorOperation(4 * math.pi, 50.0, 17.0, 500.0, 5.0),
    ActuatorType.ROBSTRIDE_01: ActuatorOperation(4 * math.pi, 44.0, 17.0, 500.0, 5.0),
    ActuatorType.ROBSTRIDE_02: ActuatorOperation(4 * math.pi, 44.0, 17.0, 500.0, 5.0),
    ActuatorType.ROBSTRIDE_03: ActuatorOperation(4 * math.pi, 50.0, 60.0, 5000.0, 100.0),
    ActuatorType.ROBSTRIDE_04: ActuatorOperation(4 * math.pi, 15.0, 120.0, 5000.0, 100.0),
    ActuatorType.ROBSTRIDE_05: ActuatorOperation(4 * math.pi, 33.0, 17.0, 500.0, 5.0),
    ActuatorType.ROBSTRIDE_06: ActuatorOperation(4 * math.pi, 20.0, 60.0, 5000.0, 100.0),
}


class ParameterIndex(IntEnum):
    """Parameter index enumeration.

    Based on official RobStride RS02 specification.
    See ref/spec/spec.md for details.
    """

    RUN_MODE = 0x7005  # 運転モード: 0=モーション制御, 1=位置PP, 2=速度, 3=電流, 5=位置CSP
    IQ_REF = 0x7006  # 電流ループ制御指令 (A), float, -23~23A
    ID_REF = 0x7007  # D軸電流指令 (A), float
    LIMIT_SPD_VEL = 0x7008  # 速度モード速度制限 (rad/s), float, -44~44
    SPD_REF = 0x700A  # 速度モード速度指令 (rad/s), float
    LIMIT_TORQUE = 0x700F  # トルク制限 (Nm), float, 0~17
    CUR_KP = 0x7010  # 電流ループKp, float, default 0.17
    CUR_KI = 0x7011  # 電流ループKi, float, default 0.012
    CUR_FILT_GAIN = 0x7014  # 電流ループフィルタゲイン, float, 0~1.0
    LOC_REF = 0x7016  # 位置制御基準値 (rad), float
    LIMIT_SPD_CSP = 0x7017  # CSPモード速度制限 (rad/s), float, -44~44
    LIMIT_CUR = 0x7018  # 速度・位置モード電流制限 (A), float, 0~23
    MECH_POS = 0x7019  # エンコーダ機械角度 (rad), float, R
    IQF = 0x701A  # iq電流値 (A), float, R
    MECH_VEL = 0x701B  # エンコーダ速度 (rad/s), float, R
    VBUS = 0x701C  # 母線電圧 (V), float, R
    LOC_KP = 0x701E  # 位置制御Kp, float, default 40
    SPD_KP = 0x701F  # 速度制御Kp, float, default 16
    SPD_KI = 0x7020  # 速度制御Ki, float, default 0.02
    SPD_FILT_GAIN = 0x7021  # 速度ループフィルタゲイン, float, default 0.1
    ACC_RAD = 0x7022  # 位置モード加速度 (rad/s²), float, default 20
    VEL_MAX = 0x7024  # 位置モードPP最大速度 (rad/s), float, default 10
    ACC_SET = 0x7025  # 位置モードPP加速度設定 (rad/s²), float, default 10
    EPSCAN_TIME = 0x7026  # 上報時間設定 (1単位=10ms), uint16, default 1


@dataclass
class MotorFeedback:
    """Motor feedback data structure."""

    position: float  # rad
    velocity: float  # rad/s
    torque: float  # Nm
    temperature: float  # °C


@dataclass
class ParameterValue:
    """Parameter value with index."""

    index: int
    value: float


@dataclass
class MotorStatus:
    """Motor status information extracted from CAN frame."""

    mode: int
    uncalibrated: bool
    hall_encoder_fault: bool
    magnetic_encoder_fault: bool
    overtemperature: bool
    overcurrent: bool
    undervoltage: bool
    device_id: int


# Constants from C++ code
SC_MAX = 23.0
SC_MIN = 0.0
SV_MAX = 20.0
SV_MIN = -20.0
SCIQ_MIN = -23.0
