# RobStride Motor Control Library - Data Structures Specification
# RobStride モーター制御ライブラリ - データ構造仕様書

**Document Version:** 1.0  
**Date:** 2025-10-09

---

## 1. Overview / 概要

本ドキュメントは、RobStride モーター制御ライブラリで使用される全データ構造、列挙型、定数を定義する。

---

## 2. Enumerations / 列挙型

### 2.1 Control Modes

```python
from enum import IntEnum

class ControlMode(IntEnum):
    """制御モード"""
    MOTION_CONTROL = 0  # 複合運動制御（トルク・位置・速度）
    POSITION_PP = 1     # PP 位置制御
    SPEED = 2           # 速度制御
    CURRENT = 3         # 電流制御
    SET_ZERO = 4        # ゼロ点設定モード
    POSITION_CSP = 5    # CSP 位置制御
```

**C++ Reference:**
```cpp
#define move_control_mode  0
#define Pos_control_mode   1
#define Speed_control_mode 2
#define Elect_control_mode 3
#define Set_Zero_mode      4
#define CSP_control_mode   5
```

### 2.2 MIT Motor Types

```python
class MITMotorType(IntEnum):
    """MIT モータータイプ"""
    OPERATION_CONTROL = 0x01  # 複合制御
    POSITION_CONTROL = 0x02   # 位置制御
    SPEED_CONTROL = 0x03      # 速度制御
```

**C++ Reference:**
```cpp
enum MIT_TYPE {
    operationControl = 0,
    positionControl = 1,
    speedControl = 2
};
```

### 2.3 Protocol Modes

```python
class ProtocolMode(IntEnum):
    """プロトコルモード"""
    PRIVATE = 0x00   # RobStride 独自プロトコル
    CANOPEN = 0x01   # CANopen プロトコル
    MIT = 0x02       # MIT プロトコル
```

### 2.4 Communication Types (Private Protocol)

```python
class CommunicationType(IntEnum):
    """Private プロトコル通信タイプ"""
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
    MOTOR_DATA_SAVE = 0x16
    BAUD_RATE_CHANGE = 0x17
    PROACTIVE_ESCALATION_SET = 0x18
    MOTOR_MODE_SET = 0x19
```

**C++ Reference:**
```cpp
#define Communication_Type_Get_ID 0x00
#define Communication_Type_MotionControl 0x01
#define Communication_Type_MotorRequest 0x02
// ... (全 15 種類)
```

### 2.5 Baud Rates

```python
class BaudRate(IntEnum):
    """CAN ボーレート"""
    RATE_1M = 0x01    # 1 Mbps
    RATE_500K = 0x02  # 500 Kbps
    RATE_250K = 0x03  # 250 Kbps
    RATE_125K = 0x04  # 125 Kbps
```

### 2.6 Error Flags

```python
class ErrorFlag(IntEnum):
    """エラーフラグ (bit positions)"""
    UNDER_VOLTAGE = 0   # Bit 0: バス電圧低下
    OVER_CURRENT = 1    # Bit 1: 過電流
    OVER_TEMP = 2       # Bit 2: 過温度
    ENCODER_ERROR = 3   # Bit 3: エンコーダエラー
    OVER_VOLTAGE = 4    # Bit 4: 過電圧
    NOT_CALIBRATED = 5  # Bit 5: 未校正
```

### 2.7 Motor Status Pattern

```python
class MotorPattern(IntEnum):
    """モーター動作パターン"""
    TORQUE = 0    # トルク制御中
    POSITION = 1  # 位置制御中
    SPEED = 2     # 速度制御中
    RUNNING = 3   # 実行中
```

---

## 3. Data Classes / データクラス

### 3.1 MotorStatus

```python
from dataclasses import dataclass

@dataclass
class MotorStatus:
    """
    モーター状態情報
    
    Private および MIT プロトコル共通で使用される
    モーターからのフィードバック情報を格納
    """
    angle: float = 0.0          # 角度 [rad], 範囲: -12.5 ~ 12.5
    speed: float = 0.0          # 速度 [rad/s], 範囲: -44 ~ 44
    torque: float = 0.0         # トルク [Nm], 範囲: -17 ~ 17
    temperature: float = 0.0    # 温度 [°C], 範囲: 0 ~ 200
    pattern: int = 0            # 制御パターン (0-3)
    error_code: int = 0         # エラーコード (8-bit bitmap)
    
    @property
    def has_error(self) -> bool:
        """エラーフラグが立っているか"""
        return self.error_code != 0
    
    @property
    def is_running(self) -> bool:
        """動作中か（パターンが 0 以外）"""
        return self.pattern > 0
    
    def get_error_names(self) -> list[str]:
        """エラーフラグ名のリストを返す"""
        errors = []
        if self.error_code & (1 << ErrorFlag.UNDER_VOLTAGE):
            errors.append("UNDER_VOLTAGE")
        if self.error_code & (1 << ErrorFlag.OVER_CURRENT):
            errors.append("OVER_CURRENT")
        if self.error_code & (1 << ErrorFlag.OVER_TEMP):
            errors.append("OVER_TEMP")
        if self.error_code & (1 << ErrorFlag.ENCODER_ERROR):
            errors.append("ENCODER_ERROR")
        if self.error_code & (1 << ErrorFlag.OVER_VOLTAGE):
            errors.append("OVER_VOLTAGE")
        if self.error_code & (1 << ErrorFlag.NOT_CALIBRATED):
            errors.append("NOT_CALIBRATED")
        return errors
```

**C++ Reference:**
```cpp
typedef struct {
    float Angle;
    float Speed;
    float Torque;
    float Temp;
    int pattern;
} Motor_Pos_RobStride_Info;
```

### 3.2 ParameterData

```python
@dataclass
class ParameterData:
    """
    パラメータ読み書き用データ
    
    モーターの各種パラメータを格納
    index と data のペアで管理
    """
    # 書き込み可能パラメータ
    run_mode: float = 0.0           # 動作モード (0-5)
    iq_ref: float = 0.0             # 電流指令値 [A], -23 ~ 23
    spd_ref: float = 0.0            # 速度指令値 [rad/s], -30 ~ 30
    limit_torque: float = 0.0       # トルク制限 [Nm], 0 ~ 12
    cur_kp: float = 0.125           # 電流制御 Kp (デフォルト: 0.125)
    cur_ki: float = 0.0158          # 電流制御 Ki (デフォルト: 0.0158)
    cur_filt_gain: float = 0.1      # 電流フィルタゲイン, 0 ~ 1.0 (デフォルト: 0.1)
    loc_ref: float = 0.0            # 位置指令値 [rad]
    limit_spd: float = 0.0          # 速度制限 [rad/s], 0 ~ 30
    limit_cur: float = 0.0          # 電流制限 [A], 0 ~ 23
    
    # 読み取り専用パラメータ
    mech_pos: float = 0.0           # 機械位置 [rad] (read-only)
    iqf: float = 0.0                # 電流フィルタ値 [A] (read-only)
    mech_vel: float = 0.0           # 機械速度 [rad/s] (read-only)
    vbus: float = 0.0               # バス電圧 [V] (read-only)
    rotation: int = 0               # 回転数 (read-only)
```

**C++ Reference:**
```cpp
class data_read_write {
public:
    data_read_write_one run_mode;
    data_read_write_one iq_ref;
    data_read_write_one spd_ref;
    // ... (全 15 フィールド)
};
```

### 3.3 MotorConfiguration

```python
@dataclass
class MotorConfiguration:
    """モーター設定情報"""
    motor_id: int                    # CAN ID (0x00-0x7F)
    unique_id: int = 0               # 64-bit MCU Unique ID
    master_id: int = 0xFD            # Master CAN ID
    mit_mode: bool = False           # MIT プロトコル使用フラグ
    mit_type: MITMotorType = MITMotorType.OPERATION_CONTROL
    offset_function: callable = None # 角度オフセット関数
    
    def __post_init__(self):
        if not 0 <= self.motor_id <= 0x7F:
            raise ValueError(f"motor_id must be 0x00-0x7F, got {hex(self.motor_id)}")
```

### 3.4 ControlCommand

```python
@dataclass
class ControlCommand:
    """
    制御コマンド（内部使用）
    
    送信する制御コマンドを一時保存
    """
    mode: ControlMode = ControlMode.MOTION_CONTROL
    torque: float = 0.0      # [Nm]
    angle: float = 0.0       # [rad]
    speed: float = 0.0       # [rad/s]
    kp: float = 0.0
    kd: float = 0.0
    current: float = 0.0     # [A]
    
    def validate(self):
        """パラメータ範囲チェック"""
        if not -4.0 <= self.torque <= 4.0:
            raise ValueError(f"Torque must be -4.0 ~ 4.0 Nm, got {self.torque}")
        if not -12.5 <= self.angle <= 12.5:
            raise ValueError(f"Angle must be -12.5 ~ 12.5 rad, got {self.angle}")
        if not -30.0 <= self.speed <= 30.0:
            raise ValueError(f"Speed must be -30 ~ 30 rad/s, got {self.speed}")
        if not 0.0 <= self.kp <= 500.0:
            raise ValueError(f"Kp must be 0 ~ 500, got {self.kp}")
        if not 0.0 <= self.kd <= 5.0:
            raise ValueError(f"Kd must be 0 ~ 5, got {self.kd}")
        if not -23.0 <= self.current <= 23.0:
            raise ValueError(f"Current must be -23 ~ 23 A, got {self.current}")
```

---

## 4. Constants / 定数

### 4.1 Parameter Indices

```python
class ParameterIndex:
    """パラメータインデックス（0x7xxx シリーズ）"""
    RUN_MODE = 0x7005        # 動作モード
    IQ_REF = 0x7006          # 電流指令値
    SPD_REF = 0x700A         # 速度指令値
    LIMIT_TORQUE = 0x700B    # トルク制限
    CUR_KP = 0x7010          # 電流制御 Kp
    CUR_KI = 0x7011          # 電流制御 Ki
    CUR_FILT_GAIN = 0x7014   # 電流フィルタゲイン
    LOC_REF = 0x7016         # 位置指令値
    LIMIT_SPD = 0x7017       # 速度制限（CSP）
    LIMIT_CUR = 0x7018       # 電流制限
    MECH_POS = 0x7019        # 機械位置 (read-only)
    IQF = 0x701A             # 電流フィルタ値 (read-only)
    MECH_VEL = 0x701B        # 機械速度 (read-only)
    VBUS = 0x701C            # バス電圧 (read-only)
    ROTATION = 0x701D        # 回転数 (read-only)
    LIMIT_SPD_PP = 0x7024    # 速度制限（PP モード）
    ACCELERATION = 0x7025    # 加速度設定
    ACCEL_SPD = 0x7022       # 加速度（速度モード）

# インデックスリスト（順序重要）
PARAMETER_INDEX_LIST = [
    ParameterIndex.RUN_MODE,      # 0
    ParameterIndex.IQ_REF,        # 1
    ParameterIndex.SPD_REF,       # 2
    ParameterIndex.LIMIT_TORQUE,  # 3
    ParameterIndex.CUR_KP,        # 4
    ParameterIndex.CUR_KI,        # 5
    ParameterIndex.CUR_FILT_GAIN, # 6
    ParameterIndex.LOC_REF,       # 7
    ParameterIndex.LIMIT_SPD,     # 8
    ParameterIndex.LIMIT_CUR,     # 9
    ParameterIndex.MECH_POS,      # 10
    ParameterIndex.IQF,           # 11
    ParameterIndex.MECH_VEL,      # 12
    ParameterIndex.VBUS,          # 13
    ParameterIndex.ROTATION,      # 14
]
```

**C++ Reference:**
```cpp
static const uint16_t Index_List[] = {
    0X7005, 0X7006, 0X700A, 0X700B, 0X7010, 0X7011, 0X7014,
    0X7016, 0X7017, 0X7018, 0x7019, 0x701A, 0x701B, 0x701C, 0x701D
};
```

### 4.2 Value Limits

```python
class ValueLimits:
    """各種パラメータの物理的制限"""
    # 角度・位置
    ANGLE_MIN = -12.5    # [rad] (約 -4π)
    ANGLE_MAX = 12.5     # [rad] (約 4π)
    
    # 速度
    SPEED_MIN = -30.0    # [rad/s] (Private protocol)
    SPEED_MAX = 30.0     # [rad/s] (Private protocol)
    SPEED_MIN_MIT = -44.0  # [rad/s] (MIT protocol)
    SPEED_MAX_MIT = 44.0   # [rad/s] (MIT protocol)
    
    # トルク
    TORQUE_MIN = -4.0    # [Nm] (制御指令)
    TORQUE_MAX = 4.0     # [Nm] (制御指令)
    TORQUE_MIN_FEEDBACK = -17.0  # [Nm] (フィードバック)
    TORQUE_MAX_FEEDBACK = 17.0   # [Nm] (フィードバック)
    
    # 電流
    CURRENT_MIN = -23.0  # [A]
    CURRENT_MAX = 23.0   # [A]
    
    # ゲイン
    KP_MIN = 0.0
    KP_MAX = 500.0
    KD_MIN = 0.0
    KD_MAX = 5.0
    
    # 温度
    TEMP_MIN = 0.0       # [°C]
    TEMP_MAX = 200.0     # [°C]
    
    # 電圧
    VBUS_MIN = 0.0       # [V]
    VBUS_MAX = 60.0      # [V] (推定)
```

**C++ Reference:**
```cpp
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -44.0f
#define V_MAX 44.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -17.0f
#define T_MAX 17.0f
```

### 4.3 MIT Command Sequences

```python
class MITCommand:
    """MIT プロトコル固定コマンドシーケンス"""
    ENABLE = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
    DISABLE = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])
    SET_ZERO = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE])
    
    # 可変部分（Data[6]）を含むテンプレート
    CLEAR_ERROR_TEMPLATE = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFB])
    SET_TYPE_TEMPLATE = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFC])
    SET_ID_TEMPLATE = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x01])
    PROTOCOL_SWITCH_TEMPLATE = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFD])
```

### 4.4 Default Values

```python
class DefaultValues:
    """デフォルト値"""
    MASTER_CAN_ID = 0xFD
    MOTOR_CAN_ID = 0x7F
    BAUD_RATE = BaudRate.RATE_1M
    CONTROL_MODE = ControlMode.MOTION_CONTROL
    MIT_MOTOR_TYPE = MITMotorType.OPERATION_CONTROL
    
    # PID ゲイン
    CUR_KP = 0.125
    CUR_KI = 0.0158
    CUR_FILT_GAIN = 0.1
    
    # タイムアウト
    PARAMETER_READ_TIMEOUT = 0.1   # [s]
    STATUS_FEEDBACK_TIMEOUT = 0.05 # [s]
    COMMAND_RESPONSE_TIMEOUT = 0.1 # [s]
```

---

## 5. Type Aliases / 型エイリアス

```python
from typing import Callable, Optional, Union

# 角度オフセット関数の型
OffsetFunction = Callable[[float], float]

# パラメータ値の型（float または int）
ParameterValue = Union[float, int]

# CAN ID の型
CANId = int  # 0x00-0x7F for motor, 0x00-0x1FFFFFFF for extended

# エラーコードの型
ErrorCode = int  # 8-bit bitmap
```

---

## 6. Validation Functions / バリデーション関数

```python
def validate_motor_id(motor_id: int) -> None:
    """モーター ID の範囲チェック"""
    if not 0 <= motor_id <= 0x7F:
        raise ValueError(f"motor_id must be 0x00-0x7F, got {hex(motor_id)}")

def validate_angle(angle: float) -> None:
    """角度の範囲チェック"""
    if not ValueLimits.ANGLE_MIN <= angle <= ValueLimits.ANGLE_MAX:
        raise ValueError(
            f"angle must be {ValueLimits.ANGLE_MIN} ~ {ValueLimits.ANGLE_MAX} rad, "
            f"got {angle}"
        )

def validate_speed(speed: float, mit_mode: bool = False) -> None:
    """速度の範囲チェック"""
    min_val = ValueLimits.SPEED_MIN_MIT if mit_mode else ValueLimits.SPEED_MIN
    max_val = ValueLimits.SPEED_MAX_MIT if mit_mode else ValueLimits.SPEED_MAX
    
    if not min_val <= speed <= max_val:
        raise ValueError(f"speed must be {min_val} ~ {max_val} rad/s, got {speed}")

def validate_torque(torque: float, is_command: bool = True) -> None:
    """トルクの範囲チェック"""
    min_val = ValueLimits.TORQUE_MIN if is_command else ValueLimits.TORQUE_MIN_FEEDBACK
    max_val = ValueLimits.TORQUE_MAX if is_command else ValueLimits.TORQUE_MAX_FEEDBACK
    
    if not min_val <= torque <= max_val:
        raise ValueError(f"torque must be {min_val} ~ {max_val} Nm, got {torque}")

def validate_current(current: float) -> None:
    """電流の範囲チェック"""
    if not ValueLimits.CURRENT_MIN <= current <= ValueLimits.CURRENT_MAX:
        raise ValueError(
            f"current must be {ValueLimits.CURRENT_MIN} ~ {ValueLimits.CURRENT_MAX} A, "
            f"got {current}"
        )

def validate_kp(kp: float) -> None:
    """Kp ゲインの範囲チェック"""
    if not ValueLimits.KP_MIN <= kp <= ValueLimits.KP_MAX:
        raise ValueError(f"Kp must be {ValueLimits.KP_MIN} ~ {ValueLimits.KP_MAX}, got {kp}")

def validate_kd(kd: float) -> None:
    """Kd ゲインの範囲チェック"""
    if not ValueLimits.KD_MIN <= kd <= ValueLimits.KD_MAX:
        raise ValueError(f"Kd must be {ValueLimits.KD_MIN} ~ {ValueLimits.KD_MAX}, got {kd}")

def validate_parameter_index(index: int) -> None:
    """パラメータインデックスの有効性チェック"""
    if index not in PARAMETER_INDEX_LIST and not (0x7000 <= index <= 0x7FFF):
        raise ValueError(f"Invalid parameter index: {hex(index)}")
```

---

## 7. Utility Classes / ユーティリティクラス

### 7.1 BitField Helper

```python
class BitField:
    """ビットフィールド操作ヘルパー"""
    
    @staticmethod
    def get_bit(value: int, bit_pos: int) -> bool:
        """指定ビットを取得"""
        return bool((value >> bit_pos) & 1)
    
    @staticmethod
    def set_bit(value: int, bit_pos: int, bit_value: bool) -> int:
        """指定ビットを設定"""
        if bit_value:
            return value | (1 << bit_pos)
        else:
            return value & ~(1 << bit_pos)
    
    @staticmethod
    def extract_bits(value: int, start_bit: int, num_bits: int) -> int:
        """ビット範囲を抽出"""
        mask = (1 << num_bits) - 1
        return (value >> start_bit) & mask
    
    @staticmethod
    def insert_bits(target: int, value: int, start_bit: int, num_bits: int) -> int:
        """ビット範囲に値を挿入"""
        mask = (1 << num_bits) - 1
        target &= ~(mask << start_bit)  # Clear target bits
        target |= (value & mask) << start_bit
        return target
```

### 7.2 CAN ID Parser

```python
@dataclass
class ParsedExtendedID:
    """拡張 CAN ID の解析結果"""
    comm_type: int    # [28:24]
    data_byte: int    # [23:16]
    master_id: int    # [15:8]
    motor_id: int     # [7:0]
    
    @classmethod
    def from_id(cls, ext_id: int) -> 'ParsedExtendedID':
        """拡張 ID を解析"""
        return cls(
            comm_type=(ext_id >> 24) & 0x1F,
            data_byte=(ext_id >> 16) & 0xFF,
            master_id=(ext_id >> 8) & 0xFF,
            motor_id=ext_id & 0xFF
        )
    
    def to_id(self) -> int:
        """拡張 ID に変換"""
        return (
            (self.comm_type << 24) |
            (self.data_byte << 16) |
            (self.master_id << 8) |
            self.motor_id
        )
```

---

## 8. JSON Serialization / JSON シリアライゼーション

```python
import json
from dataclasses import asdict

def serialize_motor_status(status: MotorStatus) -> str:
    """MotorStatus を JSON にシリアライズ"""
    data = asdict(status)
    data['error_names'] = status.get_error_names()
    return json.dumps(data, indent=2)

def deserialize_motor_status(json_str: str) -> MotorStatus:
    """JSON から MotorStatus を復元"""
    data = json.loads(json_str)
    # error_names は計算プロパティなので除外
    data.pop('error_names', None)
    return MotorStatus(**data)

# 使用例
status = MotorStatus(angle=1.57, speed=10.0, temperature=45.2)
json_str = serialize_motor_status(status)
restored = deserialize_motor_status(json_str)
```

---

## 9. Complete Example / 完全な使用例

```python
# モーター設定の作成
config = MotorConfiguration(
    motor_id=0x01,
    master_id=0xFD,
    mit_mode=False
)

# ステータスの初期化
status = MotorStatus()

# パラメータデータの初期化
params = ParameterData()

# 制御コマンドの作成と検証
command = ControlCommand(
    mode=ControlMode.POSITION_PP,
    angle=1.57,
    speed=3.0,
    kp=100.0,
    kd=1.0
)
command.validate()  # 範囲チェック

# エラー判定
status.error_code = (1 << ErrorFlag.OVER_TEMP) | (1 << ErrorFlag.OVER_CURRENT)
if status.has_error:
    print(f"Errors detected: {status.get_error_names()}")
    # Output: Errors detected: ['OVER_CURRENT', 'OVER_TEMP']

# 拡張 ID の構築と解析
ext_id = (CommunicationType.MOTION_CONTROL << 24) | (0x80 << 16) | (0xFD << 8) | 0x01
parsed = ParsedExtendedID.from_id(ext_id)
print(f"CommType: {hex(parsed.comm_type)}, Motor ID: {hex(parsed.motor_id)}")
# Output: CommType: 0x1, Motor ID: 0x1
```

---

**End of Data Structures Specification**
