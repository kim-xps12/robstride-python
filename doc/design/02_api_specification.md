# RobStride Motor Control Library - API Specification
# RobStride モーター制御ライブラリ - API 仕様書

**Document Version:** 1.0  
**Date:** 2025-10-09  
**Language:** Python (target), C++ (reference)

---

## Table of Contents / 目次

1. [Class Overview](#1-class-overview--クラス概要)
2. [Constructor](#2-constructor--コンストラクタ)
3. [Core Control Methods](#3-core-control-methods--コア制御メソッド)
4. [Mode-Specific Control](#4-mode-specific-control--モード別制御)
5. [MIT Protocol Methods](#5-mit-protocol-methods--mitプロトコルメソッド)
6. [Parameter Management](#6-parameter-management--パラメータ管理)
7. [Configuration Methods](#7-configuration-methods--設定メソッド)
8. [Status & Feedback](#8-status--feedback--状態取得)
9. [Properties](#9-properties--プロパティ)
10. [Exceptions](#10-exceptions--例外)

---

## 1. Class Overview / クラス概要

### 1.1 Main Class: `RobStrideMotor`

```python
class RobStrideMotor:
    """
    RobStride RS01 モーター制御クラス
    
    Private プロトコルおよび MIT プロトコルをサポートし、
    位置・速度・電流・トルク制御を提供する。
    
    Attributes:
        motor_id (int): モーター CAN ID (0x00-0x7F)
        mit_mode (bool): MIT プロトコル使用フラグ
        pos_info (MotorStatus): モーター状態（読み取り専用）
        drw (DataReadWrite): パラメータデータ（読み取り専用）
    """
```

### 1.2 Related Classes

```python
class MotorStatus:
    """モーターステータス情報"""
    angle: float      # 角度 [rad]
    speed: float      # 速度 [rad/s]
    torque: float     # トルク [Nm]
    temperature: float # 温度 [°C]
    pattern: int      # 制御パターン (0=torque, 1=pos, 2=speed, 3=running)
    error_code: int   # エラーコード (8-bit)

class DataReadWrite:
    """パラメータ読み書き用データ"""
    run_mode: float           # 動作モード
    iq_ref: float            # 電流指令値 [A]
    spd_ref: float           # 速度指令値 [rad/s]
    limit_torque: float      # トルク制限 [Nm]
    cur_kp: float            # 電流制御 Kp
    cur_ki: float            # 電流制御 Ki
    cur_filt_gain: float     # 電流フィルタゲイン
    loc_ref: float           # 位置指令値 [rad]
    limit_spd: float         # 速度制限 [rad/s]
    limit_cur: float         # 電流制限 [A]
    mech_pos: float          # 機械位置 [rad] (read-only)
    iqf: float               # 電流フィルタ値 [A] (read-only)
    mech_vel: float          # 機械速度 [rad/s] (read-only)
    vbus: float              # バス電圧 [V] (read-only)
    rotation: int            # 回転数 (read-only)
```

---

## 2. Constructor / コンストラクタ

### 2.1 `__init__`

```python
def __init__(
    self,
    can_id: int,
    mit_mode: bool = False,
    can_interface: str = 'can0',
    master_id: int = 0xFD,
    offset_func: Optional[Callable[[float], float]] = None
) -> None:
    """
    RobStride モーターインスタンスを初期化
    
    Args:
        can_id: モーター CAN ID (0x00-0x7F)
        mit_mode: MIT プロトコル使用フラグ (デフォルト: False)
        can_interface: CAN インターフェース名 (例: 'can0', 'vcan0')
        master_id: マスター ID (デフォルト: 0xFD)
        offset_func: 角度オフセット関数（オプション）
        
    Raises:
        ValueError: CAN ID が範囲外の場合
        CANError: CAN インターフェース初期化失敗
        
    Example:
        >>> motor = RobStrideMotor(can_id=0x01, can_interface='can0')
        >>> motor_mit = RobStrideMotor(can_id=0x02, mit_mode=True)
    """
```

**C++ Reference:**
```cpp
RobStride_Motor::RobStride_Motor(uint8_t CAN_Id, bool MIT_mode)
RobStride_Motor::RobStride_Motor(
    float (*Offset_MotoFunc)(float Motor_Tar),
    uint8_t CAN_Id,
    bool MIT_mode
)
```

---

## 3. Core Control Methods / コア制御メソッド

### 3.1 `enable_motor`

```python
def enable_motor(self) -> None:
    """
    モーターを有効化（運転可能状態にする）
    
    Private モード: Communication Type 0x03 送信
    MIT モード: 0xFFFFFFFFFFFFFFFC 送信
    
    Raises:
        MotorError: 有効化失敗
        
    Example:
        >>> motor.enable_motor()
        >>> time.sleep(0.1)  # 有効化待ち
    """
```

**C++ Reference:**
```cpp
void RobStride_Motor::Enable_Motor()
void RobStride_Motor::RobStride_Motor_MIT_Enable()
```

### 3.2 `disable_motor`

```python
def disable_motor(self, clear_error: bool = False) -> None:
    """
    モーターを無効化（停止）
    
    Args:
        clear_error: True の場合、エラーフラグもクリア
        
    Private モード: Communication Type 0x04 送信
    MIT モード: 0xFFFFFFFFFFFFFFFD 送信
    
    Raises:
        MotorError: 無効化失敗
        
    Example:
        >>> motor.disable_motor(clear_error=True)
    """
```

**C++ Reference:**
```cpp
void RobStride_Motor::Disenable_Motor(uint8_t clear_error)
void RobStride_Motor::RobStride_Motor_MIT_Disable()
```

### 3.3 `set_zero_position`

```python
def set_zero_position(self) -> None:
    """
    現在位置を機械的ゼロ点として設定
    
    Process:
        1. モーター無効化
        2. ゼロ点設定コマンド送信 (Type 0x06)
        3. モーター再有効化
        
    Raises:
        MotorError: ゼロ点設定失敗
        
    Warning:
        この操作後、angle の基準が変わります
        
    Example:
        >>> motor.set_zero_position()
    """
```

**C++ Reference:**
```cpp
void RobStride_Motor::Set_ZeroPos()
```

---

## 4. Mode-Specific Control / モード別制御

### 4.1 `motion_control` (複合運動制御)

```python
def motion_control(
    self,
    torque: float,
    angle: float = 0.0,
    speed: float = 0.0,
    kp: float = 0.0,
    kd: float = 0.0
) -> None:
    """
    複合運動制御（トルク・位置・速度・ゲイン同時指定）
    
    Args:
        torque: 目標トルク [Nm], 範囲: -4.0 ~ 4.0
        angle: 目標角度 [rad], 範囲: -12.5 ~ 12.5 (約 -4π ~ 4π)
        speed: 目標速度 [rad/s], 範囲: -30 ~ 30
        kp: 位置ゲイン, 範囲: 0 ~ 500
        kd: 微分ゲイン, 範囲: 0 ~ 5
        
    Note:
        - 内部で run_mode を 0 (motion control) に設定
        - Communication Type 0x01 使用
        
    Raises:
        ValueError: パラメータが範囲外
        MotorError: 送信失敗
        
    Example:
        >>> # 位置制御 (Kp=100, Kd=1)
        >>> motor.motion_control(torque=0, angle=1.57, speed=0, kp=100, kd=1)
        >>> 
        >>> # トルク制御のみ
        >>> motor.motion_control(torque=2.5)
    """
```

**C++ Reference:**
```cpp
void RobStride_Motor::RobStride_Motor_move_control(
    float Torque, float Angle, float Speed, float Kp, float Kd
)
```

### 4.2 `position_control` (PP モード)

```python
def position_control(
    self,
    target_angle: float,
    target_speed: float = 2.0
) -> None:
    """
    位置制御モード (Point-to-Point)
    
    Args:
        target_angle: 目標角度 [rad]
        target_speed: 目標速度 [rad/s], 推奨: 2.0 ~ 5.0
        
    Implementation:
        1. run_mode を 1 (position mode) に設定 (0x7005)
        2. limit_spd を設定 (0x7024, オプション)
        3. target_angle を 0x7016 に書き込み
        
    Note:
        - 初回呼び出し時に HAL_Delay(1) 相当の待機推奨
        - target_speed は 0x7018 パラメータに反映される
        
    Raises:
        ValueError: angle が範囲外
        MotorError: モード設定失敗
        
    Example:
        >>> motor.position_control(target_angle=3.14, target_speed=3.0)
        >>> time.sleep(0.01)
    """
```

**C++ Reference:**
```cpp
void RobStride_Motor::RobStride_Motor_Pos_control(float Speed, float Angle)
```

### 4.3 `csp_control` (CSP 位置制御)

```python
def csp_control(
    self,
    target_angle: float,
    limit_speed: float = 10.0
) -> None:
    """
    CSP 位置制御モード (Cyclic Synchronous Position)
    
    Args:
        target_angle: 目標角度 [rad]
        limit_speed: 速度制限 [rad/s], 範囲: 0 ~ 44
        
    Note:
        - MIT モードの場合は MIT_PositionControl を使用
        - Private モードでは run_mode=5, 0x7016/0x7017 使用
        
    Raises:
        ValueError: パラメータが範囲外
        MotorError: 制御失敗
        
    Example:
        >>> motor.csp_control(target_angle=1.0, limit_speed=5.0)
    """
```

**C++ Reference:**
```cpp
void RobStride_Motor::RobStride_Motor_CSP_control(float Angle, float limit_spd)
```

### 4.4 `speed_control`

```python
def speed_control(
    self,
    target_speed: float,
    limit_current: float = 5.0
) -> None:
    """
    速度制御モード
    
    Args:
        target_speed: 目標速度 [rad/s], 範囲: -30 ~ 30
        limit_current: 電流制限 [A], 範囲: 0 ~ 23
        
    Implementation:
        1. run_mode を 2 (speed mode) に設定 (0x7005)
        2. limit_cur を設定 (0x7018)
        3. target_speed を 0x700A に書き込み
        
    Raises:
        ValueError: パラメータが範囲外
        MotorError: 制御失敗
        
    Example:
        >>> motor.speed_control(target_speed=10.0, limit_current=3.0)
    """
```

**C++ Reference:**
```cpp
void RobStride_Motor::RobStride_Motor_Speed_control(float Speed, float limit_cur)
```

### 4.5 `current_control`

```python
def current_control(self, target_current: float) -> None:
    """
    電流（トルク）制御モード
    
    Args:
        target_current: 目標電流 [A], 範囲: -23 ~ 23
        
    Implementation:
        1. run_mode を 3 (current mode) に設定 (0x7005)
        2. target_current を 0x7006 に書き込み
        
    Raises:
        ValueError: current が範囲外
        MotorError: 制御失敗
        
    Example:
        >>> motor.current_control(target_current=1.5)
    """
```

**C++ Reference:**
```cpp
void RobStride_Motor::RobStride_Motor_current_control(float current)
```

---

## 5. MIT Protocol Methods / MIT プロトコルメソッド

### 5.1 `mit_control`

```python
def mit_control(
    self,
    angle: float = 0.0,
    speed: float = 0.0,
    kp: float = 0.0,
    kd: float = 0.0,
    torque: float = 0.0
) -> None:
    """
    MIT 複合制御コマンド
    
    Args:
        angle: 目標角度 [rad], 範囲: -12.5 ~ 12.5
        speed: 目標速度 [rad/s], 範囲: -44 ~ 44
        kp: 位置ゲイン, 範囲: 0 ~ 500
        kd: 微分ゲイン, 範囲: 0 ~ 5
        torque: フィードフォワードトルク [Nm], 範囲: -17 ~ 17
        
    Encoding:
        - Standard CAN ID: motor_id
        - Payload: 8 bytes, packed format (12/16-bit scaled values)
        
    Raises:
        ProtocolError: MIT モードでない場合
        ValueError: パラメータが範囲外
        
    Example:
        >>> motor.mit_control(angle=0, speed=0, kp=0, kd=0, torque=-1.0)
    """
```

**C++ Reference:**
```cpp
void RobStride_Motor::RobStride_Motor_MIT_Control(
    float Angle, float Speed, float Kp, float Kd, float Torque
)
```

### 5.2 `mit_position_control`

```python
def mit_position_control(
    self,
    position_rad: float,
    speed_rad_per_s: float = 3.0
) -> None:
    """
    MIT 位置制御専用コマンド
    
    Args:
        position_rad: 目標位置 [rad]
        speed_rad_per_s: 目標速度 [rad/s]
        
    CAN Format:
        - Standard ID: (1 << 8) | motor_id
        - Payload: [position:float32][speed:float32]
        
    Raises:
        ProtocolError: MIT モードでない場合
        
    Example:
        >>> motor.mit_position_control(position_rad=1.57, speed_rad_per_s=3.0)
    """
```

**C++ Reference:**
```cpp
void RobStride_Motor::RobStride_Motor_MIT_PositionControl(
    float position_rad, float speed_rad_per_s
)
```

### 5.3 `mit_speed_control`

```python
def mit_speed_control(
    self,
    speed_rad_per_s: float,
    current_limit: float = 5.0
) -> None:
    """
    MIT 速度制御専用コマンド
    
    Args:
        speed_rad_per_s: 目標速度 [rad/s]
        current_limit: 電流制限 [A]
        
    CAN Format:
        - Standard ID: (2 << 8) | motor_id
        - Payload: [speed:float32][current_limit:float32]
        
    Raises:
        ProtocolError: MIT モードでない場合
        
    Example:
        >>> motor.mit_speed_control(speed_rad_per_s=4.5, current_limit=3.2)
    """
```

**C++ Reference:**
```cpp
void RobStride_Motor::RobStride_Motor_MIT_SpeedControl(
    float speed_rad_per_s, float current_limit
)
```

### 5.4 `mit_set_zero_position`

```python
def mit_set_zero_position(self) -> None:
    """
    MIT ゼロ点設定
    
    Precondition:
        MIT_Type != positionControl
        
    CAN Sequence:
        0xFFFFFFFFFFFFFFFE (8 bytes)
        
    Raises:
        ProtocolError: MIT モードでない場合
        StateError: positionControl モード中に呼び出された場合
        
    Example:
        >>> motor.mit_set_zero_position()
    """
```

**C++ Reference:**
```cpp
void RobStride_Motor::RobStride_Motor_MIT_SetZeroPos()
```

### 5.5 `mit_clear_error`

```python
def mit_clear_error(self, command: int = 0x01) -> None:
    """
    MIT エラークリア
    
    Args:
        command: 0x00=check, 0x01=clear
        
    CAN Sequence:
        0xFFFFFFFFFF[command]FB
        
    Raises:
        ProtocolError: MIT モードでない場合
        
    Example:
        >>> motor.mit_clear_error(command=0x01)
    """
```

**C++ Reference:**
```cpp
void RobStride_Motor::RobStride_Motor_MIT_ClearOrCheckError(uint8_t F_CMD)
```

### 5.6 `mit_set_motor_type`

```python
def mit_set_motor_type(self, motor_type: int) -> None:
    """
    MIT モータータイプ設定
    
    Args:
        motor_type: 0x01=operation, 0x02=position, 0x03=speed
        
    CAN Sequence:
        0xFFFFFFFFFF[motor_type]FC
        
    Raises:
        ProtocolError: MIT モードでない場合
        ValueError: motor_type が無効
        
    Example:
        >>> motor.mit_set_motor_type(motor_type=0x01)
    """
```

**C++ Reference:**
```cpp
void RobStride_Motor::RobStride_Motor_MIT_SetMotorType(uint8_t F_CMD)
```

### 5.7 `mit_set_motor_id`

```python
def mit_set_motor_id(self, new_id: int) -> None:
    """
    MIT モーター ID 変更
    
    Args:
        new_id: 新しい ID (0x00-0x7F)
        
    CAN Sequence:
        0xFFFFFFFFFF[new_id]01
        
    Warning:
        実行後はモーターの再起動が必要
        
    Raises:
        ProtocolError: MIT モードでない場合
        ValueError: new_id が範囲外
        
    Example:
        >>> motor.mit_set_motor_id(new_id=0x05)
    """
```

**C++ Reference:**
```cpp
void RobStride_Motor::RobStride_Motor_MIT_SetMotorId(uint8_t F_CMD)
```

---

## 6. Parameter Management / パラメータ管理

### 6.1 `set_parameter`

```python
def set_parameter(
    self,
    index: int,
    value: Union[float, int],
    value_mode: str = 'p'
) -> None:
    """
    パラメータ書き込み
    
    Args:
        index: パラメータアドレス (0x7xxx)
        value: 設定値
        value_mode: 
            'p' = parameter (float, 4 bytes)
            'j' = mode (uint8, 1 byte)
            
    Communication Type: 0x12
    
    Raises:
        ValueError: index が範囲外または value_mode が無効
        MotorError: 書き込み失敗
        
    Example:
        >>> # 制御モードを位置モード(1)に設定
        >>> motor.set_parameter(0x7005, 1, value_mode='j')
        >>> 
        >>> # 速度制限を 5.0 rad/s に設定
        >>> motor.set_parameter(0x7018, 5.0, value_mode='p')
    """
```

**C++ Reference:**
```cpp
void RobStride_Motor::Set_RobStride_Motor_parameter(
    uint16_t Index, float Value, char Value_mode
)
```

### 6.2 `get_parameter`

```python
def get_parameter(self, index: int) -> None:
    """
    パラメータ読み取り要求
    
    Args:
        index: パラメータアドレス (0x7xxx)
        
    Note:
        - この関数は要求を送信するのみ（非同期）
        - 結果は CAN 受信コールバックで self.drw に格納される
        - 同期的に値を取得したい場合は get_parameter_sync() を使用
        
    Communication Type: 0x11
    
    Raises:
        ValueError: index が範囲外
        MotorError: 送信失敗
        
    Example:
        >>> motor.get_parameter(0x7019)  # mechPos 読み取り要求
        >>> time.sleep(0.01)
        >>> print(motor.drw.mech_pos)
    """
```

**C++ Reference:**
```cpp
void RobStride_Motor::Get_RobStride_Motor_parameter(uint16_t Index)
```

### 6.3 `get_parameter_sync` (拡張)

```python
def get_parameter_sync(
    self,
    index: int,
    timeout: float = 0.1
) -> Union[float, int]:
    """
    パラメータ読み取り（同期版）
    
    Args:
        index: パラメータアドレス
        timeout: タイムアウト時間 [s]
        
    Returns:
        読み取った値
        
    Raises:
        TimeoutError: タイムアウト
        ValueError: index が無効
        
    Example:
        >>> mech_pos = motor.get_parameter_sync(0x7019, timeout=0.1)
        >>> print(f"Mechanical position: {mech_pos} rad")
    """
```

### 6.4 `save_parameters`

```python
def save_parameters(self) -> None:
    """
    パラメータを FLASH に保存
    
    現在の RAM パラメータを不揮発性メモリに保存し、
    次回起動時のデフォルト値として使用する。
    
    Communication Type: 0x16
    Magic Sequence: 0x0102030405060708
    
    Warning:
        書き込み回数制限あり（FLASH 寿命）
        
    Example:
        >>> motor.save_parameters()
    """
```

**C++ Reference:**
```cpp
void RobStride_Motor::RobStride_Motor_MotorDataSave()
```

---

## 7. Configuration Methods / 設定メソッド

### 7.1 `get_can_id`

```python
def get_can_id(self) -> None:
    """
    CAN ID および 64-bit MCU Unique ID 取得要求
    
    Communication Type: 0x00
    
    Note:
        結果は受信コールバックで self.unique_id に格納される
        
    Example:
        >>> motor.get_can_id()
        >>> time.sleep(0.05)
        >>> print(f"Unique ID: {hex(motor.unique_id)}")
    """
```

**C++ Reference:**
```cpp
void RobStride_Motor::RobStride_Get_CAN_ID()
```

### 7.2 `set_can_id`

```python
def set_can_id(self, new_can_id: int) -> None:
    """
    CAN ID 変更
    
    Args:
        new_can_id: 新しい CAN ID (0x00-0x7F)
        
    Process:
        1. モーター無効化
        2. ID 変更コマンド送信 (Type 0x07)
        
    Warning:
        変更後はモーター再起動が必要
        
    Raises:
        ValueError: new_can_id が範囲外
        
    Example:
        >>> motor.set_can_id(new_can_id=0x10)
    """
```

**C++ Reference:**
```cpp
void RobStride_Motor::Set_CAN_ID(uint8_t Set_CAN_ID)
```

### 7.3 `change_baud_rate`

```python
def change_baud_rate(self, rate_code: int) -> None:
    """
    CAN ボーレート変更
    
    Args:
        rate_code:
            0x01 = 1M bps
            0x02 = 500K bps
            0x03 = 250K bps
            0x04 = 125K bps
            
    Communication Type: 0x17
    
    Warning:
        変更後はモーター再起動が必要
        ホスト側の CAN インターフェースも同じレートに変更すること
        
    Raises:
        ValueError: rate_code が無効
        
    Example:
        >>> motor.change_baud_rate(rate_code=0x02)  # 500K bps
    """
```

**C++ Reference:**
```cpp
void RobStride_Motor::RobStride_Motor_BaudRateChange(uint8_t F_CMD)
```

### 7.4 `set_proactive_reporting`

```python
def set_proactive_reporting(self, enable: bool) -> None:
    """
    自動状態報告の有効/無効
    
    Args:
        enable: True=有効, False=無効
        
    Note:
        有効時、モーターは 10ms 間隔で自動的に状態を送信
        
    Communication Type: 0x18
    
    Example:
        >>> motor.set_proactive_reporting(enable=True)
    """
```

**C++ Reference:**
```cpp
void RobStride_Motor::RobStride_Motor_ProactiveEscalationSet(uint8_t F_CMD)
```

### 7.5 `set_motor_mode`

```python
def set_motor_mode(self, protocol_mode: int) -> None:
    """
    プロトコルモード切り替え
    
    Args:
        protocol_mode:
            0x00 = Private (RobStride)
            0x01 = CANopen
            0x02 = MIT
            
    Communication Type: 0x19
    
    Warning:
        切り替え後はモーター再起動が必要
        
    Raises:
        ValueError: protocol_mode が無効
        
    Example:
        >>> motor.set_motor_mode(protocol_mode=0x02)  # MIT に切り替え
    """
```

**C++ Reference:**
```cpp
void RobStride_Motor::RobStride_Motor_MotorModeSet(uint8_t F_CMD)
void RobStride_Motor::RobStride_Motor_MIT_MotorModeSet(uint8_t F_CMD)
```

---

## 8. Status & Feedback / 状態取得

### 8.1 `process_can_message`

```python
def process_can_message(
    self,
    can_id: int,
    data: bytes,
    is_extended: bool
) -> None:
    """
    受信 CAN メッセージを解析
    
    Args:
        can_id: CAN ID (標準 or 拡張)
        data: 8-byte payload
        is_extended: True=Extended ID, False=Standard ID
        
    Note:
        - 内部で pos_info, drw, error_code 等を更新
        - 通常は CAN 受信スレッドから自動的に呼ばれる
        
    Example:
        >>> # Manual processing (normally automatic)
        >>> msg = can_bus.recv(timeout=0.1)
        >>> motor.process_can_message(
        ...     can_id=msg.arbitration_id,
        ...     data=msg.data,
        ...     is_extended=msg.is_extended_id
        ... )
    """
```

**C++ Reference:**
```cpp
void RobStride_Motor::RobStride_Motor_Analysis(uint8_t *DataFrame, uint32_t ID_ExtId)
```

---

## 9. Properties / プロパティ

### 9.1 Read-Only Status Properties

```python
@property
def angle(self) -> float:
    """現在角度 [rad] (読み取り専用)"""
    return self.pos_info.angle

@property
def speed(self) -> float:
    """現在速度 [rad/s] (読み取り専用)"""
    return self.pos_info.speed

@property
def torque(self) -> float:
    """現在トルク [Nm] (読み取り専用)"""
    return self.pos_info.torque

@property
def temperature(self) -> float:
    """現在温度 [°C] (読み取り専用)"""
    return self.pos_info.temperature

@property
def error_code(self) -> int:
    """エラーコード (8-bit, 読み取り専用)"""
    return self._error_code

@property
def is_enabled(self) -> bool:
    """モーター有効状態 (読み取り専用)"""
    return self.pos_info.pattern in [1, 2, 3]
```

### 9.2 Configuration Properties

```python
@property
def motor_id(self) -> int:
    """モーター CAN ID (読み取り専用)"""
    return self._motor_id

@property
def mit_mode(self) -> bool:
    """MIT モード使用フラグ (読み取り専用)"""
    return self._mit_mode

@property
def unique_id(self) -> int:
    """64-bit MCU Unique ID (読み取り専用)"""
    return self._unique_id
```

---

## 10. Exceptions / 例外

### 10.1 Exception Hierarchy

```python
class RobStrideError(Exception):
    """Base exception for RobStride library"""
    pass

class ProtocolError(RobStrideError):
    """Protocol mismatch or invalid protocol operation"""
    pass

class MotorError(RobStrideError):
    """Motor-related error (communication, state, etc.)"""
    pass

class CANError(RobStrideError):
    """CAN bus error"""
    pass

class TimeoutError(RobStrideError):
    """Operation timeout"""
    pass

class StateError(RobStrideError):
    """Invalid state for requested operation"""
    pass

class ParameterError(RobStrideError):
    """Invalid parameter value or address"""
    pass
```

### 10.2 Error Handling Example

```python
from robstride import RobStrideMotor, MotorError, TimeoutError

try:
    motor = RobStrideMotor(can_id=0x01, can_interface='can0')
    motor.enable_motor()
    motor.position_control(target_angle=1.57, target_speed=3.0)
    
except CANError as e:
    print(f"CAN interface error: {e}")
except MotorError as e:
    print(f"Motor error: {e}")
    motor.disable_motor(clear_error=True)
except TimeoutError as e:
    print(f"Timeout: {e}")
finally:
    motor.disable_motor()
```

---

## 11. Context Manager Support / コンテキストマネージャー

```python
class RobStrideMotor:
    def __enter__(self):
        """
        Context manager entry
        
        自動的にモーターを有効化
        """
        self.enable_motor()
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        """
        Context manager exit
        
        自動的にモーターを無効化
        """
        self.disable_motor()
        return False

# Usage example
with RobStrideMotor(can_id=0x01, can_interface='can0') as motor:
    motor.position_control(target_angle=1.57)
    time.sleep(1.0)
# Motor automatically disabled here
```

---

## 12. Async Support (Optional) / 非同期サポート（オプション）

```python
class AsyncRobStrideMotor:
    """
    非同期版 RobStride モーター制御
    
    asyncio と組み合わせて使用
    """
    
    async def enable_motor(self) -> None:
        """非同期モーター有効化"""
        pass
        
    async def position_control_async(
        self,
        target_angle: float,
        target_speed: float = 2.0
    ) -> None:
        """非同期位置制御"""
        pass
        
    async def get_parameter_async(
        self,
        index: int,
        timeout: float = 0.1
    ) -> Union[float, int]:
        """非同期パラメータ取得"""
        pass

# Usage example
import asyncio

async def main():
    motor = AsyncRobStrideMotor(can_id=0x01)
    await motor.enable_motor()
    await motor.position_control_async(target_angle=1.57)
    await motor.disable_motor()

asyncio.run(main())
```

---

## Appendix A: Complete API Reference Table

| Method                      | Mode        | Async | Description               |
| --------------------------- | ----------- | ----- | ------------------------- |
| `__init__`                  | Both        | No    | Constructor               |
| `enable_motor`              | Both        | No    | Enable motor              |
| `disable_motor`             | Both        | No    | Disable motor             |
| `set_zero_position`         | Both        | No    | Set zero position         |
| `motion_control`            | Private     | No    | Composite motion control  |
| `position_control`          | Private     | No    | PP position mode          |
| `csp_control`               | Both        | No    | CSP position mode         |
| `speed_control`             | Private     | No    | Speed mode                |
| `current_control`           | Private     | No    | Current mode              |
| `mit_control`               | MIT         | No    | MIT composite control     |
| `mit_position_control`      | MIT         | No    | MIT position control      |
| `mit_speed_control`         | MIT         | No    | MIT speed control         |
| `mit_set_zero_position`     | MIT         | No    | MIT zero setting          |
| `mit_clear_error`           | MIT         | No    | MIT error clear           |
| `mit_set_motor_type`        | MIT         | No    | MIT motor type setting    |
| `mit_set_motor_id`          | MIT         | No    | MIT ID setting            |
| `set_parameter`             | Private     | No    | Write parameter           |
| `get_parameter`             | Private     | No    | Read parameter (async)    |
| `get_parameter_sync`        | Private     | No    | Read parameter (sync)     |
| `save_parameters`           | Private     | No    | Save to FLASH             |
| `get_can_id`                | Private     | No    | Get CAN ID & Unique ID    |
| `set_can_id`                | Private     | No    | Set CAN ID                |
| `change_baud_rate`          | Private     | No    | Change baud rate          |
| `set_proactive_reporting`   | Private     | No    | Enable/disable reporting  |
| `set_motor_mode`            | Both        | No    | Switch protocol           |
| `process_can_message`       | Both        | No    | Parse received CAN msg    |

---

**End of API Specification**
