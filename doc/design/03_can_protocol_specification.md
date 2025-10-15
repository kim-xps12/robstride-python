# RobStride Motor Control Library - CAN Protocol Specification
# RobStride モーター制御ライブラリ - CAN プロトコル仕様書

**Document Version:** 1.0  
**Date:** 2025-10-09  
**Protocols:** RobStride Private Protocol, MIT Protocol

---

## Table of Contents / 目次

1. [Protocol Overview](#1-protocol-overview--プロトコル概要)
2. [Private Protocol](#2-private-protocol--private-プロトコル)
3. [MIT Protocol](#3-mit-protocol--mit-プロトコル)
4. [Data Encoding](#4-data-encoding--データエンコーディング)
5. [Message Examples](#5-message-examples--メッセージ例)
6. [Timing & Sequence](#6-timing--sequence--タイミングとシーケンス)

---

## 1. Protocol Overview / プロトコル概要

### 1.1 Supported Protocols

RobStride RS01 モーターは以下のプロトコルをサポート：

| Protocol | CAN ID Type | Typical Use Case            |
| -------- | ----------- | --------------------------- |
| Private  | Extended    | 高機能制御、パラメータ設定         |
| MIT      | Standard    | シンプル制御、リアルタイム性重視    |
| CANopen  | Standard    | 標準化プロトコル（詳細仕様は別途） |

**Note:** 本ドキュメントは Private および MIT プロトコルのみを扱う。

### 1.2 Protocol Switching

```
Initial State → Set Protocol Mode → Power Cycle Motor → New Protocol Active
```

- **Private → MIT:** `set_motor_mode(0x02)` 実行後、モーター再起動
- **MIT → Private:** MIT モードで `mit_motor_mode_set(0x00)` 実行後、モーター再起動

---

## 2. Private Protocol / Private プロトコル

### 2.1 CAN Message Format

#### 2.1.1 Extended CAN ID Structure (29-bit)

```
Bit 28-24 | Bit 23-16      | Bit 15-8     | Bit 7-0
----------|----------------|--------------|----------
CommType  | ErrorCode/Data | Master_ID    | Motor_ID
(5 bits)  | (8 bits)       | (8 bits)     | (8 bits)
```

**Field Descriptions:**

- **CommType [28:24]:** Communication Type (0x00-0x19, see section 2.2)
- **ErrorCode/Data [23:16]:** 
  - For TX (host→motor): Additional data (e.g., torque in motion control)
  - For RX (motor→host): Error code or control pattern
- **Master_ID [15:8]:** Host/Master CAN ID (default: 0xFD)
- **Motor_ID [7:0]:** Target motor CAN ID (0x00-0x7F)

**Python Construction Example:**
```python
ext_id = (comm_type << 24) | (data_byte << 16) | (master_id << 8) | motor_id
```

#### 2.1.2 Data Payload

- **DLC:** Always 8 bytes
- **Byte Order:** Little-endian for multi-byte values
- **Format:** Depends on Communication Type

### 2.2 Communication Types

| Type | Name                        | Direction | Description                    |
| ---- | --------------------------- | --------- | ------------------------------ |
| 0x00 | Get_ID                      | Host→Mot  | Get CAN ID & Unique ID         |
| 0x01 | MotionControl               | Host→Mot  | Composite motion control       |
| 0x02 | MotorRequest                | Mot→Host  | Motor status feedback          |
| 0x03 | MotorEnable                 | Host→Mot  | Enable motor                   |
| 0x04 | MotorStop                   | Host→Mot  | Disable motor                  |
| 0x06 | SetPosZero                  | Host→Mot  | Set zero position              |
| 0x07 | Can_ID                      | Host→Mot  | Set CAN ID                     |
| 0x11 | GetSingleParameter          | Host→Mot  | Read parameter                 |
| 0x12 | SetSingleParameter/Mode     | Host→Mot  | Write parameter                |
| 0x15 | ErrorFeedback               | Mot→Host  | Error report                   |
| 0x16 | MotorDataSave               | Host→Mot  | Save parameters to FLASH       |
| 0x17 | BaudRateChange              | Host→Mot  | Change CAN baud rate           |
| 0x18 | ProactiveEscalationSet      | Host→Mot  | Enable/disable auto-reporting  |
| 0x19 | MotorModeSet                | Host→Mot  | Switch protocol mode           |

### 2.3 Detailed Message Formats

#### 2.3.1 Get CAN ID (Type 0x00)

**Request (Host → Motor):**
```
Extended ID: 0x00 << 24 | 0xFD << 8 | motor_id
Data[8]:     [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
```

**Response (Motor → Host):**
```
Extended ID: 0xFE (in Motor_ID field) | motor_id << 8
Data[8]:     [Unique_ID[0:7]]  // 64-bit MCU Unique ID, little-endian
```

#### 2.3.2 Motion Control (Type 0x01)

**Request (Host → Motor):**
```
Extended ID: 0x01 << 24 | torque_scaled << 16 | 0xFD << 8 | motor_id
Data[0:1]:   angle (16-bit, scaled)
Data[2:3]:   speed (16-bit, scaled)
Data[4:5]:   Kp (16-bit, scaled)
Data[6:7]:   Kd (16-bit, scaled)
```

**Scaling (see section 4.1):**
- Torque: -4.0 ~ 4.0 Nm → 16-bit (in ExtID[23:16])
- Angle: -12.5 ~ 12.5 rad → 16-bit
- Speed: -30 ~ 30 rad/s → 16-bit
- Kp: 0 ~ 500 → 16-bit
- Kd: 0 ~ 5 → 16-bit

#### 2.3.3 Motor Status Feedback (Type 0x02)

**Response (Motor → Host):**
```
Extended ID: 0x02 << 24 | error_code << 16 | pattern << 22 | motor_id << 8
Data[0:1]:   angle (16-bit, scaled)
Data[2:3]:   speed (16-bit, scaled)
Data[4:5]:   torque (16-bit, scaled)
Data[6:7]:   temperature (16-bit, 0.1°C resolution)
```

**ExtID Fields:**
- `error_code [23:16]`: 8-bit error flags (see section 8)
- `pattern [23:22]`: Control mode (0=torque, 1=position, 2=speed, 3=running)

**Scaling:**
- Angle: -12.5 ~ 12.5 rad
- Speed: -44 ~ 44 rad/s
- Torque: -17 ~ 17 Nm
- Temperature: value * 0.1 °C

#### 2.3.4 Motor Enable (Type 0x03)

**Request (Host → Motor):**
```
Extended ID: 0x03 << 24 | 0xFD << 8 | motor_id
Data[8]:     [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
```

#### 2.3.5 Motor Disable (Type 0x04)

**Request (Host → Motor):**
```
Extended ID: 0x04 << 24 | 0xFD << 8 | motor_id
Data[0]:     clear_error (0x00=no clear, 0x01=clear error)
Data[1:7]:   [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
```

#### 2.3.6 Set Zero Position (Type 0x06)

**Request (Host → Motor):**
```
Extended ID: 0x06 << 24 | 0xFD << 8 | motor_id
Data[0]:     0x01
Data[1:7]:   [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
```

**Precondition:** Motor must be disabled first

#### 2.3.7 Set CAN ID (Type 0x07)

**Request (Host → Motor):**
```
Extended ID: 0x07 << 24 | new_id << 16 | 0xFD << 8 | current_motor_id
Data[8]:     [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
```

**Note:** Motor restart required after ID change

#### 2.3.8 Get Single Parameter (Type 0x11)

**Request (Host → Motor):**
```
Extended ID: 0x11 << 24 | 0xFD << 8 | motor_id
Data[0:1]:   index (16-bit, little-endian, e.g., 0x7005)
Data[2:7]:   [0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
```

**Response (Motor → Host):**
```
Extended ID: 0x11 << 24 | motor_id << 8
Data[0:1]:   index (16-bit, little-endian)
Data[2:3]:   [0x00, 0x00]
Data[4:7]:   value (float32, little-endian OR uint8 for mode parameters)
```

#### 2.3.9 Set Single Parameter (Type 0x12)

**Request (Host → Motor):**
```
Extended ID: 0x12 << 24 | 0xFD << 8 | motor_id
Data[0:1]:   index (16-bit, little-endian)
Data[2:3]:   [0x00, 0x00]
Data[4:7]:   value (float32 for 'p' mode, uint8 for 'j' mode)
```

**Value Modes:**
- `'p'` (parameter): Data[4:7] = float32 (e.g., 5.0 for speed)
- `'j'` (mode): Data[4] = uint8 (e.g., 1 for position mode), Data[5:7] = 0x00

#### 2.3.10 Save Parameters (Type 0x16)

**Request (Host → Motor):**
```
Extended ID: 0x16 << 24 | 0xFD << 8 | motor_id
Data[8]:     [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08]
             (Magic sequence for confirmation)
```

#### 2.3.11 Change Baud Rate (Type 0x17)

**Request (Host → Motor):**
```
Extended ID: 0x17 << 24 | 0xFD << 8 | motor_id
Data[6]:     baud_rate_code
             0x01 = 1M bps
             0x02 = 500K bps
             0x03 = 250K bps
             0x04 = 125K bps
Data[0:5,7]: [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x08]
```

#### 2.3.12 Proactive Reporting (Type 0x18)

**Request (Host → Motor):**
```
Extended ID: 0x18 << 24 | 0xFD << 8 | motor_id
Data[6]:     mode
             0x00 = Disable auto-reporting
             0x01 = Enable auto-reporting (10ms interval)
Data[0:5,7]: [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x08]
```

#### 2.3.13 Protocol Mode Set (Type 0x19)

**Request (Host → Motor):**
```
Extended ID: 0x19 << 24 | 0xFD << 8 | motor_id
Data[6]:     protocol_mode
             0x00 = Private (RobStride)
             0x01 = CANopen
             0x02 = MIT
Data[0:5,7]: [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x08]
```

---

## 3. MIT Protocol / MIT プロトコル

### 3.1 CAN Message Format

#### 3.1.1 Standard CAN ID Structure (11-bit)

**For most commands:**
```
Standard ID: motor_id (0x00-0x7F)
```

**For position/speed control:**
```
Standard ID: (command_type << 8) | motor_id
```
- `command_type = 1`: Position control
- `command_type = 2`: Speed control

#### 3.1.2 Data Payload

- **DLC:** Always 8 bytes
- **Encoding:** Depends on command (see below)

### 3.2 MIT Command Messages

#### 3.2.1 MIT Enable

**Request (Host → Motor):**
```
Standard ID: motor_id
Data[8]:     [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]
```

#### 3.2.2 MIT Disable

**Request (Host → Motor):**
```
Standard ID: motor_id
Data[8]:     [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]
```

#### 3.2.3 MIT Composite Control

**Request (Host → Motor):**
```
Standard ID: motor_id
Data[0:1]:   angle (16-bit, scaled -12.5~12.5 rad)
Data[2]:     speed (12-bit MSB, scaled -44~44 rad/s)
Data[3]:     speed (4-bit LSB) | Kp (12-bit MSB, scaled 0~500)
Data[4]:     Kp (8-bit LSB)
Data[5]:     Kd (12-bit MSB, scaled 0~5)
Data[6]:     Kd (4-bit LSB) | torque (12-bit MSB, scaled -17~17 Nm)
Data[7]:     torque (8-bit LSB)
```

**Bit Packing Details:**
```
Byte 0-1: p_int (16-bit)
Byte 2:   v_int[11:4]
Byte 3:   v_int[3:0] (upper 4 bits) | kp_int[11:8] (lower 4 bits)
Byte 4:   kp_int[7:0]
Byte 5:   kd_int[11:4]
Byte 6:   kd_int[3:0] (upper 4 bits) | t_int[11:8] (lower 4 bits)
Byte 7:   t_int[7:0]
```

**Scaling:**
```python
p_int  = float_to_uint(angle, -12.5, 12.5, 16)
v_int  = float_to_uint(speed, -44.0, 44.0, 12)
kp_int = float_to_uint(kp, 0.0, 500.0, 12)
kd_int = float_to_uint(kd, 0.0, 5.0, 12)
t_int  = float_to_uint(torque, -17.0, 17.0, 12)
```

#### 3.2.4 MIT Position Control

**Request (Host → Motor):**
```
Standard ID: (1 << 8) | motor_id  = 0x100 | motor_id
Data[0:3]:   position (float32, little-endian, rad)
Data[4:7]:   speed (float32, little-endian, rad/s)
```

**Example:**
```python
import struct
can_id = (1 << 8) | 0x01  # motor_id=0x01
data = struct.pack('<ff', 1.57, 3.0)  # position=1.57, speed=3.0
```

#### 3.2.5 MIT Speed Control

**Request (Host → Motor):**
```
Standard ID: (2 << 8) | motor_id  = 0x200 | motor_id
Data[0:3]:   speed (float32, little-endian, rad/s)
Data[4:7]:   current_limit (float32, little-endian, A)
```

#### 3.2.6 MIT Set Zero Position

**Request (Host → Motor):**
```
Standard ID: motor_id
Data[8]:     [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE]
```

**Precondition:** MIT_Type != positionControl

#### 3.2.7 MIT Clear/Check Error

**Request (Host → Motor):**
```
Standard ID: motor_id
Data[6]:     command
             0x00 = Check error
             0x01 = Clear error
Data[0:5,7]: [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB]
```

#### 3.2.8 MIT Set Motor Type

**Request (Host → Motor):**
```
Standard ID: motor_id
Data[6]:     motor_type
             0x01 = operationControl
             0x02 = positionControl
             0x03 = speedControl
Data[0:5,7]: [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]
```

#### 3.2.9 MIT Set Motor ID

**Request (Host → Motor):**
```
Standard ID: current_motor_id
Data[6]:     new_motor_id (0x00-0x7F)
Data[0:5,7]: [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x01]
```

#### 3.2.10 MIT Protocol Mode Set

**Request (Host → Motor):**
```
Standard ID: motor_id
Data[6]:     protocol_mode
             0x00 = Private
             0x01 = CANopen
             0x02 = MIT
Data[0:5,7]: [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]
```

### 3.3 MIT Status Feedback

**Response (Motor → Host):**
```
Standard ID: 0xFD (broadcast) or motor_id
Data[0:1]:   angle (16-bit, scaled -12.5~12.5 rad)
Data[2]:     speed (12-bit MSB, scaled -44~44 rad/s)
Data[3]:     speed (4-bit LSB) | torque (12-bit MSB, scaled -17~17 Nm)
Data[4]:     torque (8-bit LSB)
Data[5]:     reserved
Data[6:7]:   temperature (16-bit, 0.1°C resolution)
```

**Special Case - Error Only:**
```
If Data[3:7] == 0x00:
    Data[1:2] = fault code (16-bit)
    Decoded to 8-bit error code via mapFaults()
```

---

## 4. Data Encoding / データエンコーディング

### 4.1 Scaling Functions

#### 4.1.1 Float to Unsigned Integer

```python
def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    """
    Float を固定ビット幅の符号なし整数にスケール
    
    Args:
        x: 入力値
        x_min: 最小値
        x_max: 最大値
        bits: ビット幅
        
    Returns:
        スケールされた整数値
    """
    span = x_max - x_min
    offset = x_min
    
    # Clamp
    if x > x_max:
        x = x_max
    elif x < x_min:
        x = x_min
    
    return int((x - offset) * ((1 << bits) - 1) / span)
```

**Example:**
```python
# Angle: 1.57 rad → 16-bit
angle_int = float_to_uint(1.57, -12.5, 12.5, 16)
# Result: 34816

# Speed: 10.0 rad/s → 12-bit
speed_int = float_to_uint(10.0, -44.0, 44.0, 12)
# Result: 2559
```

#### 4.1.2 Unsigned Integer to Float

```python
def uint_to_float(x: int, x_min: float, x_max: float, bits: int) -> float:
    """
    符号なし整数を Float にデスケール
    
    Args:
        x: 整数値
        x_min: 最小値
        x_max: 最大値
        bits: ビット幅
        
    Returns:
        デスケールされた float 値
    """
    span = (1 << bits) - 1
    x &= span  # Mask to bit width
    offset = x_max - x_min
    return offset * x / span + x_min
```

**Example:**
```python
# 34816 (16-bit) → angle
angle = uint_to_float(34816, -12.5, 12.5, 16)
# Result: 1.57 rad

# 2559 (12-bit) → speed
speed = uint_to_float(2559, -44.0, 44.0, 12)
# Result: 10.0 rad/s
```

### 4.2 Byte Order

- **Private Protocol:** Little-endian for multi-byte integers and floats
- **MIT Protocol:** 
  - Little-endian for float32 (position/speed control)
  - Big-endian (MSB first) for packed scaled values (composite control)

### 4.3 Float32 Encoding (IEEE 754)

```python
import struct

# Encode
data = struct.pack('<f', 1.57)  # Little-endian float32
# Result: b'\xc3\xf5\xc8\x3f'

# Decode
value = struct.unpack('<f', data)[0]
# Result: 1.57
```

---

## 5. Message Examples / メッセージ例

### 5.1 Private Protocol Examples

#### Example 1: Enable Motor (ID=0x01)

```
TX (Host → Motor):
  Extended ID: 0x0300FD01
    CommType: 0x03
    Data:     0x00
    Master:   0xFD
    Motor:    0x01
  DLC:  8
  Data: 00 00 00 00 00 00 00 00
```

#### Example 2: Position Control (Target=1.57 rad, Speed=3.0 rad/s)

Step 1: Set mode to position (1)
```
TX:
  Extended ID: 0x1200FD01
  Data: 05 70 00 00 01 00 00 00
        ^  ^        ^
        |  |        └─ mode=1 (position)
        └──┴─ index=0x7005
```

Step 2: Write target angle
```
TX:
  Extended ID: 0x1200FD01
  Data: 16 70 00 00 C3 F5 C8 3F
        ^  ^        ^---------^
        |  |        └─ 1.57 (float32, little-endian)
        └──┴─ index=0x7016
```

#### Example 3: Motion Control (Torque=2.0, Angle=0, Kp=100)

```
TX:
  Extended ID: 0x01800FD01
    CommType: 0x01
    Torque:   0x80 (scaled 2.0 Nm)
    Master:   0xFD
    Motor:    0x01
  Data: 00 80 00 00 CD 4C 00 00
        ^  ^  ^  ^  ^  ^  ^  ^
        |  |  |  |  |  |  |  └─ Kd=0 (LSB)
        |  |  |  |  |  └──┴─ Kd=0 (MSB) + Kp=19661 (LSB)
        |  |  |  |  └─ Kp=19661 (scaled 100.0)
        |  |  └──┴─ Speed=0
        └──┴─ Angle=0 (scaled, 16-bit)
```

### 5.2 MIT Protocol Examples

#### Example 1: MIT Enable (ID=0x02)

```
TX:
  Standard ID: 0x02
  DLC:  8
  Data: FF FF FF FF FF FF FF FC
```

#### Example 2: MIT Position Control (Position=1.57, Speed=3.0)

```
TX:
  Standard ID: 0x102 (256 + 2)
  DLC:  8
  Data: C3 F5 C8 3F 00 00 40 40
        ^---------^ ^---------^
        |           └─ speed=3.0 (float32)
        └─ position=1.57 (float32)
```

#### Example 3: MIT Composite Control

```
Inputs:
  angle = 0.5 rad
  speed = 10.0 rad/s
  kp = 100.0
  kd = 1.0
  torque = -1.0 Nm

Scaled:
  p_int  = 34406 (0x8666)
  v_int  = 2559  (0x9FF)
  kp_int = 819   (0x333)
  kd_int = 819   (0x333)
  t_int  = 1706  (0x6AA)

TX:
  Standard ID: 0x02
  Data: 66 86 9F F3 33 33 36 AA
        ^  ^  ^  ^  ^  ^  ^  ^
        └──┴─ p_int (16-bit)
           └─ v_int[11:4]
              └─ v_int[3:0] | kp_int[11:8]
                 └─ kp_int[7:0]
                    └─ kd_int[11:4]
                       └─ kd_int[3:0] | t_int[11:8]
                          └─ t_int[7:0]
```

---

## 6. Timing & Sequence / タイミングとシーケンス

### 6.1 Command Timing

| Operation             | Minimum Interval | Typical Interval |
| --------------------- | ---------------- | ---------------- |
| Enable/Disable        | 50 ms            | 100 ms           |
| Parameter Write       | 1 ms             | 5 ms             |
| Control Command       | 1 ms             | 10 ms (100 Hz)   |
| Parameter Read        | 5 ms             | 10 ms            |
| Status Feedback (RX)  | -                | 10 ms (auto)     |

### 6.2 Typical Sequences

#### 6.2.1 Initialization Sequence

```
1. Create motor instance
2. Enable motor (wait 100ms)
3. (Optional) Set zero position
4. Set control mode if needed
5. Ready for control commands
```

#### 6.2.2 Position Control Sequence (Private)

```
1. Set run_mode to 1 (position) via 0x7005
2. (Optional) Set speed limit via 0x7024
3. Write target position to 0x7016
4. Monitor status feedback (Type 0x02)
5. Repeat step 3 for new targets
```

#### 6.2.3 Position Control Sequence (MIT)

```
1. Enable motor (0xFC command)
2. (Optional) Set motor type to position (0x02)
3. Send position control command (ID 0x100 | motor_id)
4. Monitor status feedback
5. Repeat step 3 for new targets
```

#### 6.2.4 Protocol Switch Sequence

```
1. Disable motor
2. Send protocol mode set command (Type 0x19 or MIT 0xFD)
3. Power-cycle motor (hardware reset)
4. Motor boots in new protocol mode
5. Re-initialize motor instance with new protocol
```

### 6.3 Error Handling Sequence

```
1. Detect error (error_code != 0)
2. Log error details
3. Disable motor
4. (Optional) Clear error flag
5. Wait for condition to be resolved
6. Re-enable motor
```

### 6.4 Response Timeout

Recommended timeouts for parameter read operations:

- **Parameter Read:** 100 ms
- **CAN ID Request:** 100 ms
- **Status Feedback:** 50 ms (with auto-reporting enabled)

If no response within timeout → raise TimeoutError

---

## 7. CAN Bus Configuration / CAN バス設定

### 7.1 Physical Layer

- **Standard:** CAN 2.0B (ISO 11898-1)
- **Termination:** 120Ω resistors at both ends
- **Cable:** Twisted pair, max length depends on baud rate

### 7.2 Baud Rates

| Baud Rate | Code | Max Bus Length | Typical Use         |
| --------- | ---- | -------------- | ------------------- |
| 1M bps    | 0x01 | ~40 m          | High-speed control  |
| 500K bps  | 0x02 | ~100 m         | Standard            |
| 250K bps  | 0x03 | ~250 m         | Long distance       |
| 125K bps  | 0x04 | ~500 m         | Very long distance  |

**Default:** 1M bps (factory setting)

### 7.3 Bus Arbitration

- **Extended ID (Private):** Lower priority than Standard ID
- **Standard ID (MIT):** Higher priority
- **Collision Resolution:** Automatic (CAN protocol)

### 7.4 Error Detection

CAN protocol provides:
- **CRC:** 15-bit cyclic redundancy check
- **ACK:** Acknowledgment checking
- **Bit Monitoring:** Transmitter checks its own transmission
- **Frame Check:** Valid frame format checking

---

## 8. Error Codes / エラーコード

### 8.1 Private Protocol Error Code (8-bit)

| Bit | Name           | Description                |
| --- | -------------- | -------------------------- |
| 0   | Under-voltage  | Bus voltage too low        |
| 1   | Over-current   | Current exceeds limit      |
| 2   | Over-temp      | Temperature too high       |
| 3   | Encoder error  | Encoder signal fault       |
| 4   | Over-voltage   | Bus voltage too high       |
| 5   | Not calibrated | Motor not calibrated       |
| 6-7 | Reserved       | -                          |

### 8.2 MIT Protocol Fault Mapping

MIT 16-bit fault code is mapped to 8-bit error code:

```python
def map_faults(fault16: int) -> int:
    fault8 = 0
    if fault16 & (1 << 14): fault8 |= (1 << 4)  # Over-voltage
    if fault16 & (1 << 7):  fault8 |= (1 << 5)  # Not calibrated
    if fault16 & (1 << 3):  fault8 |= (1 << 3)  # Encoder error
    if fault16 & (1 << 2):  fault8 |= (1 << 0)  # Under-voltage
    if fault16 & (1 << 1):  fault8 |= (1 << 1)  # Over-current
    if fault16 & (1 << 0):  fault8 |= (1 << 2)  # Over-temp
    return fault8
```

---

## 9. Implementation Notes / 実装ノート

### 9.1 Python CAN Message Construction

```python
import can
import struct

# Private Protocol - Motion Control
def send_motion_control(bus, motor_id, torque, angle, speed, kp, kd):
    torque_scaled = float_to_uint(torque, -4.0, 4.0, 8)
    ext_id = (0x01 << 24) | (torque_scaled << 16) | (0xFD << 8) | motor_id
    
    data = bytearray(8)
    struct.pack_into('<H', data, 0, float_to_uint(angle, -12.5, 12.5, 16))
    struct.pack_into('<H', data, 2, float_to_uint(speed, -30.0, 30.0, 16))
    struct.pack_into('<H', data, 4, float_to_uint(kp, 0.0, 500.0, 16))
    struct.pack_into('<H', data, 6, float_to_uint(kd, 0.0, 5.0, 16))
    
    msg = can.Message(
        arbitration_id=ext_id,
        data=data,
        is_extended_id=True
    )
    bus.send(msg)

# MIT Protocol - Position Control
def send_mit_position_control(bus, motor_id, position, speed):
    std_id = (1 << 8) | motor_id
    data = struct.pack('<ff', position, speed)
    
    msg = can.Message(
        arbitration_id=std_id,
        data=data,
        is_extended_id=False
    )
    bus.send(msg)
```

### 9.2 Message Parsing

```python
def parse_status_feedback(msg: can.Message):
    if msg.is_extended_id:
        # Private protocol
        ext_id = msg.arbitration_id
        comm_type = (ext_id >> 24) & 0x1F
        
        if comm_type == 0x02:  # Status feedback
            error_code = (ext_id >> 16) & 0xFF
            pattern = (ext_id >> 22) & 0x03
            
            angle = uint_to_float(
                struct.unpack_from('<H', msg.data, 0)[0],
                -12.5, 12.5, 16
            )
            speed = uint_to_float(
                struct.unpack_from('<H', msg.data, 2)[0],
                -44.0, 44.0, 16
            )
            torque = uint_to_float(
                struct.unpack_from('<H', msg.data, 4)[0],
                -17.0, 17.0, 16
            )
            temp = struct.unpack_from('<H', msg.data, 6)[0] * 0.1
            
            return {
                'angle': angle,
                'speed': speed,
                'torque': torque,
                'temperature': temp,
                'error_code': error_code,
                'pattern': pattern
            }
    else:
        # MIT protocol
        if msg.arbitration_id == 0xFD or msg.arbitration_id < 0x80:
            # Check if error-only message
            if msg.data[3:8] == b'\x00\x00\x00\x00\x00':
                fault16 = struct.unpack_from('<H', msg.data, 1)[0]
                error_code = map_faults(fault16)
                return {'error_code': error_code}
            else:
                # Normal status
                p_int = struct.unpack_from('>H', msg.data, 0)[0]
                v_int = (msg.data[2] << 4) | (msg.data[3] >> 4)
                t_int = ((msg.data[3] & 0x0F) << 8) | msg.data[4]
                temp = struct.unpack_from('>H', msg.data, 6)[0] * 0.1
                
                angle = uint_to_float(p_int, -12.5, 12.5, 16)
                speed = uint_to_float(v_int, -44.0, 44.0, 12)
                torque = uint_to_float(t_int, -17.0, 17.0, 12)
                
                return {
                    'angle': angle,
                    'speed': speed,
                    'torque': torque,
                    'temperature': temp
                }
```

---

## Appendix A: Quick Reference Table

### A.1 Private Protocol Command Summary

| CommType | Name            | ExtID Format                            | Data[0]      | Data[4:7]        |
| -------- | --------------- | --------------------------------------- | ------------ | ---------------- |
| 0x00     | Get ID          | `0x00__FD[motor]`                       | 0x00         | 0x00             |
| 0x01     | Motion Control  | `0x01[torque]FD[motor]`                 | angle        | Kp               |
| 0x03     | Enable          | `0x03__FD[motor]`                       | 0x00         | 0x00             |
| 0x04     | Disable         | `0x04__FD[motor]`                       | clear_err    | 0x00             |
| 0x06     | Set Zero        | `0x06__FD[motor]`                       | 0x01         | 0x00             |
| 0x07     | Set CAN ID      | `0x07[new_id]FD[motor]`                 | 0x00         | 0x00             |
| 0x11     | Get Param       | `0x11__FD[motor]`                       | index_low    | 0x00             |
| 0x12     | Set Param       | `0x12__FD[motor]`                       | index_low    | value (float32)  |
| 0x16     | Save Params     | `0x16__FD[motor]`                       | 0x01         | 0x05             |
| 0x17     | Baud Rate       | `0x17__FD[motor]`                       | 0x01         | 0x05             |
| 0x18     | Auto Report     | `0x18__FD[motor]`                       | 0x01         | 0x05             |
| 0x19     | Protocol Switch | `0x19__FD[motor]`                       | 0x01         | 0x05             |

### A.2 MIT Protocol Command Summary

| StdID Pattern         | Data Pattern                      | Function              |
| --------------------- | --------------------------------- | --------------------- |
| `motor_id`            | `FF FF FF FF FF FF FF FC`         | Enable                |
| `motor_id`            | `FF FF FF FF FF FF FF FD`         | Disable               |
| `motor_id`            | `FF FF FF FF FF FF FF FE`         | Set Zero              |
| `motor_id`            | `FF FF FF FF FF FF [cmd] FB`      | Clear Error           |
| `motor_id`            | `FF FF FF FF FF FF [type] FC`     | Set Motor Type        |
| `motor_id`            | `FF FF FF FF FF FF [id] 01`       | Set Motor ID          |
| `motor_id`            | `FF FF FF FF FF FF [mode] FD`     | Protocol Switch       |
| `motor_id`            | `[p_int] [v_int] [kp] [kd] [t]`   | Composite Control     |
| `(1<<8) \| motor_id`  | `[pos:f32] [spd:f32]`             | Position Control      |
| `(2<<8) \| motor_id`  | `[spd:f32] [cur_lim:f32]`         | Speed Control         |

---

**End of CAN Protocol Specification**
