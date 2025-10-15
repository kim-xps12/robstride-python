# RobStride Motor Control Library - State Machine Design
# RobStride モーター制御ライブラリ - 状態機械設計

**Document Version:** 1.0  
**Date:** 2025-10-09

---

## 1. Overview / 概要

本ドキュメントは、RobStride モーター制御の状態遷移、初期化シーケンス、プロトコル切り替え、エラー時の挙動を定義する。

---

## 2. Motor States / モーター状態

### 2.1 Primary States

```
┌──────────────┐
│ UNINITIALIZED│  (初期状態、CAN 未接続)
└──────┬───────┘
       │ CAN Init
       ▼
┌──────────────┐
│   DISABLED   │  (CAN 接続済み、モーター無効)
└──┬─────────┬─┘
   │ Enable  │
   ▼         │
┌──────────────┐  Disable
│   ENABLED    │◄─────────┐
└──┬───────────┘          │
   │ Start Control        │
   ▼                      │
┌──────────────┐          │
│   RUNNING    │──────────┘
└──┬───────────┘
   │ Error
   ▼
┌──────────────┐
│   FAULT      │  (エラー状態、手動復旧必要)
└──────────────┘
```

### 2.2 State Descriptions

#### UNINITIALIZED (未初期化)
- **Entry Condition:** プログラム起動直後
- **Exit Condition:** CAN インターフェース初期化完了
- **Allowed Operations:** 初期化関数のみ
- **Python Implementation:**
  ```python
  class MotorState(Enum):
      UNINITIALIZED = 0
      DISABLED = 1
      ENABLED = 2
      RUNNING = 3
      FAULT = 4
  ```

#### DISABLED (無効化)
- **Entry Condition:** 
  - UNINITIALIZED → CAN 初期化完了
  - ENABLED/RUNNING → `Disable_Motor()` 呼び出し
- **Exit Condition:** `Enable_Motor()` 呼び出し
- **Allowed Operations:**
  - パラメータ設定（一部制限あり）
  - プロトコル切り替え
  - ゼロ点設定
- **Characteristics:**
  - モーターはフリー（トルク OFF）
  - CAN 通信可能
  - 安全状態

#### ENABLED (有効化)
- **Entry Condition:** DISABLED → `Enable_Motor()` 成功
- **Exit Condition:** 
  - `Disable_Motor()` → DISABLED
  - 制御コマンド受信 → RUNNING
  - エラー発生 → FAULT
- **Allowed Operations:**
  - 全パラメータ設定
  - 全読み取りコマンド
  - 制御コマンド送信
- **Characteristics:**
  - モーターは保持トルク ON
  - フィードバック受信可能
  - 制御ループ準備完了

#### RUNNING (運転中)
- **Entry Condition:** ENABLED → 制御コマンド（位置/速度/電流）送信
- **Exit Condition:**
  - `Disable_Motor()` → DISABLED
  - 全指令値が 0 で一定時間 → ENABLED
  - エラー発生 → FAULT
- **Allowed Operations:**
  - 全 ENABLED 状態の操作
  - リアルタイム制御コマンド更新
- **Characteristics:**
  - モーターが動作中
  - フィードバック高速更新
  - 負荷トルク発生

#### FAULT (故障)
- **Entry Condition:** エラー検出時
- **Exit Condition:** 
  - `Clear_Errors()` → DISABLED（エラー解消済み）
  - `Restart_Motor()` → DISABLED（強制リセット）
- **Allowed Operations:**
  - エラー状態読み取り
  - エラークリア
  - 緊急停止
- **Characteristics:**
  - モーター即座停止
  - 安全状態
  - CAN 通信は継続

---

## 3. State Transition Sequences / 状態遷移シーケンス

### 3.1 Normal Startup Sequence

```python
# 1. UNINITIALIZED → DISABLED
can_bus = can.interface.Bus(channel='can0', bustype='socketcan')
motor = RobStrideMotor(can_id=1, can_bus=can_bus)
# State: DISABLED

# 2. DISABLED → ENABLED
motor.enable_motor()
time.sleep(0.1)  # Wait for confirmation
# State: ENABLED

# 3. ENABLED → RUNNING
motor.set_parameter(0x7005, 2, value_mode='j')  # Speed mode
motor.set_parameter(0x700A, 10.0, value_mode='p')  # 10 rad/s
# State: RUNNING

# 4. RUNNING → DISABLED (graceful stop)
motor.set_parameter(0x700A, 0.0, value_mode='p')  # Zero speed
time.sleep(2.0)  # Wait for deceleration
motor.disable_motor()
# State: DISABLED
```

### 3.2 Mode Change Sequence

**Important:** モード変更時は必ず無効化→再有効化が必要

```python
# Current state: RUNNING (Mode 2: Speed control)

# 1. Disable motor
motor.disable_motor()
time.sleep(0.1)
# State: DISABLED

# 2. Change mode
motor.set_parameter(0x7005, 1, value_mode='j')  # Position mode
time.sleep(0.05)

# 3. Re-enable
motor.enable_motor()
time.sleep(0.1)
# State: ENABLED

# 4. Send new command
motor.set_parameter(0x7016, 1.57, value_mode='p')  # Position target
# State: RUNNING
```

### 3.3 Emergency Stop Sequence

```python
# Any state (except FAULT)

# 1. Immediate disable
motor.disable_motor()
# State: DISABLED

# 2. (Optional) Clear any pending commands
motor.stop_mit_mode()  # If in MIT protocol

# 3. Read error status
motor.get_parameter(0x701E)  # Error code
time.sleep(0.05)

# 4. If errors exist
if motor.drw.error_flags != 0:
    motor.clear_errors()
    # State: DISABLED (if clear successful)
```

### 3.4 Error Recovery Sequence

```python
# State: FAULT

# 1. Read error details
motor.get_parameter(0x701E)
time.sleep(0.05)
error_code = motor.drw.error_flags

# 2. Analyze error
if error_code & 0x01:  # Over-temperature
    print("Cooling required")
    time.sleep(60)  # Wait for cooling
elif error_code & 0x02:  # Over-current
    print("Reduce load or limit_cur")
    motor.set_parameter(0x7018, 5.0, value_mode='p')

# 3. Clear errors
motor.clear_errors()
time.sleep(0.1)
# State: DISABLED (if successful)

# 4. Re-enable if safe
if is_safe_to_enable():
    motor.enable_motor()
    # State: ENABLED
```

---

## 4. Control Mode State Machines / 制御モード別状態機械

### 4.1 Position Control (PP Mode) State Machine

```
┌─────────────┐
│   INIT      │  (mode != 1)
└──────┬──────┘
       │ set_parameter(0x7005, 1)
       ▼
┌─────────────┐
│ PP_INACTIVE │  (mode == 1, disabled)
└──┬───────┬──┘
   │ enable│
   ▼       │
┌─────────────┐  disable
│  PP_IDLE    │◄────────────┐
└──┬──────────┘             │
   │ write loc_ref          │
   ▼                        │
┌─────────────┐  target     │
│ PP_MOVING   │  reached    │
│             │─────────────┘
└─────────────┘
```

**Transitions:**
1. **INIT → PP_INACTIVE:** `set_parameter(0x7005, 1)`
2. **PP_INACTIVE → PP_IDLE:** `enable_motor()`
3. **PP_IDLE → PP_MOVING:** `set_parameter(0x7016, target)` with `|target - current| > threshold`
4. **PP_MOVING → PP_IDLE:** Position error < 0.01 rad for > 500 ms

**Python Implementation:**
```python
class PositionController:
    def __init__(self, motor):
        self.motor = motor
        self.state = "INIT"
    
    def setup_mode(self):
        """INIT → PP_INACTIVE"""
        self.motor.set_parameter(0x7005, 1, value_mode='j')
        self.motor.set_parameter(0x7024, 3.0, value_mode='p')  # Speed limit
        self.motor.set_parameter(0x7025, 5.0, value_mode='p')  # Acceleration
        self.state = "PP_INACTIVE"
    
    def activate(self):
        """PP_INACTIVE → PP_IDLE"""
        if self.state != "PP_INACTIVE":
            raise StateError("Must be in PP_INACTIVE state")
        self.motor.enable_motor()
        time.sleep(0.1)
        self.state = "PP_IDLE"
    
    def move_to(self, target_rad):
        """PP_IDLE → PP_MOVING"""
        if self.state != "PP_IDLE":
            raise StateError("Must be in PP_IDLE state")
        self.motor.set_parameter(0x7016, target_rad, value_mode='p')
        self.state = "PP_MOVING"
    
    def check_arrived(self, threshold=0.01):
        """PP_MOVING → PP_IDLE (if reached)"""
        if self.state != "PP_MOVING":
            return False
        
        current = self.motor.drw.mech_pos
        target = self.motor.drw.loc_ref
        if abs(current - target) < threshold:
            self.state = "PP_IDLE"
            return True
        return False
```

### 4.2 Speed Control State Machine

```
┌─────────────┐
│   INIT      │
└──────┬──────┘
       │ set_parameter(0x7005, 2)
       ▼
┌─────────────┐
│SPD_INACTIVE │
└──┬──────────┘
   │ enable
   ▼
┌─────────────┐
│  SPD_IDLE   │  (spd_ref == 0)
└──┬──────────┘
   │ write spd_ref != 0
   ▼
┌─────────────┐
│SPD_RUNNING  │  (spd_ref != 0)
└──┬──────────┘
   │ write spd_ref == 0
   └─────────────┐
                 ▼
          (back to SPD_IDLE)
```

**Python Implementation:**
```python
class SpeedController:
    def __init__(self, motor):
        self.motor = motor
        self.state = "INIT"
    
    def setup_mode(self):
        self.motor.set_parameter(0x7005, 2, value_mode='j')
        self.motor.set_parameter(0x7018, 5.0, value_mode='p')  # Current limit
        self.motor.set_parameter(0x7022, 10.0, value_mode='p')  # Acceleration
        self.state = "SPD_INACTIVE"
    
    def activate(self):
        if self.state != "SPD_INACTIVE":
            raise StateError("Must be in SPD_INACTIVE state")
        self.motor.enable_motor()
        time.sleep(0.1)
        self.state = "SPD_IDLE"
    
    def set_speed(self, speed_rad_s):
        if self.state not in ["SPD_IDLE", "SPD_RUNNING"]:
            raise StateError("Must be activated")
        
        self.motor.set_parameter(0x700A, speed_rad_s, value_mode='p')
        
        if speed_rad_s == 0.0:
            self.state = "SPD_IDLE"
        else:
            self.state = "SPD_RUNNING"
```

### 4.3 MIT Protocol State Machine

```
┌─────────────┐
│ PRIVATE     │  (default protocol)
└──────┬──────┘
       │ Switch_Private2MIT()
       ▼
┌─────────────┐
│ MIT_INACTIVE│  (MIT protocol, disabled)
└──┬──────────┘
   │ Enable_MIT_Mode()
   ▼
┌─────────────┐
│  MIT_IDLE   │  (MIT enabled, zero command)
└──┬──────────┘
   │ Send_MIT_command(non-zero)
   ▼
┌─────────────┐
│ MIT_RUNNING │  (MIT active control)
└──┬──────────┘
   │ Stop_MIT_Mode() or Switch_MIT2Private()
   └────────────┐
                ▼
         (back to PRIVATE)
```

**Transitions:**
1. **PRIVATE → MIT_INACTIVE:** `Switch_Private2MIT()` + wait 50ms
2. **MIT_INACTIVE → MIT_IDLE:** `Enable_MIT_Mode()` (sends 0xFFFFFFFFFFFFFFFC)
3. **MIT_IDLE → MIT_RUNNING:** `Send_MIT_command()` with non-zero torque/position
4. **MIT_RUNNING → MIT_IDLE:** `Send_MIT_command()` with all zeros
5. **MIT_IDLE → PRIVATE:** `Stop_MIT_Mode()` (sends 0xFFFFFFFFFFFFFFFD)

**Python Implementation:**
```python
class MITController:
    def __init__(self, motor):
        self.motor = motor
        self.state = "PRIVATE"
    
    def switch_to_mit(self):
        """PRIVATE → MIT_INACTIVE"""
        if self.state != "PRIVATE":
            raise StateError("Must be in PRIVATE protocol")
        self.motor.switch_private2mit()
        time.sleep(0.05)
        self.state = "MIT_INACTIVE"
    
    def enable_mit(self):
        """MIT_INACTIVE → MIT_IDLE"""
        if self.state != "MIT_INACTIVE":
            raise StateError("Must be in MIT_INACTIVE state")
        self.motor.enable_mit_mode()
        time.sleep(0.01)
        self.state = "MIT_IDLE"
    
    def send_command(self, torque=0.0, position=0.0, velocity=0.0, kp=0.0, kd=0.0):
        """MIT_IDLE ⇄ MIT_RUNNING"""
        if self.state not in ["MIT_IDLE", "MIT_RUNNING"]:
            raise StateError("MIT mode not enabled")
        
        self.motor.send_mit_command(torque, position, velocity, kp, kd)
        
        if torque == 0.0 and kp == 0.0:
            self.state = "MIT_IDLE"
        else:
            self.state = "MIT_RUNNING"
    
    def switch_to_private(self):
        """MIT_* → PRIVATE"""
        if self.state == "MIT_RUNNING":
            self.send_command()  # Zero command first
            time.sleep(0.05)
        
        self.motor.stop_mit_mode()
        time.sleep(0.05)
        self.state = "PRIVATE"
```

---

## 5. Protocol Switching Procedure / プロトコル切り替え手順

### 5.1 Private → MIT Protocol

**Step-by-step:**
```python
# Precondition: Motor in DISABLED state

# 1. Ensure motor is disabled
if motor.state != MotorState.DISABLED:
    motor.disable_motor()
    time.sleep(0.1)

# 2. Switch protocol
motor.switch_private2mit()
time.sleep(0.05)  # CRITICAL: Wait for protocol change

# 3. Enable MIT mode
motor.enable_mit_mode()
time.sleep(0.05)

# 4. Now can send MIT commands
motor.send_mit_command(torque=0.5, position=0.0, velocity=0.0, kp=5.0, kd=0.5)
```

**CAN Messages:**
1. **Switch command:** ID=0x0C (Private), Data=[0x00] (1 byte)
2. **Enable command:** ID=0x7FF, Data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]

### 5.2 MIT → Private Protocol

**Step-by-step:**
```python
# Precondition: Motor in MIT mode

# 1. Stop MIT control (zero command)
motor.send_mit_command(0, 0, 0, 0, 0)
time.sleep(0.05)

# 2. Disable MIT mode
motor.stop_mit_mode()
time.sleep(0.05)  # CRITICAL: Wait for protocol change

# 3. Back to Private protocol, motor is disabled
# State: DISABLED

# 4. Can now use Private protocol commands
motor.enable_motor()
motor.set_parameter(0x7005, 1, value_mode='j')
```

**CAN Messages:**
1. **Stop command:** ID=0x7FF, Data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]
2. **Motor automatically switches back to Private protocol**

### 5.3 Protocol State Persistence

**Important Notes:**
- Protocol state is **volatile** (RAM only)
- After power cycle, motor resets to **Private protocol**
- No FLASH save for protocol mode
- Must re-execute switch sequence after each boot

---

## 6. Initialization Sequences / 初期化シーケンス

### 6.1 Cold Start (Power-On)

```python
def initialize_motor_cold_start(can_id: int, can_channel: str):
    """Complete cold start initialization"""
    
    # 1. Initialize CAN bus
    can_bus = can.interface.Bus(channel=can_channel, bustype='socketcan')
    
    # 2. Create motor instance
    motor = RobStrideMotor(can_id=can_id, can_bus=can_bus)
    # State: DISABLED
    
    # 3. Clear any stale errors
    motor.clear_errors()
    time.sleep(0.1)
    
    # 4. Read and verify motor ID
    motor.get_parameter(0x7000)  # Motor ID parameter
    time.sleep(0.05)
    
    # 5. Read current mode
    motor.get_parameter(0x7005)
    time.sleep(0.05)
    print(f"Current mode: {motor.drw.run_mode}")
    
    # 6. Set desired mode
    motor.set_parameter(0x7005, 5, value_mode='j')  # CSP mode
    time.sleep(0.05)
    
    # 7. Configure limits
    motor.set_parameter(0x7017, 10.0, value_mode='p')  # Speed limit
    motor.set_parameter(0x7018, 5.0, value_mode='p')  # Current limit
    time.sleep(0.1)
    
    # 8. Enable motor
    motor.enable_motor()
    time.sleep(0.1)
    # State: ENABLED
    
    # 9. Optionally set zero position
    # motor.set_zero_position()
    # time.sleep(0.1)
    
    print("Motor initialization complete")
    return motor
```

### 6.2 Warm Start (After Disable)

```python
def reinitialize_motor_warm_start(motor):
    """Re-enable previously initialized motor"""
    
    # 1. Verify state
    if motor.state == MotorState.FAULT:
        motor.clear_errors()
        time.sleep(0.1)
    
    # 2. Re-enable
    motor.enable_motor()
    time.sleep(0.1)
    # State: ENABLED
    
    # 3. Verify mode (should be retained)
    motor.get_parameter(0x7005)
    time.sleep(0.05)
    
    return motor
```

### 6.3 Zero Position Calibration Sequence

```python
def calibrate_zero_position(motor):
    """Set current position as mechanical zero"""
    
    # 1. Ensure motor is enabled
    if motor.state != MotorState.ENABLED:
        motor.enable_motor()
        time.sleep(0.1)
    
    # 2. Move to calibration position (manual or automated)
    print("Move motor to zero position, then press Enter...")
    input()
    
    # 3. Set zero
    motor.set_zero_position()
    time.sleep(0.1)
    
    # 4. Verify
    motor.get_parameter(0x7019)  # mech_pos
    time.sleep(0.05)
    
    if abs(motor.drw.mech_pos) < 0.01:
        print("Zero position set successfully")
    else:
        print(f"Warning: Position is {motor.drw.mech_pos} rad after zeroing")
```

---

## 7. State Monitoring / 状態監視

### 7.1 State Variables to Monitor

| Variable            | Parameter | Update Rate | Critical Threshold       |
| ------------------- | --------- | ----------- | ------------------------ |
| Motor enabled       | -         | Event       | -                        |
| Current mode        | 0x7005    | On change   | -                        |
| Position            | 0x7019    | 10-50 ms    | ±10 rad (limit)          |
| Velocity            | 0x701B    | 10-50 ms    | ±30 rad/s (limit)        |
| Current (filtered)  | 0x701A    | 10-50 ms    | ±23 A (over-current)     |
| Bus voltage         | 0x701C    | 100 ms      | < 12 V or > 50 V         |
| Temperature         | 0x701F    | 500 ms      | > 80°C (over-temp)       |
| Error flags         | 0x701E    | On error    | Any bit set              |

### 7.2 State Monitoring Loop

```python
class MotorStateMonitor:
    def __init__(self, motor, update_rate_hz=50):
        self.motor = motor
        self.update_interval = 1.0 / update_rate_hz
        self.last_update = time.time()
        self.state_history = []
    
    def update(self):
        """Call this in main loop"""
        now = time.time()
        if now - self.last_update < self.update_interval:
            return
        
        # Request critical parameters
        self.motor.get_parameter(0x7019)  # Position
        time.sleep(0.005)
        self.motor.get_parameter(0x701B)  # Velocity
        time.sleep(0.005)
        self.motor.get_parameter(0x701A)  # Current
        time.sleep(0.005)
        
        # Check limits
        self.check_limits()
        
        # Log state
        self.log_state()
        
        self.last_update = now
    
    def check_limits(self):
        """Check for limit violations"""
        if abs(self.motor.drw.mech_vel) > 30.0:
            print("WARNING: Speed limit exceeded")
        
        if abs(self.motor.drw.iqf) > 20.0:
            print("WARNING: High current detected")
        
        if self.motor.drw.vbus < 12.0:
            print("ERROR: Under-voltage")
            self.motor.disable_motor()
    
    def log_state(self):
        """Log current state for analysis"""
        state = {
            'time': time.time(),
            'pos': self.motor.drw.mech_pos,
            'vel': self.motor.drw.mech_vel,
            'cur': self.motor.drw.iqf,
            'vbus': self.motor.drw.vbus
        }
        self.state_history.append(state)
        
        # Keep only last 1000 samples
        if len(self.state_history) > 1000:
            self.state_history.pop(0)
```

---

## 8. Watchdog and Timeout Management / ウォッチドッグとタイムアウト管理

### 8.1 CAN Communication Watchdog

**Requirement:** モーターは CAN メッセージが一定時間（通常 500ms）受信されないと自動的に FAULT 状態に移行する。

**Implementation:**
```python
class CANWatchdog:
    def __init__(self, motor, timeout_s=0.5):
        self.motor = motor
        self.timeout = timeout_s
        self.last_tx = time.time()
    
    def feed(self):
        """Call after every CAN TX"""
        self.last_tx = time.time()
    
    def check(self):
        """Call periodically to ensure communication"""
        now = time.time()
        if now - self.last_tx > self.timeout * 0.8:  # 80% of timeout
            # Send keep-alive (read parameter)
            self.motor.get_parameter(0x7019)
            self.feed()

# Usage in control loop:
watchdog = CANWatchdog(motor, timeout_s=0.5)

while running:
    motor.set_parameter(0x7016, target, value_mode='p')
    watchdog.feed()
    
    time.sleep(0.01)
    
    watchdog.check()  # Ensures communication
```

### 8.2 Parameter Read Timeout

```python
def read_parameter_with_timeout(motor, index, timeout_s=0.5):
    """Read parameter with timeout handling"""
    
    # Send request
    motor.get_parameter(index)
    
    # Wait for response
    start = time.time()
    while time.time() - start < timeout_s:
        # Check if response received (implementation-specific)
        if motor.drw.last_updated_index == index:
            return motor.drw.get_value(index)
        time.sleep(0.001)
    
    raise TimeoutError(f"No response for parameter {hex(index)}")
```

---

## 9. State Persistence and Recovery / 状態永続化と復旧

### 9.1 State Save (Before Shutdown)

```python
def save_motor_state(motor, filename="motor_state.json"):
    """Save current motor configuration to file"""
    
    state = {
        'motor_id': motor.can_id,
        'run_mode': motor.drw.run_mode,
        'limit_spd': motor.drw.limit_spd,
        'limit_cur': motor.drw.limit_cur,
        'limit_torque': motor.drw.limit_torque,
        'cur_kp': motor.drw.cur_kp,
        'cur_ki': motor.drw.cur_ki,
        'cur_filt_gain': motor.drw.cur_filt_gain,
        'timestamp': time.time()
    }
    
    with open(filename, 'w') as f:
        json.dump(state, f, indent=2)
```

### 9.2 State Load (After Restart)

```python
def load_motor_state(motor, filename="motor_state.json"):
    """Restore motor configuration from file"""
    
    with open(filename, 'r') as f:
        state = json.load(f)
    
    # Restore parameters
    motor.set_parameter(0x7005, state['run_mode'], value_mode='j')
    time.sleep(0.05)
    motor.set_parameter(0x7017, state['limit_spd'], value_mode='p')
    motor.set_parameter(0x7018, state['limit_cur'], value_mode='p')
    motor.set_parameter(0x700B, state['limit_torque'], value_mode='p')
    motor.set_parameter(0x7010, state['cur_kp'], value_mode='p')
    motor.set_parameter(0x7011, state['cur_ki'], value_mode='p')
    motor.set_parameter(0x7014, state['cur_filt_gain'], value_mode='p')
    time.sleep(0.1)
    
    print(f"State restored from {filename}")
```

---

**End of State Machine Design Document**
