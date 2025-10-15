# RobStride Motor Control Library - Error Handling Specification
# RobStride モーター制御ライブラリ - エラー処理仕様

**Document Version:** 1.0  
**Date:** 2025-10-09

---

## 1. Overview / 概要

本ドキュメントは、RobStride モーター制御におけるエラー検出、分類、復旧手順、ログ記録を定義する。

---

## 2. Error Code Definitions / エラーコード定義

### 2.1 Error Flag Bitmap (8-bit)

モーターは 8 ビットのエラーフラグを返す（パラメータ 0x701E）。

| Bit | Hex  | Name                | Description                          | Severity |
| --- | ---- | ------------------- | ------------------------------------ | -------- |
| 0   | 0x01 | Over-Temperature    | モーター温度 > 80°C                  | Critical |
| 1   | 0x02 | Over-Current        | 電流 > 23A 継続                      | Critical |
| 2   | 0x04 | Over-Voltage        | バス電圧 > 50V                       | Critical |
| 3   | 0x08 | Under-Voltage       | バス電圧 < 12V                       | Critical |
| 4   | 0x10 | Encoder Error       | エンコーダ通信異常                   | Critical |
| 5   | 0x20 | Phase Error         | モーター相電流不均衡                 | Warning  |
| 6   | 0x40 | Reserved            | (未使用)                             | -        |
| 7   | 0x80 | CAN Timeout         | 500ms 以上コマンド未受信             | Critical |

**Python Enum:**
```python
class ErrorFlag(IntFlag):
    NONE = 0x00
    OVER_TEMPERATURE = 0x01
    OVER_CURRENT = 0x02
    OVER_VOLTAGE = 0x04
    UNDER_VOLTAGE = 0x08
    ENCODER_ERROR = 0x10
    PHASE_ERROR = 0x20
    CAN_TIMEOUT = 0x80
```

### 2.2 Error Reading

```python
# Read error flags
motor.get_parameter(0x701E)
time.sleep(0.05)

error_flags = motor.drw.error_flags

if error_flags & ErrorFlag.OVER_TEMPERATURE:
    print("Over-temperature error detected")

if error_flags & ErrorFlag.OVER_CURRENT:
    print("Over-current error detected")
```

---

## 3. Error Detection Mechanisms / エラー検出メカニズム

### 3.1 Temperature Monitoring

**Detection:**
- モーター内部温度センサーで測定
- 温度 > 80°C で Over-Temperature フラグ設定

**Response:**
1. モーター自動停止
2. エラーフラグ 0x01 設定
3. 状態 → FAULT

**Recovery:**
```python
def recover_from_over_temperature(motor):
    """Over-temperature recovery procedure"""
    
    # 1. Wait for cooling
    print("Motor over-temperature. Cooling down...")
    
    while True:
        motor.get_parameter(0x701F)  # Temperature parameter
        time.sleep(1.0)
        
        if motor.drw.temperature < 60.0:  # Safe threshold
            print(f"Temperature: {motor.drw.temperature}°C - Safe to restart")
            break
        else:
            print(f"Temperature: {motor.drw.temperature}°C - Waiting...")
    
    # 2. Clear error
    motor.clear_errors()
    time.sleep(0.1)
    
    # 3. Re-enable
    motor.enable_motor()
    time.sleep(0.1)
    
    print("Recovery complete")
```

### 3.2 Current Monitoring

**Detection:**
- 電流センサーで測定（フィルタ後）
- 電流 > 23A が 100ms 以上継続で Over-Current フラグ

**Response:**
1. モーター即座停止
2. エラーフラグ 0x02 設定
3. 状態 → FAULT

**Prevention:**
```python
# Set current limit before operation
motor.set_parameter(0x7018, 5.0, value_mode='p')  # Limit to 5A

# In control loop, monitor current
def check_current_limit(motor, threshold=20.0):
    motor.get_parameter(0x701A)
    time.sleep(0.01)
    
    if abs(motor.drw.iqf) > threshold:
        print(f"WARNING: High current {motor.drw.iqf} A")
        # Reduce load or command
        return False
    return True
```

**Recovery:**
```python
def recover_from_over_current(motor):
    """Over-current recovery procedure"""
    
    # 1. Identify cause
    print("Over-current detected. Possible causes:")
    print("- Mechanical jam")
    print("- Excessive load")
    print("- limit_cur too high")
    
    # 2. Clear error
    motor.clear_errors()
    time.sleep(0.1)
    
    # 3. Reduce current limit
    motor.set_parameter(0x7018, 3.0, value_mode='p')  # Lower limit
    time.sleep(0.05)
    
    # 4. Re-enable with caution
    motor.enable_motor()
    time.sleep(0.1)
    
    print("Recovery complete. Current limit reduced to 3A")
```

### 3.3 Voltage Monitoring

#### Over-Voltage (> 50V)

**Detection:**
- バス電圧モニター
- 電圧 > 50V で Over-Voltage フラグ

**Response:**
1. モーター停止
2. エラーフラグ 0x04 設定

**Recovery:**
```python
def recover_from_over_voltage(motor):
    """Over-voltage recovery"""
    
    # 1. Check power supply
    print("Over-voltage detected. Check power supply voltage.")
    print("Rated voltage: 24V, Max: 50V")
    
    # 2. Wait for voltage to stabilize
    while True:
        motor.get_parameter(0x701C)
        time.sleep(0.5)
        
        if motor.drw.vbus < 48.0:
            print(f"Voltage: {motor.drw.vbus}V - Safe")
            break
        else:
            print(f"Voltage: {motor.drw.vbus}V - Still high")
    
    # 3. Clear and re-enable
    motor.clear_errors()
    time.sleep(0.1)
    motor.enable_motor()
```

#### Under-Voltage (< 12V)

**Detection:**
- 電圧 < 12V で Under-Voltage フラグ

**Response:**
1. モーター停止
2. エラーフラグ 0x08 設定

**Recovery:**
```python
def recover_from_under_voltage(motor):
    """Under-voltage recovery"""
    
    # 1. Check power supply
    print("Under-voltage detected. Check:")
    print("- Power supply connection")
    print("- Supply capacity")
    print("- Cable resistance")
    
    # 2. Wait for voltage to recover
    while True:
        motor.get_parameter(0x701C)
        time.sleep(0.5)
        
        if motor.drw.vbus > 18.0:  # Safe margin
            print(f"Voltage: {motor.drw.vbus}V - Recovered")
            break
        else:
            print(f"Voltage: {motor.drw.vbus}V - Still low")
    
    # 3. Clear and re-enable
    motor.clear_errors()
    time.sleep(0.1)
    motor.enable_motor()
```

### 3.4 Encoder Error

**Detection:**
- エンコーダ通信エラー（SPI/I2C 異常）
- データ整合性エラー

**Response:**
1. モーター即座停止
2. エラーフラグ 0x10 設定

**Recovery:**
```python
def recover_from_encoder_error(motor):
    """Encoder error recovery"""
    
    print("Encoder error detected. This is a hardware issue.")
    print("Possible causes:")
    print("- Loose encoder cable")
    print("- Encoder damage")
    print("- EMI interference")
    
    # 1. Clear error (may not work if hardware fault)
    motor.clear_errors()
    time.sleep(0.1)
    
    # 2. Try re-enable
    try:
        motor.enable_motor()
        time.sleep(0.1)
        
        # 3. Test by reading position
        motor.get_parameter(0x7019)
        time.sleep(0.05)
        
        print(f"Position read: {motor.drw.mech_pos} rad")
        print("Encoder appears functional")
    except Exception as e:
        print(f"Encoder still faulty: {e}")
        print("Hardware inspection required")
```

### 3.5 CAN Timeout

**Detection:**
- モーターが 500ms 以上 CAN メッセージを受信しない

**Response:**
1. モーター自動停止
2. エラーフラグ 0x80 設定

**Prevention:**
```python
class CANHeartbeat:
    """Prevent CAN timeout by periodic messages"""
    
    def __init__(self, motor, interval_s=0.3):
        self.motor = motor
        self.interval = interval_s
        self.last_tx = 0
        self.running = False
        self.thread = None
    
    def start(self):
        """Start heartbeat thread"""
        self.running = True
        self.thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self.thread.start()
    
    def stop(self):
        """Stop heartbeat"""
        self.running = False
        if self.thread:
            self.thread.join()
    
    def _heartbeat_loop(self):
        """Background heartbeat"""
        while self.running:
            now = time.time()
            if now - self.last_tx > self.interval:
                # Send keep-alive (read parameter)
                try:
                    self.motor.get_parameter(0x7019)
                    self.last_tx = now
                except Exception as e:
                    print(f"Heartbeat failed: {e}")
            
            time.sleep(0.01)
    
    def notify_tx(self):
        """Call after any TX to reset timer"""
        self.last_tx = time.time()

# Usage:
heartbeat = CANHeartbeat(motor, interval_s=0.3)
heartbeat.start()

# In control loop, heartbeat automatically sends messages
# If control loop sends frequently, heartbeat is inactive
```

**Recovery:**
```python
def recover_from_can_timeout(motor):
    """CAN timeout recovery"""
    
    print("CAN timeout detected.")
    
    # 1. Clear error
    motor.clear_errors()
    time.sleep(0.1)
    
    # 2. Re-establish communication
    motor.get_parameter(0x7005)
    time.sleep(0.05)
    
    # 3. Re-enable
    motor.enable_motor()
    time.sleep(0.1)
    
    print("Communication restored")
```

---

## 4. Error Recovery Strategies / エラー復旧戦略

### 4.1 Automatic Recovery (Transient Errors)

**Applicable Errors:**
- CAN Timeout
- Under-Voltage (if power recovers)
- Phase Error (minor imbalance)

**Strategy:**
```python
class AutoRecovery:
    def __init__(self, motor, max_retries=3):
        self.motor = motor
        self.max_retries = max_retries
        self.retry_count = 0
    
    def attempt_recovery(self, error_code):
        """Automatic recovery attempt"""
        
        if self.retry_count >= self.max_retries:
            print("Max retries reached. Manual intervention required.")
            return False
        
        print(f"Auto-recovery attempt {self.retry_count + 1}/{self.max_retries}")
        
        # Clear error
        self.motor.clear_errors()
        time.sleep(0.5)
        
        # Re-enable
        try:
            self.motor.enable_motor()
            time.sleep(0.1)
            
            # Verify no errors
            self.motor.get_parameter(0x701E)
            time.sleep(0.05)
            
            if self.motor.drw.error_flags == 0:
                print("Auto-recovery successful")
                self.retry_count = 0
                return True
            else:
                print(f"Errors persist: {hex(self.motor.drw.error_flags)}")
                self.retry_count += 1
                return False
        
        except Exception as e:
            print(f"Recovery failed: {e}")
            self.retry_count += 1
            return False
```

### 4.2 Manual Recovery (Critical Errors)

**Applicable Errors:**
- Over-Temperature (requires cooling)
- Over-Current (requires load reduction)
- Encoder Error (hardware issue)

**Strategy:**
```python
def manual_recovery_procedure(motor, error_code):
    """Manual recovery with user intervention"""
    
    print(f"\n=== Manual Recovery Required ===")
    print(f"Error code: {hex(error_code)}")
    
    if error_code & ErrorFlag.OVER_TEMPERATURE:
        print("\n1. Wait for motor to cool (< 60°C)")
        print("2. Check ventilation and ambient temperature")
        print("3. Reduce continuous load if necessary")
        input("Press Enter when ready to retry...")
        recover_from_over_temperature(motor)
    
    elif error_code & ErrorFlag.OVER_CURRENT:
        print("\n1. Remove mechanical jam or excessive load")
        print("2. Check if limit_cur is too high")
        print("3. Verify no short circuits")
        input("Press Enter when ready to retry...")
        recover_from_over_current(motor)
    
    elif error_code & ErrorFlag.ENCODER_ERROR:
        print("\n1. Check encoder cable connection")
        print("2. Inspect for physical damage")
        print("3. Test with different motor if available")
        input("Press Enter when ready to retry...")
        recover_from_encoder_error(motor)
    
    else:
        print("\n1. Check hardware connections")
        print("2. Verify power supply")
        print("3. Restart system if necessary")
        input("Press Enter when ready to retry...")
        motor.clear_errors()
        time.sleep(0.1)
        motor.enable_motor()
```

### 4.3 Progressive Degradation

**Strategy:** システムを完全停止せず、制限モードで運転継続

```python
class ProgressiveDegradation:
    def __init__(self, motor):
        self.motor = motor
        self.degradation_level = 0
        self.limits = [
            {'name': 'Normal', 'spd': 20.0, 'cur': 10.0},
            {'name': 'Degraded-1', 'spd': 10.0, 'cur': 5.0},
            {'name': 'Degraded-2', 'spd': 5.0, 'cur': 3.0},
            {'name': 'Minimal', 'spd': 2.0, 'cur': 1.0}
        ]
    
    def handle_error(self, error_code):
        """Degrade limits instead of full stop"""
        
        if error_code & (ErrorFlag.OVER_CURRENT | ErrorFlag.OVER_TEMPERATURE):
            self.degradation_level += 1
            
            if self.degradation_level >= len(self.limits):
                print("Cannot degrade further. Full stop required.")
                self.motor.disable_motor()
                return False
            
            # Apply reduced limits
            limits = self.limits[self.degradation_level]
            print(f"Degrading to: {limits['name']}")
            
            self.motor.set_parameter(0x7017, limits['spd'], value_mode='p')
            self.motor.set_parameter(0x7018, limits['cur'], value_mode='p')
            time.sleep(0.1)
            
            # Clear error and continue
            self.motor.clear_errors()
            time.sleep(0.1)
            self.motor.enable_motor()
            
            return True
        
        return False
    
    def restore_normal(self):
        """Restore normal limits when safe"""
        if self.degradation_level > 0:
            self.degradation_level = 0
            limits = self.limits[0]
            print(f"Restoring to: {limits['name']}")
            
            self.motor.set_parameter(0x7017, limits['spd'], value_mode='p')
            self.motor.set_parameter(0x7018, limits['cur'], value_mode='p')
```

---

## 5. Error Logging / エラーログ記録

### 5.1 Log Entry Structure

```python
from dataclasses import dataclass
from datetime import datetime

@dataclass
class ErrorLogEntry:
    timestamp: datetime
    error_code: int
    error_names: List[str]
    motor_state: dict
    recovery_action: str
    recovery_success: bool

class ErrorLogger:
    def __init__(self, log_file="motor_errors.log"):
        self.log_file = log_file
        self.entries = []
    
    def log_error(self, motor, error_code, recovery_action, success):
        """Log error event"""
        
        # Decode error names
        error_names = []
        for flag in ErrorFlag:
            if error_code & flag:
                error_names.append(flag.name)
        
        # Capture motor state
        state = {
            'position': motor.drw.mech_pos,
            'velocity': motor.drw.mech_vel,
            'current': motor.drw.iqf,
            'voltage': motor.drw.vbus,
            'temperature': motor.drw.temperature
        }
        
        # Create entry
        entry = ErrorLogEntry(
            timestamp=datetime.now(),
            error_code=error_code,
            error_names=error_names,
            motor_state=state,
            recovery_action=recovery_action,
            recovery_success=success
        )
        
        self.entries.append(entry)
        self._write_to_file(entry)
    
    def _write_to_file(self, entry):
        """Write log entry to file"""
        with open(self.log_file, 'a') as f:
            f.write(f"{entry.timestamp.isoformat()} | ")
            f.write(f"Error: {hex(entry.error_code)} ({', '.join(entry.error_names)}) | ")
            f.write(f"State: {entry.motor_state} | ")
            f.write(f"Action: {entry.recovery_action} | ")
            f.write(f"Success: {entry.recovery_success}\n")
    
    def get_error_statistics(self):
        """Analyze error frequency"""
        stats = {}
        for entry in self.entries:
            for name in entry.error_names:
                stats[name] = stats.get(name, 0) + 1
        return stats
    
    def get_recent_errors(self, n=10):
        """Get last N errors"""
        return self.entries[-n:]
```

### 5.2 Usage Example

```python
logger = ErrorLogger("motor_errors.log")

def safe_operation(motor):
    """Operation with error logging"""
    
    try:
        motor.set_parameter(0x700A, 15.0, value_mode='p')
    
    except Exception as e:
        # Read error
        motor.get_parameter(0x701E)
        time.sleep(0.05)
        error_code = motor.drw.error_flags
        
        # Attempt recovery
        recovery_action = "Auto-clear and re-enable"
        motor.clear_errors()
        time.sleep(0.1)
        
        try:
            motor.enable_motor()
            success = True
        except:
            success = False
        
        # Log event
        logger.log_error(motor, error_code, recovery_action, success)
        
        if not success:
            raise

# Later, analyze logs
print("Error statistics:")
for error, count in logger.get_error_statistics().items():
    print(f"  {error}: {count} occurrences")
```

---

## 6. Error Prevention / エラー予防

### 6.1 Pre-Operation Checks

```python
def pre_operation_check(motor):
    """Verify motor is ready for operation"""
    
    checks = {
        'CAN communication': False,
        'Motor enabled': False,
        'No errors': False,
        'Voltage OK': False,
        'Temperature OK': False
    }
    
    # 1. Check communication
    try:
        motor.get_parameter(0x7005)
        time.sleep(0.05)
        checks['CAN communication'] = True
    except:
        pass
    
    # 2. Check enabled state
    if motor.state == MotorState.ENABLED:
        checks['Motor enabled'] = True
    
    # 3. Check errors
    motor.get_parameter(0x701E)
    time.sleep(0.05)
    if motor.drw.error_flags == 0:
        checks['No errors'] = True
    
    # 4. Check voltage
    motor.get_parameter(0x701C)
    time.sleep(0.05)
    if 18.0 <= motor.drw.vbus <= 48.0:
        checks['Voltage OK'] = True
    
    # 5. Check temperature
    motor.get_parameter(0x701F)
    time.sleep(0.05)
    if motor.drw.temperature < 70.0:
        checks['Temperature OK'] = True
    
    # Report results
    print("\n=== Pre-Operation Check ===")
    all_ok = True
    for check, status in checks.items():
        symbol = "✓" if status else "✗"
        print(f"{symbol} {check}")
        if not status:
            all_ok = False
    
    return all_ok
```

### 6.2 Runtime Monitoring

```python
class RuntimeMonitor:
    """Continuous monitoring during operation"""
    
    def __init__(self, motor):
        self.motor = motor
        self.warning_thresholds = {
            'current': 18.0,  # 80% of max
            'temperature': 70.0,  # Below critical
            'voltage_low': 18.0,
            'voltage_high': 45.0
        }
    
    def check(self):
        """Check all parameters"""
        warnings = []
        
        # Current
        if abs(self.motor.drw.iqf) > self.warning_thresholds['current']:
            warnings.append(f"High current: {self.motor.drw.iqf} A")
        
        # Temperature
        if self.motor.drw.temperature > self.warning_thresholds['temperature']:
            warnings.append(f"High temperature: {self.motor.drw.temperature}°C")
        
        # Voltage
        if self.motor.drw.vbus < self.warning_thresholds['voltage_low']:
            warnings.append(f"Low voltage: {self.motor.drw.vbus} V")
        elif self.motor.drw.vbus > self.warning_thresholds['voltage_high']:
            warnings.append(f"High voltage: {self.motor.drw.vbus} V")
        
        return warnings

# Usage:
monitor = RuntimeMonitor(motor)

while running:
    # Update state
    motor.get_parameter(0x701A)  # Current
    motor.get_parameter(0x701F)  # Temperature
    motor.get_parameter(0x701C)  # Voltage
    time.sleep(0.01)
    
    # Check warnings
    warnings = monitor.check()
    if warnings:
        for w in warnings:
            print(f"WARNING: {w}")
    
    # Continue control...
```

---

## 7. Error Handling Best Practices / エラー処理ベストプラクティス

### 7.1 Fail-Safe Principle

```python
class FailSafeController:
    """Controller with automatic safe shutdown"""
    
    def __init__(self, motor):
        self.motor = motor
        self.safe_state_entered = False
    
    def enter_safe_state(self, reason):
        """Enter safe state on any error"""
        if self.safe_state_entered:
            return
        
        print(f"Entering safe state: {reason}")
        
        try:
            # 1. Stop immediately
            self.motor.disable_motor()
            
            # 2. Log event
            logger.log_error(self.motor, 0xFF, reason, True)
            
            # 3. Set flag
            self.safe_state_entered = True
        
        except Exception as e:
            print(f"Failed to enter safe state: {e}")
    
    def exit_safe_state(self):
        """Exit safe state after verification"""
        if not self.safe_state_entered:
            return
        
        # Check if safe to exit
        if not pre_operation_check(self.motor):
            print("Pre-operation check failed. Cannot exit safe state.")
            return False
        
        self.safe_state_entered = False
        print("Exited safe state")
        return True
```

### 7.2 Error Escalation

```python
class ErrorEscalation:
    """Escalate errors based on frequency"""
    
    def __init__(self, motor):
        self.motor = motor
        self.error_history = []
        self.escalation_threshold = 3  # 3 errors in 60s
        self.time_window = 60.0
    
    def record_error(self, error_code):
        """Record error and check for escalation"""
        now = time.time()
        
        # Add to history
        self.error_history.append({'time': now, 'code': error_code})
        
        # Remove old entries
        self.error_history = [
            e for e in self.error_history
            if now - e['time'] < self.time_window
        ]
        
        # Check escalation
        if len(self.error_history) >= self.escalation_threshold:
            self.escalate()
    
    def escalate(self):
        """Escalate to full shutdown"""
        print("ERROR ESCALATION: Too many errors in short time")
        print("Shutting down motor for safety")
        
        self.motor.disable_motor()
        
        # Notify user/system
        # send_alert("Motor shutdown due to repeated errors")
        
        # Clear history
        self.error_history.clear()
```

---

**End of Error Handling Specification**
