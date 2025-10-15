# RobStride Motor Control Library - Test Specification
# RobStride モーター制御ライブラリ - テスト仕様

**Document Version:** 1.0  
**Date:** 2025-10-09

---

## 1. Overview / 概要

本ドキュメントは、RobStride モーター制御ライブラリの包括的なテスト戦略、テストケース、検証手順を定義する。

---

## 2. Test Strategy / テスト戦略

### 2.1 Test Levels

| Level | Scope | Tools | Coverage Target |
| ----- | ----- | ----- | --------------- |
| Unit Tests | Individual functions/methods | pytest | > 80% |
| Integration Tests | Module interactions | pytest + mock CAN | > 70% |
| System Tests | Full CAN communication | Real hardware | Critical paths |
| Hardware-in-Loop (HIL) | Motor control loops | Real motor + testbench | Key scenarios |
| Acceptance Tests | End-user scenarios | Real application | User stories |

### 2.2 Test Environment

#### Software Environment
- Python 3.8+
- pytest 7.0+
- pytest-cov (coverage)
- pytest-mock (mocking)
- python-can with virtual CAN (vcan)

#### Hardware Environment
- RobStride RS01 motor
- CAN interface (USB-CAN adapter or embedded)
- Power supply (24V, 10A+)
- Safety emergency stop button
- Load simulator (optional, for torque testing)

---

## 3. Unit Tests / 単体テスト

### 3.1 Motor Initialization Tests

**Test File:** `tests/unit/test_motor_init.py`

#### TC-U-001: Valid Motor ID
```python
def test_valid_motor_id():
    """Test motor creation with valid ID"""
    for motor_id in [1, 16, 32]:
        motor = RobStrideMotor(can_id=motor_id, can_bus=mock_bus)
        assert motor.can_id == motor_id
```

#### TC-U-002: Invalid Motor ID
```python
def test_invalid_motor_id():
    """Test motor creation with invalid ID"""
    with pytest.raises(ValueError):
        RobStrideMotor(can_id=0, can_bus=mock_bus)
    
    with pytest.raises(ValueError):
        RobStrideMotor(can_id=33, can_bus=mock_bus)
    
    with pytest.raises(ValueError):
        RobStrideMotor(can_id=-1, can_bus=mock_bus)
```

#### TC-U-003: Default Configuration
```python
def test_default_configuration():
    """Test default motor configuration"""
    motor = RobStrideMotor(can_id=1, can_bus=mock_bus, auto_enable=False)
    
    assert motor.protocol_mode == ProtocolMode.PRIVATE
    assert motor.enabled == False
    assert motor.status is not None
```

---

### 3.2 CAN Message Encoding Tests

**Test File:** `tests/unit/test_can_encoding.py`

#### TC-U-010: Extended ID Construction
```python
def test_extended_id_construction():
    """Test Extended CAN ID building"""
    handler = PrivateProtocolHandler(motor_id=5, can_bus=mock_bus)
    
    # Enable command: type=0x01, motor_id=5
    can_id = handler._build_extended_id(0x01)
    
    assert can_id == 0x01000500
    assert (can_id >> 24) & 0x1F == 0x01  # Command type
    assert (can_id >> 8) & 0xFF == 5      # Motor ID
```

#### TC-U-011: Parameter Write Encoding
```python
def test_parameter_write_encoding():
    """Test parameter write message encoding"""
    handler = PrivateProtocolHandler(motor_id=1, can_bus=mock_bus)
    
    # Write float value
    result = handler.write_parameter(0x7016, 1.57, value_mode='p')
    
    # Verify CAN message
    assert mock_bus.send.called
    msg = mock_bus.send.call_args[0][0]
    
    assert msg.is_extended_id == True
    assert (msg.arbitration_id >> 24) & 0x1F == 0x12  # Write command
    assert msg.data[0] == 0x16  # Index low byte
    assert msg.data[1] == 0x70  # Index high byte
    
    # Verify float encoding
    value_bytes = struct.pack('<f', 1.57)
    assert msg.data[2:6] == list(value_bytes)
```

#### TC-U-012: MIT Command Encoding
```python
def test_mit_command_encoding():
    """Test MIT protocol command encoding"""
    handler = MITProtocolHandler(motor_id=1, can_bus=mock_bus)
    
    result = handler.send_control_command(
        torque=1.0,
        position=0.5,
        velocity=2.0,
        kp=10.0,
        kd=0.5
    )
    
    assert mock_bus.send.called
    msg = mock_bus.send.call_args[0][0]
    
    assert msg.is_extended_id == False
    assert msg.arbitration_id == 0x7FF
    assert len(msg.data) == 8
```

---

### 3.3 Data Conversion Tests

**Test File:** `tests/unit/test_conversion.py`

#### TC-U-020: Float to Uint Conversion
```python
@pytest.mark.parametrize("value,min_val,max_val,bits,expected", [
    (0.0, -10.0, 10.0, 16, 32768),
    (10.0, -10.0, 10.0, 16, 65535),
    (-10.0, -10.0, 10.0, 16, 0),
    (5.0, 0.0, 10.0, 8, 128)
])
def test_float_to_uint(value, min_val, max_val, bits, expected):
    """Test float to uint conversion"""
    result = float_to_uint(value, min_val, max_val, bits)
    assert result == expected
```

#### TC-U-021: Uint to Float Conversion
```python
@pytest.mark.parametrize("value,min_val,max_val,bits,expected", [
    (32768, -10.0, 10.0, 16, 0.0),
    (65535, -10.0, 10.0, 16, 10.0),
    (0, -10.0, 10.0, 16, -10.0)
])
def test_uint_to_float(value, min_val, max_val, bits, expected):
    """Test uint to float conversion"""
    result = uint_to_float(value, min_val, max_val, bits)
    assert abs(result - expected) < 0.01
```

---

### 3.4 Validation Tests

**Test File:** `tests/unit/test_validation.py`

#### TC-U-030: Parameter Value Validation
```python
def test_parameter_value_validation():
    """Test parameter value range validation"""
    
    # Valid values
    validate_parameter_value(ParameterIndex.IQ_REF, 10.0)
    validate_parameter_value(ParameterIndex.SPD_REF, 20.0)
    
    # Invalid values
    with pytest.raises(ValueError):
        validate_parameter_value(ParameterIndex.IQ_REF, 25.0)  # > 23
    
    with pytest.raises(ValueError):
        validate_parameter_value(ParameterIndex.SPD_REF, 50.0)  # > 30
```

---

## 4. Integration Tests / 統合テスト

### 4.1 Protocol Handler Integration

**Test File:** `tests/integration/test_protocol_integration.py`

#### TC-I-001: Enable-Disable Sequence
```python
def test_enable_disable_sequence():
    """Test motor enable and disable sequence"""
    motor = RobStrideMotor(can_id=1, can_bus=vcan_bus)
    
    # Enable
    assert motor.enable() == True
    time.sleep(0.1)
    assert motor.enabled == True
    
    # Disable
    assert motor.disable() == True
    time.sleep(0.1)
    assert motor.enabled == False
```

#### TC-I-002: Protocol Switching
```python
def test_protocol_switching():
    """Test switching between Private and MIT protocols"""
    motor = RobStrideMotor(can_id=1, can_bus=vcan_bus)
    
    # Start in Private
    assert motor.protocol_mode == ProtocolMode.PRIVATE
    
    # Switch to MIT
    assert motor.switch_to_mit() == True
    time.sleep(0.1)
    assert motor.protocol_mode == ProtocolMode.MIT
    
    # Switch back to Private
    assert motor.switch_to_private() == True
    time.sleep(0.1)
    assert motor.protocol_mode == ProtocolMode.PRIVATE
```

#### TC-I-003: Parameter Read-Write
```python
def test_parameter_read_write():
    """Test parameter read and write operations"""
    motor = RobStrideMotor(can_id=1, can_bus=vcan_bus)
    motor.enable()
    
    # Write parameter
    assert motor.set_parameter(0x7018, 5.0, value_mode='p') == True
    time.sleep(0.05)
    
    # Read back
    assert motor.get_parameter(0x7018) == True
    time.sleep(0.1)
    
    # Verify value (may need mock response for unit test)
    # assert motor.status.limit_cur == 5.0
```

---

### 4.2 Control Mode Integration

**Test File:** `tests/integration/test_control_modes.py`

#### TC-I-010: Mode Change Sequence
```python
def test_control_mode_change():
    """Test control mode switching"""
    motor = RobStrideMotor(can_id=1, can_bus=vcan_bus)
    motor.enable()
    
    # Change to position mode
    assert motor.set_control_mode(ControlMode.POSITION_PP) == True
    time.sleep(0.2)
    
    # Change to speed mode
    assert motor.set_control_mode(ControlMode.SPEED) == True
    time.sleep(0.2)
    
    # Change to current mode
    assert motor.set_control_mode(ControlMode.CURRENT) == True
    time.sleep(0.2)
```

---

## 5. System Tests / システムテスト

### 5.1 Hardware Communication Tests

**Test File:** `tests/system/test_hardware_communication.py`

**Prerequisites:** Real motor connected to CAN bus

#### TC-S-001: CAN Bus Communication
```python
@pytest.mark.hardware
def test_can_bus_communication():
    """Test basic CAN communication with real motor"""
    can_bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000)
    motor = RobStrideMotor(can_id=1, can_bus=can_bus)
    
    try:
        # Enable motor
        assert motor.enable() == True
        time.sleep(0.2)
        
        # Read parameter
        assert motor.get_parameter(0x7005) == True
        time.sleep(0.1)
        
        # Verify response received
        assert motor.status.last_update > 0
        
    finally:
        motor.disable()
        motor.close()
        can_bus.shutdown()
```

#### TC-S-002: Status Feedback
```python
@pytest.mark.hardware
def test_status_feedback():
    """Test receiving motor status feedback"""
    can_bus = can.interface.Bus(channel='can0', bustype='socketcan')
    motor = RobStrideMotor(can_id=1, can_bus=can_bus)
    
    try:
        motor.enable()
        time.sleep(0.2)
        
        # Request position
        motor.get_parameter(0x7019)
        time.sleep(0.1)
        
        # Verify status updated
        assert motor.status.mech_pos is not None
        print(f"Position: {motor.status.mech_pos} rad")
        
    finally:
        motor.disable()
        motor.close()
        can_bus.shutdown()
```

---

### 5.2 Control Performance Tests

**Test File:** `tests/system/test_control_performance.py`

#### TC-S-010: Position Control Accuracy
```python
@pytest.mark.hardware
def test_position_control_accuracy():
    """Test position control accuracy"""
    can_bus = can.interface.Bus(channel='can0', bustype='socketcan')
    motor = RobStrideMotor(can_id=1, can_bus=can_bus)
    
    try:
        # Setup
        motor.enable()
        motor.set_control_mode(ControlMode.POSITION_CSP)
        motor.set_parameter(0x7017, 5.0, value_mode='p')  # Speed limit
        motor.set_parameter(0x7018, 3.0, value_mode='p')  # Current limit
        time.sleep(0.2)
        
        # Set zero
        motor.set_zero_position()
        time.sleep(0.2)
        
        # Move to target
        target = 1.57  # 90 degrees
        motor.set_position(target)
        
        # Wait for settling
        time.sleep(3.0)
        
        # Read final position
        motor.get_parameter(0x7019)
        time.sleep(0.1)
        
        final_pos = motor.status.mech_pos
        error = abs(final_pos - target)
        
        print(f"Target: {target} rad, Final: {final_pos} rad, Error: {error} rad")
        
        # Acceptance: Error < 0.05 rad (~2.8 degrees)
        assert error < 0.05, f"Position error {error} too large"
        
    finally:
        motor.set_position(0.0)
        time.sleep(2.0)
        motor.disable()
        motor.close()
        can_bus.shutdown()
```

#### TC-S-011: Speed Control Stability
```python
@pytest.mark.hardware
def test_speed_control_stability():
    """Test speed control stability"""
    can_bus = can.interface.Bus(channel='can0', bustype='socketcan')
    motor = RobStrideMotor(can_id=1, can_bus=can_bus)
    
    try:
        # Setup
        motor.enable()
        motor.set_control_mode(ControlMode.SPEED)
        motor.set_parameter(0x7018, 5.0, value_mode='p')
        time.sleep(0.2)
        
        # Set speed
        target_speed = 10.0  # rad/s
        motor.set_speed(target_speed)
        
        # Monitor for 5 seconds
        speeds = []
        for _ in range(50):
            motor.get_parameter(0x701B)
            time.sleep(0.1)
            speeds.append(motor.status.mech_vel)
        
        # Calculate statistics
        avg_speed = sum(speeds) / len(speeds)
        speed_error = abs(avg_speed - target_speed)
        speed_std = (sum((s - avg_speed)**2 for s in speeds) / len(speeds)) ** 0.5
        
        print(f"Target: {target_speed} rad/s")
        print(f"Average: {avg_speed:.2f} rad/s")
        print(f"Error: {speed_error:.2f} rad/s")
        print(f"Std Dev: {speed_std:.2f} rad/s")
        
        # Acceptance: Error < 1 rad/s, StdDev < 0.5 rad/s
        assert speed_error < 1.0, "Speed error too large"
        assert speed_std < 0.5, "Speed variation too large"
        
    finally:
        motor.set_speed(0.0)
        time.sleep(1.0)
        motor.disable()
        motor.close()
        can_bus.shutdown()
```

#### TC-S-012: Current Control Response
```python
@pytest.mark.hardware
def test_current_control_response():
    """Test current control response"""
    can_bus = can.interface.Bus(channel='can0', bustype='socketcan')
    motor = RobStrideMotor(can_id=1, can_bus=can_bus)
    
    try:
        # Setup
        motor.enable()
        motor.set_control_mode(ControlMode.CURRENT)
        time.sleep(0.2)
        
        # Set current
        target_current = 2.0  # A
        motor.set_current(target_current)
        
        # Wait for response
        time.sleep(0.5)
        
        # Read current
        motor.get_parameter(0x701A)
        time.sleep(0.1)
        
        actual_current = motor.status.iqf
        error = abs(actual_current - target_current)
        
        print(f"Target: {target_current} A, Actual: {actual_current} A, Error: {error} A")
        
        # Acceptance: Error < 0.5 A
        assert error < 0.5, "Current error too large"
        
    finally:
        motor.set_current(0.0)
        time.sleep(0.5)
        motor.disable()
        motor.close()
        can_bus.shutdown()
```

---

### 5.3 Error Handling Tests

**Test File:** `tests/system/test_error_handling.py`

#### TC-S-020: Over-Current Detection
```python
@pytest.mark.hardware
@pytest.mark.safety
def test_over_current_detection():
    """Test over-current error detection and recovery"""
    can_bus = can.interface.Bus(channel='can0', bustype='socketcan')
    motor = RobStrideMotor(can_id=1, can_bus=can_bus)
    
    try:
        # Setup with low current limit
        motor.enable()
        motor.set_control_mode(ControlMode.CURRENT)
        motor.set_parameter(0x7018, 1.0, value_mode='p')  # Low limit
        time.sleep(0.2)
        
        # Attempt to exceed limit
        motor.set_current(5.0)  # Exceeds limit
        time.sleep(0.5)
        
        # Check for error
        motor.get_parameter(0x701E)
        time.sleep(0.1)
        
        if motor.status.error_flags & ErrorFlag.OVER_CURRENT:
            print("Over-current error correctly detected")
            
            # Test recovery
            motor.clear_errors()
            time.sleep(0.2)
            
            motor.get_parameter(0x701E)
            time.sleep(0.1)
            
            assert motor.status.error_flags == 0, "Errors not cleared"
        
    finally:
        motor.set_current(0.0)
        motor.disable()
        motor.close()
        can_bus.shutdown()
```

#### TC-S-021: CAN Timeout Recovery
```python
@pytest.mark.hardware
def test_can_timeout_recovery():
    """Test CAN timeout and automatic recovery"""
    can_bus = can.interface.Bus(channel='can0', bustype='socketcan')
    motor = RobStrideMotor(can_id=1, can_bus=can_bus)
    
    try:
        motor.enable()
        time.sleep(0.2)
        
        # Stop sending messages for 1 second (trigger timeout)
        print("Stopping communication for 1 second...")
        time.sleep(1.0)
        
        # Check for timeout error
        motor.get_parameter(0x701E)
        time.sleep(0.1)
        
        if motor.status.error_flags & ErrorFlag.CAN_TIMEOUT:
            print("Timeout error detected")
            
            # Recover by resuming communication
            motor.clear_errors()
            time.sleep(0.1)
            
            motor.enable()
            time.sleep(0.2)
            
            # Verify recovery
            motor.get_parameter(0x701E)
            time.sleep(0.1)
            
            assert motor.status.error_flags == 0, "Recovery failed"
        
    finally:
        motor.disable()
        motor.close()
        can_bus.shutdown()
```

---

## 6. Hardware-in-Loop Tests / HIL テスト

### 6.1 Trajectory Tracking

**Test File:** `tests/hil/test_trajectory_tracking.py`

#### TC-H-001: Sinusoidal Trajectory
```python
@pytest.mark.hardware
@pytest.mark.hil
def test_sinusoidal_trajectory():
    """Test sinusoidal trajectory tracking"""
    can_bus = can.interface.Bus(channel='can0', bustype='socketcan')
    motor = RobStrideMotor(can_id=1, can_bus=can_bus)
    
    try:
        # Setup
        motor.enable()
        motor.set_control_mode(ControlMode.POSITION_CSP)
        motor.set_parameter(0x7017, 10.0, value_mode='p')
        motor.set_parameter(0x7018, 5.0, value_mode='p')
        time.sleep(0.2)
        
        # Trajectory parameters
        amplitude = 1.0  # rad
        frequency = 0.5  # Hz
        duration = 10.0  # seconds
        dt = 0.01  # 100 Hz
        
        # Record data
        times = []
        targets = []
        actuals = []
        
        start_time = time.time()
        while time.time() - start_time < duration:
            t = time.time() - start_time
            
            # Calculate target
            target = amplitude * math.sin(2 * math.pi * frequency * t)
            
            # Send command
            motor.set_position(target)
            
            # Read actual
            motor.get_parameter(0x7019)
            time.sleep(dt / 2)
            
            # Record
            times.append(t)
            targets.append(target)
            actuals.append(motor.status.mech_pos)
            
            time.sleep(dt / 2)
        
        # Analyze tracking error
        errors = [abs(a - t) for a, t in zip(actuals, targets)]
        max_error = max(errors)
        avg_error = sum(errors) / len(errors)
        rms_error = (sum(e**2 for e in errors) / len(errors)) ** 0.5
        
        print(f"Max tracking error: {max_error:.4f} rad")
        print(f"Avg tracking error: {avg_error:.4f} rad")
        print(f"RMS tracking error: {rms_error:.4f} rad")
        
        # Acceptance criteria
        assert max_error < 0.1, "Max tracking error too large"
        assert rms_error < 0.05, "RMS tracking error too large"
        
        # Optional: Save data for plotting
        import json
        with open('trajectory_data.json', 'w') as f:
            json.dump({
                'times': times,
                'targets': targets,
                'actuals': actuals
            }, f)
        
    finally:
        motor.set_position(0.0)
        time.sleep(2.0)
        motor.disable()
        motor.close()
        can_bus.shutdown()
```

---

### 6.2 Multi-Motor Synchronization

**Test File:** `tests/hil/test_multi_motor_sync.py`

#### TC-H-010: Synchronized Motion
```python
@pytest.mark.hardware
@pytest.mark.hil
def test_multi_motor_synchronization():
    """Test synchronized motion of multiple motors"""
    can_bus = can.interface.Bus(channel='can0', bustype='socketcan')
    motors = [
        RobStrideMotor(can_id=1, can_bus=can_bus),
        RobStrideMotor(can_id=2, can_bus=can_bus),
        RobStrideMotor(can_id=3, can_bus=can_bus)
    ]
    
    try:
        # Initialize all motors
        for motor in motors:
            motor.enable()
            motor.set_control_mode(ControlMode.POSITION_CSP)
            motor.set_parameter(0x7017, 5.0, value_mode='p')
            motor.set_parameter(0x7018, 3.0, value_mode='p')
        
        time.sleep(0.5)
        
        # Synchronized movement
        target = 1.57  # 90 degrees
        
        # Send commands simultaneously
        for motor in motors:
            motor.set_position(target)
        
        # Monitor synchronization
        max_sync_error = 0
        for _ in range(50):
            positions = []
            for motor in motors:
                motor.get_parameter(0x7019)
                time.sleep(0.005)
                positions.append(motor.status.mech_pos)
            
            # Calculate synchronization error (max deviation)
            sync_error = max(positions) - min(positions)
            max_sync_error = max(max_sync_error, sync_error)
            
            print(f"Positions: {[f'{p:.3f}' for p in positions]}, Sync error: {sync_error:.4f} rad")
            
            time.sleep(0.1)
        
        print(f"Max synchronization error: {max_sync_error:.4f} rad")
        
        # Acceptance: Sync error < 0.05 rad
        assert max_sync_error < 0.05, "Synchronization error too large"
        
    finally:
        for motor in motors:
            motor.set_position(0.0)
        time.sleep(2.0)
        for motor in motors:
            motor.disable()
            motor.close()
        can_bus.shutdown()
```

---

## 7. Acceptance Tests / 受け入れテスト

### 7.1 User Story Tests

**Test File:** `tests/acceptance/test_user_stories.py`

#### TC-A-001: Pick and Place Operation
```python
@pytest.mark.hardware
@pytest.mark.acceptance
def test_pick_and_place():
    """
    User Story: As a robot operator, I want to move the motor to specific 
    positions for pick and place operations.
    """
    can_bus = can.interface.Bus(channel='can0', bustype='socketcan')
    motor = RobStrideMotor(can_id=1, can_bus=can_bus)
    
    try:
        # Setup
        motor.enable()
        motor.set_control_mode(ControlMode.POSITION_PP)
        motor.set_parameter(0x7024, 3.0, value_mode='p')  # Speed
        motor.set_parameter(0x7018, 5.0, value_mode='p')  # Current
        time.sleep(0.2)
        
        # Home position
        motor.set_position(0.0)
        time.sleep(2.0)
        
        # Pick position
        print("Moving to pick position...")
        motor.set_position(1.57)
        time.sleep(3.0)
        
        # Verify arrival
        motor.get_parameter(0x7019)
        time.sleep(0.1)
        assert abs(motor.status.mech_pos - 1.57) < 0.05
        
        # Place position
        print("Moving to place position...")
        motor.set_position(3.14)
        time.sleep(3.0)
        
        # Verify arrival
        motor.get_parameter(0x7019)
        time.sleep(0.1)
        assert abs(motor.status.mech_pos - 3.14) < 0.05
        
        # Return home
        print("Returning home...")
        motor.set_position(0.0)
        time.sleep(3.0)
        
        print("Pick and place operation successful!")
        
    finally:
        motor.disable()
        motor.close()
        can_bus.shutdown()
```

#### TC-A-002: Continuous Speed Control
```python
@pytest.mark.hardware
@pytest.mark.acceptance
def test_continuous_speed_control():
    """
    User Story: As a conveyor operator, I want to control motor speed 
    smoothly for continuous operation.
    """
    can_bus = can.interface.Bus(channel='can0', bustype='socketcan')
    motor = RobStrideMotor(can_id=1, can_bus=can_bus)
    
    try:
        # Setup
        motor.enable()
        motor.set_control_mode(ControlMode.SPEED)
        motor.set_parameter(0x7018, 3.0, value_mode='p')
        motor.set_parameter(0x7022, 5.0, value_mode='p')  # Smooth acceleration
        time.sleep(0.2)
        
        # Start at low speed
        motor.set_speed(2.0)
        time.sleep(2.0)
        
        # Increase to medium speed
        motor.set_speed(10.0)
        time.sleep(5.0)
        
        # Decrease to low speed
        motor.set_speed(2.0)
        time.sleep(2.0)
        
        # Stop
        motor.set_speed(0.0)
        time.sleep(2.0)
        
        print("Continuous speed control successful!")
        
    finally:
        motor.disable()
        motor.close()
        can_bus.shutdown()
```

---

## 8. Performance Benchmarks / 性能ベンチマーク

### 8.1 Communication Latency

**Test File:** `tests/performance/test_latency.py`

#### TC-P-001: Command Response Time
```python
@pytest.mark.hardware
@pytest.mark.performance
def test_command_response_time():
    """Measure command to response latency"""
    can_bus = can.interface.Bus(channel='can0', bustype='socketcan')
    motor = RobStrideMotor(can_id=1, can_bus=can_bus)
    
    try:
        motor.enable()
        time.sleep(0.2)
        
        latencies = []
        
        for _ in range(100):
            start = time.time()
            motor.get_parameter(0x7019)
            # Wait for response (blocking or callback)
            time.sleep(0.01)  # Adjust based on implementation
            latency = time.time() - start
            latencies.append(latency * 1000)  # Convert to ms
        
        avg_latency = sum(latencies) / len(latencies)
        max_latency = max(latencies)
        min_latency = min(latencies)
        
        print(f"Average latency: {avg_latency:.2f} ms")
        print(f"Min latency: {min_latency:.2f} ms")
        print(f"Max latency: {max_latency:.2f} ms")
        
        # Acceptance: Average < 10 ms
        assert avg_latency < 10.0, "Average latency too high"
        
    finally:
        motor.disable()
        motor.close()
        can_bus.shutdown()
```

### 8.2 Control Loop Frequency

#### TC-P-002: Maximum Control Frequency
```python
@pytest.mark.hardware
@pytest.mark.performance
def test_maximum_control_frequency():
    """Measure maximum achievable control loop frequency"""
    can_bus = can.interface.Bus(channel='can0', bustype='socketcan')
    motor = RobStrideMotor(can_id=1, can_bus=can_bus)
    
    try:
        motor.enable()
        motor.set_control_mode(ControlMode.POSITION_CSP)
        time.sleep(0.2)
        
        # Run control loop for 10 seconds
        duration = 10.0
        count = 0
        start_time = time.time()
        
        while time.time() - start_time < duration:
            motor.set_position(0.0)
            count += 1
        
        frequency = count / duration
        
        print(f"Achieved control frequency: {frequency:.1f} Hz")
        
        # Acceptance: Frequency > 100 Hz
        assert frequency > 100.0, "Control frequency too low"
        
    finally:
        motor.disable()
        motor.close()
        can_bus.shutdown()
```

---

## 9. Test Execution / テスト実行

### 9.1 Running Tests

```bash
# Run all unit tests
pytest tests/unit -v

# Run with coverage
pytest tests/unit --cov=robstride --cov-report=html

# Run integration tests
pytest tests/integration -v

# Run hardware tests (requires real motor)
pytest -m hardware -v

# Run specific test
pytest tests/unit/test_motor_init.py::test_valid_motor_id -v

# Run tests with specific marker
pytest -m "not hardware" -v  # Skip hardware tests
```

### 9.2 Continuous Integration

**GitHub Actions Configuration (.github/workflows/test.yml):**
```yaml
name: Tests

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    
    steps:
    - uses: actions/checkout@v2
    
    - name: Set up Python
      uses: actions/setup-python@v2
      with:
        python-version: '3.10'
    
    - name: Install dependencies
      run: |
        pip install -e .
        pip install pytest pytest-cov pytest-mock
    
    - name: Setup virtual CAN
      run: |
        sudo modprobe vcan
        sudo ip link add dev vcan0 type vcan
        sudo ip link set up vcan0
    
    - name: Run unit tests
      run: |
        pytest tests/unit --cov=robstride --cov-report=xml
    
    - name: Upload coverage
      uses: codecov/codecov-action@v2
```

---

## 10. Test Documentation / テスト文書化

### 10.1 Test Report Template

```markdown
# Test Execution Report

**Date:** 2025-10-09  
**Tester:** Your Name  
**Environment:** Hardware / Virtual CAN  
**Software Version:** robstride-motor v1.0.0

## Summary

| Category | Total | Passed | Failed | Skipped |
| -------- | ----- | ------ | ------ | ------- |
| Unit     | 50    | 48     | 2      | 0       |
| Integration | 20 | 18     | 1      | 1       |
| System   | 15    | 14     | 1      | 0       |
| HIL      | 5     | 5      | 0      | 0       |
| Acceptance | 3   | 3      | 0      | 0       |

## Failed Tests

### TC-U-025: Float Precision
- **Error:** Assertion failed, expected 1.57, got 1.5700001
- **Root Cause:** Floating point precision
- **Action:** Increase tolerance to 1e-5

## Recommendations

1. Improve error handling in edge cases
2. Add more validation tests
3. Increase test coverage to > 90%
```

---

**End of Test Specification**
