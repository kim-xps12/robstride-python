# RobStride Motor Control Library - Python Implementation Guide
# RobStride モーター制御ライブラリ - Python 実装ガイド

**Document Version:** 1.0  
**Date:** 2025-10-09

---

## 1. Overview / 概要

本ドキュメントは、C++ 実装を Python に移植するための具体的な実装ガイドである。パッケージ構成、クラス設計、依存ライブラリ、コード例を提供する。

---

## 2. Package Structure / パッケージ構成

### 2.1 Recommended Directory Layout

```
robstride/
├── __init__.py
├── motor.py              # Main motor control class
├── protocol/
│   ├── __init__.py
│   ├── private.py        # Private protocol implementation
│   ├── mit.py            # MIT protocol implementation
│   └── can_utils.py      # CAN message utilities
├── data/
│   ├── __init__.py
│   ├── structures.py     # Data structures (MotorStatus, etc.)
│   ├── parameters.py     # Parameter definitions
│   └── enums.py          # Enumerations (ControlMode, etc.)
├── control/
│   ├── __init__.py
│   ├── position.py       # Position control strategies
│   ├── speed.py          # Speed control strategies
│   └── current.py        # Current control strategies
├── error/
│   ├── __init__.py
│   ├── handler.py        # Error handling logic
│   └── logger.py         # Error logging
├── utils/
│   ├── __init__.py
│   ├── validation.py     # Input validation
│   └── conversion.py     # Unit conversions
└── examples/
    ├── basic_position.py
    ├── speed_control.py
    ├── mit_mode.py
    └── multi_motor.py
```

### 2.2 Installation Configuration

**setup.py:**
```python
from setuptools import setup, find_packages

setup(
    name="robstride-motor",
    version="1.0.0",
    description="Python library for RobStride motor control via CAN bus",
    author="Your Name",
    author_email="your.email@example.com",
    packages=find_packages(),
    install_requires=[
        "python-can>=4.0.0",
        "dataclasses;python_version<'3.7'",
        "typing-extensions>=4.0.0"
    ],
    extras_require={
        "dev": [
            "pytest>=7.0.0",
            "pytest-cov>=3.0.0",
            "black>=22.0.0",
            "mypy>=0.950"
        ]
    },
    python_requires=">=3.8",
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Topic :: Software Development :: Libraries :: Python Modules",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11"
    ]
)
```

**requirements.txt:**
```
python-can>=4.0.0
typing-extensions>=4.0.0
```

---

## 3. Core Classes / 核となるクラス

### 3.1 RobStrideMotor Class

**robstride/motor.py:**
```python
from typing import Optional, Callable
import can
import time
from .data.structures import MotorStatus, MotorConfiguration, ControlCommand
from .data.enums import ControlMode, ProtocolMode, ErrorFlag
from .protocol.private import PrivateProtocolHandler
from .protocol.mit import MITProtocolHandler
from .error.handler import ErrorHandler

class RobStrideMotor:
    """Main interface for RobStride motor control"""
    
    def __init__(
        self,
        can_id: int,
        can_bus: can.Bus,
        protocol: ProtocolMode = ProtocolMode.PRIVATE,
        auto_enable: bool = False
    ):
        """
        Initialize motor controller
        
        Args:
            can_id: Motor CAN ID (1-32)
            can_bus: python-can Bus instance
            protocol: Initial protocol mode
            auto_enable: Automatically enable motor on init
        """
        # Validate inputs
        if not (1 <= can_id <= 32):
            raise ValueError("CAN ID must be between 1 and 32")
        
        self.can_id = can_id
        self.can_bus = can_bus
        self.protocol_mode = protocol
        
        # Internal state
        self.status = MotorStatus()
        self.config = MotorConfiguration()
        self.enabled = False
        
        # Protocol handlers
        self.private_handler = PrivateProtocolHandler(can_id, can_bus)
        self.mit_handler = MITProtocolHandler(can_id, can_bus)
        
        # Error handling
        self.error_handler = ErrorHandler(self)
        
        # Callback for status updates
        self.status_callback: Optional[Callable[[MotorStatus], None]] = None
        
        # Start CAN listener
        self._start_listener()
        
        if auto_enable:
            self.enable()
    
    def _start_listener(self):
        """Start background CAN message listener"""
        self.notifier = can.Notifier(self.can_bus, [self._can_callback])
    
    def _can_callback(self, msg: can.Message):
        """Process incoming CAN messages"""
        if self.protocol_mode == ProtocolMode.PRIVATE:
            self.private_handler.process_message(msg, self.status)
        else:
            self.mit_handler.process_message(msg, self.status)
        
        # Trigger user callback
        if self.status_callback:
            self.status_callback(self.status)
    
    # === Core Control Methods ===
    
    def enable(self) -> bool:
        """
        Enable motor (Private protocol)
        
        Returns:
            True if successful
        """
        if self.protocol_mode != ProtocolMode.PRIVATE:
            raise RuntimeError("Enable only available in Private protocol")
        
        success = self.private_handler.send_enable()
        if success:
            self.enabled = True
            time.sleep(0.1)  # Wait for motor to respond
        return success
    
    def disable(self) -> bool:
        """
        Disable motor
        
        Returns:
            True if successful
        """
        if self.protocol_mode == ProtocolMode.PRIVATE:
            success = self.private_handler.send_disable()
        else:
            success = self.mit_handler.send_stop()
        
        if success:
            self.enabled = False
        return success
    
    # === Parameter Access ===
    
    def set_parameter(self, index: int, value: float, value_mode: str = 'p') -> bool:
        """
        Set motor parameter
        
        Args:
            index: Parameter index (0x7xxx)
            value: Parameter value
            value_mode: 'p' for float, 'j' for int
        
        Returns:
            True if successful
        """
        if self.protocol_mode != ProtocolMode.PRIVATE:
            raise RuntimeError("Parameter setting only in Private protocol")
        
        return self.private_handler.write_parameter(index, value, value_mode)
    
    def get_parameter(self, index: int) -> bool:
        """
        Request parameter read
        
        Args:
            index: Parameter index (0x7xxx)
        
        Returns:
            True if request sent (value updated via callback)
        """
        if self.protocol_mode != ProtocolMode.PRIVATE:
            raise RuntimeError("Parameter reading only in Private protocol")
        
        return self.private_handler.read_parameter(index)
    
    # === Control Mode Methods ===
    
    def set_control_mode(self, mode: ControlMode) -> bool:
        """
        Set control mode
        
        Args:
            mode: ControlMode enum value
        
        Returns:
            True if successful
        """
        # Disable before mode change
        if self.enabled:
            self.disable()
            time.sleep(0.1)
        
        # Write mode parameter
        success = self.set_parameter(0x7005, mode.value, value_mode='j')
        time.sleep(0.05)
        
        # Re-enable
        if success:
            self.enable()
        
        return success
    
    def set_position(self, position_rad: float) -> bool:
        """
        Set target position (Position mode only)
        
        Args:
            position_rad: Target position in radians
        
        Returns:
            True if successful
        """
        if not self.enabled:
            raise RuntimeError("Motor must be enabled")
        
        return self.set_parameter(0x7016, position_rad, value_mode='p')
    
    def set_speed(self, speed_rad_s: float) -> bool:
        """
        Set target speed (Speed mode only)
        
        Args:
            speed_rad_s: Target speed in rad/s
        
        Returns:
            True if successful
        """
        if not self.enabled:
            raise RuntimeError("Motor must be enabled")
        
        return self.set_parameter(0x700A, speed_rad_s, value_mode='p')
    
    def set_current(self, current_a: float) -> bool:
        """
        Set target current (Current mode only)
        
        Args:
            current_a: Target current in amperes
        
        Returns:
            True if successful
        """
        if not self.enabled:
            raise RuntimeError("Motor must be enabled")
        
        return self.set_parameter(0x7006, current_a, value_mode='p')
    
    # === MIT Protocol Methods ===
    
    def switch_to_mit(self) -> bool:
        """Switch from Private to MIT protocol"""
        if self.enabled:
            self.disable()
            time.sleep(0.1)
        
        success = self.private_handler.send_protocol_switch(to_mit=True)
        if success:
            self.protocol_mode = ProtocolMode.MIT
            time.sleep(0.05)
        
        return success
    
    def switch_to_private(self) -> bool:
        """Switch from MIT to Private protocol"""
        if self.protocol_mode != ProtocolMode.MIT:
            return False
        
        success = self.mit_handler.send_stop()
        if success:
            self.protocol_mode = ProtocolMode.PRIVATE
            time.sleep(0.05)
        
        return success
    
    def enable_mit_mode(self) -> bool:
        """Enable MIT control mode"""
        if self.protocol_mode != ProtocolMode.MIT:
            raise RuntimeError("Must be in MIT protocol")
        
        success = self.mit_handler.send_enable()
        if success:
            self.enabled = True
            time.sleep(0.05)
        
        return success
    
    def send_mit_command(
        self,
        torque: float = 0.0,
        position: float = 0.0,
        velocity: float = 0.0,
        kp: float = 0.0,
        kd: float = 0.0
    ) -> bool:
        """
        Send MIT protocol command
        
        Args:
            torque: Feedforward torque (Nm)
            position: Target position (rad)
            velocity: Target velocity (rad/s)
            kp: Position gain
            kd: Velocity gain
        
        Returns:
            True if successful
        """
        if not self.enabled or self.protocol_mode != ProtocolMode.MIT:
            raise RuntimeError("MIT mode must be enabled")
        
        return self.mit_handler.send_control_command(
            torque, position, velocity, kp, kd
        )
    
    # === Utility Methods ===
    
    def set_zero_position(self) -> bool:
        """Set current position as zero reference"""
        return self.private_handler.send_zero_position()
    
    def clear_errors(self) -> bool:
        """Clear error flags"""
        return self.private_handler.send_clear_errors()
    
    def save_parameters(self) -> bool:
        """Save current parameters to FLASH"""
        return self.private_handler.send_save_parameters()
    
    def get_status(self) -> MotorStatus:
        """Get current motor status (snapshot)"""
        return self.status.copy()
    
    def close(self):
        """Cleanup resources"""
        self.disable()
        self.notifier.stop()
```

---

### 3.2 Protocol Handlers

**robstride/protocol/private.py:**
```python
import can
import struct
from typing import Optional
from ..data.structures import MotorStatus
from ..utils.conversion import float_to_uint, uint_to_float

class PrivateProtocolHandler:
    """Handles Private (Extended CAN ID) protocol"""
    
    def __init__(self, motor_id: int, can_bus: can.Bus):
        self.motor_id = motor_id
        self.can_bus = can_bus
    
    def _build_extended_id(self, cmd_type: int) -> int:
        """
        Build 29-bit Extended CAN ID
        
        Format: [28:24]=cmd_type, [23:16]=0x00, [15:8]=motor_id, [7:0]=0x00
        """
        return (cmd_type << 24) | (self.motor_id << 8)
    
    def send_enable(self) -> bool:
        """Send enable command (type 0x01)"""
        can_id = self._build_extended_id(0x01)
        msg = can.Message(
            arbitration_id=can_id,
            data=[],  # No data
            is_extended_id=True
        )
        
        try:
            self.can_bus.send(msg)
            return True
        except can.CanError as e:
            print(f"CAN send error: {e}")
            return False
    
    def send_disable(self) -> bool:
        """Send disable command (type 0x02)"""
        can_id = self._build_extended_id(0x02)
        msg = can.Message(
            arbitration_id=can_id,
            data=[],
            is_extended_id=True
        )
        
        try:
            self.can_bus.send(msg)
            return True
        except can.CanError:
            return False
    
    def write_parameter(self, index: int, value: float, value_mode: str) -> bool:
        """
        Write parameter (type 0x12)
        
        Args:
            index: Parameter index (0x7xxx)
            value: Parameter value
            value_mode: 'p' for float, 'j' for int
        """
        can_id = self._build_extended_id(0x12)
        
        # Encode value
        if value_mode == 'p':
            value_bytes = struct.pack('<f', value)  # Little-endian float
        elif value_mode == 'j':
            value_bytes = struct.pack('<I', int(value))  # Little-endian uint32
        else:
            raise ValueError("value_mode must be 'p' or 'j'")
        
        # Build data: [index_low, index_high, value[0:3]]
        data = [
            index & 0xFF,
            (index >> 8) & 0xFF
        ] + list(value_bytes[:4])
        
        msg = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=True
        )
        
        try:
            self.can_bus.send(msg)
            return True
        except can.CanError:
            return False
    
    def read_parameter(self, index: int) -> bool:
        """
        Read parameter (type 0x11)
        
        Args:
            index: Parameter index (0x7xxx)
        """
        can_id = self._build_extended_id(0x11)
        
        data = [
            index & 0xFF,
            (index >> 8) & 0xFF
        ]
        
        msg = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=True
        )
        
        try:
            self.can_bus.send(msg)
            return True
        except can.CanError:
            return False
    
    def send_zero_position(self) -> bool:
        """Send set zero position command (type 0x06)"""
        can_id = self._build_extended_id(0x06)
        msg = can.Message(
            arbitration_id=can_id,
            data=[0x01],  # Subcommand 0x01
            is_extended_id=True
        )
        
        try:
            self.can_bus.send(msg)
            return True
        except can.CanError:
            return False
    
    def send_clear_errors(self) -> bool:
        """Send clear errors command (custom implementation)"""
        # Implementation depends on firmware version
        # Typically: write 0 to error register or specific command
        return self.write_parameter(0x701E, 0, value_mode='j')
    
    def send_save_parameters(self) -> bool:
        """Send save to FLASH command (type 0x09)"""
        can_id = self._build_extended_id(0x09)
        msg = can.Message(
            arbitration_id=can_id,
            data=[],
            is_extended_id=True
        )
        
        try:
            self.can_bus.send(msg)
            return True
        except can.CanError:
            return False
    
    def send_protocol_switch(self, to_mit: bool) -> bool:
        """
        Switch protocol (type 0x0C)
        
        Args:
            to_mit: True to switch to MIT, False to Private
        """
        can_id = self._build_extended_id(0x0C)
        data = [0x00 if to_mit else 0x01]
        
        msg = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=True
        )
        
        try:
            self.can_bus.send(msg)
            return True
        except can.CanError:
            return False
    
    def process_message(self, msg: can.Message, status: MotorStatus):
        """
        Process incoming CAN message and update status
        
        Args:
            msg: CAN message
            status: MotorStatus object to update
        """
        if not msg.is_extended_id:
            return
        
        # Extract fields from Extended ID
        cmd_type = (msg.arbitration_id >> 24) & 0x1F
        motor_id = (msg.arbitration_id >> 8) & 0xFF
        
        if motor_id != self.motor_id:
            return
        
        # Parse based on command type
        if cmd_type == 0x00:  # Status feedback
            self._parse_status_feedback(msg.data, status)
        elif cmd_type == 0x11:  # Parameter read response
            self._parse_parameter_response(msg.data, status)
    
    def _parse_status_feedback(self, data: bytes, status: MotorStatus):
        """Parse status feedback (type 0x00)"""
        if len(data) < 8:
            return
        
        # Extract fields (little-endian)
        angle_raw = struct.unpack('<h', data[0:2])[0]  # int16
        speed_raw = struct.unpack('<h', data[2:4])[0]
        torque_raw = struct.unpack('<h', data[4:6])[0]
        temp_raw = data[6]
        error_flags = data[7] if len(data) > 7 else 0
        
        # Convert to physical units
        status.angle = angle_raw * 0.01  # 0.01 rad per LSB
        status.velocity = speed_raw * 0.01  # 0.01 rad/s per LSB
        status.torque = torque_raw * 0.01  # 0.01 Nm per LSB
        status.temperature = temp_raw
        status.error_flags = error_flags
        status.last_update = time.time()
    
    def _parse_parameter_response(self, data: bytes, status: MotorStatus):
        """Parse parameter read response"""
        if len(data) < 6:
            return
        
        index = data[0] | (data[1] << 8)
        value_bytes = data[2:6]
        
        # Decode as float (most common)
        value = struct.unpack('<f', bytes(value_bytes))[0]
        
        # Update status based on index
        if index == 0x7019:
            status.mech_pos = value
        elif index == 0x701B:
            status.mech_vel = value
        elif index == 0x701A:
            status.iqf = value
        elif index == 0x701C:
            status.vbus = value
        elif index == 0x701F:
            status.temperature = int(value)
        elif index == 0x7005:
            status.run_mode = int(value)
        # Add more as needed
```

**robstride/protocol/mit.py:**
```python
import can
import struct
from ..data.structures import MotorStatus
from ..utils.conversion import float_to_uint, uint_to_float

class MITProtocolHandler:
    """Handles MIT (Standard CAN ID) protocol"""
    
    def __init__(self, motor_id: int, can_bus: can.Bus):
        self.motor_id = motor_id
        self.can_bus = can_bus
        self.can_id = 0x7FF  # Standard MIT ID
    
    def send_enable(self) -> bool:
        """Send MIT enable command"""
        data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]
        
        msg = can.Message(
            arbitration_id=self.can_id,
            data=data,
            is_extended_id=False
        )
        
        try:
            self.can_bus.send(msg)
            return True
        except can.CanError:
            return False
    
    def send_stop(self) -> bool:
        """Send MIT stop/disable command"""
        data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]
        
        msg = can.Message(
            arbitration_id=self.can_id,
            data=data,
            is_extended_id=False
        )
        
        try:
            self.can_bus.send(msg)
            return True
        except can.CanError:
            return False
    
    def send_control_command(
        self,
        torque: float,
        position: float,
        velocity: float,
        kp: float,
        kd: float
    ) -> bool:
        """
        Send MIT control command
        
        Data format (8 bytes):
        [0:1]: Position (-12.5 ~ 12.5 rad, 16-bit)
        [2:3]: Velocity (-65 ~ 65 rad/s, 16-bit)
        [4:5]: Kp (0 ~ 500, 16-bit)
        [6:7]: Kd (0 ~ 5, 16-bit)
        [8:9]: Torque (-18 ~ 18 Nm, 16-bit)
        
        Note: Actual encoding may vary by motor type
        """
        # Encode parameters (simplified, see C++ for exact scaling)
        pos_enc = float_to_uint(position, -12.5, 12.5, 16)
        vel_enc = float_to_uint(velocity, -65.0, 65.0, 16)
        kp_enc = float_to_uint(kp, 0.0, 500.0, 16)
        kd_enc = float_to_uint(kd, 0.0, 5.0, 16)
        torque_enc = float_to_uint(torque, -18.0, 18.0, 16)
        
        data = [
            (pos_enc >> 8) & 0xFF, pos_enc & 0xFF,
            (vel_enc >> 8) & 0xFF, vel_enc & 0xFF,
            (kp_enc >> 8) & 0xFF, kp_enc & 0xFF,
            (kd_enc >> 8) & 0xFF, kd_enc & 0xFF
        ]
        
        # Some variants may include torque in extended data
        # Consult datasheet for exact format
        
        msg = can.Message(
            arbitration_id=self.can_id,
            data=data,
            is_extended_id=False
        )
        
        try:
            self.can_bus.send(msg)
            return True
        except can.CanError:
            return False
    
    def process_message(self, msg: can.Message, status: MotorStatus):
        """Process MIT protocol feedback"""
        if msg.is_extended_id or msg.arbitration_id != self.can_id:
            return
        
        if len(msg.data) < 8:
            return
        
        # Parse MIT feedback (format varies by motor)
        # Typically: [angle, velocity, torque, ...]
        
        angle_raw = (msg.data[0] << 8) | msg.data[1]
        vel_raw = (msg.data[2] << 8) | msg.data[3]
        torque_raw = (msg.data[4] << 8) | msg.data[5]
        
        status.angle = uint_to_float(angle_raw, -12.5, 12.5, 16)
        status.velocity = uint_to_float(vel_raw, -65.0, 65.0, 16)
        status.torque = uint_to_float(torque_raw, -18.0, 18.0, 16)
        status.last_update = time.time()
```

---

### 3.3 Utility Functions

**robstride/utils/conversion.py:**
```python
def float_to_uint(value: float, min_val: float, max_val: float, bits: int) -> int:
    """
    Convert float to unsigned int with range mapping
    
    Args:
        value: Float value to convert
        min_val: Minimum of range
        max_val: Maximum of range
        bits: Number of bits (8, 12, 16, etc.)
    
    Returns:
        Encoded unsigned integer
    """
    # Clamp value
    value = max(min_val, min(max_val, value))
    
    # Map to [0, 2^bits - 1]
    max_uint = (1 << bits) - 1
    ratio = (value - min_val) / (max_val - min_val)
    
    return int(ratio * max_uint)

def uint_to_float(value: int, min_val: float, max_val: float, bits: int) -> float:
    """
    Convert unsigned int to float with range mapping
    
    Args:
        value: Unsigned integer
        min_val: Minimum of range
        max_val: Maximum of range
        bits: Number of bits
    
    Returns:
        Decoded float value
    """
    max_uint = (1 << bits) - 1
    ratio = value / max_uint
    
    return min_val + ratio * (max_val - min_val)
```

**robstride/utils/validation.py:**
```python
from ..data.enums import ParameterIndex

def validate_motor_id(motor_id: int):
    """Validate motor CAN ID"""
    if not (1 <= motor_id <= 32):
        raise ValueError("Motor ID must be between 1 and 32")

def validate_parameter_value(index: int, value: float):
    """Validate parameter value against known limits"""
    
    limits = {
        ParameterIndex.IQ_REF: (-23.0, 23.0),
        ParameterIndex.SPD_REF: (-30.0, 30.0),
        ParameterIndex.LOC_REF: (float('-inf'), float('inf')),
        ParameterIndex.LIMIT_TORQUE: (0.0, 12.0),
        ParameterIndex.LIMIT_SPD: (0.0, 44.0),
        ParameterIndex.LIMIT_CUR: (0.0, 23.0)
    }
    
    if index in limits:
        min_val, max_val = limits[index]
        if not (min_val <= value <= max_val):
            raise ValueError(
                f"Parameter {hex(index)} value {value} out of range "
                f"[{min_val}, {max_val}]"
            )
```

---

## 4. Usage Examples / 使用例

### 4.1 Basic Position Control

**examples/basic_position.py:**
```python
import can
import time
from robstride import RobStrideMotor
from robstride.data.enums import ControlMode

# Initialize CAN bus
can_bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000)

# Create motor instance
motor = RobStrideMotor(can_id=1, can_bus=can_bus, auto_enable=False)

try:
    # Enable motor
    print("Enabling motor...")
    motor.enable()
    time.sleep(0.2)
    
    # Set to CSP position mode
    print("Setting CSP position mode...")
    motor.set_control_mode(ControlMode.POSITION_CSP)
    time.sleep(0.2)
    
    # Configure limits
    motor.set_parameter(0x7017, 5.0, value_mode='p')  # Speed limit: 5 rad/s
    motor.set_parameter(0x7018, 3.0, value_mode='p')  # Current limit: 3 A
    time.sleep(0.1)
    
    # Move to position
    print("Moving to position 1.57 rad (90 degrees)...")
    motor.set_position(1.57)
    
    # Wait and monitor
    for i in range(50):
        status = motor.get_status()
        print(f"Position: {status.mech_pos:.3f} rad, "
              f"Velocity: {status.mech_vel:.3f} rad/s")
        time.sleep(0.1)
    
    # Return to zero
    print("Returning to zero...")
    motor.set_position(0.0)
    time.sleep(2.0)

finally:
    # Cleanup
    print("Disabling motor...")
    motor.disable()
    motor.close()
    can_bus.shutdown()
```

### 4.2 Speed Control with Monitoring

**examples/speed_control.py:**
```python
import can
import time
from robstride import RobStrideMotor
from robstride.data.enums import ControlMode

can_bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000)
motor = RobStrideMotor(can_id=1, can_bus=can_bus)

try:
    motor.enable()
    motor.set_control_mode(ControlMode.SPEED)
    
    # Set limits
    motor.set_parameter(0x7018, 5.0, value_mode='p')  # Current limit
    motor.set_parameter(0x7022, 10.0, value_mode='p')  # Acceleration
    time.sleep(0.1)
    
    # Ramp up speed
    for speed in range(0, 11):
        motor.set_speed(float(speed))
        print(f"Target speed: {speed} rad/s")
        time.sleep(1.0)
    
    # Ramp down
    for speed in range(10, -1, -1):
        motor.set_speed(float(speed))
        time.sleep(1.0)

finally:
    motor.set_speed(0.0)
    time.sleep(1.0)
    motor.disable()
    motor.close()
    can_bus.shutdown()
```

### 4.3 MIT Protocol Control

**examples/mit_mode.py:**
```python
import can
import time
import math
from robstride import RobStrideMotor

can_bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000)
motor = RobStrideMotor(can_id=1, can_bus=can_bus)

try:
    # Switch to MIT protocol
    print("Switching to MIT protocol...")
    motor.switch_to_mit()
    time.sleep(0.1)
    
    # Enable MIT mode
    motor.enable_mit_mode()
    time.sleep(0.1)
    
    # Impedance control: Kp=10, Kd=0.5
    print("Running impedance control...")
    
    for i in range(200):
        # Sinusoidal target position
        t = i * 0.01
        target_pos = 1.0 * math.sin(2 * math.pi * 0.5 * t)
        
        # Send command
        motor.send_mit_command(
            torque=0.0,
            position=target_pos,
            velocity=0.0,
            kp=10.0,
            kd=0.5
        )
        
        time.sleep(0.01)
    
    # Zero command to stop
    motor.send_mit_command(0, 0, 0, 0, 0)
    time.sleep(0.1)
    
    # Switch back to Private protocol
    print("Switching back to Private protocol...")
    motor.switch_to_private()

finally:
    motor.close()
    can_bus.shutdown()
```

### 4.4 Multi-Motor Control

**examples/multi_motor.py:**
```python
import can
import time
from robstride import RobStrideMotor
from robstride.data.enums import ControlMode

can_bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000)

# Create multiple motor instances
motors = [
    RobStrideMotor(can_id=1, can_bus=can_bus),
    RobStrideMotor(can_id=2, can_bus=can_bus),
    RobStrideMotor(can_id=3, can_bus=can_bus)
]

try:
    # Initialize all motors
    for i, motor in enumerate(motors):
        print(f"Initializing motor {i+1}...")
        motor.enable()
        motor.set_control_mode(ControlMode.POSITION_CSP)
        motor.set_parameter(0x7017, 5.0, value_mode='p')
        motor.set_parameter(0x7018, 3.0, value_mode='p')
        time.sleep(0.1)
    
    # Synchronized motion
    targets = [1.0, 2.0, 3.0]
    
    print("Moving all motors...")
    for motor, target in zip(motors, targets):
        motor.set_position(target)
    
    # Monitor
    for _ in range(50):
        statuses = [m.get_status() for m in motors]
        positions = [s.mech_pos for s in statuses]
        print(f"Positions: {[f'{p:.3f}' for p in positions]}")
        time.sleep(0.1)

finally:
    for motor in motors:
        motor.disable()
        motor.close()
    can_bus.shutdown()
```

---

## 5. Testing / テスト

### 5.1 Unit Tests

**tests/test_motor.py:**
```python
import pytest
import can
from unittest.mock import Mock, patch
from robstride import RobStrideMotor
from robstride.data.enums import ControlMode

@pytest.fixture
def mock_can_bus():
    """Create mock CAN bus"""
    bus = Mock(spec=can.Bus)
    bus.send = Mock(return_value=None)
    return bus

def test_motor_initialization(mock_can_bus):
    """Test motor initialization"""
    motor = RobStrideMotor(can_id=1, can_bus=mock_can_bus, auto_enable=False)
    
    assert motor.can_id == 1
    assert motor.enabled == False
    assert motor.protocol_mode == ProtocolMode.PRIVATE

def test_enable_motor(mock_can_bus):
    """Test motor enable"""
    motor = RobStrideMotor(can_id=1, can_bus=mock_can_bus, auto_enable=False)
    
    result = motor.enable()
    
    assert result == True
    assert motor.enabled == True
    assert mock_can_bus.send.called

def test_set_position(mock_can_bus):
    """Test position setting"""
    motor = RobStrideMotor(can_id=1, can_bus=mock_can_bus)
    motor.enabled = True
    
    result = motor.set_position(1.57)
    
    assert result == True
    assert mock_can_bus.send.called

def test_invalid_motor_id():
    """Test invalid motor ID"""
    with pytest.raises(ValueError):
        RobStrideMotor(can_id=0, can_bus=Mock())
    
    with pytest.raises(ValueError):
        RobStrideMotor(can_id=33, can_bus=Mock())
```

### 5.2 Integration Tests

**tests/test_integration.py:**
```python
import pytest
import can
import time
from robstride import RobStrideMotor

@pytest.mark.hardware
def test_real_motor_enable():
    """Test with real hardware (requires CAN bus)"""
    can_bus = can.interface.Bus(channel='vcan0', bustype='socketcan')
    motor = RobStrideMotor(can_id=1, can_bus=can_bus)
    
    try:
        motor.enable()
        time.sleep(0.1)
        assert motor.enabled == True
    finally:
        motor.disable()
        motor.close()
        can_bus.shutdown()
```

---

## 6. Deployment / デプロイ

### 6.1 Installation

```bash
# From source
git clone https://github.com/yourname/robstride-motor.git
cd robstride-motor
pip install -e .

# From PyPI (if published)
pip install robstride-motor
```

### 6.2 CAN Interface Setup (Linux)

```bash
# Load CAN kernel modules
sudo modprobe can
sudo modprobe can_raw
sudo modprobe vcan

# Setup virtual CAN (for testing)
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

# Setup real CAN (SocketCAN)
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
```

### 6.3 Permissions

```bash
# Add user to dialout group (for CAN access)
sudo usermod -a -G dialout $USER

# Logout and login for changes to take effect
```

---

**End of Python Implementation Guide**
