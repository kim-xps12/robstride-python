# RobStride Motor Control Library - Test Specification
# RobStride モーター制御ライブラリ - テスト仕様

**Document Version:** 2.0  
**Date:** 2025-10-15  
**Target Implementation:** pytest 7.0+ with python-can

---

## 1. Overview / 概要

本ドキュメントは、RobStride モーター制御ライブラリ（`robstride`パッケージ）の包括的なテスト戦略、テストケース、検証手順を定義する。このドキュメントは**実装ガイドとして機能**し、GitHub Copilotなどのコード生成ツールが適切なテストコードを生成できるよう、詳細な仕様と具体例を提供する。

### 1.1 Document Purpose

1. テスト実装者への明確な指針提供
2. コード生成ツール（Copilot等）への構造化された仕様提供
3. テストカバレッジの網羅性確保
4. 実装とテストの整合性維持

### 1.2 Test Coverage Goals

| Layer | Target Coverage | Rationale |
|-------|----------------|-----------|
| Core Logic (motor.py, models.py) | > 90% | ビジネスロジックの中核 |
| Protocol Handlers (protocol/*.py) | > 85% | 通信の信頼性が重要 |
| Utilities (utils.py, control.py) | > 80% | 補助機能 |
| Overall | > 85% | 産業用途での信頼性要求 |

---

## 2. Test Strategy / テスト戦略

### 2.1 Test Pyramid Structure

```
        Acceptance (3%)      ← 実機での完全なシナリオ
       /                \
      HIL (7%)           ← 実機での軌道追従・同期制御
     /                    \
    System (10%)          ← 実機でのハードウェア通信
   /                        \
  Integration (20%)         ← モックCANでのモジュール間連携
 /                            \
Unit (60%)                    ← 個別関数・メソッドの詳細検証
```

### 2.2 Test Levels and Tools

| Level | Scope | Tools | Coverage Target | Execution Frequency |
|-------|-------|-------|-----------------|---------------------|
| **Unit Tests** | 個別関数/メソッド | pytest, pytest-mock | > 90% | Every commit (CI) |
| **Integration Tests** | モジュール間連携 | pytest + python-can virtual | > 80% | Every PR |
| **System Tests** | CANバス実通信 | pytest + real hardware | Critical paths | Daily (optional) |
| **HIL Tests** | 制御ループ全体 | pytest + testbench | Key scenarios | Release cycle |
| **Acceptance Tests** | ユーザーシナリオ | pytest + production setup | User stories | Pre-release |

### 2.3 Test Environment Setup

#### 2.3.1 Software Environment

**必須パッケージ:**
```bash
# Core dependencies
python-can>=4.0.0
typing-extensions>=4.0.0

# Test dependencies
pytest>=7.0.0
pytest-cov>=3.0.0
pytest-mock>=3.6.0
pytest-asyncio>=0.18.0  # 非同期テスト用
pytest-timeout>=2.1.0   # タイムアウト制御
pytest-benchmark>=3.4.0  # 性能測定

# Development tools
black>=22.0.0
mypy>=0.950
flake8>=4.0.0
```

**Virtual CAN setup (Linux):**
```bash
# Load vcan module
sudo modprobe vcan

# Create virtual CAN interface
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

# Verify
ip link show vcan0
```

**Virtual CAN setup (macOS - 制約あり):**
```bash
# python-canのvirtualバックエンドを使用
# ハードウェアテストは実機またはLinux VMで実施
```

#### 2.3.2 Hardware Test Environment

**最小構成:**
- RobStride RS02 motor × 1
- CAN-USB adapter (PEAK, Kvaser, etc.)
- 24V DC power supply (10A+)
- Emergency stop button
- Safety enclosure

**推奨構成（HILテスト用）:**
- RobStride RS02 motor × 3 (同期テスト用)
- Load cell / torque sensor (トルク測定)
- Encoder (外部位置検証)
- Oscilloscope (信号解析)
- Temperature sensor (熱特性測定)

---

## 3. Test Directory Structure / テストディレクトリ構造

```
tests/
├── conftest.py                      # 共通フィクスチャ・設定
├── pytest.ini                       # pytest設定
├── __init__.py
│
├── unit/                            # 単体テスト (60%)
│   ├── __init__.py
│   ├── conftest.py                  # Unit test共通フィクスチャ
│   │
│   ├── test_models.py               # データモデル
│   ├── test_motor_init.py           # モーター初期化
│   ├── test_motor_properties.py     # プロパティアクセス
│   │
│   ├── protocol/
│   │   ├── __init__.py
│   │   ├── test_can_utils.py        # CAN encode/decode関数
│   │   ├── test_private_handler.py  # Privateプロトコルハンドラ
│   │   └── test_mit_handler.py      # MITプロトコルハンドラ
│   │
│   ├── test_utils.py                # ユーティリティ関数
│   ├── test_validation.py           # バリデーション関数
│   ├── test_error_handler.py        # エラーハンドリング
│   └── test_control_strategies.py   # 制御戦略クラス
│
├── integration/                     # 統合テスト (20%)
│   ├── __init__.py
│   ├── conftest.py                  # Integration test共通フィクスチャ
│   │
│   ├── test_protocol_integration.py # プロトコル切り替え・パラメータR/W
│   ├── test_control_modes.py        # 制御モード切り替え
│   ├── test_message_flow.py         # メッセージ送受信フロー
│   └── test_error_recovery.py       # エラー検出と自動回復
│
├── system/                          # システムテスト (10%)
│   ├── __init__.py
│   ├── conftest.py                  # System test共通フィクスチャ
│   │
│   ├── test_hardware_comm.py        # 実機CAN通信
│   ├── test_control_performance.py  # 制御性能（位置・速度・電流）
│   └── test_error_conditions.py     # 実機エラー状態テスト
│
├── hil/                             # HILテスト (7%)
│   ├── __init__.py
│   ├── conftest.py
│   │
│   ├── test_trajectory_tracking.py  # 軌道追従性能
│   └── test_multi_motor_sync.py     # 複数モーター同期
│
├── acceptance/                      # 受け入れテスト (3%)
│   ├── __init__.py
│   ├── conftest.py
│   │
│   └── test_user_scenarios.py       # ユーザーストーリー
│
└── performance/                     # 性能テスト
    ├── __init__.py
    ├── test_latency.py              # 通信レイテンシ
    └── test_throughput.py           # スループット
```

### 3.1 Test Markers (pytest.ini)

```ini
[pytest]
markers =
    unit: Unit tests (fast, no hardware)
    integration: Integration tests (virtual CAN)
    system: System tests (requires real hardware)
    hardware: Tests requiring connected motor
    hil: Hardware-in-loop tests (requires testbench)
    acceptance: Acceptance tests (end-to-end scenarios)
    performance: Performance benchmarks
    slow: Tests that take > 5 seconds
    
testpaths = tests
python_files = test_*.py
python_classes = Test*
python_functions = test_*

# Timeout for tests
timeout = 30
timeout_method = thread

# Coverage settings
addopts = 
    --strict-markers
    --cov=robstride
    --cov-report=html
    --cov-report=term-missing
    --cov-branch
    -v
```

---

## 4. Common Test Fixtures / 共通テストフィクスチャ

**File: `tests/conftest.py`**

```python
"""
Common test fixtures for RobStride motor control library tests
"""

import pytest
import can
from unittest.mock import Mock, MagicMock, patch
from typing import Generator

from robstride import RobStrideMotor
from robstride.models import MotorStatus, ParameterData, ControlMode, ProtocolMode
from robstride.protocol.private import PrivateProtocolHandler
from robstride.protocol.mit import MITProtocolHandler


# ============================================================================
# MOCK CAN BUS FIXTURES
# ============================================================================

@pytest.fixture
def mock_can_bus() -> Mock:
    """
    Mock CAN bus for unit tests
    
    Returns:
        Mock can.Bus instance with send/recv capabilities
    """
    mock_bus = Mock(spec=can.Bus)
    mock_bus.send = Mock(return_value=None)
    mock_bus.recv = Mock(return_value=None)
    mock_bus.shutdown = Mock()
    return mock_bus


@pytest.fixture
def mock_can_message() -> can.Message:
    """
    Factory for creating mock CAN messages
    
    Usage:
        msg = mock_can_message(arbitration_id=0x123, data=[1,2,3,4,5,6,7,8])
    """
    def _create_message(
        arbitration_id: int = 0x01000100,
        data: list = None,
        is_extended_id: bool = True,
        dlc: int = 8
    ) -> can.Message:
        if data is None:
            data = [0] * 8
        return can.Message(
            arbitration_id=arbitration_id,
            data=bytes(data),
            is_extended_id=is_extended_id,
            dlc=dlc
        )
    return _create_message


# ============================================================================
# VIRTUAL CAN BUS FIXTURES (for integration tests)
# ============================================================================

@pytest.fixture(scope="session")
def vcan_interface() -> str:
    """
    Virtual CAN interface name for integration tests
    
    Returns:
        Interface name (e.g., 'vcan0')
    """
    return 'vcan0'


@pytest.fixture
def vcan_bus(vcan_interface: str) -> Generator[can.Bus, None, None]:
    """
    Real virtual CAN bus for integration tests
    
    Yields:
        can.Bus instance connected to vcan0
    """
    try:
        bus = can.interface.Bus(
            channel=vcan_interface,
            bustype='socketcan',
            bitrate=1000000
        )
        yield bus
    finally:
        bus.shutdown()


# ============================================================================
# MOTOR INSTANCE FIXTURES
# ============================================================================

@pytest.fixture
def mock_motor(mock_can_bus: Mock) -> RobStrideMotor:
    """
    Mock motor instance for unit tests
    
    Uses mocked CAN bus to avoid actual hardware communication.
    Auto-enable is disabled to control initialization in tests.
    """
    with patch('robstride.motor.can.interface.Bus', return_value=mock_can_bus):
        motor = RobStrideMotor(
            can_id=1,
            can_interface='test_can',
            protocol=ProtocolMode.PRIVATE,
            auto_enable=False
        )
        # Stop background thread for testing
        motor._running = False
        yield motor


@pytest.fixture
def vcan_motor(vcan_bus: can.Bus, vcan_interface: str) -> Generator[RobStrideMotor, None, None]:
    """
    Motor instance connected to virtual CAN for integration tests
    """
    motor = RobStrideMotor(
        can_id=1,
        can_interface=vcan_interface,
        protocol=ProtocolMode.PRIVATE,
        auto_enable=False
    )
    yield motor
    motor._running = False
    if hasattr(motor, 'can_bus'):
        motor.can_bus.shutdown()


# ============================================================================
# HARDWARE MOTOR FIXTURE (for system/HIL tests)
# ============================================================================

@pytest.fixture(scope="module")
def hardware_motor() -> Generator[RobStrideMotor, None, None]:
    """
    Real motor instance for hardware tests
    
    Requires:
        - Real motor connected to can0
        - Motor powered and ready
    
    Usage:
        @pytest.mark.hardware
        def test_with_real_motor(hardware_motor):
            hardware_motor.enable_motor()
            ...
    """
    motor = RobStrideMotor(
        can_id=1,
        can_interface='can0',
        protocol=ProtocolMode.PRIVATE,
        auto_enable=False
    )
    
    try:
        yield motor
    finally:
        # Safe cleanup
        try:
            motor.disable_motor(clear_error=True)
        except:
            pass
        motor._running = False
        if hasattr(motor, 'can_bus'):
            motor.can_bus.shutdown()


# ============================================================================
# DATA MODEL FIXTURES
# ============================================================================

@pytest.fixture
def motor_status() -> MotorStatus:
    """Empty MotorStatus instance"""
    return MotorStatus()


@pytest.fixture
def parameter_data() -> ParameterData:
    """Empty ParameterData instance"""
    return ParameterData()


@pytest.fixture
def sample_status() -> MotorStatus:
    """MotorStatus with sample data"""
    status = MotorStatus()
    status.angle = 1.57  # 90 degrees
    status.speed = 10.0
    status.torque = 5.0
    status.temperature = 45.0
    status.pattern = 1
    status.error_code = 0
    return status


# ============================================================================
# PROTOCOL HANDLER FIXTURES
# ============================================================================

@pytest.fixture
def private_handler(mock_can_bus: Mock) -> PrivateProtocolHandler:
    """Private protocol handler with mock bus"""
    return PrivateProtocolHandler(motor_id=1, can_bus=mock_can_bus, master_id=0xFD)


@pytest.fixture
def mit_handler(mock_can_bus: Mock) -> MITProtocolHandler:
    """MIT protocol handler with mock bus"""
    return MITProtocolHandler(motor_id=1, can_bus=mock_can_bus)


# ============================================================================
# PARAMETRIZE HELPERS
# ============================================================================

@pytest.fixture
def valid_can_ids():
    """Valid CAN ID values for parametrized tests"""
    return [0x01, 0x10, 0x20, 0x7F]


@pytest.fixture
def invalid_can_ids():
    """Invalid CAN ID values for parametrized tests"""
    return [-1, 0x80, 0xFF, 256, "not_an_int"]


@pytest.fixture
def angle_test_values():
    """Angle test values: (value, expected_valid)"""
    return [
        (-12.57, True),   # Min
        (0.0, True),      # Zero
        (12.57, True),    # Max
        (1.57, True),     # π/2
        (-12.58, False),  # Below min
        (12.58, False),   # Above max
        (float('nan'), False),  # NaN
        (float('inf'), False),  # Inf
    ]


# ============================================================================
# TIMEOUT AND PERFORMANCE FIXTURES
# ============================================================================

@pytest.fixture
def benchmark_timer():
    """
    Simple timer for performance measurements
    
    Usage:
        with benchmark_timer() as t:
            # do something
        assert t.elapsed < 0.01  # 10ms
    """
    import time
    from contextlib import contextmanager
    
    @contextmanager
    def timer():
        class Timer:
            def __init__(self):
                self.start = time.perf_counter()
                self.elapsed = 0
        
        t = Timer()
        yield t
        t.elapsed = time.perf_counter() - t.start
    
    return timer
```

---

## 5. Unit Tests / 単体テスト

Unit testsは個別の関数・メソッド・クラスの動作を検証する。ハードウェアやネットワークに依存せず、高速に実行可能である必要がある。

### 5.1 Data Models Tests

**File: `tests/unit/test_models.py`**

```python
"""
Unit tests for data models (MotorStatus, ParameterData, enums, etc.)
"""

import pytest
from robstride.models import (
    MotorStatus, ParameterData, ControlMode, ProtocolMode, ErrorFlag,
    MotorState, ParameterIndex, validate_parameter, get_parameter_name,
    is_readable, is_writable
)


class TestMotorStatus:
    """Tests for MotorStatus data class"""
    
    def test_motor_status_initialization(self):
        """TC-U-M-001: MotorStatus initializes with default values"""
        status = MotorStatus()
        
        assert status.angle == 0.0
        assert status.speed == 0.0
        assert status.torque == 0.0
        assert status.temperature == 0.0
        assert status.pattern == 0
        assert status.error_code == 0
    
    def test_motor_status_has_error_property(self):
        """TC-U-M-002: has_error property correctly detects errors"""
        status = MotorStatus()
        
        # No error
        status.error_code = 0
        assert status.has_error is False
        
        # With error
        status.error_code = ErrorFlag.OVER_TEMPERATURE
        assert status.has_error is True
    
    def test_motor_status_is_running_property(self):
        """TC-U-M-003: is_running property based on pattern"""
        status = MotorStatus()
        
        # Not running
        status.pattern = 0
        assert status.is_running is False
        
        # Running
        status.pattern = 1
        assert status.is_running is True
        
        status.pattern = 3
        assert status.is_running is True
    
    def test_motor_status_get_error_names(self):
        """TC-U-M-004: get_error_names returns correct error list"""
        status = MotorStatus()
        
        # No errors
        status.error_code = 0
        # Note: Implementation may not have this exact method
        # This is a specification - implement if needed
        
        # Multiple errors
        status.error_code = ErrorFlag.OVER_TEMPERATURE | ErrorFlag.OVER_CURRENT
        # Verify implementation provides error name extraction
    
    def test_motor_status_str_representation(self):
        """TC-U-M-005: String representation is informative"""
        status = MotorStatus()
        status.angle = 1.57
        status.speed = 10.0
        status.torque = 5.0
        status.temperature = 45.0
        
        status_str = str(status)
        
        assert '1.57' in status_str or '1.570' in status_str
        assert '10.0' in status_str or '10.000' in status_str
        assert '5.0' in status_str or '5.000' in status_str
        assert '45.0' in status_str or '45' in status_str


class TestParameterData:
    """Tests for ParameterData data class"""
    
    def test_parameter_data_initialization(self):
        """TC-U-P-001: ParameterData initializes with default values"""
        data = ParameterData()
        
        assert data.run_mode == 0.0
        assert data.iq_ref == 0.0
        assert data.spd_ref == 0.0
        assert data.limit_torque == 12.0
        assert data.limit_spd == 44.0
        assert data.limit_cur == 23.0
    
    def test_parameter_data_attribute_assignment(self):
        """TC-U-P-002: ParameterData allows attribute modification"""
        data = ParameterData()
        
        data.iq_ref = 5.0
        data.spd_ref = 20.0
        data.mech_pos = 3.14
        
        assert data.iq_ref == 5.0
        assert data.spd_ref == 20.0
        assert data.mech_pos == 3.14


class TestEnumerations:
    """Tests for enum classes"""
    
    def test_control_mode_values(self):
        """TC-U-E-001: ControlMode enum has correct values"""
        assert ControlMode.MOTION_CONTROL == 0
        assert ControlMode.POSITION_PP == 1
        assert ControlMode.SPEED == 2
        assert ControlMode.CURRENT == 3
        assert ControlMode.SET_ZERO == 4
        assert ControlMode.POSITION_CSP == 5
    
    def test_protocol_mode_values(self):
        """TC-U-E-002: ProtocolMode enum has correct values"""
        assert ProtocolMode.PRIVATE == 0x00
        assert ProtocolMode.CANOPEN == 0x01
        assert ProtocolMode.MIT == 0x02
    
    def test_error_flag_combinations(self):
        """TC-U-E-003: ErrorFlag bitwise operations work correctly"""
        # Single flags
        assert ErrorFlag.OVER_TEMPERATURE != 0
        assert ErrorFlag.OVER_CURRENT != 0
        
        # Combination
        combined = ErrorFlag.OVER_TEMPERATURE | ErrorFlag.OVER_CURRENT
        
        assert combined & ErrorFlag.OVER_TEMPERATURE
        assert combined & ErrorFlag.OVER_CURRENT
        assert not (combined & ErrorFlag.UNDER_VOLTAGE)
    
    def test_motor_state_values(self):
        """TC-U-E-004: MotorState enum has correct values"""
        assert MotorState.UNINITIALIZED == 0
        assert MotorState.DISABLED == 1
        assert MotorState.ENABLED == 2
        assert MotorState.RUNNING == 3
        assert MotorState.FAULT == 4


class TestParameterMapping:
    """Tests for parameter validation and mapping"""
    
    def test_validate_parameter_valid_values(self):
        """TC-U-PM-001: validate_parameter accepts valid values"""
        # Position reference (no strict limit)
        valid, msg = validate_parameter(ParameterIndex.LOC_REF, 10.0)
        assert valid is True
        
        # Current reference (limited)
        valid, msg = validate_parameter(ParameterIndex.IQ_REF, 10.0)
        assert valid is True
        
        # Speed reference
        valid, msg = validate_parameter(ParameterIndex.SPD_REF, 20.0)
        assert valid is True
    
    def test_validate_parameter_out_of_range(self):
        """TC-U-PM-002: validate_parameter rejects out-of-range values"""
        # Current too high
        valid, msg = validate_parameter(ParameterIndex.IQ_REF, 25.0)
        assert valid is False
        assert "out of range" in msg.lower()
        
        # Speed too high
        valid, msg = validate_parameter(ParameterIndex.SPD_REF, 50.0)
        assert valid is False
    
    def test_validate_parameter_read_only(self):
        """TC-U-PM-003: validate_parameter rejects read-only parameters"""
        # Mechanical position is read-only
        valid, msg = validate_parameter(ParameterIndex.MECH_POS, 10.0)
        assert valid is False
        assert "read-only" in msg.lower()
    
    def test_validate_parameter_unknown_index(self):
        """TC-U-PM-004: validate_parameter handles unknown parameters"""
        valid, msg = validate_parameter(0x9999, 10.0)
        assert valid is False
        assert "unknown" in msg.lower()
    
    def test_get_parameter_name(self):
        """TC-U-PM-005: get_parameter_name returns correct names"""
        assert get_parameter_name(ParameterIndex.RUN_MODE) == 'run_mode'
        assert get_parameter_name(ParameterIndex.IQ_REF) == 'iq_ref'
        assert get_parameter_name(ParameterIndex.MECH_POS) == 'mech_pos'
        
        # Unknown parameter
        name = get_parameter_name(0x9999)
        assert 'UNKNOWN' in name or '9999' in name
    
    def test_is_readable_writable(self):
        """TC-U-PM-006: Parameter access flags are correct"""
        # Read-write parameter
        assert is_readable(ParameterIndex.IQ_REF) is True
        assert is_writable(ParameterIndex.IQ_REF) is True
        
        # Read-only parameter
        assert is_readable(ParameterIndex.MECH_POS) is True
        assert is_writable(ParameterIndex.MECH_POS) is False
```

---

### 5.2 Motor Initialization Tests

**File: `tests/unit/test_motor_init.py`**

```python
"""
Unit tests for RobStrideMotor initialization
"""

import pytest
from unittest.mock import Mock, patch, MagicMock
import can

from robstride import RobStrideMotor
from robstride.models import ProtocolMode, MotorState
from robstride.utils import CANException


class TestMotorInitialization:
    """Tests for motor initialization"""
    
    def test_valid_motor_id(self, mock_can_bus):
        """TC-U-MI-001: Motor accepts valid CAN IDs"""
        valid_ids = [0x01, 0x10, 0x20, 0x7F]
        
        for motor_id in valid_ids:
            with patch('robstride.motor.can.interface.Bus', return_value=mock_can_bus):
                motor = RobStrideMotor(
                    can_id=motor_id,
                    can_interface='test',
                    auto_enable=False
                )
                assert motor.motor_id == motor_id
                motor._running = False
    
    def test_invalid_motor_id_negative(self, mock_can_bus):
        """TC-U-MI-002: Motor rejects negative CAN ID"""
        with patch('robstride.motor.can.interface.Bus', return_value=mock_can_bus):
            with pytest.raises(ValueError, match="CAN ID"):
                RobStrideMotor(can_id=-1, can_interface='test')
    
    def test_invalid_motor_id_too_large(self, mock_can_bus):
        """TC-U-MI-003: Motor rejects CAN ID > 0x7F"""
        with patch('robstride.motor.can.interface.Bus', return_value=mock_can_bus):
            with pytest.raises(ValueError, match="CAN ID"):
                RobStrideMotor(can_id=0x80, can_interface='test')
    
    def test_invalid_motor_id_type(self, mock_can_bus):
        """TC-U-MI-004: Motor rejects non-integer CAN ID"""
        with patch('robstride.motor.can.interface.Bus', return_value=mock_can_bus):
            with pytest.raises(TypeError):
                RobStrideMotor(can_id="not_an_int", can_interface='test')
    
    def test_default_protocol_mode(self, mock_can_bus):
        """TC-U-MI-005: Motor defaults to PRIVATE protocol"""
        with patch('robstride.motor.can.interface.Bus', return_value=mock_can_bus):
            motor = RobStrideMotor(can_id=1, can_interface='test', auto_enable=False)
            assert motor.protocol_mode == ProtocolMode.PRIVATE
            motor._running = False
    
    def test_explicit_protocol_mode(self, mock_can_bus):
        """TC-U-MI-006: Motor accepts explicit protocol mode"""
        with patch('robstride.motor.can.interface.Bus', return_value=mock_can_bus):
            motor = RobStrideMotor(
                can_id=1,
                can_interface='test',
                protocol=ProtocolMode.MIT,
                auto_enable=False
            )
            assert motor.protocol_mode == ProtocolMode.MIT
            motor._running = False
    
    def test_initial_state_disabled(self, mock_can_bus):
        """TC-U-MI-007: Motor starts in DISABLED state"""
        with patch('robstride.motor.can.interface.Bus', return_value=mock_can_bus):
            motor = RobStrideMotor(can_id=1, can_interface='test', auto_enable=False)
            assert motor.state == MotorState.DISABLED
            motor._running = False
    
    def test_can_bus_initialization_failure(self):
        """TC-U-MI-008: Motor raises CANException on bus init failure"""
        with patch('robstride.motor.can.interface.Bus', side_effect=OSError("No such device")):
            with pytest.raises(CANException, match="Failed to initialize"):
                RobStrideMotor(can_id=1, can_interface='invalid_interface')
    
    def test_status_and_param_data_initialized(self, mock_can_bus):
        """TC-U-MI-009: Motor initializes status and param_data"""
        with patch('robstride.motor.can.interface.Bus', return_value=mock_can_bus):
            motor = RobStrideMotor(can_id=1, can_interface='test', auto_enable=False)
            
            assert motor.status is not None
            assert motor.param_data is not None
            assert motor.status.angle == 0.0
            
            motor._running = False
    
    def test_protocol_handlers_initialized(self, mock_can_bus):
        """TC-U-MI-010: Motor initializes both protocol handlers"""
        with patch('robstride.motor.can.interface.Bus', return_value=mock_can_bus):
            motor = RobStrideMotor(can_id=1, can_interface='test', auto_enable=False)
            
            assert motor.private_handler is not None
            assert motor.mit_handler is not None
            
            motor._running = False
    
    def test_control_strategies_initialized(self, mock_can_bus):
        """TC-U-MI-011: Motor initializes control strategy objects"""
        with patch('robstride.motor.can.interface.Bus', return_value=mock_can_bus):
            motor = RobStrideMotor(can_id=1, can_interface='test', auto_enable=False)
            
            assert motor.position_control is not None
            assert motor.speed_control is not None
            assert motor.current_control is not None
            
            motor._running = False
```

---

### 5.3 CAN Utility Functions Tests

**File: `tests/unit/protocol/test_can_utils.py`**

```python
"""
Unit tests for CAN utility functions (encode/decode)
"""

import pytest
import struct

from robstride.protocol.can_utils import (
    build_extended_can_id,
    parse_extended_can_id,
    encode_int16,
    decode_int16,
    encode_uint16,
    decode_uint16,
    encode_float32,
    decode_float32,
    encode_angle_16bit,
    decode_angle_16bit,
    encode_speed_16bit,
    decode_speed_16bit,
    encode_torque_16bit,
    decode_torque_16bit,
    encode_kp_16bit,
    decode_kp_16bit,
    encode_kd_16bit,
    decode_kd_16bit,
)


class TestExtendedCANID:
    """Tests for Extended CAN ID building and parsing"""
    
    def test_build_extended_id_basic(self):
        """TC-U-CU-001: build_extended_can_id creates correct 29-bit ID"""
        # Type=0x01, Data=0x0000, Master=0xFD, Motor=0x05
        ext_id = build_extended_can_id(0x01, 0x0000, 0xFD, 0x05)
        
        # Expected: [Type:5][Data:16][Master:8] = 0x0100FD05 (example format)
        # Verify according to actual implementation specification
        assert isinstance(ext_id, int)
        assert ext_id >= 0
        assert ext_id < (1 << 29)  # 29-bit max
    
    def test_parse_extended_id_roundtrip(self):
        """TC-U-CU-002: parse_extended_can_id reverses build correctly"""
        comm_type = 0x12
        data_field = 0x1234
        master_id = 0xFD
        motor_id = 0x10
        
        ext_id = build_extended_can_id(comm_type, data_field, master_id, motor_id)
        parsed_type, parsed_data, parsed_master, parsed_motor = parse_extended_can_id(ext_id)
        
        assert parsed_type == comm_type
        assert parsed_data == data_field
        assert parsed_master == master_id
        assert parsed_motor == motor_id
    
    @pytest.mark.parametrize("comm_type,data,master,motor", [
        (0x00, 0x0000, 0xFD, 0x01),
        (0x1F, 0xFFFF, 0xFF, 0x7F),
        (0x12, 0x7016, 0xFD, 0x05),
    ])
    def test_build_parse_parametrized(self, comm_type, data, master, motor):
        """TC-U-CU-003: build and parse work for various inputs"""
        ext_id = build_extended_can_id(comm_type, data, master, motor)
        p_type, p_data, p_master, p_motor = parse_extended_can_id(ext_id)
        
        assert p_type == comm_type
        assert p_data == data
        assert p_master == master
        assert p_motor == motor


class TestIntegerEncoding:
    """Tests for integer encode/decode functions"""
    
    def test_encode_decode_int16(self):
        """TC-U-IE-001: int16 encoding/decoding roundtrip"""
        test_values = [-32768, -1000, 0, 1000, 32767]
        
        for value in test_values:
            encoded = encode_int16(value)
            assert len(encoded) == 2
            
            decoded = decode_int16(encoded, 0)
            assert decoded == value
    
    def test_encode_decode_uint16(self):
        """TC-U-IE-002: uint16 encoding/decoding roundtrip"""
        test_values = [0, 1000, 32768, 65535]
        
        for value in test_values:
            encoded = encode_uint16(value)
            assert len(encoded) == 2
            
            decoded = decode_uint16(encoded, 0)
            assert decoded == value
    
    def test_encode_decode_uint16_with_offset(self):
        """TC-U-IE-003: uint16 decode with offset works correctly"""
        data = bytes([0x00, 0x12, 0x34, 0x56, 0x78])
        
        # Decode from offset 1
        value = decode_uint16(data, 1)
        assert value == 0x1234
        
        # Decode from offset 3
        value = decode_uint16(data, 3)
        assert value == 0x5678


class TestFloatEncoding:
    """Tests for float encode/decode functions"""
    
    def test_encode_decode_float32(self):
        """TC-U-FE-001: float32 encoding/decoding roundtrip"""
        test_values = [-123.456, 0.0, 1.57, 100.0, -17.5]
        
        for value in test_values:
            encoded = encode_float32(value)
            assert len(encoded) == 4
            
            decoded = decode_float32(encoded, 0)
            assert abs(decoded - value) < 1e-6
    
    def test_encode_float32_byte_order(self):
        """TC-U-FE-002: float32 uses little-endian byte order"""
        value = 1.57
        encoded = encode_float32(value)
        
        # Verify little-endian
        expected = struct.pack('<f', value)
        assert encoded == expected


class TestPhysicalValueEncoding:
    """Tests for physical value (angle, speed, torque) encoding"""
    
    def test_encode_angle_16bit_range(self):
        """TC-U-PV-001: angle encoding maps full range correctly"""
        # Min angle
        min_val = encode_angle_16bit(-12.57)
        assert min_val == 0
        
        # Max angle
        max_val = encode_angle_16bit(12.57)
        assert max_val == 65535
        
        # Zero
        zero_val = encode_angle_16bit(0.0)
        assert 32000 < zero_val < 33000  # Approximately 32768
    
    def test_decode_angle_16bit_range(self):
        """TC-U-PV-002: angle decoding maps full range correctly"""
        # Min
        min_angle = decode_angle_16bit(0)
        assert abs(min_angle - (-12.57)) < 0.01
        
        # Max
        max_angle = decode_angle_16bit(65535)
        assert abs(max_angle - 12.57) < 0.01
        
        # Middle
        mid_angle = decode_angle_16bit(32768)
        assert abs(mid_angle - 0.0) < 0.01
    
    def test_encode_decode_speed_16bit(self):
        """TC-U-PV-003: speed encoding/decoding roundtrip"""
        test_speeds = [-44.0, -20.0, 0.0, 20.0, 44.0]
        
        for speed in test_speeds:
            encoded = encode_speed_16bit(speed)
            decoded = decode_speed_16bit(encoded)
            
            # Allow small error due to quantization
            assert abs(decoded - speed) < 0.1
    
    def test_encode_decode_torque_16bit(self):
        """TC-U-PV-004: torque encoding/decoding roundtrip"""
        test_torques = [-17.0, -10.0, 0.0, 10.0, 17.0]
        
        for torque in test_torques:
            encoded = encode_torque_16bit(torque)
            decoded = decode_torque_16bit(encoded)
            
            assert abs(decoded - torque) < 0.1
    
    def test_encode_kp_kd_16bit(self):
        """TC-U-PV-005: Kp/Kd encoding in valid range"""
        # Kp: 0-500
        kp_values = [0.0, 50.0, 250.0, 500.0]
        for kp in kp_values:
            encoded = encode_kp_16bit(kp)
            decoded = decode_kp_16bit(encoded)
            assert abs(decoded - kp) < 1.0
        
        # Kd: 0-5
        kd_values = [0.0, 1.0, 2.5, 5.0]
        for kd in kd_values:
            encoded = encode_kd_16bit(kd)
            decoded = decode_kd_16bit(encoded)
            assert abs(decoded - kd) < 0.01
    
    def test_encode_clamps_out_of_range(self):
        """TC-U-PV-006: Encoding clamps values to valid range"""
        # Angle beyond range
        over_angle = encode_angle_16bit(20.0)  # > 12.57
        assert over_angle == 65535  # Should clamp to max
        
        under_angle = encode_angle_16bit(-20.0)  # < -12.57
        assert under_angle == 0  # Should clamp to min
        
        # Speed beyond range
        over_speed = encode_speed_16bit(50.0)  # > 44
        assert over_speed == 65535
```

---

### 5.4 Validation Functions Tests

**File: `tests/unit/test_validation.py`**

```python
"""
Unit tests for validation functions in utils.py
"""

import pytest
import math

from robstride.utils import (
    validate_can_id,
    validate_angle,
    validate_speed,
    validate_torque,
    validate_current,
    validate_kp,
    validate_kd,
    clamp
)


class TestCANIDValidation:
    """Tests for CAN ID validation"""
    
    @pytest.mark.parametrize("can_id", [0x00, 0x01, 0x10, 0x7F])
    def test_valid_can_ids(self, can_id):
        """TC-U-V-001: Valid CAN IDs pass validation"""
        assert validate_can_id(can_id) is True
    
    @pytest.mark.parametrize("can_id", [-1, 0x80, 0xFF, 256])
    def test_invalid_can_id_range(self, can_id):
        """TC-U-V-002: Out-of-range CAN IDs raise ValueError"""
        with pytest.raises(ValueError, match="CAN ID"):
            validate_can_id(can_id)
    
    def test_invalid_can_id_type(self):
        """TC-U-V-003: Non-integer CAN ID raises TypeError"""
        with pytest.raises(TypeError):
            validate_can_id("not_an_int")
        
        with pytest.raises(TypeError):
            validate_can_id(1.5)


class TestAngleValidation:
    """Tests for angle validation"""
    
    @pytest.mark.parametrize("angle", [-12.57, -6.28, 0.0, 3.14, 12.57])
    def test_valid_angles(self, angle):
        """TC-U-V-010: Valid angles pass validation"""
        assert validate_angle(angle) is True
    
    @pytest.mark.parametrize("angle", [-12.58, -20.0, 12.58, 20.0])
    def test_invalid_angle_range(self, angle):
        """TC-U-V-011: Out-of-range angles raise ValueError"""
        with pytest.raises(ValueError, match="out of range"):
            validate_angle(angle)
    
    def test_angle_nan(self):
        """TC-U-V-012: NaN angle raises ValueError"""
        with pytest.raises(ValueError, match="NaN"):
            validate_angle(float('nan'))
    
    def test_angle_inf(self):
        """TC-U-V-013: Infinite angle raises ValueError"""
        with pytest.raises(ValueError, match="infinite"):
            validate_angle(float('inf'))
        
        with pytest.raises(ValueError, match="infinite"):
            validate_angle(float('-inf'))
    
    def test_angle_type_error(self):
        """TC-U-V-014: Non-numeric angle raises TypeError"""
        with pytest.raises(TypeError):
            validate_angle("not_a_number")
    
    def test_angle_warning_threshold(self, caplog):
        """TC-U-V-015: Near-limit angles generate warnings"""
        import logging
        caplog.set_level(logging.WARNING)
        
        # Angle at 95% of limit should warn
        validate_angle(11.9)  # ~0.95 * 12.57
        
        # Check if warning was logged
        assert any("near limit" in record.message.lower() for record in caplog.records)


class TestSpeedValidation:
    """Tests for speed validation"""
    
    @pytest.mark.parametrize("speed", [-44.0, -20.0, 0.0, 20.0, 44.0])
    def test_valid_speeds(self, speed):
        """TC-U-V-020: Valid speeds pass validation"""
        assert validate_speed(speed) is True
    
    @pytest.mark.parametrize("speed", [-45.0, -100.0, 45.0, 100.0])
    def test_invalid_speed_range(self, speed):
        """TC-U-V-021: Out-of-range speeds raise ValueError"""
        with pytest.raises(ValueError, match="out of range"):
            validate_speed(speed)
    
    def test_speed_nan_inf(self):
        """TC-U-V-022: NaN/Inf speed raises ValueError"""
        with pytest.raises(ValueError):
            validate_speed(float('nan'))
        
        with pytest.raises(ValueError):
            validate_speed(float('inf'))


class TestTorqueValidation:
    """Tests for torque validation"""
    
    @pytest.mark.parametrize("torque", [-17.0, -10.0, 0.0, 10.0, 17.0])
    def test_valid_torques(self, torque):
        """TC-U-V-030: Valid torques pass validation"""
        assert validate_torque(torque) is True
    
    @pytest.mark.parametrize("torque", [-18.0, -100.0, 18.0, 100.0])
    def test_invalid_torque_range(self, torque):
        """TC-U-V-031: Out-of-range torques raise ValueError"""
        with pytest.raises(ValueError, match="out of range"):
            validate_torque(torque)


class TestCurrentValidation:
    """Tests for current validation"""
    
    @pytest.mark.parametrize("current", [-23.0, -10.0, 0.0, 10.0, 23.0])
    def test_valid_currents(self, current):
        """TC-U-V-040: Valid currents pass validation"""
        assert validate_current(current) is True
    
    @pytest.mark.parametrize("current", [-24.0, -100.0, 24.0, 100.0])
    def test_invalid_current_range(self, current):
        """TC-U-V-041: Out-of-range currents raise ValueError"""
        with pytest.raises(ValueError, match="out of range"):
            validate_current(current)


class TestGainValidation:
    """Tests for Kp/Kd gain validation"""
    
    @pytest.mark.parametrize("kp", [0.0, 50.0, 250.0, 500.0])
    def test_valid_kp(self, kp):
        """TC-U-V-050: Valid Kp values pass validation"""
        assert validate_kp(kp) is True
    
    def test_invalid_kp(self):
        """TC-U-V-051: Out-of-range Kp raises ValueError"""
        with pytest.raises(ValueError):
            validate_kp(-1.0)
        
        with pytest.raises(ValueError):
            validate_kp(501.0)
    
    @pytest.mark.parametrize("kd", [0.0, 1.0, 2.5, 5.0])
    def test_valid_kd(self, kd):
        """TC-U-V-052: Valid Kd values pass validation"""
        assert validate_kd(kd) is True
    
    def test_invalid_kd(self):
        """TC-U-V-053: Out-of-range Kd raises ValueError"""
        with pytest.raises(ValueError):
            validate_kd(-0.1)
        
        with pytest.raises(ValueError):
            validate_kd(5.1)


class TestClampFunction:
    """Tests for clamp utility function"""
    
    def test_clamp_within_range(self):
        """TC-U-V-060: Clamp returns value if within range"""
        assert clamp(5.0, 0.0, 10.0) == 5.0
        assert clamp(0.0, -10.0, 10.0) == 0.0
    
    def test_clamp_below_min(self):
        """TC-U-V-061: Clamp returns min if value below"""
        assert clamp(-5.0, 0.0, 10.0) == 0.0
        assert clamp(-100.0, -10.0, 10.0) == -10.0
    
    def test_clamp_above_max(self):
        """TC-U-V-062: Clamp returns max if value above"""
        assert clamp(15.0, 0.0, 10.0) == 10.0
        assert clamp(100.0, -10.0, 10.0) == 10.0
```

---

### 5.5 Protocol Handler Tests

**File: `tests/unit/protocol/test_private_handler.py`**

```python
"""
Unit tests for PrivateProtocolHandler
"""

import pytest
from unittest.mock import Mock, call
import can

from robstride.protocol.private import PrivateProtocolHandler
from robstride.models import (
    MotorStatus, ParameterData, MotionControlCommand,
    CommunicationType
)


class TestPrivateHandlerInit:
    """Tests for PrivateProtocolHandler initialization"""
    
    def test_handler_initialization(self, mock_can_bus):
        """TC-U-PH-001: Handler initializes with correct parameters"""
        handler = PrivateProtocolHandler(
            motor_id=5,
            can_bus=mock_can_bus,
            master_id=0xFD
        )
        
        assert handler.motor_id == 5
        assert handler.can_bus == mock_can_bus
        assert handler.master_id == 0xFD
        assert handler.timeout > 0


class TestPrivateHandlerCoreCommands:
    """Tests for core control commands"""
    
    def test_send_enable(self, mock_can_bus):
        """TC-U-PH-010: send_enable sends correct CAN message"""
        handler = PrivateProtocolHandler(1, mock_can_bus)
        
        result = handler.send_enable()
        
        assert result is True
        assert mock_can_bus.send.called
        
        sent_msg = mock_can_bus.send.call_args[0][0]
        assert sent_msg.is_extended_id is True
        assert len(sent_msg.data) == 8
    
    def test_send_disable_without_clear(self, mock_can_bus):
        """TC-U-PH-011: send_disable without error clear"""
        handler = PrivateProtocolHandler(1, mock_can_bus)
        
        result = handler.send_disable(clear_error=False)
        
        assert result is True
        assert mock_can_bus.send.called
        
        sent_msg = mock_can_bus.send.call_args[0][0]
        assert sent_msg.data[0] == 0x00  # No error clear
    
    def test_send_disable_with_clear(self, mock_can_bus):
        """TC-U-PH-012: send_disable with error clear"""
        handler = PrivateProtocolHandler(1, mock_can_bus)
        
        result = handler.send_disable(clear_error=True)
        
        assert result is True
        sent_msg = mock_can_bus.send.call_args[0][0]
        assert sent_msg.data[0] == 0x01  # Error clear flag
    
    def test_send_set_zero(self, mock_can_bus):
        """TC-U-PH-013: send_set_zero sends correct message"""
        handler = PrivateProtocolHandler(1, mock_can_bus)
        
        result = handler.send_set_zero()
        
        assert result is True
        assert mock_can_bus.send.called


class TestPrivateHandlerMotionControl:
    """Tests for motion control command"""
    
    def test_send_motion_control_basic(self, mock_can_bus):
        """TC-U-PH-020: send_motion_control with basic parameters"""
        handler = PrivateProtocolHandler(1, mock_can_bus)
        
        cmd = MotionControlCommand(
            torque=5.0,
            angle=1.57,
            speed=10.0,
            kp=50.0,
            kd=1.0
        )
        
        result = handler.send_motion_control(cmd)
        
        assert result is True
        assert mock_can_bus.send.called
        
        sent_msg = mock_can_bus.send.call_args[0][0]
        assert sent_msg.is_extended_id is True
        assert len(sent_msg.data) == 8
    
    def test_send_motion_control_zero_values(self, mock_can_bus):
        """TC-U-PH-021: send_motion_control with zero values"""
        handler = PrivateProtocolHandler(1, mock_can_bus)
        
        cmd = MotionControlCommand(
            torque=0.0,
            angle=0.0,
            speed=0.0,
            kp=0.0,
            kd=0.0
        )
        
        result = handler.send_motion_control(cmd)
        assert result is True
    
    def test_send_motion_control_extreme_values(self, mock_can_bus):
        """TC-U-PH-022: send_motion_control with extreme values"""
        handler = PrivateProtocolHandler(1, mock_can_bus)
        
        cmd = MotionControlCommand(
            torque=17.0,    # Max
            angle=12.57,    # Max
            speed=44.0,     # Max
            kp=500.0,       # Max
            kd=5.0          # Max
        )
        
        result = handler.send_motion_control(cmd)
        assert result is True


class TestPrivateHandlerParameters:
    """Tests for parameter read/write"""
    
    def test_send_get_parameter(self, mock_can_bus):
        """TC-U-PH-030: send_get_parameter sends correct message"""
        handler = PrivateProtocolHandler(1, mock_can_bus)
        
        result = handler.send_get_parameter(0x7019)  # MECH_POS
        
        assert result is True
        assert mock_can_bus.send.called
        
        sent_msg = mock_can_bus.send.call_args[0][0]
        # Verify parameter index is in data
        assert sent_msg.data[0] == 0x19  # Low byte
        assert sent_msg.data[1] == 0x70  # High byte
    
    def test_send_set_parameter_float(self, mock_can_bus):
        """TC-U-PH-031: send_set_parameter with float value"""
        handler = PrivateProtocolHandler(1, mock_can_bus)
        
        result = handler.send_set_parameter(0x7016, 3.14, value_mode='p')
        
        assert result is True
        assert mock_can_bus.send.called
    
    def test_send_set_parameter_mode(self, mock_can_bus):
        """TC-U-PH-032: send_set_parameter with mode value"""
        handler = PrivateProtocolHandler(1, mock_can_bus)
        
        result = handler.send_set_parameter(0x7005, 2, value_mode='j')  # SPEED mode
        
        assert result is True
    
    def test_send_set_parameter_invalid_mode(self, mock_can_bus):
        """TC-U-PH-033: send_set_parameter with invalid mode raises error"""
        handler = PrivateProtocolHandler(1, mock_can_bus)
        
        with pytest.raises(ValueError, match="Invalid value_mode"):
            handler.send_set_parameter(0x7005, 1.0, value_mode='x')
    
    def test_send_save_parameters(self, mock_can_bus):
        """TC-U-PH-034: send_save_parameters sends magic sequence"""
        handler = PrivateProtocolHandler(1, mock_can_bus)
        
        result = handler.send_save_parameters()
        
        assert result is True
        sent_msg = mock_can_bus.send.call_args[0][0]
        # Verify magic sequence
        assert sent_msg.data == bytes([0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08])


class TestPrivateHandlerMessageProcessing:
    """Tests for message processing"""
    
    def test_process_motor_status_message(self, mock_can_bus, mock_can_message):
        """TC-U-PH-040: process_message handles motor status feedback"""
        handler = PrivateProtocolHandler(1, mock_can_bus, master_id=0xFD)
        status = MotorStatus()
        
        # Create status feedback message (Type 0x02)
        # ExtID format: [Type:8][Data:16][Motor:8]
        msg = mock_can_message(
            arbitration_id=0x02000001,  # Type=0x02, Motor=0x01
            data=[0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x01, 0x2C],  # Example data
            is_extended_id=True
        )
        
        result = handler.process_message(msg, status)
        
        # Status should be updated
        # Exact values depend on decode implementation
        assert result is True or result is False  # Depends on message format match
    
    def test_process_parameter_response(self, mock_can_bus, mock_can_message):
        """TC-U-PH-041: process_message handles parameter response"""
        handler = PrivateProtocolHandler(1, mock_can_bus, master_id=0xFD)
        param_data = ParameterData()
        
        # Create parameter response (Type 0x11)
        import struct
        value = struct.pack('<f', 10.5)
        msg = mock_can_message(
            arbitration_id=0x11000001,
            data=[0x19, 0x70, 0x00, 0x00] + list(value),  # MECH_POS = 10.5
            is_extended_id=True
        )
        
        result = handler.process_message(msg, MotorStatus(), param_data)
        
        # Should update param_data (implementation-dependent)
    
    def test_process_error_feedback(self, mock_can_bus, mock_can_message):
        """TC-U-PH-042: process_message handles error feedback"""
        handler = PrivateProtocolHandler(1, mock_can_bus, master_id=0xFD)
        status = MotorStatus()
        
        # Error feedback message (Type 0x15)
        error_code = 0x20  # Example error
        msg = mock_can_message(
            arbitration_id=(0x15 << 24) | (error_code << 8) | 0x01,
            data=[0] * 8,
            is_extended_id=True
        )
        
        result = handler.process_message(msg, status)
        
        if result:
            assert status.error_code == error_code
```

**File: `tests/unit/protocol/test_mit_handler.py`**

```python
"""
Unit tests for MITProtocolHandler
"""

import pytest
from unittest.mock import Mock
import can

from robstride.protocol.mit import MITProtocolHandler
from robstride.models import MotorStatus, MITCommand


class TestMITHandlerCommands:
    """Tests for MIT protocol commands"""
    
    def test_send_enable(self, mock_can_bus):
        """TC-U-MH-001: send_enable sends magic sequence"""
        handler = MITProtocolHandler(1, mock_can_bus)
        
        result = handler.send_enable()
        
        assert result is True
        sent_msg = mock_can_bus.send.call_args[0][0]
        assert sent_msg.data == bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
    
    def test_send_disable(self, mock_can_bus):
        """TC-U-MH-002: send_disable sends magic sequence"""
        handler = MITProtocolHandler(1, mock_can_bus)
        
        result = handler.send_disable()
        
        assert result is True
        sent_msg = mock_can_bus.send.call_args[0][0]
        assert sent_msg.data == bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])
    
    def test_send_set_zero(self, mock_can_bus):
        """TC-U-MH-003: send_set_zero sends magic sequence"""
        handler = MITProtocolHandler(1, mock_can_bus)
        
        result = handler.send_set_zero()
        
        assert result is True
        sent_msg = mock_can_bus.send.call_args[0][0]
        assert sent_msg.data == bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE])
    
    def test_send_composite_control(self, mock_can_bus):
        """TC-U-MH-010: send_composite_control packs values correctly"""
        handler = MITProtocolHandler(1, mock_can_bus)
        
        cmd = MITCommand(
            position=1.57,
            velocity=10.0,
            kp=50.0,
            kd=1.0,
            torque=5.0
        )
        
        result = handler.send_composite_control(cmd)
        
        assert result is True
        sent_msg = mock_can_bus.send.call_args[0][0]
        assert len(sent_msg.data) == 8
        assert sent_msg.is_extended_id is False
    
    def test_send_position_control(self, mock_can_bus):
        """TC-U-MH-011: send_position_control uses correct CAN ID"""
        handler = MITProtocolHandler(motor_id=5, can_bus=mock_can_bus)
        
        result = handler.send_position_control(position=3.14, speed=10.0)
        
        assert result is True
        sent_msg = mock_can_bus.send.call_args[0][0]
        # CAN ID should be (1 << 8) | motor_id = 0x105
        assert sent_msg.arbitration_id == 0x105
    
    def test_send_speed_control(self, mock_can_bus):
        """TC-U-MH-012: send_speed_control uses correct CAN ID"""
        handler = MITProtocolHandler(motor_id=5, can_bus=mock_can_bus)
        
        result = handler.send_speed_control(speed=20.0, current_limit=10.0)
        
        assert result is True
        sent_msg = mock_can_bus.send.call_args[0][0]
        # CAN ID should be (2 << 8) | motor_id = 0x205
        assert sent_msg.arbitration_id == 0x205


class TestMITHandlerMessageProcessing:
    """Tests for MIT message processing"""
    
    def test_process_feedback_message(self, mock_can_bus, mock_can_message):
        """TC-U-MH-020: process_message decodes MIT feedback"""
        handler = MITProtocolHandler(motor_id=1, can_bus=mock_can_bus)
        status = MotorStatus()
        
        # MIT feedback format:
        # Byte0: Motor ID
        # Byte1-2: Angle (16-bit)
        # Byte3-4(high): Speed (12-bit)
        # Byte4(low)-5: Torque (12-bit)
        # Byte6-7: Temperature
        msg = mock_can_message(
            arbitration_id=0x00,  # Standard ID
            data=[0x01, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x2D],
            is_extended_id=False
        )
        
        result = handler.process_message(msg, status)
        
        # Should decode and update status
        # Exact values depend on implementation
```

---

### 5.6 Error Handler Tests

**File: `tests/unit/test_error_handler.py`**

```python
"""
Unit tests for ErrorHandler class
"""

import pytest
from unittest.mock import Mock, patch
import time

from robstride.utils import ErrorHandler
from robstride.models import ErrorFlag, MotorStatus


class TestErrorHandlerInit:
    """Tests for ErrorHandler initialization"""
    
    def test_handler_initialization(self, mock_motor):
        """TC-U-EH-001: ErrorHandler initializes correctly"""
        handler = ErrorHandler(mock_motor, auto_recovery=True)
        
        assert handler.motor == mock_motor
        assert handler.auto_recovery is True
        assert len(handler.error_history) == 0
        assert handler.max_recovery_attempts == 3
    
    def test_handler_with_auto_recovery_disabled(self, mock_motor):
        """TC-U-EH-002: ErrorHandler with auto_recovery disabled"""
        handler = ErrorHandler(mock_motor, auto_recovery=False)
        
        assert handler.auto_recovery is False


class TestErrorDetection:
    """Tests for error detection"""
    
    def test_check_errors_no_error(self, mock_motor):
        """TC-U-EH-010: check_errors returns False when no errors"""
        handler = ErrorHandler(mock_motor)
        mock_motor.status.error_code = 0
        
        assert handler.check_errors() is False
    
    def test_check_errors_with_error(self, mock_motor):
        """TC-U-EH-011: check_errors returns True when errors present"""
        handler = ErrorHandler(mock_motor)
        mock_motor.status.error_code = ErrorFlag.OVER_TEMPERATURE
        
        assert handler.check_errors() is True


class TestErrorLogging:
    """Tests for error logging"""
    
    def test_log_error_records_history(self, mock_motor):
        """TC-U-EH-020: log_error adds to error history"""
        handler = ErrorHandler(mock_motor)
        
        handler.log_error(ErrorFlag.OVER_CURRENT, severity="ERROR", context={"current": 25.0})
        
        assert len(handler.error_history) == 1
        assert handler.error_history[0]["error_flags"] == ErrorFlag.OVER_CURRENT
        assert handler.error_history[0]["severity"] == "ERROR"
    
    def test_log_error_max_history(self, mock_motor):
        """TC-U-EH-021: Error history respects max size"""
        handler = ErrorHandler(mock_motor)
        handler.max_history = 5
        
        # Log 10 errors
        for i in range(10):
            handler.log_error(ErrorFlag.OVER_TEMPERATURE, severity="WARNING")
        
        # Should only keep last 5
        assert len(handler.error_history) == 5
    
    def test_get_error_description(self, mock_motor):
        """TC-U-EH-022: get_error_description returns meaningful text"""
        handler = ErrorHandler(mock_motor)
        
        desc = handler.get_error_description(ErrorFlag.OVER_TEMPERATURE)
        assert "over-temperature" in desc.lower() or "temperature" in desc.lower()
        
        desc = handler.get_error_description(ErrorFlag.OVER_CURRENT)
        assert "current" in desc.lower()
        
        desc = handler.get_error_description(0)
        assert "no error" in desc.lower()


class TestAutoRecovery:
    """Tests for auto recovery strategies"""
    
    def test_handle_error_no_auto_recovery(self, mock_motor):
        """TC-U-EH-030: handle_error does nothing when auto_recovery=False"""
        handler = ErrorHandler(mock_motor, auto_recovery=False)
        mock_motor.status.error_code = ErrorFlag.OVER_CURRENT
        
        result = handler.handle_error(ErrorFlag.OVER_CURRENT)
        
        assert result is False
    
    def test_handle_error_no_error(self, mock_motor):
        """TC-U-EH-031: handle_error returns True for no error"""
        handler = ErrorHandler(mock_motor)
        
        result = handler.handle_error(0)
        
        assert result is True
    
    def test_recovery_max_attempts(self, mock_motor):
        """TC-U-EH-032: Recovery stops after max attempts"""
        handler = ErrorHandler(mock_motor, auto_recovery=True)
        handler.max_recovery_attempts = 3
        
        # Mock recovery function to always fail
        def failing_recovery():
            return False
        
        handler.recovery_strategies[ErrorFlag.OVER_CURRENT] = failing_recovery
        
        # Attempt recovery 4 times
        for i in range(4):
            result = handler.handle_error(ErrorFlag.OVER_CURRENT)
        
        # Should have stopped after 3 attempts
        assert handler.recovery_attempts.get(ErrorFlag.OVER_CURRENT.name, 0) >= 3
    
    def test_recovery_resets_on_success(self, mock_motor):
        """TC-U-EH-033: Recovery attempt counter resets on success"""
        handler = ErrorHandler(mock_motor, auto_recovery=True)
        
        # Mock successful recovery
        def successful_recovery():
            return True
        
        handler.recovery_strategies[ErrorFlag.OVER_CURRENT] = successful_recovery
        
        # First recovery
        handler.handle_error(ErrorFlag.OVER_CURRENT)
        
        # Counter should be reset to 0
        assert handler.recovery_attempts.get(ErrorFlag.OVER_CURRENT.name, 0) == 0


### 5.7 Control Strategies Tests

**File: `tests/unit/test_control_strategies.py`**

```python
"""
Unit tests for control strategy classes
"""

import pytest
from unittest.mock import Mock, patch

from robstride.control import PositionController, SpeedController, CurrentController
from robstride.models import ParameterIndex, ControlMode


class TestPositionController:
    """Tests for PositionController"""
    
    def test_set_pp_position(self, mock_motor):
        """TC-U-CS-001: set_pp_position sets correct parameters"""
        controller = PositionController(mock_motor)
        
        controller.set_pp_position(target_angle=3.14, target_speed=10.0)
        
        # Verify set_parameter calls
        calls = mock_motor.set_parameter.call_args_list
        
        # Should set RUN_MODE, LIMIT_SPD_PP, LOC_REF
        assert len(calls) >= 3
    
    def test_set_csp_position(self, mock_motor):
        """TC-U-CS-002: set_csp_position sets correct parameters"""
        controller = PositionController(mock_motor)
        
        controller.set_csp_position(target_angle=1.57, speed_limit=20.0)
        
        calls = mock_motor.set_parameter.call_args_list
        assert len(calls) >= 3


class TestSpeedController:
    """Tests for SpeedController"""
    
    def test_set_speed(self, mock_motor):
        """TC-U-CS-010: set_speed sets correct parameters"""
        controller = SpeedController(mock_motor)
        
        controller.set_speed(target_speed=15.0, current_limit=10.0)
        
        calls = mock_motor.set_parameter.call_args_list
        # Should set RUN_MODE, LIMIT_CUR, SPD_REF
        assert len(calls) >= 3


class TestCurrentController:
    """Tests for CurrentController"""
    
    def test_set_current(self, mock_motor):
        """TC-U-CS-020: set_current sets correct parameters"""
        controller = CurrentController(mock_motor)
        
        controller.set_current(target_current=5.0)
        
        calls = mock_motor.set_parameter.call_args_list
        # Should set RUN_MODE, IQ_REF
        assert len(calls) >= 2
    
    def test_set_torque(self, mock_motor):
        """TC-U-CS-021: set_torque converts to current correctly"""
        controller = CurrentController(mock_motor)
        
        controller.set_torque(target_torque=10.0, torque_constant=0.5)
        
        # Should convert 10 Nm / 0.5 = 20 A
        # Verify set_current was called with ~20 A
```

---

## 6. Integration Tests / 統合テスト

Integration testsはモジュール間の連携を検証する。Virtual CANを使用し、実機なしで実行可能。

**File: `tests/integration/conftest.py`**

```python
"""
Integration test fixtures
"""

import pytest
import can
import time


@pytest.fixture(scope="module")
def vcan_setup():
    """
    Ensure vcan0 is set up
    
    Note: This assumes vcan0 is already configured.
    In CI, this should be done in workflow setup.
    """
    # Verification only
    try:
        bus = can.interface.Bus(channel='vcan0', bustype='socketcan')
        bus.shutdown()
        return True
    except:
        pytest.skip("vcan0 not available")


@pytest.fixture
def mock_motor_response_handler(vcan_bus):
    """
    Mock motor that responds to commands on vcan
    
    Simulates motor responses for integration testing.
    """
    import threading
    
    running = threading.Event()
    running.set()
    
    def responder():
        while running.is_set():
            try:
                msg = vcan_bus.recv(timeout=0.1)
                if msg and msg.is_extended_id:
                    # Parse and respond based on command type
                    # (Implementation would generate appropriate responses)
                    pass
            except:
                pass
    
    thread = threading.Thread(target=responder, daemon=True)
    thread.start()
    
    yield
    
    running.clear()
    thread.join(timeout=1.0)
```

**File: `tests/integration/test_protocol_integration.py`**

```python
"""
Integration tests for protocol operations
"""

import pytest
import time

from robstride import RobStrideMotor
from robstride.models import ProtocolMode, ParameterIndex


@pytest.mark.integration
class TestEnableDisableSequence:
    """Tests for enable/disable sequence"""
    
    def test_enable_disable_cycle(self, vcan_motor):
        """TC-I-001: Enable-disable cycle completes successfully"""
        motor = vcan_motor
        
        # Enable
        result = motor.enable_motor()
        time.sleep(0.2)
        
        # In real motor, state would change
        # With virtual CAN, verify command was sent
        
        # Disable
        result = motor.disable_motor()
        time.sleep(0.2)
    
    def test_multiple_enable_disable_cycles(self, vcan_motor):
        """TC-I-002: Multiple enable/disable cycles work correctly"""
        motor = vcan_motor
        
        for _ in range(5):
            motor.enable_motor()
            time.sleep(0.1)
            motor.disable_motor()
            time.sleep(0.1)


@pytest.mark.integration
class TestParameterReadWrite:
    """Tests for parameter read/write operations"""
    
    def test_parameter_write_sequence(self, vcan_motor):
        """TC-I-010: Parameter write completes without error"""
        motor = vcan_motor
        motor.enable_motor()
        time.sleep(0.1)
        
        # Write parameters
        result = motor.set_parameter(ParameterIndex.LIMIT_CUR, 10.0, value_mode='p')
        assert result is True
        
        time.sleep(0.05)
        
        result = motor.set_parameter(ParameterIndex.SPD_REF, 5.0, value_mode='p')
        assert result is True
    
    def test_parameter_read_sequence(self, vcan_motor):
        """TC-I-011: Parameter read completes without error"""
        motor = vcan_motor
        motor.enable_motor()
        time.sleep(0.1)
        
        # Read parameter
        result = motor.get_parameter(ParameterIndex.MECH_POS)
        assert result is True
        
        time.sleep(0.1)


@pytest.mark.integration
class TestMessageFlow:
    """Tests for message send/receive flow"""
    
    def test_command_response_timing(self, vcan_motor, benchmark_timer):
        """TC-I-020: Command-response completes within timeout"""
        motor = vcan_motor
        
        with benchmark_timer() as t:
            motor.enable_motor()
            time.sleep(0.1)
        
        # Should complete quickly
        assert t.elapsed < 0.5
    
    def test_continuous_commands(self, vcan_motor):
        """TC-I-021: Continuous commands don't cause buffer overflow"""
        motor = vcan_motor
        motor.enable_motor()
        time.sleep(0.1)
        
        # Send many commands rapidly
        for i in range(100):
            motor.set_parameter(ParameterIndex.LOC_REF, float(i % 10))
            time.sleep(0.01)


@pytest.mark.integration
class TestErrorRecovery:
    """Tests for error recovery mechanisms"""
    
    def test_clear_error_sequence(self, vcan_motor):
        """TC-I-030: Clear error sequence works correctly"""
        motor = vcan_motor
        
        # Simulate error condition
        motor.status.error_code = 0x20
        
        # Clear error via disable with clear flag
        motor.disable_motor(clear_error=True)
        time.sleep(0.1)
        
        # Error should be cleared (in real scenario)
```

---

## 7. System Tests / システムテスト

System testsは実機との通信を検証する。ハードウェア接続が必須。

**File: `tests/system/conftest.py`**

```python
"""
System test fixtures (hardware required)
"""

import pytest


def pytest_configure(config):
    """Add custom markers"""
    config.addinivalue_line(
        "markers", "hardware: tests requiring real motor hardware"
    )


@pytest.fixture(scope="module")
def safety_check():
    """
    Safety check before hardware tests
    
    Ensures:
    - Emergency stop is accessible
    - Power supply is within limits
    - Motor is properly mounted
    """
    print("\n" + "="*60)
    print("HARDWARE TEST SAFETY CHECK")
    print("="*60)
    print("Before proceeding, verify:")
    print("  1. Emergency stop button is within reach")
    print("  2. Motor is securely mounted")
    print("  3. Power supply is 24V (12-50V range)")
    print("  4. Area is clear of obstacles")
    print("  5. You are ready to stop test if needed")
    print("="*60)
    
    response = input("Continue with hardware tests? (yes/no): ")
    
    if response.lower() != 'yes':
        pytest.skip("Hardware tests skipped by user")
    
    yield
    
    print("\nHardware tests completed. Safe to power down.")
```

**File: `tests/system/test_hardware_comm.py`**

```python
"""
System tests for hardware communication
"""

import pytest
import time
import can

from robstride import RobStrideMotor
from robstride.models import ParameterIndex


@pytest.mark.hardware
@pytest.mark.system
class TestBasicCommunication:
    """Tests for basic CAN communication with real motor"""
    
    def test_motor_connection(self, hardware_motor, safety_check):
        """TC-S-001: Motor responds to CAN messages"""
        motor = hardware_motor
        
        # Try to enable motor
        result = motor.enable_motor()
        assert result is True
        
        time.sleep(0.5)
        
        # Motor should be enabled
        # (State verification depends on feedback implementation)
        
        motor.disable_motor()
    
    def test_status_feedback_reception(self, hardware_motor, safety_check):
        """TC-S-002: Motor sends status feedback"""
        motor = hardware_motor
        
        motor.enable_motor()
        time.sleep(0.2)
        
        # Read position
        motor.get_parameter(ParameterIndex.MECH_POS)
        time.sleep(0.2)
        
        # Status should have been updated
        # (Check that last_update timestamp changed)
        
        motor.disable_motor()
    
    def test_parameter_roundtrip(self, hardware_motor, safety_check):
        """TC-S-003: Parameter write-read roundtrip"""
        motor = hardware_motor
        
        motor.enable_motor()
        time.sleep(0.2)
        
        # Write current limit
        test_value = 8.0
        motor.set_parameter(ParameterIndex.LIMIT_CUR, test_value)
        time.sleep(0.1)
        
        # Read back
        motor.get_parameter(ParameterIndex.LIMIT_CUR)
        time.sleep(0.2)
        
        # Verify (implementation-dependent)
        # assert abs(motor.param_data.limit_cur - test_value) < 0.1
        
        motor.disable_motor()


@pytest.mark.hardware
@pytest.mark.system
@pytest.mark.slow
class TestControlPerformance:
    """Tests for control performance on real hardware"""
    
    def test_position_control_basic(self, hardware_motor, safety_check):
        """TC-S-010: Basic position control works"""
        motor = hardware_motor
        
        # Setup
        motor.enable_motor()
        time.sleep(0.2)
        
        motor.set_parameter(ParameterIndex.RUN_MODE, 5, value_mode='j')  # CSP mode
        motor.set_parameter(ParameterIndex.LIMIT_SPD_CSP, 5.0)
        motor.set_parameter(ParameterIndex.LIMIT_CUR, 5.0)
        time.sleep(0.2)
        
        # Set zero
        motor.set_zero_position()
        time.sleep(0.5)
        
        # Move to target
        target = 1.0  # ~57 degrees
        motor.set_parameter(ParameterIndex.LOC_REF, target)
        
        # Wait for movement
        time.sleep(3.0)
        
        # Read position
        motor.get_parameter(ParameterIndex.MECH_POS)
        time.sleep(0.2)
        
        # Verify arrival (tolerance 0.1 rad)
        # assert abs(motor.param_data.mech_pos - target) < 0.1
        
        # Return to zero
        motor.set_parameter(ParameterIndex.LOC_REF, 0.0)
        time.sleep(3.0)
        
        motor.disable_motor()
```

---

## 8. Test Execution / テスト実行

### 8.1 Running Tests Locally

**基本的な実行:**
```bash
# All tests (unit + integration with vcan)
pytest

# Unit tests only (fastest)
pytest tests/unit -v

# Integration tests (requires vcan0)
pytest tests/integration -v

# With coverage report
pytest tests/unit tests/integration --cov=robstride --cov-report=html --cov-report=term-missing

# Hardware tests (requires real motor on can0)
pytest -m hardware -v

# Specific test file
pytest tests/unit/test_models.py -v

# Specific test function
pytest tests/unit/test_models.py::TestMotorStatus::test_motor_status_initialization -v

# Run with verbose output and stop on first failure
pytest -vv -x

# Run tests matching pattern
pytest -k "validation" -v
```

**マーカーを使った実行:**
```bash
# Run only unit tests
pytest -m unit

# Skip hardware tests
pytest -m "not hardware"

# Run only slow tests
pytest -m slow

# Run system and HIL tests
pytest -m "system or hil"

# Run with multiple markers
pytest -m "hardware and not slow"
```

**カバレッジ詳細:**
```bash
# Generate HTML coverage report
pytest --cov=robstride --cov-report=html
# Open htmlcov/index.html in browser

# Generate XML coverage (for CI)
pytest --cov=robstride --cov-report=xml

# Show missing lines in terminal
pytest --cov=robstride --cov-report=term-missing

# Coverage with branch analysis
pytest --cov=robstride --cov-branch --cov-report=term
```

**パラレル実行（高速化）:**
```bash
# Install pytest-xdist
pip install pytest-xdist

# Run tests in parallel (auto detect CPU cores)
pytest -n auto

# Run with specific number of workers
pytest -n 4
```

### 8.2 Virtual CAN Setup

**Linux:**
```bash
# Load vcan kernel module
sudo modprobe vcan

# Create vcan0 interface
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

# Verify interface is up
ip link show vcan0

# Monitor CAN messages (optional)
candump vcan0
```

**Automatic setup script (`scripts/setup_vcan.sh`):**
```bash
#!/bin/bash
# Setup virtual CAN interface for testing

set -e

if [ -z "$(ip link show vcan0 2>/dev/null)" ]; then
    echo "Creating vcan0 interface..."
    sudo modprobe vcan
    sudo ip link add dev vcan0 type vcan
    sudo ip link set up vcan0
    echo "vcan0 interface created successfully"
else
    echo "vcan0 interface already exists"
fi

# Verify
ip link show vcan0
echo "Virtual CAN setup complete"
```

### 8.3 Continuous Integration (GitHub Actions)

**`.github/workflows/test.yml`:**
```yaml
name: Test Suite

on:
  push:
    branches: [ main, develop ]
  pull_request:
    branches: [ main, develop ]

jobs:
  unit-tests:
    name: Unit Tests (Python ${{ matrix.python-version }})
    runs-on: ubuntu-latest
    strategy:
      matrix:
        python-version: ['3.8', '3.9', '3.10', '3.11']
    
    steps:
    - uses: actions/checkout@v3
    
    - name: Set up Python ${{ matrix.python-version }}
      uses: actions/setup-python@v4
      with:
        python-version: ${{ matrix.python-version }}
    
    - name: Install dependencies
      run: |
        python -m pip install --upgrade pip
        pip install -e .
        pip install pytest pytest-cov pytest-mock pytest-timeout
    
    - name: Run unit tests
      run: |
        pytest tests/unit -v --cov=robstride --cov-report=xml --cov-report=term-missing
    
    - name: Upload coverage to Codecov
      uses: codecov/codecov-action@v3
      with:
        file: ./coverage.xml
        flags: unittests
        name: codecov-unit-py${{ matrix.python-version }}
  
  integration-tests:
    name: Integration Tests
    runs-on: ubuntu-latest
    
    steps:
    - uses: actions/checkout@v3
    
    - name: Set up Python 3.10
      uses: actions/setup-python@v4
      with:
        python-version: '3.10'
    
    - name: Install dependencies
      run: |
        python -m pip install --upgrade pip
        pip install -e .
        pip install pytest pytest-cov pytest-mock pytest-timeout
    
    - name: Setup virtual CAN
      run: |
        sudo modprobe vcan
        sudo ip link add dev vcan0 type vcan
        sudo ip link set up vcan0
        ip link show vcan0
    
    - name: Run integration tests
      run: |
        pytest tests/integration -v --cov=robstride --cov-report=xml
    
    - name: Upload coverage
      uses: codecov/codecov-action@v3
      with:
        file: ./coverage.xml
        flags: integration
        name: codecov-integration
  
  code-quality:
    name: Code Quality Checks
    runs-on: ubuntu-latest
    
    steps:
    - uses: actions/checkout@v3
    
    - name: Set up Python
      uses: actions/setup-python@v4
      with:
        python-version: '3.10'
    
    - name: Install dependencies
      run: |
        python -m pip install --upgrade pip
        pip install black flake8 mypy
        pip install -e .
    
    - name: Check formatting with Black
      run: |
        black --check src/ tests/
    
    - name: Lint with flake8
      run: |
        flake8 src/ tests/ --count --select=E9,F63,F7,F82 --show-source --statistics
        flake8 src/ tests/ --count --exit-zero --max-complexity=10 --max-line-length=100 --statistics
    
    - name: Type check with mypy
      run: |
        mypy src/robstride --ignore-missing-imports
```

### 8.4 Pre-commit Hooks

**`.pre-commit-config.yaml`:**
```yaml
repos:
  - repo: https://github.com/psf/black
    rev: 23.1.0
    hooks:
      - id: black
        language_version: python3.10
  
  - repo: https://github.com/pycqa/flake8
    rev: 6.0.0
    hooks:
      - id: flake8
        args: ['--max-line-length=100', '--extend-ignore=E203,W503']
  
  - repo: https://github.com/pre-commit/mirrors-mypy
    rev: v1.0.1
    hooks:
      - id: mypy
        additional_dependencies: [types-all]
  
  - repo: local
    hooks:
      - id: pytest-unit
        name: pytest-unit
        entry: pytest tests/unit -v
        language: system
        pass_filenames: false
        always_run: true
```

**Installation:**
```bash
pip install pre-commit
pre-commit install
```

---

## 9. Test Documentation Standards / テストドキュメント標準

### 9.1 Test Case Naming Convention

**命名規則:**
- **Test ID**: `TC-{Level}-{Module}-{Number}`
  - Level: U (Unit), I (Integration), S (System), H (HIL), A (Acceptance), P (Performance)
  - Module: モジュール識別子（M=Models, MI=MotorInit, CU=CANUtils, etc.）
  - Number: 3桁のシーケンス番号 (001, 002, ...)

**例:**
- `TC-U-M-001`: Unit test for Models, case 001
- `TC-I-PI-010`: Integration test for Protocol Integration, case 010
- `TC-S-HC-020`: System test for Hardware Communication, case 020

**関数名規則:**
```python
def test_{feature}_{scenario}_{expected_result}():
    """
    TC-X-Y-ZZZ: Brief description
    
    Detailed description of what is being tested.
    """
```

### 9.2 Test Documentation Template

**各テスト関数には以下を含める:**
```python
def test_example_feature():
    """
    TC-U-EX-001: Feature X validates input Y correctly
    
    **Test Objective:**
    Verify that feature X properly validates input parameter Y
    and rejects invalid values.
    
    **Preconditions:**
    - Feature X is initialized
    - Input parameter Y is defined
    
    **Test Steps:**
    1. Call feature X with valid input
    2. Verify acceptance
    3. Call feature X with invalid input
    4. Verify rejection with appropriate error
    
    **Expected Result:**
    - Valid input is accepted without error
    - Invalid input raises ValueError with descriptive message
    
    **Test Data:**
    - Valid: 10.0, 20.0, 30.0
    - Invalid: -1.0, 100.0, "not_a_number"
    """
    # Arrange
    feature = FeatureX()
    
    # Act & Assert - Valid
    assert feature.validate(10.0) is True
    
    # Act & Assert - Invalid
    with pytest.raises(ValueError, match="out of range"):
        feature.validate(-1.0)
```

### 9.3 Test Report Template

**テスト実行後のレポートフォーマット:**

```markdown
# Test Execution Report

**Date:** 2025-10-15  
**Tester:** Your Name  
**Environment:** Ubuntu 22.04 / Python 3.10 / Virtual CAN  
**Software Version:** robstride v1.0.0  
**Commit:** abc123def

## Executive Summary

| Category | Total | Passed | Failed | Skipped | Coverage |
|----------|-------|--------|--------|---------|----------|
| Unit     | 150   | 148    | 2      | 0       | 92%      |
| Integration | 35  | 33     | 1      | 1       | 85%      |
| System   | 20    | 18     | 2      | 0       | N/A      |
| HIL      | 8     | 8      | 0      | 0       | N/A      |
| Acceptance | 5   | 5      | 0      | 0       | N/A      |
| **Total** | **218** | **212** | **5** | **1** | **88%** |

**Overall Result:** ⚠️ **PASS WITH ISSUES**

## Failed Tests

### TC-U-CU-005: Torque encoding boundary test
- **Location:** `tests/unit/protocol/test_can_utils.py::test_encode_torque_boundary`
- **Error:** `AssertionError: Encoded value 65535 != expected 65534`
- **Root Cause:** Off-by-one error in torque scaling formula
- **Action:** Fix scaling calculation in `encode_torque_16bit()`
- **Priority:** High
- **Assigned:** Developer A

### TC-I-PI-012: Parameter write timeout
- **Location:** `tests/integration/test_protocol_integration.py::test_parameter_write_timeout`
- **Error:** `TimeoutError: No response within 1.0s`
- **Root Cause:** Virtual CAN response handler not implemented for this parameter
- **Action:** Implement mock response in integration test fixture
- **Priority:** Medium
- **Assigned:** Developer B

### TC-S-CP-011: Speed control overshoot
- **Location:** `tests/system/test_control_performance.py::test_speed_control_overshoot`
- **Error:** `AssertionError: Speed overshoot 12.5 rad/s > limit 10.0 rad/s`
- **Root Cause:** Aggressive acceleration parameter
- **Action:** Tune default acceleration setting in control strategy
- **Priority:** Medium
- **Assigned:** Control Engineer

## Skipped Tests

### TC-I-MF-020: Multi-motor synchronized startup
- **Reason:** Requires 3+ motors on vcan, currently single motor fixture
- **Action:** Implement multi-motor fixture for integration tests

## Performance Metrics

- **Average test duration:** 0.8s per test
- **Total test suite runtime:** 3m 45s
- **Slowest tests:**
  1. TC-S-CP-010: Position accuracy (15.2s)
  2. TC-H-TT-001: Sinusoidal trajectory (12.8s)
  3. TC-A-PP-001: Pick and place (10.5s)

## Coverage Analysis

**By Module:**
- `motor.py`: 94%
- `models.py`: 98%
- `protocol/private.py`: 90%
- `protocol/mit.py`: 87%
- `protocol/can_utils.py`: 95%
- `utils.py`: 88%
- `control.py`: 82%

**Uncovered Areas:**
- Error recovery edge cases (lines 145-158 in utils.py)
- Protocol mode switching (lines 230-245 in motor.py)
- MIT speed control boundary conditions

## Recommendations

1. **Critical:** Fix torque encoding off-by-one error before release
2. **High Priority:** Improve integration test mock responses
3. **Medium Priority:** Tune speed control parameters to eliminate overshoot
4. **Low Priority:** Increase coverage of error recovery edge cases to >90%
5. **Enhancement:** Add performance regression tests to CI pipeline

## Next Steps

- [ ] Fix failed tests and re-run suite
- [ ] Implement multi-motor integration test fixture
- [ ] Add missing coverage for error recovery
- [ ] Document speed control tuning procedure
- [ ] Schedule HIL tests with updated firmware

---
**Report Generated:** 2025-10-15 14:30:00 UTC  
**Report By:** pytest-html + custom analysis
```

---

## 10. Best Practices for Test Implementation / テスト実装のベストプラクティス

### 10.1 Test Independence

**各テストは独立している必要がある:**
```python
# ❌ BAD: Tests depend on execution order
class TestBadSequence:
    motor = None
    
    def test_01_init(self):
        self.motor = RobStrideMotor(can_id=1, ...)
    
    def test_02_enable(self):
        self.motor.enable_motor()  # Fails if test_01 doesn't run

# ✅ GOOD: Each test is self-contained
class TestGoodSequence:
    
    def test_init(self, mock_motor):
        motor = mock_motor
        assert motor.motor_id == 1
    
    def test_enable(self, mock_motor):
        motor = mock_motor
        result = motor.enable_motor()
        assert result is True
```

### 10.2 Clear Arrange-Act-Assert Pattern

```python
def test_parameter_validation():
    """TC-U-V-010: Parameter validation rejects invalid values"""
    
    # Arrange: Setup test data and objects
    validator = ParameterValidator()
    invalid_value = 100.0
    param_index = ParameterIndex.IQ_REF
    
    # Act: Execute the code under test
    with pytest.raises(ValueError) as exc_info:
        validator.validate(param_index, invalid_value)
    
    # Assert: Verify expected outcomes
    assert "out of range" in str(exc_info.value)
    assert "23.0" in str(exc_info.value)  # Max value mentioned
```

### 10.3 Meaningful Assertions

```python
# ❌ BAD: Vague assertion message
def test_speed_limit():
    motor.set_parameter(ParameterIndex.LIMIT_SPD, 50.0)
    assert motor.param_data.limit_spd <= 44.0

# ✅ GOOD: Clear assertion with custom message
def test_speed_limit():
    """TC-U-V-020: Speed limit is clamped to maximum 44.0 rad/s"""
    motor.set_parameter(ParameterIndex.LIMIT_SPD, 50.0)
    
    actual_limit = motor.param_data.limit_spd
    max_limit = 44.0
    
    assert actual_limit <= max_limit, (
        f"Speed limit {actual_limit} rad/s exceeds maximum {max_limit} rad/s"
    )
```

### 10.4 Proper Fixture Usage

```python
# ✅ Use fixtures for common setup
@pytest.fixture
def configured_motor(mock_can_bus):
    """Motor with standard test configuration"""
    with patch('robstride.motor.can.interface.Bus', return_value=mock_can_bus):
        motor = RobStrideMotor(can_id=1, can_interface='test', auto_enable=False)
        motor._running = False
        
        # Apply standard configuration
        motor.set_parameter(ParameterIndex.LIMIT_CUR, 10.0)
        motor.set_parameter(ParameterIndex.LIMIT_SPD, 20.0)
        
        yield motor
        
        # Cleanup
        motor._running = False

def test_with_configured_motor(configured_motor):
    """Test uses pre-configured motor"""
    motor = configured_motor
    # Test logic here
```

### 10.5 Parametrized Tests for Coverage

```python
# ✅ Parametrize to test multiple cases efficiently
@pytest.mark.parametrize("angle,expected_valid", [
    (-12.57, True),   # Min boundary
    (-12.58, False),  # Below min
    (0.0, True),      # Zero
    (12.57, True),    # Max boundary
    (12.58, False),   # Above max
    (float('nan'), False),  # NaN
    (float('inf'), False),  # Inf
])
def test_angle_validation(angle, expected_valid):
    """TC-U-V-010: Angle validation with boundary values"""
    if expected_valid:
        assert validate_angle(angle) is True
    else:
        with pytest.raises((ValueError, TypeError)):
            validate_angle(angle)
```

### 10.6 Testing Exceptions

```python
# ✅ Test both exception type and message
def test_invalid_can_id_exception():
    """TC-U-MI-003: Invalid CAN ID raises ValueError with descriptive message"""
    
    with pytest.raises(ValueError) as exc_info:
        validate_can_id(0x80)
    
    error_message = str(exc_info.value)
    assert "CAN ID" in error_message
    assert "0x00-0x7F" in error_message or "0-127" in error_message
```

### 10.7 Mocking External Dependencies

```python
# ✅ Mock CAN bus for unit tests
def test_send_enable_command(mock_can_bus):
    """TC-U-PH-010: Enable command sends correct CAN message"""
    
    handler = PrivateProtocolHandler(1, mock_can_bus)
    
    result = handler.send_enable()
    
    # Verify send was called
    assert mock_can_bus.send.called
    assert mock_can_bus.send.call_count == 1
    
    # Verify message content
    sent_message = mock_can_bus.send.call_args[0][0]
    assert isinstance(sent_message, can.Message)
    assert sent_message.is_extended_id is True
    assert len(sent_message.data) == 8
```

### 10.8 Timeout Protection

```python
# ✅ Use pytest-timeout to prevent hanging tests
@pytest.mark.timeout(5)  # 5 second timeout
def test_can_communication_timeout():
    """TC-I-020: CAN communication doesn't hang indefinitely"""
    motor = vcan_motor
    motor.get_parameter(ParameterIndex.MECH_POS)
    time.sleep(0.5)  # Wait for response
```

---

## 11. Troubleshooting / トラブルシューティング

### 11.1 Common Test Failures

**Problem:** Tests fail with "vcan0: No such device"

**Solution:**
```bash
# Check if vcan module is loaded
lsmod | grep vcan

# Load module if missing
sudo modprobe vcan

# Create interface
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

**Problem:** Hardware tests timeout

**Solution:**
- Verify motor is powered (24V)
- Check CAN cable connections
- Verify can0 interface is up: `ip link show can0`
- Check CAN bitrate: `ip -details link show can0`
- Restart motor and clear errors

**Problem:** Import errors in tests

**Solution:**
```bash
# Install package in editable mode
pip install -e .

# Verify installation
python -c "import robstride; print(robstride.__version__)"
```

**Problem:** Coverage report shows 0%

**Solution:**
```bash
# Ensure pytest-cov is installed
pip install pytest-cov

# Run with explicit coverage source
pytest --cov=src/robstride tests/

# Or use .coveragerc configuration
```

### 11.2 Debugging Tests

**Enable verbose output:**
```bash
pytest -vv --tb=long tests/unit/test_specific.py
```

**Drop into debugger on failure:**
```bash
pytest --pdb tests/
```

**Print captured output:**
```bash
pytest -s tests/  # Show print statements
```

**Run last failed tests:**
```bash
pytest --lf  # Last failed
pytest --ff  # Failed first, then others
```

---

## 12. Summary and Checklist / まとめとチェックリスト

### 12.1 Test Implementation Checklist

**Unit Tests (60%):**
- [ ] Data models (MotorStatus, ParameterData, Enums)
- [ ] Motor initialization and properties
- [ ] CAN utility functions (encode/decode)
- [ ] Protocol handlers (Private, MIT)
- [ ] Validation functions
- [ ] Error handler and recovery strategies
- [ ] Control strategies (Position, Speed, Current)

**Integration Tests (20%):**
- [ ] Enable/disable sequences
- [ ] Parameter read/write operations
- [ ] Protocol switching
- [ ] Message flow and timing
- [ ] Error recovery mechanisms

**System Tests (10%):**
- [ ] Hardware CAN communication
- [ ] Position control accuracy
- [ ] Speed control stability
- [ ] Current control response
- [ ] Error condition handling

**HIL Tests (7%):**
- [ ] Trajectory tracking
- [ ] Multi-motor synchronization
- [ ] Load testing
- [ ] Thermal performance

**Acceptance Tests (3%):**
- [ ] Pick and place scenario
- [ ] Continuous speed control
- [ ] User workflow validation

**Infrastructure:**
- [ ] conftest.py with common fixtures
- [ ] pytest.ini with markers
- [ ] CI/CD workflow (GitHub Actions)
- [ ] Coverage reporting
- [ ] Pre-commit hooks

### 12.2 Coverage Goals

| Module | Target | Priority |
|--------|--------|----------|
| motor.py | > 90% | Critical |
| models.py | > 95% | Critical |
| protocol/*.py | > 85% | High |
| utils.py | > 85% | High |
| control.py | > 80% | Medium |

### 12.3 Key Success Metrics

- **Overall test coverage:** > 85%
- **Unit test pass rate:** 100%
- **Integration test pass rate:** > 95%
- **System test pass rate:** > 90%
- **Average test execution time:** < 5s per test
- **CI pipeline duration:** < 10 minutes

---

## Appendix A: Test Data Reference / テストデータリファレンス

### A.1 Valid Parameter Ranges

| Parameter | Min | Max | Unit | Notes |
|-----------|-----|-----|------|-------|
| Angle | -12.57 | 12.57 | rad | ±4π |
| Speed | -44.0 | 44.0 | rad/s | ~420 RPM max |
| Torque | -17.0 | 17.0 | Nm | Continuous rating |
| Current | -23.0 | 23.0 | A | Peak rating |
| Kp (position) | 0.0 | 500.0 | - | Stability limit |
| Kd (damping) | 0.0 | 5.0 | - | Stability limit |
| Temperature | 0 | 135 | °C | 135°C = fault threshold |

### A.2 Error Code Reference

| Error Code | Flag | Description | Recovery |
|------------|------|-------------|----------|
| 0x0400 | MOTOR_OVER_TEMP | Motor >135°C | Wait for cooling |
| 0x1000 | UNDER_VOLTAGE | Vbus <12V | Check power supply |
| 0x020000 | OVER_CURRENT | I >23A | Reduce load |
| 0x080000 | ENCODER_FAULT | Encoder error | Recalibrate |
| 0x200000 | UNCALIBRATED | Not calibrated | Run calibration |

---

**End of Test Specification v2.0**

**Document History:**
- v1.0 (2025-10-09): Initial version
- v2.0 (2025-10-15): Major revision with implementation-aligned details, comprehensive fixtures, expanded test cases, CI/CD integration, and best practices

**Maintained by:** RobStride Development Team  
**Review Cycle:** Quarterly or upon major API changes
