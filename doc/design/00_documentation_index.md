# RobStride Motor Control Library - Documentation Index
# RobStride モーター制御ライブラリ - ドキュメント索引

**Documentation Version:** 1.0  
**Last Updated:** 2025-10-09

---

## 📚 Documentation Overview / ドキュメント概要

本ドキュメントセットは、RobStride モーター制御ライブラリ（C++実装）を **Python に完全移植** するために必要な全仕様を網羅している。

### Purpose / 目的
- C++ 実装の完全な仕様化
- Python 実装者への明確なガイドライン提供
- プロトコル、データ構造、状態機械の詳細記述
- テスト戦略とベストプラクティスの提示

---

## 📖 Document List / ドキュメント一覧

### Core Specification Documents / 核となる仕様書

| # | Document | File | Pages | Description |
|---|----------|------|-------|-------------|
| 01 | **Requirements Specification**<br/>要件定義書 | `01_requirements_specification.md` | ~50KB | 機能要件・非機能要件・制約条件・受け入れ基準 |
| 02 | **API Specification**<br/>API 仕様書 | `02_api_specification.md` | ~60KB | 全 API メソッド仕様、引数、戻り値、使用例 |
| 03 | **CAN Protocol Specification**<br/>CAN プロトコル仕様書 | `03_can_protocol_specification.md` | ~45KB | Private/MIT プロトコルの詳細、CAN ID 構成、メッセージフォーマット |
| 04 | **Data Structures Specification**<br/>データ構造仕様書 | `04_data_structures_specification.md` | ~65KB | Python データクラス、列挙型、定数、バリデーション関数 |
| 05 | **Parameter Mapping**<br/>パラメータマッピング | `05_parameter_mapping.md` | ~40KB | 全パラメータ (0x7xxx) の詳細、範囲、読み書き権限 |
| 06 | **State Machine Design**<br/>状態機械設計 | `06_state_machine_design.md` | ~50KB | モーター状態、遷移シーケンス、初期化手順 |
| 07 | **Error Handling Specification**<br/>エラー処理仕様 | `07_error_handling_specification.md` | ~45KB | エラー検出、分類、復旧戦略、ログ記録 |
| 08 | **Python Implementation Guide**<br/>Python 実装ガイド | `08_python_implementation_guide.md` | ~60KB | パッケージ構成、クラス設計、コード例 |
| 09 | **Test Specification**<br/>テスト仕様 | `09_test_specification.md` | ~55KB | テスト戦略、ユニット/統合/システムテスト、HIL テスト |
| 10 | **Documentation Index** (This File)<br/>ドキュメント索引 | `00_documentation_index.md` | ~15KB | 全ドキュメントの索引、用語集、参照資料 |

**Total Documentation Size:** ~485KB of detailed specifications

---

## 🗺️ Reading Guide / 読解ガイド

### For First-Time Implementers / 初めて実装する方

**推奨読解順序:**

```
1. 01_requirements_specification.md
   ↓ 全体像を把握
   
2. 02_api_specification.md
   ↓ API インターフェースを理解
   
3. 03_can_protocol_specification.md
   ↓ CAN 通信の詳細を学ぶ
   
4. 04_data_structures_specification.md
   ↓ データ型を確認
   
5. 08_python_implementation_guide.md
   ↓ 実装方法を習得
   
6. 05_parameter_mapping.md
   ↓ パラメータ詳細を参照
   
7. 06_state_machine_design.md
   ↓ 状態管理を設計
   
8. 07_error_handling_specification.md
   ↓ エラー処理を実装
   
9. 09_test_specification.md
   ↓ テストを作成
```

### For Experienced Developers / 経験豊富な開発者

**クイックスタート:**
- `08_python_implementation_guide.md` で実装アーキテクチャを確認
- `03_can_protocol_specification.md` で CAN メッセージフォーマットを把握
- `02_api_specification.md` で必要な API を探す
- `05_parameter_mapping.md` で特定パラメータを検索

### For Testing Engineers / テストエンジニア

**テスト設計:**
- `09_test_specification.md` でテスト戦略を確認
- `01_requirements_specification.md` で受け入れ基準を把握
- `07_error_handling_specification.md` でエラーケースを理解

---

## 🔍 Quick Reference / クイックリファレンス

### Key Concepts / 重要概念

#### Protocols / プロトコル
- **Private Protocol:** Extended CAN ID (29-bit), 汎用パラメータアクセス
- **MIT Protocol:** Standard CAN ID (11-bit), 高速インピーダンス制御

#### Control Modes / 制御モード
- **Mode 0:** Motion Control (複合運動制御)
- **Mode 1:** Position PP (位置制御、事前設定速度)
- **Mode 2:** Speed (速度制御)
- **Mode 3:** Current (電流制御)
- **Mode 4:** Set Zero (ゼロ点設定)
- **Mode 5:** Position CSP (位置制御、動的速度制限)

#### Critical Parameters / 重要パラメータ
- `0x7005`: run_mode (制御モード)
- `0x7016`: loc_ref (位置指令)
- `0x700A`: spd_ref (速度指令)
- `0x7006`: iq_ref (電流指令)
- `0x7019`: mech_pos (機械位置、読み取り専用)
- `0x7018`: limit_cur (電流制限)
- `0x7017`: limit_spd (速度制限、CSP)

#### State Transitions / 状態遷移
```
UNINITIALIZED → DISABLED → ENABLED → RUNNING → DISABLED
                    ↑                   ↓
                    └─────── FAULT ←────┘
```

---

## 📑 Document Summaries / ドキュメントサマリー

### 01. Requirements Specification
**Purpose:** プロジェクトの要件定義  
**Key Sections:**
- Functional Requirements (30+ features)
- Non-Functional Requirements (performance, reliability, compatibility)
- Constraints (CAN bitrate, Python version)
- Acceptance Criteria

**Critical Requirements:**
- FR-001: Motor enable/disable control
- FR-005: Position control with 0.05 rad accuracy
- NFR-002: Control loop frequency > 100 Hz
- NFR-003: CAN communication latency < 10 ms

---

### 02. API Specification
**Purpose:** 全 API メソッドの詳細定義  
**Key Sections:**
- Core Control Methods (Enable, Disable, Set_Position, Set_Speed, etc.)
- Parameter Access (Read/Write with 0x7xxx indices)
- MIT Protocol Methods (30+ methods total)

**Most Used APIs:**
```python
motor.enable()
motor.set_control_mode(ControlMode.POSITION_CSP)
motor.set_position(1.57)
motor.get_parameter(0x7019)
```

---

### 03. CAN Protocol Specification
**Purpose:** CAN バスメッセージフォーマット定義  
**Key Sections:**
- Extended CAN ID format (29-bit) for Private protocol
- Standard CAN ID (11-bit 0x7FF) for MIT protocol
- Command types (0x00-0x19)
- Data encoding (float/int packing)

**Example Message:**
```
Private Enable: ID=0x01000100, Data=[]
MIT Enable: ID=0x7FF, Data=[0xFF]*7 + [0xFC]
Parameter Write: ID=0x12000100, Data=[0x16, 0x70, <float bytes>]
```

---

### 04. Data Structures Specification
**Purpose:** Python データ構造の完全定義  
**Key Sections:**
- MotorStatus dataclass (angle, velocity, torque, temperature, etc.)
- ParameterData, MotorConfiguration
- Enums (ControlMode, MITMotorType, ProtocolMode, ErrorFlag)
- Constants (ParameterIndex, ValueLimits)
- Validation functions

**Example:**
```python
@dataclass
class MotorStatus:
    angle: float = 0.0
    velocity: float = 0.0
    torque: float = 0.0
    temperature: int = 0
    error_flags: int = 0
```

---

### 05. Parameter Mapping
**Purpose:** 全パラメータ (0x7xxx) の詳細仕様  
**Key Sections:**
- 18+ parameter definitions
- Data types, ranges, read/write permissions
- Mode-specific parameters
- Common configurations

**Parameter Summary:**
- Read/Write: run_mode, iq_ref, spd_ref, loc_ref, limits
- Read-Only: mech_pos, mech_vel, iqf, vbus, temperature, rotation

---

### 06. State Machine Design
**Purpose:** モーター状態管理の設計  
**Key Sections:**
- 5 primary states (UNINITIALIZED, DISABLED, ENABLED, RUNNING, FAULT)
- State transition sequences
- Control mode state machines
- Protocol switching procedures
- Initialization sequences

**Key Transition:**
```python
# Mode change requires disable → enable
motor.disable()
motor.set_parameter(0x7005, new_mode)
motor.enable()
```

---

### 07. Error Handling Specification
**Purpose:** エラー検出と復旧戦略  
**Key Sections:**
- 8-bit error flag bitmap (Over-temperature, Over-current, etc.)
- Error detection mechanisms
- Recovery strategies (automatic/manual)
- Error logging

**Error Codes:**
- 0x01: Over-Temperature (> 80°C)
- 0x02: Over-Current (> 23A)
- 0x04: Over-Voltage (> 50V)
- 0x08: Under-Voltage (< 12V)
- 0x10: Encoder Error
- 0x80: CAN Timeout (> 500ms)

---

### 08. Python Implementation Guide
**Purpose:** Python 実装の具体的手順  
**Key Sections:**
- Package structure (robstride/ with submodules)
- Core classes (RobStrideMotor, PrivateProtocolHandler, MITProtocolHandler)
- Utility functions (float_to_uint, uint_to_float, validation)
- Usage examples (position control, speed control, MIT mode, multi-motor)

**Package Layout:**
```
robstride/
├── motor.py
├── protocol/ (private.py, mit.py)
├── data/ (structures.py, enums.py)
├── control/ (position.py, speed.py)
├── error/ (handler.py, logger.py)
└── utils/ (validation.py, conversion.py)
```

---

### 09. Test Specification
**Purpose:** 包括的テスト戦略  
**Key Sections:**
- Unit tests (50+ test cases)
- Integration tests (protocol, control modes)
- System tests (hardware communication, performance)
- HIL tests (trajectory tracking, multi-motor sync)
- Acceptance tests (user stories)

**Coverage Targets:**
- Unit: > 80%
- Integration: > 70%
- Critical paths: 100%

---

## 🔧 Implementation Checklist / 実装チェックリスト

### Phase 1: Foundation (Week 1-2)
- [ ] Setup Python package structure
- [ ] Implement data structures (MotorStatus, enums)
- [ ] Implement CAN message encoding/decoding
- [ ] Create unit tests for encoding

### Phase 2: Core Functionality (Week 3-4)
- [ ] Implement PrivateProtocolHandler
- [ ] Implement RobStrideMotor class (basic methods)
- [ ] Add parameter read/write
- [ ] Create integration tests

### Phase 3: Advanced Features (Week 5-6)
- [ ] Implement MIT protocol
- [ ] Add control mode management
- [ ] Implement error handling
- [ ] Create system tests

### Phase 4: Optimization & Testing (Week 7-8)
- [ ] Performance optimization
- [ ] Complete test coverage
- [ ] Documentation review
- [ ] User acceptance testing

### Phase 5: Deployment (Week 9)
- [ ] Package for PyPI
- [ ] CI/CD setup
- [ ] User manual
- [ ] Release v1.0.0

---

## 📊 Glossary / 用語集

### Technical Terms / 技術用語

| Term | Japanese | Definition |
|------|----------|------------|
| CAN Bus | CAN バス | Controller Area Network, 車載ネットワーク規格 |
| Extended ID | 拡張 ID | 29-bit CAN identifier (vs 11-bit Standard ID) |
| MIT Protocol | MIT プロトコル | MIT Cheetah ロボット由来の高速制御プロトコル |
| Private Protocol | プライベートプロトコル | RobStride 独自の汎用パラメータアクセスプロトコル |
| CSP Mode | CSP モード | Cyclic Synchronous Position (周期同期位置制御) |
| PP Mode | PP モード | Point-to-Point position control (点間位置制御) |
| Impedance Control | インピーダンス制御 | 位置・力の同時制御 (Kp, Kd ゲイン使用) |
| Encoder | エンコーダ | 角度センサー |
| BLDC | ブラシレス DC モーター | Brushless DC Motor |
| Torque Constant | トルク定数 | 電流→トルク変換係数 |
| Gear Ratio | 減速比 | モーター軸→負荷軸の回転比 |

### Parameter Names / パラメータ名

| Index | Name | Japanese | Unit |
|-------|------|----------|------|
| 0x7005 | run_mode | 動作モード | - |
| 0x7006 | iq_ref | 電流指令 | A |
| 0x700A | spd_ref | 速度指令 | rad/s |
| 0x7016 | loc_ref | 位置指令 | rad |
| 0x7017 | limit_spd | 速度制限 (CSP) | rad/s |
| 0x7018 | limit_cur | 電流制限 | A |
| 0x7019 | mech_pos | 機械位置 | rad |
| 0x701A | iqf | フィルタ電流 | A |
| 0x701B | mech_vel | 機械速度 | rad/s |
| 0x701C | vbus | バス電圧 | V |
| 0x701E | error_flags | エラーフラグ | bitmap |
| 0x701F | temperature | 温度 | °C |

---

## 🔗 Cross-References / 相互参照

### API → Protocol Mapping

| API Method | Protocol | Document Reference |
|------------|----------|-------------------|
| `enable()` | Private 0x01 | 02_API §3.1, 03_CAN §3.1 |
| `disable()` | Private 0x02 | 02_API §3.2, 03_CAN §3.2 |
| `set_parameter()` | Private 0x12 | 02_API §4.1, 03_CAN §3.7 |
| `get_parameter()` | Private 0x11 | 02_API §4.2, 03_CAN §3.6 |
| `send_mit_command()` | MIT | 02_API §6.4, 03_CAN §4.3 |

### Error → Recovery Mapping

| Error Code | Error Name | Recovery Document |
|------------|------------|-------------------|
| 0x01 | Over-Temperature | 07_Error §3.1 |
| 0x02 | Over-Current | 07_Error §3.2 |
| 0x04 | Over-Voltage | 07_Error §3.3 |
| 0x08 | Under-Voltage | 07_Error §3.3 |
| 0x10 | Encoder Error | 07_Error §3.4 |
| 0x80 | CAN Timeout | 07_Error §3.5 |

### Test → Requirement Mapping

| Test Case | Requirement | Document Reference |
|-----------|-------------|-------------------|
| TC-S-010 | FR-005 (Position accuracy) | 01_Req §2.1, 09_Test §5.2 |
| TC-S-011 | FR-006 (Speed control) | 01_Req §2.1, 09_Test §5.2 |
| TC-P-001 | NFR-003 (Latency) | 01_Req §2.2, 09_Test §8.1 |
| TC-H-001 | FR-008 (Trajectory) | 01_Req §2.1, 09_Test §6.1 |

---

## 📚 External References / 外部参照資料

### Original Source Code
- **Location:** `/home/yutaro/Downloads/SampleProgram/`
- **Key Files:**
  - `RS/Robstride.h` - C++ class definitions
  - `RS/Robstride01.cpp` - Implementation (831 lines)
  - `Src/main.c` - Usage examples
  - `README.md` - Chinese/English documentation

### Related Standards
- **CAN Specification:** ISO 11898-1 (CAN 2.0)
- **Python CAN Library:** [python-can documentation](https://python-can.readthedocs.io/)
- **MIT Cheetah Protocol:** [MIT Biomimetics Lab](https://biomimetics.mit.edu/)

### Recommended Reading
- "Embedded Systems Firmware Demystified" - Ed Sutter
- "Designing Embedded Systems with PIC Microcontrollers" - Tim Wilmshurst (CAN chapter)
- "Modern Robotics: Mechanics, Planning, and Control" - Kevin M. Lynch (Control theory)

---

## 📝 Document Maintenance / ドキュメントメンテナンス

### Version History

| Version | Date | Changes | Author |
|---------|------|---------|--------|
| 1.0.0 | 2025-10-09 | Initial complete documentation set | GitHub Copilot |

### Review Schedule
- **Minor Updates:** As needed (bug fixes, clarifications)
- **Major Updates:** Quarterly (new features, protocol changes)
- **Annual Review:** Full documentation audit

### Contributing
ドキュメントの改善提案は GitHub Issues または Pull Requests で受け付けています。

---

## 🎯 Success Criteria / 成功基準

実装完了の判定基準:

✅ **Completeness / 完全性**
- [ ] All 30+ API methods implemented
- [ ] Both Private and MIT protocols functional
- [ ] All 18+ parameters accessible
- [ ] Error handling for all 6 error types

✅ **Quality / 品質**
- [ ] Unit test coverage > 80%
- [ ] Integration test coverage > 70%
- [ ] All system tests passing
- [ ] Code follows PEP 8 style

✅ **Performance / 性能**
- [ ] Control loop frequency > 100 Hz
- [ ] CAN latency < 10 ms
- [ ] Position accuracy < 0.05 rad
- [ ] Speed stability < 1 rad/s error

✅ **Usability / 使いやすさ**
- [ ] PyPI package published
- [ ] Installation < 5 minutes
- [ ] Example code working
- [ ] User documentation complete

---

## 📞 Support / サポート

### Technical Questions
- **GitHub Issues:** https://github.com/yourname/robstride-motor/issues
- **Email:** support@example.com

### Community
- **Forum:** https://forum.example.com/robstride
- **Discord:** https://discord.gg/robstride

---

**End of Documentation Index**

**Total Documentation Package:**
- 10 comprehensive documents
- ~485 KB of specifications
- 100+ code examples
- 50+ test cases
- Complete API coverage

**Ready for Python implementation! 🚀**
