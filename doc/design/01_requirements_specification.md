# RobStride Motor Control Library - Requirements Specification
# RobStride モーター制御ライブラリ - 要件定義書

**Document Version:** 1.0  
**Date:** 2025-10-09  
**Target Motor:** RobStride RS01 Series  
**Original Implementation:** C++ (STM32 HAL)  
**Target Implementation:** Python

---

## 1. Overview / 概要

### 1.1 Purpose / 目的
本ドキュメントは、RobStride RS01 モーター制御ライブラリの機能要件および非機能要件を定義し、Python への移植時に必要な全仕様を明確化する。

### 1.2 Scope / スコープ
- RobStride 独自プロトコル（Private Protocol）のサポート
- MIT プロトコルのサポート
- CAN バス通信による制御・状態取得
- 複数の制御モード（位置、速度、電流、トルク等）

### 1.3 Target Users / 対象ユーザー
- ロボット開発者
- モーション制御エンジニア
- Python でモーター制御を実装する開発者

---

## 2. Functional Requirements / 機能要件

### 2.1 Protocol Support / プロトコルサポート

#### FR-2.1.1 Private Protocol Support
- **Requirement ID:** FR-2.1.1
- **Priority:** Critical
- **Description:** RobStride 独自プロトコル（拡張 CAN ID 使用）による制御をサポート
- **Details:**
  - Extended CAN ID format: `[CommType:8][ErrorCode/Torque:8][MasterID:8][MotorID:8]`
  - Communication types: 0x00-0x19 (25 types)
  - Parameter read/write via index addressing (0x7xxx series)

#### FR-2.1.2 MIT Protocol Support
- **Requirement ID:** FR-2.1.2
- **Priority:** Critical
- **Description:** MIT Cheetah 互換プロトコルによる制御をサポート
- **Details:**
  - Standard CAN ID format
  - Position, speed, torque control with PD gains
  - Compact binary encoding (16-bit scaled values)

#### FR-2.1.3 Protocol Switching
- **Requirement ID:** FR-2.1.3
- **Priority:** High
- **Description:** 実行時のプロトコル切り替え（要モーター再起動）
- **Details:**
  - Modes: 0x00=Private, 0x01=CANopen, 0x02=MIT
  - Must power-cycle motor after protocol change

### 2.2 Motor Control Modes / モーター制御モード

#### FR-2.2.1 Enable/Disable Control
- **Requirement ID:** FR-2.2.1
- **Priority:** Critical
- **Description:** モーターの有効化・無効化
- **Details:**
  - Enable motor: Communication Type 0x03
  - Disable motor: Communication Type 0x04
  - Clear error flag option on disable

#### FR-2.2.2 Motion Control Mode (Torque/Position/Speed Composite)
- **Requirement ID:** FR-2.2.2
- **Priority:** High
- **Description:** 複合運動制御（トルク・位置・速度・ゲイン同時指定）
- **Parameters:**
  - Torque: -4 to 4 Nm
  - Angle: -4π to 4π rad
  - Speed: -30 to 30 rad/s
  - Kp: 0 to 500
  - Kd: 0 to 5
- **CAN Message:** Communication Type 0x01

#### FR-2.2.3 Position Control Mode (PP Mode)
- **Requirement ID:** FR-2.2.3
- **Priority:** High
- **Description:** 位置制御モード（Point-to-Point）
- **Parameters:**
  - Target angle (rad)
  - Target speed (rad/s)
- **Implementation:** Set mode via 0x7005, write target via 0x7016

#### FR-2.2.4 CSP Position Control Mode
- **Requirement ID:** FR-2.2.4
- **Priority:** Medium
- **Description:** 連続位置制御モード（Cyclic Synchronous Position）
- **Parameters:**
  - Target angle (rad)
  - Speed limit (0-44 rad/s)

#### FR-2.2.5 Speed Control Mode
- **Requirement ID:** FR-2.2.5
- **Priority:** High
- **Description:** 速度制御モード
- **Parameters:**
  - Target speed: -30 to 30 rad/s
  - Current limit: 0 to 23 A

#### FR-2.2.6 Current Control Mode
- **Requirement ID:** FR-2.2.6
- **Priority:** High
- **Description:** 電流（トルク）制御モード
- **Parameters:**
  - Target current: -23 to 23 A

#### FR-2.2.7 Zero Position Setting
- **Requirement ID:** FR-2.2.7
- **Priority:** Medium
- **Description:** 現在位置を機械的ゼロ点として設定
- **Process:** Disable → Set zero (0x06) → Enable

### 2.3 Parameter Management / パラメータ管理

#### FR-2.3.1 Parameter Read
- **Requirement ID:** FR-2.3.1
- **Priority:** High
- **Description:** モーターパラメータの読み取り
- **Communication Type:** 0x11
- **Supported Indices:** 0x7005-0x701D (15+ parameters)

#### FR-2.3.2 Parameter Write
- **Requirement ID:** FR-2.3.2
- **Priority:** High
- **Description:** モーターパラメータの書き込み
- **Communication Type:** 0x12
- **Write Modes:**
  - 'p': Parameter value (float, 4 bytes)
  - 'j': Control mode (uint8, 1 byte)

#### FR-2.3.3 Parameter Save
- **Requirement ID:** FR-2.3.3
- **Priority:** Medium
- **Description:** RAM 上のパラメータを FLASH に永続化
- **Communication Type:** 0x16
- **Magic Sequence:** 0x01020304050607008

### 2.4 Status Feedback / 状態フィードバック

#### FR-2.4.1 Real-time Status Reception
- **Requirement ID:** FR-2.4.1
- **Priority:** Critical
- **Description:** モーターからのリアルタイム状態受信
- **Data Fields (Private Protocol):**
  - Angle (position): -12.5 to 12.5 rad
  - Speed (velocity): -44 to 44 rad/s
  - Torque: -17 to 17 Nm
  - Temperature: 0-200°C (0.1°C resolution)
  - Error code: 8-bit bitmap
  - Control mode pattern: 0=torque, 1=position, 2=speed, 3=running

#### FR-2.4.2 Parameter Read Response
- **Requirement ID:** FR-2.4.2
- **Priority:** High
- **Description:** パラメータ読み出し要求への応答受信
- **Response Type:** Communication Type 0x11 response
- **Data:** Index (2 bytes) + Value (4 bytes float or 1 byte uint8)

#### FR-2.4.3 Error Status Monitoring
- **Requirement ID:** FR-2.4.3
- **Priority:** Critical
- **Description:** エラー状態の監視と報告
- **Error Bits:**
  - Bit 0: Under-voltage
  - Bit 1: Over-current
  - Bit 2: Over-temperature
  - Bit 3: Encoder error
  - Bit 4: Over-voltage
  - Bit 5: Not calibrated

### 2.5 MIT Protocol Specific Functions / MIT プロトコル専用機能

#### FR-2.5.1 MIT Enable/Disable
- **Requirement ID:** FR-2.5.1
- **Priority:** Critical
- **Description:** MIT モードでのモーター有効化/無効化
- **Enable Sequence:** 0xFFFFFFFFFFFFFFFC (8 bytes)
- **Disable Sequence:** 0xFFFFFFFFFFFFFFFD

#### FR-2.5.2 MIT Composite Control
- **Requirement ID:** FR-2.5.2
- **Priority:** High
- **Description:** MIT 複合制御（位置・速度・ゲイン・トルク同時指定）
- **Standard CAN ID:** motor_id
- **Payload:** 8 bytes (packed 12/16-bit values)

#### FR-2.5.3 MIT Position Control
- **Requirement ID:** FR-2.5.3
- **Priority:** High
- **Description:** MIT 位置制御専用コマンド
- **Standard CAN ID:** (1 << 8) | motor_id
- **Payload:** position (float32) + speed (float32)

#### FR-2.5.4 MIT Speed Control
- **Requirement ID:** FR-2.5.4
- **Priority:** High
- **Description:** MIT 速度制御専用コマンド
- **Standard CAN ID:** (2 << 8) | motor_id
- **Payload:** speed (float32) + current_limit (float32)

#### FR-2.5.5 MIT Zero Position Setting
- **Requirement ID:** FR-2.5.5
- **Priority:** Medium
- **Description:** MIT モードでのゼロ点設定
- **Sequence:** 0xFFFFFFFFFFFFFFFE
- **Precondition:** MIT_Type != positionControl

#### FR-2.5.6 MIT Error Clear
- **Requirement ID:** FR-2.5.6
- **Priority:** High
- **Description:** MIT モードでのエラークリア
- **Sequence:** 0xFFFFFFFFFF[cmd]FB
- **Commands:** 0x00=check, 0x01=clear

#### FR-2.5.7 MIT Motor Type Setting
- **Requirement ID:** FR-2.5.7
- **Priority:** Medium
- **Description:** MIT 動作モード設定
- **Sequence:** 0xFFFFFFFFFF[type]FC
- **Types:** 0x01=operationControl, 0x02=positionControl, 0x03=speedControl

#### FR-2.5.8 MIT Motor ID Setting
- **Requirement ID:** FR-2.5.8
- **Priority:** Low
- **Description:** モーター ID 変更（MIT モード）
- **Sequence:** 0xFFFFFFFFFF[new_id]01
- **Range:** 0x00-0x7F

### 2.6 Configuration Management / 設定管理

#### FR-2.6.1 CAN ID Management
- **Requirement ID:** FR-2.6.1
- **Priority:** Medium
- **Description:** モーター CAN ID の取得・設定
- **Get ID:** Communication Type 0x00
- **Set ID:** Communication Type 0x07
- **Returns:** 64-bit unique MCU ID

#### FR-2.6.2 Baud Rate Change
- **Requirement ID:** FR-2.6.2
- **Priority:** Low
- **Description:** CAN ボーレート変更（要再起動）
- **Communication Type:** 0x17
- **Rates:** 0x01=1M, 0x02=500K, 0x03=250K, 0x04=125K

#### FR-2.6.3 Proactive Reporting Control
- **Requirement ID:** FR-2.6.3
- **Priority:** Medium
- **Description:** 自動状態報告の有効/無効
- **Communication Type:** 0x18
- **Modes:** 0x00=disable, 0x01=enable (10ms interval)

---

## 3. Non-Functional Requirements / 非機能要件

### 3.1 Performance / パフォーマンス

#### NFR-3.1.1 Response Time
- **Requirement ID:** NFR-3.1.1
- **Priority:** High
- **Description:** CAN メッセージ送信レイテンシー < 5ms
- **Rationale:** リアルタイム制御に必要

#### NFR-3.1.2 Update Rate
- **Requirement ID:** NFR-3.1.2
- **Priority:** High
- **Description:** 制御ループ周期 10-100 Hz をサポート
- **Note:** Python GIL の影響を考慮

#### NFR-3.1.3 Message Processing
- **Requirement ID:** NFR-3.1.3
- **Priority:** Medium
- **Description:** 受信メッセージ処理時間 < 1ms
- **Implementation:** 非同期受信スレッド推奨

### 3.2 Reliability / 信頼性

#### NFR-3.2.1 Error Handling
- **Requirement ID:** NFR-3.2.1
- **Priority:** Critical
- **Description:** 全 CAN 通信エラーを検出・報告
- **Error Types:**
  - Timeout (no response)
  - CAN bus error
  - Invalid message format
  - Motor error status

#### NFR-3.2.2 State Consistency
- **Requirement ID:** NFR-3.2.2
- **Priority:** High
- **Description:** 内部状態とモーター実状態の一貫性維持
- **Mechanism:** 定期的な状態確認・同期

#### NFR-3.2.3 Thread Safety
- **Requirement ID:** NFR-3.2.3
- **Priority:** High
- **Description:** マルチスレッド環境での安全性
- **Implementation:** Lock/Queue による同期

### 3.3 Usability / 使いやすさ

#### NFR-3.3.1 Pythonic API
- **Requirement ID:** NFR-3.3.1
- **Priority:** High
- **Description:** Python らしい API 設計
- **Guidelines:**
  - Property アクセス（angle, speed, torque）
  - Context manager サポート（with 文）
  - Type hints 完備
  - Enum による定数管理

#### NFR-3.3.2 Documentation
- **Requirement ID:** NFR-3.3.2
- **Priority:** High
- **Description:** 完全な docstring とサンプルコード
- **Format:** Google style docstring

#### NFR-3.3.3 Error Messages
- **Requirement ID:** NFR-3.3.3
- **Priority:** Medium
- **Description:** 分かりやすいエラーメッセージ（日英対応推奨）

### 3.4 Compatibility / 互換性

#### NFR-3.4.1 Python Version
- **Requirement ID:** NFR-3.4.1
- **Priority:** High
- **Description:** Python 3.8+ をサポート
- **Rationale:** Type hints 完全サポート

#### NFR-3.4.2 CAN Interface
- **Requirement ID:** NFR-3.4.2
- **Priority:** Critical
- **Description:** python-can ライブラリに対応
- **Supported Interfaces:**
  - SocketCAN (Linux)
  - PCAN (Windows/Linux)
  - IXXAT
  - Virtual CAN (testing)

#### NFR-3.4.3 OS Support
- **Requirement ID:** NFR-3.4.3
- **Priority:** Medium
- **Description:** Linux, Windows, macOS 対応
- **Note:** SocketCAN は Linux のみ

### 3.5 Maintainability / 保守性

#### NFR-3.5.1 Code Structure
- **Requirement ID:** NFR-3.5.1
- **Priority:** High
- **Description:** モジュール構成の明確化
- **Structure:**
  - `robstride/protocol/` - Protocol implementations
  - `robstride/motor.py` - Main motor class
  - `robstride/constants.py` - Constants/enums
  - `robstride/exceptions.py` - Custom exceptions

#### NFR-3.5.2 Testing
- **Requirement ID:** NFR-3.5.2
- **Priority:** High
- **Description:** 単体テスト カバレッジ > 80%
- **Framework:** pytest

#### NFR-3.5.3 Logging
- **Requirement ID:** NFR-3.5.3
- **Priority:** Medium
- **Description:** 標準 logging モジュール使用
- **Levels:** DEBUG (CAN messages), INFO (state changes), WARNING/ERROR

---

## 4. Constraints / 制約条件

### 4.1 Hardware Constraints / ハードウェア制約

#### C-4.1.1 CAN Bus Baud Rate
- **Constraint:** 125K, 250K, 500K, 1M bps のみサポート
- **Default:** 1M bps (factory setting)

#### C-4.1.2 Motor ID Range
- **Constraint:** 0x00-0x7F (標準 CAN ID の範囲)
- **Default:** 0x7F

#### C-4.1.3 CAN Message Format
- **Private Protocol:** Extended ID (29-bit), 8-byte payload
- **MIT Protocol:** Standard ID (11-bit), 8-byte payload

### 4.2 Software Constraints / ソフトウェア制約

#### C-4.2.1 Asynchronous Communication
- **Constraint:** CAN 通信は基本非同期（要求→即応答ではない）
- **Implication:** 状態取得は callback/polling 方式

#### C-4.2.2 Parameter Write Timing
- **Constraint:** 一部パラメータは特定モードでのみ変更可能
- **Example:** 0x7016 (target position) は position mode でのみ有効

#### C-4.2.3 Protocol Switch Requirement
- **Constraint:** プロトコル切り替え後は必ずモーター再起動が必要

---

## 5. Assumptions / 前提条件

### A-5.1 CAN Infrastructure
- CAN インターフェース（ハードウェア・ドライバ）が利用可能
- python-can が正常にインストール・設定されている

### A-5.2 Motor Configuration
- モーターが正しく配線・電源供給されている
- モーターの CAN ID が既知または取得可能

### A-5.3 User Knowledge
- ユーザーは基本的な CAN 通信の知識を有する
- モーター制御の基礎（位置・速度・トルク制御）を理解している

---

## 6. Dependencies / 依存関係

### 6.1 External Libraries
- **python-can** >= 4.0.0: CAN バス通信
- **numpy** (optional): 数値計算の最適化

### 6.2 Development Dependencies
- **pytest** >= 7.0: テスティング
- **mypy**: 型チェック
- **black**: コードフォーマット
- **sphinx** (optional): ドキュメント生成

---

## 7. Out of Scope / スコープ外

以下は本ライブラリのスコープ外とする：

1. **GUI アプリケーション**: CLI/API のみ提供
2. **モーター診断ツール**: 基本的な状態監視のみ
3. **軌道計画**: 上位レイヤーで実装
4. **CANopen 完全実装**: RobStride 独自プロトコルのみ
5. **リアルタイム OS サポート**: 汎用 Linux/Windows での動作

---

## 8. Acceptance Criteria / 受入基準

### 8.1 Functional Tests
- [ ] 全制御モード（位置・速度・電流・トルク）で動作確認
- [ ] Private/MIT 両プロトコルで通信成功
- [ ] パラメータ読み書きの正常動作
- [ ] エラー検出・報告の動作確認

### 8.2 Performance Tests
- [ ] 10Hz 制御ループで安定動作
- [ ] CAN メッセージ送信レイテンシー < 5ms
- [ ] CPU 使用率 < 20% (単一モーター制御時)

### 8.3 Compatibility Tests
- [ ] Linux (Ubuntu 22.04) での動作確認
- [ ] Windows 10/11 での動作確認
- [ ] Python 3.8, 3.9, 3.10, 3.11 での動作確認

### 8.4 Documentation
- [ ] 全 public API に docstring
- [ ] 最低 5 つのサンプルコード
- [ ] README with quick start guide

---

## 9. Revision History / 改訂履歴

| Version | Date       | Author | Changes                |
| ------- | ---------- | ------ | ---------------------- |
| 1.0     | 2025-10-09 | -      | Initial specification  |

---

## 10. Approval / 承認

本要件定義書は、RobStride RS01 モーター制御ライブラリの Python 実装において、全ての必須要件を網羅している。

**Document Status:** Draft / 草案  
**Next Review:** Implementation Phase
