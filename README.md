# RobStride モータ制御ライブラリ (Pure Python)

CANインターフェース経由でRobStride BLDCモータを制御するためのPure Pythonライブラリです。C++ ROS2実装を移植し、ROSに依存しないクリーンなモータ制御APIを提供します。

## 特徴

- **Pure Python実装** - ROS依存なし
- **複数の制御モード**:
  - 運控モード (Mode 0): トルク、位置、速度をKp/Kdと組み合わせた制御
  - 位置PPモード (Mode 1): Point-to-Point位置制御
  - 速度モード (Mode 2): 速度制御
  - 電流モード (Mode 3): Iq/Id直接電流制御
  - 位置CSPモード (Mode 5): Cyclic Synchronous Position制御
- **型安全** - mypy対応の完全な型アノテーション
- **PEP 8準拠** - ruffによるフォーマット

## 要件

- Python 3.11以上
- SocketCANインターフェース (Linux)
- CANハードウェアインターフェース (例: canable)

## インストール

### uvを使用 (推奨)

```bash
cd python
uv sync
```

### pipを使用

```bash
cd python
pip install -e .
```

## クイックスタート

```python
from robstride_motor import RobStrideMotor, ActuatorType

# モータを初期化
motor = RobStrideMotor(
    can_interface="can0",
    master_id=0xFF,
    motor_id=0x01,
    actuator_type=ActuatorType.ROBSTRIDE_00,
)

# モータを有効化
motor.enable_motor()

# 運控モードコマンドを送信
feedback = motor.send_motion_command(
    torque=0.0,
    position=1.57,  # rad
    velocity=0.1,   # rad/s
    kp=0.1,
    kd=0.1,
)

print(f"位置: {feedback.position} rad")
print(f"速度: {feedback.velocity} rad/s")
print(f"トルク: {feedback.torque} Nm")
print(f"温度: {feedback.temperature} °C")

# モータを無効化
motor.disable_motor()
```

## サンプルの実行

```bash
cd python
uv run python examples/basic_control.py
```

## CANインターフェースのセットアップ

ライブラリを使用する前に、CANインターフェースが適切に設定されていることを確認してください：

```bash
# 1 MbpsでCANインターフェースを起動
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0

# インターフェースが起動していることを確認
ip link show can0
```

## APIリファレンス

### RobStrideMotorクラス

#### 初期化

```python
motor = RobStrideMotor(
    can_interface: str,      # CANインターフェース名 (例: "can0")
    master_id: int,          # マスターデバイスID (通常 0xFF)
    motor_id: int,           # モータデバイスID
    actuator_type: ActuatorType  # パラメータマッピング用のアクチュエータタイプ
)
```

#### 制御メソッド

- `enable_motor() -> MotorFeedback` - モータを有効化
- `disable_motor(clear_error: bool = False) -> None` - モータを無効化
- `send_motion_command(torque, position, velocity, kp, kd) -> MotorFeedback` - 運控モード
- `send_velocity_command(velocity, acceleration) -> MotorFeedback` - 速度制御モード
- `send_position_pp_command(angle, speed, acceleration) -> MotorFeedback` - PP位置モード
- `send_position_csp_command(angle, speed) -> MotorFeedback` - CSP位置モード
- `send_current_command(iq, id_val) -> MotorFeedback` - 電流制御モード
- `set_zero_position() -> None` - 現在位置を零点に設定
- `get_feedback() -> MotorFeedback` - 現在のモータ状態を取得

### ActuatorType列挙型

```python
ActuatorType.ROBSTRIDE_00  # ROBSTRIDE_06まで
```

各アクチュエータタイプには、事前定義された動作パラメータ（位置範囲、速度範囲、トルク範囲、Kp/Kd範囲）があります。

### MotorFeedback

```python
@dataclass
class MotorFeedback:
    position: float      # rad
    velocity: float      # rad/s
    torque: float        # Nm
    temperature: float   # °C
```

## 開発

### 型チェック

```bash
uv run mypy robstride_motor
```

### LintとFormat

```bash
# チェック
uv run ruff check robstride_motor

# フォーマット
uv run ruff format robstride_motor
```

## アーキテクチャ

本ライブラリは`cpp/`にあるC++ ROS2実装のPure Python移植版です。主な違い：

- **ROS依存なし** - SocketCAN通信に`python-can`を使用
- **Pythonic API** - dataclass、enum、型ヒントを使用
- **シンプル化** - ROS固有のスレッドとノードインフラを削除
- **コア機能を保持** - 全制御モードとCANプロトコル詳細を維持

## プロトコル詳細

詳細なCANプロトコル仕様については`docs/doc.md`を参照してください：
- 29ビット拡張CAN ID構造
- 通信タイプとペイロード
- パラメータインデックス
- 制御モード仕様

## ライセンス

リポジトリのライセンスを参照してください。

## 参考資料

- オリジナルC++実装: `cpp/`
- プロトコルドキュメント: `docs/doc.md`
