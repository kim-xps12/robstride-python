# RobStride Library for Python + uv

RobStride RS02モーターをCANバス経由で制御するためのPython実装です．PrivateプロトコルとMITプロトコルの両方に対応予定です．[公式のROS Bridge](https://github.com/RobStride/robstride_actuator_bridge)のC++クラス実装を参考にPythonへ移植しています．

## Feature

- 複数の制御モード:
  - 運控モード (Mode 0): 目標位置，目標速度，補正トルク，比例ゲイン，微分ゲインの5パラメータを指定して制御
  - 位置PPモード (Mode 1): Profile Position 位置制御
  - 速度モード (Mode 2): 速度制御
  - 電流モード (Mode 3): 電流制御
  - 位置CSPモード (Mode 5): Cyclic Synchronous Position制御

## Environment

- OS:
    - Linux: Ubuntu 24.04（SocketCANインターフェース）
    - macOS: macOS 10.15（gs_usb経由のUSB CANアダプタ）
- CANアダプタ:
    - Linux: DSD TECH SH-C30G（Amazon: https://amzn.asia/d/4n2BXfD）
        - VID:0x1D50, PID:0x606F
- Python 3.11以上
- uvパッケージマネージャ

## Installation

### uv

```bash
# uvをインストール（まだの場合）
curl -LsSf https://astral.sh/uv/install.sh | sh
```

### dependencies

```bash
uv sync
```

## Setup CAN Interface

### Linux (SocketCAN)

ライブラリを使用する前に，CANインターフェースが適切に設定されていることを確認してください：

```bash
# 1 MbpsでCANインターフェースを起動
sudo ip link set up can0 type can bitrate 1000000

# インターフェースが起動していることを確認
ip link show can0
```

### macOS (gs_usb)

macOSではUSB CANアダプタを使用します：

```bash
# USB CANアダプタを接続
# candleLight互換デバイス（VID:0x1D50, PID:0x606F）が必要

# macOSではroot権限が必要な場合があります
sudo uv run examples/cross_platform_example.py
```

### CANインターフェースの指定方法（コード例）

`RobStrideMotor`クラスの`can_interface`パラメータでCANバックエンドを指定します：

```python
from robstride_motor import RobStrideMotor, ActuatorType

# macOS: gs_usbバックエンドを使用
motor = RobStrideMotor(
    can_interface="gs_usb",  # gs_usbを指定
    master_id=0xFF,
    motor_id=127,
    actuator_type=ActuatorType.ROBSTRIDE_02,
    bitrate=1000000,
)

# Linux: socketcanバックエンドを使用
motor = RobStrideMotor(
    can_interface="socketcan",  # socketcanを指定
    master_id=0xFF,
    motor_id=127,
    actuator_type=ActuatorType.ROBSTRIDE_02,
    bitrate=1000000,  # 注: SocketCANでは事前に`ip link`で設定が必要
)

# レガシーモード: チャンネル名を直接指定（SocketCANのみ）
motor = RobStrideMotor(
    can_interface="can0",  # チャンネル名を指定（自動的にsocketcanを使用）
    master_id=0xFF,
    motor_id=127,
    actuator_type=ActuatorType.ROBSTRIDE_02,
)
```

注意:
- `can_interface`に`"gs_usb"`または`"socketcan"`を指定すると、ライブラリの`create_can_bus()`関数を使用してバスを初期化します
- それ以外の値（例: `"can0"`, `"can1"`）を指定すると、レガシーモードとしてSocketCANのチャンネル名として扱われます
- SocketCANを使用する場合、事前に`sudo ip link set up can0 type can bitrate 1000000`でインターフェースを有効化する必要があります

## How to use (example code)

`examples/`ディレクトリにサンプル実装が用意されています．

### モーターのスキャン

CANバス上のモーターを検出します：

```bash
# RobStrideのデフォルトID 127を指定してスキャン
 uv run examples/scan_motors.py --start 127 --end 127

# デフォルト範囲（1～127）をスキャン
uv run examples/scan_motors.py

# ID範囲を指定してスキャン
uv run examples/scan_motors.py --start 1 --end 10

# CANインターフェースを指定
uv run examples/scan_motors.py --interface can1 --start 125 --end 128
```

### CAn Node IDの変更

```bash
# ID 127 を 1 に変更
uv run examples/set_motor_id.py --current-id 127 --new-id 1 --save
```

### 位置制御（運控モード / MIT方式）

運控モード（Mode 0）を使用した位置制御です．ホスト側で軌道を計算し，トルク・位置・速度・Kp/Kdを毎制御周期送信します：

```bash
# 制御周期を精度良く保つループでゼロ点へ移動
uv run examples/move_to_zero_mit.py --motor-id 127

# リアルタイムプロット付き
uv run examples/move_to_zero_mit_plot.py --motor-id 127

# その場でトルクをオンする
uv run examples/enable_torque_mit.py --motor-id 127

# 柔らかめの位置制御
uv run examples/enable_torque_mit.py --motor-id 127 --kp 2 --kd 0.02 
```

### 位置制御（PPモード）

PPモード（Mode 1）を使用した位置制御です．目標位置・速度・加速度を指定すると，モーター内部で軌道を生成して移動します：

```bash
uv run examples/move_to_zero_pp.py --motor-id 11
```

### ゼロ点の設定

現在のモーター位置をエンコーダのゼロ点として設定します：

```bash
uv run examples/set_custom_zero.py --motor-id 127

# CANインターフェースを指定する場合
uv run examples/set_custom_zero.py --interface can0 --motor-id 127
```

### ゼロ点モードの変更

```bash
# 現在のzero_sta値を表示
uv run examples/set_zero_sta.py --motor-id 127

# zero_staを1に設定（-π～πモード）
uv run examples/set_zero_sta.py --motor-id 127 --set 1

# zero_staを0に設定（0～2πモード、デフォルト）
uv run examples/set_zero_sta.py --motor-id 127 --set 0
```

### (WIP) ファームウェアバージョンの確認

```bash
uv run examples/get_firmware_version.py --motor-id 127
```

## (WIP) Quick Start

```python
from robstride_motor import RobStrideMotor, ActuatorType

# モータを初期化
motor = RobStrideMotor(
    can_interface="can0",
    master_id=255,
    motor_id=1,
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

## API Reference

### RobStrideMotorクラス

#### 初期化

```python
motor = RobStrideMotor(
    can_interface: str,      # CANインターフェース名 (例: "can0")
    master_id: int,          # マスターデバイスID (通常 255)
    motor_id: int,           # モータデバイスID
    actuator_type: ActuatorType  # パラメータマッピング用のアクチュエータタイプ
)
```

#### (WIP)制御メソッド

- `enable_motor() -> MotorFeedback` - モータを有効化
- `disable_motor(clear_error: bool = False) -> None` - モータを無効化
- `send_motion_command(torque, position, velocity, kp, kd) -> MotorFeedback` - 運控モード
- `send_velocity_command(velocity, acceleration) -> MotorFeedback` - 速度制御モード
- `send_position_pp_command(angle, speed, acceleration) -> MotorFeedback` - PP位置モード
- `send_position_csp_command(angle, speed) -> MotorFeedback` - CSP位置モード
- `send_current_command(iq, id_val) -> MotorFeedback` - 電流制御モード
- `set_zero_position() -> None` - 現在位置を零点に設定
- `get_feedback() -> MotorFeedback` - 現在のモータ状態を取得

### (WIP)ActuatorType列挙型

```python
ActuatorType.ROBSTRIDE_00  # ROBSTRIDE_06まで
```

各アクチュエータタイプには，事前定義された動作パラメータ（位置範囲，速度範囲，トルク範囲，Kp/Kd範囲）があります．

### MotorFeedback

```python
@dataclass
class MotorFeedback:
    position: float      # rad
    velocity: float      # rad/s
    torque: float        # Nm
    temperature: float   # °C
```

## for Developer

### Type check

```bash
uv run mypy robstride_motor
```

### Lint / Format

```bash
# Linter
uv run ruff check robstride_motor

# Formater
uv run ruff format robstride_motor
```

## Reference

- Orginal Implementation: 
  - https://github.com/RobStride/robstride_actuator_bridge