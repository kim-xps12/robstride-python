# (WIP)RobStride モーター制御ライブラリ for Python

RobStride RS02モーターをCANバス経由で制御するためのPython実装です。PrivateプロトコルとMITプロトコルの両方に対応しています。

## 特徴

- **デュアルプロトコル対応**: Private（RobStride独自）およびMIT（Cheetah）プロトコル
- **多様な制御モード**: 位置、速度、電流/トルク、および複合モーション制御
- **包括的なAPI**: すべてのモーターパラメータと制御機能へのフルアクセス
- **エラーハンドリング**: エラー検出、分類、復旧戦略を内蔵
- **スレッドセーフ**: リアルタイムステータス更新用のバックグラウンドCANメッセージリスナー

## インストール

### 必要要件

- Python 3.8 以上
- CANインターフェース
- `uv`パッケージマネージャー（推奨）
- OS: Ubuntu 24.04（推奨，親しい環境でも動作する可能性はあります）

### uvのインストール

```bash
# uvをインストール（まだの場合）
curl -LsSf https://astral.sh/uv/install.sh | sh
```

### プロジェクトのセットアップ

```bash
git clone https://github.com/yourusername/robstride-python.git
cd robstride-python

# uvで依存関係を自動インストール（仮想環境も自動作成）
uv sync
```

## クイックスタート

### CANノードIDスキャン

複数モーターが接続されている場合やCAN IDが不明なときは、`src/examples/scan_ids.py` を使ってバス上の応答するノードを検出できます。スクリプトは0x00〜0x7Fを順にプローブし、応答があればCAN IDと64bitユニークIDを表示します。

```bash
# 例: can0インターフェースでスキャン

cd robstride-python/
uv run python src/examples/scan_ids.py --interface can0 --start 0x00 --end 0x7F
```

### 基本的な位置制御

**ファイル作成**: `test_position.py`

```python
from robstride import RobStrideMotor, ProtocolMode

# モーターを初期化
motor = RobStrideMotor(
    can_id=0x01,
    can_interface='can0',
    protocol=ProtocolMode.PRIVATE,
    auto_enable=True
)

# 位置を設定
motor.position_control.set_pp_position(target_angle=1.57, target_speed=3.0)

# ステータスを取得
print(f"角度: {motor.angle:.3f} rad")
print(f"速度: {motor.speed:.3f} rad/s")
print(f"トルク: {motor.torque:.3f} Nm")
print(f"温度: {motor.temperature:.1f}°C")

# 終了時にモーターを無効化
motor.disable_motor()
```

**実行方法**:
```bash
uv run python test_position.py
```

### 速度制御

**ファイル作成**: `test_speed.py`

```python
from robstride import RobStrideMotor

motor = RobStrideMotor(can_id=0x01, can_interface='can0', auto_enable=True)

# 電流制限付きで速度を設定
motor.speed_control.set_speed(target_speed=10.0, current_limit=5.0)

# 速度を監視
print(f"現在の速度: {motor.speed:.2f} rad/s")
```

**実行方法**:
```bash
uv run python test_speed.py
```

### MITプロトコル制御

**ファイル作成**: `test_mit.py`

```python
from robstride import RobStrideMotor, ProtocolMode

motor = RobStrideMotor(
    can_id=0x01,
    can_interface='can0',
    protocol=ProtocolMode.MIT,
    auto_enable=True
)

# PDゲイン付きMIT複合制御
motor.send_mit_control(
    position=1.57,      # 目標位置 [rad]
    velocity=0.0,       # 目標速度 [rad/s]
    kp=50.0,           # 位置ゲイン
    kd=1.0,            # ダンピングゲイン
    torque=0.0         # フィードフォワードトルク [Nm]
)
```

**実行方法**:
```bash
uv run python test_mit.py
```

## 制御モード

### Privateプロトコル

1. **モーション制御モード (0)**: トルク、位置、速度、PDゲインを使った複合制御
2. **位置制御 PP (1)**: ポイントツーポイント位置制御
3. **速度制御 (2)**: 電流制限付き速度制御
4. **電流制御 (3)**: 直接電流（トルク）制御
5. **位置制御 CSP (5)**: サイクリック同期位置制御

### MITプロトコル

- **複合制御**: 位置、速度、Kp、Kd、フィードフォワードトルク
- **位置制御**: 位置と速度目標値
- **速度制御**: 電流制限付き速度

## パラメータアクセス

```python
from robstride.models import ParameterIndex

# パラメータ読み取り
motor.get_parameter(ParameterIndex.VBUS)
print(f"バス電圧: {motor.param_data.vbus:.1f} V")

# パラメータ書き込み
motor.set_parameter(ParameterIndex.LIMIT_CUR, 5.0, value_mode='p')

# パラメータをFLASHに保存
motor.save_parameters()
```

## エラーハンドリング

```python
# エラーチェック
if motor.has_error:
    print(f"エラー検出: {motor.error_description}")
    
    # 自動復旧
    motor.error_handler.handle_error(motor.status.error_code)
```

## サンプルコード

`src/examples/` ディレクトリに完全なサンプルがあります：

- `basic_position.py`: 基本的な位置制御
- `speed_control.py`: 可変速度制御
- `mit_mode.py`: MITプロトコル複合制御
- `multi_motor.py`: 複数モーターの協調制御

**サンプルの実行方法**:
```bash
# 基本的な位置制御
uv run python src/examples/basic_position.py

# 速度制御
uv run python src/examples/speed_control.py

# MITモード
uv run python src/examples/mit_mode.py

# マルチモーター制御
uv run python src/examples/multi_motor.py
```

## API リファレンス

### 主要クラス

- `RobStrideMotor`: メインのモーター制御インターフェース
- `MotorStatus`: モーターステータスデータ構造
- `ParameterData`: モーターパラメータデータ構造

### 制御戦略

- `PositionController`: 位置制御メソッド
- `SpeedController`: 速度制御メソッド
- `CurrentController`: 電流/トルク制御メソッド

### 列挙型

- `ControlMode`: 制御モード列挙型
- `ProtocolMode`: プロトコルモード列挙型
- `ErrorFlag`: エラーフラグビットマップ
- `MotorState`: モーター状態マシンの状態

## CANインターフェース設定

### Linux (SocketCAN)

```bash
# CANインターフェースを起動
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# 仮想CAN（テスト用）
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up
```

## ドキュメント

`doc/` ディレクトリに包括的なドキュメントがあります：

- `00_documentation_index.md`: ドキュメント概要
- `01_requirements_specification.md`: 機能要件
- `02_api_specification.md`: 完全なAPIリファレンス
- `03_can_protocol_specification.md`: CANプロトコル詳細
- `04_data_structures_specification.md`: データ構造
- `05_parameter_mapping.md`: パラメータ仕様
- `06_state_machine_design.md`: 状態マシン設計
- `07_error_handling_specification.md`: エラーハンドリング
- `08_python_implementation_guide.md`: 実装ガイド
- `09_test_specification.md`: テスト戦略

## 動作要件

- Python >= 3.8
- python-can >= 4.0.0
- typing-extensions >= 4.0.0

## 開発環境

### テストの実行

```bash
# テストを実行
uv run pytest

# カバレッジ付きテスト
uv run pytest --cov=robstride
```

### コードフォーマット

```bash
# Blackでフォーマット
uv run black src/

# 型チェック
uv run mypy src/robstride

# Lintチェック
uv run flake8 src/robstride
```

## ライセンス

MIT License

## 貢献

プルリクエストを歓迎します！お気軽に投稿してください。

## サポート

問題、質問、貢献については、[GitHubリポジトリ](https://github.com/yourusername/robstride-python)をご覧ください。

## 謝辞

RobStride RS02モーター制御仕様とC++リファレンス実装に基づいています。
