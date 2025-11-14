# (WIP)RobStride モーター制御ライブラリ for Python

RobStride RS02モーターをCANバス経由で制御するためのPython実装です。PrivateプロトコルとMITプロトコルの両方に対応予定です．

## 想定動作環境

- OS: Ubuntu 24.04（推奨，親しい環境でも動作する可能性はあります）
- CANアダプタ: DSD TECH SH-C30G
  - Amazon: https://amzn.asia/d/4n2BXfD

## インストール

### 必要要件

- Python 3.8 以上
- `uv`パッケージマネージャー

### uvのインストール

```bash
# uvをインストール（まだの場合）
curl -LsSf https://astral.sh/uv/install.sh | sh
```

### プロジェクトのセットアップ

```bash
git clone https://github.com/kim-xps12/robstride-python.git
cd robstride-python

# uvで依存関係を自動インストール（仮想環境も自動作成）
uv sync --extra test --extra dev
```

## クイックスタート

### CANインターフェースの有効化

```bash
sudo ip link set can0 up type can bitrate 1000000
```

`can0`の箇所は適宜読み替えてください．

### CANノードIDのスキャン

複数モーターが接続されている場合やCAN IDが不明なときは、`src/examples/scan_ids.py` を使ってバス上の応答するノードを検出できます。スクリプトは0x00〜0x7Fを順にプローブし、応答があればCAN IDと64bitユニークIDを表示します。

```bash
# 例: can0インターフェースでスキャン

cd robstride-python/
uv run python src/examples/scan_ids.py --interface can0 --start 0x00 --end 0x7F
```

### ping

`start`と`end`を同じIDにするとpingとして利用できます．

```bash
uv run python src/examples/scan_ids.py --interface can0 --start 0x7F --end 0x7F
```

### ゼロ点の設定

ゼロ点として設定したい位置へモータを回したあとに以下を実行すると，その位置をゼロ点として記憶します．

```bash
uv run python src/examples/set_zero_position.py
```

### 位置制御

#### ゼロ点への移動の例

以下を実行すると，前節で設定したゼロ点へ移動します．

ただし，ゼロ点の設定の後に電源を再投入した場合，回転方向が「移動距離が最小になる」が担保されないようです．

```bash
uv run python src/examples/go_zero_position.py 
```

### 速度制御

(WIP)

### MITプロトコル制御

(WIP)


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

## サンプルコード

`src/examples/` ディレクトリにサンプルがあります：

- `basic_position.py`: 基本的な位置制御
- `speed_control.py`: 可変速度制御
- `mit_mode.py`: MITプロトコル複合制御
- `multi_motor.py`: 複数モーターの協調制御


## API リファレンス

(WIP)

## 開発環境

### テストの実行

```bash
# テストを実行
uv run pytest
```

### コードフォーマット

(WIP)