# RobStride Motor Control Library - Parameter Mapping Document
# RobStride モーター制御ライブラリ - パラメータマッピング資料

**Document Version:** 1.0  
**Date:** 2025-10-09

---

## 1. Overview / 概要

本ドキュメントは、RobStride モーターの全パラメータ（0x7xxx シリーズ）の詳細仕様を定義する。各パラメータの役割、データ型、範囲、読み書き可否、使用モードを網羅する。

---

## 2. Parameter Index List / パラメータインデックス一覧

### 2.1 Core Parameters (Index_List)

| Index  | Name             | Type    | R/W | Range            | Unit    | Description                    |
| ------ | ---------------- | ------- | --- | ---------------- | ------- | ------------------------------ |
| 0x7005 | run_mode         | uint8   | R/W | 0-5              | -       | 動作モード                     |
| 0x7006 | iq_ref           | float32 | R/W | -23 ~ 23         | A       | 電流指令値（電流モード）       |
| 0x700A | spd_ref          | float32 | R/W | -30 ~ 30         | rad/s   | 速度指令値（速度モード）       |
| 0x700B | limit_torque     | float32 | R/W | 0 ~ 12           | Nm      | トルク制限                     |
| 0x7010 | cur_kp           | float32 | R/W | 0 ~ 10           | -       | 電流制御 Kp ゲイン             |
| 0x7011 | cur_ki           | float32 | R/W | 0 ~ 1            | -       | 電流制御 Ki ゲイン             |
| 0x7014 | cur_filt_gain    | float32 | R/W | 0 ~ 1.0          | -       | 電流フィルタゲイン             |
| 0x7016 | loc_ref          | float32 | R/W | -∞ ~ +∞          | rad     | 位置指令値（位置モード）       |
| 0x7017 | limit_spd        | float32 | R/W | 0 ~ 44           | rad/s   | 速度制限（CSP モード）         |
| 0x7018 | limit_cur        | float32 | R/W | 0 ~ 23           | A       | 電流制限（速度・位置モード）   |
| 0x7019 | mech_pos         | float32 | R   | -∞ ~ +∞          | rad     | 機械側多回転角度（累積）       |
| 0x701A | iqf              | float32 | R   | -23 ~ 23         | A       | 電流フィルタ値（実測）         |
| 0x701B | mech_vel         | float32 | R   | -30 ~ 30         | rad/s   | 機械側速度（実測）             |
| 0x701C | vbus             | float32 | R   | 0 ~ 60           | V       | バス電圧（実測）               |
| 0x701D | rotation         | int16   | R   | -32768 ~ 32767   | rounds  | 回転数（整数）                 |

### 2.2 Extended Parameters

| Index  | Name             | Type    | R/W | Range     | Unit    | Description                    |
| ------ | ---------------- | ------- | --- | --------- | ------- | ------------------------------ |
| 0x7022 | accel_spd        | float32 | R/W | 0 ~ 100   | rad/s²  | 加速度設定（速度モード）       |
| 0x7024 | limit_spd_pp     | float32 | R/W | 0 ~ 30    | rad/s   | 速度制限（PP 位置モード）      |
| 0x7025 | acceleration     | float32 | R/W | 0 ~ 100   | rad/s²  | 加速度設定（位置モード）       |

**Note:** 0x7022, 0x7024, 0x7025 は Index_List には含まれないが、C++ コードで使用される追加パラメータ。

---

## 3. Detailed Parameter Specifications / パラメータ詳細仕様

### 3.1 0x7005: run_mode (動作モード)

**Description:**  
モーターの制御モードを設定。この値によって有効なパラメータが変わる。

**Data Type:** uint8 (1 byte)

**Valid Values:**
- `0`: Motion Control Mode (複合運動制御)
- `1`: Position Control Mode (PP 位置制御)
- `2`: Speed Control Mode (速度制御)
- `3`: Current Control Mode (電流制御)
- `4`: Set Zero Mode (ゼロ点設定)
- `5`: CSP Position Control Mode (CSP 位置制御)

**Read/Write:** R/W

**Usage:**
```python
# 位置モードに切り替え
motor.set_parameter(0x7005, 1, value_mode='j')
```

**Notes:**
- モード変更後は `Enable_Motor()` を呼ぶこと
- モード変更前に `Get_RobStride_Motor_parameter(0x7005)` で確認推奨

---

### 3.2 0x7006: iq_ref (電流指令値)

**Description:**  
電流制御モード（mode=3）での目標電流。直接トルクに対応。

**Data Type:** float32 (4 bytes)

**Range:** -23.0 ~ 23.0 A

**Read/Write:** R/W

**Active Mode:** Current Control (mode=3)

**Usage:**
```python
# 1.5A の電流指令
motor.set_parameter(0x7006, 1.5, value_mode='p')
```

**Notes:**
- 符号：正=CW、負=CCW（エンコーダ方向依存）
- limit_cur (0x7018) による制限も考慮される

---

### 3.3 0x700A: spd_ref (速度指令値)

**Description:**  
速度制御モード（mode=2）での目標速度。

**Data Type:** float32 (4 bytes)

**Range:** -30.0 ~ 30.0 rad/s

**Read/Write:** R/W

**Active Mode:** Speed Control (mode=2)

**Usage:**
```python
# 10 rad/s で回転
motor.set_parameter(0x700A, 10.0, value_mode='p')
```

**Notes:**
- 加速度は 0x7022 で制限可能
- limit_cur (0x7018) で電流制限

---

### 3.4 0x700B: limit_torque (トルク制限)

**Description:**  
運動制御モードでのトルク上限値。

**Data Type:** float32 (4 bytes)

**Range:** 0.0 ~ 12.0 Nm

**Read/Write:** R/W

**Active Mode:** Motion Control (mode=0)

**Default:** 12.0 Nm (unlimited)

**Usage:**
```python
# トルクを 5Nm に制限
motor.set_parameter(0x700B, 5.0, value_mode='p')
```

---

### 3.5 0x7010: cur_kp (電流制御 Kp)

**Description:**  
電流ループ PI 制御の比例ゲイン。

**Data Type:** float32 (4 bytes)

**Range:** 0.0 ~ 10.0

**Read/Write:** R/W

**Default:** 0.125

**Usage:**
```python
# Kp を 0.2 に変更
motor.set_parameter(0x7010, 0.2, value_mode='p')
```

**Notes:**
- 変更後は動作確認必須（振動の可能性）
- 通常はデフォルト値で十分

---

### 3.6 0x7011: cur_ki (電流制御 Ki)

**Description:**  
電流ループ PI 制御の積分ゲイン。

**Data Type:** float32 (4 bytes)

**Range:** 0.0 ~ 1.0

**Read/Write:** R/W

**Default:** 0.0158

**Usage:**
```python
# Ki を 0.02 に変更
motor.set_parameter(0x7011, 0.02, value_mode='p')
```

**Notes:**
- 大きすぎるとオーバーシュート
- 小さすぎると定常偏差

---

### 3.7 0x7014: cur_filt_gain (電流フィルタゲイン)

**Description:**  
電流測定値のローパスフィルタゲイン。

**Data Type:** float32 (4 bytes)

**Range:** 0.0 ~ 1.0

**Read/Write:** R/W

**Default:** 0.1

**Usage:**
```python
# フィルタを強くする
motor.set_parameter(0x7014, 0.05, value_mode='p')
```

**Notes:**
- 小さい値 = 強いフィルタ（遅延増）
- 大きい値 = 弱いフィルタ（ノイズ増）

---

### 3.8 0x7016: loc_ref (位置指令値)

**Description:**  
位置制御モード（mode=1, 5）での目標角度。

**Data Type:** float32 (4 bytes)

**Range:** -∞ ~ +∞ rad (multi-turn)

**Read/Write:** R/W

**Active Mode:** Position PP (mode=1), Position CSP (mode=5)

**Usage:**
```python
# 1.57 rad（約90°）に移動
motor.set_parameter(0x7016, 1.57, value_mode='p')
```

**Notes:**
- 多回転対応（累積角度）
- mode=1 では速度制限 (0x7024) を事前設定
- mode=5 では速度制限 (0x7017) を使用

---

### 3.9 0x7017: limit_spd (CSP 速度制限)

**Description:**  
CSP 位置制御モード（mode=5）での速度上限。

**Data Type:** float32 (4 bytes)

**Range:** 0.0 ~ 44.0 rad/s

**Read/Write:** R/W

**Active Mode:** Position CSP (mode=5)

**Default:** 30.0 rad/s

**Usage:**
```python
# CSP モードで速度を 5 rad/s に制限
motor.set_parameter(0x7017, 5.0, value_mode='p')
```

---

### 3.10 0x7018: limit_cur (電流制限)

**Description:**  
速度・位置制御モードでの電流（トルク）上限。

**Data Type:** float32 (4 bytes)

**Range:** 0.0 ~ 23.0 A

**Read/Write:** R/W

**Active Mode:** Speed (mode=2), Position (mode=1, 5)

**Default:** 23.0 A (unlimited)

**Usage:**
```python
# 電流を 3A に制限
motor.set_parameter(0x7018, 3.0, value_mode='p')
```

**Notes:**
- 安全のため適切な制限を設定推奨
- 大きなトルクが必要な場合は緩める

---

### 3.11 0x7019: mech_pos (機械位置)

**Description:**  
負荷側（機械側）の多回転絶対角度。**読み取り専用。**

**Data Type:** float32 (4 bytes)

**Range:** -∞ ~ +∞ rad (累積)

**Read/Write:** **Read-Only**

**Usage:**
```python
# 読み取り要求
motor.get_parameter(0x7019)
time.sleep(0.01)
print(f"Mechanical position: {motor.drw.mech_pos} rad")
```

**Notes:**
- 減速比が反映された値
- ゼロ点設定 (Set_ZeroPos) の影響を受ける
- 電源 OFF で保持されない（相対値）

---

### 3.12 0x701A: iqf (電流フィルタ値)

**Description:**  
実測電流のフィルタ後の値。**読み取り専用。**

**Data Type:** float32 (4 bytes)

**Range:** -23.0 ~ 23.0 A

**Read/Write:** **Read-Only**

**Usage:**
```python
motor.get_parameter(0x701A)
time.sleep(0.01)
print(f"Filtered current: {motor.drw.iqf} A")
```

---

### 3.13 0x701B: mech_vel (機械速度)

**Description:**  
負荷側（機械側）の速度。**読み取り専用。**

**Data Type:** float32 (4 bytes)

**Range:** -30.0 ~ 30.0 rad/s

**Read/Write:** **Read-Only**

**Usage:**
```python
motor.get_parameter(0x701B)
time.sleep(0.01)
print(f"Mechanical velocity: {motor.drw.mech_vel} rad/s")
```

---

### 3.14 0x701C: vbus (バス電圧)

**Description:**  
モーター駆動電圧（バス電圧）。**読み取り専用。**

**Data Type:** float32 (4 bytes)

**Range:** 0.0 ~ 60.0 V

**Read/Write:** **Read-Only**

**Usage:**
```python
motor.get_parameter(0x701C)
time.sleep(0.01)
print(f"Bus voltage: {motor.drw.vbus} V")
```

**Notes:**
- 低電圧（< 12V 等）で Under-voltage エラー
- 高電圧（> 50V 等）で Over-voltage エラー

---

### 3.15 0x701D: rotation (回転数)

**Description:**  
モーター回転数（整数）。**読み取り専用。**

**Data Type:** int16 (2 bytes)

**Range:** -32768 ~ 32767 rounds

**Read/Write:** **Read-Only**

**Usage:**
```python
motor.get_parameter(0x701D)
time.sleep(0.01)
print(f"Rotation count: {motor.drw.rotation} rounds")
```

**Notes:**
- ゼロ点からの累積回転数
- オーバーフロー注意（±32767 回転まで）

---

### 3.16 0x7022: accel_spd (速度モード加速度)

**Description:**  
速度制御モードでの加速度制限。

**Data Type:** float32 (4 bytes)

**Range:** 0.0 ~ 100.0 rad/s²

**Read/Write:** R/W

**Active Mode:** Speed Control (mode=2)

**Default:** 10.0 rad/s² (C++ コード内で設定)

**Usage:**
```python
# 加速度を 10 rad/s² に設定
motor.set_parameter(0x7022, 10.0, value_mode='p')
```

---

### 3.17 0x7024: limit_spd_pp (PP 速度制限)

**Description:**  
PP 位置制御モード（mode=1）での速度上限。

**Data Type:** float32 (4 bytes)

**Range:** 0.0 ~ 30.0 rad/s

**Read/Write:** R/W

**Active Mode:** Position PP (mode=1)

**Usage:**
```python
# PP モードで速度を 3 rad/s に設定
motor.set_parameter(0x7024, 3.0, value_mode='p')
```

**Notes:**
- PP モードでは事前設定が必須
- 設定しないと動作しないことがある

---

### 3.18 0x7025: acceleration (位置モード加速度)

**Description:**  
位置制御モードでの加速度制限。

**Data Type:** float32 (4 bytes)

**Range:** 0.0 ~ 100.0 rad/s²

**Read/Write:** R/W

**Active Mode:** Position PP (mode=1), Position CSP (mode=5)

**Usage:**
```python
# 加速度を 5 rad/s² に設定
motor.set_parameter(0x7025, 5.0, value_mode='p')
```

---

## 4. Parameter Dependencies / パラメータ依存関係

### 4.1 Mode-Specific Parameters

| Mode | Required Params          | Optional Params               |
| ---- | ------------------------ | ----------------------------- |
| 0    | run_mode=0               | limit_torque                  |
| 1    | run_mode=1, loc_ref      | limit_spd_pp, acceleration, limit_cur |
| 2    | run_mode=2, spd_ref      | accel_spd, limit_cur          |
| 3    | run_mode=3, iq_ref       | -                             |
| 4    | run_mode=4               | -                             |
| 5    | run_mode=5, loc_ref      | limit_spd, limit_cur          |

### 4.2 Parameter Write Sequence

#### Position Control (PP Mode)
```python
# 1. Set mode
motor.set_parameter(0x7005, 1, value_mode='j')

# 2. (Optional) Set speed limit
motor.set_parameter(0x7024, 3.0, value_mode='p')

# 3. (Optional) Set acceleration
motor.set_parameter(0x7025, 5.0, value_mode='p')

# 4. (Optional) Set current limit
motor.set_parameter(0x7018, 5.0, value_mode='p')

# 5. Enable motor
motor.enable_motor()

# 6. Write target position
motor.set_parameter(0x7016, 1.57, value_mode='p')
```

#### Speed Control Mode
```python
# 1. Set mode
motor.set_parameter(0x7005, 2, value_mode='j')

# 2. (Optional) Set current limit
motor.set_parameter(0x7018, 3.0, value_mode='p')

# 3. (Optional) Set acceleration
motor.set_parameter(0x7022, 10.0, value_mode='p')

# 4. Enable motor
motor.enable_motor()

# 5. Write target speed
motor.set_parameter(0x700A, 10.0, value_mode='p')
```

---

## 5. Parameter Read/Write Timing / 読み書きタイミング

### 5.1 Write Timing

| Parameter       | Min Interval | Typical Use Case           |
| --------------- | ------------ | -------------------------- |
| run_mode        | 100 ms       | Mode change                |
| loc_ref         | 1 ms         | Position update (CSP)      |
| spd_ref         | 1 ms         | Speed update               |
| iq_ref          | 1 ms         | Current update             |
| limit_*         | 50 ms        | Safety limit change        |
| cur_kp/ki/filt  | -            | One-time tuning            |

### 5.2 Read Timing

| Parameter | Response Time | Typical Interval |
| --------- | ------------- | ---------------- |
| mech_pos  | < 10 ms       | 10-50 ms         |
| mech_vel  | < 10 ms       | 10-50 ms         |
| vbus      | < 10 ms       | 100 ms           |
| iqf       | < 10 ms       | 10-50 ms         |

**Note:** Auto-reporting (0x18) を有効にすると、一部パラメータは 10ms 周期で自動送信される。

---

## 6. Common Configurations / 一般的な設定例

### 6.1 High-Speed Position Control

```python
# 高速位置制御（CSP モード）
motor.set_parameter(0x7005, 5, value_mode='j')  # CSP mode
motor.set_parameter(0x7017, 20.0, value_mode='p')  # Speed limit: 20 rad/s
motor.set_parameter(0x7018, 10.0, value_mode='p')  # Current limit: 10 A
motor.enable_motor()

# Control loop (100 Hz)
while True:
    target = calculate_trajectory(time.time())
    motor.set_parameter(0x7016, target, value_mode='p')
    time.sleep(0.01)
```

### 6.2 Smooth Speed Ramp

```python
# 滑らかな速度制御
motor.set_parameter(0x7005, 2, value_mode='j')  # Speed mode
motor.set_parameter(0x7022, 2.0, value_mode='p')  # Low acceleration for smooth ramp
motor.set_parameter(0x7018, 5.0, value_mode='p')  # Current limit: 5 A
motor.enable_motor()

motor.set_parameter(0x700A, 15.0, value_mode='p')  # Target: 15 rad/s
```

### 6.3 Torque Control with Limit

```python
# トルク制限付き電流制御
motor.set_parameter(0x7005, 3, value_mode='j')  # Current mode
motor.enable_motor()

motor.set_parameter(0x7006, 2.5, value_mode='p')  # Target: 2.5 A (~1 Nm)
```

---

## 7. Troubleshooting / トラブルシューティング

### 7.1 Position Mode Not Moving

**Symptoms:** PP 位置モードで目標を書き込んでも動かない

**Possible Causes:**
1. `limit_spd_pp` (0x7024) が未設定または 0
2. `limit_cur` (0x7018) が小さすぎる
3. `run_mode` が正しく設定されていない

**Solution:**
```python
# Verify mode
motor.get_parameter(0x7005)
time.sleep(0.01)
assert motor.drw.run_mode == 1, "Mode not set to position"

# Set speed limit
motor.set_parameter(0x7024, 3.0, value_mode='p')

# Set current limit
motor.set_parameter(0x7018, 5.0, value_mode='p')

# Write target again
motor.set_parameter(0x7016, 1.57, value_mode='p')
```

### 7.2 Speed Oscillation

**Symptoms:** 速度が振動する、不安定

**Possible Causes:**
1. `cur_kp` が大きすぎる
2. `cur_filt_gain` が大きすぎる（フィルタ弱）
3. 負荷慣性が大きい

**Solution:**
```python
# Reduce Kp
motor.set_parameter(0x7010, 0.08, value_mode='p')

# Increase filtering
motor.set_parameter(0x7014, 0.05, value_mode='p')
```

### 7.3 Parameter Read Timeout

**Symptoms:** `get_parameter()` で応答がない

**Possible Causes:**
1. モーターが無効化状態
2. CAN バス切断
3. パラメータインデックスが無効

**Solution:**
```python
# Enable motor first
motor.enable_motor()
time.sleep(0.1)

# Then read
motor.get_parameter(0x7019)
time.sleep(0.1)

if motor.drw.mech_pos == 0.0:
    print("Warning: No response or value is actually zero")
```

---

## 8. Parameter Persistence / パラメータ永続化

### 8.1 Volatile Parameters (RAM)

以下のパラメータは電源 OFF で消失：
- `loc_ref` (0x7016) - 位置指令
- `spd_ref` (0x700A) - 速度指令
- `iq_ref` (0x7006) - 電流指令

### 8.2 Persistent Parameters (FLASH)

`save_parameters()` で FLASH に保存可能：
- `run_mode` (0x7005)
- `limit_*` 系パラメータ
- `cur_kp`, `cur_ki`, `cur_filt_gain`

**Usage:**
```python
# Configure parameters
motor.set_parameter(0x7018, 5.0, value_mode='p')  # Current limit
motor.set_parameter(0x7010, 0.1, value_mode='p')  # Kp

# Save to FLASH
motor.save_parameters()

# These will be default values on next boot
```

**Warning:** FLASH has limited write cycles (~10,000). Do not save repeatedly in control loops.

---

**End of Parameter Mapping Document**
