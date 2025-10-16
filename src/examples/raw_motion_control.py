#!/usr/bin/env python3
"""
RS02 モーション制御モード（通信タイプ1）の直接実装テスト
doc/reference_specification/rs02_ja.md の仕様に基づく実装
"""

import can
import time
import struct

# ===== 定数定義 (RS02仕様より) =====
MOTOR_CAN_ID = 0x7F  # スキャンで検出されたモータID
MASTER_ID = 0xFD     # ホスト/マスターID

# 通信タイプ
COMM_TYPE_GET_ID = 0x00
COMM_TYPE_MOTION_CONTROL = 0x01
COMM_TYPE_FEEDBACK = 0x02
COMM_TYPE_ENABLE = 0x03
COMM_TYPE_DISABLE = 0x04

# エンコード範囲（仕様書より）
P_MIN = -12.57  # -4π rad
P_MAX = 12.57   # +4π rad
V_MIN = -44.0   # rad/s
V_MAX = 44.0    # rad/s
T_MIN = -17.0   # Nm
T_MAX = 17.0    # Nm
KP_MIN = 0.0
KP_MAX = 500.0
KD_MIN = 0.0
KD_MAX = 5.0


def build_extended_can_id(comm_type: int, data16: int, target_id: int) -> int:
    """
    29ビット拡張CANIDを構築
    Bit28-24: 通信タイプ (5ビット)
    Bit23-8:  データ領域(2) (16ビット)
    Bit7-0:   目標アドレス (8ビット)
    """
    return ((comm_type & 0x1F) << 24) | ((data16 & 0xFFFF) << 8) | (target_id & 0xFF)


def float_to_uint(value: float, min_val: float, max_val: float, bits: int) -> int:
    """浮動小数点数を指定ビット数の符号なし整数にマッピング"""
    # 範囲制限
    if value > max_val:
        value = max_val
    elif value < min_val:
        value = min_val
    
    # 正規化して整数値に変換
    span = max_val - min_val
    normalized = (value - min_val) / span
    max_int = (1 << bits) - 1
    return int(normalized * max_int)


def uint_to_float(value: int, min_val: float, max_val: float, bits: int) -> float:
    """符号なし整数を浮動小数点数に逆マッピング"""
    max_int = (1 << bits) - 1
    normalized = value / max_int
    return min_val + normalized * (max_val - min_val)


def send_enable(bus: can.Bus, motor_id: int, master_id: int) -> bool:
    """モータ有効化コマンド（通信タイプ3）を送信"""
    ext_id = build_extended_can_id(COMM_TYPE_ENABLE, master_id << 8, motor_id)
    msg = can.Message(
        arbitration_id=ext_id,
        is_extended_id=True,
        data=[0x00] * 8,
        dlc=8
    )
    try:
        bus.send(msg, timeout=0.1)
        print(f"[TX] ENABLE: EXTID=0x{ext_id:08X}")
        return True
    except can.CanError as e:
        print(f"Enable送信失敗: {e}")
        return False


def send_disable(bus: can.Bus, motor_id: int, master_id: int) -> bool:
    """モータ無効化コマンド（通信タイプ4）を送信"""
    ext_id = build_extended_can_id(COMM_TYPE_DISABLE, master_id << 8, motor_id)
    msg = can.Message(
        arbitration_id=ext_id,
        is_extended_id=True,
        data=[0x00] * 8,
        dlc=8
    )
    try:
        bus.send(msg, timeout=0.1)
        print(f"[TX] DISABLE: EXTID=0x{ext_id:08X}")
        return True
    except can.CanError as e:
        print(f"Disable送信失敗: {e}")
        return False


def send_motion_control(bus: can.Bus, motor_id: int, master_id: int,
                       torque: float, angle: float, speed: float,
                       kp: float, kd: float) -> bool:
    """
    モーション制御コマンド（通信タイプ1）を送信
    
    仕様:
    - ExtID: [Type:5][Torque:16][MotorID:8]
    - Data: [Angle:16][Speed:16][Kp:16][Kd:16] (ビッグエンディアン、高バイトが前)
    """
    # パラメータをエンコード
    torque_uint = float_to_uint(torque, T_MIN, T_MAX, 16)
    angle_uint = float_to_uint(angle, P_MIN, P_MAX, 16)
    speed_uint = float_to_uint(speed, V_MIN, V_MAX, 16)
    kp_uint = float_to_uint(kp, KP_MIN, KP_MAX, 16)
    kd_uint = float_to_uint(kd, KD_MIN, KD_MAX, 16)
    
    # 拡張CANIDを構築（data16にトルクを設定）
    ext_id = build_extended_can_id(COMM_TYPE_MOTION_CONTROL, torque_uint, motor_id)
    
    # データペイロード（ビッグエンディアン、高バイトが前）
    data = bytearray(8)
    data[0] = (angle_uint >> 8) & 0xFF  # 角度 高バイト
    data[1] = angle_uint & 0xFF         # 角度 低バイト
    data[2] = (speed_uint >> 8) & 0xFF  # 速度 高バイト
    data[3] = speed_uint & 0xFF         # 速度 低バイト
    data[4] = (kp_uint >> 8) & 0xFF     # Kp 高バイト
    data[5] = kp_uint & 0xFF            # Kp 低バイト
    data[6] = (kd_uint >> 8) & 0xFF     # Kd 高バイト
    data[7] = kd_uint & 0xFF            # Kd 低バイト
    
    msg = can.Message(
        arbitration_id=ext_id,
        is_extended_id=True,
        data=data,
        dlc=8
    )
    
    try:
        bus.send(msg, timeout=0.1)
        print(f"[TX] MOTION_CONTROL: EXTID=0x{ext_id:08X}")
        print(f"     Torque={torque:.2f}Nm({torque_uint}), Angle={angle:.2f}rad({angle_uint})")
        print(f"     Speed={speed:.2f}rad/s({speed_uint}), Kp={kp:.1f}({kp_uint}), Kd={kd:.1f}({kd_uint})")
        return True
    except can.CanError as e:
        print(f"Motion control送信失敗: {e}")
        return False


def parse_feedback(msg: can.Message, master_id: int):
    """
    フィードバックメッセージ（通信タイプ2または0x18）をパース
    
    Type 0x02 ExtID: [Type:5][Error:6+Pattern:2+MotorID:8][MasterID:8]
    Type 0x18 ExtID: [Type:5][Error:6+Pattern:2+MotorID:8][MasterID:8]
    Data: [Angle:16][Speed:16][Torque:16][Temp:16] (ビッグエンディアン)
    """
    if not msg.is_extended_id:
        return None
    
    ext_id = msg.arbitration_id
    comm_type = (ext_id >> 24) & 0x1F
    data_field = (ext_id >> 8) & 0xFFFF
    id8 = ext_id & 0xFF
    
    # 通信タイプ2または0x18のフィードバックをチェック
    # 0x18の応答は id8=0x00 になる（マスターIDではなく0）
    if (comm_type == COMM_TYPE_FEEDBACK and id8 == master_id) or \
       (comm_type == 0x18 and id8 == 0x00):
        # data_fieldから情報を抽出
        pattern = (data_field >> 14) & 0x03
        error_code = (data_field >> 8) & 0x3F
        motor_id = data_field & 0xFF
        
        # データペイロードをデコード（ビッグエンディアン）
        angle_uint = (msg.data[0] << 8) | msg.data[1]
        speed_uint = (msg.data[2] << 8) | msg.data[3]
        torque_uint = (msg.data[4] << 8) | msg.data[5]
        temp_raw = (msg.data[6] << 8) | msg.data[7]
        
        # 物理値に変換
        # Type 0x18は角度範囲が-12.57～+12.57（Type 0x02と同じ）
        # 速度は-33～+33（Type 0x02の-44～+44と異なる）
        # トルクは-14～+14（Type 0x02の-17～+17と異なる）
        if comm_type == 0x18:
            angle = uint_to_float(angle_uint, -12.57, 12.57, 16)
            speed = uint_to_float(speed_uint, -33.0, 33.0, 16)
            torque = uint_to_float(torque_uint, -14.0, 14.0, 16)
        else:
            angle = uint_to_float(angle_uint, P_MIN, P_MAX, 16)
            speed = uint_to_float(speed_uint, V_MIN, V_MAX, 16)
            torque = uint_to_float(torque_uint, T_MIN, T_MAX, 16)
        
        temperature = temp_raw / 10.0  # 温度は×10で送られてくる
        
        return {
            'motor_id': motor_id,
            'pattern': pattern,
            'error_code': error_code,
            'angle': angle,
            'speed': speed,
            'torque': torque,
            'temperature': temperature,
            'comm_type': comm_type
        }
    
    return None


def main():
    print("=" * 60)
    print("RS02 モーション制御モード 直接実装テスト")
    print("=" * 60)
    print(f"モータCAN ID: 0x{MOTOR_CAN_ID:02X}")
    print(f"マスターID  : 0x{MASTER_ID:02X}")
    print()
    
    # CANバス初期化
    try:
        bus = can.interface.Bus(
            channel='can0',
            interface='socketcan',  # bustypeの代わりにinterfaceを使用
            bitrate=1_000_000
        )
        print("✓ CAN interface opened (can0 @ 1Mbps)")
    except Exception as e:
        print(f"✗ CANインターフェースの初期化失敗: {e}")
        return
    
    try:
        # 1. モータを有効化
        print("\n[1] モータ有効化コマンド送信...")
        if not send_enable(bus, MOTOR_CAN_ID, MASTER_ID):
            print("✗ Enable失敗")
            return
        time.sleep(3.0)  # モータ起動待ち
        
        # フィードバック確認
        print("    応答待ち...")
        deadline = time.time() + 1.0
        received_feedback = False
        while time.time() < deadline:
            msg = bus.recv(timeout=0.1)
            if msg:
                feedback = parse_feedback(msg, MASTER_ID)
                if feedback:
                    print(f"[RX] FEEDBACK: Motor=0x{feedback['motor_id']:02X}, "
                          f"Type=0x{feedback['comm_type']:02X}, "
                          f"Angle={feedback['angle']:+.3f}rad, "
                          f"Speed={feedback['speed']:+.2f}rad/s, "
                          f"Torque={feedback['torque']:+.2f}Nm, "
                          f"Temp={feedback['temperature']:.1f}°C")
                    received_feedback = True
                    break
        
        if not received_feedback:
            print("    ⚠ フィードバックなし（モータがまだ応答していない可能性）")
        
        # 2. モーション制御で目標位置0 radへ移動
        print("\n[2] モーション制御: 目標位置 0.0 rad へ移動")
        print("    パラメータ: Kp=50.0, Kd=1.0, 速度=0.0, トルク=0.0")
        
        # 複数回送信して確実に制御
        for i in range(10):
            success = send_motion_control(
                bus=bus,
                motor_id=MOTOR_CAN_ID,
                master_id=MASTER_ID,
                torque=0.0,   # フィードフォワードトルク
                angle=0,    # 目標角度
                speed=0.0,    # 目標速度
                kp=50.0,      # 位置ゲイン
                kd=5.0        # 速度ゲイン
            )
            
            if not success:
                print(f"✗ 送信失敗 ({i+1}/10)")
                continue
            
            # フィードバック受信
            time.sleep(0.05)
            msg = bus.recv(timeout=0.1)
            if msg:
                feedback = parse_feedback(msg, MASTER_ID)
                if feedback:
                    print(f"[RX] FEEDBACK #{i+1}: "
                          f"Type=0x{feedback['comm_type']:02X}, "
                          f"Angle={feedback['angle']:+.3f}rad, "
                          f"Speed={feedback['speed']:+.2f}rad/s, "
                          f"Torque={feedback['torque']:+.2f}Nm")
        
        # 3. 最終状態確認
        print("\n[3] 最終状態確認（2秒間フィードバック監視）")
        deadline = time.time() + 2.0
        feedback_count = 0
        
        while time.time() < deadline:
            msg = bus.recv(timeout=0.1)
            if msg:
                feedback = parse_feedback(msg, MASTER_ID)
                if feedback:
                    feedback_count += 1
                    print(f"[RX] #{feedback_count}: "
                          f"Angle={feedback['angle']:+.3f}rad, "
                          f"Speed={feedback['speed']:+.2f}rad/s, "
                          f"Torque={feedback['torque']:+.2f}Nm, "
                          f"Temp={feedback['temperature']:.1f}°C, "
                          f"Error=0x{feedback['error_code']:02X}")
        
        if feedback_count == 0:
            print("    ⚠ フィードバックが受信できませんでした")
            print("    考えられる原因:")
            print("    - モータが実際に接続されていない")
            print("    - モータIDが間違っている")
            print("    - 自動レポートが無効になっている")
        
        print("\n" + "=" * 60)
        print("テスト完了")
        print("=" * 60)
    
    except KeyboardInterrupt:
        print("\n中断されました")
    
    finally:
        # 最終的にモータを無効化してからバスを閉じる
        try:
            # send_disable may fail if bus is already down; ignore errors
            send_disable(bus, MOTOR_CAN_ID, MASTER_ID)
            print("モータ無効化コマンド送信済み")
        except Exception:
            pass

        try:
            bus.shutdown()
            print("CANバスをクローズしました")
        except Exception as e:
            print(f"CANバスのクローズでエラー: {e}")


if __name__ == "__main__":
    main()
