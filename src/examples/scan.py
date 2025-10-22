#!/usr/bin/env python3
"""
RS05 CAN IDツール (python-can版)
通信タイプ0でスキャン / 通信タイプ7でID書き換え
1Mbps CAN通信
"""

import can
import time
import sys

# 定数定義
COMM_GET_ID = 0x00  # モード0: GET_ID
COMM_SET_ID = 0x07  # モード7: SET_ID
RESP_MARKER = 0xFE  # 応答ID下位8bit
MASTER_ID = 0xFD    # 任意の上位機ID

last_found_id = None


def build_ext_id(mode, data16, id8):
    """拡張IDを構築 (29bit)"""
    return ((mode & 0x1F) << 24) | ((data16 & 0xFFFF) << 8) | (id8 & 0xFF)


def send_get_id(bus, target_can_id):
    """GET_IDコマンドを送信"""
    ext_id = build_ext_id(COMM_GET_ID, (MASTER_ID << 8) | 0x00, target_can_id)
    msg = can.Message(
        arbitration_id=ext_id,
        is_extended_id=True,
        data=[0] * 8
    )
    try:
        bus.send(msg, timeout=0.005)
        return True
    except can.CanError:
        return False


def send_set_id(bus, current_id, new_id):
    """SET_IDコマンドを送信"""
    data16 = (new_id << 8) | MASTER_ID
    ext_id = build_ext_id(COMM_SET_ID, data16, current_id)
    msg = can.Message(
        arbitration_id=ext_id,
        is_extended_id=True,
        data=[0] * 8
    )
    try:
        bus.send(msg, timeout=0.005)
        return True
    except can.CanError:
        return False


def scan_ids(bus, wait_ms=1500):
    """全ID(0-127)にGET_IDを送信してスキャン"""
    global last_found_id
    
    print("全IDにGET_ID(タイプ0)を送信...")
    for target_id in range(128):
        send_get_id(bus, target_id)
        time.sleep(0.002)  # 200us待機
    
    print("応答待ち...")
    start_time = time.time()
    last_found_id = None
    
    while (time.time() - start_time) < (wait_ms / 1000.0):
        msg = bus.recv(timeout=0.01)
        if msg is None:
            continue
        
        if not msg.is_extended_id:
            continue
        
        if len(msg.data) != 8:
            continue
        
        # 拡張IDをデコード
        ext_id = msg.arbitration_id
        mode = (ext_id >> 24) & 0x1F
        data16 = (ext_id >> 8) & 0xFFFF
        id8 = ext_id & 0xFF
        
        # GET_IDの応答をチェック
        if mode == 0x00 and id8 == RESP_MARKER:
            motor_can_id = data16 & 0x00FF  # 応答data16の下位8bit=CAN ID
            last_found_id = motor_can_id
            print(f"[RX] EXTID=0x{ext_id:08X} mode={mode} data16=0x{data16:04X} "
                  f"id8=0x{id8:02X} => CAN_ID=0x{motor_can_id:02X}")
    
    if last_found_id is None:
        print("応答なし")


def main():
    print("\nRS05 IDツール (Python版)")
    print("コマンド:")
    print("  G           : スキャン")
    print("  S <hex>     : ID変更 (例: S 3B)")
    print("  Q           : 終了")
    
    # CANバスの初期化
    # インターフェース名は環境に応じて変更してください
    # 例: 'can0', 'vcan0', 'socketcan', etc.
    try:
        bus = can.interface.Bus(
            channel='can0',  # 環境に応じて変更
            bustype='socketcan',  # Linux SocketCAN
            bitrate=1000000  # 1Mbps
        )
        print(f"CAN start OK (channel=can0 @1Mbps)")
    except Exception as e:
        print(f"CANバスの初期化に失敗: {e}")
        print("ヒント: sudo ip link set can0 type can bitrate 1000000")
        print("       sudo ip link set up can0")
        return
    
    # 起動時に一発スキャン
    scan_ids(bus)
    
    # メインループ
    try:
        while True:
            try:
                cmd = input("\n> ").strip()
            except EOFError:
                break
            
            if not cmd:
                continue
            
            cmd_upper = cmd.upper()
            
            if cmd_upper == "G":
                scan_ids(bus)
            
            elif cmd_upper.startswith("S "):
                # ID変更コマンド
                parts = cmd.split()
                if len(parts) >= 2:
                    try:
                        new_id = int(parts[1], 16)
                        if 0 <= new_id <= 0x7F:
                            if last_found_id is None:
                                print("現在見えてるIDが無い。先にGでスキャンしろ。")
                            else:
                                print(f"SET_ID: 0x{last_found_id:02X} -> 0x{new_id:02X} ...")
                                if send_set_id(bus, last_found_id, new_id):
                                    time.sleep(0.05)
                                    # 変更直後に再スキャンで確定
                                    scan_ids(bus)
                                else:
                                    print("送信失敗")
                        else:
                            print("ID範囲外。0x00..0x7Fだけや。")
                    except ValueError:
                        print("使い方: S <hex> 例) S 3B")
                else:
                    print("使い方: S <hex> 例) S 3B")
            
            elif cmd_upper == "Q":
                print("終了")
                break
            
            else:
                print("コマンド: G / S <hex> / Q")
    
    except KeyboardInterrupt:
        print("\n中断されました")
    
    finally:
        bus.shutdown()
        print("CANバスをクローズしました")


if __name__ == "__main__":
    main()