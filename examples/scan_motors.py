#!/usr/bin/env python3
"""CANバス上のRobStrideモーターをスキャンするスクリプト.

指定したID範囲でモーターとの疎通を確認します。

使用方法:
    uv run examples/scan_motors.py
    uv run examples/scan_motors.py --start 1 --end 10
    uv run examples/scan_motors.py --interface can0 --start 0x01 --end 0x7F
"""

import argparse
import time

import can


def scan_motor(bus: can.Bus, motor_id: int, master_id: int = 0xFF, timeout: float = 0.1) -> bool:
    """指定したモーターIDと疎通を確認する.

    Args:
        bus: CANバスインスタンス
        motor_id: 確認するモーターID
        master_id: マスターID
        timeout: 応答待ちタイムアウト（秒）

    Returns:
        疎通が確認できた場合True
    """
    # Enable command (通信タイプ3) を送信
    communication_type = 0x03  # MOTOR_ENABLE
    can_id = (communication_type & 0x1F) << 24 | (master_id & 0xFFFF) << 8 | (motor_id & 0xFF)

    msg = can.Message(
        arbitration_id=can_id,
        data=bytes([0] * 8),
        is_extended_id=True,
    )

    # バッファをクリア
    while bus.recv(timeout=0.001):
        pass

    try:
        bus.send(msg)
    except can.CanError:
        return False

    # 応答を待つ
    start_time = time.time()
    while time.time() - start_time < timeout:
        response = bus.recv(timeout=0.01)
        if response is not None:
            # 応答のモーターIDを確認
            response_motor_id = (response.arbitration_id >> 8) & 0xFF
            if response_motor_id == motor_id:
                # Disable command を送信してモーターを停止
                disable_type = 0x04  # MOTOR_STOP
                disable_can_id = (disable_type & 0x1F) << 24 | (master_id & 0xFFFF) << 8 | (motor_id & 0xFF)
                disable_msg = can.Message(
                    arbitration_id=disable_can_id,
                    data=bytes([0] * 8),
                    is_extended_id=True,
                )
                bus.send(disable_msg)
                time.sleep(0.01)
                return True

    return False


def main() -> int:
    """メイン関数."""
    parser = argparse.ArgumentParser(description="Scan for RobStride motors on CAN bus")
    parser.add_argument("--interface", default="can0", help="CAN interface (default: can0)")
    parser.add_argument(
        "--start",
        type=lambda x: int(x, 0),
        default=0x01,
        help="Start motor ID (default: 0x01)",
    )
    parser.add_argument(
        "--end",
        type=lambda x: int(x, 0),
        default=0x7F,
        help="End motor ID (default: 0x7F)",
    )
    parser.add_argument(
        "--master",
        type=lambda x: int(x, 0),
        default=0xFF,
        help="Master ID (default: 0xFF)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.1,
        help="Response timeout in seconds (default: 0.1)",
    )
    args = parser.parse_args()

    print(f"CANインターフェース: {args.interface}")
    print(f"スキャン範囲: 0x{args.start:02X} ~ 0x{args.end:02X}")
    print(f"マスターID: 0x{args.master:02X}")
    print(f"タイムアウト: {args.timeout}秒")
    print("-" * 40)

    try:
        bus = can.interface.Bus(
            channel=args.interface,
            interface="socketcan",
            receive_own_messages=False,
        )
    except Exception as e:
        print(f"CANバスの初期化に失敗: {e}")
        return 1

    found_motors: list[int] = []

    try:
        for motor_id in range(args.start, args.end + 1):
            print(f"  ID 0x{motor_id:02X} をスキャン中...", end=" ", flush=True)
            if scan_motor(bus, motor_id, args.master, args.timeout):
                print("✓ 検出")
                found_motors.append(motor_id)
            else:
                print("- 応答なし")

    except KeyboardInterrupt:
        print("\n\nユーザーにより中断されました")
    finally:
        bus.shutdown()

    print("-" * 40)
    if found_motors:
        print(f"検出されたモーター ({len(found_motors)}台):")
        for motor_id in found_motors:
            print(f"  - 0x{motor_id:02X} ({motor_id})")
    else:
        print("モーターは検出されませんでした")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
