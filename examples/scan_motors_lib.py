#!/usr/bin/env python3
"""CANバス上のRobStrideモーターをスキャンするスクリプト.

robstride_motor.bus ライブラリを使用したバージョン。
バックエンド（gs_usb / socketcan）を明示的に指定する必要があります。

使用方法:
gs_usb (USB CANアダプタ、macOS/Linux):
    sudo uv run examples/scan_motors_lib.py --backend gs_usb
    sudo uv run examples/scan_motors_lib.py --backend gs_usb --start 1 --end 10

socketcan (Linux):
    sudo ip link set can0 up type can bitrate 1000000
    uv run examples/scan_motors_lib.py --backend socketcan
    uv run examples/scan_motors_lib.py --backend socketcan --channel can0
"""

import argparse
import sys
import time

import can

from robstride_motor.bus import create_can_bus, shutdown_can_bus


def scan_motor(
    bus: can.BusABC, motor_id: int, master_id: int = 0xFF, timeout: float = 0.1
) -> bool:
    """指定したモーターIDと疎通を確認する.

    GET_ID (通信タイプ0) コマンドを使用してモーターの存在を確認します。
    このコマンドはモーターを有効化しないため、安全にスキャンできます。

    Args:
        bus: CANバスインスタンス
        motor_id: 確認するモーターID
        master_id: マスターID
        timeout: 応答待ちタイムアウト（秒）

    Returns:
        疎通が確認できた場合True
    """
    # GET_ID command (通信タイプ0) を送信
    # モーターを有効化せずにIDを確認できる
    communication_type = 0x00  # GET_ID
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
    # GET_IDの応答: Bit7〜0 = 0xFE, Bit23〜8 = モータCAN_ID
    start_time = time.time()
    while time.time() - start_time < timeout:
        response = bus.recv(timeout=0.01)
        if response is not None:
            # GET_IDの応答フォーマット確認
            # 応答の Bit7〜0 が 0xFE であることを確認
            response_target = response.arbitration_id & 0xFF
            response_motor_id = (response.arbitration_id >> 8) & 0xFF
            if response_target == 0xFE and response_motor_id == motor_id:
                return True

    return False


def main() -> int:
    """メイン関数."""
    parser = argparse.ArgumentParser(
        description="Scan for RobStride motors on CAN bus (using robstride_motor.bus library)"
    )
    parser.add_argument(
        "--backend",
        type=str,
        required=True,
        choices=["gs_usb", "socketcan"],
        help="CAN backend: 'gs_usb' (USB CAN adapter) or 'socketcan' (Linux)",
    )
    parser.add_argument(
        "--channel",
        default="can0",
        help="CAN channel for socketcan (default: can0)",
    )
    parser.add_argument(
        "--bitrate",
        type=int,
        default=1000000,
        help="CAN bitrate in bps (default: 1000000)",
    )
    parser.add_argument(
        "--start",
        type=lambda x: int(x, 0),
        default=1,
        help="Start motor ID (default: 1)",
    )
    parser.add_argument(
        "--end",
        type=lambda x: int(x, 0),
        default=127,
        help="End motor ID (default: 127)",
    )
    parser.add_argument(
        "--master-id",
        type=lambda x: int(x, 0),
        default=255,
        help="Master ID (default: 255)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.1,
        help="Response timeout in seconds (default: 0.1)",
    )
    args = parser.parse_args()

    # コマンドライン引数から元の表記を取得（16進数表示対応）
    start_str = str(args.start)
    end_str = str(args.end)
    master_str = str(args.master_id)

    for i, arg in enumerate(sys.argv):
        if arg == "--start" and i + 1 < len(sys.argv):
            start_str = sys.argv[i + 1]
        elif arg == "--end" and i + 1 < len(sys.argv):
            end_str = sys.argv[i + 1]
        elif arg == "--master-id" and i + 1 < len(sys.argv):
            master_str = sys.argv[i + 1]

    print(f"CANバックエンド: {args.backend}")
    if args.backend == "socketcan":
        print(f"CANチャンネル: {args.channel}")
    print(f"ビットレート: {args.bitrate} bps")
    print(f"スキャン範囲: {start_str} ~ {end_str}")
    print(f"マスターID: {master_str}")
    print(f"タイムアウト: {args.timeout}秒")
    print("-" * 40)

    bus = None
    try:
        # robstride_motor.bus.create_can_bus を使用
        bus = create_can_bus(
            interface=args.backend,  # type: ignore[arg-type]
            channel=args.channel,
            bitrate=args.bitrate,
        )
        print("CANバスに接続しました")
    except Exception as e:
        print(f"CANバスの初期化に失敗: {e}")
        print("")
        if args.backend == "gs_usb":
            print("gs_usbではUSBデバイスへのアクセスにroot権限が必要な場合があります。")
            print("  sudo uv run examples/scan_motors_lib.py --backend gs_usb")
        elif args.backend == "socketcan":
            print("socketcanではインターフェース(can0等)がUPになっているか確認してください。")
            print("  sudo ip link set can0 up type can bitrate 1000000")
        return 1

    found_motors: list[int] = []

    try:
        for motor_id in range(args.start, args.end + 1):
            print(f"  ID {motor_id} をスキャン中...", end=" ", flush=True)
            if scan_motor(bus, motor_id, args.master_id, args.timeout):
                print("✓ 検出")
                found_motors.append(motor_id)
            else:
                print("- 応答なし")

    except KeyboardInterrupt:
        print("\n\nユーザーにより中断されました")
    finally:
        if bus is not None:
            shutdown_can_bus(bus, args.backend)

    print("-" * 40)
    if found_motors:
        print(f"検出されたモーター ({len(found_motors)}台):")
        for motor_id in found_motors:
            print(f"  - {motor_id}")
    else:
        print("モーターは検出されませんでした")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
