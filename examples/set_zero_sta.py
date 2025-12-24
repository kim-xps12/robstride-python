#!/usr/bin/env python3
"""RobStrideモーターのゼロ点フラグ(zero_sta)を設定するスクリプト.

zero_staフラグは起動時の位置範囲を決定します:
- 0: 位置範囲は 0 〜 2π（デフォルト）
- 1: 位置範囲は -π 〜 π

このフラグを変更すると、電源投入後のモーターの位置報告範囲が変わります。
例えば、flag=1に設定すると、モーターは-π〜πの範囲で位置を報告するようになります。

使用方法:
    # 現在のzero_sta値を表示
    uv run examples/set_zero_sta.py --motor-id 0x01

    # zero_staを1に設定（-π〜πモード）
    uv run examples/set_zero_sta.py --motor-id 0x01 --set 1

    # zero_staを0に設定（0〜2πモード、デフォルト）
    uv run examples/set_zero_sta.py --motor-id 0x01 --set 0

    # 保存せずに一時的に変更（電源再投入で元に戻る）
    uv run examples/set_zero_sta.py --motor-id 0x01 --set 1 --no-save
"""

import argparse

from robstride_motor import ActuatorType, RobStrideMotor


def main() -> int:
    """メイン関数."""
    parser = argparse.ArgumentParser(
        description="Get or set the zero position flag (zero_sta) of a RobStride motor"
    )
    parser.add_argument("--interface", default="can0", help="CAN interface (default: can0)")
    parser.add_argument(
        "--motor-id",
        type=lambda x: int(x, 0),
        default=0x01,
        help="Motor ID (default: 0x01)",
    )
    parser.add_argument(
        "--master-id",
        type=lambda x: int(x, 0),
        default=0xFF,
        help="Master ID (default: 0xFF)",
    )
    parser.add_argument(
        "--actuator-type",
        type=int,
        default=2,
        choices=range(7),
        help="Actuator type 0-6 (default: 2 for RS02)",
    )
    parser.add_argument(
        "--set",
        type=int,
        choices=[0, 1],
        dest="new_value",
        help="New zero_sta value to set (0 or 1)",
    )
    parser.add_argument(
        "--no-save",
        action="store_true",
        help="Don't save to flash (changes will be lost after power cycle)",
    )
    args = parser.parse_args()

    print(f"Connecting to motor ID {args.motor_id:#x} on {args.interface}...")

    try:
        motor = RobStrideMotor(
            can_interface=args.interface,
            master_id=args.master_id,
            motor_id=args.motor_id,
            actuator_type=ActuatorType(args.actuator_type),
        )

        # 現在の値を読み取る
        current_value = motor.get_zero_sta()

        if current_value is not None:
            mode_desc = "(0~2π mode)" if current_value == 0 else "(-π~π mode)"
            print(f"Current zero_sta: {current_value} {mode_desc}")
        else:
            print("Current zero_sta: Unable to read (parameter may not be supported)")

        # 新しい値を設定する場合
        if args.new_value is not None:
            new_mode_desc = "(0~2π mode)" if args.new_value == 0 else "(-π~π mode)"
            save_desc = "and saving to flash" if not args.no_save else "(not saving)"
            print(f"\nSetting zero_sta to {args.new_value} {new_mode_desc} {save_desc}...")

            success = motor.set_zero_sta(args.new_value, save=not args.no_save)

            if success:
                print("✓ Successfully set zero_sta")
                if not args.no_save:
                    print("  Setting has been saved to flash memory.")
                    print("  The new position range will take effect after power cycle.")
                else:
                    print("  Setting was NOT saved. Will revert after power cycle.")
            else:
                print("✗ Failed to set zero_sta")
                print("  The motor firmware may not support this parameter.")
                return 1

            # 設定後の値を確認
            final_value = motor.get_zero_sta()
            if final_value is not None:
                final_mode_desc = "(0~2π mode)" if final_value == 0 else "(-π~π mode)"
                print(f"\nFinal zero_sta: {final_value} {final_mode_desc}")

        return 0

    except KeyboardInterrupt:
        print("\nInterrupted by user")
        return 1
    except Exception as e:
        print(f"\nError: {e}")
        return 1


if __name__ == "__main__":
    exit(main())
