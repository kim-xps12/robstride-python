#!/usr/bin/env python3
"""RobStrideモーターのCAN IDを変更するスクリプト.

モーターのCAN IDを新しい値に変更します。
変更後は新しいIDでモーターと通信できるようになります。

注意:
- --save オプションを使用しない場合、変更は電源オフで失われます
- 誤ったIDに変更すると、スキャンツールで再検出する必要があります

使用方法:
    # ID 0x7F を 0x01 に変更（一時的）
    uv run examples/set_motor_id.py --current-id 0x7F --new-id 0x01

    # ID 0x01 を 0x02 に変更（永続化）
    uv run examples/set_motor_id.py --current-id 0x01 --new-id 0x02 --save

    # CANインターフェースを指定
    uv run examples/set_motor_id.py --interface can1 --current-id 0x7F --new-id 0x10
"""

import argparse
import sys

from robstride_motor import ActuatorType, RobStrideMotor


def main() -> int:
    """メイン関数."""
    parser = argparse.ArgumentParser(
        description="Change RobStride motor CAN ID",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Change ID from 0x7F to 0x01 (temporary)
  %(prog)s --current-id 0x7F --new-id 0x01

  # Change ID from 0x01 to 0x02 and save to flash
  %(prog)s --current-id 0x01 --new-id 0x02 --save
        """,
    )
    parser.add_argument("--interface", default="can0", help="CAN interface (default: can0)")
    parser.add_argument(
        "--current-id",
        type=lambda x: int(x, 0),
        required=True,
        help="Current motor CAN ID (e.g., 0x7F or 127)",
    )
    parser.add_argument(
        "--new-id",
        type=lambda x: int(x, 0),
        required=True,
        help="New motor CAN ID (1-127)",
    )
    parser.add_argument(
        "--master-id",
        type=lambda x: int(x, 0),
        default=255,
        help="Master ID (default: 255)",
    )
    parser.add_argument(
        "--actuator-type",
        type=int,
        default=2,
        choices=range(7),
        help="Actuator type 0-6 (default: 2 for RS02)",
    )
    parser.add_argument(
        "--save",
        action="store_true",
        help="Save new ID to flash memory (persistent across power cycles)",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Skip confirmation prompt",
    )
    args = parser.parse_args()

    # コマンドライン引数から元の表記を取得
    import sys as sys_module
    current_id_str = str(args.current_id)
    new_id_str = str(args.new_id)
    
    for i, arg in enumerate(sys_module.argv):
        if arg == "--current-id" and i + 1 < len(sys_module.argv):
            current_id_str = sys_module.argv[i + 1]
        elif arg == "--new-id" and i + 1 < len(sys_module.argv):
            new_id_str = sys_module.argv[i + 1]

    # Validate new ID
    if not 1 <= args.new_id <= 127:
        print(f"Error: New ID must be between 1 and 127, got {args.new_id}")
        return 1

    if args.current_id == args.new_id:
        print(f"Error: Current ID and new ID are the same ({current_id_str})")
        return 1

    # Confirmation
    print(f"Motor ID Change")
    print(f"===============")
    print(f"  Interface  : {args.interface}")
    print(f"  Current ID : {current_id_str}")
    print(f"  New ID     : {new_id_str}")
    print(f"  Save       : {'Yes (persistent)' if args.save else 'No (temporary)'}")
    print()

    if not args.force:
        if args.save:
            print("WARNING: This will permanently change the motor ID!")
            print("         Make sure you remember the new ID.")
        response = input("Proceed? [y/N]: ").strip().lower()
        if response != "y":
            print("Cancelled.")
            return 0

    try:
        print(f"\nConnecting to motor at ID {current_id_str}...")
        motor = RobStrideMotor(
            can_interface=args.interface,
            master_id=args.master_id,
            motor_id=args.current_id,
            actuator_type=ActuatorType(args.actuator_type),
        )

        print(f"Changing ID from {current_id_str} to {new_id_str}...")
        success = motor.set_can_id(args.new_id, save=args.save)

        if success:
            print(f"\n[Success] Motor ID changed to {new_id_str}")
            if args.save:
                print("          Settings saved to flash memory.")
            else:
                print("          Note: Change is temporary and will be lost on power off.")
                print("          Use --save option to make it persistent.")
            return 0
        else:
            print(f"\n[Failed] Could not change motor ID")
            print("         Motor may not be responding at the specified ID.")
            print("         Try scanning for motors: uv run examples/scan_motors.py")
            return 1

    except KeyboardInterrupt:
        print("\nCancelled by user")
        return 1
    except ValueError as e:
        print(f"\nError: {e}")
        return 1
    except Exception as e:
        print(f"\nError: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
