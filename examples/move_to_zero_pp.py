"""RS02モーターをゼロ点（位置0）まで回転させるサンプルコード.

このスクリプトは、RobStride RS02モーターを現在位置からゼロ点（0 rad）まで
位置制御モード（PP）で移動させます。

使用方法:
    # gs_usb (USB CANアダプタ、macOS/Linux)
    sudo uv run examples/move_to_zero_pp.py --interface gs_usb

    # socketcan (Linux)
    uv run examples/move_to_zero_pp.py --interface socketcan

    # チャンネル名直接指定 (socketcan)
    uv run examples/move_to_zero_pp.py --interface can0

注意:
    - モーターIDはデフォルトで127に設定されています
"""

import time
import argparse

from robstride_motor import ActuatorType, RobStrideMotor


def main() -> None:
    """RS02モーターをゼロ点まで回転させる."""
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='Move motor to zero position using PP mode'
    )
    parser.add_argument(
        '--interface',
        required=True,
        help="CAN interface: 'gs_usb', 'socketcan', or channel name (e.g., 'can0')",
    )
    parser.add_argument(
        '--motor-id',
        type=lambda x: int(x, 0),
        default=127,
        help='Motor ID (default: 127)',
    )
    parser.add_argument(
        '--master-id',
        type=lambda x: int(x, 0),
        default=255,
        help='Master ID (default: 255)',
    )
    parser.add_argument(
        '--actuator-type',
        type=int,
        default=2,
        choices=range(7),
        help='Actuator type 0-6 (default: 2 for RS02)',
    )
    parser.add_argument('--speed', type=float, default=5.0,
                        help='Movement speed in rad/s (default: 5.0)')
    parser.add_argument('--acceleration', type=float, default=10.0,
                        help='Movement acceleration in rad/s² (default: 10.0)')
    parser.add_argument('--tolerance', type=float, default=0.05,
                        help='Position tolerance in rad (default: 0.05)')
    args = parser.parse_args()
    
    # RS02モーターコントローラの初期化
    # ActuatorType.ROBSTRIDE_02 を使用してRS02モーターを指定
    motor = RobStrideMotor(
        can_interface=args.interface,
        master_id=args.master_id,
        motor_id=args.motor_id,
        actuator_type=ActuatorType(args.actuator_type),
    )

    try:
        # モーターを有効化
        print("モーターを有効化中...")
        feedback = motor.enable_motor()
        print(f"現在位置: {feedback.position:.3f} rad")
        time.sleep(0.1)

        # 目標位置をゼロ点（0 rad）に設定
        target_position = 0.0  # rad
        speed = args.speed  # rad/s - 移動速度
        acceleration = args.acceleration  # rad/s² - 加速度

        print(f"\nゼロ点（{target_position} rad）へ移動中...")
        print(f"速度: {speed} rad/s, 加速度: {acceleration} rad/s²")

        # 位置制御モード（PP）でゼロ点へ移動
        motor.send_position_pp_command(
            angle=target_position,
            speed=speed,
            acceleration=acceleration,
        )

        # 目標位置に到達するまで待機（位置をモニタリング）
        tolerance = args.tolerance  # 許容誤差 (rad)
        timeout = 10.0  # タイムアウト (秒)
        start_time = time.time()

        while True:
            # 現在位置を取得するために軽いコマンドを送信
            feedback = motor.send_position_pp_command(
                angle=target_position,
                speed=speed,
                acceleration=acceleration,
            )

            current_position = feedback.position
            position_error = abs(current_position - target_position)

            print(
                f"  位置: {current_position:+.3f} rad, "
                f"誤差: {position_error:.3f} rad, "
                f"温度: {feedback.temperature:.1f}°C"
            )

            # 目標位置に到達したかチェック
            if position_error < tolerance:
                print(f"\nゼロ点に到達しました！ (誤差: {position_error:.4f} rad)")
                break

            # タイムアウトチェック
            if time.time() - start_time > timeout:
                print(f"\nタイムアウト: {timeout}秒以内にゼロ点に到達できませんでした")
                break

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\nユーザーにより中断されました")
    except Exception as e:
        print(f"\nエラー: {e}")
    finally:
        # 終了時は必ずモーターを無効化
        print("\nモーターを無効化中...")
        motor.disable_motor()

        # CANバスを明示的にシャットダウン（gs_usbでは重要）
        if hasattr(motor, 'bus') and hasattr(motor, '_owns_bus') and motor._owns_bus:
            from robstride_motor.bus import shutdown_can_bus
            shutdown_can_bus(motor.bus, motor.can_interface)
        print("完了")


if __name__ == "__main__":
    main()
