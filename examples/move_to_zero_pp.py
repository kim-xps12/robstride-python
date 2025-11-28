"""RS02モーターをゼロ点（位置0）まで回転させるサンプルコード.

このスクリプトは、RobStride RS02モーターを現在位置からゼロ点（0 rad）まで
位置制御モード（PP）で移動させます。

使用方法:
    python examples/move_to_zero.py

注意:
    - CANインターフェース（can0）が有効化されている必要があります
    - モーターIDはデフォルトで0x01に設定されています
"""

import time

from robstride_motor import ActuatorType, RobStrideMotor


def main() -> None:
    """RS02モーターをゼロ点まで回転させる."""
    # RS02モーターコントローラの初期化
    # ActuatorType.ROBSTRIDE_02 を使用してRS02モーターを指定
    motor = RobStrideMotor(
        can_interface="can0",
        master_id=0xFF,
        motor_id=0x7F,
        actuator_type=ActuatorType.ROBSTRIDE_02,
    )

    try:
        # モーターを有効化
        print("モーターを有効化中...")
        feedback = motor.enable_motor()
        print(f"現在位置: {feedback.position:.3f} rad")
        time.sleep(0.1)

        # 目標位置をゼロ点（0 rad）に設定
        target_position = 0.0  # rad
        speed = 5.0  # rad/s - 移動速度
        acceleration = 10.0  # rad/s² - 加速度

        print(f"\nゼロ点（{target_position} rad）へ移動中...")
        print(f"速度: {speed} rad/s, 加速度: {acceleration} rad/s²")

        # 位置制御モード（PP）でゼロ点へ移動
        motor.send_position_pp_command(
            angle=target_position,
            speed=speed,
            acceleration=acceleration,
        )

        # 目標位置に到達するまで待機（位置をモニタリング）
        tolerance = 0.05  # 許容誤差 (rad)
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
        print("完了")


if __name__ == "__main__":
    main()
