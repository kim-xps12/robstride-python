#!/usr/bin/env python3
"""
運控モード（Mode 0）を使用してモーターをゼロ点まで移動させるサンプルコード.

robstride_motor ライブラリを使用したバージョン。
バックエンド（gs_usb / socketcan）を明示的に指定する必要があります。

使用方法:
gs_usb (USB CANアダプタ、macOS/Linux):
    sudo uv run examples/move_to_zero_mit_lib.py --backend gs_usb
    sudo uv run examples/move_to_zero_mit_lib.py --backend gs_usb --motor-id 127

socketcan (Linux):
    sudo ip link set can0 up type can bitrate 1000000
    uv run examples/move_to_zero_mit_lib.py --backend socketcan
    uv run examples/move_to_zero_mit_lib.py --backend socketcan --channel can0

This script will:
 - create RobStrideMotor with specified backend
 - enable motor
 - send motion control commands (Mode 0)
 - monitor feedback
 - always disable motor and close CAN on exit
"""

import argparse
import math
import time

from robstride_motor import ActuatorType, RobStrideMotor


def main() -> int:
    """メイン関数."""
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='Move motor to zero position using Mode 0 (MIT mode)'
    )
    parser.add_argument(
        '--backend',
        type=str,
        required=True,
        choices=['gs_usb', 'socketcan'],
        help="CAN backend: 'gs_usb' (USB CAN adapter) or 'socketcan' (Linux)",
    )
    parser.add_argument(
        '--channel',
        default='can0',
        help='CAN channel for socketcan (default: can0)',
    )
    parser.add_argument(
        '--bitrate',
        type=int,
        default=1000000,
        help='CAN bitrate in bps (default: 1000000)',
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
    parser.add_argument(
        '--kp',
        type=float,
        default=200.0,
        help='Position proportional gain (default: 200.0)',
    )
    parser.add_argument(
        '--kd',
        type=float,
        default=3.0,
        help='Position derivative gain (default: 3.0)',
    )
    parser.add_argument(
        '--duration',
        type=float,
        default=2.0,
        help='Movement duration in seconds (default: 2.0)',
    )
    parser.add_argument(
        '--freq',
        type=float,
        default=100.0,
        help='Control loop frequency in Hz (default: 100.0)',
    )
    args = parser.parse_args()

    # バックエンドに応じてcan_interfaceを設定
    if args.backend == 'gs_usb':
        can_interface = 'gs_usb'
    else:
        # socketcanの場合はチャンネルを使用
        can_interface = args.channel

    print(f'CANバックエンド: {args.backend}')
    if args.backend == 'socketcan':
        print(f'CANチャンネル: {args.channel}')
    print(f'ビットレート: {args.bitrate} bps')
    print(f'モーターID: {args.motor_id}')
    print(f'マスターID: {args.master_id}')
    print('-' * 40)

    try:
        with RobStrideMotor(
            can_interface=can_interface,
            master_id=args.master_id,
            motor_id=args.motor_id,
            actuator_type=ActuatorType(args.actuator_type),
            bitrate=args.bitrate,
        ) as motor:
            print(f'Motor initialized: motor_id={args.motor_id}')

            # Enable motor
            print('Enabling motor...')
            feedback = motor.enable_motor()
            time.sleep(0.1)

            # NOTE: do NOT call set_zero_position() here — it overwrites the encoder zero to the
            # current physical position. For motion-control to absolute 0 rad, do not reset zero.
            # If you explicitly want to redefine the zero, call motor.set_zero_position() manually.

            # Get initial position
            initial_angle = feedback.position
            target_angle = 0.0  # rad (0°に戻す)

            # Movement parameters
            duration = args.duration  # seconds - 目標位置まで移動する時間
            kp = args.kp  # 位置比例ゲイン (0.0〜500.0)
            kd = args.kd  # 位置微分ゲイン (0.0〜5.0)
            control_frequency = args.freq  # Hz - 制御ループの周波数
            dt = 1.0 / control_frequency

            # Spin-wait threshold: スリープ残り時間がこの値以下になったらビジーウェイトする
            # 0.5ms に設定して、sleep の早起き分も考慮
            spin_threshold = 0.0005  # 0.5ms

            print(
                f'Moving from {math.degrees(initial_angle):+.2f}° to '
                f'{math.degrees(target_angle):+.2f}° over {duration:.1f} seconds...'
            )
            print(f'Control loop frequency: {control_frequency} Hz (period: {dt*1000:.2f}ms)')
            print(f'Kp: {kp}, Kd: {kd}')

            # High-precision control loop using perf_counter
            start_time = time.perf_counter()
            next_control_time = start_time
            prev_loop_start = start_time
            loop_count = 0

            # 周期精度の統計用
            period_errors: list[float] = []
            actual_periods: list[float] = []  # 各ループの実際の周期を記録

            # 次の制御タイミングを初期化
            next_control_time = start_time + dt

            while True:
                loop_start_time = time.perf_counter()
                current_time = loop_start_time
                elapsed = current_time - start_time

                # 経過時間が目標時間を超えたら終了
                if elapsed >= duration:
                    # 最後に目標位置へ送信（運控モード Mode 0）
                    feedback = motor.send_motion_command(
                        torque=0.0,
                        position=target_angle,
                        velocity=0.0,
                        kp=kp,
                        kd=kd,
                    )
                    angle_deg = math.degrees(feedback.position)
                    process_time_ms = (time.perf_counter() - loop_start_time) * 1000.0
                    print(
                        f'Final: process: {process_time_ms:.3f}ms, '
                        f'id: 0x{motor.motor_id:02X}, pos: {angle_deg:.2f} deg'
                    )

                    # 周期精度の統計を表示
                    if period_errors:
                        avg_error = sum(period_errors) / len(period_errors)
                        max_error = max(period_errors)
                        min_error = min(period_errors)

                        # 実際の周期統計
                        avg_period = sum(actual_periods) / len(actual_periods)
                        max_period = max(actual_periods)
                        min_period = min(actual_periods)
                        jitter = max_period - min_period

                        # 標準偏差を計算
                        variance = sum((p - avg_period) ** 2 for p in actual_periods) / len(
                            actual_periods
                        )
                        std_dev = variance**0.5

                        print('\n制御周期精度統計:')
                        print(f'  目標周期: {dt*1000:.3f}ms')
                        print(f'  実測平均周期: {avg_period*1000:.3f}ms')
                        print(f'  周期範囲: {min_period*1000:.3f}ms ~ {max_period*1000:.3f}ms')
                        print(f'  ジッター: {jitter*1000:.3f}ms')
                        print(f'  標準偏差: {std_dev*1000:.3f}ms')
                        print(f'  平均誤差: {avg_error*1000:.3f}ms')
                        print(f'  誤差範囲: {min_error*1000:.3f}ms ~ {max_error*1000:.3f}ms')

                    break

                # 線形補間で中間目標位置を計算
                progress = elapsed / duration
                interpolated_angle = initial_angle + (target_angle - initial_angle) * progress

                # 制御コマンド送信（運控モード Mode 0）
                feedback = motor.send_motion_command(
                    torque=0.0,
                    position=interpolated_angle,
                    velocity=0.0,
                    kp=kp,
                    kd=kd,
                )

                # 現在の状態を取得
                angle_deg = math.degrees(feedback.position)
                process_time_ms = (time.perf_counter() - loop_start_time) * 1000.0

                # 表示は10回に1回だけ実行（制御周期への影響を最小化）
                loop_count += 1

                # 各ループの実際の周期を記録（2ループ目以降）
                if loop_count > 1:
                    actual_period = loop_start_time - prev_loop_start
                    actual_periods.append(actual_period)
                    period_errors.append(abs(actual_period - dt))

                # 周期測定のために前回のループ開始時刻を記録
                prev_loop_start = loop_start_time

                # 次の制御タイミングまで待機（高精度スリープ + スピンウェイト）
                sleep_time = next_control_time - time.perf_counter()
                wait_start = time.perf_counter()

                if sleep_time > 0:
                    # スリープ時間がspin_thresholdより大きい場合は通常のsleepを使う
                    # ただし、sleep_time自体を使わず、より保守的にスリープする
                    if sleep_time > spin_threshold:
                        # 安全マージンを持ってスリープ（早起き対策）
                        safe_sleep = max(0, sleep_time - spin_threshold)
                        if safe_sleep > 0:
                            time.sleep(safe_sleep)

                    # 残り時間はビジーウェイトで高精度に待機
                    while time.perf_counter() < next_control_time:
                        pass
                else:
                    # 制御周期を逃した場合は次の周期に同期
                    missed_cycles = int(-sleep_time / dt) + 1
                    next_control_time += missed_cycles * dt
                    continue

                # 実際の待機時間を記録
                actual_wait_time = time.perf_counter() - wait_start

                # 表示は10回に1回だけ実行（制御周期への影響を最小化）
                if loop_count % 10 == 0:
                    # 直前のループの周期を表示
                    if len(actual_periods) > 0:
                        last_period_ms = actual_periods[-1] * 1000.0
                        period_error_ms = abs(last_period_ms - dt * 1000.0)
                        wait_time_ms = actual_wait_time * 1000.0

                        print(
                            f'process: {process_time_ms:.3f}ms, wait: {wait_time_ms:.3f}ms, '
                            f'period: {last_period_ms:.3f}ms, error: {period_error_ms:.3f}ms, '
                            f'pos: {angle_deg:.2f}°'
                        )

                # 次の制御タイミングを更新
                next_control_time += dt

            print('Done')

            # モーターを無効化（with ブロック内で CAN バスがまだ有効な間に実行）
            motor.disable_motor()
            time.sleep(0.1)
            print('Motor disabled')

    except Exception as e:
        print(f'Error: {e}')
        return 1

    print('CAN bus closed')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
