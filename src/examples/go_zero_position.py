#!/usr/bin/env python3
"""
Move motor to 0 rad using robstride library (Private protocol, PP mode)

Usage: run from repository root:
    uv run python src/examples/go_zero_position.py
    uv run python src/examples/go_zero_position.py --plot  # with real-time plot

This script will:
 - create RobStrideMotor(can_id=0x7F)
 - enable motor
 - enable auto-report
 - set zero position and send PP position=0.0
 - monitor a few auto-reports
 - always disable motor and close CAN on exit
 - optionally plot current and target angles in real-time
"""

import time
import math
import logging
import argparse

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from robstride import RobStrideMotor, ProtocolMode

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Move motor to zero position with optional real-time plotting')
    parser.add_argument('--plot', action='store_true', help='Enable real-time plotting of angles')
    args = parser.parse_args()
    
    # Import matplotlib only if plotting is enabled
    if args.plot:

        
        # Setup plot
        plt.ion()  # Interactive mode
        fig, ax = plt.subplots(figsize=(10, 6))
        times = []
        current_angles = []
        target_angles = []
        
        line_current, = ax.plot([], [], 'b-', label='Current Angle', linewidth=2)
        line_target, = ax.plot([], [], 'r--', label='Target Angle', linewidth=2)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angle (deg)')
        ax.set_title('Motor Position Control')
        ax.legend()
        ax.grid(True)
    
    motor = None
    try:
        motor = RobStrideMotor(can_id=0x7F, can_interface='can0', protocol=ProtocolMode.PRIVATE, auto_enable=False)
        
        # Disable DEBUG logging after motor initialization
        robstride_logger = logging.getLogger('RobStride')
        robstride_logger.setLevel(logging.WARNING)
        for handler in robstride_logger.handlers:
            handler.setLevel(logging.WARNING)
        
        print(f"Motor initialized: {motor}")

        # Enable motor
        print("Enabling motor...")
        motor.enable_motor()
        time.sleep(0.1)

        # Enable auto-report for status updates
        print("Enabling auto-report...")
        motor.set_auto_report(True)
        time.sleep(0.1)

        # NOTE: do NOT call set_zero_position() here — it overwrites the encoder zero to the
        # current physical position. For motion-control to absolute 0 rad, do not reset zero.
        # If you explicitly want to redefine the zero, call motor.set_zero_position() manually.

        # Get initial position
        initial_angle = motor.status.angle
        target_angle = 0.0  # rad
        
        # Movement parameters
        duration = 2.0  # seconds - 目標位置まで移動する時間
        control_frequency = 100.0  # Hz - 制御ループの周波数
        dt = 1.0 / control_frequency
        
        # Spin-wait threshold: スリープ残り時間がこの値以下になったらビジーウェイトする
        # 0.5ms に設定して、sleep の早起き分も考慮
        spin_threshold = 0.0005  # 0.5ms
        
        print(f"Moving from {math.degrees(initial_angle):+.2f}° to {math.degrees(target_angle):+.2f}° over {duration:.1f} seconds...")
        print(f"Control loop frequency: {control_frequency} Hz (period: {dt*1000:.2f}ms)")
        
        # High-precision control loop using perf_counter
        start_time = time.perf_counter()
        next_control_time = start_time
        prev_loop_start = start_time
        loop_count = 0
        
        # 周期精度の統計用
        period_errors = []
        actual_periods = []  # 各ループの実際の周期を記録
        
        # 次の制御タイミングを初期化
        next_control_time = start_time + dt
        
        while True:
            loop_start_time = time.perf_counter()
            current_time = loop_start_time
            elapsed = current_time - start_time
            
            # 経過時間が目標時間を超えたら終了
            if elapsed >= duration:
                # 最後に目標位置へ送信
                motor.send_motion_control(torque=0.0, angle=target_angle, speed=0.0, kp=50.0, kd=1.0)
                status = motor.status
                angle_deg = math.degrees(status.angle)
                process_time_ms = (time.perf_counter() - loop_start_time) * 1000.0
                print(f"Final: process: {process_time_ms:.3f}ms, id: {motor.motor_id}, pos: {angle_deg:.2f} deg")
                
                # Update plot data
                if args.plot:
                    times.append(elapsed)
                    current_angles.append(angle_deg)
                    target_angles.append(math.degrees(target_angle))
                
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
                    variance = sum((p - avg_period) ** 2 for p in actual_periods) / len(actual_periods)
                    std_dev = variance ** 0.5
                    
                    print(f"\n制御周期精度統計:")
                    print(f"  目標周期: {dt*1000:.3f}ms")
                    print(f"  実測平均周期: {avg_period*1000:.3f}ms")
                    print(f"  周期範囲: {min_period*1000:.3f}ms ~ {max_period*1000:.3f}ms")
                    print(f"  ジッター: {jitter*1000:.3f}ms")
                    print(f"  標準偏差: {std_dev*1000:.3f}ms")
                    print(f"  平均誤差: {avg_error*1000:.3f}ms")
                    print(f"  誤差範囲: {min_error*1000:.3f}ms ~ {max_error*1000:.3f}ms")
                
                break
            
            # 線形補間で中間目標位置を計算
            progress = elapsed / duration
            interpolated_angle = initial_angle + (target_angle - initial_angle) * progress
            
            # 制御コマンド送信
            motor.send_motion_control(torque=0.0, angle=interpolated_angle, speed=0.0, kp=50.0, kd=1.0)
            
            # 現在の状態を取得
            status = motor.status
            angle_deg = math.degrees(status.angle)
            process_time_ms = (time.perf_counter() - loop_start_time) * 1000.0
            
            # 表示とプロット更新は10回に1回だけ実行（制御周期への影響を最小化）
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
                    
                    print(f"process: {process_time_ms:.3f}ms, wait: {wait_time_ms:.3f}ms, period: {last_period_ms:.3f}ms, "
                          f"error: {period_error_ms:.3f}ms, pos: {angle_deg:.2f}°")

                # Update plot data
                if args.plot:
                    times.append(elapsed)
                    current_angles.append(angle_deg)
                    target_angles.append(math.degrees(target_angle))
                    
                    # Update plot every 100ms to avoid slowing down control loop
                    line_current.set_data(times, current_angles)
                    line_target.set_data(times, target_angles)
                    ax.relim()
                    ax.autoscale_view()
                    plt.pause(0.001)
            
            # 次の制御タイミングを更新
            next_control_time += dt

        print("Done")
        
        # Keep plot window open if plotting was enabled
        if args.plot:
            plt.ioff()
            plt.show()

    except Exception as e:
        print(f"Error: {e}")

    finally:
        if motor is not None:
            try:
                motor.disable_motor()
                print("Motor disabled")
            except Exception:
                pass


if __name__ == '__main__':
    main()
