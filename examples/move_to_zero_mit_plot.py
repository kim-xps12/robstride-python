#!/usr/bin/env python3
"""
運控モード（Mode 0）を使用してモーターをゼロ点まで移動させるサンプルコード.

Usage: run from repository root:
    uv run examples/move_to_zero_mit_plot.py

This script will:
 - create RobStrideMotor(motor_id=0x7F)
 - enable motor
 - send motion control commands (Mode 0)
 - monitor feedback
 - always disable motor and close CAN on exit
 - plot current and target angles in real-time (using separate thread with FuncAnimation)
"""

import time
import math
import threading
import argparse
from collections import deque

from robstride_motor import ActuatorType, RobStrideMotor


class PlotterThread(threading.Thread):
    """別スレッドでmatplotlibアニメーションを実行するクラス."""
    
    def __init__(self, max_points: int = 500, update_interval: int = 50):
        """
        Args:
            max_points: 表示する最大データポイント数
            update_interval: アニメーション更新間隔 (ms)
        """
        super().__init__(daemon=True)
        self.max_points = max_points
        self.update_interval = update_interval
        
        # スレッドセーフなデータ共有用deque
        self.times = deque(maxlen=max_points)
        self.current_angles = deque(maxlen=max_points)
        self.target_angles = deque(maxlen=max_points)
        self.lock = threading.Lock()
        
        # 制御フラグ
        self.running = True
        self.control_finished = False
        
        # matplotlib関連（スレッド内で初期化）
        self.fig = None
        self.ax = None
        self.line_current = None
        self.line_target = None
        self.ani = None
    
    def add_data(self, t: float, current_angle: float, target_angle: float):
        """データを追加（メインスレッドから呼び出し）."""
        with self.lock:
            self.times.append(t)
            self.current_angles.append(current_angle)
            self.target_angles.append(target_angle)
    
    def stop(self):
        """描画スレッドを停止."""
        self.control_finished = True
    
    def close(self):
        """プロットウィンドウを閉じる."""
        self.running = False
    
    def run(self):
        """別スレッドでアニメーションを実行."""
        import matplotlib
        matplotlib.use('TkAgg')  # 非メインスレッド対応のバックエンド
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
        
        # プロットのセットアップ
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.line_current, = self.ax.plot([], [], 'b-', label='Current Angle', linewidth=2)
        self.line_target, = self.ax.plot([], [], 'r--', label='Target Angle', linewidth=2)
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Angle (deg)')
        self.ax.set_title('Motor Position Control (Real-time)')
        self.ax.legend()
        self.ax.grid(True)
        self.ax.set_xlim(0, 3)  # 初期X軸範囲
        self.ax.set_ylim(-180, 180)  # 初期Y軸範囲
        
        def init():
            """アニメーション初期化."""
            self.line_current.set_data([], [])
            self.line_target.set_data([], [])
            return self.line_current, self.line_target
        
        def update(frame):
            """アニメーション更新関数."""
            with self.lock:
                if len(self.times) > 0:
                    times_list = list(self.times)
                    current_list = list(self.current_angles)
                    target_list = list(self.target_angles)
                else:
                    return self.line_current, self.line_target
            
            self.line_current.set_data(times_list, current_list)
            self.line_target.set_data(times_list, target_list)
            
            # 軸の自動調整
            if times_list:
                self.ax.set_xlim(0, max(times_list[-1] * 1.1, 0.5))
                all_angles = current_list + target_list
                if all_angles:
                    min_angle = min(all_angles) - 10
                    max_angle = max(all_angles) + 10
                    self.ax.set_ylim(min_angle, max_angle)
            
            return self.line_current, self.line_target
        
        # FuncAnimationでアニメーション作成
        self.ani = FuncAnimation(
            self.fig,
            update,
            init_func=init,
            interval=self.update_interval,
            blit=False,
            cache_frame_data=False
        )
        
        # 制御終了後も表示を維持
        plt.show(block=True)


def main():
    # コマンドライン引数のパース
    parser = argparse.ArgumentParser(
        description='Move motor to zero position using Mode 0 (MIT mode) with real-time plotting'
    )
    parser.add_argument('--interface', default='can0', help='CAN interface (default: can0)')
    parser.add_argument('--motor-id', type=lambda x: int(x, 0), default=127,
                        help='Motor ID (default: 127)')
    parser.add_argument('--master-id', type=lambda x: int(x, 0), default=255,
                        help='Master ID (default: 255)')
    parser.add_argument('--actuator-type', type=int, default=2, choices=range(7),
                        help='Actuator type 0-6 (default: 2 for RS02)')
    parser.add_argument('--kp', type=float, default=200.0,
                        help='Position proportional gain (default: 200.0)')
    parser.add_argument('--kd', type=float, default=3.0,
                        help='Position derivative gain (default: 3.0)')
    parser.add_argument('--duration', type=float, default=2.0,
                        help='Movement duration in seconds (default: 2.0)')
    parser.add_argument('--freq', type=float, default=100.0,
                        help='Control loop frequency in Hz (default: 100.0)')
    args = parser.parse_args()
    
    # プロットスレッドの初期化と開始
    plotter = PlotterThread(max_points=1000, update_interval=50)
    plotter.start()
    time.sleep(0.5)  # プロットウィンドウが開くまで少し待機
    
    motor = None
    try:
        motor = RobStrideMotor(
            can_interface=args.interface,
            master_id=args.master_id,
            motor_id=args.motor_id,
            actuator_type=ActuatorType(args.actuator_type),
        )
        
        print(f"Motor initialized: motor_id={args.motor_id}")

        # Enable motor
        print("Enabling motor...")
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
        kd = args.kd   # 位置微分ゲイン (0.0〜5.0)
        control_frequency = args.freq  # Hz - 制御ループの周波数
        dt = 1.0 / control_frequency
        
        # Spin-wait threshold: スリープ残り時間がこの値以下になったらビジーウェイトする
        # 0.5ms に設定して、sleep の早起き分も考慮
        spin_threshold = 0.0005  # 0.5ms
        
        print(f"Moving from {math.degrees(initial_angle):+.2f}° to {math.degrees(target_angle):+.2f}° over {duration:.1f} seconds...")
        print(f"Control loop frequency: {control_frequency} Hz (period: {dt*1000:.2f}ms)")
        print(f"Kp: {kp}, Kd: {kd}")
        
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
                print(f"Final: process: {process_time_ms:.3f}ms, id: 0x{motor.motor_id:02X}, pos: {angle_deg:.2f} deg")
                
                # Update plot data
                if plotter:
                    plotter.add_data(elapsed, angle_deg, math.degrees(target_angle))

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

                # Update plot data (別スレッドのプロッターにデータを渡す)
                if plotter:
                    plotter.add_data(elapsed, angle_deg, math.degrees(interpolated_angle))
            
            # 次の制御タイミングを更新
            next_control_time += dt

        print("Done")
        
        # プロット終了を通知（ウィンドウは開いたまま）
        if plotter:
            plotter.stop()
            print("Press Ctrl+C or close the plot window to exit...")
            # プロットウィンドウが閉じられるまで待機
            try:
                while plotter.is_alive():
                    time.sleep(0.1)
            except KeyboardInterrupt:
                pass

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
