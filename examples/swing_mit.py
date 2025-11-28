#!/usr/bin/env python3
"""
運控モード（Mode 0）を使用してモーターを正弦波状に揺動させるサンプルコード.

Usage: run from repository root:
    uv run examples/swing_mit.py

This script will:
 - create RobStrideMotor(motor_id=0x7F)
 - enable motor
 - send sinusoidal position commands (Mode 0)
 - monitor feedback
 - always disable motor and close CAN on exit
 - plot current and target angles in real-time (using separate thread with FuncAnimation)
"""

import time
import math
import threading
from collections import deque

from robstride_motor import ActuatorType, RobStrideMotor


# ==============================================================================
# 正弦波パラメータ（変更可能）
# ==============================================================================
SWING_AMPLITUDE_DEG = 90.0  # 振幅 [deg] (±90°)
SWING_FREQUENCY_HZ = 2.0    # 周波数 [Hz]
SWING_DURATION_SEC = 5.0   # 動作時間 [sec]


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
        self.ax.set_title('Sinusoidal Swing Control (Real-time)')
        self.ax.legend()
        self.ax.grid(True)
        self.ax.set_xlim(0, 3)  # 初期X軸範囲
        self.ax.set_ylim(-90, 90)  # 初期Y軸範囲（振幅に合わせて調整）
        
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
    # 正弦波パラメータをラジアンに変換
    amplitude_rad = math.radians(SWING_AMPLITUDE_DEG)
    angular_frequency = 2.0 * math.pi * SWING_FREQUENCY_HZ  # ω = 2πf
    
    print("=" * 60)
    print("Sinusoidal Swing Control")
    print("=" * 60)
    print(f"Amplitude: ±{SWING_AMPLITUDE_DEG}° (±{amplitude_rad:.4f} rad)")
    print(f"Frequency: {SWING_FREQUENCY_HZ} Hz")
    print(f"Duration:  {SWING_DURATION_SEC} sec")
    print("=" * 60)
    
    # プロットスレッドの初期化と開始
    plotter = PlotterThread(max_points=2000, update_interval=50)
    plotter.start()
    time.sleep(0.5)  # プロットウィンドウが開くまで少し待機
    
    motor = None
    try:
        motor = RobStrideMotor(
            can_interface='can0',
            master_id=0xFF,
            motor_id=0x7F,
            actuator_type=ActuatorType.ROBSTRIDE_05,
        )
        
        print(f"Motor initialized: motor_id=0x{motor.motor_id:02X}")

        # Enable motor
        print("Enabling motor...")
        feedback = motor.enable_motor()
        time.sleep(0.1)

        # Movement parameters
        kp = 200.0  # 位置比例ゲイン (0.0〜500.0)
        kd = 3.0    # 位置微分ゲイン (0.0〜5.0)
        control_frequency = 100.0  # Hz - 制御ループの周波数
        dt = 1.0 / control_frequency
        
        # Spin-wait threshold: スリープ残り時間がこの値以下になったらビジーウェイトする
        spin_threshold = 0.0005  # 0.5ms
        
        print(f"Control loop frequency: {control_frequency} Hz (period: {dt*1000:.2f}ms)")
        print(f"Kp: {kp}, Kd: {kd}")
        print("Starting sinusoidal swing...")
        
        # High-precision control loop using perf_counter
        start_time = time.perf_counter()
        next_control_time = start_time
        prev_loop_start = start_time
        loop_count = 0
        
        # 周期精度の統計用
        period_errors = []
        actual_periods = []
        
        # 次の制御タイミングを初期化
        next_control_time = start_time + dt
        
        while True:
            loop_start_time = time.perf_counter()
            current_time = loop_start_time
            elapsed = current_time - start_time
            
            # 経過時間が目標時間を超えたら終了
            if elapsed >= SWING_DURATION_SEC:
                # 最後に目標位置へ送信（最終位置で停止）
                target_angle = amplitude_rad * math.sin(angular_frequency * elapsed)
                feedback = motor.send_motion_command(
                    torque=0.0,
                    position=target_angle,
                    velocity=0.0,
                    kp=kp,
                    kd=kd,
                )
                angle_deg = math.degrees(feedback.position)
                target_angle_deg = math.degrees(target_angle)
                process_time_ms = (time.perf_counter() - loop_start_time) * 1000.0
                print(f"Final: process: {process_time_ms:.3f}ms, id: 0x{motor.motor_id:02X}, pos: {angle_deg:.2f}°")
                
                # Update plot data
                if plotter:
                    plotter.add_data(elapsed, angle_deg, target_angle_deg)

                # 周期精度の統計を表示
                if period_errors:
                    avg_error = sum(period_errors) / len(period_errors)
                    max_error = max(period_errors)
                    min_error = min(period_errors)

                    avg_period = sum(actual_periods) / len(actual_periods)
                    max_period = max(actual_periods)
                    min_period = min(actual_periods)
                    jitter = max_period - min_period

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

            # 正弦波で目標位置を計算: θ(t) = A * sin(ωt)
            target_angle = amplitude_rad * math.sin(angular_frequency * elapsed)

            # 制御コマンド送信（運控モード Mode 0）
            feedback = motor.send_motion_command(
                torque=0.0,
                position=target_angle,
                velocity=0.0,
                kp=kp,
                kd=kd,
            )
            
            # 現在の状態を取得
            angle_deg = math.degrees(feedback.position)
            target_angle_deg = math.degrees(target_angle)
            process_time_ms = (time.perf_counter() - loop_start_time) * 1000.0
            
            loop_count += 1
            
            # 各ループの実際の周期を記録（2ループ目以降）
            if loop_count > 1:
                actual_period = loop_start_time - prev_loop_start
                actual_periods.append(actual_period)
                period_errors.append(abs(actual_period - dt))
            
            prev_loop_start = loop_start_time
            
            # 次の制御タイミングまで待機（高精度スリープ + スピンウェイト）
            sleep_time = next_control_time - time.perf_counter()
            wait_start = time.perf_counter()
            
            if sleep_time > 0:
                if sleep_time > spin_threshold:
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
            
            actual_wait_time = time.perf_counter() - wait_start
            
            # 表示は10回に1回だけ実行（制御周期への影響を最小化）
            if loop_count % 10 == 0:
                if len(actual_periods) > 0:
                    last_period_ms = actual_periods[-1] * 1000.0
                    period_error_ms = abs(last_period_ms - dt * 1000.0)
                    wait_time_ms = actual_wait_time * 1000.0
                    
                    print(f"process: {process_time_ms:.3f}ms, wait: {wait_time_ms:.3f}ms, period: {last_period_ms:.3f}ms, "
                          f"error: {period_error_ms:.3f}ms, pos: {angle_deg:.2f}°, target: {target_angle_deg:.2f}°")

                # Update plot data
                if plotter:
                    plotter.add_data(elapsed, angle_deg, target_angle_deg)
            
            next_control_time += dt

        print("Done")
        
        # プロット終了を通知（ウィンドウは開いたまま）
        if plotter:
            plotter.stop()
            print("Press Ctrl+C or close the plot window to exit...")
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
