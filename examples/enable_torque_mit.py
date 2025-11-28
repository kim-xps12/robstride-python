#!/usr/bin/env python3
"""
運控モード（Mode 0）を使用して現在位置でトルクオンし、位置を維持するサンプルコード.

kp, kdを調整して関節の硬さを変える実験などに使用できます。

Usage: run from repository root:
    uv run examples/enable_torque_mit.py
    uv run examples/enable_torque_mit.py --kp 100 --kd 2.0

This script will:
 - create RobStrideMotor(motor_id=0x7F)
 - enable motor
 - hold current position using motion control (Mode 0)
 - Ctrl+C to disable motor and exit
"""

import time
import math
import argparse

from robstride_motor import ActuatorType, RobStrideMotor


def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='Hold motor position using Mode 0 (MIT mode) with adjustable kp/kd'
    )
    parser.add_argument('--kp', type=float, default=200.0,
                        help='Position proportional gain (0.0-500.0, default: 200.0)')
    parser.add_argument('--kd', type=float, default=3.0,
                        help='Position derivative gain (0.0-5.0, default: 3.0)')
    parser.add_argument('--freq', type=float, default=100.0,
                        help='Control loop frequency in Hz (default: 100.0)')
    args = parser.parse_args()
    
    kp = args.kp
    kd = args.kd
    control_frequency = args.freq
    dt = 1.0 / control_frequency
    
    motor = None
    try:
        motor = RobStrideMotor(
            can_interface='can0',
            master_id=0xFF,
            motor_id=0x7F,
            actuator_type=ActuatorType.ROBSTRIDE_02,
        )
        
        print(f"Motor initialized: motor_id=0x{motor.motor_id:02X}")

        # Enable motor
        print("Enabling motor...")
        feedback = motor.enable_motor()
        time.sleep(0.1)

        # Get current position and hold it
        hold_position = feedback.position
        
        print(f"Holding position at {math.degrees(hold_position):+.2f}°")
        print(f"Control frequency: {control_frequency} Hz")
        print(f"Kp: {kp}, Kd: {kd}")
        print("Press Ctrl+C to stop...")
        
        loop_count = 0
        
        while True:
            # 制御コマンド送信（運控モード Mode 0）- 現在位置を維持
            feedback = motor.send_motion_command(
                torque=0.0,
                position=hold_position,
                velocity=0.0,
                kp=kp,
                kd=kd,
            )
            
            loop_count += 1
            
            # 10回に1回だけ状態表示
            if loop_count % 10 == 0:
                angle_deg = math.degrees(feedback.position)
                error_deg = math.degrees(hold_position - feedback.position)
                print(f"pos: {angle_deg:+.2f}°, error: {error_deg:+.3f}°, "
                      f"vel: {feedback.velocity:.2f} rad/s, torque: {feedback.torque:.3f} Nm")
            
            time.sleep(dt)

    except KeyboardInterrupt:
        print("\nStopping...")

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
