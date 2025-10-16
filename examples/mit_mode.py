"""
MIT protocol control example for RobStride motor

Demonstrates MIT protocol composite control.
"""

import can
import time
import math
from robstride import RobStrideMotor, ProtocolMode

def main():
    # Initialize motor in MIT mode
    print("Initializing motor in MIT protocol...")
    motor = RobStrideMotor(
        can_id=0x01,
        can_interface='can0',
        protocol=ProtocolMode.MIT,
        auto_enable=True
    )
    
    print(f"Motor initialized: {motor}")
    
    try:
        time.sleep(0.5)
        
        # MIT composite control with PD gains
        print("\nStarting MIT composite control...")
        
        # Sinusoidal position tracking
        duration = 10.0  # seconds
        frequency = 0.5  # Hz
        amplitude = 1.0  # rad
        
        start_time = time.time()
        
        while time.time() - start_time < duration:
            t = time.time() - start_time
            
            # Generate sinusoidal reference
            target_pos = amplitude * math.sin(2 * math.pi * frequency * t)
            target_vel = amplitude * 2 * math.pi * frequency * math.cos(2 * math.pi * frequency * t)
            
            # Send MIT control command with PD gains
            motor.send_mit_control(
                position=target_pos,
                velocity=target_vel,
                kp=50.0,
                kd=1.0,
                torque=0.0
            )
            
            # Print status every 0.5 seconds
            if int(t * 2) != int((t - 0.05) * 2):
                print(f"t={t:.1f}s: target={target_pos:.3f}, actual={motor.angle:.3f}, "
                      f"speed={motor.speed:.2f}, torque={motor.torque:.2f}")
            
            time.sleep(0.05)  # 20 Hz control loop
        
        print("\nMIT control test completed!")
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    
    except Exception as e:
        print(f"\nError occurred: {e}")
    
    finally:
        # Disable motor
        print("\nDisabling motor...")
        motor.disable_motor()
        print("Motor disabled. Exiting.")

if __name__ == "__main__":
    main()
