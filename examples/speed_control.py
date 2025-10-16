"""
Speed control example for RobStride motor

Demonstrates speed control mode with varying speeds.
"""

import can
import time
from robstride import RobStrideMotor, ProtocolMode

def main():
    # Initialize motor
    print("Initializing motor...")
    motor = RobStrideMotor(
        can_id=0x01,
        can_interface='can0',
        protocol=ProtocolMode.PRIVATE,
        auto_enable=True
    )
    
    print(f"Motor initialized: {motor}")
    
    try:
        time.sleep(0.5)
        
        # Test different speeds
        speeds = [5.0, 10.0, 15.0, 10.0, 5.0, 0.0]  # rad/s
        
        for target_speed in speeds:
            print(f"\nSetting speed to {target_speed:.1f} rad/s...")
            motor.speed_control.set_speed(target_speed, current_limit=5.0)
            
            # Run for 3 seconds
            for i in range(3):
                time.sleep(1.0)
                print(f"  Speed: {motor.speed:.2f} rad/s, Torque: {motor.torque:.2f} Nm")
        
        print("\nSpeed control test completed!")
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    
    except Exception as e:
        print(f"\nError occurred: {e}")
    
    finally:
        # Stop motor
        print("\nStopping motor...")
        motor.speed_control.set_speed(0.0)
        time.sleep(0.5)
        motor.disable_motor()
        print("Motor stopped. Exiting.")

if __name__ == "__main__":
    main()
