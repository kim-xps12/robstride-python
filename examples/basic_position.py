"""
Basic position control example for RobStride motor

Demonstrates position control using PP (Point-to-Point) mode.
"""

import can
import time
from robstride import RobStrideMotor, ProtocolMode

def main():
    # Initialize CAN interface
    print("Initializing CAN interface...")
    
    # Create motor instance
    motor = RobStrideMotor(
        can_id=0x01,
        can_interface='can0',
        protocol=ProtocolMode.PRIVATE,
        auto_enable=True
    )
    
    print(f"Motor initialized: {motor}")
    
    try:
        # Wait for motor to stabilize
        time.sleep(0.5)
        
        # Set zero position
        print("Setting zero position...")
        motor.set_zero_position()
        motor.enable_motor()
        time.sleep(0.5)
        
        # Move to different positions
        positions = [0.0, 1.57, 3.14, 1.57, 0.0]  # 0, 90, 180, 90, 0 degrees in radians
        
        for target_pos in positions:
            print(f"\nMoving to {target_pos:.2f} rad...")
            motor.position_control.set_pp_position(target_pos, target_speed=3.0)
            
            # Wait for movement to complete
            time.sleep(2.0)
            
            # Print current status
            print(f"Current angle: {motor.angle:.3f} rad")
            print(f"Current speed: {motor.speed:.3f} rad/s")
            print(f"Current torque: {motor.torque:.3f} Nm")
            print(f"Temperature: {motor.temperature:.1f}°C")
        
        print("\nPosition control test completed!")
        
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
