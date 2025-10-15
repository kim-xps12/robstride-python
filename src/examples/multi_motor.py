"""
Multi-motor control example for RobStride motors

Demonstrates coordinated control of multiple motors.
"""

import can
import time
from robstride_old import RobStrideMotor, ProtocolMode

def main():
    # Initialize CAN bus
    print("Initializing CAN interface...")
    
    # Create multiple motor instances
    motors = {
        'motor1': RobStrideMotor(can_id=0x01, can_interface='can0', auto_enable=True),
        'motor2': RobStrideMotor(can_id=0x02, can_interface='can0', auto_enable=True),
        'motor3': RobStrideMotor(can_id=0x03, can_interface='can0', auto_enable=True),
    }
    
    print(f"Initialized {len(motors)} motors")
    
    try:
        time.sleep(0.5)
        
        # Coordinated position control
        print("\nPerforming coordinated position control...")
        
        # Move all motors to home position
        print("Moving to home position...")
        for name, motor in motors.items():
            motor.position_control.set_pp_position(0.0, target_speed=3.0)
        time.sleep(2.0)
        
        # Synchronized movement
        target_positions = [1.57, 3.14, 1.57]  # Different positions for each motor
        
        print("Moving to target positions...")
        for i, (name, motor) in enumerate(motors.items()):
            motor.position_control.set_pp_position(target_positions[i], target_speed=3.0)
        
        time.sleep(3.0)
        
        # Print status of all motors
        print("\nMotor Status:")
        for name, motor in motors.items():
            print(f"  {name}: angle={motor.angle:.3f} rad, "
                  f"speed={motor.speed:.2f} rad/s, "
                  f"temp={motor.temperature:.1f}°C")
        
        print("\nMulti-motor control test completed!")
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    
    except Exception as e:
        print(f"\nError occurred: {e}")
    
    finally:
        # Disable all motors
        print("\nDisabling all motors...")
        for name, motor in motors.items():
            motor.disable_motor()
        print("All motors disabled. Exiting.")

if __name__ == "__main__":
    main()
