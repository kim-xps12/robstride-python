"""
Main motor control class for RobStride RS02

Provides high-level interface for motor control using Private or MIT protocol.
"""

import can
import time
import threading
from typing import Optional, Callable
from .models import (
    MotorStatus, ParameterData, MotionControlCommand, MITCommand,
    ControlMode, ProtocolMode, MotorState, ParameterIndex, validate_parameter
)
from .protocol.private import PrivateProtocolHandler
from .protocol.mit import MITProtocolHandler
from .utils import ErrorHandler, MotorException, CANException, ErrorLogger, validate_can_id
from .control import PositionController, SpeedController, CurrentController


class RobStrideMotor:
    """
    Main interface for RobStride RS02 motor control
    
    Supports both Private and MIT protocols for comprehensive motor control.
    """
    
    def __init__(
        self,
        can_id: int,
        can_interface: str = 'can0',
        protocol: ProtocolMode = ProtocolMode.PRIVATE,
        master_id: int = 0xFD,
        auto_enable: bool = False
    ):
        """
        Initialize RobStride motor controller
        
        Args:
            can_id: Motor CAN ID (0x00-0x7F)
            can_interface: CAN interface name (e.g., 'can0', 'vcan0')
            protocol: Protocol mode (PRIVATE or MIT)
            master_id: Master/host CAN ID (default: 0xFD)
            auto_enable: Automatically enable motor on init
            
        Raises:
            ValueError: If CAN ID is invalid
            CANException: If CAN interface initialization fails
        """
        # Validate CAN ID
        validate_can_id(can_id)
        
        self.motor_id = can_id
        self.protocol_mode = protocol
        self.master_id = master_id
        
        # Initialize CAN bus
        try:
            self.can_bus = can.interface.Bus(
                channel=can_interface,
                bustype='socketcan',
                bitrate=1000000  # 1 Mbps
            )
        except Exception as e:
            raise CANException(f"Failed to initialize CAN interface: {e}")
        
        # Internal state
        self.state = MotorState.DISABLED
        self.status = MotorStatus()
        self.param_data = ParameterData()
        
        # Protocol handlers
        self.private_handler = PrivateProtocolHandler(can_id, self.can_bus, master_id)
        self.mit_handler = MITProtocolHandler(can_id, self.can_bus)
        
        # Control strategies
        self.position_control = PositionController(self)
        self.speed_control = SpeedController(self)
        self.current_control = CurrentController(self)
        
        # Error handling and logging
        self.error_handler = ErrorHandler(self)
        self.logger = ErrorLogger()
        
        # Status callback
        self.status_callback: Optional[Callable[[MotorStatus], None]] = None
        
        # Start CAN listener
        self._running = True
        self._listener_thread = threading.Thread(target=self._can_listener, daemon=True)
        self._listener_thread.start()
        
        # Auto-enable if requested
        if auto_enable:
            time.sleep(0.1)
            self.enable_motor()
        
        self.logger.log_info(self.motor_id, f"Motor initialized in {protocol.name} protocol mode")
    
    def __del__(self):
        """Cleanup on object destruction"""
        self._running = False
        if hasattr(self, '_listener_thread'):
            self._listener_thread.join(timeout=1.0)
        if hasattr(self, 'can_bus'):
            self.can_bus.shutdown()
    
    # === CAN Message Listener ===
    
    def _can_listener(self):
        """Background thread for receiving CAN messages"""
        while self._running:
            try:
                msg = self.can_bus.recv(timeout=0.1)
                if msg is not None:
                    self._process_message(msg)
            except Exception as e:
                self.logger.log_debug(self.motor_id, f"Listener error: {e}")
    
    def _process_message(self, msg: can.Message):
        """Process received CAN message"""
        try:
            if self.protocol_mode == ProtocolMode.PRIVATE:
                processed = self.private_handler.process_message(msg, self.status, self.param_data)
            else:
                processed = self.mit_handler.process_message(msg, self.status)
            
            if processed and self.status_callback:
                self.status_callback(self.status)
                
        except Exception as e:
            self.logger.log_debug(self.motor_id, f"Message processing error: {e}")
    
    # === Core Control Methods ===
    
    def enable_motor(self) -> bool:
        """
        Enable motor (make it operational)
        
        Returns:
            True if successful
            
        Raises:
            MotorException: If enable fails
        """
        try:
            if self.protocol_mode == ProtocolMode.PRIVATE:
                success = self.private_handler.send_enable()
            else:
                success = self.mit_handler.send_enable()
            
            if success:
                self.state = MotorState.ENABLED
                self.logger.log_info(self.motor_id, "Motor enabled")
                time.sleep(0.1)  # Wait for motor to respond
                return True
            else:
                raise MotorException("Enable command failed")
                
        except Exception as e:
            self.logger.log_error(self.motor_id, 0, f"Enable failed: {e}")
            raise
    
    def disable_motor(self, clear_error: bool = False) -> bool:
        """
        Disable motor (stop operation)
        
        Args:
            clear_error: If True, also clear error flags
            
        Returns:
            True if successful
        """
        try:
            if self.protocol_mode == ProtocolMode.PRIVATE:
                success = self.private_handler.send_disable(clear_error)
            else:
                success = self.mit_handler.send_disable()
            
            if success:
                self.state = MotorState.DISABLED
                self.logger.log_info(self.motor_id, "Motor disabled")
                return True
            else:
                self.logger.log_warning(self.motor_id, "Disable command failed")
                return False
                
        except Exception as e:
            self.logger.log_error(self.motor_id, 0, f"Disable failed: {e}")
            return False
    
    def set_zero_position(self) -> bool:
        """
        Set current position as zero reference
        
        Returns:
            True if successful
            
        Note:
            Motor must be disabled first
        """
        try:
            # Ensure motor is disabled
            if self.state != MotorState.DISABLED:
                self.disable_motor()
                time.sleep(0.1)
            
            # Send set zero command
            if self.protocol_mode == ProtocolMode.PRIVATE:
                success = self.private_handler.send_set_zero()
            else:
                success = self.mit_handler.send_set_zero()
            
            if success:
                self.logger.log_info(self.motor_id, "Zero position set")
                time.sleep(0.1)
                return True
            else:
                self.logger.log_warning(self.motor_id, "Set zero failed")
                return False
                
        except Exception as e:
            self.logger.log_error(self.motor_id, 0, f"Set zero failed: {e}")
            return False
    
    # === Parameter Access (Private Protocol Only) ===
    
    def set_parameter(self, param_index: int, value: float, value_mode: str = 'p') -> bool:
        """
        Set motor parameter
        
        Args:
            param_index: Parameter index (0x7xxx)
            value: Parameter value
            value_mode: 'p' for float parameter, 'j' for integer mode
            
        Returns:
            True if successful
            
        Raises:
            RuntimeError: If not in Private protocol mode
            ValueError: If parameter validation fails
        """
        if self.protocol_mode != ProtocolMode.PRIVATE:
            raise RuntimeError("Parameter access only available in Private protocol")
        
        # Validate parameter (only for writable parameters)
        if value_mode == 'p':
            valid, error_msg = validate_parameter(param_index, value)
            if not valid:
                raise ValueError(error_msg)
        
        success = self.private_handler.send_set_parameter(param_index, value, value_mode)
        
        if success:
            time.sleep(0.01)  # Small delay for parameter update
        
        return success
    
    def get_parameter(self, param_index: int) -> bool:
        """
        Request parameter read (result updated in param_data)
        
        Args:
            param_index: Parameter index (0x7xxx)
            
        Returns:
            True if request sent
            
        Raises:
            RuntimeError: If not in Private protocol mode
        """
        if self.protocol_mode != ProtocolMode.PRIVATE:
            raise RuntimeError("Parameter access only available in Private protocol")
        
        return self.private_handler.send_get_parameter(param_index)
    
    def save_parameters(self) -> bool:
        """
        Save parameters to FLASH memory
        
        Returns:
            True if successful
            
        Raises:
            RuntimeError: If not in Private protocol mode
        """
        if self.protocol_mode != ProtocolMode.PRIVATE:
            raise RuntimeError("Parameter save only available in Private protocol")
        
        success = self.private_handler.send_save_parameters()
        if success:
            self.logger.log_info(self.motor_id, "Parameters saved to FLASH")
        return success
    
    # === Motion Control ===
    
    def send_motion_control(self, torque: float = 0.0, angle: float = 0.0, 
                          speed: float = 0.0, kp: float = 0.0, kd: float = 0.0) -> bool:
        """
        Send composite motion control command (Private protocol)
        
        Args:
            torque: Torque feedforward [Nm], -17 to 17
            angle: Target angle [rad], -12.57 to 12.57
            speed: Target speed [rad/s], -44 to 44
            kp: Position gain, 0 to 500
            kd: Damping gain, 0 to 5
            
        Returns:
            True if successful
        """
        if self.protocol_mode != ProtocolMode.PRIVATE:
            raise RuntimeError("Motion control only in Private protocol")
        
        cmd = MotionControlCommand(
            torque=torque,
            angle=angle,
            speed=speed,
            kp=kp,
            kd=kd
        )
        
        success = self.private_handler.send_motion_control(cmd)
        if success:
            self.state = MotorState.RUNNING
        return success
    
    def send_mit_control(self, position: float = 0.0, velocity: float = 0.0,
                        kp: float = 0.0, kd: float = 0.0, torque: float = 0.0) -> bool:
        """
        Send MIT composite control command
        
        Args:
            position: Target position [rad], -12.57 to 12.57
            velocity: Target velocity [rad/s], -44 to 44
            kp: Position gain, 0 to 500
            kd: Damping gain, 0 to 5
            torque: Feedforward torque [Nm], -18 to 18
            
        Returns:
            True if successful
        """
        if self.protocol_mode != ProtocolMode.MIT:
            raise RuntimeError("MIT control only in MIT protocol")
        
        cmd = MITCommand(
            position=position,
            velocity=velocity,
            kp=kp,
            kd=kd,
            torque=torque
        )
        
        success = self.mit_handler.send_composite_control(cmd)
        if success:
            self.state = MotorState.RUNNING
        return success
    
    # === Protocol Switching ===
    
    def set_protocol_mode(self, protocol_mode: ProtocolMode) -> bool:
        """
        Change protocol mode (requires motor restart)
        
        Args:
            protocol_mode: New protocol mode
            
        Returns:
            True if command sent
            
        Warning:
            Motor must be power-cycled after this command
        """
        if self.protocol_mode != ProtocolMode.PRIVATE:
            raise RuntimeError("Protocol switching only from Private mode")
        
        success = self.private_handler.send_set_protocol_mode(protocol_mode)
        if success:
            self.logger.log_warning(self.motor_id, 
                f"Protocol change to {protocol_mode.name} requested. Power-cycle motor!")
        return success
    
    # === Properties ===
    
    @property
    def angle(self) -> float:
        """Current angle [rad]"""
        return self.status.angle
    
    @property
    def speed(self) -> float:
        """Current speed [rad/s]"""
        return self.status.speed
    
    @property
    def torque(self) -> float:
        """Current torque [Nm]"""
        return self.status.torque
    
    @property
    def temperature(self) -> float:
        """Current temperature [°C]"""
        return self.status.temperature
    
    @property
    def has_error(self) -> bool:
        """Check if motor has any errors"""
        return self.status.has_error
    
    @property
    def error_description(self) -> str:
        """Get error description"""
        return self.error_handler.get_error_description(self.status.error_code)
    
    # === Utility Methods ===
    
    def set_status_callback(self, callback: Callable[[MotorStatus], None]):
        """
        Set callback function for status updates
        
        Args:
            callback: Function to call with MotorStatus when updated
        """
        self.status_callback = callback
    
    def print_status(self):
        """Print current motor status"""
        print(self.status)
    
    def __str__(self) -> str:
        """String representation"""
        return f"RobStrideMotor(id={self.motor_id}, protocol={self.protocol_mode.name}, state={self.state.name})"
    
    def __repr__(self) -> str:
        """Detailed representation"""
        return (f"RobStrideMotor(can_id={self.motor_id}, protocol={self.protocol_mode.name}, "
                f"state={self.state.name}, angle={self.angle:.3f}, speed={self.speed:.3f})")
