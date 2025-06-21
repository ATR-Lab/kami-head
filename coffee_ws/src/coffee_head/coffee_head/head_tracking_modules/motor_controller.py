#!/usr/bin/env python3

import time
import threading
from typing import Optional, Callable, Dict, Tuple, List, Any

from ..common.data_types import MotorPosition
from ..common.utils import degrees_to_motor_position, motor_position_to_degrees

class MotorController:
    """Controller for Dynamixel motors used in head tracking"""
    
    def __init__(self, 
                 pan_motor_id=1, 
                 tilt_motor_id=2,
                 min_position=0,
                 max_position=4095):
        # Motor IDs
        self.pan_motor_id = pan_motor_id
        self.tilt_motor_id = tilt_motor_id
        
        # Position range
        self.min_position = min_position
        self.max_position = max_position
        self.position_range = self.max_position - self.min_position
        
        # Angle conversions
        self.degrees_per_position = 360.0 / self.position_range
        self.positions_per_degree = self.position_range / 360.0
        
        # Motor angle limits (in degrees)
        self.pan_min_angle = 0.01   # Min pan angle (right)
        self.pan_max_angle = 180.0  # Max pan angle (left)
        self.tilt_min_angle = 135.0 # Min tilt angle (up)
        self.tilt_max_angle = 225.0 # Max tilt angle (down)
        
        # Default positions (center)
        self.default_pan_angle = 90.0   # degrees (center)
        self.default_tilt_angle = 180.0 # degrees (center)
        
        # Current positions
        self.current_pan_position = 0
        self.current_tilt_position = 0
        self.current_pan_angle = self.default_pan_angle
        self.current_tilt_angle = self.default_tilt_angle
        
        # Communication settings
        self.baud_rate = 1000000  # Default baud rate
        
        # Command callback (to be set by ROS node)
        self.command_callback = None
        
        # Position feedback callback (to be set by ROS node)
        self.position_feedback_callback = None
    
    def set_command_callback(self, callback: Callable[[int, int], None]) -> None:
        """Set callback for sending motor commands to ROS node"""
        self.command_callback = callback
    
    def set_position_feedback_callback(self, callback: Callable[[MotorPosition], None]) -> None:
        """Set callback for sending position feedback to ROS node"""
        self.position_feedback_callback = callback
    
    def set_baud_rate(self, baud_rate: int) -> None:
        """Set motor baud rate"""
        self.baud_rate = baud_rate
    
    def set_current_positions(self, pan_position: int, tilt_position: int) -> None:
        """Update current motor positions from feedback"""
        self.current_pan_position = pan_position
        self.current_tilt_position = tilt_position
        
        # Convert to angles
        self.current_pan_angle = motor_position_to_degrees(pan_position, self.degrees_per_position) % 360
        self.current_tilt_angle = motor_position_to_degrees(tilt_position, self.degrees_per_position) % 360
        
        # Send position feedback
        if self.position_feedback_callback:
            self.position_feedback_callback(MotorPosition(
                pan_angle=self.current_pan_angle,
                tilt_angle=self.current_tilt_angle
            ))
    
    def get_current_angles(self) -> MotorPosition:
        """Get current motor angles"""
        return MotorPosition(
            pan_angle=self.current_pan_angle,
            tilt_angle=self.current_tilt_angle
        )
    
    def send_pan_command(self, angle: float) -> None:
        """Send command to pan motor (in degrees)"""
        # Clamp angle to valid range
        angle = max(self.pan_min_angle, min(angle, self.pan_max_angle))
        
        # Convert to motor position
        position = degrees_to_motor_position(angle, self.positions_per_degree)
        
        # Send command via callback
        if self.command_callback:
            self.command_callback(self.pan_motor_id, position)
        
        # Update current position (without feedback)
        self.current_pan_position = position
        self.current_pan_angle = angle
    
    def send_tilt_command(self, angle: float) -> None:
        """Send command to tilt motor (in degrees)"""
        # Clamp angle to valid range
        angle = max(self.tilt_min_angle, min(angle, self.tilt_max_angle))
        
        # Convert to motor position
        position = degrees_to_motor_position(angle, self.positions_per_degree)
        
        # Send command via callback
        if self.command_callback:
            self.command_callback(self.tilt_motor_id, position)
        
        # Update current position (without feedback)
        self.current_tilt_position = position
        self.current_tilt_angle = angle
    
    def send_coordinated_movement(self, pan_angle: float, tilt_angle: float) -> None:
        """Send coordinated movement to both motors"""
        self.send_pan_command(pan_angle)
        self.send_tilt_command(tilt_angle)
    
    def reset_to_default(self) -> None:
        """Reset motors to default positions"""
        self.send_coordinated_movement(self.default_pan_angle, self.default_tilt_angle) 