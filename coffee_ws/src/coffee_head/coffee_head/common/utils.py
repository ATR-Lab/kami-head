#!/usr/bin/env python3

import math
import time
from typing import Tuple, List, Optional, Dict, Any

# PID controller for smooth motor control
class PIDController:
    def __init__(self, kp=0.5, ki=0.0, kd=0.1, output_limits=(-100, 100)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        
        self.error_sum = 0
        self.last_error = 0
        self.last_time = time.time()
    
    def reset(self):
        """Reset the controller state"""
        self.error_sum = 0
        self.last_error = 0
        self.last_time = time.time()
    
    def compute(self, setpoint, process_value):
        """Compute the control output based on the error"""
        current_time = time.time()
        dt = current_time - self.last_time
        
        # Cap dt to prevent huge jumps if the function hasn't been called for a while
        if dt > 1.0:
            dt = 0.1
        
        # Calculate error terms
        error = setpoint - process_value
        
        # Only accumulate error when close to target (anti-windup)
        if abs(error) < 20:
            self.error_sum += error * dt
        
        # Calculate derivative with smoothing
        if dt > 0:
            derivative = (error - self.last_error) / dt
        else:
            derivative = 0
        
        # Calculate output
        output = (self.kp * error) + (self.ki * self.error_sum) + (self.kd * derivative)
        
        # Apply output limits
        if self.output_limits:
            output = max(min(output, self.output_limits[1]), self.output_limits[0])
        
        # Store values for next iteration
        self.last_error = error
        self.last_time = current_time
        
        return output


def calculate_distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    """Calculate Euclidean distance between two points"""
    return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)


def normalize_vector(x: float, y: float) -> Tuple[float, float]:
    """Normalize a vector to unit length"""
    magnitude = math.sqrt(x*x + y*y)
    if magnitude > 0:
        return x/magnitude, y/magnitude
    return 0.0, 0.0


def calculate_vector_magnitude(x: float, y: float) -> float:
    """Calculate the magnitude of a vector"""
    return math.sqrt(x*x + y*y)


def map_range(value: float, in_min: float, in_max: float, out_min: float, out_max: float) -> float:
    """Map a value from one range to another"""
    # Ensure we don't divide by zero
    if in_max == in_min:
        return out_min
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def degrees_to_motor_position(angle: float, positions_per_degree: float) -> int:
    """Convert degrees to motor position units"""
    return int(angle * positions_per_degree)


def motor_position_to_degrees(position: int, degrees_per_position: float) -> float:
    """Convert motor position units to degrees"""
    return position * degrees_per_position


def clamp(value: float, min_val: float, max_val: float) -> float:
    """Clamp a value between min and max"""
    return max(min_val, min(value, max_val))


def smooth_value(current: float, target: float, smoothing_factor: float) -> float:
    """Apply smoothing to a value (higher smoothing_factor = more smoothing)"""
    return current * smoothing_factor + target * (1 - smoothing_factor)


def apply_deadzone(value: float, deadzone: float) -> float:
    """Apply deadzone to a value"""
    if abs(value) < deadzone:
        return 0.0
    if value > 0:
        return (value - deadzone) / (1.0 - deadzone)
    return (value + deadzone) / (1.0 - deadzone) 