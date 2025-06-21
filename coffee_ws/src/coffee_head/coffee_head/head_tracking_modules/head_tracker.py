#!/usr/bin/env python3

import time
import math
import threading
from typing import Optional, Tuple, List, Dict, Any, Callable

from ..common.data_types import FaceData, VelocityVector
from ..common.utils import PIDController, smooth_value

class HeadTracker:
    """Core head tracking logic for face tracking with pan/tilt motors"""
    
    def __init__(self):
        # Frame dimensions
        self.frame_width = 640
        self.frame_height = 480
        self.center_x = self.frame_width // 2
        self.center_y = self.frame_height // 2
        
        # Tracking state
        self.tracking_enabled = False
        self.target_face = None
        self.target_face_index = None
        self.face_positions = []
        self.last_face_update_time = time.time()
        self.face_timeout = 1.0  # seconds
        
        # Movement thresholds
        self.pan_threshold = 40  # Horizontal pixel threshold
        self.tilt_threshold = 25  # Vertical pixel threshold
        
        # Target angles
        self.target_pan_angle = 90.0
        self.target_tilt_angle = 180.0
        
        # Smoothing
        self.smoothing_factor = 0.8  # Higher = smoother but slower
        
        # Update rate limiting
        self.update_rate_hz = 30.0
        self.update_interval = 1.0 / self.update_rate_hz
        self.last_update_time = time.time()
        
        # PID controllers
        self.use_pid_smoothing = True
        self.pan_pid = PIDController(kp=0.1, ki=0.005, kd=0.08, output_limits=(-5, 5))
        self.tilt_pid = PIDController(kp=0.15, ki=0.01, kd=0.05, output_limits=(-8, 8))
        
        # Movement parameters
        self.min_pan_speed = 1.0   # deg/s
        self.min_tilt_speed = 1.0  # deg/s
        self.max_pan_speed = 20.0  # deg/s
        self.max_tilt_speed = 15.0 # deg/s
        
        # Face velocity calculation
        self.previous_face_positions = {}
        self.face_velocities = {}
        self.calculate_velocity = False
        
        # Callbacks
        self.status_callback = None
        self.velocity_callback = None
    
    def set_status_callback(self, callback: Callable[[str], None]) -> None:
        """Set callback for status updates"""
        self.status_callback = callback
    
    def set_velocity_callback(self, callback: Callable[[VelocityVector], None]) -> None:
        """Set callback for velocity updates"""
        self.velocity_callback = callback
        self.calculate_velocity = True
    
    def set_frame_size(self, width: int, height: int) -> None:
        """Update frame size and center coordinates"""
        self.frame_width = width
        self.frame_height = height
        self.center_x = self.frame_width // 2
        self.center_y = self.frame_height // 2
    
    def set_thresholds(self, pan_threshold: int, tilt_threshold: int) -> None:
        """Set movement thresholds"""
        self.pan_threshold = max(5, pan_threshold)
        self.tilt_threshold = max(5, tilt_threshold)
    
    def set_update_rate(self, rate_hz: float) -> None:
        """Set the motor update rate in Hz"""
        self.update_rate_hz = max(1.0, min(100.0, rate_hz))
        self.update_interval = 1.0 / self.update_rate_hz
    
    def set_pid_smoothing(self, enabled: bool) -> None:
        """Enable or disable PID smoothing"""
        self.use_pid_smoothing = enabled
        
        # Reset PIDs if enabled
        if enabled:
            self.pan_pid.reset()
            self.tilt_pid.reset()
    
    def set_smoothing_factor(self, factor: float) -> None:
        """Set smoothing factor"""
        self.smoothing_factor = max(0.0, min(0.99, factor))
    
    def enable_tracking(self, enabled: bool) -> None:
        """Enable or disable tracking"""
        if enabled and not self.tracking_enabled:
            # Reset PIDs when starting tracking
            self.pan_pid.reset()
            self.tilt_pid.reset()
            self._update_status("Tracking enabled - waiting for face data")
        elif not enabled and self.tracking_enabled:
            self._update_status("Tracking disabled")
            self.target_face = None
            self.target_face_index = None
        
        self.tracking_enabled = enabled
    
    def update_faces(self, faces: List[FaceData], timestamp: float) -> None:
        """Update face data from camera"""
        self.face_positions = faces
        self.last_face_update_time = timestamp
        
        # Calculate face velocities if enabled
        if self.calculate_velocity:
            self._calculate_face_velocities(timestamp)
        
        # Automatically select a target face if needed
        if self.target_face_index is None and faces:
            self._select_target_face()
        
        # Clear target if we lost the tracked face
        if self.target_face_index is not None and self.target_face_index >= len(faces):
            self.target_face_index = None
            self.target_face = None
            self._update_status("Target face lost")
    
    def check_timeout(self) -> bool:
        """Check if face data has timed out"""
        time_since_update = time.time() - self.last_face_update_time
        if time_since_update > self.face_timeout:
            # Face data has timed out
            if self.tracking_enabled and self.target_face is not None:
                self._update_status("Face data timeout")
                self.target_face = None
                self.target_face_index = None
            return True
        return False
    
    def update(self) -> Optional[Tuple[float, float]]:
        """Update head tracking and return pan/tilt angles if movement is needed"""
        if not self.tracking_enabled:
            return None
        
        # If no target face, nothing to do
        if self.target_face_index is None or not self.face_positions:
            return None
        
        # Ensure target face index is valid
        if self.target_face_index >= len(self.face_positions):
            self.target_face_index = None
            self.target_face = None
            return None
        
        # Get target face
        self.target_face = self.face_positions[self.target_face_index]
        
        # Calculate movement vector
        movement = self._calculate_movement()
        if movement is None:
            return None
        
        # Get angles
        pan_angle, tilt_angle = movement
        
        # Check if we should update based on rate limiting
        current_time = time.time()
        if current_time - self.last_update_time < self.update_interval:
            return None
        
        # Update last update time
        self.last_update_time = current_time
        
        # Return the new angles
        return (pan_angle, tilt_angle)
    
    def _select_target_face(self) -> None:
        """Select the best face to track"""
        if not self.face_positions:
            return
        
        # Choose the largest face (closest to camera) or the center-most face
        largest_area = 0
        closest_to_center = float('inf')
        
        for i, face in enumerate(self.face_positions):
            # Calculate face area
            area = face.area
            
            # Calculate distance from center
            dx = face.center_x - self.center_x
            dy = face.center_y - self.center_y
            distance_from_center = math.sqrt(dx*dx + dy*dy)
            
            # Prioritize larger faces unless they're far from center
            score = distance_from_center - area * 0.02
            
            if score < closest_to_center:
                closest_to_center = score
                largest_area = area
                self.target_face_index = i
        
        if self.target_face_index is not None:
            self.target_face = self.face_positions[self.target_face_index]
            self._update_status(f"Tracking face {self.target_face_index+1}/{len(self.face_positions)}")
    
    def _calculate_movement(self) -> Optional[Tuple[float, float]]:
        """Calculate the required pan/tilt movement"""
        if self.target_face is None:
            return None
        
        # Calculate error from center of frame
        error_x = self.center_x - self.target_face.center_x
        error_y = self.center_y - self.target_face.center_y
        
        # Calculate vector magnitude (distance from center)
        vector_magnitude = math.sqrt(error_x**2 + error_y**2)
        
        # Check if movement is needed based on separate thresholds
        need_pan_movement = abs(error_x) > self.pan_threshold
        need_tilt_movement = abs(error_y) > self.tilt_threshold
        
        # If neither axis needs movement, don't move the head
        if not need_pan_movement and not need_tilt_movement:
            return None
        
        # Current angles should come from motor feedback
        # Here we'll rely on internal tracking of target angles
        current_pan_angle = self.target_pan_angle
        current_tilt_angle = self.target_tilt_angle
        
        if self.use_pid_smoothing:
            # PID-based approach
            # Calculate the adjustments using PIDs, but only if needed
            pan_adjustment = self.pan_pid.compute(0, -error_x) if need_pan_movement else 0
            tilt_adjustment = self.tilt_pid.compute(0, -error_y) if need_tilt_movement else 0
            
            # Calculate new angles
            new_pan_angle = current_pan_angle + pan_adjustment
            new_tilt_angle = current_tilt_angle + tilt_adjustment
            
            # Update target angles
            self.target_pan_angle = new_pan_angle
            self.target_tilt_angle = new_tilt_angle
            
            # Apply smoothing for less jittery movement
            smoothed_pan_angle = smooth_value(current_pan_angle, new_pan_angle, self.smoothing_factor)
            smoothed_tilt_angle = smooth_value(current_tilt_angle, new_tilt_angle, self.smoothing_factor)
            
            return smoothed_pan_angle, smoothed_tilt_angle
        else:
            # Direct proportional approach
            # Scale the errors to angles (with gain factors)
            pan_gain = 0.05  # Direct proportional gain
            tilt_gain = 0.04  # Direct proportional gain
            
            # Normalize the vector for consistent movement speed
            if vector_magnitude > 0:
                # Apply movements only where needed
                normalized_x = error_x / vector_magnitude if need_pan_movement else 0
                normalized_y = error_y / vector_magnitude if need_tilt_movement else 0
                
                # Scale movement based on distance - farther = faster
                distance_factor = min(1.0, vector_magnitude / 300)  # Cap at 1.0
                
                # Apply graduated speed based on distance
                pan_speed = self.min_pan_speed + distance_factor * (self.max_pan_speed - self.min_pan_speed)
                tilt_speed = self.min_tilt_speed + distance_factor * (self.max_tilt_speed - self.min_tilt_speed)
                
                # Calculate adjustments with normalized direction and variable speed
                pan_adjustment = -normalized_x * pan_speed * 0.05  # Time factor for smooth movement
                tilt_adjustment = -normalized_y * tilt_speed * 0.05
            else:
                pan_adjustment = 0
                tilt_adjustment = 0
            
            # Calculate new angles
            new_pan_angle = current_pan_angle + pan_adjustment
            new_tilt_angle = current_tilt_angle + tilt_adjustment
            
            # Update target angles
            self.target_pan_angle = new_pan_angle
            self.target_tilt_angle = new_tilt_angle
            
            # Apply smoothing for less jittery movement
            smoothed_pan_angle = smooth_value(current_pan_angle, new_pan_angle, self.smoothing_factor)
            smoothed_tilt_angle = smooth_value(current_tilt_angle, new_tilt_angle, self.smoothing_factor)
            
            return smoothed_pan_angle, smoothed_tilt_angle
    
    def _calculate_face_velocities(self, timestamp: float) -> None:
        """Calculate velocities for each face based on position history"""
        if not self.face_positions:
            self.previous_face_positions = {}
            self.face_velocities = {}
            return
        
        # Update face velocities for each face
        current_face_ids = set()
        
        for i, face in enumerate(self.face_positions):
            face_id = i  # Use index as face ID
            current_face_ids.add(face_id)
            
            # Get current position
            current_pos = (face.center_x, face.center_y, timestamp)
            
            # If we don't have previous position, just store current
            if face_id not in self.previous_face_positions:
                self.previous_face_positions[face_id] = current_pos
                self.face_velocities[face_id] = VelocityVector(0.0, 0.0, 0.0)
                continue
            
            # Get previous position
            prev_x, prev_y, prev_time = self.previous_face_positions[face_id]
            
            # Calculate time difference
            dt = timestamp - prev_time
            
            if dt > 0.001:  # Avoid division by zero
                # Calculate velocity components
                vx = (face.center_x - prev_x) / dt
                vy = (face.center_y - prev_y) / dt
                
                # Calculate magnitude
                magnitude = math.sqrt(vx*vx + vy*vy)
                
                # Create velocity vector
                velocity = VelocityVector(vx, vy, magnitude)
                
                # Store velocity
                self.face_velocities[face_id] = velocity
                
                # If this is the target face, send velocity callback
                if face_id == self.target_face_index and self.velocity_callback:
                    self.velocity_callback(velocity)
            
            # Update previous position
            self.previous_face_positions[face_id] = current_pos
        
        # Clean up entries for faces that are no longer present
        for face_id in list(self.previous_face_positions.keys()):
            if face_id not in current_face_ids:
                del self.previous_face_positions[face_id]
                if face_id in self.face_velocities:
                    del self.face_velocities[face_id]
    
    def _update_status(self, status: str) -> None:
        """Update tracking status"""
        if self.status_callback:
            self.status_callback(status)
    
    def reset_target_angles(self, pan_angle: float, tilt_angle: float) -> None:
        """Reset target angles"""
        self.target_pan_angle = pan_angle
        self.target_tilt_angle = tilt_angle 