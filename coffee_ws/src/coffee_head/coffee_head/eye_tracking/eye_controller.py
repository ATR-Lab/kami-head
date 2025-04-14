#!/usr/bin/env python3

import time
import math
import numpy as np
from typing import Tuple, Dict, Any, List, Optional

class EyeController:
    """
    Controller for eye movement logic based on face tracking data.
    Handles calculations for eye positions, blinking, and other eye behaviors.
    """
    
    def __init__(self):
        """Initialize the eye controller"""
        # Default camera parameters
        self.camera_fov_h = 62.2  # degrees, horizontal field of view
        self.camera_fov_v = 48.8  # degrees, vertical field of view
        self.camera_aspect = 1.33  # 4:3 aspect ratio
        
        # Tracking parameters
        self.max_tracking_distance = 3.0  # meters
        self.tracking_scale = 1.0  # Scale factor for pupil movement
        
        # Face detection data
        self.face_detected = False
        self.face_position = (0.0, 0.0, 0.0)  # x, y, z in camera frame
        self.face_position_normalized = (0.0, 0.0)  # x, y normalized to -1..1
        self.face_size = 0.0  # normalized 0..1 scale
        self.face_velocity = (0.0, 0.0, 0.0)  # x, y, z velocity in camera frame
        self.face_detection_confidence = 0.0  # 0..1 confidence
        
        # Time tracking
        self.last_face_detection_time = 0.0
        self.face_timeout = 2.0  # seconds before considering face lost
        
        # Head position (from motor feedback)
        self.head_pan = 0.0  # radians
        self.head_tilt = 0.0  # radians
        
        # Behavior parameters
        self.idle_behavior_enabled = True
        self.blink_enabled = True
        self.track_velocity_enabled = True
        
        # Idle behavior
        self.idle_start_time = time.time()
        self.idle_duration = 0.0
        self.idle_position = (0.0, 0.0)
        self.idle_transition_time = 2.0  # seconds to transition to new idle position
        
        # Output pupil positions (-1.0 to 1.0 normalized)
        self.left_pupil_position = (0.0, 0.0)
        self.right_pupil_position = (0.0, 0.0)
        
        # Personality parameters (influences eye movement)
        self.personality = {
            "nervousness": 0.2,  # 0.0 to 1.0 (higher = more random movements)
            "attentiveness": 0.8,  # 0.0 to 1.0 (higher = more focus on face)
            "curiosity": 0.5,  # 0.0 to 1.0 (higher = more looking around)
            "drowsiness": 0.1,  # 0.0 to 1.0 (higher = more blinking, slower movement)
        }
    
    def set_camera_parameters(self, fov_h: float, fov_v: float, aspect: float):
        """Set camera parameters"""
        self.camera_fov_h = fov_h
        self.camera_fov_v = fov_v
        self.camera_aspect = aspect
    
    def set_tracking_parameters(self, max_distance: float, scale: float):
        """Set tracking parameters"""
        self.max_tracking_distance = max_distance
        self.tracking_scale = scale
    
    def set_personality(self, personality_params: Dict[str, float]):
        """Set personality parameters"""
        for param, value in personality_params.items():
            if param in self.personality:
                self.personality[param] = max(0.0, min(1.0, value))
    
    def set_head_position(self, pan: float, tilt: float):
        """Set the current head position from motor feedback"""
        self.head_pan = pan
        self.head_tilt = tilt
    
    def update_face_detection(self, 
                             position: Tuple[float, float, float], 
                             size: float,
                             confidence: float):
        """
        Update with face detection data
        
        Args:
            position: (x, y, z) position in camera frame (meters)
            size: normalized size of face (0..1)
            confidence: detection confidence (0..1)
        """
        now = time.time()
        
        # Store previous position for velocity calculation
        prev_position = self.face_position
        prev_time = self.last_face_detection_time
        
        # Update face data
        self.face_position = position
        self.face_size = size
        self.face_detection_confidence = confidence
        self.face_detected = True
        self.last_face_detection_time = now
        
        # Calculate face velocity if we have previous data
        if prev_time > 0 and now > prev_time:
            dt = now - prev_time
            if dt > 0:
                # Calculate velocity in meters per second
                self.face_velocity = (
                    (position[0] - prev_position[0]) / dt,
                    (position[1] - prev_position[1]) / dt,
                    (position[2] - prev_position[2]) / dt
                )
        
        # Calculate normalized position (-1..1 for renderer)
        self._calculate_normalized_position()
    
    def update_face_velocity(self, velocity: Tuple[float, float, float]):
        """
        Update face velocity directly (if available from external source)
        
        Args:
            velocity: (x, y, z) velocity in camera frame (meters/sec)
        """
        self.face_velocity = velocity
    
    def update(self) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        """
        Update eye controller state and return current pupil positions
        
        Returns:
            Tuple of (left_pupil_position, right_pupil_position)
            Each position is (x, y) normalized to -1..1 range
        """
        now = time.time()
        
        # Check if face is still detected (timeout)
        if self.face_detected and now - self.last_face_detection_time > self.face_timeout:
            self.face_detected = False
        
        # Calculate pupil positions based on face tracking or idle behavior
        if self.face_detected:
            self._update_tracking_mode()
        else:
            self._update_idle_mode()
        
        # Apply personality and random variations
        self._apply_personality_effects()
        
        return (self.left_pupil_position, self.right_pupil_position)
    
    def _calculate_normalized_position(self):
        """Calculate normalized face position for renderer"""
        # Convert 3D position to angular position
        if self.face_position[2] > 0:
            # Calculate angles
            angle_x = math.atan2(self.face_position[0], self.face_position[2])
            angle_y = math.atan2(self.face_position[1], self.face_position[2])
            
            # Normalize to -1..1 based on field of view
            norm_x = angle_x / math.radians(self.camera_fov_h / 2)
            norm_y = angle_y / math.radians(self.camera_fov_v / 2)
            
            # Clamp to valid range
            norm_x = max(-1.0, min(1.0, norm_x))
            norm_y = max(-1.0, min(1.0, norm_y))
            
            self.face_position_normalized = (norm_x, norm_y)
        else:
            # Face is behind camera, use previous normalized position
            pass
    
    def _update_tracking_mode(self):
        """Update pupil positions based on face tracking"""
        # Get base position from normalized face position
        base_x, base_y = self.face_position_normalized
        
        # Apply scaling factor
        scaled_x = base_x * self.tracking_scale
        scaled_y = base_y * self.tracking_scale
        
        # Apply distance-based scaling (closer faces have more effect)
        distance = self.face_position[2]
        if distance > 0:
            distance_factor = max(0.0, min(1.0, 1.0 - (distance / self.max_tracking_distance)))
            scaled_x *= 0.5 + 0.5 * distance_factor
            scaled_y *= 0.5 + 0.5 * distance_factor
        
        # Apply velocity anticipation if enabled
        if self.track_velocity_enabled and sum(v*v for v in self.face_velocity) > 0.01:
            # Scale velocity influence based on face distance
            vel_scale = 0.1 * max(0.0, min(1.0, 1.0 - (distance / self.max_tracking_distance)))
            
            # Add velocity component to position (look ahead)
            vel_x = self.face_velocity[0] * vel_scale
            vel_y = self.face_velocity[1] * vel_scale
            
            # Limit velocity influence
            vel_x = max(-0.3, min(0.3, vel_x))
            vel_y = max(-0.3, min(0.3, vel_y))
            
            scaled_x += vel_x
            scaled_y += vel_y
        
        # Ensure within valid range
        scaled_x = max(-1.0, min(1.0, scaled_x))
        scaled_y = max(-1.0, min(1.0, scaled_y))
        
        # Set both eyes to same position (could be different in future)
        self.left_pupil_position = (scaled_x, scaled_y)
        self.right_pupil_position = (scaled_x, scaled_y)
    
    def _update_idle_mode(self):
        """Update pupil positions for idle behavior when no face is detected"""
        if not self.idle_behavior_enabled:
            # Just center the eyes
            self.left_pupil_position = (0.0, 0.0)
            self.right_pupil_position = (0.0, 0.0)
            return
        
        now = time.time()
        
        # Check if we need a new idle position
        if now - self.idle_start_time > self.idle_duration:
            # Generate new idle position and duration
            self._generate_new_idle_position()
        
        # Calculate interpolation factor
        transition_progress = min(1.0, (now - self.idle_start_time) / self.idle_transition_time)
        
        # Set both eyes to same idle position
        self.left_pupil_position = self.idle_position
        self.right_pupil_position = self.idle_position
    
    def _generate_new_idle_position(self):
        """Generate a new random idle position and duration"""
        now = time.time()
        
        # Randomize based on personality
        nervousness = self.personality["nervousness"]
        curiosity = self.personality["curiosity"]
        
        # More curious = looking around more, more nervous = faster changes
        position_scale = 0.3 + 0.6 * curiosity
        
        # Generate random position (-1..1)
        rand_x = (np.random.random() * 2.0 - 1.0) * position_scale
        rand_y = (np.random.random() * 2.0 - 1.0) * position_scale
        
        # More drowsy = looking downward
        drowsiness = self.personality["drowsiness"]
        rand_y -= drowsiness * 0.5
        
        # Ensure within valid range
        rand_x = max(-1.0, min(1.0, rand_x))
        rand_y = max(-1.0, min(1.0, rand_y))
        
        # Set new idle parameters
        self.idle_position = (rand_x, rand_y)
        self.idle_start_time = now
        
        # More nervous = shorter idle durations
        base_duration = 2.0 + (1.0 - nervousness) * 4.0
        variation = base_duration * 0.5
        self.idle_duration = base_duration + (np.random.random() * 2.0 - 1.0) * variation
        
        # Transition time
        self.idle_transition_time = 0.5 + (1.0 - nervousness) * 1.5
    
    def _apply_personality_effects(self):
        """Apply personality-based random variations to eye positions"""
        # Apply nervousness effect (random jitter)
        nervousness = self.personality["nervousness"]
        if nervousness > 0.05:
            # Scale jitter by nervousness
            jitter_scale = 0.02 * nervousness
            
            # Generate random jitter
            jitter_x = (np.random.random() * 2.0 - 1.0) * jitter_scale
            jitter_y = (np.random.random() * 2.0 - 1.0) * jitter_scale
            
            # Apply jitter to both eyes
            self.left_pupil_position = (
                self.left_pupil_position[0] + jitter_x,
                self.left_pupil_position[1] + jitter_y
            )
            self.right_pupil_position = (
                self.right_pupil_position[0] + jitter_x,
                self.right_pupil_position[1] + jitter_y
            )
            
            # Ensure within valid range
            self.left_pupil_position = (
                max(-1.0, min(1.0, self.left_pupil_position[0])),
                max(-1.0, min(1.0, self.left_pupil_position[1]))
            )
            self.right_pupil_position = (
                max(-1.0, min(1.0, self.right_pupil_position[0])),
                max(-1.0, min(1.0, self.right_pupil_position[1]))
            )

# Test function
def main():
    controller = EyeController()
    
    # Simulate detecting a face that moves around
    for i in range(100):
        # Generate simulated face data
        t = i / 10.0
        x = math.sin(t) * 0.5
        y = math.cos(t) * 0.3
        z = 1.5 + math.sin(t * 0.5) * 0.5
        
        size = 0.2 + 0.1 * (1.0 / z)
        confidence = 0.9
        
        # Update controller
        controller.update_face_detection((x, y, z), size, confidence)
        
        # Get eye positions
        left, right = controller.update()
        
        print(f"Face: ({x:.2f}, {y:.2f}, {z:.2f}), Eyes: L({left[0]:.2f}, {left[1]:.2f}), R({right[0]:.2f}, {right[1]:.2f})")
        
        time.sleep(0.1)
    
    # Test idle behavior
    print("\nTesting idle behavior...")
    for i in range(50):
        left, right = controller.update()
        print(f"Idle - Eyes: L({left[0]:.2f}, {left[1]:.2f}), R({right[0]:.2f}, {right[1]:.2f})")
        time.sleep(0.1)

if __name__ == "__main__":
    main() 