#!/usr/bin/env python3

import rclpy
import numpy as np
import threading
import time
import sys
import cv2
import json
import math  # Added for vector calculations
from rclpy.node import Node
from rclpy.qos import QoSProfile
from coffee_expressions_msgs.msg import AffectiveState
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Vector3
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from dynamixel_sdk_custom_interfaces.srv import GetPosition
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                            QLabel, QPushButton, QCheckBox, QSlider, QComboBox, QGroupBox,
                            QDoubleSpinBox, QScrollArea)
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt, pyqtSignal, QObject, QTimer

# PID controller class for smooth motor control
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


class HeadTrackingSystem(QObject):
    """Head tracking system that controls pan/tilt motors to keep faces centered"""
    frame_processed = pyqtSignal(np.ndarray)
    tracking_status = pyqtSignal(str)
    face_velocity = pyqtSignal(float, float, float)  # velocity_x, velocity_y, magnitude
    
    def __init__(self, node):
        super().__init__()
        self.node = node
        
        # Parameters for head tracking
        self.tracking_enabled = False
        self.target_face = None
        self.frame_width = 640
        self.frame_height = 480
        self.center_x = self.frame_width // 2
        self.center_y = self.frame_height // 2
        
        # Face movement tracking
        self.prev_face_positions = {}  # Dictionary to store previous face positions {face_id: [(x, y, timestamp), ...]}
        self.face_positions_history = 5  # Number of positions to keep for each face
        self.face_velocities = {}  # Dictionary to store face velocities {face_id: (vx, vy, magnitude)}
        self.velocity_smoothing = 0.7  # Smoothing factor for velocity calculations
        self.min_velocity_display = 5  # Minimum velocity magnitude to display vector
        self.last_face_update = time.time()
        
        # Communication parameters
        self.update_rate_hz = 30.0  # Default 30Hz update rate (configurable)
        self.update_interval = 1.0 / self.update_rate_hz  # Calculated interval in seconds
        self.baud_rate = 1000000  # Default baud rate for Dynamixel motors (configurable)
        self.baudrate_options = [9600, 19200, 57600, 115200, 1000000, 2000000, 3000000, 4000000, 4500000]
        
        # Separate pan and tilt thresholds
        self.pan_threshold = 40  # Horizontal pixel threshold - match with eye tracking
        self.tilt_threshold = 25  # Vertical pixel threshold
        self.movement_threshold = 40  # Legacy vector threshold (kept for compatibility)
        
        # Individual deadzone region used for display purposes
        self.deadzone_x = 40  # Match with pan threshold for consistent UI
        self.deadzone_y = 25  # Match with tilt threshold for consistent UI
        
        # Pan/tilt motor IDs and parameters
        self.pan_motor_id = 1
        self.tilt_motor_id = 9 # TODO: FIX NAMING AND NUMBERING
        
        # Motor angle limits (in degrees)
        self.pan_min_angle = 143.0  # Min pan angle (right)
        self.pan_max_angle = 210.0  # Max pan angle (left)
        self.tilt_min_angle = 169.0  # Min tilt angle (up)
        self.tilt_max_angle = 206.0  # Max tilt angle (down)
        
        # Default positions (center)
        self.default_pan_angle = 180.0  # degrees (center)
        self.default_tilt_angle = 180.0  # degrees (center)
        
        self.current_pan_position = 0
        self.current_tilt_position = 0
        
        # Scanning parameters when no face is detected
        self.scanning = False
        self.scan_direction = 1  # 1 for right-to-left, -1 for left-to-right
        self.scan_speed = 3.0    # Reduced scan speed for smoother movement
        self.scan_timer = None
        self.current_scan_angle = self.default_pan_angle
        
        # Smoothing for movement
        self.target_pan_angle = self.default_pan_angle
        self.target_tilt_angle = self.default_tilt_angle
        self.smoothing_factor = 0.8  # Higher value for smoother head movement
        
        # Add toggle for PID smoothing
        self.use_pid_smoothing = True  # Default on - can be toggled from UI
        
        # Dynamixel position conversion
        self.min_position = 0
        self.max_position = 4095
        self.position_range = self.max_position - self.min_position
        self.degrees_per_position = 360.0 / self.position_range
        self.positions_per_degree = self.position_range / 360.0
        
        # PID controllers for pan and tilt (tune these for smoother movement)
        # Lower kp, higher kd for smoother movement
        self.pan_pid = PIDController(kp=0.1, ki=0.005, kd=0.08, output_limits=(-5, 5))
        self.tilt_pid = PIDController(kp=0.15, ki=0.01, kd=0.05, output_limits=(-8, 8))
        
        # Parameters for coordinated movement
        self.min_pan_speed = 1.0   # deg/s
        self.min_tilt_speed = 1.0  # deg/s
        self.max_pan_speed = 80.0  # deg/s - higher for faster response
        self.max_tilt_speed = 15.0 # deg/s - slightly slower for tilt
        
        # Create ROS publisher for motor control
        qos = QoSProfile(depth=10)
        self.position_publisher = self.node.create_publisher(
            SetPosition,
            'set_position',
            qos
        )
        
        # Publisher for face velocity vector
        self.velocity_publisher = self.node.create_publisher(
            Vector3,
            'face_velocity',
            10
        )
        
        # Publishers for coordination with eye tracking
        self.pan_angle_publisher = self.node.create_publisher(
            Float32,
            'head_pan_angle',
            10
        )
        
        self.tilt_angle_publisher = self.node.create_publisher(
            Float32,
            'head_tilt_angle',
            10
        )
        
        # Subscribe to face detection data from camera_node.py
        # self.face_subscription = self.node.create_subscription(
        #     String,
        #     'face_detection_data',
        #     self.face_data_callback,
        #     10
        # )

        self.subscription = self.node.create_subscription(
            AffectiveState,
            '/robot/affective_state',
            self.face_data_callback,
            10)
        
        # Initialize motor positions
        self.last_update_time = time.time()
        self.last_pan_update_time = time.time()
        self.last_tilt_update_time = time.time()
        
        # Read initial motor positions
        self.read_motor_positions()
        
        # Set tilt to default position at startup
        self.set_tilt_to_default()
        
        # Last time we received face data
        self.last_face_data_time = time.time()
        
        self.node.get_logger().info("Head tracking system initialized and ready for face data")
    
    def set_update_rate(self, rate_hz):
        """Set the motor update rate in Hz"""
        self.update_rate_hz = max(1.0, min(100.0, rate_hz))  # Limit between 1Hz and 100Hz
        self.update_interval = 1.0 / self.update_rate_hz
        self.node.get_logger().info(f"Motor update rate set to {self.update_rate_hz:.1f}Hz (interval: {self.update_interval:.4f}s)")
    
    def set_baud_rate(self, baud_rate):
        """Set the motor communication baud rate"""
        self.baud_rate = baud_rate
        self.node.get_logger().info(f"Motor baud rate set to {self.baud_rate}")
        
        # Attempt to update the baud rate dynamically if possible
        # Note: This may require restarting the motor service to take effect
        try:
            # Publish the baud rate change to a parameter
            self.node.declare_parameter('dynamixel_baud_rate', self.baud_rate)
            self.node.set_parameter([rclpy.parameter.Parameter(
                'dynamixel_baud_rate', 
                rclpy.parameter.Parameter.Type.INTEGER,
                self.baud_rate
            )])
            self.node.get_logger().info("Dynamixel baud rate parameter updated - may require restart to take effect")
        except Exception as e:
            self.node.get_logger().warn(f"Could not update baud rate parameter: {e}")
    
    def face_data_callback(self, msg: AffectiveState):
        """Process face data received from camera_node.py"""
        try:
            # Parse the JSON data
            # data = json.loads(msg.data)
            # self.node.get_logger().info(f"Face data received: {msg}")
            data = json.loads(msg.gaze_target_v2)
            
            # Update frame dimensions
            self.frame_width = data['frame_width']
            self.frame_height = data['frame_height']
            self.center_x = self.frame_width // 2
            self.center_y = self.frame_height // 2
            
            # Update the last face data time
            self.last_face_data_time = time.time()
            
            # Process the faces if tracking is enabled
            if self.tracking_enabled and len(data['faces']) > 0:
                faces = data['faces']
                self.node.get_logger().debug(f"Received {len(faces)} faces from camera_node")
                
                # Convert to format expected by process_faces
                opencv_faces = []
                for face in faces:
                    opencv_face = face  # Already in the correct format from camera_node
                    opencv_faces.append(opencv_face)
                
                # Process the faces (without a frame)
                self.process_faces_data(opencv_faces)
            elif self.tracking_enabled and len(data['faces']) == 0:
                # No faces detected
                if not self.scanning:
                    self.start_scanning()
                self.target_face = None
                self.tracking_status.emit("No faces detected, scanning...")
            
        except Exception as e:
            self.node.get_logger().error(f"Error processing face data: {e}")
    
    def check_face_data_timeout(self):
        """Check if we've stopped receiving face data and reset if needed"""
        if time.time() - self.last_face_data_time > 5.0:  # 5 second timeout
            if self.tracking_enabled:
                self.node.get_logger().warn("Face data timeout - no data received for 5 seconds")
                self.tracking_status.emit("Face data timeout - check camera_node")
                # Start scanning if we were tracking
                if not self.scanning:
                    self.start_scanning()
                self.target_face = None
    
    def set_frame_size(self, width, height):
        """Update frame size and center coordinates"""
        self.frame_width = width
        self.frame_height = height
        self.center_x = self.frame_width // 2
        self.center_y = self.frame_height // 2
    
    def enable_tracking(self, enabled):
        """Enable or disable tracking"""
        if enabled and not self.tracking_enabled:
            # Reset PIDs when starting tracking
            self.pan_pid.reset()
            self.tilt_pid.reset()
            self.tracking_status.emit("Tracking enabled - waiting for face data")
            # Start scanning if no faces detected
            self.start_scanning()
        elif not enabled and self.tracking_enabled:
            self.tracking_status.emit("Tracking disabled")
            self.target_face = None
            self.stop_scanning()
        
        self.tracking_enabled = enabled
    
    def start_scanning(self):
        """Start scanning motion when no face is detected"""
        if not self.scanning and self.tracking_enabled:
            self.scanning = True
            # Create ROS timer instead of QTimer
            self.scan_timer = self.node.create_timer(
                0.1,  # 100ms in seconds
                self.update_scan
            )
            self.node.get_logger().info("Starting scan for faces")
    
    def stop_scanning(self):
        """Stop scanning motion"""
        if self.scanning:
            self.scanning = False
            if self.scan_timer:
                self.scan_timer.cancel()
                self.scan_timer.destroy()
                self.scan_timer = None
            self.node.get_logger().info("Stopping scan")
    
    def set_tilt_to_default(self):
        """Set tilt motor to default position"""
        tilt_position = int(self.default_tilt_angle * self.positions_per_degree)
        self.send_motor_command(self.tilt_motor_id, tilt_position)
        self.node.get_logger().info(f"Tilt set to default: {self.default_tilt_angle}")
    
    def update_scan(self):
        """Update head position during scanning"""
        if not self.scanning or not self.tracking_enabled:
            return
        
        # Only scan if no face is detected
        if self.target_face is not None:
            return
        
        # Update scanning angle
        self.current_scan_angle += self.scan_direction * self.scan_speed
        
        # Check bounds and reverse direction if needed
        if self.current_scan_angle >= self.pan_max_angle:
            self.current_scan_angle = self.pan_max_angle
            self.scan_direction = -1
        elif self.current_scan_angle <= self.pan_min_angle:
            self.current_scan_angle = self.pan_min_angle
            self.scan_direction = 1
        
        # Apply smoothing to the pan angle change
        self.target_pan_angle = self.current_scan_angle
        smoothed_angle = self.apply_smoothing(self.target_pan_angle)
        
        # Convert to motor position
        scan_position = int(smoothed_angle * self.positions_per_degree)
        
        # Send the command
        self.send_motor_command(self.pan_motor_id, scan_position)
        
        # Log current scan position
        self.node.get_logger().debug(f"Scanning at angle: {smoothed_angle:.1f}")
    
    def apply_smoothing(self, target_angle):
        """Apply smoothing to angle changes"""
        current_angle = (self.current_pan_position * self.degrees_per_position) % 360
        smoothed_angle = (self.smoothing_factor * current_angle + 
                          (1 - self.smoothing_factor) * target_angle)
        return smoothed_angle
    
    def read_motor_positions(self):
        """Read current positions from motors"""
        self.node.get_logger().info('Reading initial motor positions...')
        self.get_motor_position(self.pan_motor_id)
        self.get_motor_position(self.tilt_motor_id)
    
    def get_motor_position(self, motor_id):
        """Get position for a specific motor"""
        client = self.node.create_client(GetPosition, 'get_position')
        
        # Don't block too long waiting for service
        if not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn('Get position service not available')
            return
        
        request = GetPosition.Request()
        request.id = motor_id
        
        future = client.call_async(request)
        future.add_done_callback(
            lambda f: self.process_position_response(f, motor_id)
        )
    
    def process_position_response(self, future, motor_id):
        """Process the response from the get_position service"""
        try:
            response = future.result()
            position = response.position
            angle = (position * self.degrees_per_position) % 360
            
            if motor_id == self.pan_motor_id:
                self.current_pan_position = position
                self.current_scan_angle = angle  # Update scan angle based on current position
                self.node.get_logger().info(f'Pan motor position: {position} (angle: {angle:.1f}°)')
            elif motor_id == self.tilt_motor_id:
                self.current_tilt_position = position
                self.node.get_logger().info(f'Tilt motor position: {position} (angle: {angle:.1f}°)')
                
        except Exception as e:
            self.node.get_logger().error(f'Service call failed: {e}')
    
    def send_motor_command(self, motor_id, position):
        """Send position command to a motor"""
        msg = SetPosition()
        msg.id = motor_id
        msg.position = position
        self.position_publisher.publish(msg)
        
        # Update stored positions
        if motor_id == self.pan_motor_id:
            self.current_pan_position = position
            # Publish current pan angle for eye tracking coordination
            angle = (position * self.degrees_per_position) % 360
            angle_msg = Float32()
            angle_msg.data = float(angle)
            self.pan_angle_publisher.publish(angle_msg)
            
        elif motor_id == self.tilt_motor_id:
            self.current_tilt_position = position
            # Publish current tilt angle for eye tracking coordination
            angle = (position * self.degrees_per_position) % 360
            angle_msg = Float32()
            angle_msg.data = float(angle)
            self.tilt_angle_publisher.publish(angle_msg)
    
    def set_pid_smoothing(self, enabled):
        """Enable or disable PID smoothing"""
        self.use_pid_smoothing = enabled
        
        # Reset PIDs if toggling
        if enabled:
            self.pan_pid.reset()
            self.tilt_pid.reset()
        
        self.node.get_logger().info(f"PID smoothing {'enabled' if enabled else 'disabled'}")
    
    def send_coordinated_movement(self, pan_angle, tilt_angle):
        """Send coordinated movement commands to both motors simultaneously"""
        # Convert angles to positions
        pan_position = int(pan_angle * self.positions_per_degree)
        tilt_position = int(tilt_angle * self.positions_per_degree)
        
        # Send both commands in quick succession
        self.send_motor_command(self.pan_motor_id, pan_position)
        self.send_motor_command(self.tilt_motor_id, tilt_position)
        
        # Log the coordinated command
        self.node.get_logger().debug(
            f"Coordinated movement: Pan={pan_angle:.1f}°, Tilt={tilt_angle:.1f}°"
        )
    
    def calculate_coordinated_movement(self, face):
        """Calculate coordinated movement vector to target face"""
        # Calculate error from center of frame
        error_x = self.center_x - face['center_x']
        error_y = self.center_y - face['center_y']
        
        # Calculate vector magnitude (distance from center)
        vector_magnitude = math.sqrt(error_x**2 + error_y**2)
        
        # Check if movement is needed based on separate thresholds
        # These thresholds should match the ones in the eye tracking system
        need_pan_movement = abs(error_x) > self.pan_threshold
        need_tilt_movement = abs(error_y) > self.tilt_threshold
        
        # If neither axis needs movement, don't move the head
        # This is where we ensure exclusive movement - eyes will handle tracking within thresholds
        if not need_pan_movement and not need_tilt_movement:
            return None, None, 0, 0, 0
        
        # Current angles
        current_pan_angle = (self.current_pan_position * self.degrees_per_position) % 360
        current_tilt_angle = (self.current_tilt_position * self.degrees_per_position) % 360
        
        if self.use_pid_smoothing:
            # PID-based approach
            # Calculate the adjustments using PIDs, but only if needed
            pan_adjustment = self.pan_pid.compute(0, error_x) if need_pan_movement else 0
            tilt_adjustment = self.tilt_pid.compute(0, -error_y) if need_tilt_movement else 0
            
            # Calculate new angles
            new_pan_angle = current_pan_angle + pan_adjustment
            new_tilt_angle = current_tilt_angle + tilt_adjustment
            
            # Apply margins near limits
            margin = 5.0  # degrees from limit
            if new_tilt_angle > (self.tilt_max_angle - margin):
                factor = 1.0 - ((new_tilt_angle - (self.tilt_max_angle - margin)) / margin)
                tilt_adjustment *= max(0.3, factor)
                new_tilt_angle = current_tilt_angle + tilt_adjustment
            elif new_tilt_angle < (self.tilt_min_angle + margin):
                factor = 1.0 - (((self.tilt_min_angle + margin) - new_tilt_angle) / margin)
                tilt_adjustment *= max(0.3, factor)
                new_tilt_angle = current_tilt_angle + tilt_adjustment
            
            # Apply smoothing for less jittery movement
            smoothed_pan_angle = (self.smoothing_factor * current_pan_angle) + ((1.0 - self.smoothing_factor) * new_pan_angle)
            smoothed_tilt_angle = (self.smoothing_factor * current_tilt_angle) + ((1.0 - self.smoothing_factor) * new_tilt_angle)
            
            # Ensure angles are within valid range
            smoothed_pan_angle = max(self.pan_min_angle, min(self.pan_max_angle, smoothed_pan_angle))
            smoothed_tilt_angle = max(self.tilt_min_angle, min(self.tilt_max_angle, smoothed_tilt_angle))
            
            return smoothed_pan_angle, smoothed_tilt_angle, error_x, error_y, vector_magnitude
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
            
            # Ensure angles are within valid range
            new_pan_angle = max(self.pan_min_angle, min(self.pan_max_angle, new_pan_angle))
            new_tilt_angle = max(self.tilt_min_angle, min(self.tilt_max_angle, new_tilt_angle))
            
            # Apply smoothing for less jittery movement
            smoothed_pan_angle = (self.smoothing_factor * current_pan_angle) + ((1.0 - self.smoothing_factor) * new_pan_angle)
            smoothed_tilt_angle = (self.smoothing_factor * current_tilt_angle) + ((1.0 - self.smoothing_factor) * new_tilt_angle)
            
            return smoothed_pan_angle, smoothed_tilt_angle, error_x, error_y, vector_magnitude
    
    def process_faces_data(self, faces):
        """Process face data received from camera_node.py (no frame needed)"""
        if not self.tracking_enabled:
            return
            
        if not faces:
            # No faces detected, make sure scanning is active
            if not self.scanning:
                self.start_scanning()
            self.target_face = None
            return
        else:
            # Faces detected, stop scanning
            self.stop_scanning()
        
        # Calculate face velocities
        self.calculate_face_velocities(faces)
        
        # Select target face if we don't have one
        if self.target_face is None and faces:
            # Choose the largest face (closest to camera) or the center-most face
            largest_area = 0
            closest_to_center = float('inf')
            
            for i, face in enumerate(faces):
                # Calculate face area
                width = face['x2'] - face['x1']
                height = face['y2'] - face['y1']
                area = width * height
                
                # Calculate distance from center
                dx = face['center_x'] - self.center_x
                dy = face['center_y'] - self.center_y
                distance_from_center = (dx*dx + dy*dy) ** 0.5
                
                # Prioritize larger faces unless they're far from center
                score = distance_from_center - area * 0.02
                
                if score < closest_to_center:
                    closest_to_center = score
                    largest_area = area
                    self.target_face = i
            
            if self.target_face is not None:
                self.tracking_status.emit(f"Tracking face {self.target_face+1}/{len(faces)}")
        
        # If we have a target face, track it
        if self.target_face is not None and self.target_face < len(faces):
            face = faces[self.target_face]
            
            # Calculate coordinated movement to target
            pan_angle, tilt_angle, error_x, error_y, vector_magnitude = self.calculate_coordinated_movement(face)
            
            # Skip if no movement needed
            if pan_angle is None:
                # Face is close enough to center - no movement needed
                return
            
            # Update current scan angle for scanning continuity
            self.current_scan_angle = pan_angle
                
            # Send coordinated movement command
            current_time = time.time()
            if current_time - self.last_update_time > self.update_interval:  # Use configurable rate
                self.send_coordinated_movement(pan_angle, tilt_angle)
                self.last_update_time = current_time
                
                # Publish face velocity if available
                face_id = self.target_face
                if face_id in self.face_velocities:
                    vx, vy, magnitude = self.face_velocities[face_id]
                    velocity_msg = Vector3()
                    velocity_msg.x = float(vx)
                    velocity_msg.y = float(vy)
                    velocity_msg.z = float(magnitude)  # Use z for magnitude
                    self.velocity_publisher.publish(velocity_msg)
                    self.face_velocity.emit(vx, vy, magnitude)
                
                # Log tracking data
                self.node.get_logger().debug(
                    f"Vector: mag={vector_magnitude:.1f}, coords=({error_x:.1f}, {error_y:.1f}), " +
                    f"Pan={pan_angle:.1f}°, Tilt={tilt_angle:.1f}°"
                )
            
            # Update tracking status
            vel_info = ""
            if self.target_face in self.face_velocities:
                vx, vy, magnitude = self.face_velocities[self.target_face]
                vel_info = f", V={magnitude:.1f}"
            
            self.tracking_status.emit(f"Tracking: C={face['confidence']:.2f} D={vector_magnitude:.1f}{vel_info}")
            
        else:
            # If we lost the target face, clear the target and search again
            self.target_face = None
            self.tracking_status.emit("Target lost - searching for face")
            self.start_scanning()
    
    def calculate_face_velocities(self, faces):
        """Calculate velocities for each face based on position history"""
        current_time = time.time()
        time_delta = current_time - self.last_face_update
        
        # Skip if time delta is too small to avoid division by zero
        if time_delta < 0.01:
            return
            
        # Update face position history and calculate velocities
        for i, face in enumerate(faces):
            face_id = i  # Use index as face ID
            center_x = face['center_x']
            center_y = face['center_y']
            
            # Add current position to history
            if face_id not in self.prev_face_positions:
                self.prev_face_positions[face_id] = []
            
            # Add new position with timestamp
            self.prev_face_positions[face_id].append((center_x, center_y, current_time))
            
            # Keep only the most recent positions
            if len(self.prev_face_positions[face_id]) > self.face_positions_history:
                self.prev_face_positions[face_id].pop(0)
            
            # Need at least 2 positions to calculate velocity
            if len(self.prev_face_positions[face_id]) >= 2:
                # Use the most recent positions for velocity calculation
                prev_x, prev_y, prev_time = self.prev_face_positions[face_id][-2]
                
                # Calculate pixel displacement
                dx = center_x - prev_x
                dy = center_y - prev_y
                
                # Calculate time difference
                dt = current_time - prev_time
                
                if dt > 0:
                    # Calculate velocity in pixels per second
                    vx = dx / dt
                    vy = dy / dt
                    
                    # Calculate velocity magnitude
                    magnitude = math.sqrt(vx*vx + vy*vy)
                    
                    # Apply smoothing if we have previous velocity
                    if face_id in self.face_velocities:
                        prev_vx, prev_vy, prev_magnitude = self.face_velocities[face_id]
                        
                        # Smooth the velocity components
                        vx = self.velocity_smoothing * prev_vx + (1 - self.velocity_smoothing) * vx
                        vy = self.velocity_smoothing * prev_vy + (1 - self.velocity_smoothing) * vy
                        
                        # Recalculate magnitude after smoothing
                        magnitude = math.sqrt(vx*vx + vy*vy)
                    
                    # Store the calculated velocity
                    self.face_velocities[face_id] = (vx, vy, magnitude)
        
        # Clean up velocities for faces not in current frame
        faces_ids = set(range(len(faces)))
        for face_id in list(self.face_velocities.keys()):
            if face_id not in faces_ids:
                del self.face_velocities[face_id]
                if face_id in self.prev_face_positions:
                    del self.prev_face_positions[face_id]
        
        self.last_face_update = current_time
    
    def process_faces(self, frame, faces):
        """Process detected faces and control motors if tracking is enabled (legacy method)"""
        if not self.tracking_enabled:
            # If tracking disabled, just return the frame
            return frame
            
        if not faces:
            # No faces detected, make sure scanning is active
            if not self.scanning:
                self.start_scanning()
            self.target_face = None
            return frame
        else:
            # Faces detected, stop scanning
            self.stop_scanning()
        
        # Calculate face velocities
        self.calculate_face_velocities(faces)
        
        # Select target face if we don't have one
        if self.target_face is None and faces:
            # Choose the largest face (closest to camera) or the center-most face
            largest_area = 0
            closest_to_center = float('inf')
            
            for i, face in enumerate(faces):
                # Calculate face area
                width = face['x2'] - face['x1']
                height = face['y2'] - face['y1']
                area = width * height
                
                # Calculate distance from center
                dx = face['center_x'] - self.center_x
                dy = face['center_y'] - self.center_y
                distance_from_center = (dx*dx + dy*dy) ** 0.5
                
                # Prioritize larger faces unless they're far from center
                score = distance_from_center - area * 0.02
                
                if score < closest_to_center:
                    closest_to_center = score
                    largest_area = area
                    self.target_face = i
            
            if self.target_face is not None:
                self.tracking_status.emit(f"Tracking face {self.target_face+1}/{len(faces)}")
        
        # If we have a target face, track it
        if self.target_face is not None and self.target_face < len(faces):
            face = faces[self.target_face]
            
            # Draw an extra indicator on the target face
            cv2.rectangle(frame, 
                         (face['x1'], face['y1']), 
                         (face['x2'], face['y2']), 
                         (0, 0, 255), 2)
            
            # Draw velocity vector if available and magnitude is significant
            face_id = self.target_face
            if face_id in self.face_velocities:
                vx, vy, magnitude = self.face_velocities[face_id]
                
                # Only draw if velocity is significant
                if magnitude > self.min_velocity_display:
                    # Get face center point
                    start_x = int(face['center_x'])
                    start_y = int(face['center_y'])
                    
                    # Scale vector for display (adjustable scale factor)
                    scale = min(0.5, 50.0 / max(1.0, magnitude))  # Limit max length
                    end_x = int(start_x + vx * scale)
                    end_y = int(start_y + vy * scale)
                    
                    # Draw arrow showing velocity vector
                    cv2.arrowedLine(frame, (start_x, start_y), (end_x, end_y), 
                                   (0, 255, 255), 2, tipLength=0.3)
                    
                    # Display velocity magnitude
                    cv2.putText(frame, f"V: {magnitude:.1f} px/s", 
                               (start_x + 10, start_y - 10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            
            # Calculate error from center of frame (only care about x-axis for panning)
            error_x = self.center_x - face['center_x']
            error_y = self.center_y - face['center_y']  # Add for vertical tracking
            
            # Draw crosshair at center of frame
            cv2.line(frame, (self.center_x, 0), (self.center_x, self.frame_height), (0, 255, 0), 1)
            cv2.line(frame, (0, self.center_y), (self.frame_width, self.center_y), (0, 255, 0), 1)
            
            # Draw deadzone box
            cv2.rectangle(frame, 
                        (self.center_x - self.deadzone_x, self.center_y - self.deadzone_y),
                        (self.center_x + self.deadzone_x, self.center_y + self.deadzone_y),
                        (255, 255, 0), 1)
            
            # Only move if outside deadzone
            if abs(error_x) > self.deadzone_x:
                # Calculate PID outputs - invert error_x for correct pan direction
                pan_adjustment = self.pan_pid.compute(0, -error_x)  # Negative to match motor directions
                
                # Current positions in degrees
                current_pan_angle = (self.current_pan_position * self.degrees_per_position) % 360
                
                # Calculate new pan angle
                new_pan_angle = current_pan_angle + pan_adjustment
                
                # Ensure angle is within valid range
                new_pan_angle = max(self.pan_min_angle, min(new_pan_angle, self.pan_max_angle))
                
                # Set as target for smoothing
                self.target_pan_angle = new_pan_angle
                
                # Apply smoothing for less jittery movement
                smoothed_angle = self.apply_smoothing(self.target_pan_angle)
                
                # Update scan angle to current position
                self.current_scan_angle = smoothed_angle
                
                # Convert to motor position
                new_pan_position = int(smoothed_angle * self.positions_per_degree)
                
                # Limit update rate to prevent overwhelming motors
                current_time = time.time()
                if current_time - self.last_update_time > self.update_interval:  # Use configurable rate
                    self.send_motor_command(self.pan_motor_id, new_pan_position)
                    self.last_update_time = current_time
                    
                    # Log control values
                    self.node.get_logger().debug(
                        f"Error X: {error_x}, "
                        f"PID: {pan_adjustment:.2f}, "
                        f"Angle: {smoothed_angle:.1f}"
                    )
            
            # Add tilt control based on vertical error
            if abs(error_y) > self.deadzone_y:
                # FIXED: Invert the error_y for correct tilt behavior
                tilt_adjustment = self.tilt_pid.compute(0, -error_y)  # Invert error for correct direction
                
                # Current tilt angle in degrees
                current_tilt_angle = (self.current_tilt_position * self.degrees_per_position) % 360
                
                # Calculate new tilt angle
                new_tilt_angle = current_tilt_angle + tilt_adjustment
                
                # Ensure angle is within valid range (135=down, 225=up, 180=forward)
                new_tilt_angle = max(self.tilt_min_angle, min(new_tilt_angle, self.tilt_max_angle))
                
                # Apply stronger smoothing to tilt movement
                tilt_smoothing = 0.8  # Higher = smoother but slower
                smoothed_tilt = (tilt_smoothing * current_tilt_angle) + ((1.0 - tilt_smoothing) * new_tilt_angle)
                
                # Convert to motor position
                new_tilt_position = int(smoothed_tilt * self.positions_per_degree)
                
                # Send tilt command with rate limiting
                current_time = time.time()
                if current_time - self.last_update_time > self.update_interval:  # Use configurable rate
                    self.send_motor_command(self.tilt_motor_id, new_tilt_position)
                    
                    # Log control values
                    self.node.get_logger().debug(
                        f"Error Y: {error_y}, "
                        f"PID: {tilt_adjustment:.2f}, "
                        f"Target: {new_tilt_angle:.1f}, "
                        f"Smoothed: {smoothed_tilt:.1f}"
                    )
            
            # Show tracking info on frame
            cv2.putText(frame, f"Tracking: {face['confidence']:.2f}", (10, self.frame_height - 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            # If we lost the target face, clear the target and search again
            self.target_face = None
            self.tracking_status.emit("Target lost - searching for face")
            self.start_scanning()
        
        # Emit the processed frame
        self.frame_processed.emit(frame)
        return frame
    
    def reset_head_position(self):
        """Reset head to default position"""
        # Convert default angles to positions
        pan_position = int(self.default_pan_angle * self.positions_per_degree)
        tilt_position = int(self.default_tilt_angle * self.positions_per_degree)
        
        # Send commands
        self.send_motor_command(self.pan_motor_id, pan_position)
        self.send_motor_command(self.tilt_motor_id, tilt_position)
        
        # Reset scan angle
        self.current_scan_angle = self.default_pan_angle
        self.target_pan_angle = self.default_pan_angle
        self.target_tilt_angle = self.default_tilt_angle
        
        # Reset PID controllers
        self.pan_pid.reset()
        self.tilt_pid.reset()
        
        self.tracking_status.emit("Head position reset")


class HeadTrackingUI(QMainWindow):
    """UI for head tracking system"""
    def __init__(self, node, head_tracker):
        super().__init__()
        self.node = node
        self.head_tracker = head_tracker
        self.head_tracker.tracking_status.connect(self.update_status)
        self.head_tracker.face_velocity.connect(self.update_velocity)
        
        # Add timer to check for data timeouts
        self.timeout_timer = QTimer()
        self.timeout_timer.timeout.connect(self.head_tracker.check_face_data_timeout)
        self.timeout_timer.start(1000)  # Check every second
        
        # Velocity display
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.velocity_magnitude = 0.0
        
        self.initUI()
    
    def initUI(self):
        self.setWindowTitle('Head Tracking Control')
        self.setGeometry(100, 100, 500, 600)  # Increased height for additional controls
        
        # Create a scroll area
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        self.setCentralWidget(scroll_area)
        
        # Main widget and layout
        main_widget = QWidget()
        scroll_area.setWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        
        # Status display
        status_group = QGroupBox("Status")
        status_layout = QVBoxLayout()
        self.status_label = QLabel("Tracking disabled")
        status_layout.addWidget(self.status_label)
        
        # Add velocity status label
        self.velocity_label = QLabel("Velocity: 0.0 px/s")
        status_layout.addWidget(self.velocity_label)
        
        status_group.setLayout(status_layout)
        main_layout.addWidget(status_group)
        
        # Tracking controls
        tracking_group = QGroupBox("Tracking Controls")
        tracking_layout = QVBoxLayout()
        
        # Enable tracking checkbox
        self.tracking_checkbox = QCheckBox("Enable Head Tracking")
        self.tracking_checkbox.setChecked(False)
        self.tracking_checkbox.stateChanged.connect(self.toggle_tracking)
        tracking_layout.addWidget(self.tracking_checkbox)
        
        # Reset position button
        self.reset_button = QPushButton("Reset Head Position")
        self.reset_button.clicked.connect(self.reset_head_position)
        tracking_layout.addWidget(self.reset_button)
        
        # PID Smoothing toggle
        self.pid_smoothing_checkbox = QCheckBox("Use PID Smoothing")
        self.pid_smoothing_checkbox.setChecked(self.head_tracker.use_pid_smoothing)
        self.pid_smoothing_checkbox.stateChanged.connect(self.toggle_pid_smoothing)
        tracking_layout.addWidget(self.pid_smoothing_checkbox)
        
        # Add to main layout
        tracking_group.setLayout(tracking_layout)
        main_layout.addWidget(tracking_group)
        
        # Communication settings group
        comm_group = QGroupBox("Communication Settings")
        comm_layout = QVBoxLayout()
        
        # Update rate control
        comm_layout.addWidget(QLabel("Update Rate (Hz):"))
        update_rate_layout = QHBoxLayout()
        
        # Use a more precise spinbox for update rate
        self.update_rate_spinbox = QDoubleSpinBox()
        self.update_rate_spinbox.setRange(1.0, 100.0)
        self.update_rate_spinbox.setValue(self.head_tracker.update_rate_hz)
        self.update_rate_spinbox.setDecimals(1)
        self.update_rate_spinbox.setSingleStep(1.0)
        self.update_rate_spinbox.valueChanged.connect(self.update_rate_changed)
        
        # Also provide a slider for quick adjustments
        self.update_rate_slider = QSlider(Qt.Horizontal)
        self.update_rate_slider.setRange(10, 100)
        self.update_rate_slider.setValue(int(self.head_tracker.update_rate_hz))
        self.update_rate_slider.setTickPosition(QSlider.TicksBelow)
        self.update_rate_slider.setTickInterval(10)
        self.update_rate_slider.valueChanged.connect(self.update_rate_slider_changed)
        
        update_rate_layout.addWidget(self.update_rate_spinbox)
        update_rate_layout.addWidget(self.update_rate_slider)
        comm_layout.addLayout(update_rate_layout)
        
        # Baud rate dropdown
        comm_layout.addWidget(QLabel("Baud Rate:"))
        self.baud_rate_combo = QComboBox()
        for rate in self.head_tracker.baudrate_options:
            self.baud_rate_combo.addItem(f"{rate}", rate)
        
        # Set current value
        current_index = self.baud_rate_combo.findData(self.head_tracker.baud_rate)
        if current_index >= 0:
            self.baud_rate_combo.setCurrentIndex(current_index)
        
        self.baud_rate_combo.currentIndexChanged.connect(self.baud_rate_changed)
        comm_layout.addWidget(self.baud_rate_combo)
        
        # Add a note about baud rate
        note_label = QLabel("Note: Baud rate changes may require restarting the motor service")
        note_label.setStyleSheet("color: #666; font-size: 10px;")
        comm_layout.addWidget(note_label)
        
        comm_group.setLayout(comm_layout)
        main_layout.addWidget(comm_group)
        
        # Movement Threshold Controls
        threshold_group = QGroupBox("Movement Thresholds")
        threshold_layout = QVBoxLayout()
        
        # Pan threshold slider with value display
        threshold_layout.addWidget(QLabel("Pan Threshold (pixels):"))
        pan_threshold_layout = QHBoxLayout()
        self.pan_threshold_slider = QSlider(Qt.Horizontal)
        self.pan_threshold_slider.setRange(10, 100)  # 10-100 pixel range
        self.pan_threshold_slider.setValue(int(self.head_tracker.pan_threshold))
        self.pan_threshold_slider.setTickPosition(QSlider.TicksBelow)
        self.pan_threshold_slider.setTickInterval(10)
        self.pan_threshold_slider.valueChanged.connect(self.update_pan_threshold)
        
        self.pan_threshold_value_label = QLabel(f"{self.head_tracker.pan_threshold}")
        
        pan_threshold_layout.addWidget(self.pan_threshold_slider)
        pan_threshold_layout.addWidget(self.pan_threshold_value_label)
        threshold_layout.addLayout(pan_threshold_layout)
        
        # Tilt threshold slider with value display
        threshold_layout.addWidget(QLabel("Tilt Threshold (pixels):"))
        tilt_threshold_layout = QHBoxLayout()
        self.tilt_threshold_slider = QSlider(Qt.Horizontal)
        self.tilt_threshold_slider.setRange(5, 80)  # 5-80 pixel range (lower for vertical)
        self.tilt_threshold_slider.setValue(int(self.head_tracker.tilt_threshold))
        self.tilt_threshold_slider.setTickPosition(QSlider.TicksBelow)
        self.tilt_threshold_slider.setTickInterval(5)
        self.tilt_threshold_slider.valueChanged.connect(self.update_tilt_threshold)
        
        self.tilt_threshold_value_label = QLabel(f"{self.head_tracker.tilt_threshold}")
        
        tilt_threshold_layout.addWidget(self.tilt_threshold_slider)
        tilt_threshold_layout.addWidget(self.tilt_threshold_value_label)
        threshold_layout.addLayout(tilt_threshold_layout)
        
        threshold_group.setLayout(threshold_layout)
        main_layout.addWidget(threshold_group)
        
        # Movement Speed Controls
        speed_group = QGroupBox("Pan/Tilt Speed Settings")
        speed_layout = QVBoxLayout()
        
        # Min Pan Speed
        speed_layout.addWidget(QLabel("Min Pan Speed (deg/s):"))
        min_pan_layout = QHBoxLayout()
        self.min_pan_slider = QSlider(Qt.Horizontal)
        self.min_pan_slider.setRange(1, 10)
        self.min_pan_slider.setValue(int(self.head_tracker.min_pan_speed))
        self.min_pan_slider.valueChanged.connect(self.update_min_pan_speed)
        self.min_pan_value_label = QLabel(f"{self.head_tracker.min_pan_speed:.1f}")
        min_pan_layout.addWidget(self.min_pan_slider)
        min_pan_layout.addWidget(self.min_pan_value_label)
        speed_layout.addLayout(min_pan_layout)
        
        # Max Pan Speed
        speed_layout.addWidget(QLabel("Max Pan Speed (deg/s):"))
        max_pan_layout = QHBoxLayout()
        self.max_pan_slider = QSlider(Qt.Horizontal)
        self.max_pan_slider.setRange(10, 50)
        self.max_pan_slider.setValue(int(self.head_tracker.max_pan_speed))
        self.max_pan_slider.valueChanged.connect(self.update_max_pan_speed)
        self.max_pan_value_label = QLabel(f"{self.head_tracker.max_pan_speed:.1f}")
        max_pan_layout.addWidget(self.max_pan_slider)
        max_pan_layout.addWidget(self.max_pan_value_label)
        speed_layout.addLayout(max_pan_layout)
        
        # Min Tilt Speed
        speed_layout.addWidget(QLabel("Min Tilt Speed (deg/s):"))
        min_tilt_layout = QHBoxLayout()
        self.min_tilt_slider = QSlider(Qt.Horizontal)
        self.min_tilt_slider.setRange(1, 10)
        self.min_tilt_slider.setValue(int(self.head_tracker.min_tilt_speed))
        self.min_tilt_slider.valueChanged.connect(self.update_min_tilt_speed)
        self.min_tilt_value_label = QLabel(f"{self.head_tracker.min_tilt_speed:.1f}")
        min_tilt_layout.addWidget(self.min_tilt_slider)
        min_tilt_layout.addWidget(self.min_tilt_value_label)
        speed_layout.addLayout(min_tilt_layout)
        
        # Max Tilt Speed
        speed_layout.addWidget(QLabel("Max Tilt Speed (deg/s):"))
        max_tilt_layout = QHBoxLayout()
        self.max_tilt_slider = QSlider(Qt.Horizontal)
        self.max_tilt_slider.setRange(10, 50)
        self.max_tilt_slider.setValue(int(self.head_tracker.max_tilt_speed))
        self.max_tilt_slider.valueChanged.connect(self.update_max_tilt_speed)
        self.max_tilt_value_label = QLabel(f"{self.head_tracker.max_tilt_speed:.1f}")
        max_tilt_layout.addWidget(self.max_tilt_slider)
        max_tilt_layout.addWidget(self.max_tilt_value_label)
        speed_layout.addLayout(max_tilt_layout)
        
        speed_group.setLayout(speed_layout)
        main_layout.addWidget(speed_group)
        
        # PID tuning controls
        tuning_group = QGroupBox("PID Tuning")
        tuning_layout = QVBoxLayout()
        
        # Pan PID controls
        tuning_layout.addWidget(QLabel("Pan PID:"))
        pan_layout = QHBoxLayout()
        
        # P control
        p_layout = QVBoxLayout()
        p_layout.addWidget(QLabel("P:"))
        p_control_layout = QHBoxLayout()
        self.pan_p_slider = QSlider(Qt.Horizontal)
        self.pan_p_slider.setRange(0, 50)  # Finer control
        self.pan_p_slider.setValue(int(self.head_tracker.pan_pid.kp * 100))
        self.pan_p_slider.valueChanged.connect(self.update_pan_pid)
        self.pan_p_value_label = QLabel(f"{self.head_tracker.pan_pid.kp:.2f}")
        p_control_layout.addWidget(self.pan_p_slider)
        p_control_layout.addWidget(self.pan_p_value_label)
        p_layout.addLayout(p_control_layout)
        pan_layout.addLayout(p_layout)
        
        # I control
        i_layout = QVBoxLayout()
        i_layout.addWidget(QLabel("I:"))
        i_control_layout = QHBoxLayout()
        self.pan_i_slider = QSlider(Qt.Horizontal)
        self.pan_i_slider.setRange(0, 50)
        self.pan_i_slider.setValue(int(self.head_tracker.pan_pid.ki * 1000))
        self.pan_i_slider.valueChanged.connect(self.update_pan_pid)
        self.pan_i_value_label = QLabel(f"{self.head_tracker.pan_pid.ki:.3f}")
        i_control_layout.addWidget(self.pan_i_slider)
        i_control_layout.addWidget(self.pan_i_value_label)
        i_layout.addLayout(i_control_layout)
        pan_layout.addLayout(i_layout)
        
        # D control
        d_layout = QVBoxLayout()
        d_layout.addWidget(QLabel("D:"))
        d_control_layout = QHBoxLayout()
        self.pan_d_slider = QSlider(Qt.Horizontal)
        self.pan_d_slider.setRange(0, 50)
        self.pan_d_slider.setValue(int(self.head_tracker.pan_pid.kd * 100))
        self.pan_d_slider.valueChanged.connect(self.update_pan_pid)
        self.pan_d_value_label = QLabel(f"{self.head_tracker.pan_pid.kd:.2f}")
        d_control_layout.addWidget(self.pan_d_slider)
        d_control_layout.addWidget(self.pan_d_value_label)
        d_layout.addLayout(d_control_layout)
        pan_layout.addLayout(d_layout)
        
        tuning_layout.addLayout(pan_layout)
        
        # Tilt PID controls
        tuning_layout.addWidget(QLabel("Tilt PID:"))
        tilt_layout = QHBoxLayout()
        
        # P control
        p_layout = QVBoxLayout()
        p_layout.addWidget(QLabel("P:"))
        p_control_layout = QHBoxLayout()
        self.tilt_p_slider = QSlider(Qt.Horizontal)
        self.tilt_p_slider.setRange(0, 50)
        self.tilt_p_slider.setValue(int(self.head_tracker.tilt_pid.kp * 100))
        self.tilt_p_slider.valueChanged.connect(self.update_tilt_pid)
        self.tilt_p_value_label = QLabel(f"{self.head_tracker.tilt_pid.kp:.2f}")
        p_control_layout.addWidget(self.tilt_p_slider)
        p_control_layout.addWidget(self.tilt_p_value_label)
        p_layout.addLayout(p_control_layout)
        tilt_layout.addLayout(p_layout)
        
        # I control
        i_layout = QVBoxLayout()
        i_layout.addWidget(QLabel("I:"))
        i_control_layout = QHBoxLayout()
        self.tilt_i_slider = QSlider(Qt.Horizontal)
        self.tilt_i_slider.setRange(0, 50)
        self.tilt_i_slider.setValue(int(self.head_tracker.tilt_pid.ki * 1000))
        self.tilt_i_slider.valueChanged.connect(self.update_tilt_pid)
        self.tilt_i_value_label = QLabel(f"{self.head_tracker.tilt_pid.ki:.3f}")
        i_control_layout.addWidget(self.tilt_i_slider)
        i_control_layout.addWidget(self.tilt_i_value_label)
        i_layout.addLayout(i_control_layout)
        tilt_layout.addLayout(i_layout)
        
        # D control
        d_layout = QVBoxLayout()
        d_layout.addWidget(QLabel("D:"))
        d_control_layout = QHBoxLayout()
        self.tilt_d_slider = QSlider(Qt.Horizontal)
        self.tilt_d_slider.setRange(0, 50)
        self.tilt_d_slider.setValue(int(self.head_tracker.tilt_pid.kd * 100))
        self.tilt_d_slider.valueChanged.connect(self.update_tilt_pid)
        self.tilt_d_value_label = QLabel(f"{self.head_tracker.tilt_pid.kd:.2f}")
        d_control_layout.addWidget(self.tilt_d_slider)
        d_control_layout.addWidget(self.tilt_d_value_label)
        d_layout.addLayout(d_control_layout)
        tilt_layout.addLayout(d_layout)
        
        tuning_layout.addLayout(tilt_layout)
        
        # Smoothing slider
        tuning_layout.addWidget(QLabel("Smoothing:"))
        smoothing_layout = QHBoxLayout()
        self.smoothing_slider = QSlider(Qt.Horizontal)
        self.smoothing_slider.setRange(50, 95)  # 0.5 to 0.95
        self.smoothing_slider.setValue(int(self.head_tracker.smoothing_factor * 100))
        self.smoothing_slider.valueChanged.connect(self.update_smoothing)
        self.smoothing_value_label = QLabel(f"{self.head_tracker.smoothing_factor:.2f}")
        smoothing_layout.addWidget(self.smoothing_slider)
        smoothing_layout.addWidget(self.smoothing_value_label)
        tuning_layout.addLayout(smoothing_layout)
        
        # Scanning controls
        tuning_layout.addWidget(QLabel("Scan Speed:"))
        scan_layout = QHBoxLayout()
        self.scan_speed_slider = QSlider(Qt.Horizontal)
        self.scan_speed_slider.setRange(1, 10)
        self.scan_speed_slider.setValue(int(self.head_tracker.scan_speed))
        self.scan_speed_slider.valueChanged.connect(self.update_scan_speed)
        self.scan_speed_value_label = QLabel(f"{self.head_tracker.scan_speed:.1f}")
        scan_layout.addWidget(self.scan_speed_slider)
        scan_layout.addWidget(self.scan_speed_value_label)
        tuning_layout.addLayout(scan_layout)
        
        tuning_group.setLayout(tuning_layout)
        main_layout.addWidget(tuning_group)
    
    def update_rate_changed(self):
        """Handle update rate change from the spinbox"""
        value = self.update_rate_spinbox.value()
        # Update slider without triggering its callback
        self.update_rate_slider.blockSignals(True)
        self.update_rate_slider.setValue(int(value))
        self.update_rate_slider.blockSignals(False)
        # Update the tracking system
        self.head_tracker.set_update_rate(value)
    
    def update_rate_slider_changed(self):
        """Handle update rate change from the slider"""
        value = float(self.update_rate_slider.value())
        # Update spinbox without triggering its callback
        self.update_rate_spinbox.blockSignals(True)
        self.update_rate_spinbox.setValue(value)
        self.update_rate_spinbox.blockSignals(False)
        # Update the tracking system
        self.head_tracker.set_update_rate(value)
    
    def baud_rate_changed(self):
        """Handle baud rate change"""
        value = self.baud_rate_combo.currentData()
        if value:
            self.head_tracker.set_baud_rate(value)
    
    def update_pan_threshold(self):
        """Update pan movement threshold"""
        value = self.pan_threshold_slider.value()
        self.head_tracker.pan_threshold = value
        self.pan_threshold_value_label.setText(f"{value}")
        self.node.get_logger().info(f"Pan threshold set to {value} pixels")
    
    def update_tilt_threshold(self):
        """Update tilt movement threshold"""
        value = self.tilt_threshold_slider.value()
        self.head_tracker.tilt_threshold = value
        self.tilt_threshold_value_label.setText(f"{value}")
        self.node.get_logger().info(f"Tilt threshold set to {value} pixels")
    
    def toggle_tracking(self, state):
        """Toggle head tracking"""
        self.head_tracker.enable_tracking(bool(state))
        
        # Ensure tilt is at default position
        if bool(state):
            self.head_tracker.set_tilt_to_default()
    
    def toggle_pid_smoothing(self, state):
        """Toggle PID smoothing on/off"""
        self.head_tracker.set_pid_smoothing(bool(state))
    
    def reset_head_position(self):
        """Reset head to default position"""
        self.head_tracker.reset_head_position()
    
    def update_pan_pid(self):
        """Update PID values from sliders"""
        p_value = self.pan_p_slider.value() / 100.0
        i_value = self.pan_i_slider.value() / 1000.0
        d_value = self.pan_d_slider.value() / 100.0
        
        self.head_tracker.pan_pid.kp = p_value
        self.head_tracker.pan_pid.ki = i_value
        self.head_tracker.pan_pid.kd = d_value
        
        # Update value labels
        self.pan_p_value_label.setText(f"{p_value:.2f}")
        self.pan_i_value_label.setText(f"{i_value:.3f}")
        self.pan_d_value_label.setText(f"{d_value:.2f}")
        
        self.node.get_logger().info(f"Pan PID updated: P={p_value:.2f}, I={i_value:.3f}, D={d_value:.2f}")
    
    def update_tilt_pid(self):
        """Update tilt PID values from sliders"""
        p_value = self.tilt_p_slider.value() / 100.0
        i_value = self.tilt_i_slider.value() / 1000.0
        d_value = self.tilt_d_slider.value() / 100.0
        
        self.head_tracker.tilt_pid.kp = p_value
        self.head_tracker.tilt_pid.ki = i_value
        self.head_tracker.tilt_pid.kd = d_value
        
        # Update value labels
        self.tilt_p_value_label.setText(f"{p_value:.2f}")
        self.tilt_i_value_label.setText(f"{i_value:.3f}")
        self.tilt_d_value_label.setText(f"{d_value:.2f}")
        
        self.node.get_logger().info(f"Tilt PID updated: P={p_value:.2f}, I={i_value:.3f}, D={d_value:.2f}")
    
    def update_smoothing(self):
        """Update motion smoothing factor"""
        value = self.smoothing_slider.value() / 100.0
        self.head_tracker.smoothing_factor = value
        self.smoothing_value_label.setText(f"{value:.2f}")
        self.node.get_logger().info(f"Smoothing updated: {value:.2f}")
    
    def update_scan_speed(self):
        """Update scanning speed"""
        value = self.scan_speed_slider.value()
        self.head_tracker.scan_speed = value
        self.scan_speed_value_label.setText(f"{value:.1f}")
        self.node.get_logger().info(f"Scan speed updated: {value:.1f}")
    
    def update_min_pan_speed(self):
        """Update minimum pan speed"""
        value = float(self.min_pan_slider.value())
        self.head_tracker.min_pan_speed = value
        self.min_pan_value_label.setText(f"{value:.1f}")
        self.node.get_logger().info(f"Min pan speed set to {value:.1f} deg/s")
    
    def update_max_pan_speed(self):
        """Update maximum pan speed"""
        value = float(self.max_pan_slider.value())
        self.head_tracker.max_pan_speed = value
        self.max_pan_value_label.setText(f"{value:.1f}")
        self.node.get_logger().info(f"Max pan speed set to {value:.1f} deg/s")
    
    def update_min_tilt_speed(self):
        """Update minimum tilt speed"""
        value = float(self.min_tilt_slider.value())
        self.head_tracker.min_tilt_speed = value
        self.min_tilt_value_label.setText(f"{value:.1f}")
        self.node.get_logger().info(f"Min tilt speed set to {value:.1f} deg/s")
    
    def update_max_tilt_speed(self):
        """Update maximum tilt speed"""
        value = float(self.max_tilt_slider.value())
        self.head_tracker.max_tilt_speed = value
        self.max_tilt_value_label.setText(f"{value:.1f}")
        self.node.get_logger().info(f"Max tilt speed set to {value:.1f} deg/s")
    
    def update_status(self, status):
        """Update status label"""
        self.status_label.setText(status)
    
    def update_velocity(self, vx, vy, magnitude):
        """Update velocity display"""
        self.velocity_x = vx
        self.velocity_y = vy
        self.velocity_magnitude = magnitude
        
        # Calculate direction in degrees (0 = right, 90 = down, 180 = left, 270 = up)
        direction = math.degrees(math.atan2(vy, vx)) % 360
        direction_text = ""
        
        # Convert to cardinal direction
        if magnitude > 5.0:  # Only show direction if velocity is significant
            if 22.5 <= direction < 67.5:
                direction_text = "↘ SE"
            elif 67.5 <= direction < 112.5:
                direction_text = "↓ S"
            elif 112.5 <= direction < 157.5:
                direction_text = "↙ SW"
            elif 157.5 <= direction < 202.5:
                direction_text = "← W"
            elif 202.5 <= direction < 247.5:
                direction_text = "↖ NW"
            elif 247.5 <= direction < 292.5:
                direction_text = "↑ N"
            elif 292.5 <= direction < 337.5:
                direction_text = "↗ NE"
            else:
                direction_text = "→ E"
        
        # Update the label
        self.velocity_label.setText(f"Velocity: {magnitude:.1f} px/s {direction_text}")
    
    def closeEvent(self, event):
        """Handle window close"""
        self.head_tracker.enable_tracking(False)
        event.accept()


# Main node class
class HeadTrackingNode(Node):
    def __init__(self):
        super().__init__('head_tracking_node')
        self.get_logger().info('Head tracking node starting...')
        
        # Initialize Qt Application before creating any Qt objects
        self.app = QApplication(sys.argv)
        
        # Initialize head tracking system
        self.head_tracker = HeadTrackingSystem(self)
        
        # Initialize UI
        self.tracking_ui = HeadTrackingUI(self, self.head_tracker)
        self.tracking_ui.show()
        
        # Create a ROS thread for spinning
        self.ros_thread = threading.Thread(target=self.ros_spin)
        self.ros_thread.daemon = True
        self.ros_thread.start()
        
        # Start Qt application
        sys.exit(self.app.exec_())
    
    def ros_spin(self):
        """ROS spin in separate thread to keep Qt happy"""
        rclpy.spin(self)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = HeadTrackingNode()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main() 