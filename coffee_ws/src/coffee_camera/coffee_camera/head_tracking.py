#!/usr/bin/env python3

import rclpy
import numpy as np
import threading
import time
import sys
import cv2
import json
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from dynamixel_sdk_custom_interfaces.srv import GetPosition
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                            QLabel, QPushButton, QCheckBox, QSlider, QComboBox, QGroupBox)
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
        
        # Deadzone region where we don't move motors (to prevent jitter)
        self.deadzone_x = 60  # Increased deadzone for smoother movement
        self.deadzone_y = 30  # pixels from center
        
        # Pan/tilt motor IDs and parameters
        self.pan_motor_id = 1
        self.tilt_motor_id = 2
        
        # Motor angle limits (in degrees)
        self.pan_min_angle = 0.01  # Min pan angle (right)
        self.pan_max_angle = 180.0  # Max pan angle (left)
        self.tilt_min_angle = 135.0  # Min tilt angle (up)
        self.tilt_max_angle = 225.0  # Max tilt angle (down)
        
        # Default positions (center)
        self.default_pan_angle = 90.0   # degrees (center)
        self.default_tilt_angle = 180.0  # degrees (center)
        
        self.current_pan_position = 0
        self.current_tilt_position = 0
        
        # Scanning parameters when no face is detected
        self.scanning = False
        self.scan_direction = 1  # 1 for right-to-left, -1 for left-to-right
        self.scan_speed = 3.0    # Reduced scan speed for smoother movement
        self.scan_timer = None
        self.current_scan_angle = self.default_pan_angle
        
        # Smoothing for pan movements
        self.target_pan_angle = self.default_pan_angle
        self.smoothing_factor = 0.85  # Higher = smoother but slower response (between 0-1)
        
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
        
        # Create ROS publisher for motor control
        qos = QoSProfile(depth=10)
        self.position_publisher = self.node.create_publisher(
            SetPosition,
            'set_position',
            qos
        )
        
        # Subscribe to face detection data from camera_node.py
        self.face_subscription = self.node.create_subscription(
            String,
            'face_detection_data',
            self.face_data_callback,
            10
        )
        
        # Initialize motor positions
        self.last_update_time = time.time()
        
        # Read initial motor positions
        self.read_motor_positions()
        
        # Set tilt to default position at startup
        self.set_tilt_to_default()
        
        # Last time we received face data
        self.last_face_data_time = time.time()
        
        self.node.get_logger().info("Head tracking system initialized and ready for face data")
    
    def face_data_callback(self, msg):
        """Process face data received from camera_node.py"""
        try:
            # Parse the JSON data
            data = json.loads(msg.data)
            
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
            self.scan_timer = QTimer()
            self.scan_timer.timeout.connect(self.update_scan)
            self.scan_timer.start(100)  # Update scan position every 100ms
            self.node.get_logger().info("Starting scan for faces")
    
    def stop_scanning(self):
        """Stop scanning motion"""
        if self.scanning:
            self.scanning = False
            if self.scan_timer:
                self.scan_timer.stop()
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
        elif motor_id == self.tilt_motor_id:
            self.current_tilt_position = position
    
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
            
            # Calculate error from center of frame
            error_x = self.center_x - face['center_x']
            error_y = self.center_y - face['center_y']
            
            # Track both horizontally and vertically now
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
                if current_time - self.last_update_time > 0.05:  # 20Hz max update rate
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
                # Calculate PID output for tilt
                # FIXED: Invert the error_y for correct tilt behavior
                # When face is above center (negative error_y), tilt up (toward 225)
                # When face is below center (positive error_y), tilt down (toward 135)
                tilt_adjustment = self.tilt_pid.compute(0, -error_y)  # Inverted to match correct direction
                
                # Current tilt angle in degrees
                current_tilt_angle = (self.current_tilt_position * self.degrees_per_position) % 360
                
                # Log the current and target angles
                self.node.get_logger().debug(
                    f"Tilt: current={current_tilt_angle:.1f}, error_y={error_y}, " +
                    f"adjustment={tilt_adjustment:.2f}"
                )
                
                # Calculate new tilt angle
                new_tilt_angle = current_tilt_angle + tilt_adjustment
                
                # Ensure angle is within valid range (135=down, 225=up, 180=forward)
                new_tilt_angle = max(self.tilt_min_angle, min(new_tilt_angle, self.tilt_max_angle))
                
                # Apply a non-linear damping when close to limits to prevent oscillation
                margin = 5.0  # degrees from limit
                if new_tilt_angle > (self.tilt_max_angle - margin):
                    # Close to upper limit (looking up)
                    factor = 1.0 - ((new_tilt_angle - (self.tilt_max_angle - margin)) / margin)
                    tilt_adjustment *= max(0.3, factor)  # Slow down but don't completely stop
                    new_tilt_angle = current_tilt_angle + tilt_adjustment
                elif new_tilt_angle < (self.tilt_min_angle + margin):
                    # Close to lower limit (looking down)
                    factor = 1.0 - (((self.tilt_min_angle + margin) - new_tilt_angle) / margin)
                    tilt_adjustment *= max(0.3, factor)  # Slow down but don't completely stop
                    new_tilt_angle = current_tilt_angle + tilt_adjustment
                
                # Apply stronger smoothing to tilt movement than pan
                # Increase this factor to reduce tilt jitter
                tilt_smoothing = 0.8  # Higher = smoother but slower
                smoothed_tilt = (tilt_smoothing * current_tilt_angle) + ((1.0 - tilt_smoothing) * new_tilt_angle)
                
                # Convert to motor position
                new_tilt_position = int(smoothed_tilt * self.positions_per_degree)
                
                # Send tilt command with rate limiting
                current_time = time.time()
                if current_time - self.last_update_time > 0.05:  # 20Hz max update rate
                    self.send_motor_command(self.tilt_motor_id, new_tilt_position)
                    
                    # Log control values
                    self.node.get_logger().debug(
                        f"Error Y: {error_y}, "
                        f"PID: {tilt_adjustment:.2f}, "
                        f"Target: {new_tilt_angle:.1f}, "
                        f"Smoothed: {smoothed_tilt:.1f}"
                    )
            
            # Update tracking status
            self.tracking_status.emit(f"Tracking: C={face['confidence']:.2f} X={error_x} Y={error_y}")
            
        else:
            # If we lost the target face, clear the target and search again
            self.target_face = None
            self.tracking_status.emit("Target lost - searching for face")
            self.start_scanning()
    
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
                if current_time - self.last_update_time > 0.05:  # 20Hz max update rate
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
                if current_time - self.last_update_time > 0.05:  # 20Hz max update rate
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
        
        # Add timer to check for data timeouts
        self.timeout_timer = QTimer()
        self.timeout_timer.timeout.connect(self.head_tracker.check_face_data_timeout)
        self.timeout_timer.start(1000)  # Check every second
        
        self.initUI()
    
    def initUI(self):
        self.setWindowTitle('Head Tracking Control')
        self.setGeometry(100, 100, 400, 350)
        
        # Main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        
        # Status display
        status_group = QGroupBox("Status")
        status_layout = QVBoxLayout()
        self.status_label = QLabel("Tracking disabled")
        status_layout.addWidget(self.status_label)
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
        
        # Add to main layout
        tracking_group.setLayout(tracking_layout)
        main_layout.addWidget(tracking_group)
        
        # PID tuning controls
        tuning_group = QGroupBox("Pan Tuning")
        tuning_layout = QVBoxLayout()
        
        # Pan PID controls
        pan_layout = QHBoxLayout()
        pan_layout.addWidget(QLabel("P:"))
        self.pan_p_slider = QSlider(Qt.Horizontal)
        self.pan_p_slider.setRange(0, 50)  # Reduced range for finer control
        self.pan_p_slider.setValue(int(self.head_tracker.pan_pid.kp * 100))
        self.pan_p_slider.valueChanged.connect(self.update_pan_pid)
        pan_layout.addWidget(self.pan_p_slider)
        
        pan_layout.addWidget(QLabel("I:"))
        self.pan_i_slider = QSlider(Qt.Horizontal)
        self.pan_i_slider.setRange(0, 50)  # Reduced range for finer control
        self.pan_i_slider.setValue(int(self.head_tracker.pan_pid.ki * 100))
        self.pan_i_slider.valueChanged.connect(self.update_pan_pid)
        pan_layout.addWidget(self.pan_i_slider)
        
        pan_layout.addWidget(QLabel("D:"))
        self.pan_d_slider = QSlider(Qt.Horizontal)
        self.pan_d_slider.setRange(0, 50)  # Reduced range for finer control
        self.pan_d_slider.setValue(int(self.head_tracker.pan_pid.kd * 100))
        self.pan_d_slider.valueChanged.connect(self.update_pan_pid)
        pan_layout.addWidget(self.pan_d_slider)
        
        tuning_layout.addLayout(pan_layout)
        
        # Smoothing slider
        smoothing_layout = QHBoxLayout()
        smoothing_layout.addWidget(QLabel("Smoothing:"))
        self.smoothing_slider = QSlider(Qt.Horizontal)
        self.smoothing_slider.setRange(50, 95)  # 0.5 to 0.95
        self.smoothing_slider.setValue(int(self.head_tracker.smoothing_factor * 100))
        self.smoothing_slider.valueChanged.connect(self.update_smoothing)
        smoothing_layout.addWidget(self.smoothing_slider)
        tuning_layout.addLayout(smoothing_layout)
        
        # Scanning controls
        scan_layout = QHBoxLayout()
        scan_layout.addWidget(QLabel("Scan Speed:"))
        self.scan_speed_slider = QSlider(Qt.Horizontal)
        self.scan_speed_slider.setRange(1, 10)  # Reduced range for smoother scanning
        self.scan_speed_slider.setValue(int(self.head_tracker.scan_speed))
        self.scan_speed_slider.valueChanged.connect(self.update_scan_speed)
        scan_layout.addWidget(self.scan_speed_slider)
        tuning_layout.addLayout(scan_layout)
        
        # Add to main layout
        tuning_group.setLayout(tuning_layout)
        main_layout.addWidget(tuning_group)
    
    def toggle_tracking(self, state):
        """Toggle head tracking"""
        self.head_tracker.enable_tracking(bool(state))
        
        # Ensure tilt is at default position
        if bool(state):
            self.head_tracker.set_tilt_to_default()
    
    def reset_head_position(self):
        """Reset head to default position"""
        self.head_tracker.reset_head_position()
    
    def update_pan_pid(self):
        """Update PID values from sliders"""
        self.head_tracker.pan_pid.kp = self.pan_p_slider.value() / 100.0
        self.head_tracker.pan_pid.ki = self.pan_i_slider.value() / 100.0
        self.head_tracker.pan_pid.kd = self.pan_d_slider.value() / 100.0
        self.node.get_logger().info(f"Pan PID updated: P={self.head_tracker.pan_pid.kp:.2f}, I={self.head_tracker.pan_pid.ki:.2f}, D={self.head_tracker.pan_pid.kd:.2f}")
    
    def update_smoothing(self):
        """Update motion smoothing factor"""
        self.head_tracker.smoothing_factor = self.smoothing_slider.value() / 100.0
        self.node.get_logger().info(f"Smoothing updated: {self.head_tracker.smoothing_factor:.2f}")
    
    def update_scan_speed(self):
        """Update scanning speed"""
        self.head_tracker.scan_speed = self.scan_speed_slider.value()
        self.node.get_logger().info(f"Scan speed updated: {self.head_tracker.scan_speed}")
    
    def update_status(self, status):
        """Update status label"""
        self.status_label.setText(status)
    
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