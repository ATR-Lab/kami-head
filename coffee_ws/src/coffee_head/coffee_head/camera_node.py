#!/usr/bin/env python3

import sys
import cv2
import rclpy
import time
import numpy as np
import threading
import os
import subprocess
import json
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QLabel, QPushButton, QComboBox, QHBoxLayout, QCheckBox, QMessageBox
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from std_msgs.msg import Float32MultiArray, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Models directory for face detection models
MODELS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models")
os.makedirs(MODELS_DIR, exist_ok=True)


class HeadController:
    """Controls head servos based on face position without PyQt5 dependencies"""
    def __init__(self, node):
        self.node = node
        
        try:
            # Create publisher for head position commands
            self.head_pub = node.create_publisher(Float32MultiArray, 'head_position', 10)
    
            # Current head position (pan, tilt)
            self.current_position = [0.0, 0.0]  # [pan, tilt] in degrees
            
            # Target position 
            self.target_position = [0.0, 0.0]
            
            # Movement parameters (increased from original)
            self.max_speed = 90.0  # degrees per second (increased for faster response)
            self.min_speed = 5.0   # minimum speed to avoid very slow movements
            self.acceleration = 60.0  # degrees per second^2 (increased)
            self.current_speed = [0.0, 0.0]  # Current speed for pan and tilt
            
            # Separate parameters for pan/tilt control (axis 0=pan, 1=tilt)
            # Tilt generally needs slower, more careful control
            self.max_speed_tilt = 80.0  # degrees per second for tilt (increased for better response)
            self.damping_distance_tilt = 15.0  # degrees - for tilt
            self.approach_factor_tilt = 0.6  # Controls how aggressively to approach target (lower = more aggressive)
            
            # Enhanced parameters for smoother motion
            self.damping_distance = [20.0, 15.0]  # [pan, tilt] - tilt has shorter damping distance
            self.approach_factor = [0.7, 0.6]  # Controls how aggressively to approach target
            
            # Improved PID control parameters - separate for pan and tilt
            # Make tilt more responsive with higher gain
            self.kp = [0.9, 0.85]  # Proportional gain [pan, tilt] - increased for tilt
            self.kd = [0.4, 0.4]  # Derivative gain [pan, tilt] - lowered damping for tilt
            self.ki = [0.05, 0.06]  # Integral gain [pan, tilt] - increased for tilt
            self.prev_error = [0.0, 0.0]
            self.integral = [0.0, 0.0]  # Integral term for slow error correction
            self.max_integral = [10.0, 10.0]  # Limit integral windup [pan, tilt]
            
            # Prediction for motion (anticipatory control)
            self.target_history = [[], []]  # Store recent target positions
            self.history_size = 5  # Number of past targets to remember
            self.prev_targets = [[0.0, 0.0], [0.0, 0.0]]  # Last two targets for velocity estimation
            self.target_velocity = [0.0, 0.0]  # Estimated target velocity
            
            # Dead zone (ignore very small movements)
            self.dead_zone = [0.2, 0.1]  # [pan, tilt] degrees - smaller dead zone for tilt
            
            # Update rate
            self.update_interval = 0.02  # 50Hz update rate
            self.last_update_time = time.time()
            
            # Movement thresholds from Pimoroni implementation - reduce for tilt
            self.move_threshold = [3.0, 1.0]  # [pan, tilt] - Reduced threshold for tilt
            
            # Limits for the head movement
            self.pan_limits = [-90.0, 90.0]   # Minimum and maximum pan angles
            self.tilt_limits = [-45.0, 30.0]  # Minimum and maximum tilt angles
            
            # Flag to determine if we're using internal control (True) or delegating to head_tracking.py (False)
            self.use_internal_control = True
            
            # Start control thread
            self.running = True
            self.control_thread = threading.Thread(target=self._control_loop)
            self.control_thread.daemon = True
            self.control_thread.start()
            
            self.node.get_logger().info("Enhanced head controller initialized with improved tilt control")
            
        except Exception as e:
            self.node.get_logger().error(f"Error initializing head controller: {str(e)}")
            self.running = False
            raise  # Re-raise so we don't silently fail
    
    def set_target(self, pan, tilt):
        """Set target position for the head with smoother transitions"""
        # Constrain to valid range 
        pan = max(self.pan_limits[0], min(self.pan_limits[1], pan))
        tilt = max(self.tilt_limits[0], min(self.tilt_limits[1], tilt))
        
        # Store previous target for velocity calculation
        self.prev_targets[1] = self.prev_targets[0]
        self.prev_targets[0] = self.target_position.copy()
        
        # Update target position
        self.target_position = [pan, tilt]
        
        # Add to history for prediction
        self.target_history[0].append(pan)
        self.target_history[1].append(tilt)
        
        # Keep history at fixed size
        if len(self.target_history[0]) > self.history_size:
            self.target_history[0].pop(0)
            self.target_history[1].pop(0)
        
        # Calculate target velocity for predictive control
        if len(self.target_history[0]) >= 2:
            dt = self.update_interval  # Approximate time between updates
            self.target_velocity[0] = (self.target_history[0][-1] - self.target_history[0][-2]) / dt
            self.target_velocity[1] = (self.target_history[1][-1] - self.target_history[1][-2]) / dt
    
    def stop(self):
        """Stop the control thread"""
        self.running = False
        if self.control_thread.is_alive():
            self.control_thread.join(timeout=1.0)
    
    def _predict_target_position(self, axis, lookahead_time):
        """Predict where the target will be in the near future based on recent movement"""
        # Simple linear prediction based on current velocity
        predicted_pos = self.target_position[axis] + self.target_velocity[axis] * lookahead_time
        
        # Constrain prediction to valid ranges
        if axis == 0:  # pan
            return max(self.pan_limits[0], min(self.pan_limits[1], predicted_pos))
        else:  # tilt
            return max(self.tilt_limits[0], min(self.tilt_limits[1], predicted_pos))
    
    def _calculate_adaptive_speed(self, error, axis, dt):
        """Calculate appropriate speed based on distance to target with enhanced PID control"""
        # Skip tiny movements (dead zone)
        if abs(error) < self.dead_zone[axis]:
            self.integral[axis] = 0  # Reset integral when in dead zone
            return 0.0
        
        # Calculate PID terms
        p_term = self.kp[axis] * error
        
        # Integral term with anti-windup
        self.integral[axis] += error * dt
        self.integral[axis] = max(-self.max_integral[axis], min(self.max_integral[axis], self.integral[axis]))
        i_term = self.ki[axis] * self.integral[axis]
        
        # Derivative term (damping)
        d_term = self.kd[axis] * (error - self.prev_error[axis]) / dt if dt > 0 else 0
        self.prev_error[axis] = error
        
        # Calculate raw speed from PID controller
        raw_speed = p_term + i_term + d_term
        
        # Distance-based speed limiting to prevent overshooting
        distance = abs(error)
        
        # Apply stronger damping when getting close to target
        if distance < self.damping_distance[axis]:
            # Progressive speed reduction - slower as we get closer (non-linear)
            speed_factor = (distance / self.damping_distance[axis]) ** self.approach_factor[axis]
            raw_speed *= speed_factor
        
        # Apply predictive control for moving targets
        # Add a component to compensate for target movement
        if abs(self.target_velocity[axis]) > 1.0:  # Only if target is moving significantly
            # Add a component that tends to match target velocity
            velocity_match = self.target_velocity[axis] * 0.8  # 80% of target velocity
            raw_speed += velocity_match
        
        # Apply speed limits - different for pan vs tilt
        max_speed = self.max_speed if axis == 0 else self.max_speed_tilt
        
        if abs(raw_speed) < self.min_speed and abs(error) > 1.0:
            # Ensure we move at least at min_speed if not very close to target
            raw_speed = self.min_speed if raw_speed > 0 else -self.min_speed
        elif abs(raw_speed) > max_speed:
            # Cap at max_speed
            raw_speed = max_speed if raw_speed > 0 else -max_speed
        
        return raw_speed
    
    def _control_loop(self):
        """Main control loop for head movement"""
        while self.running:
            current_time = time.time()
            dt = current_time - self.last_update_time
            
            if dt < self.update_interval:
                # Wait until update interval has passed
                time.sleep(0.001)
                continue
            
            # Only process if we're using internal control
            if self.use_internal_control:
                # Calculate errors from predicted target positions (look ahead by a small amount)
                lookahead_time = 0.1  # 100ms prediction
                predicted_pan = self._predict_target_position(0, lookahead_time)
                predicted_tilt = self._predict_target_position(1, lookahead_time)
                
                pan_error = predicted_pan - self.current_position[0]
                tilt_error = predicted_tilt - self.current_position[1]
                
                # Log significant tilt errors for debugging
                if abs(tilt_error) > 5.0:
                    self.node.get_logger().debug(
                        f"Tilt: target={predicted_tilt:.2f}, current={self.current_position[1]:.2f}, " + 
                        f"error={tilt_error:.2f}"
                    )
                
                # Only move if error exceeds threshold (reduces jitter)
                if abs(pan_error) < self.move_threshold[0]:
                    pan_error = 0
                    self.integral[0] = 0  # Reset integral term
                
                if abs(tilt_error) < self.move_threshold[1]:
                    tilt_error = 0
                    self.integral[1] = 0  # Reset integral term
                    
                # Calculate adaptive speeds with time-aware calculation
                self.current_speed[0] = self._calculate_adaptive_speed(pan_error, 0, dt)
                self.current_speed[1] = self._calculate_adaptive_speed(tilt_error, 1, dt)
                
                # Update positions based on calculated speeds
                self.current_position[0] += self.current_speed[0] * dt
                self.current_position[1] += self.current_speed[1] * dt
                
                # Ensure positions stay within limits
                self.current_position[0] = max(self.pan_limits[0], min(self.pan_limits[1], self.current_position[0]))
                self.current_position[1] = max(self.tilt_limits[0], min(self.tilt_limits[1], self.current_position[1]))
                
                # Publish head position
                msg = Float32MultiArray()
                msg.data = self.current_position
                self.head_pub.publish(msg)
                
                # Add debugging for tilt publishing
                if abs(self.current_speed[1]) > 0.1:  # Only log when actually moving
                    self.node.get_logger().debug(
                        f"Publishing tilt: pos={self.current_position[1]:.2f}, speed={self.current_speed[1]:.2f}"
                    )
            
            # Update last update time
            self.last_update_time = current_time
            
            # Small sleep to prevent tight loops
            time.sleep(0.001)


class FrameGrabber(QObject):
    """Dedicated thread for frame capture to improve performance"""
    frame_ready = pyqtSignal(np.ndarray)
    error = pyqtSignal(str)
    
    def __init__(self, node=None):
        super().__init__()
        self.node = node
        self.camera = None
        self.camera_index = 0
        self.running = False
        self.lock = threading.Lock()
        self.capture_thread = None
        self.frame_width = 1280  # Default to 720p (16:9)
        self.frame_height = 720
        self.high_quality = False
        self.backend = cv2.CAP_ANY  # Default backend
        
        # Face detection
        self.enable_face_detection = True
        self.face_detector = None
        self.face_net = None
        self.face_confidence_threshold = 0.5
        
        # Smoothing for face detection
        self.prev_faces = []
        self.smoothing_factor = 0.7  # Higher value = more smoothing
        self.smoothing_frames = 5    # Number of frames to average
        
        # ROS publishers for face data and images
        if self.node:
            self.face_pub = node.create_publisher(String, 'face_detection_data', 10)
            self.frame_pub = node.create_publisher(Image, 'camera_frame', 10)
            self.face_image_pub = node.create_publisher(Image, 'face_images', 10)
            self.bridge = CvBridge()
            
            # Face recognition subscription
            self.face_recognition_sub = node.create_subscription(
                String,
                'face_recognition_data',
                self.face_recognition_callback,
                10
            )
            
        # Face recognition data
        self.face_ids = {}  # Map of face index to recognized face ID
        self.last_recognition_time = 0
        self.recognition_timeout = 3.0  # Clear recognition data after 3 seconds
        
        # Head tracking
        self.head_controller = None
        self.tracking_enabled = False
        self.target_face_index = 0  # Track the first face by default
        
        self.init_face_detector()
    
    def init_head_controller(self):
        """Initialize head controller if node is available"""
        if self.node and self.head_controller is None:
            try:
                self.head_controller = HeadController(self.node)
                self.node.get_logger().info("Head controller initialized successfully")
            except Exception as e:
                self.node.get_logger().error(f"Error initializing head controller: {e}")
                self.tracking_enabled = False
    
    def enable_tracking(self, enabled):
        """Enable or disable face tracking"""
        self.tracking_enabled = enabled
        if enabled and self.head_controller is None and self.node:
            self.init_head_controller()
    
    def face_recognition_callback(self, msg):
        """Process face recognition data from face recognition node"""
        try:
            recognition_data = json.loads(msg.data)
            self.last_recognition_time = time.time()
            
            # Clear existing face IDs
            self.face_ids = {}
            
            # Process each recognized face
            for face in recognition_data.get('faces', []):
                face_id = face.get('id')
                position = face.get('position', {})
                center_x = int(position.get('center_x', 0))
                center_y = int(position.get('center_y', 0))
                
                # Find the corresponding detected face
                # Match based on position
                best_match_idx = -1
                best_match_dist = float('inf')
                
                for i, detected_face in enumerate(self.prev_faces[-1] if self.prev_faces else []):
                    # Calculate distance between centers
                    dx = float(detected_face.get('center_x', 0)) - center_x
                    dy = float(detected_face.get('center_y', 0)) - center_y
                    dist = (dx**2 + dy**2)**0.5
                    
                    if dist < best_match_dist and dist < 100:  # Max 100px distance for a match
                        best_match_dist = dist
                        best_match_idx = i
                
                if best_match_idx >= 0:
                    # Store the ID with this face index
                    self.face_ids[best_match_idx] = {
                        'id': str(face_id),
                        'confidence': float(face.get('confidence', 0.0)),
                        'is_new': bool(face.get('is_new', False))
                    }
            
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error processing face recognition data: {e}")
                import traceback
                traceback.print_exc()
    
    def track_face(self, face):
        """Track a face with the head controller"""
        if not self.head_controller or not self.tracking_enabled:
            return
            
        try:
            # Convert face position to pan/tilt angles
            center_x = float(face['center_x'])
            center_y = float(face['center_y'])
            
            # Calculate normalized coordinates (-1 to 1)
            norm_x = (center_x / float(self.frame_width)) * 2 - 1
            norm_y = (center_y / float(self.frame_height)) * 2 - 1
            
            # Invert Y-axis (positive down in image, positive up in tilt)
            norm_y = -norm_y
            
            # Scale to head angles
            max_pan_angle = 80.0
            max_tilt_angle = 30.0
            
            pan_angle = float(norm_x * max_pan_angle)
            tilt_angle = float(norm_y * max_tilt_angle)
            
            # Send to head controller
            self.head_controller.set_target(pan_angle, tilt_angle)
            
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error tracking face: {e}")
                import traceback
                traceback.print_exc()
    
    def publish_face_data(self, faces):
        """Publish face detection data for other nodes"""
        if not self.node or not faces:
            return
            
        # Create JSON with face data - convert NumPy types to Python native types
        face_data = {
            "timestamp": float(time.time()),
            "frame_width": int(self.frame_width),
            "frame_height": int(self.frame_height),
            "faces": [
                {
                    "x1": int(face["x1"]),
                    "y1": int(face["y1"]),
                    "x2": int(face["x2"]),
                    "y2": int(face["y2"]),
                    "center_x": int(face["center_x"]),
                    "center_y": int(face["center_y"]),
                    "confidence": float(face["confidence"]),
                    "id": str(face.get("id", "Unknown"))
                }
                for face in faces
            ]
        }
        
        # Publish
        msg = String()
        msg.data = json.dumps(face_data)
        self.face_pub.publish(msg)
    
    def publish_face_images(self, frame, faces):
        """Extract and publish individual face images"""
        if not self.node:
            return
            
        try:
            for i, face in enumerate(faces):
                # Extract face with margin
                margin = 0.2  # 20%
                
                # Get dimensions
                x1, y1 = face['x1'], face['y1']
                x2, y2 = face['x2'], face['y2']
                h, w = frame.shape[:2]
                
                # Calculate margin
                margin_x = int((x2 - x1) * margin)
                margin_y = int((y2 - y1) * margin)
                
                # Apply margin
                x1_margin = max(0, x1 - margin_x)
                y1_margin = max(0, y1 - margin_y)
                x2_margin = min(w, x2 + margin_x)
                y2_margin = min(h, y2 + margin_y)
                
                # Extract face
                face_img = frame[y1_margin:y2_margin, x1_margin:x2_margin]
                
                # Skip if too small
                if face_img.size == 0 or face_img.shape[0] < 30 or face_img.shape[1] < 30:
                    continue
                    
                # Resize
                face_img = cv2.resize(face_img, (150, 150))
                
                # Publish
                face_msg = self.bridge.cv2_to_imgmsg(face_img, encoding="bgr8")
                face_msg.header.stamp = self.node.get_clock().now().to_msg()
                face_msg.header.frame_id = f"face_{i}"
                self.face_image_pub.publish(face_msg)
                
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error publishing face images: {e}")
    
    def publish_frame(self, frame):
        """Publish camera frame to ROS topics"""
        if not self.node:
            return
            
        try:
            frame_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            frame_msg.header.stamp = self.node.get_clock().now().to_msg()
            self.frame_pub.publish(frame_msg)
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error publishing frame: {e}")
    
    def init_face_detector(self):
        """Initialize the OpenCV DNN face detector"""
        try:
            # Try to get the models from disk, or download them if not present
            model_file = os.path.join(MODELS_DIR, "opencv_face_detector_uint8.pb")
            config_file = os.path.join(MODELS_DIR, "opencv_face_detector.pbtxt")
            
            # Download the model files if they don't exist
            if not os.path.exists(model_file) or not os.path.exists(config_file):
                self.download_face_model(model_file, config_file)
            
            # Load the DNN face detector
            self.face_net = cv2.dnn.readNet(model_file, config_file)
            
            # Switch to a more accurate backend if available
            if cv2.cuda.getCudaEnabledDeviceCount() > 0:
                self.face_net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
                self.face_net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
            else:
                self.face_net.setPreferableBackend(cv2.dnn.DNN_BACKEND_DEFAULT)
                self.face_net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            
            print("Face detector (OpenCV DNN) initialized successfully")
        except Exception as e:
            print(f"Error initializing face detector: {e}")
            self.face_net = None
    
    def download_face_model(self, model_file, config_file):
        """Download the face detection model if needed"""
        try:
            # Model URLs
            model_url = "https://github.com/spmallick/learnopencv/raw/refs/heads/master/AgeGender/opencv_face_detector_uint8.pb"
            config_url = "https://raw.githubusercontent.com/spmallick/learnopencv/refs/heads/master/AgeGender/opencv_face_detector.pbtxt"
            
            # Download the files
            import urllib.request
            print("Downloading face detection model...")
            urllib.request.urlretrieve(model_url, model_file)
            urllib.request.urlretrieve(config_url, config_file)
            print("Face detection model downloaded successfully")
        except Exception as e:
            print(f"Error downloading face model: {e}")
            raise
    
    def start(self, camera_index, backend=cv2.CAP_ANY):
        with self.lock:
            if self.running:
                self.stop()
            
            self.camera_index = camera_index
            self.backend = backend
            self.running = True
            self.capture_thread = threading.Thread(target=self._capture_loop)
            self.capture_thread.daemon = True
            self.capture_thread.start()
    
    def stop(self):
        with self.lock:
            self.running = False
            if self.capture_thread:
                if self.camera and self.camera.isOpened():
                    self.camera.release()
                    self.camera = None
                self.capture_thread.join(timeout=1.0)
                self.capture_thread = None
    
    def set_quality(self, high_quality):
        """Toggle between low resolution and high resolution"""
        with self.lock:
            if high_quality:
                self.frame_width = 1920
                self.frame_height = 1080
            else:
                # Keep these values low to reduce latency
                self.frame_width = 1280
                self.frame_height = 720
            self.high_quality = high_quality
            
            # Re-initialize the camera with the new settings
            if self.camera and self.camera.isOpened():
                self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
                self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
    
    def toggle_face_detection(self, enable):
        """Enable or disable face detection"""
        with self.lock:
            self.enable_face_detection = enable
            if enable and self.face_net is None:
                self.init_face_detector()
            
            # Reset face tracking when toggling
            self.prev_faces = []
    
    def detect_faces_dnn(self, frame):
        """Detect faces using OpenCV's DNN-based face detector"""
        if self.face_net is None:
            return []
        
        # Get frame dimensions
        h, w = frame.shape[:2]
        
        # Prepare input blob for the network
        blob = cv2.dnn.blobFromImage(frame, 1.0, (300, 300), [104, 117, 123], False, False)
        self.face_net.setInput(blob)
        
        # Run forward pass
        detections = self.face_net.forward()
        
        # Parse detections
        faces = []
        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > self.face_confidence_threshold:
                # Get face bounding box
                x1 = int(detections[0, 0, i, 3] * w)
                y1 = int(detections[0, 0, i, 4] * h)
                x2 = int(detections[0, 0, i, 5] * w)
                y2 = int(detections[0, 0, i, 6] * h)
                
                # Make sure the coordinates are within the frame
                x1 = max(0, min(x1, w-1))
                y1 = max(0, min(y1, h-1))
                x2 = max(0, min(x2, w-1))
                y2 = max(0, min(y2, h-1))
                
                if x2 > x1 and y2 > y1:  # Valid face
                    faces.append({
                        'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
                        'center_x': (x1 + x2) // 2,
                        'center_y': (y1 + y2) // 2,
                        'radius': max((x2 - x1), (y2 - y1)) // 2,
                        'confidence': confidence
                    })
        
        return faces
    
    def smooth_face_detections(self, faces):
        """Apply temporal smoothing to face detections to reduce flickering"""
        if not faces:
            # If no faces detected in current frame but we have previous faces,
            # decay them but keep showing them for a while
            if self.prev_faces:
                # Slowly reduce confidence of previous faces
                for face in self.prev_faces:
                    face['confidence'] *= 0.8  # Decay factor
                
                # Remove faces with very low confidence
                self.prev_faces = [f for f in self.prev_faces if f['confidence'] > 0.2]
                return self.prev_faces
            return []
        
        # If we have new faces, smoothly transition to them
        if not self.prev_faces:
            # First detection, just use it
            self.prev_faces = faces
            return faces
        
        # Try to match new faces with previous faces
        new_faces = []
        for new_face in faces:
            # Find closest previous face
            best_match = None
            min_distance = float('inf')
            
            for i, prev_face in enumerate(self.prev_faces):
                # Calculate distance between centers
                dx = new_face['center_x'] - prev_face['center_x']
                dy = new_face['center_y'] - prev_face['center_y']
                distance = (dx*dx + dy*dy) ** 0.5
                
                if distance < min_distance:
                    min_distance = distance
                    best_match = i
            
            # If we found a close enough match, smooth the transition
            if best_match is not None and min_distance < 100:  # Threshold distance
                prev_face = self.prev_faces[best_match]
                
                # Smooth position and size
                smoothed_face = {
                    'center_x': int(self.smoothing_factor * prev_face['center_x'] + 
                                    (1 - self.smoothing_factor) * new_face['center_x']),
                    'center_y': int(self.smoothing_factor * prev_face['center_y'] + 
                                    (1 - self.smoothing_factor) * new_face['center_y']),
                    'radius': int(self.smoothing_factor * prev_face['radius'] + 
                                 (1 - self.smoothing_factor) * new_face['radius']),
                    'confidence': new_face['confidence']
                }
                
                # Calculate new bounding box from smoothed center and radius
                r = smoothed_face['radius']
                cx = smoothed_face['center_x']
                cy = smoothed_face['center_y']
                smoothed_face['x1'] = cx - r
                smoothed_face['y1'] = cy - r
                smoothed_face['x2'] = cx + r
                smoothed_face['y2'] = cy + r
                
                new_faces.append(smoothed_face)
                # Remove the matched face to prevent double matching
                self.prev_faces.pop(best_match)
            else:
                # No match, add as new face
                new_faces.append(new_face)
        
        # Add any remaining unmatched previous faces with decayed confidence
        for face in self.prev_faces:
            face['confidence'] *= 0.5  # Faster decay for unmatched faces
            if face['confidence'] > 0.3:  # Only keep if still confident enough
                new_faces.append(face)
        
        # Update previous faces for next frame
        self.prev_faces = new_faces
        return new_faces
    
    def draw_face_circles(self, frame, faces):
        """Draw transparent circles over detected faces with IDs"""
        if not faces:
            return frame
        
        # Create an overlay for transparency
        overlay = frame.copy()
        
        # Draw circles and face data on overlay
        for i, face in enumerate(faces):
            # Get face ID if available
            face_id = face.get('id', 'Unknown')
            
            # Choose color based on face ID
            if face_id != 'Unknown':
                # Use different color for recognized faces
                color = (0, 200, 255)  # Orange for recognized faces
            else:
                color = (0, 255, 0)  # Green for detected faces
            
            # Draw circle on overlay
            cv2.circle(overlay, 
                      (face['center_x'], face['center_y']), 
                      face['radius'], 
                      color, 
                      -1)
            
            # Draw rectangle around face
            cv2.rectangle(frame, (face['x1'], face['y1']), (face['x2'], face['y2']), color, 2)
            
            # Display face ID and confidence
            face_conf = face.get('confidence', 0.0)
            id_text = f"ID: {face_id}" if face_id != 'Unknown' else "Unknown"
            conf_text = f"Conf: {face_conf:.2f}"
            
            cv2.putText(frame, id_text, (face['x1'], face['y1'] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            cv2.putText(frame, conf_text, (face['x1'], face['y1'] + face['height'] if 'height' in face else (face['y2']-face['y1']) + 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # Highlight tracked face
            if i == self.target_face_index and self.tracking_enabled:
                # Draw special marker for tracked face
                cv2.circle(frame, (face['center_x'], face['center_y']), 5, (0, 0, 255), -1)
                cv2.putText(frame, "TRACKING", (face['center_x'] - 40, face['center_y'] - 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        # Blend the overlay with the original frame for transparency
        alpha = 0.3  # Transparency factor
        cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)
        
        # Add indicator text if faces detected
        cv2.putText(frame, f"Faces: {len(faces)}", (10, 70), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Display number of recognized faces
        recog_count = len([f for f in faces if f.get('id', 'Unknown') != 'Unknown'])
        if recog_count > 0:
            cv2.putText(frame, f"Recognized: {recog_count}/{len(faces)}", (10, 110), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return frame
    
    def _capture_loop(self):
        try:
            # Try different backends if the default doesn't work
            backends_to_try = [
                (cv2.CAP_V4L2, "V4L2"),        # Linux V4L2
                (cv2.CAP_GSTREAMER, "GStreamer"),  # GStreamer
                (cv2.CAP_ANY, "Auto-detect")    # Let OpenCV choose
            ]
            
            # Start with the specified backend
            if self.backend != cv2.CAP_ANY:
                backends_to_try.insert(0, (self.backend, "User selected"))
            
            success = False
            error_msg = ""
            
            for backend, backend_name in backends_to_try:
                try:
                    # Try to open with this backend
                    if backend == cv2.CAP_ANY:
                        self.camera = cv2.VideoCapture(self.camera_index)
                    else:
                        self.camera = cv2.VideoCapture(self.camera_index, backend)
                    
                    if not self.camera.isOpened():
                        error_msg = f"Could not open camera {self.camera_index} with {backend_name} backend"
                        continue
                    
                    # If we got here, the camera opened successfully
                    success = True
                    print(f"Successfully opened camera with {backend_name} backend")
                    break
                    
                except Exception as e:
                    error_msg = f"Error opening camera with {backend_name} backend: {str(e)}"
                    continue
            
            if not success:
                self.error.emit(f"Failed to open camera: {error_msg}")
                return
            
            # Set camera properties - prefer speed over resolution
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            
            # Set camera buffer size to 1 for minimal latency
            self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            # Try to set camera properties for better performance
            try:
                self.camera.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # Disable autofocus
                self.camera.set(cv2.CAP_PROP_FPS, 30)       # Request 30 FPS
            except:
                # Some cameras don't support these properties
                pass
            
            # FPS tracking
            frame_count = 0
            start_time = time.time()
            fps = 0
            
            # Face detection timing
            face_detection_interval = 2  # Process every N frames
            frame_index = 0
            last_faces = []
            
            # Initialize head controller if tracking enabled
            if self.tracking_enabled and self.node:
                self.init_head_controller()
            
            while self.running:
                ret, frame = self.camera.read()
                
                if not ret:
                    time.sleep(0.01)  # Short sleep to prevent CPU overuse
                    continue
                
                # Process face detection (only on some frames to maintain performance)
                frame_index += 1
                if self.enable_face_detection:
                    if frame_index % face_detection_interval == 0:
                        # Only run detection periodically
                        faces = self.detect_faces_dnn(frame)
                        # Smooth the face positions to reduce flickering
                        last_faces = self.smooth_face_detections(faces)
                        
                        # Check if recognition data is stale
                        if time.time() - self.last_recognition_time > self.recognition_timeout:
                            self.face_ids = {}  # Clear stale data
                        
                        # Add face IDs if available
                        for i, face in enumerate(last_faces):
                            if i in self.face_ids:
                                face['id'] = self.face_ids[i]['id']
                            else:
                                face['id'] = 'Unknown'
                        
                        # Publish face data for other nodes
                        if self.node:
                            self.publish_face_data(last_faces)
                            self.publish_face_images(frame, last_faces)
                        
                        # Update face tracking if enabled
                        if self.tracking_enabled and self.head_controller and last_faces and len(last_faces) > self.target_face_index:
                            self.track_face(last_faces[self.target_face_index])
                    
                    # Draw smoothed faces on every frame
                    if last_faces:
                        frame = self.draw_face_circles(frame, last_faces)
                
                # Add FPS text to frame
                frame_count += 1
                if frame_count >= 10:  # Update FPS every 10 frames
                    current_time = time.time()
                    elapsed = current_time - start_time
                    fps = frame_count / elapsed if elapsed > 0 else 0
                    frame_count = 0
                    start_time = current_time
                
                # Draw FPS on the frame
                cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                # Publish frame to ROS topic
                if self.node:
                    self.publish_frame(frame)
                
                # Emit the frame signal for Qt UI
                self.frame_ready.emit(frame)
                
                # Short sleep to prevent tight loops
                time.sleep(0.001)
                
        except Exception as e:
            self.error.emit(f"Error in capture thread: {str(e)}")
        finally:
            if self.camera and self.camera.isOpened():
                self.camera.release()
                self.camera = None


class CameraViewer(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.frame_grabber = FrameGrabber(node)
        self.frame_grabber.frame_ready.connect(self.process_frame)
        self.frame_grabber.error.connect(self.handle_camera_error)
        self.high_quality = False
        self.face_detection_enabled = True
        self.tracking_enabled = False
        self.initUI()
        self.check_video_devices()
        self.scan_cameras()
        
    def initUI(self):
        self.setWindowTitle('Coffee Camera - Webcam Viewer')
        self.setGeometry(100, 100, 800, 600)
        
        # Main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        
        # Camera feed display
        self.image_label = QLabel("No camera feed available")
        self.image_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(self.image_label)
        
        # Controls layout
        controls_layout = QHBoxLayout()  # Horizontal layout
        
        # Camera selection
        camera_selection_layout = QVBoxLayout()
        camera_label = QLabel("Camera:")
        self.camera_combo = QComboBox()
        self.camera_combo.currentIndexChanged.connect(self.change_camera)
        
        camera_selection_layout.addWidget(camera_label)
        camera_selection_layout.addWidget(self.camera_combo)
        
        # Refresh camera list button
        self.refresh_button = QPushButton("Refresh")
        self.refresh_button.clicked.connect(self.scan_cameras)
        camera_selection_layout.addWidget(self.refresh_button)
        
        controls_layout.addLayout(camera_selection_layout)
        
        # Quality controls
        quality_layout = QVBoxLayout()
        quality_label = QLabel("Quality:")
        
        self.quality_checkbox = QCheckBox("High Quality (1080p)")
        self.quality_checkbox.setChecked(self.high_quality)
        self.quality_checkbox.stateChanged.connect(self.toggle_quality)
        
        quality_layout.addWidget(quality_label)
        quality_layout.addWidget(self.quality_checkbox)
        
        # Face detection toggle
        self.face_detection_checkbox = QCheckBox("Face Detection")
        self.face_detection_checkbox.setChecked(self.face_detection_enabled)
        self.face_detection_checkbox.stateChanged.connect(self.toggle_face_detection)
        quality_layout.addWidget(self.face_detection_checkbox)
        
        # Face tracking toggle
        self.tracking_checkbox = QCheckBox("Face Tracking")
        self.tracking_checkbox.setChecked(self.tracking_enabled)
        self.tracking_checkbox.stateChanged.connect(self.toggle_tracking)
        quality_layout.addWidget(self.tracking_checkbox)
        
        controls_layout.addLayout(quality_layout)
        
        # Camera diagnostics button
        self.diagnostics_button = QPushButton("Camera Diagnostics")
        self.diagnostics_button.clicked.connect(self.show_diagnostics)
        controls_layout.addWidget(self.diagnostics_button)
        
        # Add some spacer for better layout
        controls_layout.addStretch(1)
        
        main_layout.addLayout(controls_layout)
        
        # Pre-allocate buffers for better performance
        self.qt_image = None
        self.pixmap = None
    
    def toggle_tracking(self, state):
        """Toggle face tracking on/off"""
        self.tracking_enabled = bool(state)
        self.node.get_logger().info(f"Face tracking {'enabled' if self.tracking_enabled else 'disabled'}")
        self.frame_grabber.enable_tracking(self.tracking_enabled)
        
        # Make sure face detection is enabled if tracking is enabled
        if self.tracking_enabled and not self.face_detection_enabled:
            self.face_detection_enabled = True
            self.face_detection_checkbox.setChecked(True)
            self.frame_grabber.toggle_face_detection(True)
    
    def toggle_face_detection(self, state):
        """Toggle face detection on/off"""
        self.face_detection_enabled = bool(state)
        self.node.get_logger().info(f"Face detection {'enabled' if self.face_detection_enabled else 'disabled'}")
        self.frame_grabber.toggle_face_detection(self.face_detection_enabled)
        
        # If face detection is turned off, disable tracking too
        if not self.face_detection_enabled and self.tracking_enabled:
            self.tracking_enabled = False
            self.tracking_checkbox.setChecked(False)
            self.frame_grabber.enable_tracking(False)
    
    def check_video_devices(self):
        """Check what video devices are available in the system"""
        if not os.path.exists('/dev'):
            self.node.get_logger().warn("No /dev directory found - not a Linux system?")
            return
        
        video_devices = []
        for device in os.listdir('/dev'):
            if device.startswith('video'):
                full_path = f"/dev/{device}"
                if os.access(full_path, os.R_OK):
                    video_devices.append(full_path)
        
        if not video_devices:
            self.node.get_logger().warn("No video devices found in /dev/")
            return
        
        self.node.get_logger().info(f"Found video devices: {', '.join(video_devices)}")
        
        # Check if any are in use
        try:
            output = subprocess.check_output(['fuser'] + video_devices, stderr=subprocess.STDOUT, text=True)
            self.node.get_logger().warn(f"Some video devices are in use: {output}")
        except subprocess.CalledProcessError:
            # No devices in use (fuser returns non-zero if no processes found)
            self.node.get_logger().info("No video devices appear to be in use")
        except Exception as e:
            # Some other error with fuser
            self.node.get_logger().warn(f"Error checking device usage: {e}")
    
    def show_diagnostics(self):
        """Show camera diagnostics information"""
        info = "Camera Diagnostics:\n\n"
        
        # Check for video devices
        video_devices = []
        for device in os.listdir('/dev'):
            if device.startswith('video'):
                full_path = f"/dev/{device}"
                access = os.access(full_path, os.R_OK)
                video_devices.append(f"{full_path} (Readable: {access})")
        
        if video_devices:
            info += "Video Devices:\n" + "\n".join(video_devices) + "\n\n"
        else:
            info += "No video devices found!\n\n"
        
        # OpenCV version
        info += f"OpenCV Version: {cv2.__version__}\n"
        
        # Face detection status
        info += f"Face Detection: {'Enabled' if self.face_detection_enabled else 'Disabled'}\n"
        info += f"Using OpenCV DNN-based face detector\n\n"
        
        # Check which OpenCV backends are available
        info += "Available OpenCV Backends:\n"
        backends = [
            (cv2.CAP_V4L2, "Linux V4L2"),
            (cv2.CAP_GSTREAMER, "GStreamer"),
            (cv2.CAP_FFMPEG, "FFMPEG"),
            (cv2.CAP_IMAGES, "Images"),
            (cv2.CAP_DSHOW, "DirectShow (Windows)"),
            (cv2.CAP_ANY, "Auto-detect")
        ]
        
        for backend_id, name in backends:
            info += f"- {name}\n"
        
        # Show message box with diagnostic info
        QMessageBox.information(self, "Camera Diagnostics", info)
    
    def scan_cameras(self):
        """Scan for available cameras"""
        self.camera_combo.clear()
        
        # Stop current camera if running
        self.frame_grabber.stop()
        
        # Check for cameras using direct V4L2 access first
        self.node.get_logger().info("Scanning for cameras...")
        available_cameras = []
        
        # In Linux, we can check directly which devices are video capture devices
        if os.path.exists('/dev'):
            for i in range(10):
                device_path = f"/dev/video{i}"
                if os.path.exists(device_path) and os.access(device_path, os.R_OK):
                    try:
                        # Try to open the camera directly with V4L2
                        cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
                        if cap.isOpened():
                            # It's a valid camera
                            camera_name = f"Camera {i} ({device_path})"
                            available_cameras.append((i, camera_name))
                            cap.release()
                        else:
                            self.node.get_logger().info(f"Device {device_path} exists but couldn't be opened as a camera")
                    except Exception as e:
                        self.node.get_logger().warn(f"Error checking {device_path}: {e}")
        
        # Fallback to generic scanning
        if not available_cameras:
            self.node.get_logger().info("Direct scan failed, trying generic approach...")
            for i in range(2):  # Only try the first two indices to avoid lengthy timeouts
                try:
                    cap = cv2.VideoCapture(i)
                    if cap.isOpened():
                        camera_name = f"Camera {i}"
                        available_cameras.append((i, camera_name))
                        cap.release()
                except Exception as e:
                    self.node.get_logger().warn(f"Error scanning camera {i}: {e}")
        
        # Add available cameras to combo box
        for idx, name in available_cameras:
            self.camera_combo.addItem(name, idx)
            
        if not available_cameras:
            self.node.get_logger().error("No cameras found!")
            self.image_label.setText("No cameras found! Check connections and permissions.")
            return
            
        # If any camera is available, start the first one
        if self.camera_combo.count() > 0:
            self.change_camera(self.camera_combo.currentIndex())
    
    def change_camera(self, index):
        """Change to a different camera"""
        if index >= 0:
            camera_index = self.camera_combo.itemData(index)
            self.node.get_logger().info(f"Changing to camera index {camera_index}")
            self.frame_grabber.stop()
            self.frame_grabber.set_quality(self.high_quality)
            self.frame_grabber.toggle_face_detection(self.face_detection_enabled)
            # Try with V4L2 backend specifically on Linux
            if os.name == 'posix':
                self.frame_grabber.start(camera_index, cv2.CAP_V4L2)
            else:
                self.frame_grabber.start(camera_index)
    
    def toggle_quality(self, state):
        """Toggle between high and low quality modes"""
        self.high_quality = bool(state)
        self.node.get_logger().info(f"Quality set to {'high' if self.high_quality else 'standard'}")
        self.frame_grabber.set_quality(self.high_quality)
    
    def handle_camera_error(self, error_msg):
        """Handle errors from the camera thread"""
        self.node.get_logger().error(f"Camera error: {error_msg}")
        self.image_label.setText(f"Camera error: {error_msg}\nTry refreshing or check permissions.")
    
    def process_frame(self, frame):
        """Process and display a camera frame - optimized for speed"""
        if frame is None:
            return
            
        # Convert colors - BGR to RGB (optimized)
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Scale image to fit label if needed
        label_width = self.image_label.width()
        label_height = self.image_label.height()
        
        if label_width > 0 and label_height > 0:
            h, w = rgb_image.shape[:2]
            
            # Calculate scale factor to fit in label while preserving aspect ratio
            scale_w = label_width / w
            scale_h = label_height / h
            scale = min(scale_w, scale_h)
            
            # Only scale if necessary (smaller than label)
            if scale < 1.0:
                # Use NEAREST for fastest scaling
                new_size = (int(w * scale), int(h * scale))
                rgb_image = cv2.resize(rgb_image, new_size, interpolation=cv2.INTER_NEAREST)
        
        # Convert to QImage and display (reusing objects when possible)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        
        # Convert to QImage efficiently
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_image)
        self.image_label.setPixmap(pixmap)
    
    def closeEvent(self, event):
        """Clean up when window is closed"""
        self.frame_grabber.stop()
        event.accept()


class CameraNode(Node):
    def __init__(self):
        super().__init__('coffee_camera_node')
        self.get_logger().info('Camera node is starting...')
        
        # Start the UI
        app = QApplication(sys.argv)
        self.ui = CameraViewer(self)
        self.ui.show()
        
        # Start a background thread for ROS spinning
        self.spinning = True
        self.ros_thread = threading.Thread(target=self.spin_thread)
        self.ros_thread.daemon = True
        self.ros_thread.start()
        
        # Start Qt event loop
        sys.exit(app.exec_())
    
    def spin_thread(self):
        """Background thread for ROS spinning"""
        while self.spinning:
            rclpy.spin_once(self, timeout_sec=0.1)
    
    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info('Shutting down camera node')
        self.spinning = False
        
        # Stop the frame grabber
        if hasattr(self, 'ui') and hasattr(self.ui, 'frame_grabber'):
            # Stop head controller if it exists
            if self.ui.frame_grabber.head_controller:
                try:
                    self.ui.frame_grabber.head_controller.stop()
                    self.get_logger().info('Head controller stopped')
                except Exception as e:
                    self.get_logger().error(f'Error stopping head controller: {e}')
            
            # Stop frame grabber
            try:
                self.ui.frame_grabber.stop()
                self.get_logger().info('Frame grabber stopped')
            except Exception as e:
                self.get_logger().error(f'Error stopping frame grabber: {e}')
        
        # Clean up ROS resources
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CameraNode()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in camera node: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 