#!/usr/bin/env python3

import rclpy
import cv2
import time
import numpy as np
import threading
import os
import json
from rclpy.node import Node
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

class SimpleCamera:
    """A simple camera class without Qt dependencies"""
    def __init__(self, node):
        self.node = node
        self.running = True
        self.camera = None
        self.camera_index = 0
        self.face_detection_enabled = True
        self.face_net = None
        self.face_confidence_threshold = 0.5
        
        # Debug flags
        self.debug_mode = True
        
        # Create face publisher
        self.face_pub = node.create_publisher(String, 'face_detection_data', 10)
        
        # Create frame publisher
        self.frame_pub = node.create_publisher(Image, 'camera_frame', 10)
        
        # Create face image publisher
        self.face_image_pub = node.create_publisher(Image, 'face_images', 10)
        
        # Create CV bridge
        self.bridge = CvBridge()
        
        # Init face detector - delayed until camera starts to avoid crash
        self.face_detector_initialized = False
        
        # Camera thread
        self.camera_thread = None
        
        # Face recognition integration
        self.face_recognition_sub = node.create_subscription(
            String,
            'face_recognition_data',
            self.face_recognition_callback,
            10
        )
        self.face_ids = {}  # Map of face index to recognized face ID
        self.last_recognition_time = 0
        self.recognition_timeout = 3.0  # Clear recognition data after 3 seconds without updates
        
        # Head controller for tracking - initialize after camera starts
        self.head_controller = None
        self.tracking_enabled = True
        
        # Tracking settings
        self.frame_width = 1280
        self.frame_height = 720
        self.target_face_index = 0  # Track the first face by default
        
        # Face smoothing for tracking
        self.prev_faces = []
        self.smoothing_factor = 0.7
        self.smoothing_frames = 5
        
        # Resource monitoring
        self.last_fps_time = time.time()
        self.frames_processed = 0
        self.current_fps = 0
        
        # Display window - create in camera thread to avoid X server issues
        self.display_enabled = True
        self.window_name = "Camera Feed"
        self.window_created = False
        
        # Safety flags to avoid crashes
        self.is_shutting_down = False
                
    def init_face_detector(self):
        """Initialize OpenCV DNN face detector"""
        try:
            if self.debug_mode:
                print("Initializing face detector...")
                
            # Model files
            model_file = os.path.join(MODELS_DIR, "opencv_face_detector_uint8.pb")
            config_file = os.path.join(MODELS_DIR, "opencv_face_detector.pbtxt")
            
            # Download if needed
            if not os.path.exists(model_file) or not os.path.exists(config_file):
                self.download_face_model(model_file, config_file)
            
            # Load detector
            self.face_net = cv2.dnn.readNet(model_file, config_file)
            
            # Use GPU if available
            if cv2.cuda.getCudaEnabledDeviceCount() > 0:
                self.face_net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
                self.face_net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
            else:
                self.face_net.setPreferableBackend(cv2.dnn.DNN_BACKEND_DEFAULT)
                self.face_net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            
            self.face_detector_initialized = True
            print("Face detector (OpenCV DNN) initialized successfully")
        except Exception as e:
            print(f"Error initializing face detector: {e}")
            self.face_net = None
            self.face_detector_initialized = False
            
    def download_face_model(self, model_file, config_file):
        """Download face detection model files"""
        try:
            # URLs
            model_url = "https://github.com/spmallick/learnopencv/raw/refs/heads/master/AgeGender/opencv_face_detector_uint8.pb"
            config_url = "https://raw.githubusercontent.com/spmallick/learnopencv/refs/heads/master/AgeGender/opencv_face_detector.pbtxt"
            
            # Download
            import urllib.request
            print("Downloading face detection model...")
            urllib.request.urlretrieve(model_url, model_file)
            urllib.request.urlretrieve(config_url, config_file)
            print("Face detection model downloaded successfully")
        except Exception as e:
            print(f"Error downloading face model: {e}")
            # Don't raise exception, just return to avoid crash
    
    def create_window(self):
        """Create OpenCV window safely"""
        if not self.display_enabled or self.window_created:
            return
            
        try:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 800, 600)
            self.window_created = True
            print("Created OpenCV window successfully")
        except Exception as e:
            print(f"Failed to create window: {e}")
            self.display_enabled = False  # Disable display if window creation fails
    
    def init_head_controller(self):
        """Initialize head controller safely"""
        if self.head_controller is not None:
            return
            
        try:
            self.head_controller = HeadController(self.node)
            print("Head controller initialized successfully")
        except Exception as e:
            print(f"Error initializing head controller: {e}")
            self.tracking_enabled = False  # Disable tracking if controller fails

    def face_recognition_callback(self, msg):
        """Process face recognition data from face recognition node"""
        try:
            recognition_data = json.loads(msg.data)
            self.last_recognition_time = time.time()
            
            if self.debug_mode:
                print(f"Received face recognition data: {recognition_data}")
            
            # Store the raw recognition data for debugging
            self.last_recognition_data = recognition_data
            
            # Clear existing face IDs
            self.face_ids = {}
            
            # Process each recognized face
            for face in recognition_data.get('faces', []):
                face_id = face.get('id')
                position = face.get('position', {})
                center_x = position.get('center_x', 0)
                center_y = position.get('center_y', 0)
                
                # Find the corresponding detected face
                # Match based on position
                best_match_idx = -1
                best_match_dist = float('inf')
                
                for i, detected_face in enumerate(self.prev_faces[-1] if self.prev_faces else []):
                    # Calculate distance between centers
                    dx = detected_face.get('center_x', 0) - center_x
                    dy = detected_face.get('center_y', 0) - center_y
                    dist = (dx**2 + dy**2)**0.5
                    
                    if dist < best_match_dist and dist < 100:  # Max 100px distance for a match
                        best_match_dist = dist
                        best_match_idx = i
                
                if best_match_idx >= 0:
                    # Store the ID with this face index
                    self.face_ids[best_match_idx] = {
                        'id': face_id,
                        'confidence': face.get('confidence', 0.0),
                        'is_new': face.get('is_new', False)
                    }
                    
                    # Also update the current faces if they exist
                    if hasattr(self, 'current_faces') and self.current_faces and len(self.current_faces) > best_match_idx:
                        self.current_faces[best_match_idx]['id'] = face_id
            
            # Log the recognized faces
            face_id_str = ", ".join([f"Face {idx}: ID {data['id']}" for idx, data in self.face_ids.items()])
            if face_id_str:
                print(f"Recognized faces: {face_id_str}")
            
        except Exception as e:
            print(f"Error processing face recognition data: {e}")
            import traceback
            traceback.print_exc()

    def detect_faces(self, frame):
        """Detect faces using OpenCV DNN detector"""
        if self.face_net is None or not self.face_detector_initialized:
            return []
            
        try:
            # Get dimensions
            h, w = frame.shape[:2]
            
            # Create blob
            blob = cv2.dnn.blobFromImage(frame, 1.0, (300, 300), [104, 117, 123], False, False)
            self.face_net.setInput(blob)
            
            # Run detection
            detections = self.face_net.forward()
            
            # Parse results
            faces = []
            for i in range(detections.shape[2]):
                confidence = detections[0, 0, i, 2]
                if confidence > self.face_confidence_threshold:
                    # Get coords
                    x1 = int(detections[0, 0, i, 3] * w)
                    y1 = int(detections[0, 0, i, 4] * h)
                    x2 = int(detections[0, 0, i, 5] * w)
                    y2 = int(detections[0, 0, i, 6] * h)
                    
                    # Validate
                    x1 = max(0, min(x1, w-1))
                    y1 = max(0, min(y1, h-1))
                    x2 = max(0, min(x2, w-1))
                    y2 = max(0, min(y2, h-1))
                    
                    if x2 > x1 and y2 > y1:
                        # Create a face entry
                        face = {
                            'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
                            'center_x': (x1 + x2) // 2,
                            'center_y': (y1 + y2) // 2,
                            'width': (x2 - x1),
                            'height': (y2 - y1),
                            'radius': max((x2 - x1), (y2 - y1)) // 2,
                            'confidence': float(confidence),
                            'id': 'Unknown'  # Default ID
                        }
                        
                        # Check if we have an ID for this face from face recognition
                        if i in self.face_ids:
                            face['id'] = self.face_ids[i]['id']
                        
                        faces.append(face)
            
            # Store the current detected faces for use in the recognition callback
            self.current_faces = faces
            
            # Apply temporal smoothing to face positions
            smoothed_faces = self.smooth_face_detections(faces)
            
            # Transfer face IDs from original faces to smoothed faces
            for i, (original, smoothed) in enumerate(zip(faces, smoothed_faces)):
                if 'id' in original and original['id'] != 'Unknown':
                    smoothed['id'] = original['id']
            
            return smoothed_faces
        except Exception as e:
            print(f"Error in face detection: {e}")
            return []
    
    def smooth_face_detections(self, faces):
        """Apply temporal smoothing to face detections"""
        if not faces:
            # Clear history if no faces detected
            self.prev_faces = []
            return []
        
        # Add current detection to history
        self.prev_faces.append(faces)
        
        # Keep history at fixed size
        if len(self.prev_faces) > self.smoothing_frames:
            self.prev_faces.pop(0)
        
        # If we don't have enough history yet, just return current detection
        if len(self.prev_faces) < 2:
            return faces
        
        # Average faces across frames
        smoothed_faces = []
        
        # For each face in current frame
        for i, current_face in enumerate(faces):
            smoothed_face = current_face.copy()
            
            # Find corresponding faces in previous frames
            for prev_frame in self.prev_faces[:-1]:  # Skip current frame (last in list)
                best_match = None
                best_match_dist = float('inf')
                
                # Find best matching face in previous frame
                for prev_face in prev_frame:
                    dx = prev_face['center_x'] - current_face['center_x']
                    dy = prev_face['center_y'] - current_face['center_y']
                    dist = (dx**2 + dy**2)**0.5
                    
                    if dist < best_match_dist and dist < 100:  # Max 100px distance for match
                        best_match_dist = dist
                        best_match = prev_face
                
                # If matching face found, blend positions
                if best_match:
                    # Apply smoothing factor to position
                    for key in ['x1', 'y1', 'x2', 'y2', 'center_x', 'center_y']:
                        smoothed_face[key] = int(self.smoothing_factor * best_match[key] + 
                                                (1 - self.smoothing_factor) * smoothed_face[key])
                    
                    # Recalculate derived values
                    smoothed_face['width'] = smoothed_face['x2'] - smoothed_face['x1']
                    smoothed_face['height'] = smoothed_face['y2'] - smoothed_face['y1']
                    smoothed_face['radius'] = max(smoothed_face['width'], smoothed_face['height']) // 2
            
            smoothed_faces.append(smoothed_face)
        
        return smoothed_faces
                    
    def start(self):
        """Start the camera thread"""
        print("Starting simple camera node")
        self.camera_thread = threading.Thread(target=self._camera_loop)
        self.camera_thread.daemon = True
        self.camera_thread.start()
        
    def _camera_loop(self):
        """Main camera processing loop"""
        print("Camera thread starting")
        
        # Safe window creation in the same thread as display
        if self.display_enabled:
            self.create_window()
            
        # Try to open camera
        try:
            self.camera = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
            if not self.camera.isOpened():
                print(f"Failed to open camera at index {self.camera_index} with V4L2, trying default backend")
                self.camera = cv2.VideoCapture(self.camera_index)  # Try default backend
                if not self.camera.isOpened():
                    print("Failed to open camera with any backend, creating dummy frame")
                    # Create a black dummy frame instead of failing
                    self.camera = None
                    dummy_frame = np.zeros((720, 1280, 3), dtype=np.uint8)
                    # Draw text indicating camera failure
                    cv2.putText(dummy_frame, "Camera failed to open!", (400, 360),
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                else:
                    print(f"Successfully opened camera at index {self.camera_index} with default backend")
            else:
                print(f"Successfully opened camera at index {self.camera_index} with V4L2 backend")
        except Exception as e:
            print(f"Exception opening camera: {e}")
            self.camera = None
            dummy_frame = np.zeros((720, 1280, 3), dtype=np.uint8)
            cv2.putText(dummy_frame, f"Camera error: {str(e)}", (400, 360),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        # Set resolution only if camera opened successfully
        if self.camera and self.camera.isOpened():
            try:
                self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                self.frame_width = 1280
                self.frame_height = 720
            except Exception as e:
                print(f"Failed to set camera resolution: {e}")
        
        # Initialize face detector after camera is set up
        if not self.face_detector_initialized:
            self.init_face_detector()
            
        # Initialize head controller
        self.init_head_controller()
        
        # Add storage for current detected faces
        self.current_faces = []
            
        # Frame counter
        frame_count = 0
        
        while self.running and not self.is_shutting_down:
            try:
                # Get frame - either from camera or use dummy frame
                if self.camera and self.camera.isOpened():
                    ret, frame = self.camera.read()
                    if not ret:
                        print("Failed to read frame, creating dummy frame")
                        frame = np.zeros((720, 1280, 3), dtype=np.uint8)
                        cv2.putText(frame, "No camera feed", (400, 360),
                                  cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                else:
                    # Use dummy frame if no camera
                    frame = dummy_frame.copy()
                
                # Update FPS counter
                self.frames_processed += 1
                current_time = time.time()
                if current_time - self.last_fps_time >= 1.0:
                    self.current_fps = self.frames_processed / (current_time - self.last_fps_time)
                    self.frames_processed = 0
                    self.last_fps_time = current_time
                
                # Detect faces - only if face detector initialized
                faces = []
                if self.face_detection_enabled and self.face_detector_initialized and frame_count % 3 == 0:
                    try:
                        faces = self.detect_faces(frame)
                        if faces:
                            self.publish_face_data(faces)
                            self.publish_face_images(frame, faces)
                            
                            # Update face tracking
                            if self.tracking_enabled and self.head_controller and faces and len(faces) > self.target_face_index:
                                self.track_face(faces[self.target_face_index])
                    except Exception as e:
                        print(f"Error in face detection: {e}")
                
                # Draw basic info and faces on frame
                try:
                    self.draw_info_on_frame(frame, faces)
                except Exception as e:
                    print(f"Error drawing on frame: {e}")
                
                # Publish frame
                try:
                    self.publish_frame(frame)
                except Exception as e:
                    print(f"Error publishing frame: {e}")
                
                # Display frame in window
                if self.display_enabled and self.window_created:
                    try:
                        cv2.imshow(self.window_name, frame)
                        key = cv2.waitKey(1)
                        if key == 27:  # ESC key
                            print("ESC pressed, stopping camera")
                            self.running = False
                            break
                    except Exception as e:
                        print(f"Error displaying frame: {e}")
                        self.display_enabled = False  # Disable display on error
                
                # Update counter
                frame_count += 1
                
                # Short sleep to prevent CPU overuse
                time.sleep(0.01)
                
            except Exception as e:
                print(f"Error in camera loop: {e}")
                import traceback
                traceback.print_exc()
                time.sleep(0.1)
                
        # Clean up
        print("Cleaning up camera resources...")
        if self.camera and self.camera.isOpened():
            try:
                self.camera.release()
                print("Camera released")
            except Exception as e:
                print(f"Error releasing camera: {e}")
        
        # Close window
        if self.display_enabled and self.window_created:
            try:
                cv2.destroyWindow(self.window_name)
                cv2.waitKey(1)  # Give time for window destruction
                print("Window destroyed")
            except Exception as e:
                print(f"Error destroying windows: {e}")
        
        print("Camera cleanup completed")
    
    def draw_info_on_frame(self, frame, faces):
        """Draw information overlay on frame"""
        # Draw camera info
        cv2.putText(frame, f"Camera: {self.camera_index}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # Draw FPS
        cv2.putText(frame, f"FPS: {self.current_fps:.1f}", (10, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # Check if recognition data is stale
        if time.time() - self.last_recognition_time > self.recognition_timeout:
            if self.debug_mode:
                print("Recognition data is stale, clearing face IDs")
            self.face_ids = {}  # Clear stale data
        
        # Debug info for face recognition status
        recog_status = "Active" if time.time() - self.last_recognition_time < self.recognition_timeout else "Inactive"
        cv2.putText(frame, f"Face Recog: {recog_status}", (10, 110),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Draw face rectangles and IDs
        for i, face in enumerate(faces):
            # Get face ID 
            face_id = face.get('id', 'Unknown')
            color = (0, 255, 0)  # Green for detected faces
            
            if face_id != 'Unknown':
                # Use different color for recognized faces
                color = (0, 200, 255)  # Orange for recognized faces
            
            # Draw rectangle
            cv2.rectangle(frame, (face['x1'], face['y1']), (face['x2'], face['y2']), color, 2)
            
            # Draw ID and confidence
            face_conf = face.get('confidence', 0.0)
            id_text = f"ID: {face_id}" if face_id != 'Unknown' else "Unknown"
            conf_text = f"Conf: {face_conf:.2f}"
            
            cv2.putText(frame, id_text, (face['x1'], face['y1'] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            cv2.putText(frame, conf_text, (face['x1'], face['y1'] + face['height'] + 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # Highlight tracked face
            if i == self.target_face_index and self.tracking_enabled:
                # Draw special marker for tracked face
                cv2.circle(frame, (face['center_x'], face['center_y']), 5, (0, 0, 255), -1)
                cv2.putText(frame, "TRACKING", (face['center_x'] - 40, face['center_y'] - 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        # Display number of recognized faces
        recog_count = len([f for f in faces if f.get('id', 'Unknown') != 'Unknown'])
        cv2.putText(frame, f"Recognized: {recog_count}/{len(faces)}", (10, 150), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
    def track_face(self, face):
        """Track a face with the head controller"""
        try:
            # Convert face position to pan/tilt angles
            center_x = face['center_x']
            center_y = face['center_y']
            
            # Calculate normalized coordinates (-1 to 1)
            norm_x = (center_x / self.frame_width) * 2 - 1
            norm_y = (center_y / self.frame_height) * 2 - 1
            
            # Invert Y-axis (positive down in image, positive up in tilt)
            norm_y = -norm_y
            
            # Scale to head angles - adjust these values based on your setup
            # These are the maximum angles to move when face is at edge of frame
            max_pan_angle = 80
            max_tilt_angle = 30
            
            pan_angle = norm_x * max_pan_angle
            tilt_angle = norm_y * max_tilt_angle
            
            # Send to head controller
            self.head_controller.set_target(pan_angle, tilt_angle)
            
        except Exception as e:
            self.node.get_logger().error(f"Error tracking face: {e}")
            
    def stop(self):
        """Stop the camera thread"""
        print("Stopping camera...")
        self.is_shutting_down = True
        self.running = False
        
        # Stop head controller first
        if self.head_controller:
            try:
                self.head_controller.stop()
                print("Head controller stopped")
            except Exception as e:
                print(f"Error stopping head controller: {e}")
        
        # Wait for camera thread to finish
        if self.camera_thread:
            try:
                self.camera_thread.join(timeout=2.0)
                print("Camera thread joined")
            except Exception as e:
                print(f"Error joining camera thread: {e}")
        
        # Release camera if still open
        if self.camera and self.camera.isOpened():
            try:
                self.camera.release()
                print("Camera released")
            except Exception as e:
                print(f"Error releasing camera: {e}")
        
        # Close OpenCV windows
        if self.display_enabled and self.window_created:
            try:
                cv2.destroyAllWindows()
                cv2.waitKey(1)  # Give time for window destruction
                print("All windows destroyed")
            except Exception as e:
                print(f"Error destroying windows: {e}")
        
        print("Camera cleanup completed")
            
    def publish_face_data(self, faces):
        """Publish face detection data"""
        if not faces:
            return
            
        # Get frame dimensions for the head tracking node
        frame_width = self.frame_width
        frame_height = self.frame_height
            
        # Create JSON with face data - include frame dimensions
        face_data = {
            "timestamp": time.time(),
            "frame_width": frame_width,
            "frame_height": frame_height,
            "faces": [
                {
                    "x1": face["x1"],
                    "y1": face["y1"],
                    "x2": face["x2"],
                    "y2": face["y2"],
                    "center_x": face["center_x"],
                    "center_y": face["center_y"],
                    "confidence": face["confidence"],
                    "id": face.get("id", "Unknown")
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
            print(f"Error publishing face images: {e}")
            
    def publish_frame(self, frame):
        """Publish camera frame"""
        try:
            frame_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            frame_msg.header.stamp = self.node.get_clock().now().to_msg()
            self.frame_pub.publish(frame_msg)
        except Exception as e:
            print(f"Error publishing frame: {e}")


class SimpleCameraNode(Node):
    """Simple camera node without Qt dependencies"""
    def __init__(self):
        super().__init__('camera_node')
        self.get_logger().info('Starting simple camera node')
        
        # Create camera
        self.camera = SimpleCamera(self)
        
        # Start camera
        self.camera.start()
        
        # Handle shutdown
        self.get_logger().info('Simple camera node started')
        
    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info('Shutting down camera node')
        if hasattr(self, 'camera'):
            self.camera.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SimpleCameraNode()
        rclpy.spin(node)
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