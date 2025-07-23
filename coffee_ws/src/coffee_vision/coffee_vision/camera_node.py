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
import collections
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray, String, Bool, Int32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

from .coordinate_utils import transform_camera_to_eye_coords
from .face_detection import FaceDetector


class FrameGrabber:
    """Dedicated thread for frame capture to improve performance"""
    
    def __init__(self, node=None):
        self.node = node
        # Camera properties
        self.camera = None
        self.camera_index = 0
        self.frame_width = 640
        self.frame_height = 480
        self.backend = cv2.CAP_ANY
        
        # Camera optimization settings
        self.target_fps = 30
        self.target_exposure = 100  # Auto exposure target (0-255)
        self.camera_props = {
            cv2.CAP_PROP_FRAME_WIDTH: self.frame_width,
            cv2.CAP_PROP_FRAME_HEIGHT: self.frame_height,
            cv2.CAP_PROP_FPS: self.target_fps,
            cv2.CAP_PROP_BUFFERSIZE: 1,  # Minimal latency
            # cv2.CAP_PROP_AUTOFOCUS: 0,  # Disable autofocus
            # cv2.CAP_PROP_AUTO_EXPOSURE: 1,  # Enable auto exposure
            # cv2.CAP_PROP_AUTO_WB: 1,  # Enable auto white balance
            # cv2.CAP_PROP_EXPOSURE: self.target_exposure,
        }
        self.running = False
        self.lock = threading.Lock()
        self.capture_thread = None
        self.process_thread = None
        self.publish_thread = None
        self.high_quality = False
        
        # Initialize shared frame buffer
        self.current_frame = None
        self.processed_frame = None
        self.current_faces = []
        self.frame_lock = threading.Lock()
        self.frame_timestamp = 0  # Track frame freshness
        
        # Frame processing control
        # (UI-related frame rate controls removed in headless mode)
        
        # Face detection control
        self.last_detection_time = 0
        self.min_detection_interval = 1.0 / 6  # Max 6 face detections per second
        self.max_detection_time = 0.1  # Maximum 100ms for face detection
        self.detection_skip_frames = 5  # Process every 5th frame
        self.detection_frame_counter = 0
        
        # Publishing rate control
        self.last_publish_time = 0
        self.min_publish_interval = 1.0 / 30.0  # Max 30 fps for publishing
        
        # Face detection
        self.enable_face_detection = True
        self.face_detector = FaceDetector(
            confidence_threshold=0.5,
            smoothing_factor=0.4,
            logger=self.node.get_logger() if self.node else None
        )

        # # Get parameters
        # self.invert_x = self.get_parameter('invert_x').value
        # self.invert_y = self.get_parameter('invert_y').value
        # self.eye_range = self.get_parameter('eye_range').value
        
        # TODO: Added this hot fix
        self.invert_x = False
        self.invert_y = False
        # self.eye_range = 3.0
        self.eye_range = 1.0 # The constraint for the values of the eye movement in the UI

        # ROS publishers for face data and images
        if self.node:
            self.face_pub = node.create_publisher(String, 'face_detection_data', 10)
            self.face_position_pub = node.create_publisher(Point, '/vision/face_position', 10)
            self.face_position_pub_v2 = node.create_publisher(String, '/vision/face_position_v2', 10)
            self.frame_pub = node.create_publisher(Image, '/coffee_bot/camera/image_raw', 10)
            self.face_image_pub = node.create_publisher(Image, 'face_images', 10)
            self.bridge = CvBridge()
            
        # Face recognition data
        self.face_ids = {}  # Map of face index to recognized face ID
        self.last_recognition_time = 0
        self.recognition_timeout = 3.0  # Clear recognition data after 3 seconds
     
    def publish_face_data(self, faces):
        """Publish face detection data for other nodes"""
        if not self.node:
            return
        
        # Ensure faces is at least an empty list
        faces = faces if faces else []
            
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

    def publish_face_position_v2(self, faces):
        """Publish face detection data for other nodes"""
        if not self.node:
            return
            
        # Ensure faces is at least an empty list
        faces = faces if faces else []
            
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
        self.face_position_pub_v2.publish(msg)

        
    def publish_face_position(self, faces):
        """Process incoming face detection data"""

        try:
            # If no faces detected or all faces have very low confidence, publish zero position
            if not faces:
                self.target_face_position = Point()
                self.target_face_position.x = 0.0
                self.target_face_position.y = 0.0
                self.target_face_position.z = 0.0
                self.face_position_pub.publish(self.target_face_position)
                return

            # Select target face (largest/closest)
            largest_area = 0
            largest_face = None
            
            for face in faces:
                width = face['x2'] - face['x1']
                height = face['y2'] - face['y1']
                area = width * height
                
                if area > largest_area:
                    largest_area = area
                    largest_face = face
            
            if largest_face:
                self.target_face_position = (
                    largest_face['center_x'], 
                    largest_face['center_y']
                )
                
                # Log face position before transformation
                face_x = self.target_face_position[0]
                face_y = self.target_face_position[1]
                center_x = self.frame_width / 2
                center_y = self.frame_height / 2
                dx = face_x - center_x
                dy = face_y - center_y
                
                self.node.get_logger().debug(f"Face detected at ({face_x:.1f}, {face_y:.1f}), offset from center: ({dx:.1f}, {dy:.1f})")
                
                # Transform camera coordinates to eye controller coordinates
                eye_position = transform_camera_to_eye_coords(
                    camera_x=self.target_face_position[0],
                    camera_y=self.target_face_position[1],
                    frame_width=self.frame_width,
                    frame_height=self.frame_height,
                    eye_range=self.eye_range,
                    sensitivity=1.5,
                    invert_x=self.invert_x,
                    invert_y=self.invert_y,
                    logger=self.node.get_logger() if self.node else None
                )
                
                # Call go_to_pos only if we have a valid position
                if eye_position:
                    # self.controller.go_to_pos(eye_position)
                    # self.node.get_logger().info(f'Moving eyes to position: ({eye_position[0]:.2f}, {eye_position[1]:.2f})')
                    point_msg = Point()
                    point_msg.x = eye_position[0]
                    point_msg.y = eye_position[1]
                    point_msg.z = 0.0
                    self.face_position_pub.publish(point_msg)

        except Exception as e:
            self.node.get_logger().error(f"Error processing face position data: {e}")

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
    

    
    def start(self, camera_index, backend=cv2.CAP_ANY):
        with self.lock:
            if self.running:
                self.stop()
            
            self.camera_index = camera_index
            self.backend = backend
            self.running = True
            
            # Start capture thread
            self.capture_thread = threading.Thread(target=self._capture_loop)
            self.capture_thread.daemon = True
            self.capture_thread.start()
            
            # Start processing thread
            self.process_thread = threading.Thread(target=self._process_loop)
            self.process_thread.daemon = True
            self.process_thread.start()
            
            # Start publishing thread if ROS node exists
            if self.node:
                self.publish_thread = threading.Thread(target=self._publish_loop)
                self.publish_thread.daemon = True
                self.publish_thread.start()
    
    def stop(self):
        """Stop the frame grabber threads"""
        with self.frame_lock:
            self.running = False
            self.current_frame = None
            self.processed_frame = None
            self.current_faces = []
        
        # Wait for threads to finish
        if self.capture_thread and self.capture_thread.is_alive():
            self.capture_thread.join()
        
        if self.process_thread and self.process_thread.is_alive():
            self.process_thread.join()
            
        if self.publish_thread and self.publish_thread.is_alive():
            self.publish_thread.join()
        
        # Release camera if it's open
        if self.camera and self.camera.isOpened():
            self.camera.release()
            self.camera = None
    
    def set_quality(self, high_quality):
        """Toggle between low resolution and high resolution with optimal settings"""
        if high_quality:
            self.frame_width = 1280
            self.frame_height = 720
            self.target_fps = 24  # Lower FPS for higher resolution
        else:
            self.frame_width = 640
            self.frame_height = 480
            self.target_fps = 30  # Higher FPS for lower resolution
        
        # Update camera properties
        self.camera_props.update({
            cv2.CAP_PROP_FRAME_WIDTH: self.frame_width,
            cv2.CAP_PROP_FRAME_HEIGHT: self.frame_height,
            cv2.CAP_PROP_FPS: self.target_fps
        })
        
        if self.camera and self.camera.isOpened():
            # Apply new settings
            for prop, value in self.camera_props.items():
                try:
                    self.camera.set(prop, value)
                except:
                    pass
            
            # Verify new settings
            actual_width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.camera.get(cv2.CAP_PROP_FPS)
            
            print(f"Quality changed to: {actual_width}x{actual_height} @ {actual_fps:.1f} FPS")
            
            # No UI update needed in headless mode
    
    def toggle_face_detection(self, enable):
        """Enable or disable face detection"""
        with self.lock:
            self.enable_face_detection = enable
            
            # Reset face tracking when toggling
            if self.face_detector:
                self.face_detector.reset_tracking()
    

    

    

    
    def _capture_loop(self):
        """Main capture loop for camera frames"""
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
            
            # Try each backend until one works
            for backend, backend_name in backends_to_try:
                try:
                    if backend == cv2.CAP_ANY:
                        self.camera = cv2.VideoCapture(self.camera_index)
                    else:
                        self.camera = cv2.VideoCapture(self.camera_index, backend)
                    
                    if not self.camera.isOpened():
                        error_msg = f"Could not open camera {self.camera_index} with {backend_name} backend"
                        continue
                    
                    success = True
                    print(f"Successfully opened camera with {backend_name} backend")
                    break
                except Exception as e:
                    error_msg = f"Error opening camera with {backend_name} backend: {str(e)}"
                    continue
            
            if not success:
                if self.node:
                    self.node.get_logger().error(f"Failed to open camera: {error_msg}")
                return
            
            # Configure camera with optimal settings
            for prop, value in self.camera_props.items():
                try:
                    self.camera.set(prop, value)
                except:
                    pass  # Skip unsupported properties
            
            # Verify and adjust settings
            actual_width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.camera.get(cv2.CAP_PROP_FPS)
            
            print(f"Camera configured with: {actual_width}x{actual_height} @ {actual_fps:.1f} FPS")
            
            # Warm up the camera
            for _ in range(5):
                self.camera.read()
            
            # Read a test frame to check actual size
            ret, frame = self.camera.read()
            if ret:
                frame_h, frame_w = frame.shape[:2]
                if frame_w != actual_width or frame_h != actual_height:
                    print(f"Warning: Actual frame size ({frame_w}x{frame_h}) differs from requested ({actual_width}x{actual_height})")
                    self.frame_width = frame_w
                    self.frame_height = frame_h
            
            # Main capture loop
            while self.running:
                ret, frame = self.camera.read()
                if not ret:
                    continue
                
                # Flip frame horizontally
                frame = cv2.flip(frame, 1)
                
                # Update shared frame buffer
                with self.frame_lock:
                    self.current_frame = frame
                    self.frame_timestamp = time.time()
                        
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error in capture thread: {str(e)}")
        finally:
            if self.camera and self.camera.isOpened():
                self.camera.release()
                self.camera = None
    
    def _process_loop(self):
        """Process frames from the queue"""
        try:
            frame_count = 0
            start_time = time.time()
            fps = 0
            face_detection_interval = 3  # Reduced face detection frequency
            frame_index = 0
            
            while self.running:
                # Get latest frame
                with self.frame_lock:
                    frame = self.current_frame
                    frame_time = self.frame_timestamp
                    if frame is None:
                        continue
                    
                    # Make a copy to avoid holding the lock
                    frame = frame.copy()
                
                # Skip if frame is too old (> 100ms)
                if time.time() - frame_time > 0.1:
                    continue
                
                # Adaptive face detection with time budgeting
                self.detection_frame_counter += 1
                current_time = time.time()
                
                # Check if we should attempt face detection
                should_detect = (
                    self.enable_face_detection and
                    self.detection_frame_counter >= self.detection_skip_frames and
                    current_time - self.last_detection_time >= self.min_detection_interval
                )
                
                if should_detect:
                    detection_start = time.time()
                    faces = self.face_detector.detect_faces(frame)
                    faces = self.face_detector.smooth_detections(faces)
                    detection_time = time.time() - detection_start
                    
                    # If detection took too long, increase skip frames
                    if detection_time > self.max_detection_time:
                        self.detection_skip_frames = min(10, self.detection_skip_frames + 1)
                    else:
                        # If detection was fast, gradually decrease skip frames
                        self.detection_skip_frames = max(3, self.detection_skip_frames - 1)
                    
                    self.current_faces = faces  # No smoothing for better latency
                    self.last_detection_time = current_time
                    self.detection_frame_counter = 0
                    
                    # Check if recognition data is stale
                    if current_time - self.last_recognition_time > self.recognition_timeout:
                        self.face_ids = {}
                    
                    # Add face IDs
                    for i, face in enumerate(faces):
                        if i in self.face_ids:
                            face['id'] = self.face_ids[i]['id']
                        else:
                            face['id'] = 'Unknown'
                
                # Draw faces if available
                if self.current_faces:
                    frame = self.face_detector.draw_debug_overlay(frame, self.current_faces)
                
                # Update FPS counter
                frame_count += 1
                if frame_count >= 30:  # Increased sample size for smoother FPS
                    current_time = time.time()
                    elapsed = current_time - start_time
                    fps = frame_count / elapsed if elapsed > 0 else 0
                    frame_count = 0
                    start_time = current_time
                
                # Draw FPS
                cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                # Update processed frame
                with self.frame_lock:
                    self.processed_frame = frame
                
                # No UI frame emission needed in headless mode
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error in process thread: {str(e)}")
    
    def _publish_loop(self):
        """Handle ROS publishing at controlled rate"""
        try:
            while self.running:
                current_time = time.time()
                
                # Check if enough time has passed since last publish
                if current_time - self.last_publish_time >= self.min_publish_interval:
                    # Get latest processed frame
                    with self.frame_lock:
                        frame = self.processed_frame
                        faces = self.current_faces[:] if self.current_faces else []
                    
                    if frame is not None:
                        # Publish frame and face data
                        self.publish_frame(frame)
                        # Always publish face position, even when no faces are detected
                        # This is used so that we can re-center the eyes -- zero them in.
                        # self.publish_face_position(faces)
                        # Always publish face data, even when no faces are detected
                        self.publish_face_position_v2(faces)
                        self.publish_face_data(faces)
                        # Only publish face images when faces are actually detected
                        if faces:
                            self.publish_face_images(frame, faces)
                        
                        self.last_publish_time = current_time
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error in publish thread: {str(e)}")


class CameraNode(Node):
    def __init__(self):
        # Initialize node
        super().__init__('coffee_camera_node')
        self.get_logger().info('Camera node is starting...')
        
        # Initialize frame grabber for camera processing
        self.frame_grabber = FrameGrabber(self)
        
        # Set up ROS control interface for separated UI communication
        self._setup_ros_control_interface()
        
        # Camera state tracking
        self.available_cameras = []
        self.current_camera_index = -1
        self.high_quality = False
        self.face_detection_enabled = True
        
        # # Add parameters for mapping
        # self.declare_parameter('invert_x', False)  # Default FALSE for correct eye movement
        # self.declare_parameter('invert_y', False)  # Default FALSE for correct eye movement
        # self.declare_parameter('eye_range', 3.0)   # Max range for eye movement (-3.0 to 3.0)

        # # Get parameters
        # self.invert_x = self.get_parameter('invert_x').value
        # self.invert_y = self.get_parameter('invert_y').value
        # self.eye_range = self.get_parameter('eye_range').value

        self.invert_x = False
        self.invert_y = False
        # self.eye_range = 3.0
        self.eye_range = 1.0 # The constraint for the values of the eye movement in the UI
        
        # Initialize camera system
        self.scan_cameras()
    
    def scan_cameras(self):
        """Scan for available cameras"""
        self.get_logger().info("Scanning for cameras...")
        available_cameras = []
        
        # Stop current camera if running
        if hasattr(self, 'frame_grabber'):
            self.frame_grabber.stop()
        
        # Check for cameras using direct V4L2 access first
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
                            self.get_logger().info(f"Device {device_path} exists but couldn't be opened as a camera")
                    except Exception as e:
                        self.get_logger().warn(f"Error checking {device_path}: {e}")
        
        # Fallback to generic scanning
        if not available_cameras:
            self.get_logger().info("Direct scan failed, trying generic approach...")
            for i in range(2):  # Only try the first two indices to avoid lengthy timeouts
                try:
                    cap = cv2.VideoCapture(i)
                    if cap.isOpened():
                        camera_name = f"Camera {i}"
                        available_cameras.append((i, camera_name))
                        cap.release()
                except Exception as e:
                    self.get_logger().warn(f"Error scanning camera {i}: {e}")
        
        # Store for state queries
        self.available_cameras = available_cameras
        
        if not available_cameras:
            self.get_logger().error("No cameras found!")
            return
            
        # Start the first available camera
        if available_cameras:
            first_camera_index = available_cameras[0][0]
            self.change_camera(first_camera_index)
    
    def change_camera(self, camera_index):
        """Change to a different camera"""
        self.get_logger().info(f"Changing to camera index {camera_index}")
        self.current_camera_index = camera_index
        
        self.frame_grabber.stop()
        self.frame_grabber.set_quality(self.high_quality)
        self.frame_grabber.toggle_face_detection(self.face_detection_enabled)
        
        # Try with V4L2 backend specifically on Linux
        if os.name == 'posix':
            self.frame_grabber.start(camera_index, cv2.CAP_V4L2)
        else:
            self.frame_grabber.start(camera_index)
    
    def set_quality(self, high_quality):
        """Set camera quality"""
        self.high_quality = high_quality
        self.get_logger().info(f"Quality set to {'high' if high_quality else 'standard'}")
        self.frame_grabber.set_quality(high_quality)
    
    def set_face_detection(self, enabled):
        """Set face detection state"""
        self.face_detection_enabled = enabled
        self.get_logger().info(f"Face detection {'enabled' if enabled else 'disabled'}")
        self.frame_grabber.toggle_face_detection(enabled)
    
    def _setup_ros_control_interface(self):
        """Set up ROS subscribers and publishers for separated UI control"""
        self.get_logger().info('Setting up ROS control interface for separated UI communication')
        
        # Publishers for status updates to separated UI
        self.camera_status_pub = self.create_publisher(String, '/coffee_bot/camera/status/info', 10)
        self.available_cameras_pub = self.create_publisher(String, '/coffee_bot/camera/status/available', 10)
        self.diagnostics_pub = self.create_publisher(String, '/coffee_bot/camera/status/diagnostics', 10)
        
        # Subscribers for commands from separated UI
        self.camera_select_sub = self.create_subscription(
            Int32, '/coffee_bot/camera/cmd/select', self._on_camera_select_command, 10)
        self.quality_control_sub = self.create_subscription(
            Bool, '/coffee_bot/camera/cmd/quality', self._on_quality_change_command, 10)
        self.face_detection_sub = self.create_subscription(
            Bool, '/coffee_bot/camera/cmd/face_detection', self._on_face_detection_command, 10)
        self.camera_refresh_sub = self.create_subscription(
            String, '/coffee_bot/camera/cmd/refresh', self._on_camera_refresh_command, 10)
        self.diagnostics_request_sub = self.create_subscription(
            String, '/coffee_bot/camera/cmd/diagnostics', self._on_diagnostics_request, 10)
        
        # Subscriber for state queries from separated UI
        self.state_query_sub = self.create_subscription(
            String, '/coffee_bot/camera/query/state', self._on_state_query, 10)
        
        self.get_logger().info('ROS control interface setup complete')
    
    def _on_camera_select_command(self, msg):
        """Handle camera selection command from separated UI"""
        camera_index = msg.data
        self.get_logger().info(f'Received camera selection command: {camera_index}')
        
        # Directly control camera
        self.change_camera(camera_index)
        
        # Publish status update
        status_msg = String()
        status_msg.data = f"Camera selection changed to index {camera_index}"
        self.camera_status_pub.publish(status_msg)
    
    def _on_quality_change_command(self, msg):
        """Handle quality change command from separated UI"""
        high_quality = msg.data
        self.get_logger().info(f'Received quality change command: {"high" if high_quality else "standard"}')
        
        # Directly control quality
        self.set_quality(high_quality)
        
        # Publish status update
        status_msg = String()
        status_msg.data = f"Quality changed to {'high (1080p)' if high_quality else 'standard (480p)'}"
        self.camera_status_pub.publish(status_msg)
    
    def _on_face_detection_command(self, msg):
        """Handle face detection toggle command from separated UI"""
        enabled = msg.data
        self.get_logger().info(f'Received face detection command: {"enabled" if enabled else "disabled"}')
        
        # Directly control face detection
        self.set_face_detection(enabled)
        
        # Publish status update
        status_msg = String()
        status_msg.data = f"Face detection {'enabled' if enabled else 'disabled'}"
        self.camera_status_pub.publish(status_msg)
    
    def _on_camera_refresh_command(self, msg):
        """Handle camera refresh command from separated UI"""
        self.get_logger().info('Received camera refresh command')
        
        # Directly scan cameras
        self.scan_cameras()
        
        # Publish status update
        status_msg = String()
        status_msg.data = "Camera scan completed"
        self.camera_status_pub.publish(status_msg)
    
    def _on_diagnostics_request(self, msg):
        """Handle diagnostics request from separated UI"""
        self.get_logger().info('Received diagnostics request from separated UI')
        
        # Generate diagnostics information
        diagnostics_info = self._generate_diagnostics_info()
        
        # Publish diagnostics response
        diagnostics_msg = String()
        diagnostics_msg.data = diagnostics_info
        self.diagnostics_pub.publish(diagnostics_msg)
        
        self.get_logger().info('Published diagnostics information')
    
    def _generate_diagnostics_info(self):
        """Generate comprehensive diagnostics information"""
        import os
        import subprocess
        import cv2
        
        info = "Camera Node Diagnostics:\n\n"
        
        # Check for video devices
        video_devices = []
        if os.path.exists('/dev'):
            for device in os.listdir('/dev'):
                if device.startswith('video'):
                    full_path = f"/dev/{device}"
                    access = os.access(full_path, os.R_OK)
                    video_devices.append(f"{full_path} (Readable: {access})")
        
        if video_devices:
            info += "Video Devices:\n" + "\n".join(video_devices) + "\n\n"
        else:
            info += "No video devices found!\n\n"
        
        # OpenCV version and backend info
        info += f"OpenCV Version: {cv2.__version__}\n"
        
        # Camera state
        if hasattr(self, 'frame_grabber'):
            info += f"Frame Grabber Running: {self.frame_grabber.running}\n"
            info += f"Current Camera Index: {getattr(self.frame_grabber, 'camera_index', 'Unknown')}\n"
            info += f"Frame Dimensions: {getattr(self.frame_grabber, 'frame_width', 'Unknown')}x{getattr(self.frame_grabber, 'frame_height', 'Unknown')}\n"
            info += f"Face Detection: {'Enabled' if getattr(self.frame_grabber, 'enable_face_detection', False) else 'Disabled'}\n\n"
        
        # ROS Topics
        info += "Active ROS Publishers:\n"
        info += "- /coffee_bot/camera/image_raw (camera frames)\n"
        info += "- /coffee_bot/camera/status/info (status updates)\n"
        info += "- /coffee_bot/camera/status/available (camera list)\n"
        info += "- /coffee_bot/camera/status/diagnostics (this message)\n"
        info += "- /vision/face_position (face tracking)\n"
        info += "- face_detection_data (face data)\n\n"
        
        info += "Active ROS Subscribers:\n"
        info += "- /coffee_bot/camera/cmd/select (camera selection)\n"
        info += "- /coffee_bot/camera/cmd/quality (quality control)\n"
        info += "- /coffee_bot/camera/cmd/face_detection (face detection toggle)\n"
        info += "- /coffee_bot/camera/cmd/refresh (camera refresh)\n"
        info += "- /coffee_bot/camera/cmd/diagnostics (this request)\n"
        info += "- /coffee_bot/camera/query/state (state queries)\n\n"
        
        # Available cameras
        camera_count = len(self.available_cameras)
        info += f"Available Cameras: {camera_count}\n"
        for idx, name in self.available_cameras:
            info += f"  - {name} (index: {idx})\n"
        
        return info
    
    def _on_state_query(self, msg):
        """Handle state query from separated UI and respond with current camera state"""
        self.get_logger().info('Received state query from separated UI')
        
        # Gather current state directly from CameraNode
        current_state = {
            'camera_index': self.current_camera_index,
            'high_quality': self.high_quality,
            'face_detection_enabled': self.face_detection_enabled,
            'available_cameras': [
                {"index": idx, "name": name} 
                for idx, name in self.available_cameras
            ]
        }
        
        # Publish current state as JSON
        import json
        state_msg = String()
        state_msg.data = json.dumps(current_state)
        self.available_cameras_pub.publish(state_msg)  # Reuse existing publisher
        
        self.get_logger().info(f'Published current state: camera_index={current_state.get("camera_index", -1)}, '
                              f'high_quality={current_state.get("high_quality", False)}, '
                              f'face_detection={current_state.get("face_detection_enabled", True)}')
    
    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info('Shutting down camera node')
        
        # Stop the frame grabber
        if hasattr(self, 'frame_grabber'):
            try:
                self.frame_grabber.stop()
                self.get_logger().info('Frame grabber stopped')
            except Exception as e:
                self.get_logger().error(f'Error stopping frame grabber: {e}')
        
        # Clean up ROS resources
        super().destroy_node()


def main(args=None):
    try:
        # Initialize ROS2
        rclpy.init(args=args)
        
        # Create and run the node
        node = CameraNode()
        
        # Standard ROS spinning
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info('Camera node interrupted by user')
        finally:
            node.destroy_node()
            
    except Exception as e:
        print(f"Error in camera node: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()