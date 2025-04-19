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
import mediapipe as mp
import math
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QLabel, QPushButton, QComboBox, QHBoxLayout, QCheckBox, QMessageBox
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt, pyqtSignal, QObject
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Models directory for face detection models
MODELS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models")
os.makedirs(MODELS_DIR, exist_ok=True)

# 3D model points for head pose estimation
# Standard 3D facial model coordinates
MODEL_POINTS = np.array([
    (0.0, 0.0, 0.0),             # Nose tip
    (0.0, -330.0, -65.0),        # Chin
    (-225.0, 170.0, -135.0),     # Left eye left corner
    (225.0, 170.0, -135.0),      # Right eye right corner
    (-150.0, -150.0, -125.0),    # Left mouth corner
    (150.0, -150.0, -125.0)      # Right mouth corner
], dtype=np.float64)

# MediaPipe indices for key facial landmarks corresponding to MODEL_POINTS
# (these indices are specific to MediaPipe Face Mesh)
FACE_LANDMARK_INDICES = [
    1,    # Nose tip
    152,  # Chin
    33,   # Left eye left corner
    263,  # Right eye right corner
    61,   # Left mouth corner
    291   # Right mouth corner
]

# MediaPipe face mesh initialization
mp_face_mesh = mp.solutions.face_mesh
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

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
        self.smoothing_factor = 0.4  # Higher value = more smoothing
        self.smoothing_frames = 5    # Number of frames to average
        
        # Face mesh detection for landmarks and head pose
        self.enable_head_pose = True
        self.face_mesh = None
        self.face_mesh_confidence = 0.5
        self.landmark_detection_interval = 3  # Process landmarks every N frames
        
        # Camera intrinsic parameters (will be updated based on frame size)
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Pose visualization settings
        self.show_landmarks = False
        self.show_axes = True
        self.axis_length = 100.0
        
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
        
        self.init_face_detector()
        self.init_face_mesh()
        self.update_camera_matrix()
    
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
                    "id": str(face.get("id", "Unknown")),
                    # Include pose data if available
                    "pose": face.get("pose", None)
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
            
            # Update camera matrix for new resolution
            self.update_camera_matrix()
            
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
    
    def toggle_head_pose(self, enable):
        """Enable or disable head pose estimation"""
        with self.lock:
            self.enable_head_pose = enable
            
            # Reinitialize face mesh if needed
            if enable and self.face_mesh is None:
                self.init_face_mesh()
    
    def toggle_landmarks(self, show):
        """Toggle landmark visualization"""
        self.show_landmarks = show
    
    def toggle_axes(self, show):
        """Toggle axes visualization"""
        self.show_axes = show
    
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
        """Draw transparent circles over detected faces with IDs and pose information"""
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
            
            # Draw head pose annotations if available
            if 'pose' in face and 'landmarks' in face:
                frame = self.draw_pose_annotations(frame, face, face['landmarks'], face['pose'])
        
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
    
    def init_face_mesh(self):
        """Initialize MediaPipe face mesh for landmark detection"""
        try:
            self.face_mesh = mp_face_mesh.FaceMesh(
                static_image_mode=False,
                max_num_faces=3,  # Limit to 3 faces for performance
                refine_landmarks=True,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5
            )
            
            if self.node:
                self.node.get_logger().info("MediaPipe Face Mesh initialized successfully")
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error initializing MediaPipe Face Mesh: {e}")
            else:
                print(f"Error initializing MediaPipe Face Mesh: {e}")
            self.face_mesh = None
    
    def update_camera_matrix(self):
        """Update camera intrinsic parameters based on frame size"""
        # Approximate camera matrix based on frame size
        focal_length = self.frame_width
        center = (self.frame_width / 2, self.frame_height / 2)
        self.camera_matrix = np.array(
            [[focal_length, 0, center[0]],
             [0, focal_length, center[1]],
             [0, 0, 1]], dtype=np.float64
        )
        
        # Assume no lens distortion
        self.dist_coeffs = np.zeros((4, 1))
        
        if self.node:
            self.node.get_logger().debug(f"Camera matrix updated for size {self.frame_width}x{self.frame_height}")
    
    def extract_face_mesh_landmarks(self, frame, face_rect):
        """Extract face mesh landmarks for a detected face"""
        if self.face_mesh is None:
            return None
        
        try:
            # Extract face ROI with margin
            x1, y1, x2, y2 = face_rect['x1'], face_rect['y1'], face_rect['x2'], face_rect['y2']
            
            # Add margin
            margin_x = int((x2 - x1) * 0.2)
            margin_y = int((y2 - y1) * 0.2)
            
            # Ensure coordinates are within frame
            h, w = frame.shape[:2]
            x1_m = max(0, x1 - margin_x)
            y1_m = max(0, y1 - margin_y)
            x2_m = min(w - 1, x2 + margin_x)
            y2_m = min(h - 1, y2 + margin_y)
            
            # Skip if ROI is too small
            if x2_m <= x1_m or y2_m <= y1_m:
                return None
            
            # Extract ROI
            face_roi = frame[y1_m:y2_m, x1_m:x2_m]
            
            # Process with MediaPipe
            # Convert to RGB for MediaPipe
            rgb_roi = cv2.cvtColor(face_roi, cv2.COLOR_BGR2RGB)
            
            # Process ROI with MediaPipe
            results = self.face_mesh.process(rgb_roi)
            
            if not results.multi_face_landmarks or len(results.multi_face_landmarks) == 0:
                return None
            
            # Get first face landmarks
            face_landmarks = results.multi_face_landmarks[0]
            
            # Convert landmarks to image coordinates
            roi_h, roi_w = face_roi.shape[:2]
            landmarks = []
            
            for idx, landmark in enumerate(face_landmarks.landmark):
                # Convert from normalized coordinates to pixel coordinates in ROI
                x = int(landmark.x * roi_w)
                y = int(landmark.y * roi_h)
                z = landmark.z
                
                # Adjust coordinates to original frame
                x_frame = x + x1_m
                y_frame = y + y1_m
                
                landmarks.append((idx, x_frame, y_frame, z))
            
            return landmarks
            
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error extracting face mesh landmarks: {e}")
            return None
    
    def calculate_head_pose(self, landmarks):
        """Calculate head pose using solvePnP from 2D landmarks"""
        if not landmarks or len(landmarks) < max(FACE_LANDMARK_INDICES) + 1:
            return None
        
        # Get 2D points for the 6 landmarks we need
        image_points = []
        for idx in FACE_LANDMARK_INDICES:
            for lm in landmarks:
                if lm[0] == idx:
                    image_points.append((lm[1], lm[2]))
                    break
        
        # Make sure we found all required landmarks
        if len(image_points) != len(FACE_LANDMARK_INDICES):
            return None
        
        image_points = np.array(image_points, dtype=np.float64)
        
        # Solve PnP problem
        success, rotation_vector, translation_vector = cv2.solvePnP(
            MODEL_POINTS, image_points, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE
        )
        
        if not success:
            return None
        
        # Calculate Euler angles
        rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
        pose_matrix = cv2.hconcat([rotation_matrix, translation_vector])
        _, _, _, _, _, _, euler_angles = cv2.decomposeProjectionMatrix(pose_matrix)
        
        # Convert angles to degrees
        pitch, yaw, roll = [float(angle) for angle in euler_angles]
        
        # Calculate lookAt vector (head direction)
        # The third column of the rotation matrix is the Z-axis
        z_axis = rotation_matrix[:, 2]
        look_at = {
            "x": float(z_axis[0]),
            "y": float(z_axis[1]),
            "z": float(z_axis[2])
        }
        
        # Return pose information
        return {
            "euler_angles": {
                "pitch": pitch,
                "yaw": yaw,
                "roll": roll
            },
            "rotation_vector": [float(v) for v in rotation_vector.flatten()],
            "translation_vector": [float(v) for v in translation_vector.flatten()],
            "look_at": look_at
        }
    
    def draw_pose_annotations(self, frame, face, landmarks, pose):
        """Draw head pose annotations on the frame"""
        if face is None or pose is None:
            return frame
        
        try:
            # Get face center
            center_x = face['center_x']
            center_y = face['center_y']
            
            # Draw landmarks if enabled
            if self.show_landmarks and landmarks:
                # Draw each landmark as a small circle
                for _, x, y, _ in landmarks:
                    cv2.circle(frame, (x, y), 1, (0, 255, 255), -1)
                
                # Draw connections for key landmarks used in pose estimation
                for i in range(len(FACE_LANDMARK_INDICES)):
                    idx = FACE_LANDMARK_INDICES[i]
                    for lm in landmarks:
                        if lm[0] == idx:
                            cv2.circle(frame, (lm[1], lm[2]), 3, (0, 0, 255), -1)
                            break
            
            # Draw axes if enabled
            if self.show_axes and pose:
                rotation_vector = np.array(pose['rotation_vector'], dtype=np.float64).reshape(3, 1)
                translation_vector = np.array(pose['translation_vector'], dtype=np.float64).reshape(3, 1)
                
                # Project axis points
                axis_points = self.axis_length * np.array([
                    [1, 0, 0],  # X-axis
                    [0, 1, 0],  # Y-axis
                    [0, 0, 1]   # Z-axis
                ], dtype=np.float64)
                
                # Convert to world coordinates
                axis_points_world = axis_points + np.array([0, 0, 0])
                
                # Project 3D axis points to image
                axis_points_2d, _ = cv2.projectPoints(
                    axis_points_world, rotation_vector, translation_vector,
                    self.camera_matrix, self.dist_coeffs
                )
                
                # Origin point (nose tip)
                origin = (center_x, center_y)
                
                # Draw each axis
                axis_colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0)]  # Red=X, Green=Y, Blue=Z
                
                for i, (point, color) in enumerate(zip(axis_points_2d, axis_colors)):
                    p = (int(point[0][0]), int(point[0][1]))
                    cv2.line(frame, origin, p, color, 2)
                
                # Add angle text
                euler_angles = pose['euler_angles']
                angle_text = f"Pitch: {euler_angles['pitch']:.1f}, Yaw: {euler_angles['yaw']:.1f}, Roll: {euler_angles['roll']:.1f}"
                cv2.putText(frame, angle_text, (face['x1'], face['y2'] + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                
                # Draw look-at vector
                look_at = pose['look_at']
                lx, ly, lz = look_at['x'], look_at['y'], look_at['z']
                length = math.sqrt(lx*lx + ly*ly + lz*lz)
                if length > 0:
                    # Normalize and scale
                    scale = 50.0  # Length of the arrow
                    lx = lx / length * scale
                    ly = ly / length * scale
                    lz = lz / length * scale
                    
                    # Project look-at point
                    look_point = np.array([[0, 0, scale]], dtype=np.float64)
                    look_point_2d, _ = cv2.projectPoints(
                        look_point, rotation_vector, translation_vector,
                        self.camera_matrix, self.dist_coeffs
                    )
                    
                    look_p = (int(look_point_2d[0][0][0]), int(look_point_2d[0][0][1]))
                    cv2.arrowedLine(frame, origin, look_p, (0, 255, 255), 2)
                
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error drawing pose annotations: {e}")
        
        return frame
    
    def process_face_with_landmarks(self, frame, face, frame_index):
        """Process a face to extract landmarks and estimate head pose"""
        # Only process landmarks every N frames for performance
        if frame_index % self.landmark_detection_interval != 0:
            return face
        
        # Extract landmarks
        landmarks = self.extract_face_mesh_landmarks(frame, face)
        
        # If landmarks found, calculate head pose
        if landmarks:
            pose = self.calculate_head_pose(landmarks)
            
            # Add pose to face data if calculated
            if pose:
                face['landmarks'] = landmarks
                face['pose'] = pose
        
        return face
    
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
                        
                        # Process landmarks and head pose for each face if enabled
                        if self.enable_head_pose and self.face_mesh:
                            for i, face in enumerate(last_faces):
                                last_faces[i] = self.process_face_with_landmarks(frame, face, frame_index)
                        
                        # Publish face data for other nodes
                        if self.node:
                            self.publish_face_data(last_faces)
                            self.publish_face_images(frame, last_faces)
                    
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
        self.head_pose_enabled = True
        self.show_landmarks = False
        self.show_axes = True
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
        
        # Head pose estimation toggle
        self.head_pose_checkbox = QCheckBox("Head Pose Estimation")
        self.head_pose_checkbox.setChecked(self.head_pose_enabled)
        self.head_pose_checkbox.stateChanged.connect(self.toggle_head_pose)
        quality_layout.addWidget(self.head_pose_checkbox)
        
        # Show landmarks toggle
        self.landmarks_checkbox = QCheckBox("Show Landmarks")
        self.landmarks_checkbox.setChecked(self.show_landmarks)
        self.landmarks_checkbox.stateChanged.connect(self.toggle_landmarks)
        quality_layout.addWidget(self.landmarks_checkbox)
        
        # Show axes toggle
        self.axes_checkbox = QCheckBox("Show Pose Axes")
        self.axes_checkbox.setChecked(self.show_axes)
        self.axes_checkbox.stateChanged.connect(self.toggle_axes)
        quality_layout.addWidget(self.axes_checkbox)
        
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
    
    def toggle_face_detection(self, state):
        """Toggle face detection on/off"""
        self.face_detection_enabled = bool(state)
        self.node.get_logger().info(f"Face detection {'enabled' if self.face_detection_enabled else 'disabled'}")
        self.frame_grabber.toggle_face_detection(self.face_detection_enabled)
        
        # Disable head pose if face detection is disabled
        if not self.face_detection_enabled and self.head_pose_enabled:
            self.head_pose_enabled = False
            self.head_pose_checkbox.setChecked(False)
            self.frame_grabber.toggle_head_pose(False)
    
    def toggle_head_pose(self, state):
        """Toggle head pose estimation on/off"""
        self.head_pose_enabled = bool(state)
        self.node.get_logger().info(f"Head pose estimation {'enabled' if self.head_pose_enabled else 'disabled'}")
        self.frame_grabber.toggle_head_pose(self.head_pose_enabled)
        
        # Make sure face detection is enabled if head pose is enabled
        if self.head_pose_enabled and not self.face_detection_enabled:
            self.face_detection_enabled = True
            self.face_detection_checkbox.setChecked(True)
            self.frame_grabber.toggle_face_detection(True)
    
    def toggle_landmarks(self, state):
        """Toggle landmark visualization"""
        self.show_landmarks = bool(state)
        self.node.get_logger().info(f"Landmark visualization {'enabled' if self.show_landmarks else 'disabled'}")
        self.frame_grabber.toggle_landmarks(self.show_landmarks)
    
    def toggle_axes(self, state):
        """Toggle axes visualization"""
        self.show_axes = bool(state)
        self.node.get_logger().info(f"Pose axes visualization {'enabled' if self.show_axes else 'disabled'}")
        self.frame_grabber.toggle_axes(self.show_axes)
    
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
            self.frame_grabber.toggle_head_pose(self.head_pose_enabled)
            self.frame_grabber.toggle_landmarks(self.show_landmarks)
            self.frame_grabber.toggle_axes(self.show_axes)
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
        
        # Register node parameters
        self.initialize_node_parameters()
        
        # Start a background thread for ROS spinning
        self.spinning = True
        self.ros_thread = threading.Thread(target=self.spin_thread)
        self.ros_thread.daemon = True
        self.ros_thread.start()
        
        # Start Qt event loop
        sys.exit(app.exec_())
    
    def initialize_node_parameters(self):
        """Declare configurable parameters for this node"""
        # Camera parameters
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('high_quality', False)
        self.declare_parameter('frame_width', 1280)
        self.declare_parameter('frame_height', 720)
        
        # Detection parameters
        self.declare_parameter('enable_face_detection', True)
        self.declare_parameter('enable_head_pose', True)
        self.declare_parameter('face_confidence_threshold', 0.5)
        self.declare_parameter('landmark_detection_interval', 3)
        
        # Visualization parameters
        self.declare_parameter('show_landmarks', False)
        self.declare_parameter('show_axes', True)
        self.declare_parameter('axis_length', 100.0)
    
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
            # Stop frame grabber
            try:
                # Release MediaPipe resources if initialized
                if self.ui.frame_grabber.face_mesh:
                    try:
                        self.ui.frame_grabber.face_mesh.close()
                    except:
                        pass
                
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