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
        
        # Create face publisher
        self.face_pub = node.create_publisher(String, 'face_detection_data', 10)
        
        # Create frame publisher
        self.frame_pub = node.create_publisher(Image, 'camera_frame', 10)
        
        # Create face image publisher
        self.face_image_pub = node.create_publisher(Image, 'face_images', 10)
        
        # Create CV bridge
        self.bridge = CvBridge()
        
        # Init face detector
        self.init_face_detector()
        
        # Camera thread
        self.camera_thread = None
        
    def init_face_detector(self):
        """Initialize OpenCV DNN face detector"""
        try:
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
            
            print("Face detector (OpenCV DNN) initialized successfully")
        except Exception as e:
            print(f"Error initializing face detector: {e}")
            self.face_net = None
            
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
            raise
            
    def detect_faces(self, frame):
        """Detect faces using OpenCV DNN detector"""
        if self.face_net is None:
            return []
            
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
                    faces.append({
                        'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
                        'center_x': (x1 + x2) // 2,
                        'center_y': (y1 + y2) // 2,
                        'radius': max((x2 - x1), (y2 - y1)) // 2,
                        'confidence': float(confidence)
                    })
                    
        return faces
        
    def start(self):
        """Start the camera thread"""
        print("Starting simple camera node")
        self.camera_thread = threading.Thread(target=self._camera_loop)
        self.camera_thread.daemon = True
        self.camera_thread.start()
        
    def _camera_loop(self):
        """Main camera processing loop"""
        print("Camera thread starting")
        
        # Try to open camera
        self.camera = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
        if not self.camera.isOpened():
            print(f"Failed to open camera at index {self.camera_index}")
            self.camera = cv2.VideoCapture(self.camera_index)  # Try default backend
            if not self.camera.isOpened():
                print("Failed to open camera with any backend")
                return
                
        print(f"Successfully opened camera at index {self.camera_index}")
        
        # Set resolution - 720p for better performance
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        # Frame counter
        frame_count = 0
        
        while self.running:
            try:
                # Read frame
                ret, frame = self.camera.read()
                if not ret:
                    print("Failed to read frame")
                    time.sleep(0.1)
                    continue
                
                # Detect faces
                if self.face_detection_enabled and frame_count % 3 == 0:  # Every 3rd frame
                    faces = self.detect_faces(frame)
                    if faces:
                        self.publish_face_data(faces)
                        self.publish_face_images(frame, faces)
                
                # Draw basic info on frame
                cv2.putText(frame, f"Camera: {self.camera_index}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                # Publish frame
                self.publish_frame(frame)
                
                # Update counter
                frame_count += 1
                
                # Short sleep to prevent CPU overuse
                time.sleep(0.01)
                
            except Exception as e:
                print(f"Error in camera loop: {e}")
                time.sleep(0.1)
                
        # Clean up
        if self.camera and self.camera.isOpened():
            self.camera.release()
            
    def stop(self):
        """Stop the camera thread"""
        self.running = False
        if self.camera_thread:
            self.camera_thread.join(timeout=2.0)
        if self.camera and self.camera.isOpened():
            self.camera.release()
            
    def publish_face_data(self, faces):
        """Publish face detection data"""
        if not faces:
            return
            
        # Get frame dimensions for the head tracking node
        frame_width = 1280  # Default frame width
        frame_height = 720  # Default frame height
        
        if self.camera:
            frame_width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
            frame_height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
            
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
                    "confidence": face["confidence"]
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