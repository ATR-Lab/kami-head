#!/usr/bin/env python3

import cv2
import rclpy
import numpy as np
import json
import os
import threading
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge

from .data_types import FaceData

class FaceDetectionNode(Node):
    """
    Face detection node that subscribes to camera images and publishes face detection results.
    Uses OpenCV DNN for face detection.
    """
    
    def __init__(self):
        super().__init__('face_detection_node')
        self.get_logger().info('Face detection node starting...')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Face detection parameters
        self.confidence_threshold = 0.5
        self.nms_threshold = 0.4
        self.face_net = None
        self.models_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models")
        os.makedirs(self.models_dir, exist_ok=True)
        
        # Detection smoothing
        self.face_history = []
        self.max_history = 5
        self.detection_lock = threading.Lock()
        
        # Publishers
        self.face_data_publisher = self.create_publisher(
            String,
            '/vision/face_detection_data',
            10
        )
        
        self.face_positions_publisher = self.create_publisher(
            Vector3,
            '/vision/face_position',
            10
        )
        
        self.status_publisher = self.create_publisher(
            String,
            '/vision/face_detection_status',
            10
        )
        
        # Subscribers
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Parameters
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('nms_threshold', 0.4)
        self.declare_parameter('enable_smoothing', True)
        
        # Get parameters
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.nms_threshold = self.get_parameter('nms_threshold').get_parameter_value().double_value
        self.enable_smoothing = self.get_parameter('enable_smoothing').get_parameter_value().bool_value
        
        # Initialize face detection model
        self.initialize_face_detector()
        
        # Create timer for publishing status
        self.create_timer(1.0, self.publish_status)
    
    def initialize_face_detector(self):
        """Initialize the OpenCV DNN face detection model"""
        try:
            # Model files
            model_file = os.path.join(self.models_dir, "opencv_face_detector_uint8.pb")
            config_file = os.path.join(self.models_dir, "opencv_face_detector.pbtxt")
            
            # Download models if they don't exist
            if not os.path.exists(model_file) or not os.path.exists(config_file):
                self.get_logger().info("Downloading face detection models...")
                self.download_face_models()
            
            # Load the DNN model
            self.face_net = cv2.dnn.readNetFromTensorflow(model_file, config_file)
            
            # Set computation backend (prefer CUDA if available)
            try:
                self.face_net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
                self.face_net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
                self.get_logger().info("Using CUDA backend for face detection")
            except:
                self.face_net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
                self.face_net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
                self.get_logger().info("Using CPU backend for face detection")
            
            self.get_logger().info("Face detection model initialized successfully")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize face detector: {e}")
            return False
    
    def download_face_models(self):
        """Download OpenCV face detection models"""
        import urllib.request
        
        base_url = "https://raw.githubusercontent.com/opencv/opencv_extra/master/testdata/dnn/"
        
        files = {
            "opencv_face_detector_uint8.pb": "opencv_face_detector_uint8.pb",
            "opencv_face_detector.pbtxt": "opencv_face_detector.pbtxt"
        }
        
        for filename, url_filename in files.items():
            url = base_url + url_filename
            filepath = os.path.join(self.models_dir, filename)
            
            try:
                self.get_logger().info(f"Downloading {filename}...")
                urllib.request.urlretrieve(url, filepath)
                self.get_logger().info(f"Downloaded {filename}")
            except Exception as e:
                self.get_logger().error(f"Failed to download {filename}: {e}")
                raise e
    
    def image_callback(self, msg):
        """Process incoming camera images for face detection"""
        try:
            # Convert ROS Image to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Detect faces
            faces = self.detect_faces(frame)
            
            # Apply smoothing if enabled
            if self.enable_smoothing:
                faces = self.smooth_detections(faces)
            
            # Publish results
            self.publish_face_data(faces)
            if faces:
                self.publish_face_positions(faces)
                
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
    
    def detect_faces(self, frame):
        """Detect faces in the given frame using OpenCV DNN"""
        if self.face_net is None:
            return []
        
        try:
            h, w = frame.shape[:2]
            
            # Create blob from image
            blob = cv2.dnn.blobFromImage(frame, 1.0, (300, 300), [104, 117, 123])
            
            # Set input to the network
            self.face_net.setInput(blob)
            
            # Run forward pass
            detections = self.face_net.forward()
            
            faces = []
            
            # Process detections
            for i in range(detections.shape[2]):
                confidence = detections[0, 0, i, 2]
                
                if confidence > self.confidence_threshold:
                    # Get bounding box coordinates
                    x1 = int(detections[0, 0, i, 3] * w)
                    y1 = int(detections[0, 0, i, 4] * h)
                    x2 = int(detections[0, 0, i, 5] * w)
                    y2 = int(detections[0, 0, i, 6] * h)
                    
                    # Ensure coordinates are within frame bounds
                    x1 = max(0, x1)
                    y1 = max(0, y1)
                    x2 = min(w, x2)
                    y2 = min(h, y2)
                    
                    # Calculate center
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2
                    
                    # Create FaceData object
                    face = FaceData(
                        x1=x1, y1=y1, x2=x2, y2=y2,
                        center_x=center_x, center_y=center_y,
                        confidence=float(confidence)
                    )
                    faces.append(face)
            
            return faces
            
        except Exception as e:
            self.get_logger().error(f"Face detection error: {e}")
            return []
    
    def smooth_detections(self, faces):
        """Apply temporal smoothing to face detections"""
        with self.detection_lock:
            # Add current detections to history
            self.face_history.append(faces)
            
            # Keep only recent history
            if len(self.face_history) > self.max_history:
                self.face_history.pop(0)
            
            # If we don't have enough history, return current detections
            if len(self.face_history) < 3:
                return faces
            
            # Apply simple averaging for smoothing
            # This is a basic implementation - could be improved with more sophisticated tracking
            smoothed_faces = []
            for face in faces:
                # Find similar faces in recent history and average their positions
                similar_faces = []
                for hist_faces in self.face_history[-3:]:  # Use last 3 frames
                    for hist_face in hist_faces:
                        # Check if faces are similar (overlap > 50%)
                        if self.calculate_overlap(face, hist_face) > 0.5:
                            similar_faces.append(hist_face)
                
                if similar_faces:
                    # Average the positions
                    avg_x1 = int(sum(f.x1 for f in similar_faces) / len(similar_faces))
                    avg_y1 = int(sum(f.y1 for f in similar_faces) / len(similar_faces))
                    avg_x2 = int(sum(f.x2 for f in similar_faces) / len(similar_faces))
                    avg_y2 = int(sum(f.y2 for f in similar_faces) / len(similar_faces))
                    
                    smoothed_face = FaceData(
                        x1=avg_x1, y1=avg_y1, x2=avg_x2, y2=avg_y2,
                        center_x=(avg_x1 + avg_x2) // 2,
                        center_y=(avg_y1 + avg_y2) // 2,
                        confidence=face.confidence
                    )
                    smoothed_faces.append(smoothed_face)
                else:
                    smoothed_faces.append(face)
            
            return smoothed_faces
    
    def calculate_overlap(self, face1, face2):
        """Calculate overlap ratio between two face bounding boxes"""
        # Calculate intersection area
        x1 = max(face1.x1, face2.x1)
        y1 = max(face1.y1, face2.y1)
        x2 = min(face1.x2, face2.x2)
        y2 = min(face1.y2, face2.y2)
        
        if x1 >= x2 or y1 >= y2:
            return 0.0
        
        intersection = (x2 - x1) * (y2 - y1)
        
        # Calculate union area
        area1 = face1.area
        area2 = face2.area
        union = area1 + area2 - intersection
        
        return intersection / union if union > 0 else 0.0
    
    def publish_face_data(self, faces):
        """Publish face detection data as JSON string"""
        try:
            face_list = [face.to_dict() for face in faces]
            
            data = {
                'timestamp': self.get_clock().now().to_msg().sec,
                'faces': face_list,
                'count': len(faces)
            }
            
            msg = String()
            msg.data = json.dumps(data)
            self.face_data_publisher.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Failed to publish face data: {e}")
    
    def publish_face_positions(self, faces):
        """Publish position of the largest/closest face"""
        try:
            if not faces:
                return
            
            # Find the largest face (closest to camera)
            largest_face = max(faces, key=lambda f: f.area)
            
            # Publish position as Vector3 (x, y, confidence)
            position_msg = Vector3()
            position_msg.x = float(largest_face.center_x)
            position_msg.y = float(largest_face.center_y)
            position_msg.z = float(largest_face.confidence)
            
            self.face_positions_publisher.publish(position_msg)
            
        except Exception as e:
            self.get_logger().error(f"Failed to publish face position: {e}")
    
    def publish_status(self):
        """Publish face detection status"""
        try:
            if self.face_net is not None:
                status = "active"
            else:
                status = "inactive"
                
            status_msg = String()
            status_msg.data = status
            self.status_publisher.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f"Failed to publish status: {e}")


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = FaceDetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in face detection node: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 