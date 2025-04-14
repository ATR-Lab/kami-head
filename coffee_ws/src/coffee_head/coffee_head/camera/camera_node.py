#!/usr/bin/env python3

import rclpy
import threading
import time
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from .camera_manager import CameraManager
from .face_detector import FaceDetector
from ..common.data_types import FrameData, FaceData


class CameraNode(Node):
    """ROS node for camera operations and face detection"""
    
    def __init__(self):
        super().__init__('camera_node')
        self.get_logger().info('Camera node is starting...')
        
        # Camera and face detection components
        self.camera_manager = CameraManager()
        self.face_detector = FaceDetector()
        self.cv_bridge = CvBridge()
        
        # Configure camera manager with frame callback
        self.camera_manager.set_callback(self.process_frame)
        
        # Face detection settings
        self.enable_face_detection = True
        self.frame_width = 0
        self.frame_height = 0
        self.process_every_n_frames = 2  # Process every Nth frame for face detection
        self.frame_counter = 0
        
        # Publishers
        self.create_publishers()
        
        # Camera parameters
        self.declare_parameters()
        
        # Start the camera
        self.start_camera()
    
    def create_publishers(self):
        """Create ROS publishers"""
        # Face detection data publisher
        self.face_publisher = self.create_publisher(
            String,
            'face_detection_data',
            10
        )
        
        # Image publisher for visualization
        self.image_publisher = self.create_publisher(
            Image,
            'camera_image',
            10
        )
    
    def declare_parameters(self):
        """Declare ROS parameters"""
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('high_quality', False)
        self.declare_parameter('enable_face_detection', True)
        self.declare_parameter('confidence_threshold', 0.5)
        
        # Apply parameters
        self.camera_manager.set_camera_index(
            self.get_parameter('camera_index').value
        )
        self.camera_manager.set_high_quality(
            self.get_parameter('high_quality').value
        )
        self.enable_face_detection = self.get_parameter('enable_face_detection').value
        self.face_detector.set_confidence_threshold(
            self.get_parameter('confidence_threshold').value
        )
    
    def start_camera(self):
        """Start the camera"""
        if not self.camera_manager.start():
            self.get_logger().error('Failed to start camera')
    
    def process_frame(self, frame, timestamp):
        """Process frames from camera"""
        if frame is None:
            return
        
        # Update frame dimensions
        self.frame_width = frame.shape[1]
        self.frame_height = frame.shape[0]
        
        # Increment frame counter
        self.frame_counter += 1
        
        # Detect faces (on every Nth frame)
        faces = []
        if self.enable_face_detection and self.frame_counter % self.process_every_n_frames == 0:
            # Detect faces
            faces = self.face_detector.detect_faces(frame)
            
            # Apply smoothing to reduce jitter
            faces = self.face_detector.apply_smoothing(faces)
            
            # Draw faces on the frame (for visualization)
            frame = self.face_detector.draw_faces(frame, faces)
            
            # Publish face detection data
            self.publish_face_data(faces, timestamp)
        
        # Publish the processed frame
        self.publish_frame(frame)
    
    def publish_face_data(self, faces, timestamp):
        """Publish face detection data"""
        if not faces:
            # Empty face list
            frame_data = FrameData(
                frame_width=self.frame_width,
                frame_height=self.frame_height,
                faces=[],
                timestamp=timestamp
            )
        else:
            frame_data = FrameData(
                frame_width=self.frame_width,
                frame_height=self.frame_height,
                faces=faces,
                timestamp=timestamp
            )
        
        # Convert to JSON and publish
        json_data = frame_data.to_json()
        msg = String()
        msg.data = json_data
        self.face_publisher.publish(msg)
    
    def publish_frame(self, frame):
        """Publish the current frame as an image message"""
        try:
            # Convert to ROS image
            ros_image = self.cv_bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            
            # Publish the image
            self.image_publisher.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f'Error publishing image: {e}')
    
    def on_shutdown(self):
        """Cleanup on node shutdown"""
        self.get_logger().info('Shutting down camera node')
        self.camera_manager.stop()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        camera_node = CameraNode()
        
        # Add a shutdown hook to stop the camera
        camera_node.on_shutdown()
        
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        rclpy.shutdown()


if __name__ == '__main__':
    main() 