#!/usr/bin/env python3

import cv2
import rclpy
import time
import numpy as np
import threading
import os
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraNode(Node):
    """
    Simplified camera node for capturing and publishing camera frames.
    Focuses on core camera functionality without UI components.
    """
    
    def __init__(self):
        super().__init__('camera_node')
        self.get_logger().info('Camera node starting...')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Camera properties
        self.camera = None
        self.camera_index = 0
        self.frame_width = 640
        self.frame_height = 480
        self.target_fps = 30
        self.is_running = False
        
        # Threading for camera capture
        self.capture_thread = None
        self.frame_lock = threading.Lock()
        self.latest_frame = None
        
        # Publishers
        self.frame_publisher = self.create_publisher(
            Image, 
            '/camera/image_raw', 
            10
        )
        
        self.status_publisher = self.create_publisher(
            String,
            '/camera/status',
            10
        )
        
        # Parameters
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('target_fps', 30)
        
        # Get parameters
        self.camera_index = self.get_parameter('camera_index').get_parameter_value().integer_value
        self.frame_width = self.get_parameter('frame_width').get_parameter_value().integer_value
        self.frame_height = self.get_parameter('frame_height').get_parameter_value().integer_value
        self.target_fps = self.get_parameter('target_fps').get_parameter_value().integer_value
        
        # Initialize camera
        self.initialize_camera()
        
        # Create timer for publishing status
        self.create_timer(1.0, self.publish_status)
        
    def initialize_camera(self):
        """Initialize the camera with optimal settings"""
        try:
            self.camera = cv2.VideoCapture(self.camera_index)
            
            if not self.camera.isOpened():
                self.get_logger().error(f"Failed to open camera {self.camera_index}")
                return False
                
            # Set camera properties
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            self.camera.set(cv2.CAP_PROP_FPS, self.target_fps)
            self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimal latency
            
            # Verify settings
            actual_width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.camera.get(cv2.CAP_PROP_FPS)
            
            self.get_logger().info(f"Camera initialized: {actual_width}x{actual_height} @ {actual_fps}fps")
            
            # Start capture thread
            self.is_running = True
            self.capture_thread = threading.Thread(target=self._capture_loop)
            self.capture_thread.daemon = True
            self.capture_thread.start()
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Camera initialization failed: {e}")
            return False
    
    def _capture_loop(self):
        """Main camera capture loop running in separate thread"""
        while self.is_running and rclpy.ok():
            try:
                ret, frame = self.camera.read()
                if ret:
                    with self.frame_lock:
                        self.latest_frame = frame.copy()
                    
                    # Publish frame
                    self.publish_frame(frame)
                else:
                    self.get_logger().warning("Failed to capture frame")
                    time.sleep(0.1)
                    
            except Exception as e:
                self.get_logger().error(f"Capture error: {e}")
                time.sleep(0.1)
    
    def publish_frame(self, frame):
        """Publish camera frame as ROS Image message"""
        try:
            # Convert OpenCV image to ROS Image message
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = 'camera_frame'
            
            # Publish
            self.frame_publisher.publish(image_msg)
            
        except Exception as e:
            self.get_logger().error(f"Failed to publish frame: {e}")
    
    def publish_status(self):
        """Publish camera status"""
        try:
            if self.camera and self.camera.isOpened() and self.is_running:
                status = "active"
            else:
                status = "inactive"
                
            status_msg = String()
            status_msg.data = status
            self.status_publisher.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f"Failed to publish status: {e}")
    
    def get_latest_frame(self):
        """Get the latest captured frame (thread-safe)"""
        with self.frame_lock:
            return self.latest_frame.copy() if self.latest_frame is not None else None
    
    def cleanup(self):
        """Clean up camera resources"""
        self.get_logger().info("Cleaning up camera resources...")
        
        self.is_running = False
        
        if self.capture_thread and self.capture_thread.is_alive():
            self.capture_thread.join(timeout=2.0)
        
        if self.camera:
            self.camera.release()
            self.camera = None
            
        self.get_logger().info("Camera cleanup complete")
    
    def destroy_node(self):
        """Override destroy_node to ensure cleanup"""
        self.cleanup()
        super().destroy_node()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = CameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in camera node: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 