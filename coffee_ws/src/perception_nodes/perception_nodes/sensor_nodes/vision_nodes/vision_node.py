#!/usr/bin/env python3

import rclpy
import cv2
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from shared_configs import (
    VISION_TOPIC,
    VISION_RESOLUTION_WIDTH,
    VISION_RESOLUTION_HEIGHT,
    VISION_CAMERA_INDEX,
    VISION_FPS,
    VISION_ENCODING_FORMAT,
    VISION_RESOLUTION_WIDTH_DEFAULT,
    VISION_RESOLUTION_HEIGHT_DEFAULT,
    VISION_CAMERA_INDEX_DEFAULT,
    VISION_FPS_DEFAULT,
    VISION_ENCODING_FORMAT_DEFAULT,
    VISION_NODE
)

class VisionNode(Node):
    def __init__(self):
        super().__init__(VISION_NODE)

        # ROS2 parameters
        self._declare_parameters()
        self._get_parameters()
        
        # CV2 Camera properties
        self.camera = None
        self.backend = cv2.CAP_ANY
        self.bridge = CvBridge()

        # ROS2 publishers
        self.frame_pub = self.create_publisher(Image, VISION_TOPIC, 10)

        # Initialize camera
        self._init_camera()
        
        # Set up timer for frame capture and publishing at the specified FPS
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self._timer_callback)

        self.get_logger().info(f"Vision node initialized with camera {self.camera_index}, "
                               f"resolution {self.resolution_width}x{self.resolution_height}, "
                               f"FPS {self.fps}, "
                               f"encoding format {self.encoding_format}")

    def _declare_parameters(self):
        """Declare all ROS2 parameters for this node."""
        self.declare_parameter(
            VISION_RESOLUTION_WIDTH,
            VISION_RESOLUTION_WIDTH_DEFAULT,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description='Resolution width of the camera'
            )
        )
        self.declare_parameter(
            VISION_RESOLUTION_HEIGHT,
            VISION_RESOLUTION_HEIGHT_DEFAULT,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description='Resolution height of the camera'
            )
        )
        self.declare_parameter(
            VISION_CAMERA_INDEX,
            VISION_CAMERA_INDEX_DEFAULT,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description='Index of the camera to use'
            )
        )
        self.declare_parameter(
            VISION_FPS,
            VISION_FPS_DEFAULT,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description='FPS of the camera'
            )
        )
        self.declare_parameter(
            VISION_ENCODING_FORMAT, 
            VISION_ENCODING_FORMAT_DEFAULT, 
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Encoding format of the camera'
            )
        )

    def _get_parameters(self):
        """Get parameter values from the ROS2 parameter system."""
        self.resolution_width = self.get_parameter(VISION_RESOLUTION_WIDTH).value
        self.resolution_height = self.get_parameter(VISION_RESOLUTION_HEIGHT).value
        self.camera_index = self.get_parameter(VISION_CAMERA_INDEX).value
        self.fps = self.get_parameter(VISION_FPS).value
        self.encoding_format = self.get_parameter(VISION_ENCODING_FORMAT).value

    def _init_camera(self):
        """Initialize and configure the camera with specified parameters."""
        try:
            self.camera = cv2.VideoCapture(self.camera_index, self.backend)
            
            if not self.camera.isOpened():
                self.get_logger().error(f"Failed to open camera {self.camera_index}")
                return False
            
            # Define camera properties
            camera_props = {
                cv2.CAP_PROP_FRAME_WIDTH: self.resolution_width,
                cv2.CAP_PROP_FRAME_HEIGHT: self.resolution_height,
                cv2.CAP_PROP_FPS: self.fps,
                cv2.CAP_PROP_BUFFERSIZE: 1
            }
            
            # Apply camera properties
            for prop, value in camera_props.items():
                try:
                    self.camera.set(prop, value)
                except Exception as e:
                    self.get_logger().warn(f"Failed to set camera property {prop}: {e}")
            
            # Verify camera settings
            actual_width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.camera.get(cv2.CAP_PROP_FPS)
            
            # If settings don't match, try to enforce them
            if actual_width != self.resolution_width or actual_height != self.resolution_height or abs(actual_fps - self.fps) > 0.1:
                self.get_logger().info("Camera settings don't match requested values. Attempting to enforce settings...")
                
                # Some cameras need to be reset with explicit settings
                self.camera.release()
                self.camera = cv2.VideoCapture(self.camera_index, self.backend)
                
                # Apply settings
                for prop, value in camera_props.items():
                    try:
                        # First set to 0 to ensure reset
                        if prop in [cv2.CAP_PROP_FRAME_WIDTH, cv2.CAP_PROP_FRAME_HEIGHT, cv2.CAP_PROP_FPS]:
                            self.camera.set(prop, 0)
                        # Then set to desired value
                        self.camera.set(prop, value)
                    except:
                        pass
                
                # Re-verify settings
                actual_width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
                actual_height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
                actual_fps = self.camera.get(cv2.CAP_PROP_FPS)
            
            self.get_logger().info(f"Camera configured: {actual_width}x{actual_height} @ {actual_fps} FPS")
            
            # Final check if camera settings match requested settings
            if actual_width != self.resolution_width or actual_height != self.resolution_height:
                self.get_logger().warn(f"Camera resolution differs from requested: "
                                      f"got {actual_width}x{actual_height}, "
                                      f"wanted {self.resolution_width}x{self.resolution_height}")
                
            # Read a test frame to ensure camera is working
            ret, _ = self.camera.read()
            if not ret:
                self.get_logger().warn("Could not read initial test frame from camera")

            return True
            
        except Exception as e:
            self.get_logger().error(f"Error initializing camera: {e}")
            return False

    def _timer_callback(self):
        """Callback function for the timer to capture and publish frames."""
        if self.camera is None or not self.camera.isOpened():
            self.get_logger().error("Camera not available")
            return

        try:
            ret, frame = self.camera.read()
            if ret:
                self.publish_frame(frame)
            else:
                self.get_logger().warn("Failed to capture frame")
        except Exception as e:
            self.get_logger().error(f"Error in camera capture: {e}")

    def publish_frame(self, frame):
        """Publish camera frame to ROS topics"""            
        try:
            frame_msg = self.bridge.cv2_to_imgmsg(frame, encoding=self.encoding_format)
            frame_msg.header.stamp = self.get_clock().now().to_msg()
            self.frame_pub.publish(frame_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing frame: {e}")
    
    def destroy_node(self):
        """Clean up resources before shutting down."""
        self.get_logger().info("Shutting down vision node")
        if self.camera is not None and self.camera.isOpened():
            self.camera.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    vision_node = VisionNode()
    rclpy.spin(vision_node)
    vision_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()