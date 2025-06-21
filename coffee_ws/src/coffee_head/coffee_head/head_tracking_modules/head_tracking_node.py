#!/usr/bin/env python3

import rclpy
import json
import time
import threading
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Vector3
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from dynamixel_sdk_custom_interfaces.srv import GetPosition

from .head_tracker import HeadTracker
from .motor_controller import MotorController
from ..common.data_types import FaceData, FrameData, VelocityVector


class HeadTrackingNode(Node):
    """ROS node for head tracking"""
    
    def __init__(self):
        super().__init__('head_tracking_node')
        self.get_logger().info('Head tracking node is starting...')
        
        # Create core components
        self.head_tracker = HeadTracker()
        self.motor_controller = MotorController()
        
        # Connect components
        self.setup_component_connections()
        
        # ROS publishers and subscribers
        self.create_publishers()
        self.create_subscriptions()
        
        # Parameters
        self.declare_parameters()
        
        # Timers
        self.create_timers()
        
        # Initialize head position
        self.init_head_position()
    
    def setup_component_connections(self):
        """Connect head tracker and motor controller callbacks"""
        # Set motor command callback
        self.motor_controller.set_command_callback(self.send_motor_command)
        
        # Set head tracker callbacks
        self.head_tracker.set_status_callback(self.update_status)
        self.head_tracker.set_velocity_callback(self.publish_velocity)
    
    def create_publishers(self):
        """Create ROS publishers"""
        qos = QoSProfile(depth=10)
        
        # Motor position publisher
        self.position_publisher = self.create_publisher(
            SetPosition,
            'set_position',
            qos
        )
        
        # Pan angle publisher (for eye tracking)
        self.pan_publisher = self.create_publisher(
            Float32,
            'head_pan_angle',
            10
        )
        
        # Tilt angle publisher (for eye tracking)
        self.tilt_publisher = self.create_publisher(
            Float32,
            'head_tilt_angle',
            10
        )
        
        # Face velocity publisher
        self.velocity_publisher = self.create_publisher(
            Vector3,
            'face_velocity',
            10
        )
        
        # Status publisher
        self.status_publisher = self.create_publisher(
            String,
            'head_tracking_status',
            10
        )
    
    def create_subscriptions(self):
        """Create ROS subscriptions"""
        # Face detection subscription
        self.face_subscription = self.create_subscription(
            String,
            'face_detection_data',
            self.face_data_callback,
            10
        )
    
    def declare_parameters(self):
        """Declare ROS parameters"""
        # Tracking parameters
        self.declare_parameter('enable_tracking', True)
        self.declare_parameter('pan_threshold', 40)
        self.declare_parameter('tilt_threshold', 25)
        self.declare_parameter('update_rate_hz', 30.0)
        self.declare_parameter('smoothing_factor', 0.8)
        self.declare_parameter('use_pid_smoothing', True)
        
        # Apply parameter values
        self.head_tracker.enable_tracking(
            self.get_parameter('enable_tracking').value
        )
        self.head_tracker.set_thresholds(
            self.get_parameter('pan_threshold').value,
            self.get_parameter('tilt_threshold').value
        )
        self.head_tracker.set_update_rate(
            self.get_parameter('update_rate_hz').value
        )
        self.head_tracker.set_smoothing_factor(
            self.get_parameter('smoothing_factor').value
        )
        self.head_tracker.set_pid_smoothing(
            self.get_parameter('use_pid_smoothing').value
        )
    
    def create_timers(self):
        """Create ROS timers"""
        # Timer for tracking updates
        self.tracking_timer = self.create_timer(
            1.0 / 60.0,  # 60 Hz update rate
            self.tracking_callback
        )
        
        # Timer for timeout checks
        self.timeout_timer = self.create_timer(
            1.0,  # 1 Hz check rate
            self.check_timeout
        )
    
    def init_head_position(self):
        """Initialize head position"""
        # Read current motor positions
        self.read_motor_positions()
        
        # Reset to default position
        self.motor_controller.reset_to_default()
    
    def read_motor_positions(self):
        """Read current motor positions"""
        self.get_logger().info('Reading initial motor positions...')
        self.get_motor_position(self.motor_controller.pan_motor_id)
        self.get_motor_position(self.motor_controller.tilt_motor_id)
    
    def get_motor_position(self, motor_id):
        """Get position for a specific motor"""
        client = self.create_client(GetPosition, 'get_position')
        
        # Don't block too long waiting for service
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Get position service not available')
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
            
            # Update motor controller with position feedback
            if motor_id == self.motor_controller.pan_motor_id:
                pan_position = position
                tilt_position = self.motor_controller.current_tilt_position
            else:
                pan_position = self.motor_controller.current_pan_position
                tilt_position = position
            
            self.motor_controller.set_current_positions(pan_position, tilt_position)
            
            # Log position
            motor_name = "Pan" if motor_id == self.motor_controller.pan_motor_id else "Tilt"
            angle = self.motor_controller.motor_position_to_degrees(
                position, 
                self.motor_controller.degrees_per_position
            ) % 360
            
            self.get_logger().info(f'{motor_name} motor position: {position} (angle: {angle:.1f}°)')
                
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
    
    def face_data_callback(self, msg):
        """Process face detection data"""
        try:
            # Parse the JSON data
            frame_data = FrameData.from_json(msg.data)
            
            # Update frame dimensions in head tracker
            self.head_tracker.set_frame_size(
                frame_data.frame_width,
                frame_data.frame_height
            )
            
            # Update face positions in head tracker
            self.head_tracker.update_faces(
                frame_data.faces,
                frame_data.timestamp
            )
            
        except Exception as e:
            self.get_logger().error(f'Error processing face data: {e}')
    
    def tracking_callback(self):
        """Periodic update for head tracking"""
        # Update head tracking
        movement = self.head_tracker.update()
        
        # If movement is needed, send motor commands
        if movement:
            pan_angle, tilt_angle = movement
            self.motor_controller.send_coordinated_movement(pan_angle, tilt_angle)
    
    def check_timeout(self):
        """Check for face data timeout"""
        self.head_tracker.check_timeout()
    
    def send_motor_command(self, motor_id, position):
        """Send command to motor via ROS publisher"""
        msg = SetPosition()
        msg.id = motor_id
        msg.position = position
        self.position_publisher.publish(msg)
        
        # Publish current angles for eye tracking coordination
        if motor_id == self.motor_controller.pan_motor_id:
            angle = self.motor_controller.motor_position_to_degrees(
                position, 
                self.motor_controller.degrees_per_position
            ) % 360
            angle_msg = Float32()
            angle_msg.data = float(angle)
            self.pan_publisher.publish(angle_msg)
            
        elif motor_id == self.motor_controller.tilt_motor_id:
            angle = self.motor_controller.motor_position_to_degrees(
                position, 
                self.motor_controller.degrees_per_position
            ) % 360
            angle_msg = Float32()
            angle_msg.data = float(angle)
            self.tilt_publisher.publish(angle_msg)
    
    def update_status(self, status):
        """Update and publish tracking status"""
        # Publish status message
        msg = String()
        msg.data = status
        self.status_publisher.publish(msg)
        
        # Log status
        self.get_logger().info(status)
    
    def publish_velocity(self, velocity: VelocityVector):
        """Publish face velocity data"""
        msg = Vector3()
        msg.x = float(velocity.x)
        msg.y = float(velocity.y)
        msg.z = float(velocity.magnitude)
        self.velocity_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    head_tracking_node = HeadTrackingNode()
    
    try:
        rclpy.spin(head_tracking_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        rclpy.shutdown()


if __name__ == '__main__':
    main() 