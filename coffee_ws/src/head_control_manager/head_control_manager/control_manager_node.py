#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from std_msgs.msg import String

class HeadControlManager(Node):
    """
    Head Control Manager Node
    
    Manages access to head servos between tracking and motion playback nodes.
    """
    
    def __init__(self):
        super().__init__('head_control_manager')
        
        # State management
        self.default_controller = "head_tracking"
        self.current_controller = self.default_controller
        self.motion_in_progress = False
        
        # Create subscribers
        self.create_subscription(
            SetPosition,
            'set_position',
            self.handle_position_command,
            10
        )
        
        # Create publishers
        self.position_pub = self.create_publisher(
            SetPosition,
            'filtered_set_position',
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            'head_control_status',
            10
        )
        
        # Publish initial status
        self.publish_status()
        
        self.get_logger().info('Head Control Manager initialized')
    
    def handle_position_command(self, msg: SetPosition):
        """Handle incoming position commands and filter based on current controller."""
        # Extract controller ID from topic name or message
        # For now, we'll assume it's in the topic name
        # TODO: Add controller ID to message or use different topics
        
        controller_id = "head_tracking"  # Default assumption
        
        # Determine if command should be forwarded
        should_forward = False
        
        if controller_id == "head_motion_server":
            # Motion server always gets priority when it has control
            should_forward = self.motion_in_progress
        elif controller_id == "head_tracking":
            # Tracking commands only go through when no motion is in progress
            should_forward = not self.motion_in_progress
        
        # Forward command if allowed
        if should_forward:
            self.position_pub.publish(msg)
    
    def set_motion_state(self, in_progress: bool):
        """Update the motion state and controller."""
        self.motion_in_progress = in_progress
        self.current_controller = "head_motion_server" if in_progress else self.default_controller
        self.publish_status()
    
    def publish_status(self):
        """Publish current control status."""
        status_msg = String()
        status_msg.data = f"current_controller:{self.current_controller}"
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HeadControlManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
