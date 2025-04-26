#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from std_msgs.msg import String
from head_control_interfaces.srv import RequestControl

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
        
        # Create subscribers for each controller
        self.tracking_sub = self.create_subscription(
            SetPosition,
            'head_tracking/set_position',
            lambda msg: self.handle_position_command(msg, "head_tracking"),
            10
        )
        
        self.motion_sub = self.create_subscription(
            SetPosition,
            'head_motion/set_position',
            lambda msg: self.handle_position_command(msg, "head_motion_server"),
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
        
        # Create service
        self.control_service = self.create_service(
            RequestControl,
            'request_head_control',
            self.handle_control_request
        )
        
        # Publish initial status
        self.publish_status()
        
        self.get_logger().info('Head Control Manager initialized')
    
    def handle_position_command(self, msg: SetPosition, controller_id: str):
        """Handle incoming position commands and filter based on current controller."""
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
            self.get_logger().debug(f'Forwarded command from {controller_id}')
    
    def handle_control_request(self, request: RequestControl.Request, 
                             response: RequestControl.Response):
        """Handle requests for control of the head."""
        response.current_owner = self.current_controller
        
        if request.controller_id == "head_motion_server":
            # Motion server can always take control
            self.set_motion_state(True)
            response.success = True
            response.message = "Control granted to motion server"
        elif request.controller_id == "head_tracking":
            # Tracking can only take control if no motion is in progress
            if not self.motion_in_progress:
                self.current_controller = "head_tracking"
                response.success = True
                response.message = "Control granted to tracking"
            else:
                response.success = False
                response.message = "Motion in progress, cannot take control"
        else:
            response.success = False
            response.message = f"Unknown controller: {request.controller_id}"
        
        self.publish_status()
        return response
    
    def set_motion_state(self, in_progress: bool):
        """Update the motion state and controller."""
        self.motion_in_progress = in_progress
        self.current_controller = "head_motion_server" if in_progress else self.default_controller
        self.publish_status()
        self.get_logger().info(f'Motion state set to: {in_progress}')
    
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
