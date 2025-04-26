#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from std_msgs.msg import String
from head_control_interfaces.srv import RequestControl
from dataclasses import dataclass
from typing import Optional
import time

@dataclass
class ControllerState:
    id: str
    priority: float
    timeout: float
    last_command: float
    start_time: float

class HeadControlManager(Node):
    """Head Control Manager Node
    
    Manages access to head servos between tracking and motion playback nodes.
    Implements priority-based control with timeouts and watchdog monitoring.
    """
    
    # Constants
    WATCHDOG_TIMEOUT = 1.0  # Seconds without command before considering controller dead
    DEFAULT_TIMEOUT = 5.0   # Default timeout for motion server control
    
    def __init__(self):
        super().__init__('head_control_manager')
        
        # Create callback groups
        self.timer_group = MutuallyExclusiveCallbackGroup()
        self.service_group = MutuallyExclusiveCallbackGroup()
        
        # State management
        self.default_controller = "head_tracking"
        self.current_controller: Optional[ControllerState] = None
        self.controllers = {}
        
        # Register known controllers with default priorities
        self.register_controller("head_tracking", priority=1.0)
        self.register_controller("head_motion_server", priority=2.0)
        
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
            self.handle_control_request,
            callback_group=self.service_group
        )
        
        # Create watchdog timer
        self.watchdog_timer = self.create_timer(
            0.1,  # 10Hz
            self.watchdog_check,
            callback_group=self.timer_group
        )
        
        # Publish initial status
        self.publish_status()
        self.get_logger().info('Head Control Manager initialized')
    
    def register_controller(self, controller_id: str, priority: float = 1.0):
        """Register a known controller with its default priority."""
        self.controllers[controller_id] = ControllerState(
            id=controller_id,
            priority=priority,
            timeout=0.0,
            last_command=0.0,
            start_time=0.0
        )
    
    def handle_position_command(self, msg: SetPosition, controller_id: str):
        """Handle incoming position commands and filter based on current controller."""
        now = time.time()
        
        # Update controller's last command time
        if controller_id in self.controllers:
            self.controllers[controller_id].last_command = now
        
        # Only forward commands from current controller
        if (self.current_controller and 
            self.current_controller.id == controller_id):
            self.position_pub.publish(msg)
            self.get_logger().debug(f'Forwarded command from {controller_id}')
    
    def handle_control_request(self, request: RequestControl.Request, 
                             response: RequestControl.Response):
        """Handle requests for control of the head."""
        now = time.time()
        
        # Validate controller
        if request.controller_id not in self.controllers:
            response.success = False
            response.message = f"Unknown controller: {request.controller_id}"
            return response
        
        controller = self.controllers[request.controller_id]
        response.current_owner = self.current_controller.id if self.current_controller else "none"
        
        # Handle force takeover
        if request.force:
            self.grant_control(controller, request.timeout)
            response.success = True
            response.message = f"Control forced to {request.controller_id}"
            response.granted_timeout = controller.timeout
            return response
        
        # Check if requester has higher priority
        if (self.current_controller and 
            controller.priority <= self.current_controller.priority and
            not self.is_controller_expired(self.current_controller)):
            response.success = False
            response.message = "Insufficient priority for control"
            return response
        
        # Grant control
        timeout = request.timeout if request.timeout > 0 else self.DEFAULT_TIMEOUT
        self.grant_control(controller, timeout)
        
        response.success = True
        response.message = f"Control granted to {request.controller_id}"
        response.granted_timeout = timeout
        return response
    
    def grant_control(self, controller: ControllerState, timeout: float):
        """Grant control to a controller."""
        now = time.time()
        controller.start_time = now
        controller.timeout = timeout
        controller.last_command = now
        self.current_controller = controller
        self.publish_status()
        self.get_logger().info(
            f'Control granted to {controller.id} for {timeout:.1f}s'
        )
    
    def is_controller_expired(self, controller: ControllerState) -> bool:
        """Check if a controller's control has expired."""
        now = time.time()
        
        # Check timeout
        if (controller.timeout > 0 and 
            now - controller.start_time > controller.timeout):
            return True
        
        # Check watchdog
        if now - controller.last_command > self.WATCHDOG_TIMEOUT:
            return True
        
        return False
    
    def watchdog_check(self):
        """Periodic check for expired controllers."""
        if not self.current_controller:
            return
        
        if self.is_controller_expired(self.current_controller):
            self.get_logger().info(
                f'Controller {self.current_controller.id} expired, '
                'reverting to default'
            )
            self.current_controller = self.controllers[self.default_controller]
            self.publish_status()
    
    def publish_status(self):
        """Publish current control status."""
        status_msg = String()
        controller_id = self.current_controller.id if self.current_controller else "none"
        status_msg.data = f"current_controller:{controller_id}"
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HeadControlManager()
    
    # Use MultiThreadedExecutor for concurrent callbacks
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
