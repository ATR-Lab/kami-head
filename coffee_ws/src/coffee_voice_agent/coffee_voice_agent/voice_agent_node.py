#!/usr/bin/env python3
"""
Coffee Voice Agent ROS2 Node

A simple ROS2 wrapper for the coffee barista voice agent.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import logging

logger = logging.getLogger(__name__)


class CoffeeVoiceAgentNode(Node):
    """Simple ROS2 voice agent node"""
    
    def __init__(self):
        super().__init__('coffee_voice_agent')
        
        self.get_logger().info("Starting Coffee Voice Agent ROS2 Node...")
        
        # Create publishers
        self.state_pub = self.create_publisher(String, '/coffee_voice_agent/state', 10)
        self.emotion_pub = self.create_publisher(String, '/coffee_voice_agent/emotion', 10)
        
        # Create subscribers  
        self.virtual_request_sub = self.create_subscription(
            String,
            '/coffee_voice_agent/virtual_request',
            self.virtual_request_callback,
            10
        )
        
        # Timer for testing
        self.timer = self.create_timer(5.0, self.timer_callback)
        
        self.get_logger().info("Coffee Voice Agent ROS2 Node started successfully!")
    
    def virtual_request_callback(self, msg):
        """Handle virtual request messages"""
        self.get_logger().info(f"Received virtual request: {msg.data}")
    
    def timer_callback(self):
        """Test timer callback"""
        msg = String()
        msg.data = "active"
        self.state_pub.publish(msg)
        self.get_logger().info("Published state: active")


def main(args=None):
    """Main entry point"""
    
    rclpy.init(args=args)
    
    try:
        node = CoffeeVoiceAgentNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 