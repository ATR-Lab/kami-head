#!/usr/bin/env python3

import rclpy
import pygame
import numpy as np
from rclpy.node import Node
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from geometry_msgs.msg import Vector3

class JoystickControlNode(Node):
    """Node for controlling robot head using a gaming joystick"""
    
    def __init__(self):
        super().__init__('joystick_control_node')
        
        # Initialize pygame and joystick
        pygame.init()
        pygame.joystick.init()
        
        # Verify joystick is connected
        if pygame.joystick.get_count() == 0:
            self.get_logger().error('No joystick found!')
            return
            
        # Initialize the first joystick
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f'Initialized joystick: {self.joystick.get_name()}')
        
        # Publishers for head control
        self.yaw_pub = self.create_publisher(
            SetPosition, 
            'set_position_yaw',  # Topic name for yaw control
            10
        )
        self.pitch_pub = self.create_publisher(
            SetPosition, 
            'set_position_pitch',  # Topic name for pitch control
            10
        )
        self.roll_pub = self.create_publisher(
            SetPosition, 
            'set_position_roll',  # Topic name for roll control
            10
        )
        
        # Control parameters
        self.deadzone = 0.1  # Ignore small joystick movements
        self.max_angle = 1.0  # Maximum angle in radians
        self.smooth_factor = 0.3  # For smooth motion (0 = no smoothing, 1 = max smoothing)
        
        # Current position tracking
        self.current_yaw = 0.0
        self.current_pitch = 0.0
        self.current_roll = 0.0
        
        # Create timer for joystick polling
        self.create_timer(0.02, self.joystick_callback)  # 50Hz update rate
        
    def map_joystick_to_angle(self, value, invert=False):
        """Map joystick value (-1 to 1) to angle with deadzone"""
        if abs(value) < self.deadzone:
            return 0.0
            
        # Apply deadzone and map to range
        mapped = np.sign(value) * (abs(value) - self.deadzone) / (1 - self.deadzone)
        mapped *= self.max_angle
        
        return -mapped if invert else mapped
        
    def smooth_motion(self, current, target):
        """Apply smoothing to motion"""
        return current + (target - current) * (1.0 - self.smooth_factor)
        
    def joystick_callback(self):
        """Process joystick input and publish commands"""
        try:
            # Process pygame events to get fresh joystick data
            pygame.event.pump()
            
            # Read joystick axes
            yaw_input = self.joystick.get_axis(0)  # Left stick X-axis
            pitch_input = self.joystick.get_axis(3)  # Right stick Y-axis
            roll_left = self.joystick.get_axis(2)  # Left trigger
            roll_right = self.joystick.get_axis(5)  # Right trigger
            
            # Map inputs to angles
            target_yaw = self.map_joystick_to_angle(yaw_input)
            target_pitch = self.map_joystick_to_angle(pitch_input)
            
            # Combine triggers for roll (-1 = full left, 1 = full right)
            roll_input = (roll_right - roll_left) / 2.0
            target_roll = self.map_joystick_to_angle(roll_input)
            
            # Apply smoothing
            self.current_yaw = self.smooth_motion(self.current_yaw, target_yaw)
            self.current_pitch = self.smooth_motion(self.current_pitch, target_pitch)
            self.current_roll = self.smooth_motion(self.current_roll, target_roll)
            
            # Create and publish messages
            yaw_msg = SetPosition()
            yaw_msg.position = int(self.current_yaw * 1000)  # Convert to motor units
            self.yaw_pub.publish(yaw_msg)
            
            pitch_msg = SetPosition()
            pitch_msg.position = int(self.current_pitch * 1000)
            self.pitch_pub.publish(pitch_msg)
            
            roll_msg = SetPosition()
            roll_msg.position = int(self.current_roll * 1000)
            self.roll_pub.publish(roll_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error reading joystick: {str(e)}')
            
    def destroy_node(self):
        """Clean up pygame on shutdown"""
        pygame.quit()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = JoystickControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
