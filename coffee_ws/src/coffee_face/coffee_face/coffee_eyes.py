#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import pygame
import time
import json
from .bezier.bezier_viz import AnimatedEyes, EyeConfig
from .controllers.eye_controller import EyeController
from std_msgs.msg import String
from geometry_msgs.msg import Vector3


class CoffeeEyesNode(Node):
    def __init__(self):
        super().__init__('coffee_eyes')
        
        # Set up logging
        self.get_logger().info('Starting Coffee Eyes Node')
        
        # Initialize Pygame for the eye display
        pygame.init()
        
        # Create the animated eyes and controller
        self.config = EyeConfig(
            width=120,
            height=480,
            spacing=140,
            blink_interval=120,
            blink_speed=0.1,
            blink_close_frames=5,
            outline_color=(255, 255, 255),
            fill_color=(255, 255, 255),
            outline_width=2,
            background_color=(0, 0, 0)
        )
        
        self.animated_eyes = AnimatedEyes(1080, 600, self.config)
        # Set node reference for debug logging
        self.animated_eyes.node = self
        self.controller = EyeController(self.animated_eyes)
        
        # Set up timer for regular updates (60 Hz)
        self.update_timer = self.create_timer(1/60, self.update_callback)
        
        # Add parameters for customization
        self.declare_parameter('screen_width', 1080)
        self.declare_parameter('screen_height', 600)
        self.declare_parameter('movement_speed', 1.0)
        self.declare_parameter('face_tracking_enabled', True)
        
        # Add parameters for camera frame dimensions
        self.declare_parameter('frame_width', 640)  # Default camera width
        self.declare_parameter('frame_height', 480)  # Default camera height
        
        # Add parameters for mapping
        self.declare_parameter('invert_x', False)  # Default FALSE for correct eye movement
        self.declare_parameter('invert_y', False)  # Default FALSE for correct eye movement
        self.declare_parameter('eye_range', 3.0)   # Max range for eye movement (-3.0 to 3.0)
        
        # Get parameters
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        self.invert_x = self.get_parameter('invert_x').value
        self.invert_y = self.get_parameter('invert_y').value
        self.eye_range = self.get_parameter('eye_range').value
        self.face_tracking_enabled = self.get_parameter('face_tracking_enabled').value
        
        # Initialize tracking related variables
        self.face_positions = []
        self.target_face_position = None
        self.last_face_update = time.time()
        self.face_lost_timeout = 1.0  # Seconds before considering face lost
        
        # Subscribe to face detection data
        if self.face_tracking_enabled:
            self.face_subscription = self.create_subscription(
                String,
                'face_detection_data',
                self.face_data_callback,
                10
            )
            self.get_logger().info('Subscribed to face_detection_data')
            
            # Subscribe to face velocity
            self.velocity_subscription = self.create_subscription(
                Vector3,
                'face_velocity',
                self.face_velocity_callback,
                10
            )
            self.get_logger().info('Subscribed to face_velocity')
        
        # Initialize flags
        self.running = True
        
    def face_data_callback(self, msg):
        """Process incoming face detection data"""
        try:
            # Parse the JSON data
            data = json.loads(msg.data)
            
            # Update frame dimensions if provided
            if 'frame_width' in data and 'frame_height' in data:
                self.frame_width = data['frame_width']
                self.frame_height = data['frame_height']
            
            # Update face positions
            self.face_positions = data.get('faces', [])
            self.last_face_update = time.time()
            
            # If no faces detected, just return
            if not self.face_positions:
                self.target_face_position = None
                return
                
            # Select target face (largest/closest)
            largest_area = 0
            largest_face = None
            
            for face in self.face_positions:
                width = face['x2'] - face['x1']
                height = face['y2'] - face['y1']
                area = width * height
                
                if area > largest_area:
                    largest_area = area
                    largest_face = face
            
            if largest_face:
                self.target_face_position = (
                    largest_face['center_x'], 
                    largest_face['center_y']
                )
                
                # Log face position before transformation
                face_x = self.target_face_position[0]
                face_y = self.target_face_position[1]
                center_x = self.frame_width / 2
                center_y = self.frame_height / 2
                dx = face_x - center_x
                dy = face_y - center_y
                
                self.get_logger().debug(f"Face detected at ({face_x:.1f}, {face_y:.1f}), offset from center: ({dx:.1f}, {dy:.1f})")
                
                # Transform camera coordinates to eye controller coordinates
                eye_position = self.transform_camera_to_eye_coords(
                    self.target_face_position[0],
                    self.target_face_position[1]
                )
                
                # Call go_to_pos only if we have a valid position
                if eye_position:
                    self.controller.go_to_pos(eye_position)
                    self.get_logger().info(f'Moving eyes to position: ({eye_position[0]:.2f}, {eye_position[1]:.2f})')

        except Exception as e:
            self.get_logger().error(f"Error processing face data: {e}")
    
    def face_velocity_callback(self, msg):
        """Process incoming face velocity data (for future use)"""
        # Could use velocity to predict or smooth eye movement
        pass
    
    def transform_camera_to_eye_coords(self, camera_x, camera_y):
        """Transform camera coordinates to eye controller coordinates (-3.0 to 3.0 range)"""
        # Normalize to -1.0 to 1.0
        # Note: We invert the coordinates to ensure proper eye direction
        # (When face is on right side, eyes should look right)
        norm_x = (camera_x - self.frame_width/2) / (self.frame_width/2)
        norm_y = (camera_y - self.frame_height/2) / (self.frame_height/2)
        
        # Add sensitivity multiplier (like in eye_tracking.py)
        sensitivity = 1.5  # Higher = more sensitive eye movement
        norm_x *= sensitivity
        norm_y *= sensitivity
        
        # Apply inversions if configured
        # Note: By default we want norm_x to be positive when face is on right side
        # So default should have invert_x=False
        if self.invert_x:
            norm_x = -norm_x
        if self.invert_y:
            norm_y = -norm_y
        
        # Scale to eye controller range (-3.0 to 3.0)
        eye_x = norm_x * self.eye_range
        eye_y = norm_y * self.eye_range
        
        # Clamp values to valid range
        eye_x = max(-self.eye_range, min(self.eye_range, eye_x))
        eye_y = max(-self.eye_range, min(self.eye_range, eye_y))
        
        # Debug output for tuning
        self.get_logger().debug(f'Camera coords: ({camera_x}, {camera_y}) -> Eye coords: ({eye_x}, {eye_y})')
        
        return (eye_x, eye_y)
    
    def update_callback(self):
        """Main update function called by the timer"""
        if not self.running:
            return
            
        # Check if pygame is still running
        running = self.animated_eyes.process_events()
        if not running:
            self.get_logger().info('Pygame window closed, shutting down node')
            self.running = False
            rclpy.shutdown()
            return
        
        # Update controller (handles movement)
        self.controller.update()
        
        # Update the animation frame
        self.animated_eyes.run_single_frame()
    
    def on_shutdown(self):
        """Cleanup when the node is shutting down"""
        pygame.quit()
        self.get_logger().info('Coffee Eyes Node shutting down')


def main(args=None):
    rclpy.init(args=args)
    
    node = CoffeeEyesNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()