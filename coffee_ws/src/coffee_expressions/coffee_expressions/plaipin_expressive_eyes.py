#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pygame
import time
from geometry_msgs.msg import Point
from coffee_expressions_msgs.msg import AffectiveState
import json
import os
import sys

# Add the src directory to Python path
src_path = os.path.join(os.path.dirname(__file__), 'plaipin', 'display_pi', 'src')
if src_path not in sys.path:
    sys.path.insert(0, src_path)

# Import plaipin modules
from viz.application import Application
from viz.config import EyeConfig
from viz.state_manager import StateManager
from viz_eye_controller import VizEyeController

class PlaipinExpressiveEyes(Node):
    """ROS2 node for controlling expressive eyes using the plaipin visualization package."""
    
    def __init__(self):
        super().__init__('plaipin_expressive_eyes')
        
        # Initialize Pygame and Plaipin components
        pygame.init()
        self.screen_width = 1080
        self.screen_height = 600
        # self.screen = pygame.display.set_mode((self.screen_width, self.screen_height))
        pygame.display.set_caption("Coffee Buddy - Plaipin Eyes")
        
        # Add parameters for mapping
        self.declare_parameter('invert_x', False)  # Default FALSE for correct eye movement
        self.declare_parameter('invert_y', False)  # Default FALSE for correct eye movement

        # The constraint for the values of the eye movement in the UI
        self.declare_parameter('eye_range', 1.0)   # Max range for eye movement (-3.0 to 3.0)

        self.invert_x = self.get_parameter('invert_x').value
        self.invert_y = self.get_parameter('invert_y').value
        self.eye_range = self.get_parameter('eye_range').value

        # Create custom eye configuration
        config = EyeConfig(
            width=200,  # Scaled down for 800x400 display
            height=720,  # Scaled down for 800x400 display
            spacing=140,  # Scaled down for 800x400 display
            blink_interval=120,
            blink_speed=0.1,
            blink_close_frames=5,
            outline_color=(255, 255, 255),
            fill_color=(255, 255, 255),
            outline_width=2,
            background_color=(0, 0, 0),
            base_screen_width=self.screen_width,    # Match our display
            base_screen_height=self.screen_height   # Match our display
        )
        
        # Get package share directory path
        from ament_index_python.packages import get_package_share_directory
        package_share_dir = get_package_share_directory('coffee_expressions')
        
        # Get absolute path to expressions.json in the package share directory
        expressions_path = os.path.join(package_share_dir, 'plaipin', 'display_pi', 'src', 'expressions.json')

        # Initialize Plaipin application and controller with expressions file path
        self.app = Application(self.screen_width, self.screen_height, config, expressions_file=expressions_path)
        self.eye_controller = VizEyeController(self.app)
        
        # Initialize state
        self.current_expression = "base_blob"  # Default neutral expression
        self.eye_controller.set_expression(self.current_expression)
        self.running = True
        
        # Create subscription
        self.subscription = self.create_subscription(
            AffectiveState,
            '/robot/affective_state',
            self.affective_state_callback,
            10)
    
    def affective_state_callback(self, msg: AffectiveState):
        """Handle incoming affective state messages."""
        # Map ROS2 expressions to plaipin expressions
        expression = msg.expression.lower()
        if expression == "happy":
            plaipin_expression = "joy1"
        elif expression == "angry":
            plaipin_expression = "ang"
        elif expression == "sad":
            plaipin_expression = "sad_1"
        elif expression == "loving":
            plaipin_expression = "base_waitingforyoutobuy"
        else:
            plaipin_expression = "base_blob"
        
        # Only update if expression changed
        if plaipin_expression != self.current_expression:
            self.current_expression = plaipin_expression
            self.eye_controller.set_expression(plaipin_expression)
        
        # Update gaze target if not idle
        if not msg.is_idle:
            # Convert ROS Point to normalized coordinates for plaipin
            # Assuming gaze_target is in the range [-1, 1] for x and y
            # TODO: COMMENTED THIS OUT
            # self.eye_controller.set_eye_positions(
            #     (msg.gaze_target.x, msg.gaze_target.y)
            # )
            self.handle_faces(msg.gaze_target_v2)
            # self.eye_controller.set_eye_positions((gaze_target_x, gaze_target_y))

        else:
            # Return to center when idle
            self.eye_controller.set_eye_positions((0.0, 0.0))
            # self.eye_controller.set_eye_positions((msg.gaze_target.x, msg.gaze_target.y))
    
    # SEE `face_data_callback` in `coffee_eyes.py` for details
    def handle_faces(self, msg):
        """Process incoming face detection data"""
        try:
            # Parse the JSON data
            data = json.loads(msg)
            
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
                    # self.controller.go_to_pos(eye_position)
                    self.eye_controller.set_eye_positions((eye_position[0], eye_position[1]))
                    self.get_logger().info(f'Moving eyes to position: ({eye_position[0]:.2f}, {eye_position[1]:.2f})')

        except Exception as e:
            self.get_logger().error(f"Error processing face data: {e}")
            

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
    
    def run(self):
        """Main animation loop"""
        clock = pygame.time.Clock()
        
        while self.running and rclpy.ok():
            # Handle events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                    break
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        self.running = False
                        break
                
                # Let the application handle other events
                self.app.input_handler.handle_keyboard(event)
                
                # Handle UI events
                ui_result = self.app.ui_manager.handle_event(event)
                if ui_result:
                    if ui_result == "control_points_changed":
                        self.app.animated_eyes.update_control_points()
                    elif ui_result == "config_changed":
                        self.app.animated_eyes.update_eye_positions()
            
            # Update the eye controller
            self.eye_controller.update()
            
            # Update animated eyes
            self.app.animated_eyes.update()
            
            # Drawing
            self.app.screen.fill(self.app.config.background_color)
            self.app.ui_manager.draw_grid()
            self.app.animated_eyes.draw()
            
            # Update display
            pygame.display.flip()
            
            # Maintain frame rate and process ROS callbacks
            clock.tick(60)
            rclpy.spin_once(self, timeout_sec=0)
    
    def destroy_node(self):
        """Clean up resources."""
        self.running = False
        pygame.quit()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = PlaipinExpressiveEyes()
    node.run()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
