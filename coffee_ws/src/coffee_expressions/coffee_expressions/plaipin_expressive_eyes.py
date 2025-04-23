#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pygame
import time
from geometry_msgs.msg import Point
from coffee_expressions_msgs.msg import AffectiveState
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
        
        # Create custom eye configuration
        config = EyeConfig(
            width=120,  # Scaled down for 800x400 display
            height=480,  # Scaled down for 800x400 display
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
            self.eye_controller.set_eye_positions(
                (msg.gaze_target.x, msg.gaze_target.y)
            )
        else:
            # Return to center when idle
            # self.eye_controller.set_eye_positions((0.0, 0.0))
            self.eye_controller.set_eye_positions((msg.gaze_target.x, msg.gaze_target.y))
    
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
