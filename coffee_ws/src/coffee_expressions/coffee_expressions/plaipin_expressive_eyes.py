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
        self.screen_width = 800
        self.screen_height = 400
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height))
        pygame.display.set_caption("Coffee Buddy - Plaipin Eyes")
        
        # Create custom eye configuration
        config = EyeConfig(
            width=60,  # Scaled down for 800x400 display
            height=240,  # Scaled down for 800x400 display
            spacing=70,  # Scaled down for 800x400 display
            blink_interval=120,
            blink_speed=0.1,
            blink_close_frames=5,
            outline_color=(255, 255, 255),
            fill_color=(255, 255, 255),
            outline_width=2,
            background_color=(0, 0, 0),
            base_screen_width=800,  # Match our display
            base_screen_height=400  # Match our display
        )
        
        # Initialize Plaipin application and controller
        self.app = Application(self.screen_width, self.screen_height, config)
        self.eye_controller = VizEyeController(self.app)
        
        # Initialize state
        self.current_expression = "neutral"
        self.last_update = self.get_clock().now()
        
        # Create subscription
        self.subscription = self.create_subscription(
            AffectiveState,
            'affective_state',
            self.affective_state_callback,
            10)
        
        # Create timer for animation updates
        self.create_timer(0.016, self.update_animation)  # ~60 FPS
        
    def affective_state_callback(self, msg: AffectiveState):
        """Handle incoming affective state messages."""
        # Map ROS expression to plaipin expression
        expression = msg.expression.lower()
        if expression == "happy":
            plaipin_expression = "joy1"
        elif expression == "curious":
            plaipin_expression = "curious"
        elif expression == "angry":
            plaipin_expression = "ang"
        elif expression == "sad":
            plaipin_expression = "sad_tired"
        elif expression == "loving":
            plaipin_expression = "leftheart"
        else:
            plaipin_expression = "neutral"
            
        # Set the expression
        self.eye_controller.set_expression(plaipin_expression)
        
        # Update gaze target if not idle
        if not msg.is_idle:
            # Convert ROS Point to normalized coordinates for plaipin
            # Assuming gaze_target is in the range [-1, 1] for x and y
            self.eye_controller.set_target_position(
                (msg.gaze_target.x, msg.gaze_target.y)
            )
        else:
            # Return to center when idle
            self.eye_controller.set_target_position((0.0, 0.0))
    
    def update_animation(self):
        """Update the animation state"""
        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.destroy_node()
                return
        
        # Update plaipin animation
        self.app.update()
        
        # Render
        self.screen.fill((0, 0, 0))  # Clear screen
        self.app.render()
        pygame.display.flip()
    
    def destroy_node(self):
        """Clean up resources."""
        pygame.quit()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PlaipinExpressiveEyes()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
