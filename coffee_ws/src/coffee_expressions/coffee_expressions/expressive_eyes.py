#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pygame
import math
from geometry_msgs.msg import Point
from coffee_expressions.msg import AffectiveState

class BezierCurve:
    @staticmethod
    def quadratic(p0, p1, p2, t):
        x = (1 - t) * (1 - t) * p0[0] + 2 * (1 - t) * t * p1[0] + t * t * p2[0]
        y = (1 - t) * (1 - t) * p0[1] + 2 * (1 - t) * t * p1[1] + t * t * p2[1]
        return (x, y)

class Eye:
    def __init__(self, center_pos, size):
        self.center = center_pos
        self.size = size
        self.pupil_pos = (0, 0)  # Normalized position (-1 to 1)
        self.current_expression = "neutral"
        self.expression_transition = 0.0
        self.target_control_points = self._get_expression_control_points("neutral")
        self.current_control_points = self._get_expression_control_points("neutral")
        
    def _get_expression_control_points(self, expression):
        """Get Bezier control points for different expressions"""
        if expression == "happy":
            return [
                [(0, -0.8), (0.5, -1.0), (1.0, -0.8)],  # Upper curve
                [(1.0, -0.8), (0.5, -0.4), (0, -0.8)]   # Lower curve
            ]
        elif expression == "curious":
            return [
                [(0, -1.0), (0.5, -1.2), (1.0, -1.0)],  # Upper curve - raised
                [(1.0, -1.0), (0.5, -0.6), (0, -1.0)]   # Lower curve - normal
            ]
        else:  # neutral
            return [
                [(0, -1.0), (0.5, -1.0), (1.0, -1.0)],  # Upper curve
                [(1.0, -1.0), (0.5, -1.0), (0, -1.0)]   # Lower curve
            ]

    def update_expression(self, expression, dt):
        """Update the eye expression with smooth transition"""
        if expression != self.current_expression:
            self.target_control_points = self._get_expression_control_points(expression)
            self.current_expression = expression
            self.expression_transition = 0.0
        
        # Update transition
        if self.expression_transition < 1.0:
            self.expression_transition = min(1.0, self.expression_transition + dt * 2.0)
            target = self.target_control_points
            current = self.current_control_points
            
            # Interpolate control points
            for i in range(len(current)):
                for j in range(len(current[i])):
                    current[i][j] = (
                        current[i][j][0] + (target[i][j][0] - current[i][j][0]) * self.expression_transition,
                        current[i][j][1] + (target[i][j][1] - current[i][j][1]) * self.expression_transition
                    )

    def update_gaze(self, target_pos):
        """Update pupil position based on gaze target"""
        self.pupil_pos = (
            max(-1.0, min(1.0, target_pos[0])),
            max(-1.0, min(1.0, target_pos[1]))
        )

    def draw(self, surface):
        """Draw the eye using Bezier curves"""
        # Draw white background
        pygame.draw.ellipse(surface, (255, 255, 255),
                          (self.center[0] - self.size//2,
                           self.center[1] - self.size//2,
                           self.size, self.size))
        
        # Draw eye shape using Bezier curves
        points_upper = []
        points_lower = []
        steps = 20
        
        for i in range(steps + 1):
            t = i / steps
            # Scale and position the control points
            cp_upper = [(self.center[0] + p[0] * self.size//2,
                        self.center[1] + p[1] * self.size//2)
                       for p in self.current_control_points[0]]
            cp_lower = [(self.center[0] + p[0] * self.size//2,
                        self.center[1] + p[1] * self.size//2)
                       for p in self.current_control_points[1]]
            
            points_upper.append(BezierCurve.quadratic(cp_upper[0], cp_upper[1], cp_upper[2], t))
            points_lower.append(BezierCurve.quadratic(cp_lower[0], cp_lower[1], cp_lower[2], t))
        
        # Draw the curves
        if len(points_upper) > 1:
            pygame.draw.lines(surface, (0, 0, 0), False, points_upper, 2)
        if len(points_lower) > 1:
            pygame.draw.lines(surface, (0, 0, 0), False, points_lower, 2)
        
        # Draw pupil
        pupil_x = self.center[0] + self.pupil_pos[0] * self.size//4
        pupil_y = self.center[1] + self.pupil_pos[1] * self.size//4
        pupil_size = self.size // 5
        pygame.draw.circle(surface, (0, 0, 0),
                         (int(pupil_x), int(pupil_y)),
                         pupil_size)

class ExpressiveEyes(Node):
    def __init__(self):
        super().__init__('expressive_eyes')
        
        # Initialize Pygame
        pygame.init()
        self.screen_width = 800
        self.screen_height = 400
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height))
        pygame.display.set_caption("Coffee Buddy - Expressive Eyes")
        
        # Create eyes
        eye_size = 200
        left_center = (self.screen_width//4, self.screen_height//2)
        right_center = (3*self.screen_width//4, self.screen_height//2)
        self.left_eye = Eye(left_center, eye_size)
        self.right_eye = Eye(right_center, eye_size)
        
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
        
    def affective_state_callback(self, msg):
        """Handle incoming affective state messages"""
        self.current_expression = msg.expression.lower()
        
        # Convert gaze target to normalized coordinates
        # Assuming gaze_target is in meters, scale appropriately
        scale = 2.0  # Adjust this to control gaze sensitivity
        gaze_x = msg.gaze_target.x * scale
        gaze_y = msg.gaze_target.y * scale
        
        self.left_eye.update_gaze((gaze_x, gaze_y))
        self.right_eye.update_gaze((gaze_x, gaze_y))
        
    def update_animation(self):
        """Update animation state and render"""
        # Calculate delta time
        now = self.get_clock().now()
        dt = (now - self.last_update).nanoseconds / 1e9
        self.last_update = now
        
        # Update expressions
        self.left_eye.update_expression(self.current_expression, dt)
        self.right_eye.update_expression(self.current_expression, dt)
        
        # Clear screen
        self.screen.fill((200, 200, 200))  # Light gray background
        
        # Draw eyes
        self.left_eye.draw(self.screen)
        self.right_eye.draw(self.screen)
        
        # Update display
        pygame.display.flip()
        
        # Handle Pygame events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.destroy_node()
                pygame.quit()
                rclpy.shutdown()
                
    def destroy_node(self):
        """Clean up resources"""
        pygame.quit()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ExpressiveEyes()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
