#!/usr/bin/env python3

import rclpy
import pygame
import math
import sys
import time
import json
import numpy as np
import random
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Vector3
from dynamixel_sdk_custom_interfaces.msg import SetPosition

class EyeTrackingNode(Node):
    """ROS node that displays a pair of eyes tracking faces using PyGame"""
    
    def __init__(self):
        super().__init__('eye_tracking_node')
        self.get_logger().info('Eye tracking node starting...')
        
        # Eye tracking parameters
        self.window_width = 800
        self.window_height = 600
        self.background_color = (240, 240, 240)  # Light gray
        
        # Eye properties
        self.eye_radius = 100
        self.pupil_radius = 40
        self.eye_spacing = 250  # Distance between eyes
        self.eye_color = (255, 255, 255)  # White
        self.pupil_color = (50, 50, 50)  # Dark gray
        self.outline_color = (0, 0, 0)  # Black
        self.outline_width = 3
        
        # Eye positions (center of each eye)
        self.left_eye_pos = (self.window_width//2 - self.eye_spacing//2, self.window_height//2)
        self.right_eye_pos = (self.window_width//2 + self.eye_spacing//2, self.window_height//2)
        
        # Pupil tracking
        self.max_pupil_displacement = self.eye_radius * 0.5  # Limit how far pupils can move
        self.mouse_pos = (self.window_width//2, self.window_height//2)  # Initial position
        
        # Face tracking data
        self.frame_width = 640  # Default camera frame size
        self.frame_height = 480
        self.face_positions = []  # List of detected faces from camera
        self.target_face_position = None  # Current target face position (x, y)
        self.face_velocity = (0, 0, 0)  # (vx, vy, magnitude)
        self.last_face_update = time.time()
        self.face_lost_timeout = 1.0  # Seconds before considering face lost
        
        # Head position (from motor feedback)
        self.pan_angle = 90.0   # Default center position (degrees)
        self.tilt_angle = 180.0  # Default center position (degrees)
        
        # These thresholds should match those in head_tracking.py
        # When face is within these thresholds, eyes move; outside, head moves
        self.center_threshold_x = 40  # Match pan_threshold from head_tracking.py
        self.center_threshold_y = 25  # Match tilt_threshold from head_tracking.py
        
        # Smooth eye movement
        self.pupil_target = {
            'left': (self.left_eye_pos[0], self.left_eye_pos[1]),
            'right': (self.right_eye_pos[0], self.right_eye_pos[1])
        }
        self.pupil_position = {
            'left': (self.left_eye_pos[0], self.left_eye_pos[1]),
            'right': (self.right_eye_pos[0], self.right_eye_pos[1])
        }
        self.eye_smoothing = 0.8  # Higher = smoother but slower eye movement
        
        # Idle behavior parameters
        self.idle_timer = 0
        self.idle_threshold = 3.0  # Seconds without face before idle behavior
        self.blink_timer = random.uniform(2.0, 5.0)  # Random initial blink time
        self.is_blinking = False
        self.blink_duration = 0.15  # Seconds per blink
        self.blink_start_time = 0
        self.idle_look_timer = random.uniform(1.0, 3.0)
        self.idle_look_direction = None
        
        # Publisher for eye gaze direction
        self.gaze_publisher = self.create_publisher(
            Vector3,
            'eye_gaze_direction',
            10
        )
        
        # Subscribe to face detection data
        self.face_subscription = self.create_subscription(
            String,
            'face_detection_data',
            self.face_data_callback,
            10
        )
        
        # Subscribe to face velocity
        self.velocity_subscription = self.create_subscription(
            Vector3,
            'face_velocity',
            self.face_velocity_callback,
            10
        )
        
        # Subscribe to head position (pan/tilt motor positions)
        self.pan_subscription = self.create_subscription(
            Float32,
            'head_pan_angle', 
            self.pan_angle_callback,
            10
        )
        
        self.tilt_subscription = self.create_subscription(
            Float32, 
            'head_tilt_angle',
            self.tilt_angle_callback,
            10
        )
        
        # Initialize pygame
        pygame.init()
        self.screen = pygame.display.set_mode((self.window_width, self.window_height))
        pygame.display.set_caption('Eye Tracking')
        self.clock = pygame.time.Clock()
        self.running = True
        
        # Timer for ROS updates
        self.create_timer(0.05, self.update_ros)  # 20Hz ROS updates
        
        # Debug visualization
        self.show_debug = True
        self.show_threshold_box = True
    
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
            self.face_positions = data['faces']
            self.last_face_update = time.time()
            
            # Reset idle timer when face detected
            if self.face_positions:
                self.idle_timer = 0
                
                # Select target face (largest/closest)
                if self.face_positions:
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
        
        except Exception as e:
            self.get_logger().error(f"Error processing face data: {e}")
    
    def face_velocity_callback(self, msg):
        """Process incoming face velocity data"""
        self.face_velocity = (msg.x, msg.y, abs(msg.z))  # (vx, vy, magnitude)
    
    def pan_angle_callback(self, msg):
        """Update current pan angle from motor feedback"""
        self.pan_angle = msg.data
    
    def tilt_angle_callback(self, msg):
        """Update current tilt angle from motor feedback"""
        self.tilt_angle = msg.data
        
    def update_ros(self):
        """Update ROS components and publish data"""
        # Process any pending callbacks
        rclpy.spin_once(self, timeout_sec=0)
        
        # Check if face is lost
        if time.time() - self.last_face_update > self.face_lost_timeout:
            self.target_face_position = None
            # Increment idle timer
            self.idle_timer += 0.05  # 20Hz update rate
    
    def map_camera_to_screen(self, camera_x, camera_y):
        """Map camera coordinates to screen coordinates"""
        # Calculate normalized position (-1 to 1) from camera frame
        # Note: We need to flip the normalized coordinates to correct the eye direction
        norm_x = (camera_x - self.frame_width/2) / (self.frame_width/2)  # Face on right → eyes look right
        norm_y = (camera_y - self.frame_height/2) / (self.frame_height/2)
        
        # Scale the movement - increase sensitivity within the threshold
        # Apply a multiplier to make eye movement more pronounced
        sensitivity = 1.5  # Higher = more sensitive eye movement
        norm_x *= sensitivity
        norm_y *= sensitivity
        
        # Scale to screen
        screen_x = self.window_width/2 + norm_x * (self.window_width/4)  # Limit range to 1/4 width
        screen_y = self.window_height/2 + norm_y * (self.window_height/4)  # Limit range to 1/4 height
        
        return (screen_x, screen_y)
    
    def calculate_pupil_position(self, eye_pos, target_pos):
        """Calculate pupil position based on target position and constraints"""
        # For default/centered position, return the eye center
        if target_pos == eye_pos:
            return eye_pos
            
        # Vector from eye to target
        dx = target_pos[0] - eye_pos[0]
        dy = target_pos[1] - eye_pos[1]
        
        # Calculate distance
        distance = math.sqrt(dx**2 + dy**2)
        
        # Normalize and scale
        if distance > 0:
            normalized_dx = dx / distance
            normalized_dy = dy / distance
        else:
            normalized_dx, normalized_dy = 0, 0
        
        # Limit displacement to max_pupil_displacement
        displacement = min(distance, self.max_pupil_displacement)
        
        # Calculate new pupil position
        pupil_x = eye_pos[0] + normalized_dx * displacement
        pupil_y = eye_pos[1] + normalized_dy * displacement
        
        return (pupil_x, pupil_y)
    
    def draw_eye(self, eye_pos, pupil_pos, is_blinking=False):
        """Draw an eye with pupil at the specified positions"""
        # Draw eye (white circle with black outline)
        pygame.draw.circle(self.screen, self.eye_color, eye_pos, self.eye_radius)
        pygame.draw.circle(self.screen, self.outline_color, eye_pos, self.eye_radius, self.outline_width)
        
        # If blinking, draw eyelid (black line across eye)
        if is_blinking:
            blink_progress = min(1.0, (time.time() - self.blink_start_time) / self.blink_duration)
            # Determine how closed the eye is (0 = open, 1 = closed, then back to 0)
            if blink_progress < 0.5:
                closure = blink_progress * 2  # 0 to 1 (closing)
            else:
                closure = (1 - blink_progress) * 2  # 1 to 0 (opening)
                
            # Draw upper and lower eyelids
            lid_offset = int(self.eye_radius * closure)
            upper_y = eye_pos[1] - self.eye_radius + lid_offset
            lower_y = eye_pos[1] + self.eye_radius - lid_offset
            
            # Draw filled rectangles for eyelids
            upper_rect = pygame.Rect(
                eye_pos[0] - self.eye_radius, 
                eye_pos[1] - self.eye_radius, 
                self.eye_radius * 2, 
                lid_offset
            )
            lower_rect = pygame.Rect(
                eye_pos[0] - self.eye_radius,
                lower_y,
                self.eye_radius * 2,
                lid_offset
            )
            
            pygame.draw.rect(self.screen, self.background_color, upper_rect)
            pygame.draw.rect(self.screen, self.background_color, lower_rect)
            
            # Draw outline again to maintain circle shape
            pygame.draw.circle(self.screen, self.outline_color, eye_pos, self.eye_radius, self.outline_width)
            
            # Only draw pupil if eye isn't completely closed
            if closure < 0.95:
                # Adjust pupil based on how closed the eye is
                adjusted_radius = int(self.pupil_radius * (1 - closure * 0.5))
                pygame.draw.circle(self.screen, self.pupil_color, pupil_pos, adjusted_radius)
        else:
            # Draw pupil (dark circle)
            pygame.draw.circle(self.screen, self.pupil_color, pupil_pos, self.pupil_radius)
    
    def update_idle_behavior(self):
        """Update idle behavior when no face is detected"""
        current_time = time.time()
        
        # Check for blink
        if not self.is_blinking and current_time - self.blink_timer > 0:
            self.is_blinking = True
            self.blink_start_time = current_time
            self.blink_timer = current_time + random.uniform(2.0, 5.0)
        
        # End blink if duration passed
        if self.is_blinking and current_time - self.blink_start_time > self.blink_duration:
            self.is_blinking = False
        
        # Random looking around when idle
        if self.idle_timer > self.idle_threshold:
            if current_time - self.idle_look_timer > 0:
                # Set a new random look direction
                angle = random.uniform(0, 2 * math.pi)
                distance = random.uniform(0.3, 0.8) * self.max_pupil_displacement
                dx = math.cos(angle) * distance
                dy = math.sin(angle) * distance
                
                center_x = self.window_width // 2
                center_y = self.window_height // 2
                self.idle_look_direction = (center_x + dx, center_y + dy)
                
                # Set next look change
                self.idle_look_timer = current_time + random.uniform(1.0, 3.0)
            
            return self.idle_look_direction
        
        return None
    
    def draw_debug_info(self):
        """Draw debug information on screen"""
        if not self.show_debug:
            return
            
        # Only draw debug info if we have a face to track
        if not self.target_face_position:
            return
            
        # Get face position data
        face_x = self.target_face_position[0]
        face_y = self.target_face_position[1]
        center_x = self.frame_width / 2
        center_y = self.frame_height / 2
        dx = face_x - center_x
        dy = face_y - center_y
        
        # Draw face position info
        font = pygame.font.SysFont(None, 24)
        
        # Face position text
        text = f"Face: ({int(face_x)}, {int(face_y)}) Offset: ({int(dx)}, {int(dy)})"
        text_surface = font.render(text, True, (0, 0, 0))
        self.screen.blit(text_surface, (10, 10))
        
        # Head position text
        head_text = f"Head: Pan={int(self.pan_angle)}°, Tilt={int(self.tilt_angle)}°"
        head_surface = font.render(head_text, True, (0, 0, 0))
        self.screen.blit(head_surface, (10, 40))
        
        # Tracking mode indicator
        within_x = abs(dx) <= self.center_threshold_x
        within_y = abs(dy) <= self.center_threshold_y
        
        if within_x and within_y:
            mode_text = "Mode: EYE tracking (within threshold)"
            color = (0, 180, 0)
        else:
            mode_text = "Mode: HEAD tracking (outside threshold)"
            color = (180, 0, 0)
        
        mode_surface = font.render(mode_text, True, color)
        self.screen.blit(mode_surface, (10, 70))
        
        # Draw crosshair at center to help with debugging centering
        center_x = self.window_width // 2
        center_y = self.window_height // 2
        pygame.draw.line(self.screen, (200, 200, 200), (center_x-10, center_y), (center_x+10, center_y), 1)
        pygame.draw.line(self.screen, (200, 200, 200), (center_x, center_y-10), (center_x, center_y+10), 1)
        
        # Draw lines showing where pupils are looking (debug for crossed eyes)
        left_gaze = pygame.draw.line(
            self.screen, 
            (200, 200, 200), 
            self.left_eye_pos, 
            self.pupil_position['left'], 
            1
        )
        
        right_gaze = pygame.draw.line(
            self.screen, 
            (200, 200, 200), 
            self.right_eye_pos, 
            self.pupil_position['right'], 
            1
        )
    
    def run(self):
        """Main loop for the eye tracking visualization"""
        try:
            while self.running:
                # Handle events
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        self.running = False
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            self.running = False
                        elif event.key == pygame.K_d:
                            # Toggle debug info with 'd' key
                            self.show_debug = not self.show_debug
                
                # Default - both eyes look straight ahead (centered)
                left_target = self.left_eye_pos
                right_target = self.right_eye_pos
                
                # If face is detected, decide whether to move eyes or head
                if self.target_face_position:
                    # Get face position
                    face_x = self.target_face_position[0]
                    face_y = self.target_face_position[1]
                    center_x = self.frame_width / 2
                    center_y = self.frame_height / 2
                    
                    # Calculate distance from center (in pixels)
                    dx = face_x - center_x
                    dy = face_y - center_y
                    
                    # Check if face is within movement thresholds
                    within_x_threshold = abs(dx) <= self.center_threshold_x
                    within_y_threshold = abs(dy) <= self.center_threshold_y
                    
                    if within_x_threshold and within_y_threshold:
                        # Face is within thresholds - MOVE EYES ONLY, head stays still
                        
                        # Calculate normalized offset from center (-1 to 1)
                        norm_x = dx / self.center_threshold_x  # -1 to 1 within threshold
                        norm_y = dy / self.center_threshold_y
                        
                        # Scale movement to use a percentage of maximum displacement
                        scale_factor = 0.8  # Use 80% of max pupil displacement
                        offset_x = norm_x * self.max_pupil_displacement * scale_factor
                        offset_y = norm_y * self.max_pupil_displacement * scale_factor
                        
                        # Set target positions for each eye - from center of each eye
                        left_target = (self.left_eye_pos[0] + offset_x, self.left_eye_pos[1] + offset_y)
                        right_target = (self.right_eye_pos[0] + offset_x, self.right_eye_pos[1] + offset_y)
                        
                        # Add velocity-based anticipation when within thresholds
                        if self.face_velocity[2] > 10:  # Only if significant movement
                            # Look slightly ahead in the direction of motion
                            anticipation_factor = 0.1  # Gentle anticipation
                            anticipation_x = self.face_velocity[0] * anticipation_factor
                            anticipation_y = self.face_velocity[1] * anticipation_factor
                            
                            # Apply to both eyes
                            left_target = (left_target[0] + anticipation_x, left_target[1] + anticipation_y)
                            right_target = (right_target[0] + anticipation_x, right_target[1] + anticipation_y)
                    else:
                        # Face is outside thresholds - MOVE HEAD ONLY, eyes look straight
                        # Keep pupils centered in eyes - already set to default above
                        pass
                        
                elif self.idle_timer > self.idle_threshold:
                    # Use idle behavior if no face and idle time exceeded
                    idle_target = self.update_idle_behavior()
                    if idle_target:
                        # Calculate offsets from center for idle movement
                        idle_x = idle_target[0] - self.window_width//2
                        idle_y = idle_target[1] - self.window_height//2
                        
                        # Apply same offset to both eyes
                        left_target = (self.left_eye_pos[0] + idle_x * 0.5, self.left_eye_pos[1] + idle_y * 0.5)
                        right_target = (self.right_eye_pos[0] + idle_x * 0.5, self.right_eye_pos[1] + idle_y * 0.5)
                
                # Update pupil targets directly
                self.pupil_target['left'] = self.calculate_pupil_position(self.left_eye_pos, left_target)
                self.pupil_target['right'] = self.calculate_pupil_position(self.right_eye_pos, right_target)
                
                # Smooth pupil movement
                for eye in ['left', 'right']:
                    current_pos = self.pupil_position[eye]
                    target_pos = self.pupil_target[eye]
                    
                    # Apply smoothing
                    new_x = current_pos[0] * self.eye_smoothing + target_pos[0] * (1 - self.eye_smoothing)
                    new_y = current_pos[1] * self.eye_smoothing + target_pos[1] * (1 - self.eye_smoothing)
                    
                    self.pupil_position[eye] = (new_x, new_y)
                
                # Clear screen
                self.screen.fill(self.background_color)
                
                # Draw threshold visualization if enabled
                if self.show_threshold_box and self.show_debug:
                    # Represent the camera view in the background
                    camera_ratio = self.frame_width / self.frame_height
                    
                    # Calculate camera view size scaled to fit in screen
                    if self.window_width / self.window_height > camera_ratio:
                        # Screen is wider than camera ratio
                        view_height = self.window_height * 0.8
                        view_width = view_height * camera_ratio
                    else:
                        # Screen is taller than camera ratio
                        view_width = self.window_width * 0.8
                        view_height = view_width / camera_ratio
                    
                    # Calculate view position (centered)
                    view_x = (self.window_width - view_width) / 2
                    view_y = (self.window_height - view_height) / 2
                    
                    # Draw camera frame outline
                    camera_frame = pygame.Rect(view_x, view_y, view_width, view_height)
                    pygame.draw.rect(self.screen, (180, 180, 180), camera_frame, 1)
                    
                    # Draw threshold box
                    # Calculate threshold box size scaled to camera view
                    threshold_width = (self.center_threshold_x * 2) * (view_width / self.frame_width)
                    threshold_height = (self.center_threshold_y * 2) * (view_height / self.frame_height)
                    
                    # Position the threshold box at center of view
                    threshold_x = view_x + (view_width - threshold_width) / 2
                    threshold_y = view_y + (view_height - threshold_height) / 2
                    
                    # Draw the threshold box
                    threshold_rect = pygame.Rect(threshold_x, threshold_y, threshold_width, threshold_height)
                    pygame.draw.rect(self.screen, (100, 200, 100), threshold_rect, 2)
                    
                    # Label the threshold box
                    font = pygame.font.SysFont(None, 18)
                    label = font.render(f"Eye Movement Zone (Head stationary)", True, (100, 200, 100))
                    self.screen.blit(label, (threshold_x, threshold_y - 20))
                
                # Update idle behavior (blinks, random movement)
                self.update_idle_behavior()
                
                # Draw eye sclera (white part) with gradient effect
                pygame.draw.circle(self.screen, (240, 240, 255), self.left_eye_pos, self.eye_radius)
                pygame.draw.circle(self.screen, (240, 240, 255), self.right_eye_pos, self.eye_radius)
                
                # Handle blinking
                if self.is_blinking:
                    blink_progress = min(1.0, (time.time() - self.blink_start_time) / self.blink_duration)
                    # Determine how closed the eye is (0 = open, 1 = closed, then back to 0)
                    if blink_progress < 0.5:
                        closure = blink_progress * 2  # 0 to 1 (closing)
                    else:
                        closure = (1 - blink_progress) * 2  # 1 to 0 (opening)
                    
                    # Draw upper and lower eyelids
                    lid_offset = int(self.eye_radius * closure)
                    
                    for eye_pos in [self.left_eye_pos, self.right_eye_pos]:
                        # Upper eyelid
                        upper_rect = pygame.Rect(
                            eye_pos[0] - self.eye_radius, 
                            eye_pos[1] - self.eye_radius, 
                            self.eye_radius * 2, 
                            lid_offset
                        )
                        
                        # Lower eyelid
                        lower_rect = pygame.Rect(
                            eye_pos[0] - self.eye_radius,
                            eye_pos[1] + self.eye_radius - lid_offset,
                            self.eye_radius * 2,
                            lid_offset
                        )
                        
                        # Draw eyelids in background color
                        pygame.draw.rect(self.screen, self.background_color, upper_rect)
                        pygame.draw.rect(self.screen, self.background_color, lower_rect)
                        
                        # Redraw eye outline to maintain shape
                        pygame.draw.circle(self.screen, self.outline_color, eye_pos, self.eye_radius, self.outline_width)
                
                # Only draw iris and pupil if eyes aren't fully closed
                if not self.is_blinking or (self.is_blinking and (blink_progress < 0.25 or blink_progress > 0.75)):
                    # Add realistic iris colors
                    iris_color = (70, 130, 180)  # Steel blue
                    iris_radius = self.pupil_radius * 2.0
                    
                    # Draw iris for each eye (behind pupil)
                    pygame.draw.circle(self.screen, iris_color, self.pupil_position['left'], iris_radius)
                    pygame.draw.circle(self.screen, iris_color, self.pupil_position['right'], iris_radius)
                    
                    # Draw pupils
                    pygame.draw.circle(self.screen, self.pupil_color, self.pupil_position['left'], self.pupil_radius)
                    pygame.draw.circle(self.screen, self.pupil_color, self.pupil_position['right'], self.pupil_radius)
                    
                    # Add catch lights (reflections) to make eyes more realistic
                    catch_light_pos_left = (self.pupil_position['left'][0] - self.pupil_radius*0.3, 
                                        self.pupil_position['left'][1] - self.pupil_radius*0.3)
                    catch_light_pos_right = (self.pupil_position['right'][0] - self.pupil_radius*0.3, 
                                            self.pupil_position['right'][1] - self.pupil_radius*0.3)
                    
                    # Draw small white highlights (catch lights)
                    pygame.draw.circle(self.screen, (255, 255, 255), catch_light_pos_left, self.pupil_radius * 0.25)
                    pygame.draw.circle(self.screen, (255, 255, 255), catch_light_pos_right, self.pupil_radius * 0.25)
                
                # Draw eye outlines last
                pygame.draw.circle(self.screen, self.outline_color, self.left_eye_pos, self.eye_radius, self.outline_width)
                pygame.draw.circle(self.screen, self.outline_color, self.right_eye_pos, self.eye_radius, self.outline_width)
                
                # Draw debug information
                self.draw_debug_info()
                
                # Update display
                pygame.display.flip()
                
                # Cap at 60 FPS
                self.clock.tick(60)
                
                # Update ROS
                self.update_ros()
                
        except KeyboardInterrupt:
            pass
        finally:
            pygame.quit()
            self.get_logger().info('Eye tracking node shutting down')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        eye_tracker = EyeTrackingNode()
        eye_tracker.run()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main() 