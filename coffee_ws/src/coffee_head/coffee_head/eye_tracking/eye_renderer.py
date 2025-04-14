#!/usr/bin/env python3

import pygame
import math
import time
import random
import threading
import os
from typing import Tuple, Dict, List, Optional, Any, Callable

from ..common.data_types import VelocityVector
import numpy as np

class EyeRenderer:
    """Class to render and animate stylized eyes"""
    
    def __init__(self):
        """Initialize the eye renderer"""
        # Window properties
        self.window_width = 800
        self.window_height = 480
        self.fullscreen = False
        self.screen = None
        self.clock = None
        
        # Eye properties
        self.eye_radius = 120
        self.pupil_radius = 40
        self.eye_spacing = 200
        
        # Eye positions
        self.left_eye_pos = (self.window_width // 2 - self.eye_spacing // 2, 
                            self.window_height // 2)
        self.right_eye_pos = (self.window_width // 2 + self.eye_spacing // 2, 
                             self.window_height // 2)
        
        # Pupil positions (normalized coordinates -1.0 to 1.0)
        self.left_pupil_target = (0.0, 0.0)
        self.right_pupil_target = (0.0, 0.0)
        self.left_pupil_current = (0.0, 0.0)
        self.right_pupil_current = (0.0, 0.0)
        
        # Animation properties
        self.blink_state = 0.0  # 0.0 = fully open, 1.0 = fully closed
        self.blink_speed = 0.15
        self.is_blinking = False
        self.blink_timer = 0.0
        self.time_between_blinks = 5.0  # seconds
        self.last_blink_time = time.time()
        
        # Camera parameters
        self.camera_aspect = 1.33  # Default 4:3 aspect ratio
        
        # Pupil smoothing
        self.smoothing_factor = 0.25  # Higher = smoother, lower = more responsive
        
        # Callbacks
        self.exit_callback = None
        
        # Debug mode
        self.debug_mode = False
        self.font = None
        
        # Colors
        self.bg_color = (0, 0, 0)  # Black
        self.eye_color = (255, 255, 255)  # White
        self.pupil_color = (0, 0, 0)  # Black
        self.highlight_color = (255, 255, 255, 180)  # Semi-transparent white
        self.debug_color = (0, 255, 0)  # Green
        
        # State
        self.running = True
        self.initialized = False
    
    def initialize(self):
        """Initialize pygame and create the window"""
        if self.initialized:
            return
            
        # Initialize pygame
        pygame.init()
        pygame.display.set_caption("Coffee Buddy Eyes")
        
        # Hide mouse cursor
        pygame.mouse.set_visible(False)
        
        # Create the window
        if self.fullscreen:
            self.screen = pygame.display.set_mode(
                (self.window_width, self.window_height),
                pygame.FULLSCREEN
            )
        else:
            self.screen = pygame.display.set_mode(
                (self.window_width, self.window_height)
            )
        
        # Create clock for frame timing
        self.clock = pygame.time.Clock()
        
        # Initialize font for debug text
        self.font = pygame.font.Font(None, 24)
        
        # Update eye positions
        self._update_eye_positions()
        
        self.initialized = True
    
    def shutdown(self):
        """Clean up resources"""
        self.running = False
        
        if pygame.get_init():
            pygame.quit()
    
    def set_window_size(self, width: int, height: int):
        """Set the window size"""
        self.window_width = width
        self.window_height = height
        
        # Update eye positions based on new window size
        self._update_eye_positions()
        
        # Recreate window if already initialized
        if self.initialized:
            if self.fullscreen:
                self.screen = pygame.display.set_mode(
                    (self.window_width, self.window_height),
                    pygame.FULLSCREEN
                )
            else:
                self.screen = pygame.display.set_mode(
                    (self.window_width, self.window_height)
                )
    
    def set_fullscreen(self, fullscreen: bool):
        """Set fullscreen mode"""
        self.fullscreen = fullscreen
        
        # Update display mode if already initialized
        if self.initialized:
            if self.fullscreen:
                self.screen = pygame.display.set_mode(
                    (self.window_width, self.window_height),
                    pygame.FULLSCREEN
                )
            else:
                self.screen = pygame.display.set_mode(
                    (self.window_width, self.window_height)
                )
    
    def set_eye_properties(self, eye_radius: int, pupil_radius: int, eye_spacing: int):
        """Set eye appearance properties"""
        self.eye_radius = eye_radius
        self.pupil_radius = pupil_radius
        self.eye_spacing = eye_spacing
        
        # Update eye positions
        self._update_eye_positions()
    
    def set_camera_parameters(self, aspect_ratio: float):
        """Set camera parameters"""
        self.camera_aspect = aspect_ratio
    
    def set_debug(self, enable: bool):
        """Enable or disable debug visualization"""
        self.debug_mode = enable
    
    def set_exit_callback(self, callback: Callable[[], None]):
        """Set callback function to be called when the renderer exits"""
        self.exit_callback = callback
    
    def update_pupils(self, left_target: Tuple[float, float], right_target: Tuple[float, float]):
        """Update the target pupil positions"""
        # Clamp values to -1.0 to 1.0 range
        lx = max(-1.0, min(1.0, left_target[0]))
        ly = max(-1.0, min(1.0, left_target[1]))
        rx = max(-1.0, min(1.0, right_target[0]))
        ry = max(-1.0, min(1.0, right_target[1]))
        
        self.left_pupil_target = (lx, ly)
        self.right_pupil_target = (rx, ry)
    
    def handle_events(self):
        """Handle pygame events"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
                if self.exit_callback:
                    self.exit_callback()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                    if self.exit_callback:
                        self.exit_callback()
                elif event.key == pygame.K_d:
                    # Toggle debug mode
                    self.debug_mode = not self.debug_mode
                elif event.key == pygame.K_b:
                    # Force a blink
                    self._trigger_blink()
    
    def draw(self):
        """Draw the eyes"""
        if not self.initialized or not self.running:
            return
            
        # Clear screen
        self.screen.fill(self.bg_color)
        
        # Update animation states
        self._update_animations()
        
        # Draw the eyes
        self._draw_eyes()
        
        # Draw debug info if enabled
        if self.debug_mode:
            self._draw_debug()
        
        # Update display
        pygame.display.flip()
        
        # Cap the frame rate
        self.clock.tick(60)
    
    def _update_eye_positions(self):
        """Update eye positions based on window size and spacing"""
        self.left_eye_pos = (self.window_width // 2 - self.eye_spacing // 2, 
                            self.window_height // 2)
        self.right_eye_pos = (self.window_width // 2 + self.eye_spacing // 2, 
                             self.window_height // 2)
    
    def _update_animations(self):
        """Update animated properties like blinking and pupil movement"""
        # Update pupil positions with smoothing
        self._update_pupil_positions()
        
        # Handle blinking
        self._update_blink_state()
    
    def _update_pupil_positions(self):
        """Smoothly interpolate pupil positions toward target"""
        # Calculate new positions with smoothing
        lx = self.left_pupil_current[0] + (self.left_pupil_target[0] - self.left_pupil_current[0]) * self.smoothing_factor
        ly = self.left_pupil_current[1] + (self.left_pupil_target[1] - self.left_pupil_current[1]) * self.smoothing_factor
        rx = self.right_pupil_current[0] + (self.right_pupil_target[0] - self.right_pupil_current[0]) * self.smoothing_factor
        ry = self.right_pupil_current[1] + (self.right_pupil_target[1] - self.right_pupil_current[1]) * self.smoothing_factor
        
        self.left_pupil_current = (lx, ly)
        self.right_pupil_current = (rx, ry)
    
    def _update_blink_state(self):
        """Update blinking animation"""
        current_time = time.time()
        
        # Random blinking
        if not self.is_blinking and current_time - self.last_blink_time > self.time_between_blinks:
            self._trigger_blink()
            self.time_between_blinks = 3.0 + 5.0 * np.random.random()  # 3-8 seconds
        
        # Handle blink animation
        if self.is_blinking:
            if self.blink_timer < 0.5:  # First half: close eyes
                self.blink_state = self.blink_timer * 2.0
            else:  # Second half: open eyes
                self.blink_state = 1.0 - (self.blink_timer - 0.5) * 2.0
            
            self.blink_timer += self.blink_speed
            
            # End blink
            if self.blink_timer >= 1.0:
                self.is_blinking = False
                self.blink_state = 0.0
                self.last_blink_time = current_time
    
    def _trigger_blink(self):
        """Start a blink animation"""
        self.is_blinking = True
        self.blink_timer = 0.0
    
    def _draw_eyes(self):
        """Draw the eyes and pupils"""
        # Calculate actual pupil positions
        left_pupil_x = self.left_eye_pos[0] + int(self.left_pupil_current[0] * (self.eye_radius - self.pupil_radius) * 0.8)
        left_pupil_y = self.left_eye_pos[1] + int(self.left_pupil_current[1] * (self.eye_radius - self.pupil_radius) * 0.8)
        
        right_pupil_x = self.right_eye_pos[0] + int(self.right_pupil_current[0] * (self.eye_radius - self.pupil_radius) * 0.8)
        right_pupil_y = self.right_eye_pos[1] + int(self.right_pupil_current[1] * (self.eye_radius - self.pupil_radius) * 0.8)
        
        # Calculate eye shapes based on blink state
        eye_height = int(self.eye_radius * 2 * (1.0 - self.blink_state))
        
        # Draw eye whites (ellipses)
        pygame.draw.ellipse(
            self.screen, 
            self.eye_color, 
            pygame.Rect(
                self.left_eye_pos[0] - self.eye_radius,
                self.left_eye_pos[1] - eye_height // 2,
                self.eye_radius * 2,
                eye_height
            )
        )
        
        pygame.draw.ellipse(
            self.screen, 
            self.eye_color, 
            pygame.Rect(
                self.right_eye_pos[0] - self.eye_radius,
                self.right_eye_pos[1] - eye_height // 2,
                self.eye_radius * 2,
                eye_height
            )
        )
        
        # Only draw pupils if eyes are open enough
        if eye_height > self.pupil_radius:
            # Draw pupils (circles)
            pygame.draw.circle(
                self.screen,
                self.pupil_color,
                (left_pupil_x, left_pupil_y),
                self.pupil_radius
            )
            
            pygame.draw.circle(
                self.screen,
                self.pupil_color,
                (right_pupil_x, right_pupil_y),
                self.pupil_radius
            )
            
            # Add highlights
            highlight_size = self.pupil_radius // 3
            highlight_offset_x = self.pupil_radius // 4
            highlight_offset_y = -self.pupil_radius // 4
            
            # Create a surface for the highlight with alpha
            highlight_surface = pygame.Surface((highlight_size*2, highlight_size*2), pygame.SRCALPHA)
            pygame.draw.circle(
                highlight_surface,
                self.highlight_color,
                (highlight_size, highlight_size),
                highlight_size
            )
            
            # Blit the highlight onto the main surface
            self.screen.blit(
                highlight_surface,
                (left_pupil_x + highlight_offset_x - highlight_size, 
                 left_pupil_y + highlight_offset_y - highlight_size)
            )
            self.screen.blit(
                highlight_surface,
                (right_pupil_x + highlight_offset_x - highlight_size, 
                 right_pupil_y + highlight_offset_y - highlight_size)
            )
    
    def _draw_debug(self):
        """Draw debug information"""
        if not self.font:
            return
            
        # Debug heading
        debug_text = self.font.render("DEBUG MODE", True, self.debug_color)
        self.screen.blit(debug_text, (10, 10))
        
        # Eye information
        left_info = f"Left: target ({self.left_pupil_target[0]:.2f}, {self.left_pupil_target[1]:.2f}), current ({self.left_pupil_current[0]:.2f}, {self.left_pupil_current[1]:.2f})"
        right_info = f"Right: target ({self.right_pupil_target[0]:.2f}, {self.right_pupil_target[1]:.2f}), current ({self.right_pupil_current[0]:.2f}, {self.right_pupil_current[1]:.2f})"
        
        left_text = self.font.render(left_info, True, self.debug_color)
        right_text = self.font.render(right_info, True, self.debug_color)
        
        self.screen.blit(left_text, (10, 40))
        self.screen.blit(right_text, (10, 70))
        
        # Blink state
        blink_info = f"Blink: {'Active' if self.is_blinking else 'Inactive'}, State: {self.blink_state:.2f}, Next: {max(0, self.time_between_blinks - (time.time() - self.last_blink_time)):.1f}s"
        blink_text = self.font.render(blink_info, True, self.debug_color)
        self.screen.blit(blink_text, (10, 100))
        
        # Draw target crosshairs
        for eye_pos, pupil_target in [
            (self.left_eye_pos, self.left_pupil_target),
            (self.right_eye_pos, self.right_pupil_target)
        ]:
            target_x = eye_pos[0] + int(pupil_target[0] * (self.eye_radius - self.pupil_radius) * 0.8)
            target_y = eye_pos[1] + int(pupil_target[1] * (self.eye_radius - self.pupil_radius) * 0.8)
            
            # Draw crosshair
            pygame.draw.line(self.screen, self.debug_color, 
                            (target_x - 5, target_y), 
                            (target_x + 5, target_y), 1)
            pygame.draw.line(self.screen, self.debug_color, 
                            (target_x, target_y - 5), 
                            (target_x, target_y + 5), 1)
        
        # Draw eye boundaries
        for eye_pos in [self.left_eye_pos, self.right_eye_pos]:
            pygame.draw.circle(self.screen, self.debug_color, eye_pos, self.eye_radius, 1)
            
            # Draw movement limit boundary
            limit_radius = int((self.eye_radius - self.pupil_radius) * 0.8)
            pygame.draw.circle(self.screen, self.debug_color, eye_pos, limit_radius, 1)
        
        # FPS counter
        fps_text = self.font.render(f"FPS: {int(self.clock.get_fps())}", True, self.debug_color)
        self.screen.blit(fps_text, (self.window_width - 100, 10))

# Test function
def main():
    renderer = EyeRenderer()
    renderer.set_debug(True)
    
    def exit_callback():
        print("Exit requested")
    
    renderer.set_exit_callback(exit_callback)
    renderer.initialize()
    
    try:
        running = True
        while running:
            renderer.handle_events()
            
            # Circular motion for testing
            t = time.time() * 0.5
            x = math.sin(t) * 0.7
            y = math.cos(t) * 0.7
            
            renderer.update_pupils((x, y), (x, y))
            renderer.draw()
            
            # Check if still running
            if not renderer.running:
                running = False
            
            time.sleep(0.01)
    finally:
        renderer.shutdown()

if __name__ == "__main__":
    main() 