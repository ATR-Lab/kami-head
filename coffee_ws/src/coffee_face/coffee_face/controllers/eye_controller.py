from typing import List, Tuple, Optional
import time
from ..bezier.bezier_viz import AnimatedEyes, EyeConfig

class EyeController:
    def __init__(self, animated_eyes: AnimatedEyes):
        self.eyes = animated_eyes
        self.movement_queue = []
        self.current_position = (0.0, 0.0)  # Start at center
        self.target_position = None
        self.move_duration = 0.2  # Time to move between positions in seconds (reduced from 1.0)
        self.move_progress = 0.0
        
    def queue_movement(self, positions: List[Tuple[float, float]]):
        """Add a sequence of positions to the movement queue"""
        self.movement_queue.extend(positions)
        if self.target_position is None:
            self.start_next_movement()
    
    def start_next_movement(self):
        """Start movement to next position in queue"""
        if self.movement_queue:
            self.target_position = self.movement_queue[0]
            # Get current position for interpolation
            self.current_position = (
                self.eyes.config_sliders['position_x'],
                self.eyes.config_sliders['position_y']
            )
            self.move_progress = 0.0
    
    def update(self):
        """Update eye positions with smooth movement"""
        if self.target_position is None:
            return
            
        # Update progress
        self.move_progress += 1/60  # Assuming 60 FPS
        progress = min(1.0, self.move_progress / self.move_duration)
        
        # Use cubic easing for smooth movement
        t = self.ease_out_cubic(progress)
        
        # Interpolate between current and target position
        current_x = self.current_position[0] + (self.target_position[0] - self.current_position[0]) * t
        current_y = self.current_position[1] + (self.target_position[1] - self.current_position[1]) * t
        
        # Update eye positions
        self.eyes.set_eye_positions((current_x, current_y))
        
        # Debug output for tracking movement
        if hasattr(self.eyes, 'node') and self.eyes.node:
            self.eyes.node.get_logger().debug(
                f"Queue length: {len(self.movement_queue)}, "
                f"Target: ({self.target_position[0]:.2f}, {self.target_position[1]:.2f}), "
                f"Current: ({current_x:.2f}, {current_y:.2f})"
            )
        
        # Check if movement is complete
        if progress >= 1.0:
            self.movement_queue.pop(0)  # Remove completed movement
            self.target_position = None
            if self.movement_queue:  # Start next movement if queue not empty
                self.start_next_movement()
    
    @staticmethod
    def ease_out_cubic(t: float) -> float:
        """Cubic easing function for smooth movement"""
        return 1 - (1 - t) ** 3
    
    def follow_square_path(self):
        """Queue a square movement pattern"""
        positions = [
            (3.0, -3.0),  # Top right
            (3.0, 3.0),   # Bottom right
            (-3.0, 3.0),  # Bottom left
            (-3.0, -3.0), # Top left
            (0.0, 0.0),   # Back to center
        ]
        self.queue_movement(positions)
    
    def go_to_pos(self, pos: Tuple[float, float]):
        """Move eyes to a specific position"""
        # Clear the queue to immediately respond to new position
        self.movement_queue = []
        self.target_position = None  # Reset current target
        self.queue_movement([pos])