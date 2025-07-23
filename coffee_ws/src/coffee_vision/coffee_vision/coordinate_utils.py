#!/usr/bin/env python3

"""
Coordinate transformation utilities for coffee vision system.

This module provides coordinate transformation functions that convert between
different coordinate systems used in the coffee robot vision pipeline.
"""

from typing import Tuple, Optional, Any


def transform_camera_to_eye_coords(
    camera_x: float,
    camera_y: float,
    frame_width: int,
    frame_height: int,
    eye_range: float = 1.0,
    sensitivity: float = 1.5,
    invert_x: bool = False,
    invert_y: bool = False,
    logger: Optional[Any] = None
) -> Tuple[float, float]:
    """
    Transform camera coordinates to eye controller coordinates.
    
    Converts pixel coordinates from camera frame to normalized eye movement
    coordinates suitable for robot eye control.
    
    Args:
        camera_x: X coordinate in camera frame (pixels)
        camera_y: Y coordinate in camera frame (pixels)
        frame_width: Width of camera frame in pixels
        frame_height: Height of camera frame in pixels
        eye_range: Maximum range for eye movement (default: 1.0)
        sensitivity: Sensitivity multiplier for eye movement (default: 1.5)
        invert_x: Whether to invert X axis (default: False)
        invert_y: Whether to invert Y axis (default: False)
        logger: Optional logger for debug output
    
    Returns:
        Tuple of (eye_x, eye_y) coordinates in range [-eye_range, eye_range]
    
    Notes:
        - Camera coordinates: (0,0) at top-left, positive X right, positive Y down
        - Eye coordinates: (0,0) at center, positive X right, positive Y up (typically)
        - When face is on right side of frame, eyes should look right (positive X)
        - Default invert_x=False ensures this correct behavior
    """
    # Validate inputs
    if frame_width <= 0 or frame_height <= 0:
        raise ValueError(f"Invalid frame dimensions: {frame_width}x{frame_height}")
    
    if eye_range <= 0:
        raise ValueError(f"Eye range must be positive, got: {eye_range}")
    
    # Normalize camera coordinates to -1.0 to 1.0 range
    # Center the coordinates around the frame center
    norm_x = (camera_x - frame_width/2) / (frame_width/2)
    norm_y = (camera_y - frame_height/2) / (frame_height/2)
    
    # Apply sensitivity multiplier for more responsive eye movement
    norm_x *= sensitivity
    norm_y *= sensitivity
    
    # Apply coordinate inversions if configured
    # Note: By default we want norm_x positive when face is on right side
    # So default should have invert_x=False for natural eye movement
    if invert_x:
        norm_x = -norm_x
    if invert_y:
        norm_y = -norm_y
    
    # Scale to eye controller range
    eye_x = norm_x * eye_range
    eye_y = norm_y * eye_range
    
    # Clamp values to valid range to prevent out-of-bounds movement
    eye_x = max(-eye_range, min(eye_range, eye_x))
    eye_y = max(-eye_range, min(eye_range, eye_y))
    
    # Optional debug logging
    if logger is not None:
        logger.debug(f'Camera coords: ({camera_x:.1f}, {camera_y:.1f}) -> '
                    f'Eye coords: ({eye_x:.2f}, {eye_y:.2f})')
    
    return (eye_x, eye_y) 