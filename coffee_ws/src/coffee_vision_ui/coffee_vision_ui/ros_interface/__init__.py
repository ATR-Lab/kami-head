"""
ROS Interface Module for Coffee Vision UI

This module handles all ROS communication between the UI and the camera_node.
It provides clean abstractions for:
- Camera control commands (selection, quality, face detection)
- Status updates from camera_node
- Video frame reception and processing
- Diagnostic information exchange

The module separates ROS communication concerns from UI logic.
"""

from .camera_controller import CameraController
from .frame_receiver import FrameReceiver

__all__ = ['CameraController', 'FrameReceiver'] 