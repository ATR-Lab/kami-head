"""
UI Widgets Module for Coffee Vision UI

This module contains reusable Qt-based UI components for camera visualization
and control. The widgets are designed to be modular and reusable across
different camera UI applications.

Components:
- CameraDisplay: Video display widget with performance metrics
- ControlPanel: Camera control interface (selection, quality, face detection)

The widgets communicate with the main UI via Qt signals and slots.
"""

from .camera_display import CameraDisplay
from .control_panel import ControlPanel

__all__ = ['CameraDisplay', 'ControlPanel'] 