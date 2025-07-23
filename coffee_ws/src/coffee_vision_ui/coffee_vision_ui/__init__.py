"""
Coffee Vision UI Package

This package provides a separated user interface for coffee vision camera
control and monitoring. It communicates with the camera_node via ROS topics
to enable remote operation and flexible deployment scenarios.

Main Components:
- camera_ui_node: Main UI application node
- ros_interface: ROS communication layer (CameraController, FrameReceiver)
- widgets: Reusable Qt UI components (CameraDisplay, ControlPanel)

The package enables headless camera operation while providing full UI
capabilities on local or remote machines via ROS transport.
"""

__version__ = '0.0.0'
__author__ = 'Coffee Vision Team'
