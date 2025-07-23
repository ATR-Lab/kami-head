#!/usr/bin/env python3
"""
Camera UI Node for Coffee Vision System

This node provides a separated, full-featured camera control interface that
communicates with the camera_node via ROS topics. It enables:

- Complete camera control (selection, quality, face detection)
- Real-time video display with performance metrics
- Remote operation and monitoring capabilities
- Connection status monitoring and error handling

The UI publishes control commands to /coffee_bot/camera/cmd/* topics
and subscribes to status updates and video streams from camera_node.

Architecture:
- Main UI window with video display and controls
- ROS communication layer for camera control and status
- Modular widget design for reusability
- Performance monitoring and diagnostics

This enables headless camera operation while providing full UI capabilities
on local or remote machines.
"""

import sys
import rclpy
from rclpy.node import Node
from python_qt_binding.QtWidgets import QApplication, QMainWindow
from python_qt_binding.QtCore import QTimer

# Import our modular components
from .ros_interface import CameraController, FrameReceiver
from .widgets import CameraDisplay, ControlPanel


class CameraUINode(Node):
    """
    ROS2 node providing separated camera UI that communicates via ROS transport.
    
    This node creates a complete camera control interface that operates
    independently from the camera processing node, enabling flexible
    deployment scenarios.
    """
    
    def __init__(self):
        super().__init__('camera_ui_node')
        self.get_logger().info('Camera UI Node starting...')
        self.get_logger().info('Connecting to camera_node via /coffee_bot/ topics')
        
        # Create Qt application
        self.app = QApplication(sys.argv)
        
        # Create and show the UI window
        self.ui_window = CameraUIWindow(self)
        self.ui_window.show()
        
        self.get_logger().info('Camera UI window opened')


class CameraUIWindow(QMainWindow):
    """
    Main UI window for camera control and display.
    
    Combines modular widgets for video display and camera controls,
    coordinated through ROS communication layer.
    """
    
    def __init__(self, node):
        super().__init__()
        self.node = node
        
        # TODO: Initialize ROS communication components
        # self.camera_controller = CameraController(node)
        # self.frame_receiver = FrameReceiver(node)
        
        # TODO: Initialize UI components
        # self.camera_display = CameraDisplay()
        # self.control_panel = ControlPanel()
        
        self.init_ui()
        self.setup_connections()
    
    def init_ui(self):
        """Initialize the user interface layout and components."""
        # TODO: Set up main window layout
        # TODO: Add camera display widget
        # TODO: Add control panel widget
        # TODO: Add status bar and performance metrics
        
        self.setWindowTitle('Coffee Camera UI (Separated)')
        self.setGeometry(100, 100, 1000, 700)
        
        # Placeholder implementation
        self.node.get_logger().info('UI layout initialization - TODO: Implement')
    
    def setup_connections(self):
        """Set up signal/slot connections between components."""
        # TODO: Connect camera controller signals to UI updates
        # TODO: Connect UI control signals to camera controller
        # TODO: Connect frame receiver to display widget
        
        self.node.get_logger().info('Signal connections setup - TODO: Implement')
    
    def closeEvent(self, event):
        """Handle window close event."""
        self.node.get_logger().info('Camera UI window closed')
        event.accept()


def main(args=None):
    """
    Main entry point for separated camera UI.
    
    Creates a full-featured camera control UI that communicates with camera_node
    via ROS topics under the /coffee_bot/ namespace.
    """
    try:
        # Initialize ROS2
        rclpy.init(args=args)
        
        # Create the camera UI node
        node = CameraUINode()
        
        # Set up ROS spinning timer for Qt integration
        def spin_ros():
            rclpy.spin_once(node, timeout_sec=0.001)
        
        # Qt timer for ROS message handling
        ros_timer = QTimer()
        ros_timer.timeout.connect(spin_ros)
        ros_timer.start(1)  # 1ms interval for responsive ROS handling
        
        try:
            # Run Qt event loop
            exit_code = node.app.exec_()
            node.get_logger().info('Qt application closed')
            
        except KeyboardInterrupt:
            node.get_logger().info('Interrupted by user')
            
    except Exception as e:
        print(f"Error in camera UI: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main() 