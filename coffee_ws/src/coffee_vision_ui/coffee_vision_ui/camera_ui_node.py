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
from python_qt_binding.QtWidgets import QApplication, QMainWindow, QHBoxLayout, QVBoxLayout, QWidget
from python_qt_binding.QtCore import QTimer
from std_msgs.msg import String

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
        
        # Initialize ROS communication components
        self.camera_controller = CameraController(node)
        self.frame_receiver = FrameReceiver(node, '/coffee_bot/camera/image_raw')
        
        # Initialize UI components
        self.camera_display = CameraDisplay()
        self.control_panel = ControlPanel()
        
        # Set up state query publisher
        self.state_query_pub = node.create_publisher(String, '/coffee_bot/camera/query/state', 10)
        
        # Set up connection timeout monitoring
        self.connection_timer = QTimer()
        self.connection_timer.timeout.connect(self._check_connection)
        self.connection_timer.start(500)  # Check every 500ms
        
        self.init_ui()
        self.setup_connections()
        
        # Query camera_node state instead of sending commands
        QTimer.singleShot(1000, self._query_camera_node_state)
    
    def init_ui(self):
        """Initialize the user interface layout and components."""
        self.setWindowTitle('Coffee Camera UI (Separated)')
        self.setGeometry(100, 100, 1200, 800)
        
        # Create main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        
        # Left side: Camera display (takes most space)
        main_layout.addWidget(self.camera_display, 3)
        
        # Right side: Control panel
        main_layout.addWidget(self.control_panel, 1)
        
        self.node.get_logger().info('UI layout initialized with modular components')
    
    def setup_connections(self):
        """Set up signal/slot connections between components."""
        
        # Frame receiver to camera display connections
        self.frame_receiver.frame_ready.connect(self.camera_display.update_frame)
        self.frame_receiver.performance_update.connect(self.camera_display.update_performance)
        self.frame_receiver.connection_lost.connect(self.camera_display.on_connection_lost)
        self.frame_receiver.connection_restored.connect(self.camera_display.on_connection_restored)
        
        # Camera controller to control panel connections
        self.camera_controller.cameras_updated.connect(self.control_panel.update_camera_list)
        self.camera_controller.camera_status_updated.connect(self.control_panel.update_status)
        self.camera_controller.diagnostics_ready.connect(self.control_panel.show_diagnostics)
        self.camera_controller.state_received.connect(self.control_panel.sync_to_camera_state)
        
        # Control panel to camera controller connections
        self.control_panel.camera_selected.connect(self.camera_controller.select_camera)
        self.control_panel.quality_changed.connect(self.camera_controller.set_quality)
        self.control_panel.face_detection_changed.connect(self.camera_controller.toggle_face_detection)
        self.control_panel.refresh_cameras_requested.connect(self.camera_controller.refresh_cameras)
        self.control_panel.diagnostics_requested.connect(self.camera_controller.request_diagnostics)
        
        self.node.get_logger().info('Signal connections established between all components')
    
    def _query_camera_node_state(self):
        """Query camera_node for its current state instead of sending commands"""
        self.node.get_logger().info('Querying camera_node for current state')
        
        # Send state query
        query_msg = String()
        query_msg.data = "get_state"
        self.state_query_pub.publish(query_msg)
        
        # Set up one-time listener for state response
        QTimer.singleShot(2000, self._setup_state_listener)
    
    def _setup_state_listener(self):
        """Set up listener for state response from camera_node"""
        # The state response comes through the same topic as camera list updates
        # We'll handle it in the existing camera_controller callback
        self.node.get_logger().info('Listening for camera_node state response')
    
    def _check_connection(self):
        """Periodically check frame receiver connection status."""
        self.frame_receiver.check_connection_timeout()
    
    def closeEvent(self, event):
        """Handle window close event."""
        self.connection_timer.stop()
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