"""
Camera Controller for ROS Communication

This module handles all camera control commands sent to the camera_node
via ROS topics. It provides a clean interface for UI components to:

- Select camera devices
- Control quality settings
- Toggle face detection
- Request diagnostic information
- Monitor camera status

The controller publishes commands and subscribes to status updates,
abstracting ROS communication details from UI components.
"""

import json
from python_qt_binding.QtCore import QObject, pyqtSignal
from std_msgs.msg import String, Bool, Int32


class CameraController(QObject):
    """
    Handles ROS communication for camera control commands and status updates.
    
    This class provides a Qt-based interface for camera control, using
    signals to communicate with UI components while handling ROS
    communication in the background.
    
    Signals:
        cameras_updated: Emitted when available cameras list is updated
        camera_status_updated: Emitted when camera status changes
        diagnostics_ready: Emitted when diagnostic information is available
    """
    
    # Qt signals for UI updates
    cameras_updated = pyqtSignal(list)      # Available cameras: [(index, name), ...]
    camera_status_updated = pyqtSignal(str) # Current camera status
    diagnostics_ready = pyqtSignal(str)     # Diagnostic information
    state_received = pyqtSignal(dict)       # Full camera state for synchronization
    
    def __init__(self, node):
        """
        Initialize camera controller with ROS node.
        
        Args:
            node: ROS2 node instance for creating publishers/subscribers
        """
        super().__init__()
        self.node = node
        
        # Initialize ROS communication
        self._setup_publishers()
        self._setup_subscribers()
        
        self.node.get_logger().info("Camera controller initialized with /coffee_bot/ namespace")
    
    def _setup_publishers(self):
        """Set up ROS publishers for sending commands to camera_node."""
        # Command publishers
        self.camera_select_pub = self.node.create_publisher(
            Int32, '/coffee_bot/camera/cmd/select', 10)
        self.quality_control_pub = self.node.create_publisher(
            Bool, '/coffee_bot/camera/cmd/quality', 10)
        self.face_detection_control_pub = self.node.create_publisher(
            Bool, '/coffee_bot/camera/cmd/face_detection', 10)
        self.camera_refresh_pub = self.node.create_publisher(
            String, '/coffee_bot/camera/cmd/refresh', 10)
        self.diagnostics_request_pub = self.node.create_publisher(
            String, '/coffee_bot/camera/cmd/diagnostics', 10)
    
    def _setup_subscribers(self):
        """Set up ROS subscribers for receiving status from camera_node."""
        # Status subscribers
        self.camera_status_sub = self.node.create_subscription(
            String, '/coffee_bot/camera/status/info', self._camera_status_callback, 10)
        self.available_cameras_sub = self.node.create_subscription(
            String, '/coffee_bot/camera/status/available', self._available_cameras_callback, 10)
        self.diagnostics_sub = self.node.create_subscription(
            String, '/coffee_bot/camera/status/diagnostics', self._diagnostics_callback, 10)
    
    def select_camera(self, camera_index):
        """
        Request camera node to switch to specified camera.
        
        Args:
            camera_index (int): Index of camera to select
        """
        msg = Int32()
        msg.data = camera_index
        self.camera_select_pub.publish(msg)
        self.node.get_logger().info(f"Requested camera switch to index {camera_index}")
    
    def set_quality(self, high_quality):
        """
        Request camera node to change quality settings.
        
        Args:
            high_quality (bool): True for high quality (1080p), False for standard (480p)
        """
        msg = Bool()
        msg.data = high_quality
        self.quality_control_pub.publish(msg)
        quality_str = "high" if high_quality else "standard"
        self.node.get_logger().info(f"Requested quality change: {quality_str}")
    
    def toggle_face_detection(self, enabled):
        """
        Request camera node to enable/disable face detection.
        
        Args:
            enabled (bool): True to enable face detection, False to disable
        """
        msg = Bool()
        msg.data = enabled
        self.face_detection_control_pub.publish(msg)
        status_str = "enabled" if enabled else "disabled"
        self.node.get_logger().info(f"Requested face detection: {status_str}")
    
    def refresh_cameras(self):
        """Request camera node to scan for available cameras."""
        msg = String()
        msg.data = "refresh"
        self.camera_refresh_pub.publish(msg)
        self.node.get_logger().info("Requested camera refresh")
    
    def request_diagnostics(self):
        """
        Request camera node to provide diagnostic information.
        
        Sends a diagnostics command to camera_node, which will respond
        by publishing detailed diagnostic information.
        """
        msg = String()
        msg.data = "get_diagnostics"
        self.diagnostics_request_pub.publish(msg)
        self.node.get_logger().info("Requested camera diagnostics from camera_node")
    
    def _camera_status_callback(self, msg):
        """
        Handle camera status updates from camera_node.
        
        Args:
            msg: String message containing camera status
        """
        self.camera_status_updated.emit(msg.data)
    
    def _available_cameras_callback(self, msg):
        """
        Handle available cameras list or state response from camera_node.
        
        Args:
            msg: String message containing JSON list of cameras or full state
        """
        try:
            data = json.loads(msg.data)
            
            # Check if this is a full state response (has camera_index, high_quality, etc.)
            if isinstance(data, dict) and 'camera_index' in data:
                # This is a state response - emit for state sync
                self.node.get_logger().info("Received camera_node state for synchronization")
                self.state_received.emit(data)
            else:
                # This is a regular camera list
                # Expected format: [{"index": 0, "name": "Camera 0"}, ...]
                cameras_list = [(cam["index"], cam["name"]) for cam in data]
                self.cameras_updated.emit(cameras_list)
        except json.JSONDecodeError as e:
            self.node.get_logger().error(f"Error parsing cameras data: {e}")
        except (KeyError, TypeError) as e:
            self.node.get_logger().error(f"Invalid camera data format: {e}")
    
    def _diagnostics_callback(self, msg):
        """
        Handle diagnostic information from camera_node.
        
        Args:
            msg: String message containing diagnostic information
        """
        self.diagnostics_ready.emit(msg.data) 