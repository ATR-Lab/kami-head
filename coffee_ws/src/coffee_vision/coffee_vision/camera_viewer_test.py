#!/usr/bin/env python3

import sys
import cv2
import rclpy
import time
import numpy as np
import os
import subprocess
from rclpy.node import Node
from python_qt_binding.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QLabel, 
                                       QPushButton, QComboBox, QHBoxLayout, QCheckBox, QMessageBox)
from python_qt_binding.QtGui import QImage, QPixmap
from python_qt_binding.QtCore import Qt, QTimer, pyqtSignal, QObject
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Int32
from cv_bridge import CvBridge


class CameraController(QObject):
    """Handles ROS communication for camera control commands and status updates"""
    
    # Signals for UI updates
    cameras_updated = pyqtSignal(list)      # Available cameras list: [(index, name), ...]
    camera_status_updated = pyqtSignal(str) # Current camera status
    diagnostics_ready = pyqtSignal(str)     # Diagnostic information
    
    def __init__(self, node):
        super().__init__()
        self.node = node
        
        # ROS Publishers for sending commands to camera_node
        self.camera_select_pub = node.create_publisher(
            Int32, '/coffee_bot/camera/cmd/select', 10)
        self.quality_control_pub = node.create_publisher(
            Bool, '/coffee_bot/camera/cmd/quality', 10)
        self.face_detection_control_pub = node.create_publisher(
            Bool, '/coffee_bot/camera/cmd/face_detection', 10)
        self.camera_refresh_pub = node.create_publisher(
            String, '/coffee_bot/camera/cmd/refresh', 10)
        
        # ROS Subscribers for receiving status from camera_node
        self.camera_status_sub = node.create_subscription(
            String, '/coffee_bot/camera/status/info', self.camera_status_callback, 10)
        self.available_cameras_sub = node.create_subscription(
            String, '/coffee_bot/camera/status/available', self.available_cameras_callback, 10)
        self.diagnostics_sub = node.create_subscription(
            String, '/coffee_bot/camera/status/diagnostics', self.diagnostics_callback, 10)
        
        self.node.get_logger().info("Camera controller initialized with /coffee_bot/ namespace")
    
    def select_camera(self, camera_index):
        """Request camera node to switch to specified camera"""
        msg = Int32()
        msg.data = camera_index
        self.camera_select_pub.publish(msg)
        self.node.get_logger().info(f"Requested camera switch to index {camera_index}")
    
    def set_quality(self, high_quality):
        """Request camera node to change quality settings"""
        msg = Bool()
        msg.data = high_quality
        self.quality_control_pub.publish(msg)
        self.node.get_logger().info(f"Requested quality change: {'high' if high_quality else 'standard'}")
    
    def toggle_face_detection(self, enabled):
        """Request camera node to enable/disable face detection"""
        msg = Bool()
        msg.data = enabled
        self.face_detection_control_pub.publish(msg)
        self.node.get_logger().info(f"Requested face detection: {'enabled' if enabled else 'disabled'}")
    
    def refresh_cameras(self):
        """Request camera node to scan for available cameras"""
        msg = String()
        msg.data = "refresh"
        self.camera_refresh_pub.publish(msg)
        self.node.get_logger().info("Requested camera refresh")
    
    def request_diagnostics(self):
        """Request camera node to provide diagnostic information"""
        # For now, we'll generate diagnostics locally since we don't have the service implemented yet
        # TODO: Replace with actual service call to camera_node
        self.generate_local_diagnostics()
    
    def generate_local_diagnostics(self):
        """Generate diagnostic information locally (placeholder for service call)"""
        info = "Camera Diagnostics:\n\n"
        
        # Check for video devices
        video_devices = []
        if os.path.exists('/dev'):
            for device in os.listdir('/dev'):
                if device.startswith('video'):
                    full_path = f"/dev/{device}"
                    access = os.access(full_path, os.R_OK)
                    video_devices.append(f"{full_path} (Readable: {access})")
        
        if video_devices:
            info += "Video Devices:\n" + "\n".join(video_devices) + "\n\n"
        else:
            info += "No video devices found!\n\n"
        
        # OpenCV version
        info += f"OpenCV Version: {cv2.__version__}\n"
        
        # Check which OpenCV backends are available
        info += "Available OpenCV Backends:\n"
        backends = [
            (cv2.CAP_V4L2, "Linux V4L2"),
            (cv2.CAP_GSTREAMER, "GStreamer"),
            (cv2.CAP_FFMPEG, "FFMPEG"),
            (cv2.CAP_IMAGES, "Images"),
            (cv2.CAP_DSHOW, "DirectShow (Windows)"),
            (cv2.CAP_ANY, "Auto-detect")
        ]
        
        for backend_id, name in backends:
            info += f"- {name}\n"
        
        info += "\nNote: This is local diagnostic info. Full camera diagnostics require camera_node integration."
        
        self.diagnostics_ready.emit(info)
    
    def camera_status_callback(self, msg):
        """Handle camera status updates from camera_node"""
        self.camera_status_updated.emit(msg.data)
    
    def available_cameras_callback(self, msg):
        """Handle available cameras list from camera_node"""
        try:
            import json
            cameras_data = json.loads(msg.data)
            # Expected format: [{"index": 0, "name": "Camera 0"}, ...]
            cameras_list = [(cam["index"], cam["name"]) for cam in cameras_data]
            self.cameras_updated.emit(cameras_list)
        except Exception as e:
            self.node.get_logger().error(f"Error parsing cameras data: {e}")
    
    def diagnostics_callback(self, msg):
        """Handle diagnostic information from camera_node"""
        self.diagnostics_ready.emit(msg.data)


class FrameReceiver(QObject):
    """Receives ROS Image messages and emits Qt signals for display"""
    frame_ready = pyqtSignal(np.ndarray, float)  # frame, latency
    
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.bridge = CvBridge()
        
        # Performance tracking
        self.frame_count = 0
        self.start_time = time.time()
        self.last_frame_time = 0
        self.latency_history = []
        self.max_latency_samples = 100
        
        # Subscribe to camera frame topic with coffee_bot namespace
        self.subscription = self.node.create_subscription(
            Image,
            # '/coffee_bot/camera/image_raw',
            'camera_frame',
            self.frame_callback,
            10  # QoS queue size
        )
        
        self.node.get_logger().info("Subscribed to '/coffee_bot/camera/image_raw' topic")
    
    def frame_callback(self, msg):
        """Process incoming ROS Image messages"""
        try:
            # Calculate latency (approximate - based on message timestamp vs current time)
            current_time = time.time()
            msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
            # If timestamp is 0, use reception time as baseline
            if msg_time == 0:
                latency = 0.0
            else:
                latency = current_time - msg_time
            
            # Track latency statistics
            self.latency_history.append(latency)
            if len(self.latency_history) > self.max_latency_samples:
                self.latency_history.pop(0)
            
            # Convert ROS Image to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Track frame rate
            self.frame_count += 1
            
            # Emit signal for Qt display
            self.frame_ready.emit(frame, latency)
            
            # Log performance stats periodically
            if self.frame_count % 30 == 0:
                elapsed = current_time - self.start_time
                fps = self.frame_count / elapsed if elapsed > 0 else 0
                avg_latency = sum(self.latency_history) / len(self.latency_history) if self.latency_history else 0
                max_latency = max(self.latency_history) if self.latency_history else 0
                
                self.node.get_logger().info(
                    f"ROS Transport Stats - FPS: {fps:.1f}, "
                    f"Avg Latency: {avg_latency*1000:.1f}ms, "
                    f"Max Latency: {max_latency*1000:.1f}ms"
                )
        
        except Exception as e:
            self.node.get_logger().error(f"Error processing frame: {e}")


class CameraViewerTest(QMainWindow):
    """Full-featured camera UI that communicates with camera_node via ROS"""
    
    def __init__(self, node):
        super().__init__()
        self.node = node
        
        # Initialize ROS communication components
        self.frame_receiver = FrameReceiver(node)
        self.frame_receiver.frame_ready.connect(self.display_frame)
        
        self.camera_controller = CameraController(node)
        self.camera_controller.cameras_updated.connect(self.update_camera_list)
        self.camera_controller.camera_status_updated.connect(self.update_camera_status)
        self.camera_controller.diagnostics_ready.connect(self.show_diagnostics)
        
        # UI state variables
        self.available_cameras = []
        self.current_camera_index = 0
        self.high_quality = False
        self.face_detection_enabled = True
        
        # Performance tracking for display
        self.display_count = 0
        self.display_start_time = time.time()
        self.last_display_time = time.time()
        self.last_frame_received_time = 0
        
        # Frame timeout detection
        self.frame_timeout_seconds = 2.0  # Consider connection lost after 2 seconds
        self.is_receiving_frames = False
        
        # Timer to check for frame timeout
        self.timeout_timer = QTimer()
        self.timeout_timer.timeout.connect(self.check_frame_timeout)
        self.timeout_timer.start(500)  # Check every 500ms
        
        self.initUI()
        
        # Request initial camera scan
        QTimer.singleShot(1000, self.camera_controller.refresh_cameras)  # Delay to let everything initialize
    
    def initUI(self):
        """Initialize the user interface with full camera controls"""
        self.setWindowTitle('Coffee Camera UI (ROS Transport)')
        self.setGeometry(100, 100, 1000, 700)
        
        # Main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        
        # Camera feed display (main area)
        self.image_label = QLabel("Initializing...")
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setStyleSheet("border: 1px solid gray; background-color: black; color: white;")
        main_layout.addWidget(self.image_label, 1)  # Give it most of the space
        
        # Controls layout (horizontal)
        controls_layout = QHBoxLayout()
        
        # Camera selection section
        camera_section = QVBoxLayout()
        camera_label = QLabel("Camera:")
        camera_label.setStyleSheet("font-weight: bold;")
        self.camera_combo = QComboBox()
        self.camera_combo.currentIndexChanged.connect(self.on_camera_changed)
        
        self.refresh_button = QPushButton("Refresh")
        self.refresh_button.clicked.connect(self.on_refresh_cameras)
        
        camera_section.addWidget(camera_label)
        camera_section.addWidget(self.camera_combo)
        camera_section.addWidget(self.refresh_button)
        
        # Quality and face detection section
        settings_section = QVBoxLayout()
        settings_label = QLabel("Settings:")
        settings_label.setStyleSheet("font-weight: bold;")
        
        self.quality_checkbox = QCheckBox("High Quality (1080p)")
        self.quality_checkbox.setChecked(self.high_quality)
        self.quality_checkbox.stateChanged.connect(self.on_quality_changed)
        
        self.face_detection_checkbox = QCheckBox("Face Detection")
        self.face_detection_checkbox.setChecked(self.face_detection_enabled)
        self.face_detection_checkbox.stateChanged.connect(self.on_face_detection_changed)
        
        settings_section.addWidget(settings_label)
        settings_section.addWidget(self.quality_checkbox)
        settings_section.addWidget(self.face_detection_checkbox)
        
        # Diagnostics section
        diagnostics_section = QVBoxLayout()
        diagnostics_label = QLabel("Diagnostics:")
        diagnostics_label.setStyleSheet("font-weight: bold;")
        
        self.diagnostics_button = QPushButton("Camera Diagnostics")
        self.diagnostics_button.clicked.connect(self.on_diagnostics_requested)
        
        diagnostics_section.addWidget(diagnostics_label)
        diagnostics_section.addWidget(self.diagnostics_button)
        
        # Add sections to controls layout
        controls_layout.addLayout(camera_section)
        controls_layout.addLayout(settings_section)
        controls_layout.addLayout(diagnostics_section)
        controls_layout.addStretch(1)  # Add spacer
        
        main_layout.addLayout(controls_layout)
        
        # Performance info label (bottom)
        self.info_label = QLabel("Performance stats will appear here...")
        self.info_label.setAlignment(Qt.AlignCenter)
        self.info_label.setMaximumHeight(50)
        self.info_label.setStyleSheet("background-color: #f0f0f0; padding: 5px; border-radius: 3px;")
        main_layout.addWidget(self.info_label)
        
        # Connection status label
        self.status_label = QLabel("Status: Connecting to camera_node...")
        self.status_label.setMaximumHeight(30)
        self.status_label.setStyleSheet("color: #666; font-size: 11px;")
        main_layout.addWidget(self.status_label)
    
    def display_frame(self, frame, latency):
        """Display the received frame and update performance metrics"""
        try:
            # Track display performance
            current_time = time.time()
            display_interval = current_time - self.last_display_time
            self.last_display_time = current_time
            self.last_frame_received_time = current_time
            self.display_count += 1
            
            # Mark that we're receiving frames
            if not self.is_receiving_frames:
                self.is_receiving_frames = True
                self.node.get_logger().info("Camera frames detected - connection established")
            
            # Calculate display FPS
            elapsed = current_time - self.display_start_time
            display_fps = self.display_count / elapsed if elapsed > 0 else 0
            
            # Convert BGR to RGB for Qt display
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Scale image to fit label while preserving aspect ratio
            label_width = self.image_label.width()
            label_height = self.image_label.height()
            
            if label_width > 0 and label_height > 0:
                h, w = rgb_frame.shape[:2]
                scale_w = label_width / w
                scale_h = label_height / h
                scale = min(scale_w, scale_h, 1.0)  # Don't upscale
                
                if scale < 1.0:
                    new_size = (int(w * scale), int(h * scale))
                    rgb_frame = cv2.resize(rgb_frame, new_size, interpolation=cv2.INTER_AREA)
            
            # Convert to QImage and display
            h, w, ch = rgb_frame.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image)
            self.image_label.setPixmap(pixmap)
            
            # Reset styling to normal when receiving frames
            self.image_label.setStyleSheet("border: 1px solid gray;")
            
            # Update performance info
            info_text = (
                f"Display FPS: {display_fps:.1f} | "
                f"Frame Interval: {display_interval*1000:.1f}ms | "
                f"Transport Latency: {latency*1000:.1f}ms | "
                f"Frame Size: {w}x{h}"
            )
            self.info_label.setText(info_text)
            
        except Exception as e:
            self.node.get_logger().error(f"Error displaying frame: {e}")
    
    # UI Control Callbacks
    def on_camera_changed(self, index):
        """Handle camera selection change"""
        if index >= 0 and index < len(self.available_cameras):
            camera_index, camera_name = self.available_cameras[index]
            self.current_camera_index = camera_index
            self.camera_controller.select_camera(camera_index)
            self.status_label.setText(f"Status: Switching to {camera_name}")
    
    def on_refresh_cameras(self):
        """Handle refresh cameras button click"""
        self.camera_controller.refresh_cameras()
        self.status_label.setText("Status: Scanning for cameras...")
    
    def on_quality_changed(self, state):
        """Handle quality checkbox change"""
        self.high_quality = bool(state)
        self.camera_controller.set_quality(self.high_quality)
        quality_text = "high quality (1080p)" if self.high_quality else "standard quality (480p)"
        self.status_label.setText(f"Status: Switching to {quality_text}")
    
    def on_face_detection_changed(self, state):
        """Handle face detection checkbox change"""
        self.face_detection_enabled = bool(state)
        self.camera_controller.toggle_face_detection(self.face_detection_enabled)
        detection_text = "enabled" if self.face_detection_enabled else "disabled"
        self.status_label.setText(f"Status: Face detection {detection_text}")
    
    def on_diagnostics_requested(self):
        """Handle diagnostics button click"""
        self.camera_controller.request_diagnostics()
    
    # ROS Communication Callbacks
    def update_camera_list(self, cameras_list):
        """Update UI with available cameras from ROS"""
        self.available_cameras = cameras_list
        self.camera_combo.clear()
        
        for index, name in cameras_list:
            self.camera_combo.addItem(name, index)
        
        if cameras_list:
            self.status_label.setText(f"Status: Found {len(cameras_list)} camera(s)")
        else:
            self.status_label.setText("Status: No cameras found")
            self.image_label.setText("No cameras available\n\nMake sure camera_node is running and cameras are connected")
    
    def update_camera_status(self, status_info):
        """Update UI with camera status from ROS"""
        self.status_label.setText(f"Status: {status_info}")
    
    def show_diagnostics(self, diagnostics_info):
        """Show diagnostics information in a popup"""
        QMessageBox.information(self, "Camera Diagnostics", diagnostics_info)
    
    def check_frame_timeout(self):
        """Check if frames have stopped coming and update UI accordingly"""
        current_time = time.time()
        
        # If we were receiving frames but haven't gotten one recently
        if self.is_receiving_frames and self.last_frame_received_time > 0:
            time_since_last_frame = current_time - self.last_frame_received_time
            
            if time_since_last_frame > self.frame_timeout_seconds:
                self.is_receiving_frames = False
                self.node.get_logger().warn(f"No camera frames received for {time_since_last_frame:.1f} seconds - connection lost")
                
                # Update UI to show connection lost
                self.image_label.setText("No camera feed\n\nConnection lost - check that camera_node is running")
                self.image_label.setStyleSheet("border: 1px solid red; background-color: #2a1a1a; color: #ff6666;")
                
                # Update info label
                self.info_label.setText("Connection lost - No frames received")
        
        # If we've never received frames, show waiting message
        elif not self.is_receiving_frames and self.last_frame_received_time == 0:
            self.image_label.setText("Waiting for camera frames...\n\nMake sure camera_node is running")
            self.image_label.setStyleSheet("border: 1px solid gray; background-color: black; color: white;")
            self.info_label.setText("Waiting for camera_node to publish frames...")
    
    def closeEvent(self, event):
        """Handle window close event"""
        self.timeout_timer.stop()
        self.node.get_logger().info("Camera UI window closed")
        event.accept()


class CameraViewerTestNode(Node):
    """ROS2 node providing separated camera UI that communicates via ROS transport"""
    
    def __init__(self):
        super().__init__('camera_viewer_test_node')
        self.get_logger().info('Camera UI Node starting...')
        self.get_logger().info('This UI communicates with camera_node via ROS topics')
        
        # Create Qt application
        self.app = QApplication(sys.argv)
        
        # Create and show the viewer window
        self.viewer = CameraViewerTest(self)
        self.viewer.show()
        
        self.get_logger().info('Camera UI window opened')
        self.get_logger().info('Connecting to camera_node via /coffee_bot/ topics')


def main(args=None):
    """
    Main entry point for separated camera UI.
    
    This creates a full-featured camera control UI that communicates with camera_node
    via ROS topics under the /coffee_bot/ namespace. It provides:
    - Camera selection and refresh
    - Quality control (480p/1080p)
    - Face detection toggle
    - Camera diagnostics
    - Real-time video display with performance metrics
    
    The UI publishes control commands to:
    - /coffee_bot/camera/cmd/select
    - /coffee_bot/camera/cmd/quality  
    - /coffee_bot/camera/cmd/face_detection
    - /coffee_bot/camera/cmd/refresh
    
    And subscribes to:
    - /coffee_bot/camera/image_raw (video frames)
    - /coffee_bot/camera/status/* (status updates)
    """
    try:
        # Initialize ROS2
        rclpy.init(args=args)
        
        # Create the camera UI node
        node = CameraViewerTestNode()
        
        # Create a timer to spin ROS while Qt runs
        def spin_ros():
            rclpy.spin_once(node, timeout_sec=0.001)
        
        # Set up Qt timer to handle ROS spinning
        from python_qt_binding.QtCore import QTimer
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