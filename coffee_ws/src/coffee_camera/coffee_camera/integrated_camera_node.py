#!/usr/bin/env python3

import sys
import cv2
import rclpy
import time
import numpy as np
import threading
import os
import subprocess
from rclpy.node import Node
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                            QLabel, QPushButton, QCheckBox, QComboBox, QTabWidget)
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject

# Import our other modules
from coffee_camera.camera_node import FrameGrabber
from coffee_camera.head_tracking import HeadTrackingSystem, HeadTrackingUI

class IntegratedCameraNode(Node):
    """Node that integrates camera feed with head tracking"""
    def __init__(self):
        super().__init__('integrated_camera_node')
        self.get_logger().info('Integrated camera/tracking node starting...')
        
        # Initialize components
        self.init_ui()
        self.init_camera()
        self.init_head_tracker()
        
        # Start the node
        self.ros_thread = threading.Thread(target=self.spin_thread)
        self.ros_thread.daemon = True
        self.ros_thread.start()
        
        # Start Qt event loop
        sys.exit(self.app.exec_())
    
    def init_ui(self):
        """Initialize the UI"""
        self.app = QApplication(sys.argv)
        self.main_window = QMainWindow()
        self.main_window.setWindowTitle('Coffee Robot - Camera & Head Control')
        self.main_window.setGeometry(100, 100, 900, 700)
        
        # Main widget and tab controller
        self.tab_widget = QTabWidget()
        self.main_window.setCentralWidget(self.tab_widget)
        
        # Camera tab
        self.camera_widget = QWidget()
        self.camera_layout = QVBoxLayout(self.camera_widget)
        
        # Camera display
        self.image_label = QLabel("Camera initializing...")
        self.image_label.setAlignment(Qt.AlignCenter)
        self.camera_layout.addWidget(self.image_label)
        
        # Camera controls
        self.camera_controls = QHBoxLayout()
        
        # Camera selection
        self.camera_selection = QVBoxLayout()
        self.camera_selection.addWidget(QLabel("Camera:"))
        self.camera_combo = QComboBox()
        self.camera_selection.addWidget(self.camera_combo)
        self.camera_controls.addLayout(self.camera_selection)
        
        # Camera refresh button
        self.refresh_button = QPushButton("Refresh")
        self.camera_selection.addWidget(self.refresh_button)
        
        # Quality controls
        self.quality_controls = QVBoxLayout()
        self.quality_controls.addWidget(QLabel("Quality:"))
        self.quality_checkbox = QCheckBox("High Quality")
        self.quality_controls.addWidget(self.quality_checkbox)
        self.camera_controls.addLayout(self.quality_controls)
        
        # Face detection toggle
        self.face_detection_checkbox = QCheckBox("Face Detection")
        self.face_detection_checkbox.setChecked(True)
        self.quality_controls.addWidget(self.face_detection_checkbox)
        
        # Head tracking controls
        self.tracking_controls = QVBoxLayout()
        self.tracking_controls.addWidget(QLabel("Head Tracking:"))
        self.tracking_checkbox = QCheckBox("Track Faces")
        self.tracking_controls.addWidget(self.tracking_checkbox)
        
        # Reset head position button
        self.reset_head_button = QPushButton("Reset Head")
        self.tracking_controls.addWidget(self.reset_head_button)
        self.camera_controls.addLayout(self.tracking_controls)
        
        # Status display
        self.tracking_status = QLabel("Head tracking: disabled")
        self.tracking_controls.addWidget(self.tracking_status)
        
        # Add controls to camera layout
        self.camera_layout.addLayout(self.camera_controls)
        
        # Add camera tab to tab widget
        self.tab_widget.addTab(self.camera_widget, "Camera Feed")
        
        # Show the main window
        self.main_window.show()
    
    def init_camera(self):
        """Initialize the camera system"""
        self.frame_grabber = FrameGrabber()
        self.frame_grabber.frame_ready.connect(self.process_frame)
        self.frame_grabber.error.connect(self.handle_camera_error)
        
        # Connect camera UI controls
        self.camera_combo.currentIndexChanged.connect(self.change_camera)
        self.refresh_button.clicked.connect(self.scan_cameras)
        self.quality_checkbox.stateChanged.connect(self.toggle_quality)
        self.face_detection_checkbox.stateChanged.connect(self.toggle_face_detection)
        
        # Check available cameras
        self.scan_cameras()
    
    def init_head_tracker(self):
        """Initialize the head tracking system"""
        self.head_tracker = HeadTrackingSystem(self)
        
        # Create tracking UI in a separate tab
        self.tracking_ui = HeadTrackingUI(self, self.head_tracker)
        self.tab_widget.addTab(self.tracking_ui, "Head Control")
        
        # Connect tracking UI controls in camera tab
        self.tracking_checkbox.stateChanged.connect(self.toggle_tracking)
        self.reset_head_button.clicked.connect(self.head_tracker.reset_head_position)
        
        # Connect signals
        self.head_tracker.tracking_status.connect(self.update_tracking_status)
    
    def update_tracking_status(self, status):
        """Update tracking status display"""
        self.tracking_status.setText(f"Head tracking: {status}")
    
    def scan_cameras(self):
        """Scan for available cameras"""
        self.camera_combo.clear()
        
        # Stop current camera if running
        self.frame_grabber.stop()
        
        # Check for cameras using direct V4L2 access first
        self.get_logger().info("Scanning for cameras...")
        available_cameras = []
        
        # In Linux, we can check directly which devices are video capture devices
        if os.path.exists('/dev'):
            for i in range(10):
                device_path = f"/dev/video{i}"
                if os.path.exists(device_path) and os.access(device_path, os.R_OK):
                    try:
                        # Try to open the camera directly with V4L2
                        cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
                        if cap.isOpened():
                            # It's a valid camera
                            camera_name = f"Camera {i} ({device_path})"
                            available_cameras.append((i, camera_name))
                            cap.release()
                        else:
                            self.get_logger().info(f"Device {device_path} exists but couldn't be opened as a camera")
                    except Exception as e:
                        self.get_logger().warn(f"Error checking {device_path}: {e}")
        
        # Fallback to generic scanning
        if not available_cameras:
            self.get_logger().info("Direct scan failed, trying generic approach...")
            for i in range(2):  # Only try the first two indices to avoid lengthy timeouts
                try:
                    cap = cv2.VideoCapture(i)
                    if cap.isOpened():
                        camera_name = f"Camera {i}"
                        available_cameras.append((i, camera_name))
                        cap.release()
                except Exception as e:
                    self.get_logger().warn(f"Error scanning camera {i}: {e}")
        
        # Add available cameras to combo box
        for idx, name in available_cameras:
            self.camera_combo.addItem(name, idx)
            
        if not available_cameras:
            self.get_logger().error("No cameras found!")
            self.image_label.setText("No cameras found! Check connections and permissions.")
            return
            
        # If any camera is available, start the first one
        if self.camera_combo.count() > 0:
            self.change_camera(self.camera_combo.currentIndex())
    
    def change_camera(self, index):
        """Change to a different camera"""
        if index >= 0:
            camera_index = self.camera_combo.itemData(index)
            self.get_logger().info(f"Changing to camera index {camera_index}")
            self.frame_grabber.stop()
            
            # Update camera settings
            high_quality = self.quality_checkbox.isChecked()
            self.frame_grabber.set_quality(high_quality)
            self.frame_grabber.toggle_face_detection(self.face_detection_checkbox.isChecked())
            
            # Start camera
            if os.name == 'posix':
                self.frame_grabber.start(camera_index, cv2.CAP_V4L2)
            else:
                self.frame_grabber.start(camera_index)
    
    def toggle_quality(self, state):
        """Toggle camera quality"""
        high_quality = bool(state)
        self.get_logger().info(f"Setting camera quality to {'high' if high_quality else 'standard'}")
        self.frame_grabber.set_quality(high_quality)
    
    def toggle_face_detection(self, state):
        """Toggle face detection"""
        enabled = bool(state)
        self.get_logger().info(f"Face detection {'enabled' if enabled else 'disabled'}")
        self.frame_grabber.toggle_face_detection(enabled)
        
        # If face detection is disabled, also disable tracking
        if not enabled and self.tracking_checkbox.isChecked():
            self.tracking_checkbox.setChecked(False)
    
    def toggle_tracking(self, state):
        """Toggle head tracking"""
        enabled = bool(state)
        self.get_logger().info(f"Head tracking {'enabled' if enabled else 'disabled'}")
        
        # Ensure face detection is enabled if tracking is enabled
        if enabled and not self.face_detection_checkbox.isChecked():
            self.face_detection_checkbox.setChecked(True)
        
        self.head_tracker.enable_tracking(enabled)
    
    def handle_camera_error(self, error_msg):
        """Handle camera errors"""
        self.get_logger().error(f"Camera error: {error_msg}")
        self.image_label.setText(f"Camera error: {error_msg}\nTry refreshing or check permissions.")
    
    def process_frame(self, frame):
        """Process and display a camera frame"""
        if frame is None:
            return
        
        # Update frame dimensions in head tracker
        h, w = frame.shape[:2]
        self.head_tracker.set_frame_size(w, h)
        
        # If face detection is enabled and we have detected faces
        if self.face_detection_checkbox.isChecked() and hasattr(self.frame_grabber, 'prev_faces'):
            # Process tracking if enabled
            if self.tracking_checkbox.isChecked():
                # The head tracker will update the frame with tracking visuals
                frame = self.head_tracker.process_faces(frame, self.frame_grabber.prev_faces)
        
        # Convert colors - BGR to RGB for Qt
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Scale image to fit label if needed
        label_width = self.image_label.width()
        label_height = self.image_label.height()
        
        if label_width > 0 and label_height > 0:
            h, w = rgb_image.shape[:2]
            
            # Calculate scale factor to fit in label
            scale_w = label_width / w
            scale_h = label_height / h
            scale = min(scale_w, scale_h)
            
            # Only scale if necessary
            if scale < 1.0:
                new_size = (int(w * scale), int(h * scale))
                rgb_image = cv2.resize(rgb_image, new_size, interpolation=cv2.INTER_NEAREST)
        
        # Convert to QImage and display
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_image)
        self.image_label.setPixmap(pixmap)
    
    def spin_thread(self):
        """Background thread for ROS spinning"""
        rclpy.spin(self)
    
    def shutdown(self):
        """Clean shutdown of node"""
        self.frame_grabber.stop()
        self.head_tracker.enable_tracking(False)
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = IntegratedCameraNode()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main() 