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
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QLabel, QPushButton, QComboBox, QHBoxLayout, QCheckBox, QMessageBox
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject


class FrameGrabber(QObject):
    """Dedicated thread for frame capture to improve performance"""
    frame_ready = pyqtSignal(np.ndarray)
    error = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.camera = None
        self.camera_index = 0
        self.running = False
        self.lock = threading.Lock()
        self.capture_thread = None
        self.frame_width = 1280  # Default to 720p (16:9)
        self.frame_height = 720
        self.high_quality = False
        self.backend = cv2.CAP_ANY  # Default backend
    
    def start(self, camera_index, backend=cv2.CAP_ANY):
        with self.lock:
            if self.running:
                self.stop()
            
            self.camera_index = camera_index
            self.backend = backend
            self.running = True
            self.capture_thread = threading.Thread(target=self._capture_loop)
            self.capture_thread.daemon = True
            self.capture_thread.start()
    
    def stop(self):
        with self.lock:
            self.running = False
            if self.capture_thread:
                if self.camera and self.camera.isOpened():
                    self.camera.release()
                    self.camera = None
                self.capture_thread.join(timeout=1.0)
                self.capture_thread = None
    
    def set_quality(self, high_quality):
        """Toggle between 720p and 1080p"""
        with self.lock:
            if high_quality:
                self.frame_width = 1920
                self.frame_height = 1080
            else:
                self.frame_width = 500
                self.frame_height = 500
            self.high_quality = high_quality
            
            # Re-initialize the camera with the new settings
            if self.camera and self.camera.isOpened():
                self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
                self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
    
    def _capture_loop(self):
        try:
            # Try different backends if the default doesn't work
            backends_to_try = [
                (cv2.CAP_V4L2, "V4L2"),        # Linux V4L2
                (cv2.CAP_GSTREAMER, "GStreamer"),  # GStreamer
                (cv2.CAP_ANY, "Auto-detect")    # Let OpenCV choose
            ]
            
            # Start with the specified backend
            if self.backend != cv2.CAP_ANY:
                backends_to_try.insert(0, (self.backend, "User selected"))
            
            success = False
            error_msg = ""
            
            for backend, backend_name in backends_to_try:
                try:
                    # Try to open with this backend
                    if backend == cv2.CAP_ANY:
                        self.camera = cv2.VideoCapture(self.camera_index)
                    else:
                        self.camera = cv2.VideoCapture(self.camera_index, backend)
                    
                    if not self.camera.isOpened():
                        error_msg = f"Could not open camera {self.camera_index} with {backend_name} backend"
                        continue
                    
                    # If we got here, the camera opened successfully
                    success = True
                    print(f"Successfully opened camera with {backend_name} backend")
                    break
                    
                except Exception as e:
                    error_msg = f"Error opening camera with {backend_name} backend: {str(e)}"
                    continue
            
            if not success:
                self.error.emit(f"Failed to open camera: {error_msg}")
                return
            
            # Set camera properties - prefer speed over resolution
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            
            # Set camera buffer size to 1 for minimal latency
            self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            # Try to set camera properties for better performance
            try:
                self.camera.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # Disable autofocus
                self.camera.set(cv2.CAP_PROP_FPS, 30)       # Request 30 FPS
            except:
                # Some cameras don't support these properties
                pass
            
            # FPS tracking
            frame_count = 0
            start_time = time.time()
            fps = 0
            
            while self.running:
                ret, frame = self.camera.read()
                
                if not ret:
                    time.sleep(0.01)  # Short sleep to prevent CPU overuse
                    continue
                
                # Add FPS text to frame
                frame_count += 1
                if frame_count >= 10:  # Update FPS every 10 frames
                    current_time = time.time()
                    elapsed = current_time - start_time
                    fps = frame_count / elapsed if elapsed > 0 else 0
                    frame_count = 0
                    start_time = current_time
                
                # Draw FPS on the frame
                cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                # Emit the frame signal
                self.frame_ready.emit(frame)
                
                # Short sleep to prevent tight loops
                time.sleep(0.001)
                
        except Exception as e:
            self.error.emit(f"Error in capture thread: {str(e)}")
        finally:
            if self.camera and self.camera.isOpened():
                self.camera.release()
                self.camera = None


class CameraViewer(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.frame_grabber = FrameGrabber()
        self.frame_grabber.frame_ready.connect(self.process_frame)
        self.frame_grabber.error.connect(self.handle_camera_error)
        self.high_quality = False
        self.initUI()
        self.check_video_devices()
        self.scan_cameras()
        
    def initUI(self):
        self.setWindowTitle('Coffee Camera - Webcam Viewer')
        self.setGeometry(100, 100, 800, 600)
        
        # Main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        
        # Camera feed display
        self.image_label = QLabel("No camera feed available")
        self.image_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(self.image_label)
        
        # Controls layout
        controls_layout = QHBoxLayout()  # Horizontal layout
        
        # Camera selection
        camera_selection_layout = QVBoxLayout()
        camera_label = QLabel("Camera:")
        self.camera_combo = QComboBox()
        self.camera_combo.currentIndexChanged.connect(self.change_camera)
        
        camera_selection_layout.addWidget(camera_label)
        camera_selection_layout.addWidget(self.camera_combo)
        
        # Refresh camera list button
        self.refresh_button = QPushButton("Refresh")
        self.refresh_button.clicked.connect(self.scan_cameras)
        camera_selection_layout.addWidget(self.refresh_button)
        
        controls_layout.addLayout(camera_selection_layout)
        
        # Quality controls
        quality_layout = QVBoxLayout()
        quality_label = QLabel("Quality:")
        
        self.quality_checkbox = QCheckBox("High Quality (1080p)")
        self.quality_checkbox.setChecked(self.high_quality)
        self.quality_checkbox.stateChanged.connect(self.toggle_quality)
        
        quality_layout.addWidget(quality_label)
        quality_layout.addWidget(self.quality_checkbox)
        
        controls_layout.addLayout(quality_layout)
        
        # Camera diagnostics button
        self.diagnostics_button = QPushButton("Camera Diagnostics")
        self.diagnostics_button.clicked.connect(self.show_diagnostics)
        controls_layout.addWidget(self.diagnostics_button)
        
        # Add some spacer for better layout
        controls_layout.addStretch(1)
        
        main_layout.addLayout(controls_layout)
        
        # Pre-allocate buffers for better performance
        self.qt_image = None
        self.pixmap = None
    
    def check_video_devices(self):
        """Check what video devices are available in the system"""
        if not os.path.exists('/dev'):
            self.node.get_logger().warn("No /dev directory found - not a Linux system?")
            return
        
        video_devices = []
        for device in os.listdir('/dev'):
            if device.startswith('video'):
                full_path = f"/dev/{device}"
                if os.access(full_path, os.R_OK):
                    video_devices.append(full_path)
        
        if not video_devices:
            self.node.get_logger().warn("No video devices found in /dev/")
            return
        
        self.node.get_logger().info(f"Found video devices: {', '.join(video_devices)}")
        
        # Check if any are in use
        try:
            output = subprocess.check_output(['fuser'] + video_devices, stderr=subprocess.STDOUT, text=True)
            self.node.get_logger().warn(f"Some video devices are in use: {output}")
        except subprocess.CalledProcessError:
            # No devices in use (fuser returns non-zero if no processes found)
            self.node.get_logger().info("No video devices appear to be in use")
        except Exception as e:
            # Some other error with fuser
            self.node.get_logger().warn(f"Error checking device usage: {e}")
    
    def show_diagnostics(self):
        """Show camera diagnostics information"""
        info = "Camera Diagnostics:\n\n"
        
        # Check for video devices
        video_devices = []
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
        info += f"OpenCV Version: {cv2.__version__}\n\n"
        
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
        
        # Show message box with diagnostic info
        QMessageBox.information(self, "Camera Diagnostics", info)
    
    def scan_cameras(self):
        """Scan for available cameras"""
        self.camera_combo.clear()
        
        # Stop current camera if running
        self.frame_grabber.stop()
        
        # Check for cameras using direct V4L2 access first
        self.node.get_logger().info("Scanning for cameras...")
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
                            self.node.get_logger().info(f"Device {device_path} exists but couldn't be opened as a camera")
                    except Exception as e:
                        self.node.get_logger().warn(f"Error checking {device_path}: {e}")
        
        # Fallback to generic scanning
        if not available_cameras:
            self.node.get_logger().info("Direct scan failed, trying generic approach...")
            for i in range(2):  # Only try the first two indices to avoid lengthy timeouts
                try:
                    cap = cv2.VideoCapture(i)
                    if cap.isOpened():
                        camera_name = f"Camera {i}"
                        available_cameras.append((i, camera_name))
                        cap.release()
                except Exception as e:
                    self.node.get_logger().warn(f"Error scanning camera {i}: {e}")
        
        # Add available cameras to combo box
        for idx, name in available_cameras:
            self.camera_combo.addItem(name, idx)
            
        if not available_cameras:
            self.node.get_logger().error("No cameras found!")
            self.image_label.setText("No cameras found! Check connections and permissions.")
            return
            
        # If any camera is available, start the first one
        if self.camera_combo.count() > 0:
            self.change_camera(self.camera_combo.currentIndex())
    
    def change_camera(self, index):
        """Change to a different camera"""
        if index >= 0:
            camera_index = self.camera_combo.itemData(index)
            self.node.get_logger().info(f"Changing to camera index {camera_index}")
            self.frame_grabber.stop()
            self.frame_grabber.set_quality(self.high_quality)
            # Try with V4L2 backend specifically on Linux
            if os.name == 'posix':
                self.frame_grabber.start(camera_index, cv2.CAP_V4L2)
            else:
                self.frame_grabber.start(camera_index)
    
    def toggle_quality(self, state):
        """Toggle between high and low quality modes"""
        self.high_quality = bool(state)
        self.node.get_logger().info(f"Quality set to {'high' if self.high_quality else 'standard'}")
        self.frame_grabber.set_quality(self.high_quality)
    
    def handle_camera_error(self, error_msg):
        """Handle errors from the camera thread"""
        self.node.get_logger().error(f"Camera error: {error_msg}")
        self.image_label.setText(f"Camera error: {error_msg}\nTry refreshing or check permissions.")
    
    def process_frame(self, frame):
        """Process and display a camera frame - optimized for speed"""
        if frame is None:
            return
            
        # Convert colors - BGR to RGB (optimized)
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Scale image to fit label if needed
        label_width = self.image_label.width()
        label_height = self.image_label.height()
        
        if label_width > 0 and label_height > 0:
            h, w = rgb_image.shape[:2]
            
            # Calculate scale factor to fit in label while preserving aspect ratio
            scale_w = label_width / w
            scale_h = label_height / h
            scale = min(scale_w, scale_h)
            
            # Only scale if necessary (smaller than label)
            if scale < 1.0:
                # Use NEAREST for fastest scaling
                new_size = (int(w * scale), int(h * scale))
                rgb_image = cv2.resize(rgb_image, new_size, interpolation=cv2.INTER_NEAREST)
        
        # Convert to QImage and display (reusing objects when possible)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        
        # Convert to QImage efficiently
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_image)
        self.image_label.setPixmap(pixmap)
    
    def closeEvent(self, event):
        """Clean up when window is closed"""
        self.frame_grabber.stop()
        event.accept()


class CameraNode(Node):
    def __init__(self):
        super().__init__('coffee_camera_node')
        self.get_logger().info('Camera node is starting...')
        
        # Start the UI
        app = QApplication(sys.argv)
        self.ui = CameraViewer(self)
        self.ui.show()
        
        # Start a background thread for ROS spinning
        self.ros_thread = threading.Thread(target=self.spin_thread)
        self.ros_thread.daemon = True
        self.ros_thread.start()
        
        # Start Qt event loop
        sys.exit(app.exec_())
    
    def spin_thread(self):
        """Background thread for ROS spinning"""
        rclpy.spin(self)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CameraNode()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main() 