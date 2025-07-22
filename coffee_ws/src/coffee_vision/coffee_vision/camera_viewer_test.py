#!/usr/bin/env python3

import sys
import cv2
import rclpy
import time
import numpy as np
from rclpy.node import Node
from python_qt_binding.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QLabel
from python_qt_binding.QtGui import QImage, QPixmap
from python_qt_binding.QtCore import Qt, QTimer, pyqtSignal, QObject
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


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
        
        # Subscribe to camera frame topic
        self.subscription = self.node.create_subscription(
            Image,
            'camera_frame',
            self.frame_callback,
            10  # QoS queue size
        )
        
        self.node.get_logger().info("Subscribed to 'camera_frame' topic")
    
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
    """Simple Qt window to display ROS camera frames and performance metrics"""
    
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.frame_receiver = FrameReceiver(node)
        self.frame_receiver.frame_ready.connect(self.display_frame)
        
        # Performance tracking for display
        self.display_count = 0
        self.display_start_time = time.time()
        self.last_display_time = time.time()
        
        self.initUI()
    
    def initUI(self):
        """Initialize the user interface"""
        self.setWindowTitle('Coffee Camera Viewer Test (ROS Transport)')
        self.setGeometry(100, 100, 800, 600)
        
        # Main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout(main_widget)
        
        # Video display label
        self.image_label = QLabel("Waiting for camera frames...")
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setStyleSheet("border: 1px solid gray; background-color: black; color: white;")
        layout.addWidget(self.image_label)
        
        # Performance info label
        self.info_label = QLabel("Performance stats will appear here...")
        self.info_label.setAlignment(Qt.AlignCenter)
        self.info_label.setMaximumHeight(50)
        layout.addWidget(self.info_label)
    
    def display_frame(self, frame, latency):
        """Display the received frame and update performance metrics"""
        try:
            # Track display performance
            current_time = time.time()
            display_interval = current_time - self.last_display_time
            self.last_display_time = current_time
            self.display_count += 1
            
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
    
    def closeEvent(self, event):
        """Handle window close event"""
        self.node.get_logger().info("Camera viewer test window closed")
        event.accept()


class CameraViewerTestNode(Node):
    """ROS2 node for testing camera frame reception and display via ROS transport"""
    
    def __init__(self):
        super().__init__('camera_viewer_test_node')
        self.get_logger().info('Camera Viewer Test Node starting...')
        self.get_logger().info('This node tests ROS transport performance for camera frames')
        
        # Create Qt application
        self.app = QApplication(sys.argv)
        
        # Create and show the viewer window
        self.viewer = CameraViewerTest(self)
        self.viewer.show()
        
        self.get_logger().info('Camera viewer test window opened')
        self.get_logger().info('Make sure camera_node is running to see frames')


def main(args=None):
    """Main entry point"""
    try:
        # Initialize ROS2
        rclpy.init(args=args)
        
        # Create the test node
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
        print(f"Error in camera viewer test: {e}")
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