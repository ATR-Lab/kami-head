"""
Frame Receiver for Video Stream Processing

This module handles reception and processing of video frames from the
camera_node via ROS Image messages. It provides:

- Video frame reception from ROS topics
- Performance monitoring (FPS, latency)
- Connection timeout detection
- Frame format conversion for UI display

The receiver abstracts ROS Image message handling and provides
Qt signals for integration with UI components.
"""

import time
import numpy as np
from python_qt_binding.QtCore import QObject, pyqtSignal
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class FrameReceiver(QObject):
    """
    Receives ROS Image messages and processes them for UI display.
    
    This class handles video frame reception from camera_node, performs
    performance monitoring, and emits Qt signals for UI integration.
    
    Signals:
        frame_ready: Emitted when a new frame is ready for display
        connection_lost: Emitted when frame reception times out
        connection_restored: Emitted when frame reception resumes
        performance_update: Emitted with performance statistics
    """
    
    # Qt signals for UI communication
    frame_ready = pyqtSignal(np.ndarray, float)  # frame, latency
    connection_lost = pyqtSignal()
    connection_restored = pyqtSignal()
    performance_update = pyqtSignal(dict)  # performance metrics dict
    
    def __init__(self, node, topic_name='camera_frame'):
        """
        Initialize frame receiver with ROS node.
        
        Args:
            node: ROS2 node instance for creating subscribers
            topic_name: Name of the video stream topic (default: 'camera_frame')
        """
        super().__init__()
        self.node = node
        self.bridge = CvBridge()
        
        # Performance tracking
        self.frame_count = 0
        self.start_time = time.time()
        self.last_frame_time = 0
        self.last_frame_received_time = 0
        self.latency_history = []
        self.max_latency_samples = 100
        
        # Connection monitoring
        self.frame_timeout_seconds = 2.0
        self.is_receiving_frames = False
        
        # Subscribe to video stream
        self._setup_subscription(topic_name)
        
        self.node.get_logger().info(f"Frame receiver initialized for topic: {topic_name}")
    
    def _setup_subscription(self, topic_name):
        """
        Set up ROS subscription for video frames.
        
        Args:
            topic_name: Name of the video stream topic
        """
        self.subscription = self.node.create_subscription(
            Image,
            topic_name,
            self._frame_callback,
            10  # QoS queue size
        )
    
    def update_topic(self, new_topic_name):
        """
        Update the subscribed topic (useful for namespace changes).
        
        Args:
            new_topic_name: New topic name to subscribe to
        """
        # Destroy old subscription
        if hasattr(self, 'subscription'):
            self.node.destroy_subscription(self.subscription)
        
        # Create new subscription
        self._setup_subscription(new_topic_name)
        self.node.get_logger().info(f"Frame receiver updated to topic: {new_topic_name}")
        
        # Reset connection state
        self.is_receiving_frames = False
        self.last_frame_received_time = 0
    
    def check_connection_timeout(self):
        """
        Check if frame reception has timed out.
        
        This should be called periodically (e.g., via QTimer) to monitor
        connection status and emit appropriate signals.
        
        Returns:
            bool: True if connection is active, False if timed out
        """
        current_time = time.time()
        
        # Check if we've lost connection
        if self.is_receiving_frames and self.last_frame_received_time > 0:
            time_since_last_frame = current_time - self.last_frame_received_time
            
            if time_since_last_frame > self.frame_timeout_seconds:
                if self.is_receiving_frames:  # First time detecting timeout
                    self.is_receiving_frames = False
                    self.connection_lost.emit()
                    self.node.get_logger().warn(
                        f"No frames received for {time_since_last_frame:.1f} seconds - connection lost"
                    )
                return False
        
        return self.is_receiving_frames
    
    def get_performance_stats(self):
        """
        Get current performance statistics.
        
        Returns:
            dict: Performance metrics including FPS, latency stats
        """
        current_time = time.time()
        elapsed = current_time - self.start_time
        fps = self.frame_count / elapsed if elapsed > 0 else 0
        
        if self.latency_history:
            avg_latency = sum(self.latency_history) / len(self.latency_history)
            max_latency = max(self.latency_history)
            min_latency = min(self.latency_history)
        else:
            avg_latency = max_latency = min_latency = 0
        
        return {
            'fps': fps,
            'frame_count': self.frame_count,
            'avg_latency_ms': avg_latency * 1000,
            'max_latency_ms': max_latency * 1000,
            'min_latency_ms': min_latency * 1000,
            'is_connected': self.is_receiving_frames
        }
    
    def _frame_callback(self, msg):
        """
        Process incoming ROS Image messages.
        
        Args:
            msg: ROS Image message containing video frame
        """
        try:
            current_time = time.time()
            
            # Calculate latency
            msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
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
            
            # Update connection state
            self.last_frame_received_time = current_time
            if not self.is_receiving_frames:
                self.is_receiving_frames = True
                self.connection_restored.emit()
                self.node.get_logger().info("Frame reception restored - connection established")
            
            # Track frame count
            self.frame_count += 1
            
            # Emit frame for UI display
            self.frame_ready.emit(frame, latency)
            
            # Emit performance update periodically
            if self.frame_count % 30 == 0:  # Every 30 frames
                stats = self.get_performance_stats()
                self.performance_update.emit(stats)
                
                self.node.get_logger().debug(
                    f"Frame stats - FPS: {stats['fps']:.1f}, "
                    f"Avg Latency: {stats['avg_latency_ms']:.1f}ms"
                )
        
        except Exception as e:
            self.node.get_logger().error(f"Error processing frame: {e}") 