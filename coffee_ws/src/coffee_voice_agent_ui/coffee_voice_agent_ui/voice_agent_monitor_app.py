#!/usr/bin/env python3
"""
Standalone Voice Agent Monitor Application

This application creates a standalone Qt interface that hosts the Voice Agent Monitor
UI widgets directly. It enables the monitoring interface to be launched as a ROS2 node
without requiring the rqt_gui framework.

Usage:
    ros2 run coffee_voice_agent_ui voice_agent_monitor
    
Or in launch files:
    Node(package='coffee_voice_agent_ui', executable='voice_agent_monitor')
"""

import sys
import signal
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
import threading
from datetime import datetime

# Set Qt attributes BEFORE importing Qt modules
import os
os.environ['QT_AUTO_SCREEN_SCALE_FACTOR'] = '1'

from python_qt_binding.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QGridLayout
from python_qt_binding.QtCore import Qt, QTimer, pyqtSignal, pyqtSlot, QObject
from python_qt_binding.QtGui import QIcon

# ROS2 Messages
from std_msgs.msg import String, Bool
from coffee_voice_agent_msgs.msg import AgentStatus, ToolEvent, VipDetection, ExtensionEvent

# Import custom widgets directly
from .widgets.agent_status_widget import AgentStatusWidget
from .widgets.emotion_display_widget import EmotionDisplayWidget
from .widgets.conversation_widget import ConversationWidget
from .widgets.tool_monitor_widget import ToolMonitorWidget
from .widgets.analytics_widget import AnalyticsWidget
from .widgets.virtual_request_widget import VirtualRequestWidget
from .widgets.admin_override_widget import AdminOverrideWidget


class ROSBridge(QObject):
    """Qt bridge object for handling signals between ROS2 and Qt"""
    
    # PyQt signals for thread-safe UI updates
    agent_status_received = pyqtSignal(AgentStatus)
    tool_event_received = pyqtSignal(ToolEvent) 
    user_speech_received = pyqtSignal(str)
    connection_status_received = pyqtSignal(bool)
    vip_detection_received = pyqtSignal(VipDetection)
    extension_event_received = pyqtSignal(ExtensionEvent)
    
    def __init__(self):
        super().__init__()
    
    def emit_agent_status(self, status):
        """Emit agent status signal"""
        self.agent_status_received.emit(status)
    
    def emit_tool_event(self, event):
        """Emit tool event signal"""
        self.tool_event_received.emit(event)
    
    def emit_user_speech(self, speech):
        """Emit user speech signal"""
        self.user_speech_received.emit(speech)
    
    def emit_connection_status(self, connected):
        """Emit connection status signal"""
        self.connection_status_received.emit(connected)
    
    def emit_vip_detection(self, detection):
        """Emit VIP detection signal"""
        self.vip_detection_received.emit(detection)
    
    def emit_extension_event(self, event):
        """Emit extension event signal"""
        self.extension_event_received.emit(event)


class VoiceAgentMonitorNode(Node):
    """ROS2 Node for handling voice agent monitoring subscriptions"""
    
    def __init__(self, ros_bridge):
        super().__init__('voice_agent_monitor_node')
        
        # Store reference to the Qt bridge for signal emission
        self.ros_bridge = ros_bridge
        
        # Topic subscriptions
        self.status_sub = self.create_subscription(
            AgentStatus, 
            'voice_agent/status', 
            self.status_callback, 
            10
        )
        
        self.tool_sub = self.create_subscription(
            ToolEvent,
            'voice_agent/tool_events',
            self.tool_callback,
            10
        )
        
        self.speech_sub = self.create_subscription(
            String,
            'voice_agent/user_speech', 
            self.speech_callback,
            10
        )
        
        self.connection_sub = self.create_subscription(
            Bool,
            'voice_agent/connected',
            self.connection_callback,
            10
        )
        
        self.vip_detection_sub = self.create_subscription(
            VipDetection,
            'voice_agent/vip_detections',
            self.vip_detection_callback,
            10
        )
        
        self.extension_event_sub = self.create_subscription(
            ExtensionEvent,
            'voice_agent/extension_events',
            self.extension_event_callback,
            10
        )
        
        # Publisher for sending virtual requests
        self.virtual_request_pub = self.create_publisher(
            String,
            'voice_agent/virtual_requests',
            10
        )
        
        self.get_logger().info("Voice Agent Monitor Node initialized")
    
    def status_callback(self, msg):
        """Handle AgentStatus messages"""
        self.ros_bridge.emit_agent_status(msg)
    
    def tool_callback(self, msg):
        """Handle ToolEvent messages"""
        self.ros_bridge.emit_tool_event(msg)
    
    def speech_callback(self, msg):
        """Handle user speech messages"""
        self.ros_bridge.emit_user_speech(msg.data)
    
    def connection_callback(self, msg):
        """Handle connection status messages"""
        self.ros_bridge.emit_connection_status(msg.data)
    
    def vip_detection_callback(self, msg):
        """Handle VipDetection messages"""
        self.ros_bridge.emit_vip_detection(msg)
    
    def extension_event_callback(self, msg):
        """Handle ExtensionEvent messages"""
        self.ros_bridge.emit_extension_event(msg)


class VoiceAgentMonitorApp(QMainWindow):
    """Standalone Qt application hosting the Voice Agent Monitor widgets"""
    
    def __init__(self):
        super().__init__()
        
        # Set up main window
        self.setWindowTitle("Coffee Voice Agent Monitor")
        self.setMinimumSize(1200, 800)
        self.resize(1400, 900)
        
        # Set window icon (if available)
        try:
            self.setWindowIcon(QIcon.fromTheme("audio-input-microphone"))
        except:
            pass  # Icon not critical
        
        # Initialize configuration with reasonable defaults first
        self._initialize_simple_config()
        
        # Create central widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Set up the UI directly (without plugin wrapper)
        self._setup_ui(central_widget)
        
        # Initialize ROS2 node
        self._init_ros()
        
        # Start update timers
        self._setup_timers()
        
        # Set up graceful shutdown handling
        self.shutdown_requested = False
        
    def _setup_ui(self, central_widget):
        """Set up the main UI layout"""
        # Main layout - 2x3 grid for dashboard panels
        main_layout = QGridLayout()
        central_widget.setLayout(main_layout)
        
        # Create dashboard widgets
        self.agent_status_widget = AgentStatusWidget()
        self.emotion_widget = EmotionDisplayWidget()
        self.conversation_widget = ConversationWidget()
        self.tool_monitor_widget = ToolMonitorWidget()
        self.analytics_widget = AnalyticsWidget()
        self.virtual_request_widget = VirtualRequestWidget()
        self.admin_override_widget = AdminOverrideWidget()
        
        # Create left column container with vertical layout
        left_column_widget = QWidget()
        left_column_layout = QVBoxLayout()
        left_column_widget.setLayout(left_column_layout)
        
        # Add widgets to left column container
        left_column_layout.addWidget(self.agent_status_widget)
        left_column_layout.addWidget(self.emotion_widget)
        left_column_layout.addWidget(self.admin_override_widget)
        
        # Arrange widgets in dashboard layout (back to 2-row layout)
        # Row 0: Left Column Container (spans 2 rows) | Conversation Flow | Analytics
        main_layout.addWidget(left_column_widget, 0, 0, 2, 1)  # span 2 rows, 1 column
        main_layout.addWidget(self.conversation_widget, 0, 1)
        main_layout.addWidget(self.analytics_widget, 0, 2)
        
        # Row 1: (Left Column continues) | Tool Monitor | Virtual Requests
        main_layout.addWidget(self.tool_monitor_widget, 1, 1)
        main_layout.addWidget(self.virtual_request_widget, 1, 2)
        
        # Set column stretch to make conversation widget wider
        main_layout.setColumnStretch(0, 1)  # Left column container
        main_layout.setColumnStretch(1, 2)  # Conversation/Tools column (wider)
        main_layout.setColumnStretch(2, 1)  # Analytics/Controls column
        
        # Initialize conversation widget with default configuration
        if hasattr(self, 'last_config_update'):
            self.conversation_widget.update_configuration(
                self.last_config_update['config_data'], 
                "disconnected"  # Start as disconnected
            )
    
    def _init_ros(self):
        """Initialize ROS2 node and connections"""
        # Initialize rclpy if not already done
        if not rclpy.ok():
            rclpy.init()
        
        # Create Qt bridge for signals
        self.ros_bridge = ROSBridge()
        
        # Create ROS2 node with bridge reference
        self.ros_node = VoiceAgentMonitorNode(self.ros_bridge)
        
        # Connect ROS bridge signals to UI update methods
        self.ros_bridge.agent_status_received.connect(self._update_agent_status)
        self.ros_bridge.tool_event_received.connect(self._update_tool_event)
        self.ros_bridge.user_speech_received.connect(self._update_user_speech)
        self.ros_bridge.connection_status_received.connect(self._update_connection_status)
        self.ros_bridge.vip_detection_received.connect(self._update_vip_detection)
        self.ros_bridge.extension_event_received.connect(self._update_extension_event)
        
        # Connect virtual request widget signals to publishers
        self.virtual_request_widget.virtual_request_signal.connect(self._send_virtual_request)
        
        # Start ROS spinning in separate thread
        self.ros_executor = MultiThreadedExecutor()
        self.ros_executor.add_node(self.ros_node)
        self.ros_thread = threading.Thread(target=self.ros_executor.spin, daemon=True)
        self.ros_thread.start()
    
    def _setup_timers(self):
        """Set up update timers for the UI"""
        # Main update timer for real-time data
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._periodic_update)
        self.update_timer.start(100)  # 10 FPS updates
        
        # Analytics update timer (slower)
        self.analytics_timer = QTimer()
        self.analytics_timer.timeout.connect(self._update_analytics)
        self.analytics_timer.start(1000)  # 1 FPS for analytics
    
    def _initialize_simple_config(self):
        """Initialize configuration with reasonable defaults"""
        self.last_config_update = {
            'config_data': {
                'user_response_timeout': 15.0,
                'max_conversation_time': 180.0
            },
            'config_source': "fallback",
            'timestamp': datetime.now()
        }
    
    @pyqtSlot(AgentStatus)
    def _update_agent_status(self, status):
        """Update UI with new agent status"""
        self.agent_status_widget.update_status(status)
        self.emotion_widget.update_emotion(status.emotion, status.previous_emotion)
        self.conversation_widget.update_agent_state(status)
        
        # Reset admin override widget when conversation ends
        if status.behavioral_mode == "dormant":
            self.admin_override_widget.reset_vip_status()
    
    @pyqtSlot(ToolEvent)
    def _update_tool_event(self, event):
        """Update UI with new tool event"""
        self.tool_monitor_widget.add_tool_event(event)
        self.conversation_widget.add_tool_event(event)
    
    @pyqtSlot(str)
    def _update_user_speech(self, speech):
        """Update UI with user speech"""
        self.conversation_widget.add_user_speech(speech)
    
    @pyqtSlot(bool)
    def _update_connection_status(self, connected):
        """Update UI with connection status"""
        self.agent_status_widget.update_connection(connected)
        
        # Update conversation widget configuration status based on connection
        if hasattr(self, 'conversation_widget') and hasattr(self, 'last_config_update'):
            config_data = self.last_config_update['config_data']
            source = "connected" if connected else "disconnected"
            self.conversation_widget.update_configuration(config_data, source)
    
    @pyqtSlot(VipDetection)
    def _update_vip_detection(self, detection):
        """Update UI with new VIP detection"""
        self.admin_override_widget.update_vip_detection(
            detection.user_identifier,
            list(detection.matched_keywords),
            detection.importance_level,
            detection.recommended_extension_minutes
        )
    
    @pyqtSlot(ExtensionEvent)
    def _update_extension_event(self, event):
        """Update UI with new extension event"""
        self.admin_override_widget.update_extension_event(
            event.action,
            event.extension_minutes,
            event.reason,
            event.granted_by
        )
    
    @pyqtSlot(str)
    def _send_virtual_request(self, request_json):
        """Send virtual request through ROS2"""
        if self.ros_node:
            msg = String()
            msg.data = request_json
            self.ros_node.virtual_request_pub.publish(msg)
    
    def _periodic_update(self):
        """Periodic UI updates for real-time elements"""
        # Update conversation widget timestamps
        self.conversation_widget.update_timestamps()
        
        # Update tool monitor timing
        self.tool_monitor_widget.update_timing()
        
        # Update emotion transitions
        self.emotion_widget.update_animations()
    
    def _update_analytics(self):
        """Update analytics data"""
        # Collect data from other widgets for analytics
        agent_data = self.agent_status_widget.get_analytics_data()
        tool_data = self.tool_monitor_widget.get_analytics_data()
        conversation_data = self.conversation_widget.get_analytics_data()
        
        self.analytics_widget.update_analytics(agent_data, tool_data, conversation_data)
        
    def closeEvent(self, event):
        """Handle window close event"""
        self.shutdown_requested = True
        
        # Clean up timers
        if hasattr(self, 'update_timer'):
            self.update_timer.stop()
        if hasattr(self, 'analytics_timer'):
            self.analytics_timer.stop()
        
        # Clean up ROS
        if hasattr(self, 'ros_executor'):
            self.ros_executor.shutdown()
        
        if hasattr(self, 'ros_thread'):
            self.ros_thread.join(timeout=1.0)
        
        event.accept()


def signal_handler(signum, frame):
    """Handle system signals for graceful shutdown"""
    print(f"\nReceived signal {signum}, shutting down gracefully...")
    QApplication.quit()


def main(args=None):
    """Main entry point for standalone voice agent monitor application"""
    
    # Set Qt attributes BEFORE creating QApplication
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    
    # Set up signal handlers for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create Qt Application
    app = QApplication(sys.argv)
    app.setApplicationName("Coffee Voice Agent Monitor")
    app.setApplicationDisplayName("Coffee Voice Agent Monitor")
    
    # Create main window
    main_window = None
    
    try:
        # Create main application window
        main_window = VoiceAgentMonitorApp()
        main_window.show()
        
        print("Voice Agent Monitor started successfully!")
        print("- Window should be visible")
        print("- ROS2 topics are being monitored")
        print("- Use Ctrl+C to shutdown gracefully")
        
        # Run Qt event loop
        exit_code = app.exec_()
        
        print("Qt application closed")
        return exit_code
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        return 0
        
    except Exception as e:
        print(f"Error starting voice agent monitor: {e}")
        import traceback
        traceback.print_exc()
        return 1
        
    finally:
        # Clean up
        if main_window:
            main_window.shutdown_requested = True
            
        try:
            rclpy.shutdown()
        except Exception as e:
            print(f"Error during ROS2 shutdown: {e}")


if __name__ == '__main__':
    sys.exit(main(sys.argv)) 