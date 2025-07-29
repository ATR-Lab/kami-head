#!/usr/bin/env python3
"""
Coffee Voice Agent Monitor - RQT Plugin for Real-time Monitoring

This RQT plugin provides a comprehensive dashboard for monitoring the Coffee Voice Agent
system including agent status, emotions, tool usage, conversation flow, and analytics.
"""

import os
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
from datetime import datetime

from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QGridLayout
from python_qt_binding.QtCore import Qt, QTimer, pyqtSignal, pyqtSlot

# ROS2 Messages
from std_msgs.msg import String, Bool
from coffee_voice_agent_msgs.msg import AgentStatus, ToolEvent

# Import custom widgets
from .widgets.agent_status_widget import AgentStatusWidget
from .widgets.emotion_display_widget import EmotionDisplayWidget
from .widgets.conversation_widget import ConversationWidget
from .widgets.tool_monitor_widget import ToolMonitorWidget
from .widgets.analytics_widget import AnalyticsWidget
from .widgets.controls_widget import ControlsWidget


class VoiceAgentMonitorNode(Node):
    """ROS2 Node for handling voice agent monitoring subscriptions"""
    
    # PyQt signals for thread-safe UI updates
    agent_status_received = pyqtSignal(AgentStatus)
    tool_event_received = pyqtSignal(ToolEvent) 
    user_speech_received = pyqtSignal(str)
    connection_status_received = pyqtSignal(bool)
    
    def __init__(self):
        super().__init__('voice_agent_monitor_node')
        
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
        
        # Publishers for sending commands
        self.virtual_request_pub = self.create_publisher(
            String,
            'voice_agent/virtual_requests',
            10
        )
        
        self.command_pub = self.create_publisher(
            String, 
            'voice_agent/commands',
            10
        )
        
        self.get_logger().info("Voice Agent Monitor Node initialized")
    
    def status_callback(self, msg):
        """Handle AgentStatus messages"""
        self.agent_status_received.emit(msg)
    
    def tool_callback(self, msg):
        """Handle ToolEvent messages"""
        self.tool_event_received.emit(msg)
    
    def speech_callback(self, msg):
        """Handle user speech messages"""
        self.user_speech_received.emit(msg.data)
    
    def connection_callback(self, msg):
        """Handle connection status messages"""
        self.connection_status_received.emit(msg.data)


class VoiceAgentMonitor(Plugin):
    """RQT Plugin for Coffee Voice Agent Monitoring Dashboard"""
    
    def __init__(self, context):
        super(VoiceAgentMonitor, self).__init__(context)
        
        # Give QObjects reasonable names
        self.setObjectName('VoiceAgentMonitor')
        
        # Create main widget
        self._widget = QWidget()
        
        # Set up the UI
        self._setup_ui()
        
        # Initialize ROS2 node
        self._init_ros()
        
        # Add widget to the user interface
        context.add_widget(self._widget)
        
        # Start update timer
        self._setup_timers()
    
    def _setup_ui(self):
        """Set up the main UI layout"""
        # Main layout - 3x3 grid for dashboard panels
        main_layout = QGridLayout()
        self._widget.setLayout(main_layout)
        
        # Create dashboard widgets
        self.agent_status_widget = AgentStatusWidget()
        self.emotion_widget = EmotionDisplayWidget()
        self.conversation_widget = ConversationWidget()
        self.tool_monitor_widget = ToolMonitorWidget()
        self.analytics_widget = AnalyticsWidget()
        self.controls_widget = ControlsWidget()
        
        # Arrange widgets in dashboard layout
        # Row 0: Agent Status | Conversation Flow | Analytics
        main_layout.addWidget(self.agent_status_widget, 0, 0)
        main_layout.addWidget(self.conversation_widget, 0, 1)
        main_layout.addWidget(self.analytics_widget, 0, 2)
        
        # Row 1: Emotion Display | Tool Monitor | Controls
        main_layout.addWidget(self.emotion_widget, 1, 0)
        main_layout.addWidget(self.tool_monitor_widget, 1, 1)
        main_layout.addWidget(self.controls_widget, 1, 2)
        
        # Set column stretch to make conversation widget wider
        main_layout.setColumnStretch(0, 1)  # Status/Emotion column
        main_layout.setColumnStretch(1, 2)  # Conversation/Tools column (wider)
        main_layout.setColumnStretch(2, 1)  # Analytics/Controls column
        
        self._widget.setWindowTitle('Coffee Voice Agent Monitor')
        self._widget.resize(1200, 800)
    
    def _init_ros(self):
        """Initialize ROS2 node and connections"""
        # Initialize rclpy if not already done
        if not rclpy.ok():
            rclpy.init()
        
        # Create ROS2 node
        self.ros_node = VoiceAgentMonitorNode()
        
        # Connect ROS signals to UI update methods
        self.ros_node.agent_status_received.connect(self._update_agent_status)
        self.ros_node.tool_event_received.connect(self._update_tool_event)
        self.ros_node.user_speech_received.connect(self._update_user_speech)
        self.ros_node.connection_status_received.connect(self._update_connection_status)
        
        # Connect control widget signals to publishers
        self.controls_widget.virtual_request_signal.connect(self._send_virtual_request)
        self.controls_widget.command_signal.connect(self._send_command)
        
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
    
    @pyqtSlot(AgentStatus)
    def _update_agent_status(self, status):
        """Update UI with new agent status"""
        self.agent_status_widget.update_status(status)
        self.emotion_widget.update_emotion(status.emotion, status.previous_emotion)
        self.conversation_widget.update_agent_state(status)
    
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
    
    @pyqtSlot(str)
    def _send_virtual_request(self, request_json):
        """Send virtual request through ROS2"""
        if self.ros_node:
            msg = String()
            msg.data = request_json
            self.ros_node.virtual_request_pub.publish(msg)
    
    @pyqtSlot(str)
    def _send_command(self, command_json):
        """Send command through ROS2"""
        if self.ros_node:
            msg = String()
            msg.data = command_json
            self.ros_node.command_pub.publish(msg)
    
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
    
    def shutdown_plugin(self):
        """Clean up when plugin shuts down"""
        if hasattr(self, 'update_timer'):
            self.update_timer.stop()
        if hasattr(self, 'analytics_timer'):
            self.analytics_timer.stop()
        
        if hasattr(self, 'ros_executor'):
            self.ros_executor.shutdown()
        
        if hasattr(self, 'ros_thread'):
            self.ros_thread.join(timeout=1.0)


def main():
    """Standalone entry point for testing"""
    import sys
    from python_qt_binding.QtWidgets import QApplication
    
    # Initialize ROS2
    rclpy.init()
    
    # Create Qt Application
    app = QApplication(sys.argv)
    
    # Create standalone widget
    widget = QWidget()
    monitor = VoiceAgentMonitor(None)
    layout = QVBoxLayout()
    layout.addWidget(monitor._widget)
    widget.setLayout(layout)
    widget.show()
    
    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main() 