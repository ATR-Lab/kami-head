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
from rclpy.parameter import Parameter
import threading
from datetime import datetime

from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QGridLayout
from python_qt_binding.QtCore import Qt, QTimer, pyqtSignal, pyqtSlot

# ROS2 Messages
from std_msgs.msg import String, Bool
from coffee_voice_agent_msgs.msg import AgentStatus, ToolEvent, VipDetection, ExtensionEvent

# Import custom widgets
from .widgets.agent_status_widget import AgentStatusWidget
from .widgets.emotion_display_widget import EmotionDisplayWidget
from .widgets.conversation_widget import ConversationWidget
from .widgets.tool_monitor_widget import ToolMonitorWidget
from .widgets.analytics_widget import AnalyticsWidget
from .widgets.virtual_request_widget import VirtualRequestWidget
from .widgets.admin_override_widget import AdminOverrideWidget


class VoiceAgentMonitorNode(Node):
    """ROS2 Node for handling voice agent monitoring subscriptions"""
    
    # PyQt signals for thread-safe UI updates
    agent_status_received = pyqtSignal(AgentStatus)
    tool_event_received = pyqtSignal(ToolEvent)
    user_speech_received = pyqtSignal(str)
    connection_status_received = pyqtSignal(bool)
    vip_detection_received = pyqtSignal(VipDetection)
    extension_event_received = pyqtSignal(ExtensionEvent)
    
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
    
    def vip_detection_callback(self, msg):
        """Handle VipDetection messages"""
        self.vip_detection_received.emit(msg)
    
    def extension_event_callback(self, msg):
        """Handle ExtensionEvent messages"""
        self.extension_event_received.emit(msg)


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
        self.ros_node.vip_detection_received.connect(self._update_vip_detection)
        self.ros_node.extension_event_received.connect(self._update_extension_event)
        
        # Connect virtual request widget signals to publishers
        self.virtual_request_widget.virtual_request_signal.connect(self._send_virtual_request)
        
        # Start ROS spinning in separate thread
        self.ros_executor = MultiThreadedExecutor()
        self.ros_executor.add_node(self.ros_node)
        self.ros_thread = threading.Thread(target=self.ros_executor.spin, daemon=True)
        self.ros_thread.start()
        
        # Query and distribute initial configuration (with delay for bridge startup)
        QTimer.singleShot(2000, self._query_and_distribute_config)  # Wait 2 seconds for bridge to initialize
        
        # Set up periodic config refresh in case bridge connects later
        self.config_refresh_timer = QTimer()
        self.config_refresh_timer.timeout.connect(self._retry_config_if_fallback)
        self.config_refresh_timer.start(10000)  # Check every 10 seconds
    
    def _query_and_distribute_config(self):
        """Query configuration parameters from bridge and distribute to widgets"""
        try:
            # Query parameters from the voice agent bridge
            config_data = {}
            config_source = "fallback"
            
            # Try to get parameters from bridge using parameter client
            try:
                from rclpy.parameter_client import AsyncParameterClient
                
                # Create parameter client to query bridge node
                param_client = AsyncParameterClient(self.ros_node, 'voice_agent_bridge')
                
                # Wait for bridge node to be available (timeout after 5 seconds)
                if param_client.wait_for_services(timeout_sec=5.0):
                    # Query timeout parameters from bridge
                    parameter_names = ['user_response_timeout', 'max_conversation_time', 'config_received']
                    
                    # Use async parameter client - get future and wait for result
                    future = param_client.get_parameters(parameter_names)
                    
                    # Spin until the future is complete (with timeout)
                    import time
                    start_time = time.time()
                    timeout = 5.0
                    
                    while not future.done() and (time.time() - start_time) < timeout:
                        rclpy.spin_once(self.ros_node, timeout_sec=0.1)
                    
                    if future.done():
                        parameters = future.result().values
                        
                        config_data = {
                            'user_response_timeout': parameters[0].double_value if parameters[0].type == 3 else 15.0,  # PARAMETER_DOUBLE = 3
                            'max_conversation_time': parameters[1].double_value if parameters[1].type == 3 else 180.0
                        }
                        
                        # Check if we got real configuration from agent using config_received flag
                        config_received = parameters[2].bool_value if parameters[2].type == 1 else False  # PARAMETER_BOOL = 1
                        
                        if config_received:
                            config_source = "agent"  # Bridge has received agent configuration
                        else:
                            config_source = "fallback"  # Bridge still using defaults
                        
                        self.ros_node.get_logger().info(f"Retrieved configuration from bridge: {config_data} (source: {config_source})")
                    else:
                        raise Exception("Parameter query timed out")
                else:
                    raise Exception("Bridge node not available")
                
            except Exception as e:
                # Parameters not available, use fallback values
                self.ros_node.get_logger().warn(f"Could not query bridge parameters: {e}, using fallback configuration")
                config_data = {
                    'user_response_timeout': 15.0,
                    'max_conversation_time': 180.0
                }
                config_source = "fallback"
            
            # Distribute configuration to widgets
            if hasattr(self, 'conversation_widget'):
                self.conversation_widget.update_configuration(config_data, config_source)
                self.ros_node.get_logger().info("Updated conversation widget configuration")
            
            # Add configuration info to analytics data
            self.last_config_update = {
                'config_data': config_data,
                'config_source': config_source,
                'timestamp': datetime.now()
            }
            
        except Exception as e:
            self.ros_node.get_logger().error(f"Error in configuration query and distribution: {e}")
    
    def _retry_config_if_fallback(self):
        """Retry configuration query if still using fallback values"""
        if (hasattr(self, 'last_config_update') and 
            self.last_config_update.get('config_source') == 'fallback'):
            self._query_and_distribute_config()
    
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