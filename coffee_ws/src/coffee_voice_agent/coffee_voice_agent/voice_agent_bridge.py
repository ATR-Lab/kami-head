#!/usr/bin/env python3
"""
ROS2 Bridge Node for LiveKit Voice Agent

This node acts as a bridge between the LiveKit voice agent (running as a separate process)
and the ROS2 ecosystem. It communicates with the voice agent via WebSocket and provides
ROS2 topics and services for integration with other robot components.
"""

import json
import asyncio
import threading
import time
import uuid
import datetime
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.parameter import Parameter
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from coffee_voice_agent_msgs.msg import AgentStatus, ToolEvent

try:
    import websockets
    import websockets.client
except ImportError:
    print("websockets not available - bridge will not function")
    websockets = None


class VoiceAgentBridge(Node):
    """ROS2 Bridge Node for LiveKit Voice Agent Communication"""
    
    def __init__(self):
        super().__init__('voice_agent_bridge')
        
        # Parameters
        self.declare_parameter('voice_agent_host', 'localhost')
        self.declare_parameter('voice_agent_port', 8080)
        self.declare_parameter('reconnect_interval', 5.0)
        
        # Agent configuration parameters (will be updated when agent connects)
        # These provide defaults until agent shares its actual configuration
        self.declare_parameter('user_response_timeout', 15.0)
        self.declare_parameter('final_timeout', 10.0) 
        self.declare_parameter('max_conversation_time', 180.0)
        self.declare_parameter('valid_emotions', ['friendly', 'helpful', 'curious'])
        self.declare_parameter('websocket_host', 'localhost')
        self.declare_parameter('websocket_port', 8080)
        self.declare_parameter('agent_version', 'unknown')
        self.declare_parameter('config_timestamp', '')
        
        # Configuration state tracking (exposed as ROS parameter for monitor)
        self.declare_parameter('config_received', False)
        
        self.host = self.get_parameter('voice_agent_host').value
        self.port = self.get_parameter('voice_agent_port').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        
        # Configuration state tracking
        self.config_received = False
        self.current_config = {}
        
        # WebSocket connection
        self.websocket: Optional[websockets.WebSocketClientProtocol] = None
        self.connection_active = False
        self.reconnect_task: Optional[asyncio.Task] = None
        self.websocket_loop: Optional[asyncio.AbstractEventLoop] = None  # Store event loop reference
        
        # Callback group for async operations
        self.callback_group = ReentrantCallbackGroup()
        
        # ROS2 Publishers (Voice Agent → ROS2)
        self.agent_status_pub = self.create_publisher(
            AgentStatus,
            'voice_agent/status',
            10,
            callback_group=self.callback_group
        )
        
        self.tool_event_pub = self.create_publisher(
            ToolEvent,
            'voice_agent/tool_events',
            10,
            callback_group=self.callback_group
        )
        
        self.user_speech_pub = self.create_publisher(
            String,
            'voice_agent/user_speech',
            10,
            callback_group=self.callback_group
        )
        
        self.connected_pub = self.create_publisher(
            Bool, 
            'voice_agent/connected', 
            10,
            callback_group=self.callback_group
        )
        
        # ROS2 Subscribers (ROS2 → Voice Agent)
        self.virtual_request_sub = self.create_subscription(
            String,
            'voice_agent/virtual_requests',
            self.handle_virtual_request,
            10,
            callback_group=self.callback_group
        )
        
        self.command_sub = self.create_subscription(
            String,
            'voice_agent/commands',
            self.handle_command,
            10,
            callback_group=self.callback_group
        )
        
        # Start WebSocket connection in separate thread
        self.websocket_thread = threading.Thread(target=self._run_websocket_client, daemon=True)
        self.websocket_thread.start()
        
        # Status timer
        self.status_timer = self.create_timer(1.0, self.publish_connection_status)
        
        self.get_logger().info(f"Voice Agent Bridge initialized - connecting to {self.host}:{self.port}")
    
    def _run_websocket_client(self):
        """Run WebSocket client in separate thread with its own event loop"""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self.websocket_loop = loop  # Store loop reference for thread-safe calls
        
        try:
            loop.run_until_complete(self._maintain_connection())
        except Exception as e:
            self.get_logger().error(f"WebSocket client error: {e}")
        finally:
            self.websocket_loop = None  # Clear reference
            loop.close()
    
    async def _maintain_connection(self):
        """Maintain WebSocket connection with automatic reconnection"""
        while rclpy.ok():
            try:
                uri = f"ws://{self.host}:{self.port}"
                self.get_logger().info(f"Attempting to connect to voice agent at {uri}")
                
                async with websockets.client.connect(uri) as websocket:
                    self.websocket = websocket
                    self.connection_active = True
                    self.get_logger().info("Connected to voice agent WebSocket")
                    
                    # Listen for messages from voice agent
                    async for message in websocket:
                        await self._handle_websocket_message(message)
                        
            except websockets.exceptions.ConnectionClosed:
                self.get_logger().warn("WebSocket connection closed")
            except Exception as e:
                self.get_logger().error(f"WebSocket connection error: {e}")
            finally:
                self.connection_active = False
                self.websocket = None
                
                if rclpy.ok():
                    self.get_logger().info(f"Reconnecting in {self.reconnect_interval} seconds...")
                    await asyncio.sleep(self.reconnect_interval)
    
    async def _handle_websocket_message(self, message: str):
        """Handle incoming messages from voice agent"""
        try:
            data = json.loads(message)
            message_type = data.get('type', 'unknown')
            
            if message_type == 'STATUS':
                # Handle status updates
                self.get_logger().info(f"Voice agent status: {data.get('message', 'Unknown')}")
                
            elif message_type == 'STARTUP':
                # Handle startup/ready events from voice agent
                self.get_logger().info("🔧 DEBUG: Received STARTUP message")
                self.get_logger().info(f"🔧 DEBUG: Full STARTUP data: {data}")
                
                # Extract the actual event data from the nested message structure
                event_data = data.get('data', {})
                self.get_logger().info(f"🔧 DEBUG: Extracted event_data: {event_data}")
                
                self.get_logger().info(f"Voice agent startup: {event_data.get('message', 'Ready')} (version: {event_data.get('version', 'unknown')})")
                
                # Process configuration if provided
                config_data = event_data.get('config', {})
                self.get_logger().info(f"🔧 DEBUG: Config data in STARTUP: {config_data}")
                
                if config_data:
                    self.get_logger().info("🔧 DEBUG: Configuration data found, updating parameters...")
                    await self._update_configuration_parameters(config_data)
                else:
                    self.get_logger().warn("No configuration data received in STARTUP message")
                    self.get_logger().info("🔧 DEBUG: Available keys in event_data: " + str(list(event_data.keys())))
                
            elif message_type == 'AGENT_STATUS':
                # Handle unified agent status events
                status_data = data.get('data', {})
                
                self.get_logger().info(f"Agent Status: mode={status_data.get('behavioral_mode', 'unknown')}, speech={status_data.get('speech_status', 'unknown')}, emotion={status_data.get('emotion', 'unknown')}")
                
                # Publish unified agent status to ROS2 topic
                status_msg = AgentStatus()
                status_msg.behavioral_mode = status_data.get('behavioral_mode', 'unknown')
                status_msg.speech_status = status_data.get('speech_status', 'unknown') 
                status_msg.emotion = status_data.get('emotion', 'unknown')
                status_msg.speech_text = status_data.get('speech_text', '')
                status_msg.previous_emotion = status_data.get('previous_emotion', 'unknown')
                status_msg.conversation_phase = status_data.get('conversation_phase', '')
                status_msg.last_tool_used = status_data.get('last_tool_used', '')
                
                # Parse timestamp if provided, otherwise use current time
                timestamp_str = status_data.get('timestamp')
                if timestamp_str:
                    try:
                        dt = datetime.datetime.fromisoformat(timestamp_str.replace('Z', '+00:00'))
                        status_msg.timestamp.sec = int(dt.timestamp())
                        status_msg.timestamp.nanosec = int((dt.timestamp() % 1) * 1e9)
                    except:
                        status_msg.timestamp = self.get_clock().now().to_msg()
                else:
                    status_msg.timestamp = self.get_clock().now().to_msg()
                    
                self.agent_status_pub.publish(status_msg)
                
            elif message_type == 'TOOL_EVENT':
                # Handle function tool events
                tool_data = data.get('data', {})
                
                self.get_logger().info(f"Tool Event: {tool_data.get('tool_name', 'unknown')} - {tool_data.get('status', 'unknown')}")
                
                # Publish tool event to ROS2 topic
                tool_msg = ToolEvent()
                tool_msg.tool_name = tool_data.get('tool_name', 'unknown')
                tool_msg.parameters = tool_data.get('parameters', [])
                tool_msg.result = tool_data.get('result', '')
                tool_msg.status = tool_data.get('status', 'unknown')
                
                # Parse timestamp if provided, otherwise use current time
                timestamp_str = tool_data.get('timestamp')
                if timestamp_str:
                    try:
                        dt = datetime.datetime.fromisoformat(timestamp_str.replace('Z', '+00:00'))
                        tool_msg.timestamp.sec = int(dt.timestamp())
                        tool_msg.timestamp.nanosec = int((dt.timestamp() % 1) * 1e9)
                    except:
                        tool_msg.timestamp = self.get_clock().now().to_msg()
                else:
                    tool_msg.timestamp = self.get_clock().now().to_msg()
                    
                self.tool_event_pub.publish(tool_msg)
                
            elif message_type == 'USER_SPEECH':
                # Handle user speech transcription events
                speech_data = data.get('data', {})
                user_text = speech_data.get('text', '')
                
                self.get_logger().info(f"User Speech: '{user_text[:50]}{'...' if len(user_text) > 50 else ''}'")
                
                # Publish user speech to ROS2 topic
                speech_msg = String()
                speech_msg.data = user_text
                self.user_speech_pub.publish(speech_msg)
                
            elif message_type == 'ACKNOWLEDGMENT':
                # Handle acknowledgment messages from voice agent
                status = data.get('status', 'unknown')
                message = data.get('message', '')
                self.get_logger().debug(f"Voice agent acknowledgment: {status} - {message}")
                
            else:
                self.get_logger().warn(f"Unknown message type from voice agent: {message_type}")
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid JSON from voice agent: {e}")
        except Exception as e:
            self.get_logger().error(f"Error handling voice agent message: {e}")
    
    def handle_virtual_request(self, msg: String):
        """Handle virtual request from ROS2 and forward to voice agent"""
        try:
            # Parse the ROS2 message
            request_data = json.loads(msg.data)
            
            # Transform ROS2 format to voice agent WebSocket API format
            # ROS2 format: {"request_type": "NEW_COFFEE_REQUEST", "content": "Espresso", "priority": "normal"}
            # Voice agent expects: {"type": "NEW_COFFEE_REQUEST", "coffee_type": "Espresso", "order_id": "123", "priority": "normal"}
            
            # Generate unique order ID for ROS2 requests
            order_id = f"ros2_{int(time.time())}_{str(uuid.uuid4())[:8]}"
            
            # Format for voice agent WebSocket API
            command = {
                'type': request_data.get('request_type', 'NEW_COFFEE_REQUEST'),  # Map request_type → type
                'coffee_type': request_data.get('content', 'Coffee'),            # Map content → coffee_type
                'order_id': order_id,                                           # Generate missing order_id
                'priority': request_data.get('priority', 'normal')              # Keep priority as-is
            }
            
            # Send to voice agent
            if self.websocket_loop and not self.websocket_loop.is_closed():
                asyncio.run_coroutine_threadsafe(
                    self._send_to_voice_agent(command),
                    self.websocket_loop
                )
            else:
                self.get_logger().warn("Cannot send virtual request - WebSocket event loop not available")
            
            self.get_logger().info(f"Forwarded virtual request: {request_data.get('request_type')} - {request_data.get('content')} (Order: {order_id})")
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid JSON in virtual request: {e}")
        except Exception as e:
            self.get_logger().error(f"Error handling virtual request: {e}")
    
    def handle_command(self, msg: String):
        """Handle command from ROS2 and forward to voice agent"""
        try:
            # Parse the command
            command_data = json.loads(msg.data)
            
            # Format for voice agent
            command = {
                'type': 'COMMAND',
                'action': command_data.get('action'),
                'parameters': command_data.get('parameters', {}),
                'timestamp': command_data.get('timestamp')
            }
            
            # Send to voice agent
            if self.websocket_loop and not self.websocket_loop.is_closed():
                asyncio.run_coroutine_threadsafe(
                    self._send_to_voice_agent(command),
                    self.websocket_loop
                )
            else:
                self.get_logger().warn("Cannot send command - WebSocket event loop not available")
            
            self.get_logger().info(f"Forwarded command: {command_data.get('action')}")
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid JSON in command: {e}")
        except Exception as e:
            self.get_logger().error(f"Error handling command: {e}")
    
    async def _send_to_voice_agent(self, data: dict):
        """Send data to voice agent via WebSocket"""
        if self.websocket and self.connection_active:
            try:
                message = json.dumps(data)
                await self.websocket.send(message)
            except Exception as e:
                self.get_logger().error(f"Error sending to voice agent: {e}")
        else:
            self.get_logger().warn("Cannot send to voice agent - not connected")
    
    async def _update_configuration_parameters(self, config_data: dict):
        """
        Update ROS2 parameters with configuration received from voice agent.
        
        This method is called when the agent sends its configuration via STARTUP or
        CONFIG_UPDATE messages. It updates the bridge's parameters so UI components
        can query them for consistent behavior.
        
        Args:
            config_data: Configuration dictionary from voice agent
        """
        self.get_logger().info("🔧 DEBUG: _update_configuration_parameters called")
        self.get_logger().info(f"🔧 DEBUG: Received config_data: {config_data}")
        
        try:
            # Store current config for tracking
            self.current_config = config_data.copy()
            self.config_received = True
            
            self.get_logger().info("🔧 DEBUG: Starting parameter updates...")
            
            # Update timing parameters
            if 'user_response_timeout' in config_data:
                timeout_value = float(config_data['user_response_timeout'])
                self.get_logger().info(f"🔧 DEBUG: Updating user_response_timeout to: {timeout_value}")
                self.set_parameters([
                    Parameter('user_response_timeout', 
                                            Parameter.Type.DOUBLE, 
                                            timeout_value)
                ])
                
            if 'final_timeout' in config_data:
                final_timeout_value = float(config_data['final_timeout'])
                self.get_logger().info(f"🔧 DEBUG: Updating final_timeout to: {final_timeout_value}")
                self.set_parameters([
                    Parameter('final_timeout',
                                            Parameter.Type.DOUBLE, 
                                            final_timeout_value)
                ])
                
            if 'max_conversation_time' in config_data:
                max_time_value = float(config_data['max_conversation_time'])
                self.get_logger().info(f"🔧 DEBUG: Updating max_conversation_time to: {max_time_value}")
                self.set_parameters([
                    Parameter('max_conversation_time',
                                            Parameter.Type.DOUBLE,
                                            max_time_value)
                ])
                
            # Update emotion configuration
            if 'valid_emotions' in config_data:
                emotions = config_data['valid_emotions']
                self.get_logger().info(f"🔧 DEBUG: Updating valid_emotions to: {emotions}")
                if isinstance(emotions, (list, set)):
                    self.set_parameters([
                        Parameter('valid_emotions',
                                                Parameter.Type.STRING_ARRAY,
                                                list(emotions))
                    ])
            
            # Update WebSocket configuration
            if 'websocket_host' in config_data:
                host_value = str(config_data['websocket_host'])
                self.get_logger().info(f"🔧 DEBUG: Updating websocket_host to: {host_value}")
                self.set_parameters([
                    Parameter('websocket_host',
                                            Parameter.Type.STRING,
                                            host_value)
                ])
                
            if 'websocket_port' in config_data:
                port_value = int(config_data['websocket_port'])
                self.get_logger().info(f"🔧 DEBUG: Updating websocket_port to: {port_value}")
                self.set_parameters([
                    Parameter('websocket_port',
                                            Parameter.Type.INTEGER,
                                            port_value)
                ])
            
            # Update metadata
            if 'agent_version' in config_data:
                version_value = str(config_data['agent_version'])
                self.get_logger().info(f"🔧 DEBUG: Updating agent_version to: {version_value}")
                self.set_parameters([
                    Parameter('agent_version',
                                            Parameter.Type.STRING,
                                            version_value)
                ])
                
            if 'config_timestamp' in config_data:
                timestamp_value = str(config_data['config_timestamp'])
                self.get_logger().info(f"🔧 DEBUG: Updating config_timestamp to: {timestamp_value}")
                self.set_parameters([
                    Parameter('config_timestamp',
                                            Parameter.Type.STRING,
                                            timestamp_value)
                ])
            
            self.get_logger().info("🔧 DEBUG: All parameter updates completed successfully")
            self.get_logger().info(
                f"Updated configuration parameters: "
                f"timeout={config_data.get('user_response_timeout', 'N/A')}s, "
                f"emotions={len(config_data.get('valid_emotions', []))}, "
                f"version={config_data.get('agent_version', 'N/A')}"
            )
            
            # Mark configuration as received from agent
            self.get_logger().info("🔧 DEBUG: Setting config_received=True")
            self.set_parameters([
                Parameter('config_received', Parameter.Type.BOOL, True)
            ])
            self.get_logger().info("🔧 DEBUG: config_received parameter updated successfully")
            
        except Exception as e:
            self.get_logger().error(f"🔧 DEBUG: Exception in _update_configuration_parameters: {e}")
            self.get_logger().error(f"Error updating configuration parameters: {e}")
            self.config_received = False
    
    def publish_connection_status(self):
        """Publish connection status periodically"""
        status_msg = Bool()
        status_msg.data = self.connection_active
        self.connected_pub.publish(status_msg)
    
    def destroy_node(self):
        """Clean up resources"""
        self.connection_active = False
        if hasattr(self, 'websocket_thread') and self.websocket_thread.is_alive():
            # Give time for graceful shutdown
            self.websocket_thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    """Main entry point for the bridge node"""
    if websockets is None:
        print("ERROR: websockets package not available. Install with: pip install websockets")
        return
    
    rclpy.init(args=args)
    
    try:
        bridge_node = VoiceAgentBridge()
        
        # Use MultiThreadedExecutor to handle async operations
        executor = MultiThreadedExecutor()
        executor.add_node(bridge_node)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            bridge_node.get_logger().info("Shutting down voice agent bridge...")
        finally:
            bridge_node.destroy_node()
            
    except Exception as e:
        print(f"Error starting voice agent bridge: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main() 