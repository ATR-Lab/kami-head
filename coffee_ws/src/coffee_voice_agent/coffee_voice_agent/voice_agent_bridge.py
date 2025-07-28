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
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from coffee_voice_agent_msgs.msg import TtsEvent, AgentState, EmotionState, ConversationItem

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
        
        self.host = self.get_parameter('voice_agent_host').value
        self.port = self.get_parameter('voice_agent_port').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        
        # WebSocket connection
        self.websocket: Optional[websockets.WebSocketClientProtocol] = None
        self.connection_active = False
        self.reconnect_task: Optional[asyncio.Task] = None
        self.websocket_loop: Optional[asyncio.AbstractEventLoop] = None  # Store event loop reference
        
        # Callback group for async operations
        self.callback_group = ReentrantCallbackGroup()
        
        # ROS2 Publishers (Voice Agent → ROS2)
        self.state_pub = self.create_publisher(
            AgentState, 
            'voice_agent/state', 
            10,
            callback_group=self.callback_group
        )
        
        self.conversation_pub = self.create_publisher(
            ConversationItem, 
            'voice_agent/conversation', 
            10,
            callback_group=self.callback_group
        )
        
        self.emotion_pub = self.create_publisher(
            EmotionState, 
            'voice_agent/emotion', 
            10,
            callback_group=self.callback_group
        )
        
        self.tts_events_pub = self.create_publisher(
            TtsEvent,
            'voice_agent/tts_events',
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
            
            if message_type == 'STATE_CHANGE':
                # Publish agent state change
                state_msg = AgentState()
                state_msg.current_state = data.get('state', 'unknown')
                state_msg.previous_state = data.get('previous_state', 'unknown')
                # Parse timestamp if provided, otherwise use current time
                timestamp_str = data.get('timestamp')
                if timestamp_str:
                    # Convert ISO timestamp to ROS Time if needed
                    try:
                        dt = datetime.datetime.fromisoformat(timestamp_str.replace('Z', '+00:00'))
                        state_msg.timestamp.sec = int(dt.timestamp())
                        state_msg.timestamp.nanosec = int((dt.timestamp() % 1) * 1e9)
                    except:
                        # Fallback to current time
                        state_msg.timestamp = self.get_clock().now().to_msg()
                else:
                    state_msg.timestamp = self.get_clock().now().to_msg()
                self.state_pub.publish(state_msg)
                
            elif message_type == 'CONVERSATION':
                # Publish conversation transcript
                conv_msg = ConversationItem()
                conv_msg.role = data.get('role', 'unknown')
                conv_msg.text = data.get('text', '')
                # Parse timestamp if provided, otherwise use current time
                timestamp_str = data.get('timestamp')
                if timestamp_str:
                    try:
                        dt = datetime.datetime.fromisoformat(timestamp_str.replace('Z', '+00:00'))
                        conv_msg.timestamp.sec = int(dt.timestamp())
                        conv_msg.timestamp.nanosec = int((dt.timestamp() % 1) * 1e9)
                    except:
                        conv_msg.timestamp = self.get_clock().now().to_msg()
                else:
                    conv_msg.timestamp = self.get_clock().now().to_msg()
                self.conversation_pub.publish(conv_msg)
                
            elif message_type == 'EMOTION':
                # Publish emotion change
                emotion_msg = EmotionState()
                emotion_msg.emotion = data.get('emotion', 'unknown')
                emotion_msg.previous_emotion = data.get('previous_emotion', 'unknown')
                # Parse timestamp if provided, otherwise use current time
                timestamp_str = data.get('timestamp')
                if timestamp_str:
                    try:
                        dt = datetime.datetime.fromisoformat(timestamp_str.replace('Z', '+00:00'))
                        emotion_msg.timestamp.sec = int(dt.timestamp())
                        emotion_msg.timestamp.nanosec = int((dt.timestamp() % 1) * 1e9)
                    except:
                        emotion_msg.timestamp = self.get_clock().now().to_msg()
                else:
                    emotion_msg.timestamp = self.get_clock().now().to_msg()
                self.emotion_pub.publish(emotion_msg)
                
            elif message_type == 'STATUS':
                # Handle status updates
                self.get_logger().info(f"Voice agent status: {data.get('message', 'Unknown')}")
                
            elif message_type == 'STARTUP':
                # Handle startup/ready events from voice agent
                self.get_logger().info(f"Voice agent startup: {data.get('message', 'Ready')} (version: {data.get('version', 'unknown')})")
                
            elif message_type == 'TTS_EVENT':
                # Handle TTS started/finished events - parse nested data structure
                event_data = data.get('data', {})
                event = event_data.get('event', 'unknown')
                emotion = event_data.get('emotion', 'unknown')
                source = event_data.get('source', 'unknown')
                text = event_data.get('text', '')
                text_preview = text[:50] + "..." if len(text) > 50 else text
                
                self.get_logger().info(f"TTS {event}: emotion={emotion}, source={source}, text='{text_preview}'")
                
                # Publish TTS event to ROS2 topic
                tts_msg = TtsEvent()
                tts_msg.event = event
                tts_msg.emotion = emotion
                tts_msg.text = text_preview  # Use truncated text for efficiency
                tts_msg.source = source
                # Parse timestamp if provided, otherwise use current time
                timestamp_str = event_data.get('timestamp')
                if timestamp_str:
                    try:
                        dt = datetime.datetime.fromisoformat(timestamp_str.replace('Z', '+00:00'))
                        tts_msg.timestamp.sec = int(dt.timestamp())
                        tts_msg.timestamp.nanosec = int((dt.timestamp() % 1) * 1e9)
                    except:
                        tts_msg.timestamp = self.get_clock().now().to_msg()
                else:
                    tts_msg.timestamp = self.get_clock().now().to_msg()
                self.tts_events_pub.publish(tts_msg)
                
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