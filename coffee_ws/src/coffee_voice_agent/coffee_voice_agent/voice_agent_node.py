#!/usr/bin/env python3
"""
Coffee Voice Agent ROS2 Node

A ROS2 wrapper for the coffee barista voice agent that integrates
the original LiveKit voice agent with the Coffee Buddy robot system.
"""

import asyncio
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String, Bool
import logging
import json
import os

# Import the original voice agent components
from .livekit_voice_agent import (
    CoffeeBaristaAgent, 
    StateManager, 
    AgentState, 
    entrypoint
)
from livekit.agents import JobContext

logger = logging.getLogger(__name__)


class MockLiveKitContext:
    """Mock LiveKit context for ROS2 integration"""
    
    def __init__(self):
        self.room = MockRoom()
    
    async def connect(self):
        """Mock connection - LiveKit will handle actual connections"""
        logger.info("Mock LiveKit context connected for ROS2 integration")


class MockRoom:
    """Mock LiveKit room for ROS2 integration"""
    
    def __init__(self):
        self.name = "ros2_coffee_room"


class CoffeeVoiceAgentNode(Node):
    """ROS2 wrapper for the Coffee Voice Agent"""
    
    def __init__(self):
        super().__init__('coffee_voice_agent')
        
        self.get_logger().info("Starting Coffee Voice Agent ROS2 Node...")
        
        # Create callback group for async operations
        self.callback_group = ReentrantCallbackGroup()
        
        # Initialize voice agent components
        self.voice_agent = None
        self.mock_ctx = None
        self.agent_task = None
        
        # Set up ROS2 interface
        self.setup_ros_interface()
        
        # Start the voice agent
        self.start_voice_agent()
        
        self.get_logger().info("Coffee Voice Agent ROS2 Node started successfully!")
    
    def setup_ros_interface(self):
        """Set up ROS2 publishers and subscribers"""
        
        # Publishers for voice agent status
        self.state_pub = self.create_publisher(
            String, 
            '/coffee_voice_agent/state', 
            10,
            callback_group=self.callback_group
        )
        
        self.emotion_pub = self.create_publisher(
            String, 
            '/coffee_voice_agent/emotion', 
            10,
            callback_group=self.callback_group
        )
        
        self.user_input_pub = self.create_publisher(
            String, 
            '/coffee_voice_agent/user_input', 
            10,
            callback_group=self.callback_group
        )
        
        self.agent_response_pub = self.create_publisher(
            String, 
            '/coffee_voice_agent/agent_response', 
            10,
            callback_group=self.callback_group
        )
        
        self.wake_word_pub = self.create_publisher(
            Bool, 
            '/coffee_voice_agent/wake_word_detected', 
            10,
            callback_group=self.callback_group
        )
        
        # Subscribers for external integration
        self.virtual_request_sub = self.create_subscription(
            String,
            '/coffee_voice_agent/virtual_request',
            self.virtual_request_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.get_logger().info("ROS2 interface configured")
    
    def start_voice_agent(self):
        """Start the voice agent in an async task"""
        
        # Create mock context for integration
        self.mock_ctx = MockLiveKitContext()
        
        # Create a one-time timer to start the async task after ROS2 is fully initialized
        self.startup_timer = self.create_timer(0.1, self.start_voice_agent_task)
        
        self.get_logger().info("Voice agent startup scheduled")
    
    def start_voice_agent_task(self):
        """Timer callback to start the voice agent async task"""
        
        # Cancel the timer after first run
        self.startup_timer.cancel()
        
        # Start the voice agent using asyncio.ensure_future which works better with executors
        try:
            self.agent_task = asyncio.ensure_future(self.run_voice_agent())
            self.get_logger().info("Voice agent task started with ensure_future")
        except Exception as e:
            self.get_logger().error(f"Failed to start voice agent task: {e}")
            # Try alternative approach
            try:
                import concurrent.futures
                executor = concurrent.futures.ThreadPoolExecutor()
                self.agent_task = executor.submit(self._run_voice_agent_sync)
                self.get_logger().info("Voice agent started in thread executor")
            except Exception as e2:
                self.get_logger().error(f"Failed to start voice agent in thread: {e2}")
    
    def _run_voice_agent_sync(self):
        """Synchronous wrapper for running the voice agent"""
        try:
            asyncio.run(self.run_voice_agent())
        except Exception as e:
            self.get_logger().error(f"Voice agent sync wrapper error: {e}")
    
    async def run_voice_agent(self):
        """Run the original voice agent with ROS2 integration hooks"""
        
        try:
            # Validate environment variables
            required_vars = ["OPENAI_API_KEY"]
            missing_vars = [var for var in required_vars if not os.getenv(var)]
            
            if missing_vars:
                self.get_logger().error(f"Missing required environment variables: {missing_vars}")
                return
            
            # Create the voice agent
            self.voice_agent = CoffeeBaristaAgent()
            
            # Add ROS2 integration hooks
            self.add_ros2_hooks()
            
            # Run the original voice agent entrypoint
            self.get_logger().info("Starting original voice agent...")
            await entrypoint(self.mock_ctx)
            
        except Exception as e:
            self.get_logger().error(f"Voice agent error: {e}")
            import traceback
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")
    
    def add_ros2_hooks(self):
        """Add ROS2 integration hooks to the voice agent"""
        
        if not self.voice_agent or not self.voice_agent.state_manager:
            return
        
        # Store original methods
        original_transition = self.voice_agent.state_manager.transition_to_state
        original_emotion_proc = self.voice_agent.state_manager.process_emotional_response
        
        # Wrap state transitions
        async def ros2_transition_wrapper(new_state: AgentState):
            await original_transition(new_state)
            self.publish_state_change(new_state)
        
        # Wrap emotion processing
        def ros2_emotion_wrapper(llm_response: str):
            emotion, text = original_emotion_proc(llm_response)
            self.publish_emotion_change(emotion, text)
            return emotion, text
        
        # Replace methods with wrapped versions
        self.voice_agent.state_manager.transition_to_state = ros2_transition_wrapper
        self.voice_agent.state_manager.process_emotional_response = ros2_emotion_wrapper
        
        # Add reference for virtual requests
        self.voice_agent.state_manager.ros_node = self
        
        self.get_logger().info("ROS2 hooks added to voice agent")
    
    def publish_state_change(self, new_state: AgentState):
        """Publish state change to ROS2"""
        
        msg = String()
        msg.data = new_state.value
        self.state_pub.publish(msg)
        
        self.get_logger().info(f"Published state: {new_state.value}")
    
    def publish_emotion_change(self, emotion: str, text: str):
        """Publish emotion and response to ROS2"""
        
        # Publish emotion
        emotion_msg = String()
        emotion_msg.data = emotion
        self.emotion_pub.publish(emotion_msg)
        
        # Publish agent response
        response_msg = String()
        response_msg.data = text
        self.agent_response_pub.publish(response_msg)
        
        self.get_logger().info(f"Published emotion: {emotion}, response: {text[:50]}...")
    
    def virtual_request_callback(self, msg: String):
        """Handle virtual request messages from ROS2"""
        
        if self.voice_agent and self.voice_agent.state_manager:
            try:
                # Parse JSON request
                request_data = json.loads(msg.data)
                request_type = request_data.get('request_type', 'UNKNOWN')
                content = request_data.get('content', '')
                priority = request_data.get('priority', 'normal')
                
                # Queue the virtual request using original functionality
                self.voice_agent.state_manager.queue_virtual_request(request_type, content, priority)
                
                self.get_logger().info(f"Queued virtual request: {request_type} - {content}")
                
            except json.JSONDecodeError:
                self.get_logger().error(f"Invalid JSON in virtual request: {msg.data}")
            except Exception as e:
                self.get_logger().error(f"Error processing virtual request: {e}")
    
    def destroy_node(self):
        """Clean up when shutting down"""
        
        if self.agent_task and not self.agent_task.done():
            self.agent_task.cancel()
        
        if self.voice_agent:
            self.voice_agent.stop_wake_word_detection()
        
        super().destroy_node()


def main(args=None):
    """Main entry point for ROS2"""
    
    rclpy.init(args=args)
    
    try:
        # Create the voice agent node
        node = CoffeeVoiceAgentNode()
        
        # Use MultiThreadedExecutor for async operations
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        # Log startup info
        logger.info("☕ Coffee Barista Voice Agent ROS2 Node")
        logger.info(f"Wake Word: {'✅ Enabled' if os.getenv('PORCUPINE_ACCESS_KEY') else '❌ Disabled'}")
        logger.info(f"OpenAI: {'✅ Ready' if os.getenv('OPENAI_API_KEY') else '❌ Missing API Key'}")
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            logger.info("Received shutdown signal...")
        finally:
            node.destroy_node()
            executor.shutdown()
    
    except Exception as e:
        logger.error(f"Failed to start voice agent node: {e}")
        return 1
    finally:
        rclpy.shutdown()
    
    return 0


if __name__ == '__main__':
    exit(main()) 