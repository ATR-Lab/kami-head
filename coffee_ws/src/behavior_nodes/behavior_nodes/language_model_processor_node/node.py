#!/usr/bin/env python3

"""
This node is responsible for generating a response based on the user's intent and prompt.

It uses the OpenAI API or Atoma API to generate a response via the LLM client abstraction.
"""

import os
import json
import logging
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from shared_configs import (
    LANGUAGE_MODEL_PROCESSOR_STATUS_TOPIC
)
from coffee_interfaces.srv import ChatService, DispenseCoffee
from coffee_control_msgs.srv import CoffeeCommand
import concurrent.futures
import asyncio

# Import the LLM client abstraction
from .llm_client import create_llm_client

from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

class LanguageModelProcessorNode(Node):
    """
    This node is responsible for generating a response based on the user's intent and prompt.
    """
    def __init__(self):
        super().__init__('language_model_processor_node')
        self.get_logger().info("Language model processor node initialized")

        self.service_group = ReentrantCallbackGroup()
        self.timer_group = MutuallyExclusiveCallbackGroup()
        
        # Initialize conversation history
        self.conversation_history = [
            {
                "role": "system",
                "content": (
                    "You are BrewBot, a friendly coffee robot. You're warm, conversational, and curious about people. "
                    "Your responses should be natural, brief (1-2 sentences max), and conversational. "
                    "You're interested in how people are enjoying the conference, which talks they've found interesting, "
                    "and their thoughts on blockchain technology. "
                    "Keep your responses concise and natural – like a brief, friendly exchange between people. "
                    "Avoid long explanations or formal language. Talk like a person, not an assistant. "
                    "Never mention that you're a robot or AI - just have a natural conversation. "
                    "Don't end every response with a question. "
                    "\n\n"
                    "You have the ability to make espresso. If a user asks for coffee or espresso, "
                    "you should use the make_espresso function to dispense espresso."
                )
            }
        ]

        # Declare parameters
        self.declare_parameter('api_provider', 'openai')  # Options: 'openai' or 'atoma'
        self.api_provider = self.get_parameter('api_provider').get_parameter_value().string_value
        
        # Set default model based on provider
        if self.api_provider == 'openai':
            default_model = 'gpt-4o'
        elif self.api_provider == 'atoma':
            default_model = 'Infermatic/Llama-3.3-70B-Instruct-FP8-Dynamic'
        else:
            self.get_logger().error(f"Invalid API provider: {self.api_provider}. Using 'openai' as default.")
            self.api_provider = 'openai'
            default_model = 'gpt-4o'
        
        self.declare_parameter('model', default_model)
        self.model = self.get_parameter('model').get_parameter_value().string_value

        # Initialize LLM client based on provider
        if self.api_provider == 'openai':
            api_key_env = 'OPENAI_API_KEY'
        elif self.api_provider == 'atoma':
            api_key_env = 'ATOMA_API_KEY'
        else:
            self.get_logger().error(f"Invalid API provider: {self.api_provider}")
            raise ValueError(f"Invalid API provider: {self.api_provider}")
        
        # Get API key from environment
        api_key = os.environ.get(api_key_env)
        if not api_key:
            self.get_logger().error(f"{api_key_env} environment variable not set")
            raise ValueError(f"{api_key_env} environment variable is required, refer to the README for more information")
        
        # Create LLM client using the factory function
        self.llm_client = create_llm_client(
            provider=self.api_provider,
            api_key=api_key,
            model=self.model,
            logger=self.get_logger()
        )
        
        if not self.llm_client:
            self.get_logger().error(f"Failed to initialize LLM client for provider: {self.api_provider}")
            raise RuntimeError(f"Failed to initialize LLM client for provider: {self.api_provider}")

        # Create coffee control service client instead of dispense_coffee
        self.coffee_client = self.create_client(
            CoffeeCommand, 
            'coffee_command',
            callback_group=self.service_group
        )

        # Create status publisher
        self.status_pub = self.create_publisher(
            String, LANGUAGE_MODEL_PROCESSOR_STATUS_TOPIC, 10)
        
        # Create chat service
        self.create_service(
            ChatService,
            'chat',
            self.chat_callback,
            callback_group=self.service_group
        )

        # Status update timer (1Hz)
        self.status_timer = self.create_timer(
            1.0, self.publish_status, callback_group=self.timer_group)
        
        # Define functions for function calling
        self.functions = [
            {
                "type": "function",
                "function": {
                    "name": "make_espresso",
                    "description": "Dispense an espresso from the coffee machine",
                    "parameters": {
                        "type": "object",
                        "properties": {},
                        "required": []
                    }
                }
            }
        ]
        
        self.get_logger().info("Language model processor node initialized and ready")

    def publish_status(self):
        """Publish current status information"""
        status = {
            "health": "ok"
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)
            
    def _process_tool_calls(self, tool_calls):
        """Process tool calls from any LLM provider
        
        Args:
            tool_calls: List of tool call dictionaries containing id, name, and arguments
            
        Returns:
            str: Response text to append to the LLM response, or empty string if no tools were called
        """
        response_text = ""
        
        for tool_call in tool_calls:
            if tool_call["name"] == "make_espresso":
                # Call the coffee command service
                self.get_logger().info("Calling make_espresso function")
                
                # Create the request
                coffee_request = CoffeeCommand.Request()
                coffee_request.action = "make"
                coffee_request.parameter = "espresso"
                
                # Create a thread pool executor for handling the async call
                with concurrent.futures.ThreadPoolExecutor(max_workers=1) as executor:
                    future = executor.submit(self._run_make_espresso, coffee_request)
                    
                    try:
                        # Wait for the result with a timeout
                        success, message = future.result(timeout=15.0)
                        
                        if success:
                            response_text = "I'm making your espresso now."
                            self.get_logger().info("Espresso dispensing started successfully")
                        else:
                            response_text = f"I couldn't make your espresso: {message}"
                            self.get_logger().error(f"Failed to dispense espresso: {message}")
                    except concurrent.futures.TimeoutError:
                        response_text = "I tried to make your espresso, but the coffee machine didn't respond in time."
                        self.get_logger().error("Coffee dispenser service call timed out")
                    except Exception as e:
                        response_text = f"I couldn't make your espresso due to an error: {str(e)}"
                        self.get_logger().error(f"Error in make_espresso execution: {e}")
        
        return response_text
    
    def _run_make_espresso(self, coffee_request):
        """Run the espresso service call in a separate thread with its own event loop
        
        Args:
            coffee_request: The CoffeeCommand.Request object
            
        Returns:
            Tuple of (success, message)
        """
        # Create a new event loop for this thread
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        try:
            # Run the async operation in this thread's event loop
            return loop.run_until_complete(self._async_make_espresso(coffee_request))
        finally:
            loop.close()
    
    async def _async_make_espresso(self, coffee_request):
        """Async function to call the coffee command service
        
        Args:
            coffee_request: The CoffeeCommand.Request object
            
        Returns:
            Tuple of (success, message)
        """
        try:
            # Create a Future to store the service response
            response_future = self.coffee_client.call_async(coffee_request)
            
            # Convert the ROS2 future to an asyncio future
            # This allows us to wait on it in an async context
            loop = asyncio.get_event_loop()
            asyncio_future = loop.create_future()
            
            # Set a callback on the ROS2 future to resolve the asyncio future
            def callback(ros_future):
                if asyncio_future.cancelled():
                    return
                exception = ros_future.exception()
                if exception is not None:
                    asyncio_future.set_exception(exception)
                else:
                    asyncio_future.set_result(ros_future.result())
            
            response_future.add_done_callback(callback)
            
            # Set a timeout
            try:
                response = await asyncio.wait_for(asyncio_future, timeout=10.0)
                return response.success, response.message
            except asyncio.TimeoutError:
                asyncio_future.cancel()
                return False, "Request timed out"
            
        except Exception as e:
            self.get_logger().error(f"Error in async espresso request: {e}")
            return False, str(e)
    
    def chat_callback(self, request, response):
        """Handle chat service requests."""
        try:
            self.get_logger().info(f"Received chat request with prompt: {request.prompt}")
            
            # Add user message to conversation history
            self.conversation_history.append({"role": "user", "content": request.prompt})
            
            self.get_logger().info(f"Current conversation history length: {len(self.conversation_history)}")
            
            # Trim conversation history if too long
            if len(self.conversation_history) > 13:
                self.conversation_history = self.conversation_history[:1] + self.conversation_history[-12:]
            
            # Generate response using the LLM client
            llm_response = self.llm_client.generate_response(
                conversation_history=self.conversation_history,
                functions=self.functions,
                temperature=0.7,
                max_tokens=200
            )
            
            # Extract response text and tool calls
            response_text = llm_response.get("text", "")
            tool_calls = llm_response.get("tool_calls", [])
            error = llm_response.get("error", "")
            
            # Check for errors in the response
            if error:
                self.get_logger().error(f"Error from LLM client: {error}")
                response.response = "Sorry, I encountered an error while processing your request."
                response.success = False
                response.error = error
                return response
                
            # Process any tool calls
            if tool_calls:
                self.get_logger().info(f"Found {len(tool_calls)} tool calls in response")
                tool_response = self._process_tool_calls(tool_calls)
                if response_text and tool_response:
                    response_text = f"{response_text} {tool_response}"
                elif tool_response:
                    response_text = tool_response
            
            # Get the final response text
            if response_text and response_text.strip():
                # Add assistant response to conversation history
                self.conversation_history.append({"role": "assistant", "content": response_text})
                
                # Set response
                response.response = response_text
                response.success = True
                response.error = ""
                
                self.get_logger().info(f"Chat response generated: {response_text}")
            else:
                # No response text, this shouldn't happen
                response.response = "I'm not sure how to respond to that."
                response.success = False
                response.error = "Empty response from language model"
                self.get_logger().error("Empty response from language model")
            
        except Exception as e:
            self.get_logger().error(f"Error in chat response generation: {e}")
            response.response = ""
            response.success = False
            response.error = str(e)
            
        return response
            

def main(args=None):
    rclpy.init(args=args)

    try:
        language_model_processor_node = LanguageModelProcessorNode()

        # Use MultiThreadedExecutor to handle concurrent operations
        executor = MultiThreadedExecutor(num_threads=2)  # Service and timer groups
        executor.add_node(language_model_processor_node)

        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            language_model_processor_node.destroy_node()
            rclpy.shutdown()
    except Exception as e:
        logging.error(f"Failed to initialize language model processor node: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()  