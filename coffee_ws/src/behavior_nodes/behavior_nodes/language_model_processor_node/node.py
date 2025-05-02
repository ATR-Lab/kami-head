#!/usr/bin/env python3

"""
This node is responsible for generating a response based on the user's intent and prompt.

It uses the OpenAI API or Atoma API to generate a response via the LLM client abstraction.
"""

import os
import json
import logging
import time
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
                self.get_logger().info("Calling make_espresso function")
                
                # Create and send the coffee request
                coffee_request = CoffeeCommand.Request()
                coffee_request.action = "make"
                coffee_request.parameter = "espresso"
                
                try:
                    # Make async service call
                    self.get_logger().info("Calling coffee command service...")
                    future = self.coffee_client.call_async(coffee_request)
                    
                    # Wait for the future to complete with timeout
                    start_time = self.get_clock().now()
                    while (self.get_clock().now() - start_time).nanoseconds / 1e9 < 15.0:
                        if future.done():
                            response = future.result()
                            if response.success:
                                response_text = "I'm making your espresso now."
                                self.get_logger().info("Espresso dispensing started successfully")
                            else:
                                response_text = f"I couldn't make your espresso: {response.message}"
                                self.get_logger().error(f"Failed to dispense espresso: {response.message}")
                            break
                        # Small sleep to prevent busy waiting
                        time.sleep(0.1)
                    else:
                        # Timeout occurred
                        future.cancel()
                        response_text = "I tried to make your espresso, but the coffee machine didn't respond in time."
                        self.get_logger().error("Coffee dispenser service call timed out")
                        
                except Exception as e:
                    response_text = f"I couldn't make your espresso due to an error: {str(e)}"
                    self.get_logger().error(f"Error in make_espresso execution: {e}")
        
        return response_text
    
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
            # Process the response
            final_response = ""
            if response_text and response_text.strip():
                final_response = response_text
            elif tool_calls:
                # If we only have tool call results but no LLM response text,
                # use the tool call result as the response
                final_response = "I'm making your espresso now."
            
            if final_response:
                # Add assistant response to conversation history
                self.conversation_history.append({"role": "assistant", "content": final_response})
                
                # Set response
                response.response = final_response
                response.success = True
                response.error = ""
                
                self.get_logger().info(f"Chat response generated: {final_response}")
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