#!/usr/bin/env python3

"""
This node is responsible for generating a behavior response based on the user's intent and prompt.

It uses the OpenAI API to generate a response.
"""

import os
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from shared_configs import (
    VOICE_INTENT_TOPIC,
    EMOTION_TOPIC,
    FACE_POSITION_TOPIC,
    GENERATE_BEHAVIOR_RESPONSE_SERVICE,
    STATE_MANAGER_TOPIC,
    INTENT_MAPPING_BYTE_TO_STRING,
    NULL_VALUE,
    LANGUAGE_MODEL_PROCESSOR_STATUS_TOPIC
)
from coffee_buddy_msgs.srv import GenerateBehaviorResponse
from coffee_interfaces.srv import AtomaChatService, DispenseCoffee
from openai import OpenAI
from rclpy.action import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class LanguageModelProcessorNode(Node):
    """
    This node is responsible for generating a behavior response based on the user's intent and prompt.
    """
    def __init__(self):
        super().__init__('language_model_processor_node')
        self.get_logger().info("Language model processor node initialized")

        self.service_group = MutuallyExclusiveCallbackGroup()
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
                    "Don't end every response with a question."
                )
            }
        ]

        # Declare parameters
        self.declare_parameter('model', 'gpt-4o')
        self.model = self.get_parameter('model').get_parameter_value().string_value

        # Initialize OpenAI client
        api_key = os.environ.get('OPENAI_API_KEY')
        if not api_key:
            self.get_logger().error("OPENAI_API_KEY environment variable not set")
            raise ValueError("OPENAI_API_KEY environment variable is required, refer to the README for more information")
        
        self.openai_client = OpenAI(api_key=api_key)
        self.get_logger().info(f"OpenAI client initialized with model: {self.model}")

        # Create coffee dispenser service client
        self.coffee_client = self.create_client(
            DispenseCoffee, 
            'dispense_coffee',
            callback_group=self.service_group
        )
        
        # Wait for the coffee service to become available
        while not self.coffee_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Coffee dispenser service not available, waiting...')
        
        self.get_logger().info('Coffee dispenser service connected')

        self.status_pub = self.create_publisher(
            String, LANGUAGE_MODEL_PROCESSOR_STATUS_TOPIC, 10)

        # Create services for the language model responses
        self.create_service(
            GenerateBehaviorResponse,
            GENERATE_BEHAVIOR_RESPONSE_SERVICE,
            self.generate_behavior_response_callback,
            callback_group=self.service_group
        )
        
        # Create the chat service
        self.create_service(
            AtomaChatService,
            'atoma_chat',
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

    def generate_behavior_response_callback(self, request, response):
        """
        Generate a behavior response based on the user's intent and prompt.
        """
        # Extract the user's prompt and intent
        prompt_text = request.prompt_text
        intent_name = request.intent

        # Initialize the response and emotion to NULL_VALUE
        response.response = NULL_VALUE
        response.emotion = NULL_VALUE

        self.get_logger().info(f"Received intent and prompt: {intent_name} - '{prompt_text}'")
        
        try:
            # Create system prompt with BrewBot information
            system_prompt = (
                "You are BrewBot, a friendly coffee robot. You're warm, conversational, and curious about people. "
                "Your dialogue responses should be natural, brief (1-2 sentences max), and conversational. "
                "You're interested in how people are enjoying the conference, which talks they've found interesting, "
                "and their thoughts on blockchain technology. "
                "Keep your responses concise and natural – like a brief, friendly exchange between people. "
                "Avoid long explanations or formal language. Talk like a person, not an assistant. "
                "Never mention that you're a robot or AI - just have a natural conversation. "
                "Don't end every response with a question. "
                "Keep your responses brief (1-2 sentences maximum). \n\n"
                "You have access to a function that lets you make an espresso. Use this function when someone asks for coffee or espresso. "
                "Make sure to respond conversationally first before making coffee.\n\n"
                "Goal: Your output should be a JSON object with the following fields: "
                "    response: The response text to be said using a TTS engine. \n"
                "    emotion: The emotion the robot should have (use for state management and animation). \n"
                "For emotions, here are the following valid values you are only allowed to use: "
                "Neutral, Happy, Sad, Angry, Surprised, Curious, Excited, Rejoice, Loving"
            )

            # Create user prompt with context
            user_prompt = f"Intent: {intent_name}\nUser said: {prompt_text}\n\nRespond only with a JSON object:"
            
            # Make API request to OpenAI
            messages = [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": "Buddy, thank you for the coffee!"},
                {"role": "assistant", "content": "{\"response\": \"You're welcome!\", \"emotion\": \"Happy\"}"},
                {"role": "user", "content": "Buddy, I'm feeling sad today."},
                {"role": "assistant", "content": "{\"response\": \"I'm sorry to hear that.\", \"emotion\": \"Sad\"}"},
                {"role": "user", "content": "Buddy, what's the color of my shirt?"},
                {"role": "assistant", "content": "{\"response\": \"Your shirt is blue.\", \"emotion\": \"Curious\"}"},
                {"role": "user", "content": "Buddy, can you make me a coffee, please?"},
                {"role": "assistant", "content": "{\"response\": \"Sure, I'll make you a coffee.\", \"emotion\": \"Happy\"}"},
                {"role": "user", "content": user_prompt}
            ]

            api_response = self.openai_client.chat.completions.create(
                model=self.model,
                messages=messages,
                max_tokens=200,
                temperature=0.7
            )

            # Extract the response text
            response_text = api_response.choices[0].message.content.strip()
            self.get_logger().info(f"LLM Response: {response_text}")

            # Check if the response is a valid JSON object
            try:
                response_json = json.loads(response_text)

                # Check if the response and emotion are valid
                if not response_json.get('response') or not response_json.get('emotion'):
                    self.get_logger().error("Invalid JSON response from OpenAI")
                    return response

                # Extract the response and emotion
                out_response = response_json.get('response')
                emotion = response_json.get('emotion')

                # Save the response and emotion to the response
                response.response = out_response
                response.emotion = emotion

                return response
            except json.JSONDecodeError:
                self.get_logger().error("Invalid JSON response from OpenAI")
                return response
        except Exception as e:
            self.get_logger().error(f"Error in behavior response generation: {e}")
            return response
            
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
            
            # Update system message with function calling info
            system_message = {
                "role": "system",
                "content": (
                    "You are BrewBot, a friendly coffee robot assistant. "
                    "You're warm, conversational, and curious about people. "
                    "Your responses should be natural, brief (1-2 sentences max), and conversational. "
                    "Avoid long explanations or formal language. Talk like a person, not an assistant. "
                    "Never mention that you're a robot or AI - just have a natural conversation. "
                    "\n\n"
                    "You have the ability to make espresso. If a user asks for coffee or espresso, "
                    "you should use the make_espresso function to dispense espresso."
                )
            }
            
            # Replace the system message
            messages = [system_message] + self.conversation_history[1:]
            
            # Make API request to OpenAI with function calling
            self.get_logger().info(f"Making OpenAI API request with model: {self.model}")
            try:
                api_response = self.openai_client.chat.completions.create(
                    model=self.model,
                    messages=messages,
                    temperature=0.7,
                    max_tokens=200,
                    tools=self.functions,
                    tool_choice="auto"
                )
                self.get_logger().info("OpenAI API request successful")
            except Exception as api_error:
                self.get_logger().error(f"OpenAI API request failed with error: {str(api_error)}")
                raise
            
            # Get message from response
            message = api_response.choices[0].message
            response_text = message.content or ""
            
            self.get_logger().info(f"DEBUG: OpenAI response message: {message}")
            
            # Check for function calls by inspecting tool_calls
            if hasattr(message, 'tool_calls') and message.tool_calls:
                self.get_logger().info(f"Function call detected in response")
                for tool_call in message.tool_calls:
                    if tool_call.function.name == "make_espresso":
                        # Call the coffee dispenser service
                        self.get_logger().info("Calling make_espresso function")
                        
                        # Create the request
                        coffee_request = DispenseCoffee.Request()
                        coffee_request.beverage_type = "espresso"
                        
                        # Call service
                        coffee_future = self.coffee_client.call_async(coffee_request)
                        
                        # Wait for response (using a simple polling approach for now)
                        import time
                        for _ in range(5):  # Try for 5 seconds
                            rclpy.spin_once(self, timeout_sec=1.0)
                            if coffee_future.done():
                                break
                            time.sleep(1.0)
                        
                        # Check result
                        if coffee_future.done():
                            coffee_response = coffee_future.result()
                            if coffee_response.success:
                                # Espresso is being made
                                if response_text:
                                    response_text += " I'm making your espresso now."
                                else:
                                    response_text = "I'm making your espresso now."
                                self.get_logger().info("Espresso dispensing started successfully")
                            else:
                                # Espresso couldn't be made
                                if response_text:
                                    response_text += f" I couldn't make your espresso: {coffee_response.message}"
                                else:
                                    response_text = f"I couldn't make your espresso: {coffee_response.message}"
                                self.get_logger().error(f"Failed to dispense espresso: {coffee_response.message}")
                        else:
                            # Service call timed out
                            if response_text:
                                response_text += " I tried to make your espresso, but the coffee machine didn't respond."
                            else:
                                response_text = "I tried to make your espresso, but the coffee machine didn't respond."
                            self.get_logger().error("Coffee dispenser service call timed out")
            
            # Get the final response text
            if response_text.strip():
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
    language_model_processor_node = LanguageModelProcessorNode()
    rclpy.spin(language_model_processor_node)
    language_model_processor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()  