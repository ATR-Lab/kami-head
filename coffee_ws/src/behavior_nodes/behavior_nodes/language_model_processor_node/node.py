#!/usr/bin/env python3

"""
This node is responsible for generating a behavior response based on the user's intent and prompt.

It uses the Atoma API to generate a response.
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
from coffee_interfaces.srv import AtomaChatService
from atoma_sdk import AtomaSDK
from atoma_sdk.models import ChatCompletionMessage
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
        self.declare_parameter('model', 'Infermatic/Llama-3.3-70B-Instruct-FP8-Dynamic')
        self.model = self.get_parameter('model').get_parameter_value().string_value

        # Initialize Atoma SDK client
        api_key = os.environ.get('ATOMA_API_KEY')
        if not api_key:
            self.get_logger().error("ATOMA_API_KEY environment variable not set")
            raise ValueError("ATOMA_API_KEY environment variable is required, refer to the README for more information")
        
        self.atoma_client = AtomaSDK(bearer_auth=api_key)
        self.get_logger().info(f"Atoma SDK initialized with model: {self.model}")

        self.status_pub = self.create_publisher(
            String, LANGUAGE_MODEL_PROCESSOR_STATUS_TOPIC, 10)

        # Create services for the language model responses
        self.create_service(
            GenerateBehaviorResponse,
            GENERATE_BEHAVIOR_RESPONSE_SERVICE,
            self.generate_behavior_response_callback,
            callback_group=self.service_group
        )
        
        # Create the Atoma chat service
        self.create_service(
            AtomaChatService,
            'atoma_chat',
            self.atoma_chat_callback,
            callback_group=self.service_group
        )

        # Status update timer (1Hz)
        self.status_timer = self.create_timer(
            1.0, self.publish_status, callback_group=self.timer_group)
        
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
                "Goal: Your output should be a JSON object with the following fields: "
                "    response: The response text to be said using a TTS engine. \n"
                "    emotion: The emotion the robot should have (use for state management and animation). \n"
                "For emotions, here are the following valid values you are only allowed to use: "
                "Neutral, Happy, Sad, Angry, Surprised, Curious, Excited, Rejoice, Loving"
            )

            # Create user prompt with context
            user_prompt = f"Intent: {intent_name}\nUser said: {prompt_text}\n\nRespond only with a JSON object:"
            
            # Make API request to Atoma
            api_response = self.atoma_client.chat.create(
                model=self.model,
                messages=[
                    ChatCompletionMessage(role="system", content=system_prompt),
                    ChatCompletionMessage(role="user", content="Buddy, thank you for the coffee!"),
                    ChatCompletionMessage(role="assistant", content="{\"response\": \"You're welcome!\", \"emotion\": \"Happy\"}"),
                    ChatCompletionMessage(role="user", content="Buddy, I'm feeling sad today."),
                    ChatCompletionMessage(role="assistant", content="{\"response\": \"I'm sorry to hear that.\", \"emotion\": \"Sad\"}"),
                    ChatCompletionMessage(role="user", content="Buddy, what's the color of my shirt?"),
                    ChatCompletionMessage(role="assistant", content="{\"response\": \"Your shirt is blue.\", \"emotion\": \"Curious\"}"),
                    ChatCompletionMessage(role="user", content="Buddy, can you make me a coffee, please?"),
                    ChatCompletionMessage(role="assistant", content="{\"response\": \"Sure, I'll make you a coffee.\", \"emotion\": \"Happy\"}"),
                    ChatCompletionMessage(role="user", content=user_prompt)
                ],
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
                    self.get_logger().error("Invalid JSON response from Atoma")
                    return response

                # Extract the response and emotion
                out_response = response_json.get('response')
                emotion = response_json.get('emotion')

                # Save the response and emotion to the response
                response.response = out_response
                response.emotion = emotion

                return response
            except json.JSONDecodeError:
                self.get_logger().error("Invalid JSON response from Atoma")
                return response
        except Exception as e:
            self.get_logger().error(f"Error in behavior response generation: {e}")
            return response
            
    def atoma_chat_callback(self, request, response):
        """Handle Atoma chat service requests."""
        try:
            self.get_logger().info(f"Received chat request with prompt: {request.prompt}")
            
            # Add user message to conversation history
            self.conversation_history.append({"role": "user", "content": request.prompt})
            
            self.get_logger().info(f"Current conversation history length: {len(self.conversation_history)}")
            
            # Trim conversation history if too long
            if len(self.conversation_history) > 13:
                self.conversation_history = self.conversation_history[:1] + self.conversation_history[-12:]
            
            # Convert conversation history to ChatCompletionMessage objects
            messages = [ChatCompletionMessage(**msg) for msg in self.conversation_history]
            
            # Make API request to Atoma
            self.get_logger().info(f"Making Atoma API request with model: {self.model}")
            try:
                api_response = self.atoma_client.chat.create(
                    messages=messages,
                    model=self.model,
                    temperature=0.7,
                    max_tokens=100
                )
                self.get_logger().info("Atoma API request successful")
            except Exception as api_error:
                self.get_logger().error(f"Atoma API request failed with error: {str(api_error)}")
                raise
            
            # Get response text
            response_text = api_response.choices[0].message.content.strip()
            
            # Add assistant response to conversation history
            self.conversation_history.append({"role": "assistant", "content": response_text})
            
            # Set response
            response.response = response_text
            response.success = True
            response.error = ""
            
            self.get_logger().info(f"Chat response generated: {response_text}")
            
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