#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from shared_configs import (
    VOICE_INTENT_TOPIC,
    EMOTION_TOPIC,
    FACE_POSITION_TOPIC,
    LANGUAGE_MODEL_PROCESSOR_TOPIC,
    STATE_MANAGER_TOPIC,
    INTENT_MAPPING
)
from coffee_buddy_msgs.msg import IntentClassification
from atoma_sdk import AtomaSDK
from atoma_sdk.models import ChatCompletionMessage

class LanguageModelProcessorNode(Node):
    def __init__(self):
        super().__init__('language_model_processor_node')
        self.get_logger().info("Language model processor node initialized")

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

        # Create a subscriber for the voice intent topic
        self.voice_intent_subscriber = self.create_subscription(
            IntentClassification,
            VOICE_INTENT_TOPIC,
            self.voice_intent_callback,
            10
        )

        # Create a publisher for the language model responses
        self.language_model_publisher = self.create_publisher(
            String,
            LANGUAGE_MODEL_PROCESSOR_TOPIC,
            10
        )

    def voice_intent_callback(self, msg: IntentClassification):
        # Extract the user's prompt and intent
        prompt_text = msg.prompt_text
        intent_code = msg.intent
        
        # Get the intent name
        intent_name = INTENT_MAPPING.get(intent_code, "Fallback")
        
        self.get_logger().info(f"Received voice intent: {intent_name} - '{prompt_text}'")
        
        try:
            # Create system prompt with BrewBot information
            system_prompt = (
                "You are BrewBot, a friendly coffee robot. You're warm, conversational, and curious about people. "
                "Your responses should be natural, brief (1-2 sentences max), and conversational. "
                "You're interested in how people are enjoying the conference, which talks they've found interesting, "
                "and their thoughts on blockchain technology. "
                "Keep your responses concise and natural – like a brief, friendly exchange between people. "
                "Avoid long explanations or formal language. Talk like a person, not an assistant. "
                "Never mention that you're a robot or AI - just have a natural conversation. "
                "Don't end every response with a question. "
                "Keep your responses brief (1-2 sentences maximum)."
            )
            
            # Create user prompt with context
            user_prompt = f"Intent: {intent_name}\nUser said: {prompt_text}\n\nRespond briefly:"
            
            # Make API request to Atoma
            response = self.atoma_client.chat.create(
                model=self.model,
                messages=[
                    ChatCompletionMessage(role="system", content=system_prompt),
                    ChatCompletionMessage(role="user", content=user_prompt)
                ],
                max_tokens=100,
                temperature=0.7
            )
            
            # Extract the response text
            response_text = response.choices[0].message.content.strip()
            self.get_logger().info(f"LLM Response: {response_text}")
            
            # Publish the response
            msg = String()
            msg.data = response_text
            self.language_model_publisher.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error getting LLM response: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    language_model_processor_node = LanguageModelProcessorNode()
    rclpy.spin(language_model_processor_node)
    language_model_processor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()  