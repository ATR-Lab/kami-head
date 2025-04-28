#!/usr/bin/env python3

import rclpy
import os
import tempfile
import threading
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from coffee_buddy_msgs.srv import TTSQuery
from shared_configs import TTS_SERVICE, TTS_STATUS_TOPIC
import pyaudio
import wave
import json
from elevenlabs.client import ElevenLabs

class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')
        self.get_logger().info("TTS node initialized")

        self.service_group = MutuallyExclusiveCallbackGroup()
        self.timer_group = MutuallyExclusiveCallbackGroup()

        # Initialize Eleven Labs SDK client
        api_key = os.environ.get('ELEVEN_LABS_API_KEY')
        if not api_key:
            self.get_logger().error("ELEVEN_LABS_API_KEY environment variable not set")
            raise ValueError("ELEVEN_LABS_API_KEY environment variable is required, refer to the README for more information")
        
        self.eleven_labs_client = ElevenLabs(api_key=api_key)

        self.voice_id = "KTPVrSVAEUSJRClDzBw7" # https://elevenlabs.io/app/voice-library/collections/HXn5AetPOJgAHd2D60mP?voiceId=KTPVrSVAEUSJRClDzBw7
        self.model_id = "eleven_multilingual_v2"

        self.get_logger().info("Eleven Labs SDK initialized")

        self.audio_player = pyaudio.PyAudio()

        self.get_logger().info("PyAudio initialized")

        # Create a service for the TTS queries
        self.create_service(
            TTSQuery,
            TTS_SERVICE,
            self.tts_query_callback,
            callback_group=self.service_group
        )

        self.status_pub = self.create_publisher(
            String, TTS_STATUS_TOPIC, 10)

        # Status update timer (1Hz)
        self.status_timer = self.create_timer(
            1.0, self.publish_status, callback_group=self.timer_group)
        
        # Add a lock for thread safety
        self.stream_lock = threading.Lock()
        
        # Flag to track if audio is currently playing
        self.is_playing = False
        
        self.get_logger().info("TTS node initialized and ready")

    def publish_status(self):
        """Publish current status information"""
        status = {
            "health": "ok"
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

    def tts_query_callback(self, request, response):
        """
        Generate a behavior response based on the user's intent and prompt.
        """
        text = request.text
        self.get_logger().info(f"Streaming audio for text: {text}")
        
        try:
            # Start a new thread for audio streaming and playback
            playback_thread = threading.Thread(
                target=self.stream_audio_playback,
                args=(text,)
            )
            playback_thread.daemon = True
            playback_thread.start()
            
            response.success = True
            
        except Exception as e:
            self.get_logger().error(f"Error in TTS processing: {str(e)}")
            response.success = False
            
        return response

    def stream_audio_playback(self, text):
        """Stream audio directly from ElevenLabs API to PyAudio"""
        try:
            with self.stream_lock:
                self.is_playing = True
                
                # Request PCM audio format (raw audio data)
                # pcm_16000 = 16kHz sample rate, pcm_24000 = 24kHz sample rate
                output_format = "pcm_24000"
                
                self.get_logger().info(f"Starting audio streaming with format: {output_format}")
                
                # Set up audio buffer for processing
                audio_buffer = bytearray()
                
                # Generate and stream audio
                audio_stream = self.eleven_labs_client.generate(
                    text=text,
                    voice=self.voice_id,
                    model=self.model_id,
                    output_format=output_format,
                    stream=True  # Enable streaming mode
                )
                
                # Configure PyAudio based on the PCM format
                sample_rate = int(output_format.split('_')[1])  # Extract sample rate from format
                stream = self.audio_player.open(
                    format=pyaudio.paInt16,  # PCM 16-bit
                    channels=1,              # Mono
                    rate=sample_rate,        # Sample rate from the format
                    output=True
                )
                
                self.get_logger().info(f"Audio stream opened with sample rate: {sample_rate}Hz")
                
                # Process and play each chunk as it arrives
                for chunk in audio_stream:
                    if chunk:  # Skip empty chunks
                        # For PCM format, we can write directly to the stream
                        stream.write(chunk)
                
                # Clean up
                stream.stop_stream()
                stream.close()
                self.is_playing = False
                self.get_logger().info("Audio streaming completed")
                
        except Exception as e:
            self.get_logger().error(f"Error in audio streaming: {str(e)}")
            self.is_playing = False


def main(args=None):
    rclpy.init(args=args)
    tts_node = TTSNode()
    rclpy.spin(tts_node)
    tts_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()