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

        # Declare parameters
        self.declare_parameter('voice_id', "KTPVrSVAEUSJRClDzBw7")  # Default voice ID
        self.declare_parameter('model_id', "eleven_multilingual_v2")  # Default model
        self.declare_parameter('api_key', '')  # Empty by default, will check env var
        self.declare_parameter('cooldown_duration', 1.0)  # Cooldown in seconds
        self.declare_parameter('output_format', 'pcm_24000')  # Audio output format

        # Get parameters
        self.voice_id = self.get_parameter('voice_id').value
        self.model_id = self.get_parameter('model_id').value
        self.COOLDOWN_DURATION = self.get_parameter('cooldown_duration').value
        self.output_format = self.get_parameter('output_format').value

        self.service_group = MutuallyExclusiveCallbackGroup()
        self.timer_group = MutuallyExclusiveCallbackGroup()
        
        # Cooldown settings
        self.cooldown_timer = None
        self.in_cooldown = False

        # Initialize Eleven Labs SDK client
        api_key = self.get_parameter('api_key').value
        if not api_key:
            api_key = os.environ.get('ELEVEN_LABS_API_KEY')
        
        if not api_key:
            self.get_logger().error("ELEVEN_LABS_API_KEY not set in parameter or environment variable")
            raise ValueError("ELEVEN_LABS_API_KEY is required. Set it as a ROS2 parameter or environment variable. Refer to the README for more information")
        
        self.eleven_labs_client = ElevenLabs(api_key=api_key)

        self.get_logger().info(f"Eleven Labs SDK initialized with voice: {self.voice_id}, model: {self.model_id}")

        self.audio_player = pyaudio.PyAudio()

        self.get_logger().info("PyAudio initialized")

        # Create a service for the TTS queries
        self.create_service(
            TTSQuery,
            TTS_SERVICE,
            self.tts_query_callback,
            callback_group=self.service_group
        )

        # Publisher for general status
        self.status_pub = self.create_publisher(
            String, TTS_STATUS_TOPIC, 10)
            
        # Publisher for audio playback state
        self.audio_state_pub = self.create_publisher(
            String, 'tts/audio_state', 10)

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
            "health": "ok",
            "voice_id": self.voice_id,
            "model_id": self.model_id,
            "is_playing": self.is_playing,
            "in_cooldown": self.in_cooldown
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

    def tts_query_callback(self, request, response):
        """Handle TTS query service requests."""
        try:
            # Check if we're already playing audio or in cooldown
            if self.is_playing or self.in_cooldown:
                state = "playing" if self.is_playing else "cooling down"
                self.get_logger().warn(f"TTS is {state}, request rejected")
                response.success = False
                return response

            text = request.text
            self.get_logger().info(f"Streaming audio for text: {text}")
            
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
        stream = None
        try:
            with self.stream_lock:
                self.is_playing = True
                
                # Request PCM audio format (raw audio data)
                # pcm_16000 = 16kHz sample rate, pcm_24000 = 24kHz sample rate
                output_format = self.output_format
                
                self.get_logger().info(f"Starting audio streaming with format: {output_format}")
                
                # Publish audio start state
                msg = String()
                msg.data = 'playing'
                self.audio_state_pub.publish(msg)
                
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
                    if not self.is_playing:  # Check if we should stop playing
                        break
                    if chunk:  # Skip empty chunks
                        # For PCM format, we can write directly to the stream
                        stream.write(chunk)
                
        except Exception as e:
            self.get_logger().error(f"Error in audio streaming: {str(e)}")
        finally:
            # Always clean up resources and update state
            if stream:
                try:
                    stream.stop_stream()
                    stream.close()
                except Exception as e:
                    self.get_logger().error(f"Error closing audio stream: {str(e)}")
            
            # Publish completion state
            try:
                msg = String()
                msg.data = 'done'
                self.audio_state_pub.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Error publishing completion state: {str(e)}")
            
            # Start cooldown period
            self.in_cooldown = True
            msg = String()
            msg.data = 'cooldown'
            self.audio_state_pub.publish(msg)
            
            if self.cooldown_timer:
                self.cooldown_timer.cancel()
            self.cooldown_timer = self.create_timer(
                self.COOLDOWN_DURATION,
                self._end_cooldown,
                callback_group=self.timer_group
            )
            
            self.get_logger().info(f"Starting {self.COOLDOWN_DURATION}s cooldown")
            self.is_playing = False
            
            self.get_logger().info("Audio playback completed")

    def _end_cooldown(self):
        """End the cooldown period and signal that the system is ready."""
        self.in_cooldown = False
        if self.cooldown_timer:
            self.cooldown_timer.cancel()
            self.cooldown_timer = None
        
        # Publish completion state
        msg = String()
        msg.data = 'done'
        self.audio_state_pub.publish(msg)
        
        self.get_logger().info("Cooldown complete, system ready")


def main(args=None):
    rclpy.init(args=args)
    tts_node = TTSNode()
    rclpy.spin(tts_node)
    tts_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()