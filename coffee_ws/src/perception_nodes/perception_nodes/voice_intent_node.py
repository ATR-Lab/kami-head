#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyaudio
import numpy as np
import os
import time
import threading
import sys
from pathlib import Path
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# Import whisper streaming components
from perception_nodes.whisper_streaming import (
    asr_factory,
    load_audio_chunk,
    OnlineASRProcessor,
    add_shared_args
)


class VoiceIntentNode(Node):
    """
    ROS2 node that listens to microphone input and publishes transcribed speech using Whisper.
    """
    
    def __init__(self):
        super().__init__('voice_intent_node')
        
        # Declare parameters
        self.declare_parameter(
            'model_size', 
            'base', 
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Whisper model size (tiny, base, small, medium, large)'
            )
        )
        self.declare_parameter(
            'language', 
            'english', 
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Language for transcription (english, auto, etc.)'
            )
        )
        self.declare_parameter(
            'device_type', 
            'cuda', 
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Device type used for inference (cuda, cpu, mps)'
            )
        )
        self.declare_parameter(
            'compute_type', 
            'int8_float16', 
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Compute type used for inference (float16, int8_float16, int8)'
            )
        )
        
        # Get parameters
        self.model_size = self.get_parameter('model_size').value
        self.language = self.get_parameter('language').value
        self.device_type = self.get_parameter('device_type').value
        self.compute_type = self.get_parameter('compute_type').value
        
        # Create publisher
        self.publisher = self.create_publisher(String, '/voice/intent', 10)
        
        # PyAudio configuration
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 16000  # Whisper requires 16kHz
        self.CHUNK = 1024
        self.DEVICE_ID = 20  # Default microphone input device ID
        
        # Initialize PyAudio
        self.audio = pyaudio.PyAudio()
        
        # Check if Whisper model exists, download if needed
        self.get_logger().info(f'Checking if Whisper model {self.model_size} is downloaded...')
        self.ensure_model_downloaded()
        
        # Initialize ASR
        self.init_asr()
        
        # Create and start the stream thread
        self.running = True
        self.thread = threading.Thread(target=self.process_audio)
        self.thread.daemon = True
        self.thread.start()
        
        self.get_logger().info('Voice Intent Node started')
    
    def ensure_model_downloaded(self):
        """Check if the Whisper model is downloaded, and download it if not."""
        # Whisper models are typically stored in ~/.cache/whisper
        cache_dir = os.path.expanduser("~/.cache/whisper")
        model_path = os.path.join(cache_dir, f"{self.model_size}.pt")
        
        if os.path.exists(model_path):
            self.get_logger().info(f'Whisper model {self.model_size} found at {model_path}')
            return True
        
        # Model not found, we need to trigger a download
        self.get_logger().info(f'Whisper model {self.model_size} not found, downloading...')
        try:
            # This import and function call will trigger the download
            import whisper
            whisper.load_model(self.model_size)
            self.get_logger().info(f'Successfully downloaded Whisper model {self.model_size}')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to download Whisper model: {str(e)}')
            return False
    
    def init_asr(self):
        """Initialize the ASR processor."""
        import argparse
        
        # Create parser with Whisper options
        parser = argparse.ArgumentParser()
        add_shared_args(parser)
        
        # Use parse_known_args instead of parse_args to ignore ROS2 arguments
        args, _ = parser.parse_known_args()
        
        # Override parameters with ROS2 parameters
        args.model = self.model_size
        args.lan = self.language
        args.min_chunk_size = 1.0  # Process at least 1 second of audio
        args.buffer_trimming_sec = 15  # Buffer trimming in seconds
        args.vad = True  # Use voice activity detection
        args.backend = 'faster-whisper'  # Use faster-whisper for better performance
        
        # Add device_type and compute_type for WhisperModel initialization
        args.device_type = self.device_type
        args.compute_type = self.compute_type
        
        self.get_logger().info(f'Initializing Whisper ASR with model {self.model_size} and language {self.language}')
        self.get_logger().info(f'Using device type: {self.device_type}, compute type: {self.compute_type}')
        
        # Initialize ASR components
        self.asr, self.processor = asr_factory(args)
        
        # Warm up the model with a small audio chunk to make first inference faster
        dummy_audio = np.zeros(1600, dtype=np.float32)  # 0.1 seconds of silence
        self.asr.transcribe(dummy_audio)
        self.processor.init()
        
    def process_audio(self):
        """Process audio stream from microphone and publish transcriptions."""
        try:
            # Open audio stream
            stream = self.audio.open(
                format=self.FORMAT,
                channels=self.CHANNELS,
                rate=self.RATE,
                input=True,
                input_device_index=self.DEVICE_ID,
                frames_per_buffer=self.CHUNK
            )
            
            self.get_logger().info(f'Started audio stream from device ID {self.DEVICE_ID}')
            
            # Initialize the ASR processor
            self.processor.init()
            
            audio_buffer = np.array([], dtype=np.float32)
            
            while self.running:
                try:
                    # Read audio data
                    data = stream.read(self.CHUNK, exception_on_overflow=False)
                    
                    # Convert data to numpy array (int16)
                    audio_data = np.frombuffer(data, dtype=np.int16)
                    
                    # Convert to float32 and normalize to [-1, 1]
                    audio_float = audio_data.astype(np.float32) / 32768.0
                    
                    # Add to the buffer
                    audio_buffer = np.concatenate((audio_buffer, audio_float))
                    
                    # Process if buffer is large enough (at least 1 second)
                    if len(audio_buffer) >= self.RATE:
                        # Insert audio into processor
                        self.processor.insert_audio_chunk(audio_buffer)
                        
                        # Process the current buffer
                        result = self.processor.process_iter()
                        
                        # Reset buffer after processing
                        audio_buffer = np.array([], dtype=np.float32)
                        
                        # Check if we got a transcription result
                        if result[0] is not None and result[2]:
                            # Create message and publish
                            msg = String()
                            msg.data = result[2]
                            self.publisher.publish(msg)
                            self.get_logger().info(f'Published transcription: {msg.data}')
                    
                except Exception as e:
                    self.get_logger().error(f'Error processing audio: {str(e)}')
                    time.sleep(0.1)  # Small delay to avoid CPU spike on continuous errors
            
            # Clean up
            stream.stop_stream()
            stream.close()
            
        except Exception as e:
            self.get_logger().error(f'Error initializing audio stream: {str(e)}')
    
    def destroy_node(self):
        """Clean up resources when the node is destroyed."""
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.audio:
            self.audio.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceIntentNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 