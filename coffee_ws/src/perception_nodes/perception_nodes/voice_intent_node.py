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
import re
import gc
import queue
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

# Conditionally import torch for GPU memory management
try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False


class VoiceIntentNode(Node):
    """
    ROS2 node that listens to microphone input and publishes transcribed speech using Whisper.
    """
    
    def __init__(self):
        super().__init__('voice_intent_node')
        
        self.get_logger().info("Initializing VoiceIntentNode")
        
        # Declare parameters
        self.declare_parameter(
            'model_size', 
            'base', 
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Whisper model size (tiny, base, small, medium, large, turbo)'
            )
        )
        self.declare_parameter(
            'language', 
            'en', 
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Language for transcription (en, auto, etc.)'
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
            'int8', 
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Compute type used for inference (float16, int8_float16, int8, float32)'
            )
        )
        self.declare_parameter(
            'timeout_segments', 
            5, 
            ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description='Number of segments to process before timeout and publishing anyway'
            )
        )
        self.declare_parameter(
            'audio_device_id', 
            0, 
            ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description='Audio input device ID (default: 0 for system default)'
            )
        )
        self.declare_parameter(
            'verbose_logging', 
            True, 
            ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description='Enable verbose logging of all transcriptions'
            )
        )
        self.declare_parameter(
            'gpu_memory_monitoring', 
            True, 
            ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description='Enable GPU memory usage monitoring'
            )
        )
        self.declare_parameter(
            'memory_cleanup_interval', 
            10, 
            ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description='Number of inference cycles between memory cleanup operations'
            )
        )
        
        # Get parameters
        self.model_size = self.get_parameter('model_size').value
        self.language = self.get_parameter('language').value
        self.device_type = self.get_parameter('device_type').value
        self.compute_type = self.get_parameter('compute_type').value
        self.timeout_segments = self.get_parameter('timeout_segments').value
        self.audio_device_id = self.get_parameter('audio_device_id').value
        self.verbose_logging = self.get_parameter('verbose_logging').value
        self.gpu_memory_monitoring = self.get_parameter('gpu_memory_monitoring').value
        self.memory_cleanup_interval = self.get_parameter('memory_cleanup_interval').value
        
        self.get_logger().info(f"Parameters loaded: model_size={self.model_size}, language={self.language}, " +
                              f"device_type={self.device_type}, compute_type={self.compute_type}, " +
                              f"timeout_segments={self.timeout_segments}, audio_device_id={self.audio_device_id}, " +
                              f"verbose_logging={self.verbose_logging}, gpu_memory_monitoring={self.gpu_memory_monitoring}")
        
        # Create publisher
        self.publisher = self.create_publisher(String, '/voice/intent', 10)
        
        # Check for CUDA availability if device_type is cuda
        self.using_gpu = self.device_type == 'cuda' and TORCH_AVAILABLE
        if self.using_gpu:
            if torch.cuda.is_available():
                self.get_logger().info(f"CUDA is available, using GPU: {torch.cuda.get_device_name(0)}")
                # Initialize memory tracking
                self.inference_count = 0
                self.log_gpu_memory_usage("Initial GPU state")
            else:
                self.get_logger().warn("CUDA requested but not available, falling back to CPU")
                self.device_type = 'cpu'
                self.using_gpu = False
        
        # PyAudio configuration
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 16000  # Whisper requires 16kHz
        self.CHUNK = 1024
        
        # Wake word and buffer variables
        self.wake_phrase = "buddy"
        self.text_buffer = ""
        self.is_accumulating = False
        self.segment_count = 0
        
        # Audio processing queue
        self.audio_queue = queue.Queue(maxsize=100)  # Maximum 100 chunks in queue
        
        self.get_logger().info(f"Wake phrase detection initialized with phrase: '{self.wake_phrase}'")
        self.get_logger().info(f"Timeout set to {self.timeout_segments} segments")
        
        # Initialize PyAudio and check available devices
        try:
            self.audio = pyaudio.PyAudio()
            self.get_logger().info("PyAudio initialized successfully")
            self.list_audio_devices()
        except Exception as e:
            self.get_logger().error(f"Failed to initialize PyAudio: {str(e)}")
            raise
        
        # Check if Whisper model exists, download if needed
        self.get_logger().info(f'Checking if Whisper model {self.model_size} is downloaded...')
        self.ensure_model_downloaded()
        
        # Initialize ASR
        try:
            self.init_asr()
            self.get_logger().info("ASR initialization completed successfully")
        except Exception as e:
            self.get_logger().error(f"Error during ASR initialization: {str(e)}")
            raise
        
        # Create and start the threads
        self.running = True
        self.audio_thread = threading.Thread(target=self.capture_audio)
        self.audio_thread.daemon = True
        self.audio_thread.start()
        
        self.inference_thread = threading.Thread(target=self.process_audio)
        self.inference_thread.daemon = True
        self.inference_thread.start()
        
        self.get_logger().info('Voice Intent Node started successfully')
    
    def log_gpu_memory_usage(self, label=""):
        """Log current GPU memory usage if monitoring is enabled and GPU is being used."""
        if not (self.gpu_memory_monitoring and self.using_gpu):
            return
        
        try:
            # Get current GPU memory allocation
            allocated = torch.cuda.memory_allocated() / (1024 * 1024)  # Convert to MB
            reserved = torch.cuda.memory_reserved() / (1024 * 1024)    # Convert to MB
            self.get_logger().info(f"GPU Memory [{label}] - Allocated: {allocated:.2f} MB, Reserved: {reserved:.2f} MB")
        except Exception as e:
            self.get_logger().warn(f"Error monitoring GPU memory: {str(e)}")
    
    def cleanup_memory(self, force=False):
        """Perform memory cleanup operations."""
        if self.using_gpu:
            self.inference_count += 1
            
            # Only clean up periodically to avoid overhead
            if force or (self.inference_count % self.memory_cleanup_interval == 0):
                self.get_logger().debug("Performing memory cleanup")
                
                # Python garbage collection
                gc.collect()
                
                # CUDA memory cache cleanup
                if TORCH_AVAILABLE and torch.cuda.is_available():
                    torch.cuda.empty_cache()
                
                # Log memory usage after cleanup
                if force:
                    self.log_gpu_memory_usage("After forced cleanup")
                else:
                    self.log_gpu_memory_usage("After periodic cleanup")
    
    def list_audio_devices(self):
        """List available audio input devices and validate the chosen device ID."""
        info = self.audio.get_host_api_info_by_index(0)
        num_devices = info.get('deviceCount')
        
        # List all input devices
        input_devices = []
        default_device_id = None
        
        for i in range(num_devices):
            device_info = self.audio.get_device_info_by_index(i)
            if device_info.get('maxInputChannels') > 0:
                device_name = device_info.get('name')
                input_devices.append((i, device_name))
                self.get_logger().info(f"Input Device {i}: {device_name}")
                
                # Check for exact match with "default" name
                if device_name == "default":
                    default_device_id = i
                    self.get_logger().info(f"Found device with exact name 'default' (ID: {i})")
        
        # Check if selected device is valid
        valid_ids = [dev[0] for dev in input_devices]
        if not input_devices:
            self.get_logger().error("No input devices found!")
            raise RuntimeError("No audio input devices available")
        
        # If a device with name "default" was found, use it
        if default_device_id is not None:
            self.get_logger().info(f"Setting audio device to 'default' (ID: {default_device_id})")
            self.audio_device_id = default_device_id
        # Otherwise check if specified ID is valid
        elif self.audio_device_id not in valid_ids:
            self.get_logger().warn(f"Specified audio device ID {self.audio_device_id} not found. Using default device (ID: {valid_ids[0]})")
            self.audio_device_id = valid_ids[0]
        
        self.get_logger().info(f"Using audio input device: ID {self.audio_device_id}")
    
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
        
        self.get_logger().info("Starting ASR initialization")
        
        # Log initial GPU state if using GPU
        self.log_gpu_memory_usage("Before ASR initialization")
        
        # Create parser with Whisper options
        parser = argparse.ArgumentParser()
        add_shared_args(parser)
        
        # Use parse_known_args instead of parse_args to ignore ROS2 arguments
        args, _ = parser.parse_known_args()
        
        # Override parameters with ROS2 parameters
        args.model = self.model_size
        args.lan = self.language
        args.min_chunk_size = 2.0  # Process at least 2 seconds of audio
        args.buffer_trimming_sec = 30  # Buffer trimming in seconds
        args.vad = True  # Voice activity detection
        args.backend = 'faster-whisper'  # Use faster-whisper for better performance
        args.buffer_trimming = 'sentence'  # Trim at sentence boundaries
        
        # Add device_type and compute_type for WhisperModel initialization
        args.device_type = self.device_type
        args.compute_type = self.compute_type
        
        self.get_logger().info(f'Initializing Whisper ASR with model {self.model_size} and language {self.language}')
        self.get_logger().info(f'Using device type: {self.device_type}, compute type: {self.compute_type}')
        self.get_logger().info(f'Using min_chunk_size={args.min_chunk_size}s, buffer_trimming_sec={args.buffer_trimming_sec}s')
        
        # Initialize ASR components
        try:
            self.get_logger().info("Creating ASR factory")
            self.asr, self.processor = asr_factory(args)
            self.processor.buffer_trimming_way = 'sentence'  # Ensure sentence-based trimming
            self.get_logger().info("ASR factory created successfully")
            self.log_gpu_memory_usage("After ASR factory creation")
        except Exception as e:
            self.get_logger().error(f"Error creating ASR factory: {str(e)}")
            raise
        
        # Warm up the model with a small audio chunk to make first inference faster
        try:
            self.get_logger().info("Warming up the ASR model")
            dummy_audio = np.zeros(1600, dtype=np.float32)  # 0.1 seconds of silence
            self.asr.transcribe(dummy_audio)
            self.processor.init()
            self.get_logger().info("ASR model warm-up completed")
            self.log_gpu_memory_usage("After model warm-up")
        except Exception as e:
            self.get_logger().error(f"Error during ASR warm-up: {str(e)}")
            raise
    
    def contains_wake_phrase(self, text):
        """Check if the text contains the wake phrase (case-insensitive)."""
        result = self.wake_phrase.lower() in text.lower()
        if result and self.verbose_logging:
            self.get_logger().info(f"Wake phrase '{self.wake_phrase}' found in text: '{text}'")
        return result
    
    def contains_end_punctuation(self, text):
        """Check if the text contains end punctuation (., !, ?)."""
        result = bool(re.search(r'[.!?]', text))
        if result and self.verbose_logging:
            self.get_logger().info(f"End punctuation found in text: '{text}'")
        return result
    
    def extract_content_with_wake_phrase(self, text):
        """
        Extract content starting from the wake phrase and ending at the first end punctuation.
        Returns the extracted text and whether an end punctuation was found.
        """
        # Find wake phrase position (case-insensitive)
        wake_phrase_pos = text.lower().find(self.wake_phrase.lower())
        if wake_phrase_pos == -1:
            return text, False
        
        # Extract text starting from wake phrase
        content = text[wake_phrase_pos:]
        
        # Find first end punctuation
        match = re.search(r'[.!?]', content)
        if match:
            # If found, cut off everything after the punctuation
            punctuation_pos = match.start()
            return content[:punctuation_pos+1], True
        
        return content, False
    
    def process_transcription(self, transcription):
        """Process the transcribed text to handle wake phrase and end conditions."""
        if transcription is None or len(transcription.strip()) == 0:
            return
        
        if self.verbose_logging:
            self.get_logger().info(f"Processing transcription: '{transcription}'")
        
        if not self.is_accumulating:
            # Check for wake phrase
            if self.contains_wake_phrase(transcription):
                # Extract content starting from wake phrase
                extracted, has_punctuation = self.extract_content_with_wake_phrase(transcription)
                self.is_accumulating = True
                self.text_buffer = extracted
                self.segment_count = 1
                if self.verbose_logging:
                    self.get_logger().info(f"Wake phrase detected! Starting to accumulate text: '{extracted}'")
                
                # If this segment already has end punctuation, publish immediately
                if has_punctuation:
                    self.publish_buffer("End punctuation detected in initial segment")
            elif self.verbose_logging:
                self.get_logger().debug("No wake phrase detected, ignoring transcription")
        else:
            # Already accumulating text
            self.segment_count += 1
            
            # Add new text to buffer and check for end punctuation
            combined_text = self.text_buffer + " " + transcription
            extracted, has_punctuation = self.extract_content_with_wake_phrase(combined_text)
            self.text_buffer = extracted
            if self.verbose_logging:
                self.get_logger().info(f"Added segment {self.segment_count} to buffer. Current buffer: '{self.text_buffer}'")
            
            # Check for end punctuation or timeout
            if has_punctuation:
                if self.verbose_logging:
                    self.get_logger().info("End punctuation detected, publishing accumulated text")
                self.publish_buffer("End punctuation detected")
            elif self.segment_count >= self.timeout_segments:
                if self.verbose_logging:
                    self.get_logger().info(f"Segment timeout ({self.timeout_segments}) reached, publishing accumulated text")
                self.publish_buffer("Timeout threshold reached")
    
    def publish_buffer(self, reason):
        """Publish the accumulated text buffer and reset state."""
        try:
            msg = String()
            msg.data = self.text_buffer.strip()
            self.publisher.publish(msg)
            self.get_logger().info(f"Published intent: '{msg.data}' ({reason})")
            
            # Reset state
            self.is_accumulating = False
            self.text_buffer = ""
            self.segment_count = 0
            if self.verbose_logging:
                self.get_logger().debug("Buffer state reset, waiting for next wake phrase")
        except Exception as e:
            self.get_logger().error(f"Error publishing message: {str(e)}")
    
    def capture_audio(self):
        """Thread for capturing audio from microphone and adding to processing queue."""
        try:
            self.get_logger().info("Starting audio capture thread")
            # Open audio stream
            try:
                stream = self.audio.open(
                    format=self.FORMAT,
                    channels=self.CHANNELS,
                    rate=self.RATE,
                    input=True,
                    input_device_index=self.audio_device_id,
                    frames_per_buffer=self.CHUNK
                )
                self.get_logger().info(f'Started audio stream from device ID {self.audio_device_id} successfully')
            except Exception as e:
                self.get_logger().error(f"Failed to open audio stream: {str(e)}")
                return
            
            audio_buffer = np.array([], dtype=np.float32)
            
            self.get_logger().info("Entering audio capture loop")
            while self.running:
                try:
                    # Read audio data
                    data = stream.read(self.CHUNK, exception_on_overflow=False)
                    
                    # Convert data to numpy array (int16) - this stays on CPU
                    audio_data = np.frombuffer(data, dtype=np.int16)
                    
                    # Convert to float32 and normalize to [-1, 1] - still on CPU
                    audio_float = audio_data.astype(np.float32) / 32768.0
                    
                    # Add to the buffer
                    audio_buffer = np.concatenate((audio_buffer, audio_float))
                    
                    # If buffer is large enough, add to queue for processing
                    if len(audio_buffer) >= self.RATE:
                        try:
                            # Try to add to queue with timeout to avoid blocking indefinitely
                            self.audio_queue.put(audio_buffer.copy(), timeout=0.1)
                            
                            # Reset buffer after adding to queue
                            audio_buffer = np.array([], dtype=np.float32)
                        except queue.Full:
                            self.get_logger().warn("Audio processing queue is full, dropping chunk")
                            # Still reset buffer to avoid growing indefinitely
                            audio_buffer = np.array([], dtype=np.float32)
                    
                except Exception as e:
                    self.get_logger().error(f'Error capturing audio: {str(e)}')
                    time.sleep(0.1)  # Small delay to avoid CPU spike on continuous errors
            
            # Clean up
            self.get_logger().info("Shutting down audio capture")
            stream.stop_stream()
            stream.close()
            
        except Exception as e:
            self.get_logger().error(f'Error in audio capture thread: {str(e)}')
    
    def process_audio(self):
        """Thread for processing audio chunks and running inference."""
        try:
            self.get_logger().info("Starting audio processing thread")
            
            # Initialize the ASR processor in this thread
            try:
                self.processor.init()
                self.get_logger().info("ASR processor initialized in processing thread")
            except Exception as e:
                self.get_logger().error(f"Failed to initialize ASR processor: {str(e)}")
                return
            
            self.get_logger().info("Entering audio processing loop")
            while self.running:
                try:
                    # Get audio chunk from queue with timeout
                    try:
                        audio_chunk = self.audio_queue.get(timeout=0.5)
                    except queue.Empty:
                        continue  # No audio to process, try again
                    
                    # Log GPU memory before inference if monitoring is enabled
                    self.log_gpu_memory_usage("Before inference")
                    
                    # Insert audio into processor - data stays on CPU until needed by model
                    self.processor.insert_audio_chunk(audio_chunk)
                    
                    # Process the current buffer - inference happens here, using GPU if available
                    result = self.processor.process_iter()
                    
                    # Memory cleanup and logging after inference
                    self.cleanup_memory(force=False)
                    self.log_gpu_memory_usage("After inference")
                    
                    # Mark task as done
                    self.audio_queue.task_done()
                    
                    # Check if we got a transcription result
                    if result[0] is not None and result[2]:
                        transcription = result[2]
                        if self.verbose_logging:
                            self.get_logger().info(f'Transcription received: {transcription}')
                        
                        # Process the transcription with wake word logic
                        self.process_transcription(transcription)
                
                except Exception as e:
                    self.get_logger().error(f'Error processing audio: {str(e)}')
                    time.sleep(0.1)  # Small delay to avoid CPU spike on continuous errors
            
        except Exception as e:
            self.get_logger().error(f'Error in audio processing thread: {str(e)}')
    
    def destroy_node(self):
        """Clean up resources when the node is destroyed."""
        self.get_logger().info("Shutting down VoiceIntentNode")
        self.running = False
        
        # Wait for threads to terminate
        if hasattr(self, 'audio_thread') and self.audio_thread.is_alive():
            self.get_logger().info("Waiting for audio capture thread to terminate")
            self.audio_thread.join(timeout=1.0)
        
        if hasattr(self, 'inference_thread') and self.inference_thread.is_alive():
            self.get_logger().info("Waiting for inference thread to terminate")
            self.inference_thread.join(timeout=1.0)
        
        # Perform final cleanup
        if self.using_gpu:
            self.get_logger().info("Cleaning up GPU resources")
            self.cleanup_memory(force=True)
        
        if hasattr(self, 'audio') and self.audio:
            self.get_logger().info("Terminating PyAudio")
            self.audio.terminate()
        
        self.get_logger().info("VoiceIntentNode shutdown complete")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = VoiceIntentNode()
        print("VoiceIntentNode created successfully, starting spin")
    except Exception as e:
        print(f"Error creating VoiceIntentNode: {str(e)}")
        rclpy.shutdown()
        return
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard interrupt received")
    except Exception as e:
        print(f"Error during node execution: {str(e)}")
    finally:
        print("Destroying node")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 