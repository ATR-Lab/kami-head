#!/usr/bin/env python3

"""
ROS2 node for voice intent recognition using Whisper ASR and LLM classification.

This node captures audio from a microphone, performs speech recognition
using the Whisper ASR model, detects wake phrases, and classifies
the intent of the user's speech using a local LLM through Ollama.
"""

import rclpy
from rclpy.node import Node
import queue
import threading
import logging
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import time
import json
import noisereduce as nr

# Import ROS2 message types
from coffee_buddy_msgs.msg import IntentClassification

# Import our modules
from coffee_speech_processing.voice_intent_node.audio_processor import AudioProcessor
from coffee_speech_processing.voice_intent_node.asr_manager import ASRManager
from coffee_speech_processing.voice_intent_node.intent_classifier import IntentClassifier
from coffee_speech_processing.voice_intent_node.memory_utils import MemoryManager

from shared_configs import (
    GENERATE_BEHAVIOR_RESPONSE_SERVICE,
    VOICE_INTENT_RESPONSE_TOPIC,
    TTS_SERVICE,
    INTENT_MAPPING_BYTE_TO_STRING
)

from coffee_buddy_msgs.srv import GenerateBehaviorResponse, TTSQuery
from coffee_interfaces.srv import ChatService
from std_msgs.msg import String

class VoiceIntentNode(Node):
    """
    ROS2 node that listens to microphone input, performs speech recognition,
    and classifies intent using LLM.
    
    This node handles the coordination between audio capture, speech recognition,
    and intent classification components to provide a complete voice-to-intent
    pipeline.
    """
    
    def __init__(self):
        """Initialize the voice intent node with all required components."""
        super().__init__('voice_intent_node')
        
        self.get_logger().info("Initializing VoiceIntentNode")

        self.service_group = MutuallyExclusiveCallbackGroup()
        
        # Declare parameters
        self._declare_parameters()
        
        # Get parameters
        self._get_parameters()
        
        # Create client for the language model processor node
        self.language_model_processor_client = self.create_client(
            GenerateBehaviorResponse, GENERATE_BEHAVIOR_RESPONSE_SERVICE, callback_group=self.service_group)
        
        self.tts_client = self.create_client(
            TTSQuery, TTS_SERVICE, callback_group=self.service_group)
            
        # Subscribe to TTS audio state
        self.create_subscription(
            String,
            'tts/audio_state',
            self.audio_state_callback,
            10)
            
        # Create Atoma chat service client
        self.atoma_chat_client = self.create_client(
            ChatService, 'chat', callback_group=self.service_group)

        # Check for CUDA availability if device_type is cuda
        self._setup_gpu()
        
        # Initialize components
        self._initialize_components()

        # Create and start the threads
        self.running = True
        self._start_threads()
        
        self.get_logger().info('Voice Intent Node started successfully')
        
    def audio_state_callback(self, msg):
        """Handle TTS audio state changes to control microphone input."""
        state = msg.data
        
        if state == 'playing':
            # Pause microphone input while TTS is playing
            if hasattr(self, 'audio_processor'):
                self.audio_processor.pause_stream()
                self.get_logger().info('Paused microphone input for TTS playback')
        elif state == 'done':
            # Resume microphone input after TTS is done
            if hasattr(self, 'audio_processor'):
                self.audio_processor.resume_stream()
                self.get_logger().info('Resumed microphone input after TTS playback')
    
    def _declare_parameters(self):
        """Declare all ROS2 parameters for this node."""
        # ASR parameters
        self.declare_parameter(
            'model_size', 
            'base', 
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Whisper model size (tiny, base, small, medium, large, turbo)'
            )
        )
        
        # VAD parameters
        self.declare_parameter(
            'use_vad',
            False,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description='Enable Voice Activity Detection for better speech segmentation'
            )
        )
        self.declare_parameter(
            'vad_silence_duration',
            500,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description='Milliseconds of silence to consider speech ended'
            )
        )
        self.declare_parameter(
            'use_noise_reduction',
            True,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description='Enable noise reduction preprocessing before ASR'
            )
        )
        self.declare_parameter(
            'noise_reduction_amount',
            0.75,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Amount of noise to reduce (0.0-1.0)'
            )
        )
        self.declare_parameter(
            'chunk_size',
            1920,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description='Size of audio chunks for VAD (120ms at 16kHz)'
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
        
        # Wake phrase parameter
        self.declare_parameter(
            'wake_phrase',
            'buddy',
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Wake phrase to listen for'
            )
        )
    
    def _get_parameters(self):
        """Get parameter values from the ROS2 parameter system."""
        # ASR parameters
        self.model_size = self.get_parameter('model_size').value
        self.language = self.get_parameter('language').value
        self.device_type = self.get_parameter('device_type').value
        self.compute_type = self.get_parameter('compute_type').value
        self.timeout_segments = self.get_parameter('timeout_segments').value
        self.audio_device_id = self.get_parameter('audio_device_id').value
        self.verbose_logging = self.get_parameter('verbose_logging').value
        self.gpu_memory_monitoring = self.get_parameter('gpu_memory_monitoring').value
        self.memory_cleanup_interval = self.get_parameter('memory_cleanup_interval').value
        self.wake_phrase = self.get_parameter('wake_phrase').value
        
        # VAD parameters
        self.use_vad = self.get_parameter('use_vad').value
        self.vad_silence_duration = self.get_parameter('vad_silence_duration').value
        self.chunk_size = self.get_parameter('chunk_size').value
        self.wake_phrase = self.get_parameter('wake_phrase').value
        
        # Noise reduction parameters
        self.use_noise_reduction = self.get_parameter('use_noise_reduction').value
        self.noise_reduction_amount = self.get_parameter('noise_reduction_amount').value

        self.get_logger().info(f"Parameters loaded: model_size={self.model_size}, language={self.language}, " +
                              f"device_type={self.device_type}, compute_type={self.compute_type}, " +
                              f"timeout_segments={self.timeout_segments}, audio_device_id={self.audio_device_id}, " +
                              f"verbose_logging={self.verbose_logging}, gpu_memory_monitoring={self.gpu_memory_monitoring}")
        
    def _setup_gpu(self):
        """Check for GPU availability and set up memory monitoring."""
        # Conditionally import torch for GPU memory management
        try:
            import torch
            self.torch_available = True
        except ImportError:
            self.torch_available = False
            self.get_logger().warning("PyTorch not available, GPU monitoring disabled")
        
        # Check for CUDA availability
        self.using_gpu = self.device_type == 'cuda' and self.torch_available
        if self.using_gpu:
            if torch.cuda.is_available():
                self.get_logger().info(f"CUDA is available, using GPU: {torch.cuda.get_device_name(0)}")
            else:
                self.get_logger().warning("CUDA requested but not available, falling back to CPU")
                self.device_type = 'cpu'
                self.using_gpu = False
    
    def _initialize_components(self):
        """Initialize all components of the voice intent system."""
        try:
            # Initialize memory manager
            self.memory_manager = MemoryManager(
                using_gpu=self.using_gpu,
                monitoring_enabled=self.gpu_memory_monitoring,
                cleanup_interval=self.memory_cleanup_interval
            )
            self.get_logger().info("Memory manager initialized")
            
            # Initialize audio processor
            self.audio_processor = AudioProcessor(
                audio_device_id=self.audio_device_id
            )
            self.get_logger().info("Audio processor initialized")
            
            # Initialize ASR manager
            self.asr_manager = ASRManager(
                model_size=self.model_size,
                language=self.language,
                device_type=self.device_type,
                compute_type=self.compute_type,
                wake_phrase=self.wake_phrase,
                verbose=self.verbose_logging,
                use_vad=self.use_vad,
                vad_silence_duration=self.vad_silence_duration,
                chunk_size=self.chunk_size
            )
            self.asr_manager.set_timeout_segments(self.timeout_segments)
            self.get_logger().info("ASR manager initialized")
            
        except Exception as e:
            self.get_logger().error(f"Error initializing components: {str(e)}")
            raise
    
    def _start_threads(self):
        """Start all processing threads."""
        # Create inference thread
        self.inference_thread = threading.Thread(target=self._inference_thread)
        self.inference_thread.daemon = True
        
        # Start audio processor
        self.audio_processor.start()
        
        # Start threads
        self.inference_thread.start()

        self.get_logger().info("All processing threads started")
    
    def _inference_thread(self):
        """Thread for processing audio and running ASR inference."""
        self.get_logger().info("Starting inference thread")
        
        consecutive_errors = 0
        last_successful_inference_time = time.time()
        
        while self.running:
            try:
                # Check how many items are in the audio queue
                queue_size = self.audio_processor.audio_queue.qsize()
                if queue_size > self.audio_processor.audio_queue.maxsize * 0.7:
                    self.get_logger().warning(f"Audio queue filling up ({queue_size}/{self.audio_processor.audio_queue.maxsize})")
                
                # Get audio chunk with timeout
                audio_chunk = self.audio_processor.get_audio_chunk()
                if audio_chunk is None:
                    continue
                
                # Detailed profiling before inference
                before_mem = self.memory_manager.get_detailed_memory_usage()
                before_cache = self.asr_manager.get_model_cache_size()
                
                # Log memory before ASR
                self.memory_manager.log_gpu_memory_usage("Before ASR")
                
                # Process the audio chunk with timing
                start_time = time.time()
                try:
                    # The sampling rate is fixed at 16000Hz as per the AudioProcessor
                    self.get_logger().info("Applying noise reduction to audio chunk")
                    audio_chunk = nr.reduce_noise(
                        y=audio_chunk,
                        sr=16000,
                        stationary=True,
                        prop_decrease=self.noise_reduction_amount
                    )
                    self.get_logger().info("Noise reduction complete")

                    transcription = self.asr_manager.process_audio_chunk(audio_chunk)
                    if transcription:
                        # Complete utterance received from VAD
                        self.get_logger().info(f">> UTTERANCE text: {transcription}")
                        
                        # Call Chat service
                        request = ChatService.Request()
                        request.prompt = transcription
                        
                        self.get_logger().info("Calling chat service...")
                        future = self.atoma_chat_client.call_async(request)
                        
                        # Wait for response
                        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
                        
                        if future.result() is not None:
                            response = future.result()
                            if response.success:
                                self.get_logger().info(f"Processor response: {response.response}")
                                
                                # Send response to TTS
                                tts_request = TTSQuery.Request()
                                tts_request.text = response.response
                                future = self.tts_client.call_async(tts_request)
                                rclpy.spin_until_future_complete(self, future, timeout_sec=7.0)
                            else:
                                self.get_logger().error(f"Processor service error: {response.error}")
                        else:
                            self.get_logger().error("Failed to get response from Processor service")
                except Exception as e:
                    self.get_logger().error(f"ASR processing error: {e}")
                    transcription = None
                processing_time = time.time() - start_time
                
                # Log memory after ASR
                self.memory_manager.log_gpu_memory_usage("After ASR")
                
                # Detailed profiling after inference
                after_mem = self.memory_manager.get_detailed_memory_usage()
                after_cache = self.asr_manager.get_model_cache_size()
                
                # Log comprehensive metrics
                # self.get_logger().info(
                #     f"ASR Metrics:\n" \
                #     f"  Processing: {processing_time:.3f}s for {len(audio_chunk)/self.audio_processor.RATE:.2f}s audio\n" \
                #     f"  GPU Memory:\n" \
                #     f"    Current: {after_mem['gpu_used']:.1f}MB\n" \
                #     f"    Reserved: {after_mem['gpu_reserved']:.1f}MB\n" \
                #     f"    Peak: {after_mem['gpu_peak']:.1f}MB\n" \
                #     f"  RAM Memory:\n" \
                #     f"    Current: {after_mem['ram_used']:.1f}MB\n" \
                #     f"    Peak: {after_mem['ram_peak']:.1f}MB\n" \
                #     f"  Cache: {after_cache:.1f}MB"
                # )

                # Memory cleanup and logging
                self.memory_manager.cleanup_memory()
                
                # Mark task as done
                self.audio_processor.task_done()
                
                # Reset error counter on successful processing
                consecutive_errors = 0
                last_successful_inference_time = time.time()

            except Exception as e:
                consecutive_errors += 1
                self.get_logger().error(f'Error in inference thread: {str(e)}')
                
                # If we've had multiple consecutive errors or it's been too long since a successful inference
                if consecutive_errors > 5 or (time.time() - last_successful_inference_time) > 30.0:
                    self.get_logger().error(f"Multiple inference errors detected, attempting recovery")
                    consecutive_errors = 0
                    last_successful_inference_time = time.time()
                    
                    # Force memory cleanup
                    self.memory_manager.cleanup_memory(force=True)
                    
                    # Drain audio queue if it's getting full
                    if hasattr(self.audio_processor, 'drain_audio_queue'):
                        self.audio_processor.drain_audio_queue()
                    
                    # Short sleep to give system time to recover
                    time.sleep(0.5)

    def destroy_node(self):
        """Clean up resources when the node is destroyed."""
        self.get_logger().info("Shutting down VoiceIntentNode")
        self.running = False
        
        # Stop audio processor
        if hasattr(self, 'audio_processor'):
            self.audio_processor.stop()
        
        # Wait for threads to terminate
        if hasattr(self, 'inference_thread') and self.inference_thread.is_alive():
            self.get_logger().info("Waiting for inference thread to terminate")
            self.inference_thread.join(timeout=1.0)
        
        # Perform final cleanup
        if hasattr(self, 'memory_manager') and self.using_gpu:
            self.get_logger().info("Cleaning up GPU resources")
            self.memory_manager.cleanup_memory(force=True)
        
        self.get_logger().info("VoiceIntentNode shutdown complete")
        super().destroy_node()


def main(args=None):
    """Main entry point for the voice intent node."""
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