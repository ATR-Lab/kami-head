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

# Import ROS2 message types
from coffee_buddy_msgs.msg import IntentClassification

# Import our modules
from perception_nodes.audio_processor import AudioProcessor
from perception_nodes.asr_manager import ASRManager
from perception_nodes.intent_classifier import IntentClassifier
from perception_nodes.memory_utils import MemoryManager


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
        
        # Declare parameters
        self._declare_parameters()
        
        # Get parameters
        self._get_parameters()
        
        # Create publisher
        self.publisher = self.create_publisher(IntentClassification, '/voice/intent', 10)
        
        # Check for CUDA availability if device_type is cuda
        self._setup_gpu()
        
        # Initialize components
        self._initialize_components()
        
        # LLM processing queue
        self.llm_queue = queue.Queue(maxsize=10)  # Maximum 10 text prompts in queue
        
        # Create and start the threads
        self.running = True
        self._start_threads()
        
        self.get_logger().info('Voice Intent Node started successfully')
    
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
        
        # LLM classification parameters
        self.declare_parameter(
            'llm_model', 
            'gemma3:1b',
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Ollama LLM model for intent classification (e.g., gemma3:1b, tinyllama, etc.)'
            )
        )
        self.declare_parameter(
            'llm_timeout', 
            3.0, 
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Timeout in seconds for LLM classification'
            )
        )
        self.declare_parameter(
            'llm_retry', 
            1, 
            ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description='Number of retries for LLM classification before falling back'
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
        
        # LLM parameters
        self.llm_model = self.get_parameter('llm_model').value
        self.llm_timeout = self.get_parameter('llm_timeout').value
        self.llm_retry = self.get_parameter('llm_retry').value
        
        self.get_logger().info(f"Parameters loaded: model_size={self.model_size}, language={self.language}, " +
                              f"device_type={self.device_type}, compute_type={self.compute_type}, " +
                              f"timeout_segments={self.timeout_segments}, audio_device_id={self.audio_device_id}, " +
                              f"verbose_logging={self.verbose_logging}, gpu_memory_monitoring={self.gpu_memory_monitoring}")
        self.get_logger().info(f"LLM parameters loaded: llm_model={self.llm_model}, " +
                              f"llm_timeout={self.llm_timeout}, llm_retry={self.llm_retry}")
    
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
                wake_phrase='buddy',
                verbose=self.verbose_logging
            )
            self.asr_manager.set_timeout_segments(self.timeout_segments)
            self.get_logger().info("ASR manager initialized")
            
            # Initialize intent classifier
            self.intent_classifier = IntentClassifier(
                model=self.llm_model,
                timeout=self.llm_timeout,
                max_retries=self.llm_retry
            )
            self.get_logger().info("Intent classifier initialized")
            
        except Exception as e:
            self.get_logger().error(f"Error initializing components: {str(e)}")
            raise
    
    def _start_threads(self):
        """Start all processing threads."""
        # Create inference thread
        self.inference_thread = threading.Thread(target=self._inference_thread)
        self.inference_thread.daemon = True
        
        # Create LLM thread
        self.llm_thread = threading.Thread(target=self._llm_thread)
        self.llm_thread.daemon = True
        
        # Start audio processor
        self.audio_processor.start()
        
        # Start threads
        self.inference_thread.start()
        self.llm_thread.start()
        
        self.get_logger().info("All processing threads started")
    
    def _inference_thread(self):
        """Thread for processing audio and running ASR inference."""
        self.get_logger().info("Starting inference thread")
        
        while self.running:
            try:
                # Get audio chunk with timeout
                audio_chunk = self.audio_processor.get_audio_chunk()
                if audio_chunk is None:
                    continue
                
                # Log GPU memory before inference
                self.memory_manager.log_gpu_memory_usage("Before inference")
                
                # Process the audio chunk
                transcription = self.asr_manager.process_audio_chunk(audio_chunk)
                
                # Memory cleanup and logging after inference
                self.memory_manager.cleanup_memory()
                self.memory_manager.log_gpu_memory_usage("After inference")
                
                # Mark task as done
                self.audio_processor.task_done()
                
                # Process the transcription if we got one
                if transcription:
                    prompt_text, reason, is_complete = self.asr_manager.process_transcription(transcription)
                    
                    # If we have a complete prompt, add it to the LLM queue
                    if is_complete and prompt_text:
                        self._add_to_llm_queue(prompt_text, reason)
            
            except Exception as e:
                self.get_logger().error(f'Error in inference thread: {str(e)}')
    
    def _llm_thread(self):
        """Thread for processing intent classification with LLM."""
        self.get_logger().info("Starting LLM thread")
        
        while self.running:
            try:
                # Get text from queue with timeout
                try:
                    prompt_text = self.llm_queue.get(timeout=0.5)
                except queue.Empty:
                    continue  # No text to process
                
                # Process text with LLM
                intent_code, success = self.intent_classifier.classify(prompt_text)
                
                # Create and publish message
                msg = IntentClassification()
                msg.prompt_text = prompt_text
                msg.intent = intent_code
                
                self.publisher.publish(msg)
                intent_name = self.intent_classifier.get_intent_name(intent_code)
                self.get_logger().info(f"Published intent: prompt='{prompt_text}', intent={intent_code!r} ({intent_name})")
                
                # Mark task as done
                self.llm_queue.task_done()
                
            except Exception as e:
                self.get_logger().error(f'Error in LLM thread: {str(e)}')
    
    def _add_to_llm_queue(self, prompt_text, reason):
        """
        Add a text prompt to the LLM queue for intent classification.
        
        Args:
            prompt_text (str): Text prompt to classify
            reason (str): Reason for completion
        """
        try:
            self.get_logger().info(f"Adding prompt to LLM queue: '{prompt_text}' ({reason})")
            
            # Try to add to queue with timeout
            try:
                self.llm_queue.put(prompt_text, timeout=0.5)
                self.get_logger().info("Successfully added prompt to LLM queue")
            except queue.Full:
                self.get_logger().warning("LLM queue is full, using fallback intent")
                
                # Create and publish fallback message directly
                msg = IntentClassification()
                msg.prompt_text = prompt_text
                msg.intent = self.intent_classifier.get_fallback_intent()
                self.publisher.publish(msg)
                
                self.get_logger().info(f"Published fallback intent: prompt='{prompt_text}', intent=Fallback")
                
        except Exception as e:
            self.get_logger().error(f"Error adding to LLM queue: {str(e)}")
    
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
        
        if hasattr(self, 'llm_thread') and self.llm_thread.is_alive():
            self.get_logger().info("Waiting for LLM thread to terminate")
            self.llm_thread.join(timeout=1.0)
        
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