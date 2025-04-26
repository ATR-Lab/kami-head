#!/usr/bin/env python3

"""
Audio processing utilities for capturing and handling audio data.

This module handles audio device detection, microphone capturing,
and audio buffering for ASR processing.
"""

import pyaudio
import numpy as np
import queue
import threading
import time
import logging

logger = logging.getLogger(__name__)


class AudioProcessor:
    """
    Handles audio capture and processing from microphone input.
    
    This class is responsible for initializing audio hardware,
    capturing audio data, and providing it to ASR systems.
    """
    
    def __init__(self, audio_device_id=0, buffer_size=100):
        """
        Initialize the audio processor.
        
        Args:
            audio_device_id (int): ID of the audio input device to use
            buffer_size (int): Maximum size of the audio processing queue
        """
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 16000  # Whisper requires 16kHz
        self.CHUNK = 1024
        self.audio_device_id = audio_device_id
        
        # Audio processing queue
        self.audio_queue = queue.Queue(maxsize=buffer_size)
        
        # Initialize PyAudio
        self.audio = pyaudio.PyAudio()
        self.running = False
        self.audio_thread = None
        
        # Get and validate audio device
        self.list_audio_devices()
    
    def list_audio_devices(self):
        """
        List available audio input devices and validate the chosen device ID.
        
        Logs available audio devices and validates the selected device ID.
        If the selected device is invalid, falls back to a valid default.
        """
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
                logger.info(f"Input Device {i}: {device_name}")
                
                # Check for exact match with "default" name
                if device_name == "default":
                    default_device_id = i
                    logger.info(f"Found device with exact name 'default' (ID: {i})")
        
        # Check if selected device is valid
        valid_ids = [dev[0] for dev in input_devices]
        if not input_devices:
            logger.error("No input devices found!")
            raise RuntimeError("No audio input devices available")
        
        # If a device with name "default" was found, use it
        if default_device_id is not None:
            logger.info(f"Setting audio device to 'default' (ID: {default_device_id})")
            self.audio_device_id = default_device_id
        # Otherwise check if specified ID is valid
        elif self.audio_device_id not in valid_ids:
            logger.warning(f"Specified audio device ID {self.audio_device_id} not found. Using default device (ID: {valid_ids[0]})")
            self.audio_device_id = valid_ids[0]
        
        logger.info(f"Using audio input device: ID {self.audio_device_id}")
    
    def start(self):
        """
        Start the audio capture thread.
        
        Returns:
            bool: True if started successfully, False otherwise
        """
        if self.running:
            logger.warning("Audio processor is already running")
            return False
        
        self.running = True
        self.audio_thread = threading.Thread(target=self._capture_audio)
        self.audio_thread.daemon = True
        self.audio_thread.start()
        logger.info("Audio capture thread started")
        return True
    
    def stop(self):
        """
        Stop the audio capture thread and clean up resources.
        
        Returns:
            bool: True if stopped successfully, False otherwise
        """
        if not self.running:
            logger.warning("Audio processor is not running")
            return False
        
        logger.info("Stopping audio processor")
        self.running = False
        
        if self.audio_thread and self.audio_thread.is_alive():
            self.audio_thread.join(timeout=1.0)
            logger.info("Audio capture thread terminated")
        
        logger.info("Terminating PyAudio")
        self.audio.terminate()
        return True
    
    def _capture_audio(self):
        """
        Thread function for capturing audio from microphone and adding to processing queue.
        
        Continuously captures audio from the selected input device and adds
        it to the processing queue.
        """
        try:
            logger.info("Starting audio capture thread")
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
                logger.info(f'Started audio stream from device ID {self.audio_device_id} successfully')
            except Exception as e:
                logger.error(f"Failed to open audio stream: {str(e)}")
                return
            
            audio_buffer = np.array([], dtype=np.float32)
            consecutive_full_queue = 0
            
            logger.info("Entering audio capture loop")
            while self.running:
                try:
                    # Check if queue is getting critically full
                    if self.audio_queue.qsize() > self.audio_queue.maxsize * 0.8:
                        consecutive_full_queue += 1
                        if consecutive_full_queue > 5:
                            # Queue has been nearly full for multiple iterations, drain it
                            logger.warning(f"Audio queue critically full ({self.audio_queue.qsize()}/{self.audio_queue.maxsize}), draining to recover")
                            self.drain_audio_queue()
                            consecutive_full_queue = 0
                    else:
                        consecutive_full_queue = 0
                    
                    # Read audio data
                    data = stream.read(self.CHUNK, exception_on_overflow=False)
                    
                    # Convert data to numpy array (int16)
                    audio_data = np.frombuffer(data, dtype=np.int16)
                    
                    # Convert to float32 and normalize to [-1, 1]
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
                            logger.warning("Audio processing queue is full, dropping chunk")
                            # Still reset buffer to avoid growing indefinitely
                            audio_buffer = np.array([], dtype=np.float32)
                    
                except Exception as e:
                    logger.error(f'Error capturing audio: {str(e)}')
                    time.sleep(0.1)  # Small delay to avoid CPU spike on continuous errors
            
            # Clean up
            logger.info("Shutting down audio capture")
            stream.stop_stream()
            stream.close()
            
        except Exception as e:
            logger.error(f'Error in audio capture thread: {str(e)}')
    
    def drain_audio_queue(self):
        """
        Drain the audio queue when it gets too full to prevent freezing.
        Keeps the most recent chunks and removes older ones.
        """
        try:
            # Keep track of how many items we remove
            removed_count = 0
            
            # Calculate how many items to keep (25% of max size)
            items_to_keep = max(1, int(self.audio_queue.maxsize * 0.25))
            
            # Calculate how many items to remove
            current_size = self.audio_queue.qsize()
            items_to_remove = max(0, current_size - items_to_keep)
            
            # Remove items from the queue
            for _ in range(items_to_remove):
                try:
                    # Use get_nowait to avoid blocking
                    self.audio_queue.get_nowait()
                    self.audio_queue.task_done()
                    removed_count += 1
                except queue.Empty:
                    # Queue became empty while we were draining
                    break
            
            logger.info(f"Drained {removed_count} items from audio queue, new size: {self.audio_queue.qsize()}")
        except Exception as e:
            logger.error(f"Error draining audio queue: {str(e)}")
    
    def get_audio_chunk(self, timeout=0.5):
        """
        Get an audio chunk from the processing queue.
        
        Args:
            timeout (float): Maximum time to wait for audio chunk
            
        Returns:
            numpy.ndarray: Audio chunk data or None if queue is empty
        """
        try:
            return self.audio_queue.get(timeout=timeout)
        except queue.Empty:
            return None
    
    def task_done(self):
        """
        Mark the current audio chunk task as done.
        """
        self.audio_queue.task_done() 