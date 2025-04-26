#!/usr/bin/env python3

"""
ASR (Automatic Speech Recognition) management for Whisper models.

This module handles initialization and processing of audio data using
Whisper models for speech recognition, as well as wake phrase detection.
"""

import os
import re
import logging
import argparse
import numpy as np

# Import whisper streaming components
try:
    from perception_nodes.whisper_streaming import (
        asr_factory,
        load_audio_chunk,
        OnlineASRProcessor,
        add_shared_args
    )
    WHISPER_STREAMING_AVAILABLE = True
except ImportError:
    WHISPER_STREAMING_AVAILABLE = False

logger = logging.getLogger(__name__)


class ASRManager:
    """
    Manages automatic speech recognition using Whisper models.
    
    This class handles initialization of Whisper ASR models, processing
    of audio data, and wake phrase detection functionality.
    """
    
    def __init__(self, model_size='base', language='en', device_type='cuda', 
                 compute_type='int8', wake_phrase='buddy', verbose=True):
        """
        Initialize the ASR manager.
        
        Args:
            model_size (str): Whisper model size (tiny, base, small, medium, large)
            language (str): Language for transcription (en, auto, etc.)
            device_type (str): Device type used for inference (cuda, cpu, mps)
            compute_type (str): Compute type used for inference (float16, int8_float16, int8, float32)
            wake_phrase (str): Wake phrase to listen for
            verbose (bool): Whether to log detailed information
        """
        if not WHISPER_STREAMING_AVAILABLE:
            raise ImportError("whisper_streaming module is not available")
        
        self.model_size = model_size
        self.language = language
        self.device_type = device_type
        self.compute_type = compute_type
        self.wake_phrase = wake_phrase
        self.verbose = verbose
        
        # Wake word and buffer variables
        self.text_buffer = ""
        self.is_accumulating = False
        self.segment_count = 0
        self.timeout_segments = 5  # Default value
        
        # Initialize ASR components
        self.asr = None
        self.processor = None
        
        # Check if model exists and download if needed
        self.ensure_model_downloaded()
        
        # Initialize the model
        self.init_asr()
    
    def ensure_model_downloaded(self):
        """
        Check if the Whisper model is downloaded, and download it if not.
        
        Properly verifies model integrity by attempting to load it, not just
        checking if the file exists.
        
        Returns:
            bool: True if model is available, False otherwise
        """
        try:
            # Don't just check if file exists - actually try to load it
            import whisper
            logger.info(f'Attempting to load Whisper model {self.model_size}')
            try:
                whisper.load_model(self.model_size)
                logger.info(f'Whisper model {self.model_size} loaded successfully')
                return True
            except Exception as e:
                logger.error(f'Error loading Whisper model: {str(e)}')
                logger.info(f'Attempting to download/re-download the model')
                try:
                    # Force re-download by using download parameter
                    whisper.load_model(self.model_size, download=True)
                    logger.info(f'Successfully downloaded Whisper model {self.model_size}')
                    return True
                except Exception as e:
                    logger.error(f'Failed to download Whisper model: {str(e)}')
                    return False
        except ImportError:
            logger.error('Whisper not available. Please install it with pip install openai-whisper')
            return False
    
    def init_asr(self):
        """
        Initialize the ASR processor.
        
        Returns:
            bool: True if initialization was successful, False otherwise
        """
        logger.info("Starting ASR initialization")
        
        # Create parser with Whisper options
        parser = argparse.ArgumentParser()
        add_shared_args(parser)
        
        # Use parse_known_args instead of parse_args to ignore ROS2 arguments
        args, _ = parser.parse_known_args()
        
        # Override parameters with provided parameters
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
        
        logger.info(f'Initializing Whisper ASR with model {self.model_size} and language {self.language}')
        logger.info(f'Using device type: {self.device_type}, compute type: {self.compute_type}')
        
        # Initialize ASR components
        try:
            logger.info("Creating ASR factory")
            self.asr, self.processor = asr_factory(args)
            self.processor.buffer_trimming_way = 'sentence'  # Ensure sentence-based trimming
            logger.info("ASR factory created successfully")
            
            # Warm up the model with a small audio chunk to make first inference faster
            logger.info("Warming up the ASR model")
            dummy_audio = np.zeros(1600, dtype=np.float32)  # 0.1 seconds of silence
            self.asr.transcribe(dummy_audio)
            self.processor.init()
            logger.info("ASR model warm-up completed")
            return True
        except Exception as e:
            logger.error(f"Error during ASR initialization: {str(e)}")
            return False
    
    def process_audio_chunk(self, audio_chunk):
        """
        Process an audio chunk and return the transcription.
        
        Args:
            audio_chunk (numpy.ndarray): Audio data as float32 numpy array
            
        Returns:
            str: Transcribed text or None if no result
        """
        if audio_chunk is None or len(audio_chunk) == 0:
            return None
        
        try:
            # Insert audio into processor
            self.processor.insert_audio_chunk(audio_chunk)
            
            # Process the current buffer
            result = self.processor.process_iter()
            
            # Check if we got a transcription result
            if result[0] is not None and result[2]:
                transcription = result[2]
                if self.verbose:
                    logger.info(f'Transcription received: {transcription}')
                return transcription
            
            return None
        except Exception as e:
            logger.error(f'Error processing audio chunk: {str(e)}')
            return None
    
    def set_timeout_segments(self, timeout_segments):
        """
        Set the number of segments to process before timeout.
        
        Args:
            timeout_segments (int): Number of segments to process before timeout
        """
        self.timeout_segments = timeout_segments
    
    def contains_wake_phrase(self, text):
        """
        Check if the text contains the wake phrase (case-insensitive).
        
        Args:
            text (str): Text to check for wake phrase
            
        Returns:
            bool: True if wake phrase is found, False otherwise
        """
        result = self.wake_phrase.lower() in text.lower()
        if result and self.verbose:
            logger.info(f"Wake phrase '{self.wake_phrase}' found in text: '{text}'")
        return result
    
    def contains_end_punctuation(self, text):
        """
        Check if the text contains end punctuation (., !, ?).
        
        Args:
            text (str): Text to check for end punctuation
            
        Returns:
            bool: True if end punctuation is found, False otherwise
        """
        result = bool(re.search(r'[.!?]', text))
        if result and self.verbose:
            logger.info(f"End punctuation found in text: '{text}'")
        return result
    
    def extract_content_with_wake_phrase(self, text):
        """
        Extract content starting from the wake phrase and ending at the first end punctuation.
        
        Args:
            text (str): Text to extract content from
            
        Returns:
            tuple: (extracted_text, has_punctuation) where:
                extracted_text (str) is the text starting from wake phrase
                has_punctuation (bool) is whether end punctuation was found
        """
        # Find wake phrase position (case-insensitive)
        wake_phrase_pos = text.lower().find(self.wake_phrase.lower())
        if wake_phrase_pos == -1:
            return text, False
        
        # Extract text starting from wake phrase
        content = text[wake_phrase_pos:]
        
        # Check if end punctuation immediately follows wake word
        wake_end_pos = wake_phrase_pos + len(self.wake_phrase)
        if wake_end_pos < len(text) and re.match(r'[.!?]', text[wake_end_pos]):
            # Replace end punctuation after wake word with comma
            modified_text = text[:wake_end_pos] + "," + text[wake_end_pos+1:]
            content = modified_text[wake_phrase_pos:]
            if self.verbose:
                logger.info(f"Converted punctuation after wake word to comma: '{content}'")
        
        # Find first end punctuation
        match = re.search(r'[.!?]', content)
        if match:
            # If found, cut off everything after the punctuation
            punctuation_pos = match.start()
            return content[:punctuation_pos+1], True
        
        return content, False
    
    def process_transcription(self, transcription):
        """
        Process the transcribed text to handle wake phrase and end conditions.
        
        This method handles the state machine for wake phrase detection
        and sentence accumulation.
        
        Args:
            transcription (str): Transcribed text to process
            
        Returns:
            tuple: (processed_text, reason, is_complete) where:
                processed_text (str) is the processed text (if complete)
                reason (str) is the reason for completion
                is_complete (bool) is whether processing is complete
        """
        if transcription is None or len(transcription.strip()) == 0:
            return None, None, False
        
        if self.verbose:
            logger.info(f"Processing transcription: '{transcription}'")
        
        if not self.is_accumulating:
            # Check for wake phrase
            if self.contains_wake_phrase(transcription):
                # Extract content starting from wake phrase
                extracted, has_punctuation = self.extract_content_with_wake_phrase(transcription)
                self.is_accumulating = True
                self.text_buffer = extracted
                self.segment_count = 1
                if self.verbose:
                    logger.info(f"Wake phrase detected! Starting to accumulate text: '{extracted}'")
                
                # If this segment already has end punctuation, return immediately
                if has_punctuation:
                    complete_text = self.text_buffer.strip()
                    self.is_accumulating = False
                    self.text_buffer = ""
                    self.segment_count = 0
                    return complete_text, "End punctuation detected in initial segment", True
            elif self.verbose:
                logger.debug("No wake phrase detected, ignoring transcription")
            
            return None, None, False
        else:
            # Already accumulating text
            self.segment_count += 1
            
            # Instead of just concatenating with a space, we need to handle the streaming buffer more intelligently
            # If the new segment starts with the same content as the end of the buffer, it's likely an overlap
            overlap_found = False
            
            # Try different overlap lengths to find the best match
            for overlap_size in range(min(len(self.text_buffer), len(transcription)), 3, -1):
                buffer_end = self.text_buffer[-overlap_size:]
                segment_start = transcription[:overlap_size]
                
                # Check for overlap with some flexibility (lowercase comparison for robustness)
                if buffer_end.lower() == segment_start.lower():
                    # Found overlap, append only the new part
                    combined_text = self.text_buffer + transcription[overlap_size:]
                    overlap_found = True
                    if self.verbose:
                        logger.info(f"Found overlap of {overlap_size} characters")
                    break
            
            if not overlap_found:
                # No overlap found, simply append with a space
                combined_text = self.text_buffer + " " + transcription
            
            # Extract from wake phrase again to handle cases where the wake phrase appears in the new segment
            extracted, has_punctuation = self.extract_content_with_wake_phrase(combined_text)
            self.text_buffer = extracted
            if self.verbose:
                logger.info(f"Added segment {self.segment_count} to buffer. Current buffer: '{self.text_buffer}'")
            
            # Check for end punctuation or timeout
            if has_punctuation:
                if self.verbose:
                    logger.info("End punctuation detected, returning accumulated text")
                complete_text = self.text_buffer.strip()
                self.is_accumulating = False
                self.text_buffer = ""
                self.segment_count = 0
                return complete_text, "End punctuation detected", True
            elif self.segment_count >= self.timeout_segments:
                if self.verbose:
                    logger.info(f"Segment timeout ({self.timeout_segments}) reached, returning accumulated text")
                complete_text = self.text_buffer.strip()
                self.is_accumulating = False
                self.text_buffer = ""
                self.segment_count = 0
                return complete_text, "Timeout threshold reached", True
        
        return None, None, False 