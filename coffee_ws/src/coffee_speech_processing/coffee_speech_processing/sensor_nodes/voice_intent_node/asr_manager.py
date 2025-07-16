#!/usr/bin/env python3

"""
ASR (Automatic Speech Recognition) management for Whisper models.

This module handles initialization and processing of audio data using
Whisper models for speech recognition, as well as wake phrase detection.
"""

import os
import re
import time
import logging
import sys
from types import SimpleNamespace
import argparse
import numpy as np

# Import whisper streaming components
try:
    from coffee_speech_processing.utils.whisper_streaming import (
        asr_factory,
        load_audio_chunk,
        OnlineASRProcessor,
        add_shared_args
    )
    WHISPER_STREAMING_AVAILABLE = True
except ImportError:
    WHISPER_STREAMING_AVAILABLE = False

import logging
import sys

logger = logging.getLogger(__name__)


class ASRManager:
    """
    Manages automatic speech recognition using Whisper models.
    
    This class handles initialization of Whisper ASR models, processing
    of audio data, and wake phrase detection functionality.
    """
    
    def __init__(self, model_size='base', language='en', device_type='cuda', 
                 compute_type='int8', wake_phrase='buddy', verbose=True,
                 use_vad=False, vad_silence_duration=500, chunk_size=1920,
                 logfile=sys.stderr):
        """
        Initialize the ASR manager.

        Args:
            language (str, optional): Language code. Defaults to "en".
            model_size (str, optional): Model size. Defaults to "base".
            device_type (str, optional): Device type. Defaults to "cpu".
            compute_type (str, optional): Compute type. Defaults to "int8".
            use_vad (bool, optional): Whether to use Voice Activity Detection. Defaults to False.
            vad_silence_duration (int, optional): Duration of silence in ms to consider speech ended. Defaults to 500.
            chunk_size (int, optional): Size of audio chunks for VAD. Defaults to 1920 (120ms at 16kHz).
            logfile (file, optional): Log file. Defaults to sys.stderr.
            wake_phrase (str): Wake phrase to listen for
            verbose (bool): Whether to log detailed information
            use_vad (bool, optional): Whether to use Voice Activity Detection. Defaults to False.
            vad_silence_duration (int, optional): Duration of silence in ms to consider speech ended. Defaults to 500.
            chunk_size (int, optional): Size of audio chunks for VAD. Defaults to 1920 (120ms at 16kHz).
        """
        if not WHISPER_STREAMING_AVAILABLE:
            raise ImportError("whisper_streaming module is not available")
        
        self.language = language
        self.modelsize = model_size
        self.device_type = device_type
        self.compute_type = compute_type
        self.use_vad = use_vad
        self.vad_silence_duration = vad_silence_duration
        self.chunk_size = chunk_size
        self.logfile = logfile
        self.wake_phrase = wake_phrase
        self.verbose = verbose
        
        # Wake word and buffer variables
        # VAD and transcription state
        self.vad_active = False  # True when VAD detects speech
        self.current_utterance = []  # Buffer for accumulating transcriptions
        self.last_vad_update = time.time()
        self.last_transcription = None
        # self.timeout_segments = 5  # Default value
        self.timeout_duration = 3.0  # 3 seconds timeout for utterance completion
        self.min_utterance_gap = 0.5  # 500ms minimum gap between utterances
        
        # Initialize ASR components
        self.asr = None
        self.processor = None
        self.init_asr()

    def init_asr(self):
        """Initialize or reinitialize the ASR processor."""
        try:
            import torch
            from coffee_speech_processing.utils.whisper_streaming import FasterWhisperASR, VACOnlineASRProcessor, OnlineASRProcessor
            
            # Create the ASR object first
            asr = FasterWhisperASR(
                lan=self.language,
                modelsize=self.modelsize,
                device_type=self.device_type,
                compute_type=self.compute_type,
                logfile=sys.stderr
            )
            
            # Enable VAD in the ASR model if needed
            if self.use_vad:
                asr.use_vad()
            
            # Create the processor
            if self.use_vad:
                # Convert vad_silence_duration from ms to seconds for online_chunk_size
                online_chunk_size = self.vad_silence_duration / 1000
                self.processor = VACOnlineASRProcessor(
                    online_chunk_size,  # online_chunk_size in seconds
                    asr=asr,            # ASR model will determine device type
                    logfile=self.logfile
                )
                logger.info(f"Using VAD-enabled ASR processor on device: {self.device_type}")
            else:
                self.processor = OnlineASRProcessor(
                    asr=asr,
                    logfile=self.logfile
                )
                logger.info("Using standard ASR processor")
            
            self.asr = self.processor
            return True
        except Exception as e:
            logger.error(f"Error initializing ASR processor: {str(e)}")
            return False
        
        # Check if model exists and download if needed
        self.ensure_model_downloaded()
        
        # Initialize the model
        self._init_processor()
    
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
            logger.info(f'Attempting to load Whisper model {self.modelsize}')
            try:
                whisper.load_model(self.modelsize)
                logger.info(f'Whisper model {self.modelsize} loaded successfully')
                return True
            except Exception as e:
                logger.error(f'Error loading Whisper model: {str(e)}')
                logger.info(f'Attempting to download/re-download the model')
                try:
                    # Force re-download by using download parameter
                    whisper.load_model(self.modelsize, download=True)
                    logger.info(f'Successfully downloaded Whisper model {self.modelsize}')
                    return True
                except Exception as e:
                    logger.error(f'Failed to download Whisper model: {str(e)}')
                    return False
        except ImportError:
            logger.error('Whisper not available. Please install it with pip install openai-whisper')
            return False
    
    def _init_processor(self):
        """Initialize the ASR processor."""
        try:
            # Create ASR instance
            args = SimpleNamespace(
                model=self.modelsize,
                model_dir=None,
                cache_dir=None,
                device=self.device_type,
                compute_type=self.compute_type,
                lan=self.language,
                log_level="ERROR"
            )

            # Create ASR instance
            self.asr, base_processor = asr_factory(args, logfile=self.logfile)
            
            if self.use_vad:
                # Initialize VAD-enabled processor
                from ..utils.whisper_streaming.whisper_online import VACOnlineASRProcessor
                self.processor = VACOnlineASRProcessor(
                    chunk_size=self.chunk_size,
                    asr=self.asr,
                    tokenizer=None,  # Will be initialized on first use
                    buffer_trimming=("segment", 30),  # Increased buffer size for better context
                    logfile=self.logfile
                )
                logger.info("ASR processor initialized with VAD")
            else:
                # Use base processor without VAD
                self.processor = base_processor
                logger.info("ASR processor initialized without VAD")
            
            logger.info("ASR processor initialized successfully")
        except Exception as e:
            logger.error(f"Error initializing ASR processor: {e}")
            self.processor = None
    
    def get_model_cache_size(self):
        """Get the current model cache size in MB."""
        if not hasattr(self, 'processor') or self.processor is None:
            return 0
            
        # Get internal state size if available
        try:
            if hasattr(self.processor, 'audio_buffer'):
                return len(self.processor.audio_buffer) * 4 / 1024 / 1024  # float32 = 4 bytes
            elif hasattr(self.processor, '_buffer'):
                return len(self.processor._buffer) * 4 / 1024 / 1024
            elif hasattr(self.processor, 'buffer_size'):
                return self.processor.buffer_size * 4 / 1024 / 1024
            else:
                # Estimate based on args
                return self.processor.args.buffer_trimming_sec * 16000 * 4 / 1024 / 1024
        except Exception as e:
            logger.debug(f"Could not get exact cache size: {e}")
            return 0
    
    def process_audio_chunk(self, audio_chunk):
        """Process a chunk of audio data and return transcription.
        
        Args:
            audio_chunk (numpy.ndarray): Audio data as float32 numpy array
            
        Returns:
            str: Transcribed text or None if no result
        """
        if audio_chunk is None or len(audio_chunk) == 0:
            return None
        
        try:
            # Initialize tokenizer if needed
            if not hasattr(self, '_tokenizer'):
                logger.info("Initializing tokenizer...")
                from mosestokenizer import MosesTokenizer
                self._tokenizer = MosesTokenizer(self.language)
                self.processor.tokenizer = self._tokenizer
            elif self.processor.tokenizer is None:
                self.processor.tokenizer = self._tokenizer
            
            # Insert audio into processor
            self.processor.insert_audio_chunk(audio_chunk)
            
            # Process the current buffer
            result = self.processor.process_iter()
            
            # Unpack VAD status and transcription
            start_time, end_time, transcription = result
            current_time = time.time()
            
            # Handle VAD state changes
            if start_time is not None and not self.vad_active:
                # VAD detected start of speech
                self.vad_active = True
                self.current_utterance = []
                self.last_vad_update = current_time
                logger.debug("VAD: Speech started")
            
            # Update transcription buffer if we have text
            if transcription and transcription.strip():
                if self.vad_active and transcription != self.last_transcription:
                    self.current_utterance.append(transcription.strip())
                    self.last_transcription = transcription
                    self.last_vad_update = current_time
                    logger.debug(f"VAD: Added transcription: {transcription}")
            
            # Check for end of speech
            if end_time is not None and self.vad_active:
                # VAD detected end of speech
                self.vad_active = False
                if self.current_utterance:
                    # Join accumulated transcriptions and clean up
                    final_transcription = " ".join(self.current_utterance)
                    self.current_utterance = []
                    self.last_transcription = None
                    logger.info(f"VAD: Speech ended, final transcription: {final_transcription}")
                    return final_transcription
            
            # Handle timeout case
            if self.vad_active and (current_time - self.last_vad_update) > 10.0:  # 10 second timeout
                logger.warning("VAD: Timeout reached, forcing utterance end")
                self.vad_active = False
                if self.current_utterance:
                    final_transcription = " ".join(self.current_utterance)
                    self.current_utterance = []
                    self.last_transcription = None
                    return final_transcription
            
            return None

        except Exception as e:
            logger.error(f'Error processing audio chunk: {str(e)}')
            # If we get a processing error, try to recover the ASR processor
            try:
                logger.warning("Attempting to recover ASR processor...")
                self.init_asr()
                # Reset VAD state on recovery
                self.vad_active = False
                self.current_utterance = []
                self.last_transcription = None
            except Exception as recover_e:
                logger.error(f"Failed to recover ASR processor: {recover_e}")
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