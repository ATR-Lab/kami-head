#!/usr/bin/env python3
"""
TODO: Delete later once refactoring begins to streamline the framework

Example script demonstrating how to use the whisper_streaming module
as an imported Python library rather than a command-line tool.
"""

import os
import logging
import numpy as np
from whisper_streaming import (
    WhisperStreamingServer,
    asr_factory, 
    load_audio_chunk,
    OnlineASRProcessor
)

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def example_server():
    """Start a Whisper streaming server with custom configuration"""
    logger.info("Starting Whisper streaming server example")
    
    # Initialize the server with custom parameters
    server = WhisperStreamingServer(
        host='localhost',
        port=43007,
        model='base',  # Use 'base' for faster startup, 'medium' or 'large' for better accuracy
        language='auto',  # Auto-detect language
        min_chunk_size=1.0,  # Process audio in chunks of at least 1 second
        vad=True,  # Use voice activity detection
    )
    
    # Start the server (this will block until interrupted)
    logger.info("Server initialized, starting to listen for connections")
    server.start_server()


def example_direct_transcription(audio_file_path):
    """Example of using the Whisper ASR directly without the server"""
    if not os.path.exists(audio_file_path):
        logger.error(f"Audio file not found: {audio_file_path}")
        return
    
    # Create a simple args object
    class Args:
        model = 'base'
        lan = 'auto'
        backend = 'faster-whisper'
        vad = True
        min_chunk_size = 1.0
        buffer_trimming_sec = 15
    
    args = Args()
    
    # Initialize ASR system
    logger.info(f"Initializing Whisper ASR with model: {args.model}")
    asr, processor = asr_factory(args)
    
    # Load the entire audio file
    # In a real application, you might stream this in chunks
    logger.info(f"Processing audio file: {audio_file_path}")
    
    # You could either load the whole file...
    # import librosa
    # audio, _ = librosa.load(audio_file_path, sr=16000, dtype=np.float32)
    
    # ...or process it in chunks for streaming simulation
    processor.init()
    
    # Process the first 10 seconds as an example
    for i in range(0, 10):
        try:
            # Load 1-second chunk
            chunk = load_audio_chunk(audio_file_path, i, i+1)
            processor.insert_audio_chunk(chunk)
            
            # Process the current buffer
            result = processor.process_iter()
            
            # Check if we got a transcription
            if result[0] is not None:
                logger.info(f"Chunk {i}: {result[2]}")
        except Exception as e:
            logger.error(f"Error processing chunk {i}: {str(e)}")
    
    # Finish processing any remaining audio
    final_result = processor.finish()
    if final_result[0] is not None:
        logger.info(f"Final: {final_result[2]}")


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1:
        # If an audio file path is provided, run direct transcription example
        audio_file_path = sys.argv[1]
        example_direct_transcription(audio_file_path)
    else:
        # Otherwise, run the server example
        example_server() 