"""
Whisper Streaming Package for real-time speech recognition.

This package provides tools for real-time audio transcription using OpenAI's Whisper model.
It includes functionality for:
- Online ASR processing (whisper_online)
- Server for streaming audio and transcription (whisper_online_server)
- Voice activity detection (silero_vad_iterator)
- Packet handling for audio streaming (line_packet)
"""

# Import core functionality from whisper_online
from .whisper_online import (
    asr_factory, set_logging, add_shared_args, 
    load_audio_chunk, load_audio, ASRBase, OnlineASRProcessor,
    WhisperTimestampedASR, FasterWhisperASR, MLXWhisper, OpenaiApiASR
)

# Import server functionality
from .whisper_online_server import (
    ServerProcessor, Connection, WhisperStreamingServer, add_server_args
)

# Import VAD functionality
from .silero_vad_iterator import (
    VADIterator, FixedVADIterator
)

# Import packet handling
from .line_packet import send_one_line, receive_lines

# Define package version
__version__ = "0.1.0" 