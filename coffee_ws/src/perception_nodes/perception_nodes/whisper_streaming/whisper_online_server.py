#!/usr/bin/env python3
"""
Whisper Streaming Server Module

This module provides functionality for running a server that
streams audio and returns transcriptions using Whisper ASR.
It can be used as a standalone script or imported as a module.
"""

from .whisper_online import *

import sys
import argparse
import os
import logging
import numpy as np
import io
import soundfile
import socket
from .line_packet import *

logger = logging.getLogger(__name__)

# Constants
SAMPLING_RATE = 16000


class Connection:
    '''It wraps a socket connection object for convenient sending/receiving of data'''
    PACKET_SIZE = 32000*5*60  # 5 minutes # was: 65536

    def __init__(self, conn):
        self.conn = conn
        self.last_line = ""
        self.conn.setblocking(True)

    def send(self, line):
        '''It doesn't send the same line twice, because it was problematic in online-text-flow-events'''
        if line == self.last_line:
            return
        line_packet.send_one_line(self.conn, line)
        self.last_line = line

    def receive_lines(self):
        in_line = line_packet.receive_lines(self.conn)
        return in_line

    def non_blocking_receive_audio(self):
        try:
            r = self.conn.recv(self.PACKET_SIZE)
            return r
        except ConnectionResetError:
            return None


class ServerProcessor:
    """
    Wraps socket and ASR object, and serves one client connection.
    Next client should be served by a new instance of this object.
    """

    def __init__(self, connection, online_asr_proc, min_chunk):
        self.connection = connection
        self.online_asr_proc = online_asr_proc
        self.min_chunk = min_chunk
        self.last_end = None
        self.is_first = True

    def receive_audio_chunk(self):
        """
        Receive all audio that is available by this time.
        Blocks operation if less than self.min_chunk seconds is available.
        Unblocks if connection is closed or a chunk is available.
        """
        out = []
        minlimit = self.min_chunk*SAMPLING_RATE
        while sum(len(x) for x in out) < minlimit:
            raw_bytes = self.connection.non_blocking_receive_audio()
            if not raw_bytes:
                break
            sf = soundfile.SoundFile(io.BytesIO(raw_bytes), channels=1, endian="LITTLE", 
                                   samplerate=SAMPLING_RATE, subtype="PCM_16", format="RAW")
            audio, _ = librosa.load(sf, sr=SAMPLING_RATE, dtype=np.float32)
            out.append(audio)
        if not out:
            return None
        conc = np.concatenate(out)
        if self.is_first and len(conc) < minlimit:
            return None
        self.is_first = False
        return np.concatenate(out)

    def format_output_transcript(self, o):
        """
        Format the transcript output.
        Output format: "beg_timestamp end_timestamp text"
        
        This function differs from whisper_online.output_transcript in that
        succeeding [beg,end] intervals are not overlapping because ELITR protocol
        requires it. Therefore, beg is max of previous end and current beg outputted by Whisper.
        """
        if o[0] is not None:
            beg, end = o[0]*1000, o[1]*1000
            if self.last_end is not None:
                beg = max(beg, self.last_end)

            self.last_end = end
            logger.debug("%1.0f %1.0f %s" % (beg, end, o[2]))
            return "%1.0f %1.0f %s" % (beg, end, o[2])
        else:
            logger.debug("No text in this segment")
            return None

    def send_result(self, o):
        """Format and send transcription result to the client"""
        msg = self.format_output_transcript(o)
        if msg is not None:
            self.connection.send(msg)

    def process(self):
        """Handle one client connection"""
        self.online_asr_proc.init()
        while True:
            a = self.receive_audio_chunk()
            if a is None:
                break
            self.online_asr_proc.insert_audio_chunk(a)
            o = self.online_asr_proc.process_iter()
            try:
                self.send_result(o)
            except BrokenPipeError:
                logger.info("broken pipe -- connection closed?")
                break

        # Uncomment to enable final transcript processing
        # o = self.online_asr_proc.finish()
        # self.send_result(o)


class WhisperStreamingServer:
    """
    A server that streams audio and returns transcriptions using Whisper ASR.
    """
    
    def __init__(self, host='localhost', port=43007, model='base', language='auto', 
                 min_chunk_size=1.0, buffer_trimming_sec=15, warmup_file=None,
                 vad=False, backend='faster-whisper', **kwargs):
        """
        Initialize the server with configuration parameters.
        
        Args:
            host (str): Host address to bind the server
            port (int): Port to bind the server
            model (str): Whisper model size/name
            language (str): Language code or 'auto' for auto-detection
            min_chunk_size (float): Minimum audio chunk size in seconds
            buffer_trimming_sec (int): Buffer trimming seconds
            warmup_file (str, optional): Path to audio file for warming up Whisper
            vad (bool): Whether to use voice activity detection
            backend (str): Whisper backend ('whisper-timestamped', 'faster-whisper', 'mlx-whisper', 'openai-api')
            **kwargs: Additional arguments to pass to the ASR factory
        """
        self.host = host
        self.port = port
        self.model = model
        self.language = language
        self.min_chunk_size = min_chunk_size
        self.buffer_trimming_sec = buffer_trimming_sec
        self.warmup_file = warmup_file
        self.vad = vad
        self.backend = backend
        self.additional_args = kwargs
        
        # Will be initialized when start_server is called
        self.asr = None
        self.online_processor = None

    def initialize_asr(self):
        """Initialize the ASR model and processor"""
        # Create a dummy args object to pass to asr_factory
        class Args:
            pass
        
        args = Args()
        args.model = self.model
        args.lan = self.language
        args.buffer_trimming_sec = self.buffer_trimming_sec
        args.min_chunk_size = self.min_chunk_size
        args.vad = self.vad
        args.backend = self.backend
        
        # Add any additional arguments
        for k, v in self.additional_args.items():
            setattr(args, k, v)
        
        self.asr, self.online_processor = asr_factory(args)
        
        # Warm up the ASR if warmup file is provided
        if self.warmup_file and os.path.isfile(self.warmup_file):
            a = load_audio_chunk(self.warmup_file, 0, 1)
            self.asr.transcribe(a)
            logger.info("Whisper model warmed up.")
        elif self.warmup_file:
            logger.warning(f"Warmup file {self.warmup_file} not found. First transcription may be slow.")
        else:
            logger.warning("No warmup file provided. First transcription may be slow.")

    def start_server(self):
        """Start the server and listen for connections"""
        if not self.asr:
            self.initialize_asr()
            
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((self.host, self.port))
            s.listen(1)
            logger.info(f'Listening on {self.host}:{self.port}')
            
            try:
                while True:
                    conn, addr = s.accept()
                    logger.info(f'Connected to client on {addr}')
                    connection = Connection(conn)
                    proc = ServerProcessor(connection, self.online_processor, self.min_chunk_size)
                    proc.process()
                    conn.close()
                    logger.info('Connection to client closed')
            except KeyboardInterrupt:
                logger.info('Server stopped by user')
            except Exception as e:
                logger.error(f'Server error: {e}')
            finally:
                logger.info('Server shutting down')


def add_server_args(parser):
    """Add server-specific command line arguments to parser"""
    # Server options
    parser.add_argument("--host", type=str, default='localhost',
                      help="Host address to bind the server")
    parser.add_argument("--port", type=int, default=43007,
                      help="Port to bind the server")
    parser.add_argument("--warmup-file", type=str, dest="warmup_file", 
                      help="Path to a speech audio wav file to warm up Whisper")


def main():
    """Main function when script is executed directly"""
    parser = argparse.ArgumentParser(description="Whisper Streaming Server")
    
    # Add arguments
    add_server_args(parser)
    add_shared_args(parser)
    
    args = parser.parse_args()
    set_logging(args, logger, other="")
    
    # Create and start the server
    server = WhisperStreamingServer(
        host=args.host,
        port=args.port,
        model=args.model,
        language=args.lan,
        min_chunk_size=args.min_chunk_size,
        buffer_trimming_sec=args.buffer_trimming_sec,
        vad=args.vad,
        backend=args.backend,
        warmup_file=args.warmup_file
    )
    
    server.start_server()


if __name__ == "__main__":
    main()
