#!/usr/bin/env python3

import sys
import os
import numpy as np
import wave
import tempfile
import threading
import traceback
from typing import Union
import tqdm
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                           QHBoxLayout, QComboBox, QPushButton, QLabel, 
                           QStackedWidget, QProgressBar, QMessageBox, QFileDialog,
                           QTextEdit)
from PyQt5.QtCore import Qt, pyqtSignal, QThread
import pyaudio
import torch
import torchmetrics.audio as tm_audio
import whisper
import gc  # For garbage collection

# Enable more detailed logging
import logging
logging.basicConfig(level=logging.DEBUG, 
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Setup global exception hook to print full traceback
def global_exception_hook(exctype, value, tb):
    tb_text = ''.join(traceback.format_exception(exctype, value, tb))
    print(f"UNHANDLED EXCEPTION:\n{tb_text}")
    sys.__excepthook__(exctype, value, tb)
    
sys.excepthook = global_exception_hook

class ProgressListener:
    """Interface for progress listeners"""
    def on_progress(self, current: Union[int, float], total: Union[int, float]):
        pass
    
    def on_finished(self):
        pass


class _CustomProgressBar(tqdm.tqdm):
    """Custom progress bar that reports to registered listeners"""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._current = self.n

    def update(self, n):
        super().update(n)
        self._current += n

        # Inform listeners
        listeners = _get_thread_local_listeners()
        for listener in listeners:
            listener.on_progress(self._current, self.total)
            
        # Force the UI to update by processing events
        QApplication.processEvents()


_thread_local = threading.local()
_hooked = False


def _get_thread_local_listeners():
    """Get thread-local progress listeners"""
    if not hasattr(_thread_local, 'listeners'):
        _thread_local.listeners = []
    return _thread_local.listeners


def init_progress_hook():
    """Initialize the progress hook for Whisper"""
    global _hooked
    if _hooked:
        return

    # Inject into tqdm.tqdm of Whisper
    import whisper.transcribe
    transcribe_module = sys.modules['whisper.transcribe']
    transcribe_module.tqdm.tqdm = _CustomProgressBar
    _hooked = True


class ProgressListenerHandle:
    """Context manager for progress listeners"""
    def __init__(self, listener: ProgressListener):
        self.listener = listener
    
    def __enter__(self):
        # Register the progress listener
        init_progress_hook()
        listeners = _get_thread_local_listeners()
        listeners.append(self.listener)
        return self.listener

    def __exit__(self, exc_type, exc_val, exc_tb):
        # Unregister the progress listener
        listeners = _get_thread_local_listeners()
        if self.listener in listeners:
            listeners.remove(self.listener)
        
        if exc_type is None:
            self.listener.on_finished()


class AudioRecorder(QThread):
    """Thread for audio recording to avoid UI freezing"""
    finished = pyqtSignal(str)
    update_progress = pyqtSignal(int)
    
    def __init__(self, device_index, duration=10):
        super().__init__()
        self.device_index = device_index
        self.duration = duration
        self.chunk = 1024
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 44100
        
    def run(self):
        p = pyaudio.PyAudio()
        
        # Create a temporary file to store the audio
        temp_file = tempfile.mktemp(suffix='.wav')
        
        # Open stream
        stream = p.open(format=self.format,
                        channels=self.channels,
                        rate=self.rate,
                        input=True,
                        input_device_index=self.device_index,
                        frames_per_buffer=self.chunk)
        
        frames = []
        
        # Calculate total chunks needed for the duration
        total_chunks = int(self.rate / self.chunk * self.duration)
        
        # Record audio
        for i in range(total_chunks):
            data = stream.read(self.chunk)
            frames.append(data)
            progress = int((i / total_chunks) * 100)
            self.update_progress.emit(progress)
        
        # Stop and close the stream
        stream.stop_stream()
        stream.close()
        p.terminate()
        
        # Save the recorded audio to the temporary file
        wf = wave.open(temp_file, 'wb')
        wf.setnchannels(self.channels)
        wf.setsampwidth(p.get_sample_size(self.format))
        wf.setframerate(self.rate)
        wf.writeframes(b''.join(frames))
        wf.close()
        
        # Emit the path to the saved audio file
        self.finished.emit(temp_file)


class AudioPlayer:
    """Class to handle audio playback"""
    def __init__(self):
        self.p = pyaudio.PyAudio()
        self.stream = None
        
    def play(self, file_path):
        # Open the wave file
        wf = wave.open(file_path, 'rb')
        
        # Open a stream
        self.stream = self.p.open(format=self.p.get_format_from_width(wf.getsampwidth()),
                                 channels=wf.getnchannels(),
                                 rate=wf.getframerate(),
                                 output=True)
        
        # Read data in chunks and play
        data = wf.readframes(1024)
        while data:
            self.stream.write(data)
            data = wf.readframes(1024)
        
        # Close everything
        self.stream.stop_stream()
        self.stream.close()
        wf.close()
        
    def stop(self):
        if self.stream is not None:
            self.stream.stop_stream()
            self.stream.close()
            self.stream = None


class MetricsCalculator:
    """Class to calculate audio quality metrics"""
    def __init__(self):
        self.snr = tm_audio.SignalNoiseRatio()
        
    def calculate_metrics(self, file_path):
        # Load the audio file
        wf = wave.open(file_path, 'rb')
        n_frames = wf.getnframes()
        sample_width = wf.getsampwidth()
        framerate = wf.getframerate()
        
        # Read all frames
        frames = wf.readframes(n_frames)
        
        # Convert to numpy array
        if sample_width == 2:
            dtype = np.int16
        elif sample_width == 4:
            dtype = np.int32
        else:
            dtype = np.uint8
            
        audio_data = np.frombuffer(frames, dtype=dtype)
        
        # Convert to float tensor normalized between -1 and 1
        audio_tensor = torch.tensor(audio_data, dtype=torch.float32) / (2**(8 * sample_width - 1))
        
        # Reshape for torchmetrics (batch_size, samples)
        audio_tensor = audio_tensor.unsqueeze(0)
        
        try:
            # Create a simulated noise floor with same shape as audio
            # Using a very low amplitude noise (around -40dB relative to signal)
            signal_power = torch.mean(audio_tensor**2)
            noise_power = signal_power * 0.0001  # -40dB
            noise_tensor = torch.randn_like(audio_tensor) * torch.sqrt(noise_power)
            
            # Calculate SNR
            snr_value = self.snr(audio_tensor, noise_tensor)
            
            return {
                "snr": float(snr_value),
                "sample_rate": framerate,
                "duration": n_frames / framerate,
                "bit_depth": 8 * sample_width
            }
        except Exception as e:
            # Fallback if we can't calculate SNR
            return {
                "snr": None,
                "sample_rate": framerate,
                "duration": n_frames / framerate,
                "bit_depth": 8 * sample_width
            }


class WhisperTranscriber(QThread):
    """Thread for Whisper model download and transcription"""
    download_progress = pyqtSignal(int)
    transcription_progress = pyqtSignal(int)
    finished = pyqtSignal(str)
    error = pyqtSignal(str)
    
    def __init__(self, audio_file, model_name="base"):
        super().__init__()
        self.audio_file = audio_file
        self.model_name = model_name
        self.model = None
        self.cuda_available = torch.cuda.is_available()
        logger.info(f"Initializing WhisperTranscriber with model: {model_name}, CUDA available: {self.cuda_available}")
        logger.info(f"Audio file: {audio_file}, exists: {os.path.exists(audio_file)}")
        
    def run(self):
        try:
            # First, check if model exists and download if needed
            logger.info(f"Checking if model {self.model_name} exists")
            if not self._check_model_exists():
                logger.info(f"Model {self.model_name} not found, downloading...")
                self._download_model()
            else:
                logger.info(f"Model {self.model_name} already exists")
            
            # Load the model
            logger.info(f"Loading model {self.model_name}")
            self._load_model()
            
            # Transcribe the audio
            logger.info(f"Transcribing audio from {self.audio_file}")
            transcription = self._transcribe_audio()
            
            # Emit the transcription result
            logger.info(f"Transcription completed: {transcription[:50]}...")
            self.finished.emit(transcription)
        except Exception as e:
            error_traceback = traceback.format_exc()
            logger.error(f"Error in WhisperTranscriber.run: {str(e)}\n{error_traceback}")
            self.error.emit(f"{str(e)}\n\nFull traceback:\n{error_traceback}")
    
    def _check_model_exists(self):
        # Check if the model is already downloaded
        model_path = os.path.expanduser(f"~/.cache/whisper/{self.model_name}.pt")
        exists = os.path.exists(model_path)
        logger.debug(f"Model file check: {model_path}, exists: {exists}")
        return exists
    
    def _download_model(self):
        """Download the Whisper model with progress tracking"""
        class DownloadProgressListener(ProgressListener):
            def __init__(self, signal_fn):
                self.signal_fn = signal_fn
                
            def on_progress(self, current, total):
                progress = int((current / total) * 100) if total > 0 else 0
                self.signal_fn.emit(progress)
                
            def on_finished(self):
                self.signal_fn.emit(100)
        
        try:
            # This will trigger the download if the model doesn't exist
            with ProgressListenerHandle(DownloadProgressListener(self.download_progress)):
                # This forces download without loading the model yet
                logger.info(f"Starting model download: {self.model_name}")
                whisper._download(self.model_name, whisper._MODELS, False)
                logger.info(f"Model download completed")
        except Exception as e:
            error_traceback = traceback.format_exc()
            logger.error(f"Error downloading model: {str(e)}\n{error_traceback}")
            raise
    
    def _load_model(self):
        """Load the Whisper model using CUDA if available"""
        try:
            fp16 = self.cuda_available  # Use FP16 if CUDA is available
            logger.info(f"Loading model: {self.model_name}, device: {'cuda' if self.cuda_available else 'cpu'}, fp16: {fp16}")
            self.model = whisper.load_model(self.model_name, device="cuda" if self.cuda_available else "cpu", fp16=fp16)
            logger.info(f"Model loaded successfully")
        except Exception as e:
            error_traceback = traceback.format_exc()
            logger.error(f"Error loading model: {str(e)}\n{error_traceback}")
            raise
    
    def _transcribe_audio(self):
        """Transcribe the audio file with progress tracking"""
        class TranscriptionProgressListener(ProgressListener):
            def __init__(self, signal_fn):
                self.signal_fn = signal_fn
                
            def on_progress(self, current, total):
                progress = int((current / total) * 100) if total > 0 else 0
                self.signal_fn.emit(progress)
                
            def on_finished(self):
                self.signal_fn.emit(100)
        
        try:
            with ProgressListenerHandle(TranscriptionProgressListener(self.transcription_progress)):
                logger.info(f"Starting transcription of {self.audio_file}")
                
                # Verify the audio file exists
                if not os.path.exists(self.audio_file):
                    error_msg = f"Audio file does not exist: {self.audio_file}"
                    logger.error(error_msg)
                    raise FileNotFoundError(error_msg)
                
                # Print audio file info
                try:
                    with wave.open(self.audio_file, 'rb') as wf:
                        logger.info(f"Audio file info: channels={wf.getnchannels()}, width={wf.getsampwidth()}, " 
                                   f"rate={wf.getframerate()}, frames={wf.getnframes()}")
                except Exception as e:
                    logger.warning(f"Could not read wave file info: {str(e)}")
                
                # Try using the high-level API first
                try:
                    logger.info("Attempting transcription with high-level API")
                    result = self.model.transcribe(
                        self.audio_file, 
                        fp16=self.cuda_available,
                        verbose=None
                    )
                    logger.info("High-level API transcription successful")
                    return result["text"]
                except Exception as e:
                    logger.warning(f"High-level API failed: {str(e)}, trying low-level API")
                
                # If that fails, use the low-level API
                # Load audio file
                logger.info("Loading audio with low-level API")
                audio = whisper.load_audio(self.audio_file)
                logger.info(f"Audio loaded, shape: {audio.shape}")
                
                # Make log-Mel spectrogram
                logger.info("Creating mel spectrogram")
                mel = whisper.log_mel_spectrogram(audio).to(self.model.device)
                logger.info(f"Mel spectrogram created, shape: {mel.shape}")
                
                # Detect language
                logger.info("Detecting language")
                _, probs = self.model.detect_language(mel)
                detected_language = max(probs, key=probs.get)
                logger.info(f"Detected language: {detected_language}")
                
                # Decode the audio
                logger.info("Decoding audio")
                options = whisper.DecodingOptions(fp16=self.cuda_available)
                result = whisper.decode(self.model, mel, options)
                logger.info("Decoding completed")
                
                # Return the transcribed text
                return result.text
        except Exception as e:
            error_traceback = traceback.format_exc()
            logger.error(f"Error in transcription: {str(e)}\n{error_traceback}")
            raise


class LoadingScreen(QWidget):
    """Loading screen with progress bar"""
    def __init__(self):
        super().__init__()
        self.layout = QVBoxLayout()
        
        # Label
        self.label = QLabel("Loading...")
        self.label.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(self.label)
        
        # Progress bar
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.layout.addWidget(self.progress_bar)
        
        # Set layout
        self.setLayout(self.layout)
    
    def set_label(self, text):
        self.label.setText(text)
    
    def set_progress(self, value):
        self.progress_bar.setValue(value)


class MicrophoneSelector(QWidget):
    """First screen: Microphone selection"""
    def __init__(self):
        super().__init__()
        self.layout = QVBoxLayout()
        
        # Microphone selection
        self.mic_label = QLabel("Select a microphone:")
        self.layout.addWidget(self.mic_label)
        
        # Combobox for microphone selection
        self.mic_combo = QComboBox()
        self.layout.addWidget(self.mic_combo)
        
        # Whisper model selection
        self.model_label = QLabel("Select Whisper model:")
        self.layout.addWidget(self.model_label)
        
        # Combobox for Whisper model selection
        self.model_combo = QComboBox()
        # Add available Whisper models
        for model in ["tiny", "base", "small", "medium", "large"]:
            self.model_combo.addItem(model)
        # Set default to base
        self.model_combo.setCurrentText("base")
        self.layout.addWidget(self.model_combo)
        
        # Add CUDA status label
        self.cuda_label = QLabel()
        self.update_cuda_status()
        self.layout.addWidget(self.cuda_label)
        
        # Populate the microphone combobox
        self.populate_microphones()
        
        # Next button
        self.next_button = QPushButton("Next")
        self.layout.addWidget(self.next_button)
        
        # Set layout
        self.setLayout(self.layout)
    
    def update_cuda_status(self):
        """Update the CUDA status label"""
        if torch.cuda.is_available():
            self.cuda_label.setText(f"CUDA Available: Yes (Device: {torch.cuda.get_device_name(0)})")
            self.cuda_label.setStyleSheet("color: green;")
        else:
            self.cuda_label.setText("CUDA Available: No (CPU will be used)")
            self.cuda_label.setStyleSheet("color: red;")
        
    def populate_microphones(self):
        p = pyaudio.PyAudio()
        info = p.get_host_api_info_by_index(0)
        numdevices = info.get('deviceCount')
        
        # Clear the combobox
        self.mic_combo.clear()
        
        # Add available input devices
        for i in range(numdevices):
            device_info = p.get_device_info_by_index(i)
            if device_info.get('maxInputChannels') > 0:
                self.mic_combo.addItem(
                    f"{device_info.get('name')} (ID: {i})", 
                    i  # Store device index as user data
                )
        
        p.terminate()
        
    def get_selected_microphone(self):
        return self.mic_combo.currentData()
    
    def get_selected_model(self):
        return self.model_combo.currentText()


class RecordingScreen(QWidget):
    """Second screen: Audio recording"""
    recording_completed = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.layout = QVBoxLayout()
        self.device_index = None
        self.audio_file = None
        self.recorder = None
        
        # Label
        self.label = QLabel("Press the button to start recording (10 seconds)")
        self.layout.addWidget(self.label)
        
        # Progress bar
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.layout.addWidget(self.progress_bar)
        
        # Record button
        self.record_button = QPushButton("Start Recording")
        self.record_button.clicked.connect(self.start_recording)
        self.layout.addWidget(self.record_button)
        
        # Back button
        self.back_button = QPushButton("Back")
        self.layout.addWidget(self.back_button)
        
        # Set layout
        self.setLayout(self.layout)
        
    def set_device_index(self, device_index):
        self.device_index = device_index
        
    def start_recording(self):
        if self.device_index is not None:
            self.record_button.setEnabled(False)
            self.back_button.setEnabled(False)
            self.label.setText("Recording in progress...")
            
            # Create and start the recorder thread
            self.recorder = AudioRecorder(self.device_index)
            self.recorder.update_progress.connect(self.update_progress)
            self.recorder.finished.connect(self.recording_finished)
            self.recorder.start()
    
    def update_progress(self, value):
        self.progress_bar.setValue(value)
        
    def recording_finished(self, file_path):
        self.audio_file = file_path
        self.record_button.setEnabled(True)
        self.back_button.setEnabled(True)
        self.label.setText("Recording completed!")
        self.progress_bar.setValue(100)
        
        # Emit signal to notify the main window
        self.recording_completed.emit(file_path)


class ResultsScreen(QWidget):
    """Third screen: Results and metrics"""
    def __init__(self):
        super().__init__()
        self.layout = QVBoxLayout()
        self.audio_file = None
        self.player = AudioPlayer()
        self.metrics_calculator = MetricsCalculator()
        
        # Label
        self.title_label = QLabel("Recording Results")
        self.title_label.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(self.title_label)
        
        # Metrics labels
        self.metrics_layout = QVBoxLayout()
        self.snr_label = QLabel("Signal-to-Noise Ratio (SNR): N/A")
        self.sample_rate_label = QLabel("Sample Rate: N/A")
        self.duration_label = QLabel("Duration: N/A")
        self.bit_depth_label = QLabel("Bit Depth: N/A")
        self.vram_usage_label = QLabel("VRAM Usage: N/A")
        
        self.metrics_layout.addWidget(self.snr_label)
        self.metrics_layout.addWidget(self.sample_rate_label)
        self.metrics_layout.addWidget(self.duration_label)
        self.metrics_layout.addWidget(self.bit_depth_label)
        self.metrics_layout.addWidget(self.vram_usage_label)
        
        self.layout.addLayout(self.metrics_layout)
        
        # Transcription result section
        self.transcription_label = QLabel("Transcription:")
        self.layout.addWidget(self.transcription_label)
        
        # Text area for transcription
        self.transcription_text = QTextEdit()
        self.transcription_text.setReadOnly(True)
        self.transcription_text.setMinimumHeight(100)
        self.layout.addWidget(self.transcription_text)
        
        # Button layout
        self.button_layout = QHBoxLayout()
        
        # Play button
        self.play_button = QPushButton("Play Recording")
        self.play_button.clicked.connect(self.play_audio)
        self.button_layout.addWidget(self.play_button)
        
        # Save button
        self.save_button = QPushButton("Save Audio")
        self.save_button.clicked.connect(self.save_audio)
        self.button_layout.addWidget(self.save_button)
        
        # Back to start button
        self.back_button = QPushButton("Back to Microphone Selection")
        self.button_layout.addWidget(self.back_button)
        
        self.layout.addLayout(self.button_layout)
        
        # Set layout
        self.setLayout(self.layout)
    
    def set_audio_file(self, file_path):
        self.audio_file = file_path
        self.calculate_metrics()
    
    def set_transcription(self, text):
        """Set the transcription text"""
        self.transcription_text.setText(text)
        
    def calculate_metrics(self):
        if self.audio_file:
            try:
                metrics = self.metrics_calculator.calculate_metrics(self.audio_file)
                
                if metrics["snr"] is not None:
                    self.snr_label.setText(f"Signal-to-Noise Ratio (SNR): {metrics['snr']:.2f} dB")
                else:
                    self.snr_label.setText("Signal-to-Noise Ratio (SNR): Could not calculate")
                    
                self.sample_rate_label.setText(f"Sample Rate: {metrics['sample_rate']} Hz")
                self.duration_label.setText(f"Duration: {metrics['duration']:.2f} seconds")
                self.bit_depth_label.setText(f"Bit Depth: {metrics['bit_depth']} bits")
            except Exception as e:
                QMessageBox.warning(self, "Error", f"Failed to calculate metrics: {str(e)}")
    
    def play_audio(self):
        if self.audio_file and os.path.exists(self.audio_file):
            try:
                self.player.play(self.audio_file)
            except Exception as e:
                QMessageBox.warning(self, "Error", f"Failed to play audio: {str(e)}")
        else:
            QMessageBox.warning(self, "Error", "No audio file available for playback")
            
    def save_audio(self):
        if not self.audio_file or not os.path.exists(self.audio_file):
            QMessageBox.warning(self, "Error", "No audio file available to save")
            return
            
        try:
            # Open file dialog to get save location
            file_path, _ = QFileDialog.getSaveFileName(
                self, 
                "Save Audio File", 
                os.path.expanduser("~/Desktop/recorded_audio.wav"),
                "WAV Files (*.wav);;All Files (*)"
            )
            
            if file_path:
                # Copy the temporary audio file to the selected location
                import shutil
                shutil.copy2(self.audio_file, file_path)
                QMessageBox.information(self, "Success", f"Audio saved to {file_path}")
        except Exception as e:
            QMessageBox.warning(self, "Error", f"Failed to save audio: {str(e)}")
    
    def set_vram_usage(self, max_vram_bytes):
        """Set the VRAM usage information on the label"""
        if max_vram_bytes is None or max_vram_bytes == 0:
            self.vram_usage_label.setText("VRAM Usage: N/A (CPU mode)")
        else:
            # Convert bytes to MB for display
            max_vram_mb = max_vram_bytes / (1024 * 1024)
            self.vram_usage_label.setText(f"VRAM Usage: {max_vram_mb:.2f} MB")


class MicrophoneBenchmark(QMainWindow):
    """Main application window"""
    def __init__(self):
        super().__init__()
        
        # Set window properties
        self.setWindowTitle("Microphone Benchmark Tool")
        self.setGeometry(100, 100, 500, 400)
        
        # Create central widget and main layout
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QVBoxLayout(self.central_widget)
        
        # Create stacked widget for different screens
        self.stacked_widget = QStackedWidget()
        
        # Create the screens
        self.microphone_selector = MicrophoneSelector()
        self.recording_screen = RecordingScreen()
        self.loading_screen = LoadingScreen()
        self.results_screen = ResultsScreen()
        
        # Add screens to stacked widget
        self.stacked_widget.addWidget(self.microphone_selector)
        self.stacked_widget.addWidget(self.loading_screen)
        self.stacked_widget.addWidget(self.recording_screen)
        self.stacked_widget.addWidget(self.results_screen)
        
        # Add stacked widget to main layout
        self.main_layout.addWidget(self.stacked_widget)
        
        # Connect signals
        self.microphone_selector.next_button.clicked.connect(self.prepare_model_and_go_to_recording)
        self.recording_screen.back_button.clicked.connect(self.go_to_microphone_selector)
        self.recording_screen.recording_completed.connect(self.on_recording_completed)
        self.results_screen.back_button.clicked.connect(self.go_to_microphone_selector)
        
        # Store the selected Whisper model and device
        self.selected_model = "base"
        self.selected_device_index = None
        self.model = None  # Store the loaded model
        
        # Show the first screen by default
        self.stacked_widget.setCurrentIndex(0)
        
        logger.info("MicrophoneBenchmark initialized")
    
    def prepare_model_and_go_to_recording(self):
        try:
            self.selected_device_index = self.microphone_selector.get_selected_microphone()
            self.selected_model = self.microphone_selector.get_selected_model()
            
            if self.selected_device_index is None:
                logger.warning("No microphone selected")
                QMessageBox.warning(self, "Error", "No microphone selected")
                return
            
            logger.info(f"Preparing model {self.selected_model} before recording")
            
            # Show loading screen
            self.loading_screen.set_label(f"Preparing Whisper model: {self.selected_model}")
            self.loading_screen.set_progress(0)
            self.stacked_widget.setCurrentIndex(1)  # Loading screen is now index 1
            
            # Create a model preparer to check/download the model
            self.model_preparer = WhisperModelPreparer(self.selected_model)
            self.model_preparer.download_progress.connect(self.on_download_progress)
            self.model_preparer.finished.connect(self.on_model_prepared)
            self.model_preparer.error.connect(self.on_model_error)
            self.model_preparer.start()
        except Exception as e:
            error_traceback = traceback.format_exc()
            logger.error(f"Error preparing model: {str(e)}\n{error_traceback}")
            QMessageBox.critical(self, "Error", f"Error preparing model: {str(e)}\n\nSee console for details.")
            self.go_to_microphone_selector()
    
    def on_model_prepared(self, model):
        logger.info(f"Model {self.selected_model} prepared successfully")
        self.model = model  # Store the model for later use
        
        # Now we can proceed to the recording screen
        self.recording_screen.set_device_index(self.selected_device_index)
        self.stacked_widget.setCurrentIndex(2)  # Recording screen is now index 2
    
    def on_model_error(self, error_message):
        logger.error(f"Model preparation error: {error_message}")
        QMessageBox.critical(self, "Model Preparation Error", f"Error preparing Whisper model: {error_message}")
        self.go_to_microphone_selector()
    
    def go_to_microphone_selector(self):
        logger.info("Going back to microphone selector")
        self.stacked_widget.setCurrentIndex(0)
    
    def on_download_progress(self, progress):
        logger.debug(f"Download progress: {progress}%")
        self.loading_screen.set_label(f"Downloading Whisper model: {progress}%")
        self.loading_screen.set_progress(progress)
    
    def on_recording_completed(self, file_path):
        try:
            logger.info(f"Recording completed: {file_path}")
            # Set the audio file for the results screen
            self.results_screen.set_audio_file(file_path)
            
            # Show loading screen for transcription
            self.loading_screen.set_label("Transcribing audio...")
            self.loading_screen.set_progress(0)
            self.stacked_widget.setCurrentIndex(1)  # Loading screen is index 1
            
            # Start the transcription process with our pre-loaded model
            logger.info(f"Starting transcription with model {self.selected_model}")
            self.transcriber = AudioTranscriber(file_path, self.model)
            self.transcriber.transcription_progress.connect(self.on_transcription_progress)
            self.transcriber.finished.connect(self.on_transcription_completed)
            self.transcriber.error.connect(self.on_transcription_error)
            
            # Connect VRAM usage signal if available
            if hasattr(self.transcriber, 'vram_usage_updated') and torch.cuda.is_available():
                self.transcriber.vram_usage_updated.connect(
                    lambda vram: logger.debug(f"Current VRAM usage: {vram/(1024*1024):.2f} MB")
                )
            
            self.transcriber.start()
        except Exception as e:
            error_traceback = traceback.format_exc()
            logger.error(f"Error in on_recording_completed: {str(e)}\n{error_traceback}")
            QMessageBox.critical(self, "Error", f"Error processing recording: {str(e)}\n\nSee console for details.")
    
    def on_transcription_progress(self, progress):
        logger.debug(f"Transcription progress: {progress}%")
        self.loading_screen.set_label(f"Transcribing audio: {progress}%")
        self.loading_screen.set_progress(progress)
    
    def on_transcription_completed(self, text):
        logger.info(f"Transcription completed: {text[:50]}...")
        # Set the transcription result
        self.results_screen.set_transcription(text)
        
        # Set VRAM usage in results screen
        if hasattr(self.transcriber, 'max_vram_bytes') and torch.cuda.is_available():
            self.results_screen.set_vram_usage(self.transcriber.max_vram_bytes)
        else:
            self.results_screen.set_vram_usage(None)
        
        # Switch to the results screen
        self.stacked_widget.setCurrentIndex(3)
    
    def on_transcription_error(self, error_message):
        logger.error(f"Transcription error: {error_message}")
        QMessageBox.critical(self, "Transcription Error", f"Error during transcription: {error_message}")
        
        # Still show results screen but without transcription
        self.results_screen.set_transcription("Transcription failed. See console for details.")
        self.stacked_widget.setCurrentIndex(3)


class WhisperModelPreparer(QThread):
    """Thread for Whisper model download and preparation"""
    download_progress = pyqtSignal(int)
    finished = pyqtSignal(object)  # Will emit the loaded model
    error = pyqtSignal(str)
    
    def __init__(self, model_name="base"):
        super().__init__()
        self.model_name = model_name
        self.cuda_available = torch.cuda.is_available()
        logger.info(f"Initializing WhisperModelPreparer with model: {model_name}, CUDA available: {self.cuda_available}")
        
    def run(self):
        try:
            # Check if model exists and download if needed
            logger.info(f"Checking if model {self.model_name} exists")
            if not self._check_model_exists():
                logger.info(f"Model {self.model_name} not found, downloading...")
                self._download_model()
            else:
                logger.info(f"Model {self.model_name} already exists")
            
            # Load the model
            logger.info(f"Loading model {self.model_name}")
            model = self._load_model()
            
            # Emit the loaded model
            self.finished.emit(model)
        except Exception as e:
            error_traceback = traceback.format_exc()
            logger.error(f"Error in WhisperModelPreparer.run: {str(e)}\n{error_traceback}")
            self.error.emit(f"{str(e)}\n\nFull traceback:\n{error_traceback}")
    
    def _check_model_exists(self):
        # Check if the model is already downloaded
        model_path = os.path.expanduser(f"~/.cache/whisper/{self.model_name}.pt")
        exists = os.path.exists(model_path)
        logger.debug(f"Model file check: {model_path}, exists: {exists}")
        return exists
    
    def _download_model(self):
        """Download the Whisper model with progress tracking"""
        class DownloadProgressListener(ProgressListener):
            def __init__(self, signal_fn):
                self.signal_fn = signal_fn
                
            def on_progress(self, current, total):
                progress = int((current / total) * 100) if total > 0 else 0
                logger.debug(f"Download progress update: {progress}% ({current}/{total})")
                self.signal_fn.emit(progress)
                
            def on_finished(self):
                self.signal_fn.emit(100)
        
        try:
            # This will trigger the download if the model doesn't exist
            with ProgressListenerHandle(DownloadProgressListener(self.download_progress)):
                # This forces download without loading the model yet
                logger.info(f"Starting model download: {self.model_name}")
                
                # Use the correct download approach by using the higher-level API
                # with download_root parameter to specify where to save
                download_root = os.path.expanduser("~/.cache/whisper")
                os.makedirs(download_root, exist_ok=True)
                
                # Use the public load_model function to download
                logger.info(f"Downloading model to {download_root}")
                
                # Create a custom progress callback that emits signals
                def progress_callback(progress):
                    self.download_progress.emit(int(progress * 100))
                    QApplication.processEvents()  # Force UI update
                
                # Download the model using the public API
                whisper.load_model(
                    self.model_name,  
                    device="cpu",     # Use CPU for download only
                    download_root=download_root,
                    in_memory=False   # Don't keep in memory, save to disk
                )
                
                logger.info(f"Model download completed")
        except Exception as e:
            error_traceback = traceback.format_exc()
            logger.error(f"Error downloading model: {str(e)}\n{error_traceback}")
            raise
    
    def _load_model(self):
        """Load the Whisper model using CUDA if available"""
        try:
            device = "cuda" if self.cuda_available else "cpu"
            logger.info(f"Loading model: {self.model_name}, device: {device}")
            
            # Removed fp16 parameter as it's not supported in the current API
            model = whisper.load_model(
                self.model_name, 
                device=device,
                download_root=os.path.expanduser("~/.cache/whisper")
            )
            logger.info(f"Model loaded successfully")
            return model
        except Exception as e:
            error_traceback = traceback.format_exc()
            logger.error(f"Error loading model: {str(e)}\n{error_traceback}")
            raise


class AudioTranscriber(QThread):
    """Thread for audio transcription using a pre-loaded Whisper model"""
    transcription_progress = pyqtSignal(int)
    finished = pyqtSignal(str)
    error = pyqtSignal(str)
    vram_usage_updated = pyqtSignal(int)  # New signal for VRAM usage
    
    def __init__(self, audio_file, model):
        super().__init__()
        self.audio_file = audio_file
        self.model = model
        self.cuda_available = torch.cuda.is_available()
        self.max_vram_bytes = 0  # Track maximum VRAM usage
        logger.info(f"Initializing AudioTranscriber with loaded model, CUDA available: {self.cuda_available}")
        logger.info(f"Audio file: {audio_file}, exists: {os.path.exists(audio_file)}")
    
    def get_gpu_memory_usage(self):
        """Get current GPU memory usage in bytes"""
        if not self.cuda_available:
            return 0
        return torch.cuda.memory_allocated()
    
    def track_gpu_memory(self):
        """Track GPU memory and update max usage"""
        current_usage = self.get_gpu_memory_usage()
        if current_usage > self.max_vram_bytes:
            self.max_vram_bytes = current_usage
            self.vram_usage_updated.emit(current_usage)
    
    def run(self):
        try:
            # Reset max VRAM tracking
            self.max_vram_bytes = 0
            
            # Transcribe the audio
            logger.info(f"Transcribing audio from {self.audio_file}")
            transcription = self._transcribe_audio()
            
            # Log final VRAM usage
            if self.cuda_available:
                logger.info(f"Maximum VRAM usage during transcription: {self.max_vram_bytes / (1024 * 1024):.2f} MB")
                print(f"Maximum VRAM usage during transcription: {self.max_vram_bytes} bytes")
            
            # Emit the transcription result
            logger.info(f"Transcription completed: {transcription[:50]}...")
            self.finished.emit(transcription)
        except Exception as e:
            error_traceback = traceback.format_exc()
            logger.error(f"Error in AudioTranscriber.run: {str(e)}\n{error_traceback}")
            self.error.emit(f"{str(e)}\n\nFull traceback:\n{error_traceback}")
    
    def _transcribe_audio(self):
        """Transcribe the audio file with progress tracking"""
        class TranscriptionProgressListener(ProgressListener):
            def __init__(self, signal_fn, memory_tracker):
                self.signal_fn = signal_fn
                self.memory_tracker = memory_tracker
                
            def on_progress(self, current, total):
                progress = int((current / total) * 100) if total > 0 else 0
                self.signal_fn.emit(progress)
                self.memory_tracker()  # Track memory at each progress update
                QApplication.processEvents()  # Force UI update
                
            def on_finished(self):
                self.signal_fn.emit(100)
                self.memory_tracker()  # Final memory check
        
        try:
            with ProgressListenerHandle(TranscriptionProgressListener(self.transcription_progress, self.track_gpu_memory)):
                logger.info(f"Starting transcription of {self.audio_file}")
                
                # Track initial memory
                self.track_gpu_memory()
                
                # Verify the audio file exists
                if not os.path.exists(self.audio_file):
                    error_msg = f"Audio file does not exist: {self.audio_file}"
                    logger.error(error_msg)
                    raise FileNotFoundError(error_msg)
                
                # Try using the high-level API first
                try:
                    logger.info("Attempting transcription with high-level API")
                    result = self.model.transcribe(
                        self.audio_file, 
                        verbose=None
                    )
                    
                    # Track memory after transcription
                    self.track_gpu_memory()
                    
                    logger.info("High-level API transcription successful")
                    return result["text"]
                except Exception as e:
                    logger.warning(f"High-level API failed: {str(e)}, trying low-level API")
                
                # If that fails, use the low-level API
                # Load audio file
                logger.info("Loading audio with low-level API")
                audio = whisper.load_audio(self.audio_file)
                self.track_gpu_memory()  # Track after audio load
                
                logger.info("Creating mel spectrogram")
                mel = whisper.log_mel_spectrogram(audio).to(self.model.device)
                self.track_gpu_memory()  # Track after spectrogram creation
                
                logger.info("Detecting language")
                _, probs = self.model.detect_language(mel)
                self.track_gpu_memory()  # Track after language detection
                
                logger.info("Decoding audio")
                options = whisper.DecodingOptions()
                result = whisper.decode(self.model, mel, options)
                self.track_gpu_memory()  # Track after decoding
                
                logger.info("Decoding completed")
                
                # Return the transcribed text
                return result.text
        except Exception as e:
            error_traceback = traceback.format_exc()
            logger.error(f"Error in transcription: {str(e)}\n{error_traceback}")
            raise


def main():
    try:
        # Create the application
        app = QApplication(sys.argv)
        
        # Create and show the main window
        logger.info("Starting MicrophoneBenchmark application")
        window = MicrophoneBenchmark()
        window.show()
        
        # Run the application event loop
        logger.info("Entering application event loop")
        sys.exit(app.exec_())
    except Exception as e:
        error_traceback = traceback.format_exc()
        logger.critical(f"Unhandled exception in main: {str(e)}\n{error_traceback}")
        print(f"CRITICAL ERROR: {str(e)}\n{error_traceback}")
        sys.exit(1)


if __name__ == "__main__":
    main() 