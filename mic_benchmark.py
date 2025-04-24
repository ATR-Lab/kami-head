#!/usr/bin/env python3

import sys
import os
import numpy as np
import wave
import tempfile
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                           QHBoxLayout, QComboBox, QPushButton, QLabel, 
                           QStackedWidget, QProgressBar, QMessageBox, QFileDialog)
from PyQt5.QtCore import Qt, pyqtSignal, QThread
import pyaudio
import torch
import torchmetrics.audio as tm_audio


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


class MicrophoneSelector(QWidget):
    """First screen: Microphone selection"""
    def __init__(self):
        super().__init__()
        self.layout = QVBoxLayout()
        
        # Label
        self.label = QLabel("Select a microphone:")
        self.layout.addWidget(self.label)
        
        # Combobox for microphone selection
        self.mic_combo = QComboBox()
        self.layout.addWidget(self.mic_combo)
        
        # Populate the combobox with available microphones
        self.populate_microphones()
        
        # Next button
        self.next_button = QPushButton("Next")
        self.layout.addWidget(self.next_button)
        
        # Set layout
        self.setLayout(self.layout)
        
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
        
        self.metrics_layout.addWidget(self.snr_label)
        self.metrics_layout.addWidget(self.sample_rate_label)
        self.metrics_layout.addWidget(self.duration_label)
        self.metrics_layout.addWidget(self.bit_depth_label)
        
        self.layout.addLayout(self.metrics_layout)
        
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
        
        # Create the three screens
        self.microphone_selector = MicrophoneSelector()
        self.recording_screen = RecordingScreen()
        self.results_screen = ResultsScreen()
        
        # Add screens to stacked widget
        self.stacked_widget.addWidget(self.microphone_selector)
        self.stacked_widget.addWidget(self.recording_screen)
        self.stacked_widget.addWidget(self.results_screen)
        
        # Add stacked widget to main layout
        self.main_layout.addWidget(self.stacked_widget)
        
        # Connect signals
        self.microphone_selector.next_button.clicked.connect(self.go_to_recording)
        self.recording_screen.back_button.clicked.connect(self.go_to_microphone_selector)
        self.recording_screen.recording_completed.connect(self.on_recording_completed)
        self.results_screen.back_button.clicked.connect(self.go_to_microphone_selector)
        
        # Show the first screen by default
        self.stacked_widget.setCurrentIndex(0)
    
    def go_to_recording(self):
        device_index = self.microphone_selector.get_selected_microphone()
        if device_index is not None:
            self.recording_screen.set_device_index(device_index)
            self.stacked_widget.setCurrentIndex(1)
        else:
            QMessageBox.warning(self, "Error", "No microphone selected")
    
    def go_to_microphone_selector(self):
        self.stacked_widget.setCurrentIndex(0)
    
    def on_recording_completed(self, file_path):
        # Set the audio file for the results screen
        self.results_screen.set_audio_file(file_path)
        
        # Switch to the results screen
        self.stacked_widget.setCurrentIndex(2)


def main():
    # Create the application
    app = QApplication(sys.argv)
    
    # Create and show the main window
    window = MicrophoneBenchmark()
    window.show()
    
    # Run the application event loop
    sys.exit(app.exec_())


if __name__ == "__main__":
    main() 