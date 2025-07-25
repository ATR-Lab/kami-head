"""Wake word detection service using Porcupine for 'hey barista' detection"""

import asyncio
import logging
import os
import threading
from typing import Callable, Optional

import pvporcupine
from pvrecorder import PvRecorder

logger = logging.getLogger(__name__)


class WakeWordService:
    """Wake word detection service that extracts the proven Porcupine integration
    
    This service maintains the exact same threading approach and resource management
    that works in the original implementation.
    """
    
    def __init__(self, on_wake_word_detected: Optional[Callable] = None):
        # Configuration
        self.porcupine_access_key = os.getenv("PORCUPINE_ACCESS_KEY")
        self.wake_word = "hey barista"
        
        # Callback for when wake word is detected
        self.on_wake_word_detected = on_wake_word_detected
        
        # Porcupine resources
        self.porcupine = None
        self.recorder = None
        
        # Threading control
        self.wake_word_thread = None
        self.wake_word_active = False
        self.wake_word_paused = False
        self.event_loop = None
        
        logger.info(f"WakeWordService initialized - Access key: {'✅ Available' if self.porcupine_access_key else '❌ Missing'}")
    
    async def start(self, room=None):
        """Start wake word detection in a separate thread (same logic as original)"""
        if not self.porcupine_access_key:
            logger.info("No Porcupine access key found, skipping wake word detection")
            return False
            
        try:
            # Initialize Porcupine with "hey barista" wake word (same as original)
            self.porcupine = pvporcupine.create(
                access_key=self.porcupine_access_key,
                keywords=[self.wake_word]
            )
            
            # Initialize recorder (same as original)
            self.recorder = PvRecorder(
                device_index=-1,  # default device
                frame_length=self.porcupine.frame_length
            )
            
            self.wake_word_active = True
            self.event_loop = asyncio.get_event_loop()
            
            # Start wake word detection in separate thread (same as original)
            self.wake_word_thread = threading.Thread(
                target=self._wake_word_detection_loop,
                args=(room,),
                daemon=True
            )
            self.wake_word_thread.start()
            
            logger.info(f"Wake word detection started - listening for '{self.wake_word}'")
            return True
            
        except Exception as e:
            logger.error(f"Failed to start wake word detection: {e}")
            return False
    
    def _wake_word_detection_loop(self, room):
        """Wake word detection loop running in separate thread (same logic as original)"""
        try:
            self.recorder.start()
            
            while self.wake_word_active:
                if self.wake_word_paused:
                    # Sleep briefly when paused to avoid busy waiting (same as original)
                    threading.Event().wait(0.1)
                    continue
                    
                pcm = self.recorder.read()
                result = self.porcupine.process(pcm)
                
                if result >= 0:  # Wake word detected
                    logger.info(f"Wake word '{self.wake_word}' detected!")
                    
                    # Trigger callback if provided
                    if self.on_wake_word_detected:
                        # Use thread-safe method to trigger callback (same as original)
                        asyncio.run_coroutine_threadsafe(
                            self.on_wake_word_detected(room), 
                            self.event_loop
                        )
                    
        except Exception as e:
            logger.error(f"Wake word detection error: {e}")
        finally:
            if self.recorder:
                self.recorder.stop()
    
    def stop(self):
        """Stop wake word detection (same cleanup logic as original)"""
        self.wake_word_active = False
        self.wake_word_paused = False
        
        if self.wake_word_thread and self.wake_word_thread.is_alive():
            self.wake_word_thread.join(timeout=2.0)
            
        if self.recorder:
            try:
                self.recorder.stop()
                self.recorder.delete()
            except:
                pass
                
        if self.porcupine:
            try:
                self.porcupine.delete()
            except:
                pass
        
        logger.info("Wake word detection stopped")
    
    def pause(self):
        """Pause wake word detection (e.g., during conversation)"""
        self.wake_word_paused = True
        logger.info("Wake word detection paused")
    
    def resume(self):
        """Resume wake word detection"""
        self.wake_word_paused = False
        logger.info("Wake word detection resumed")
    
    def is_active(self) -> bool:
        """Check if wake word detection is active"""
        return self.wake_word_active
    
    def is_paused(self) -> bool:
        """Check if wake word detection is paused"""
        return self.wake_word_paused
    
    def set_callback(self, callback: Callable):
        """Set or update the wake word detection callback"""
        self.on_wake_word_detected = callback
        logger.info("Wake word detection callback updated")
    
    def __repr__(self):
        return f"WakeWordService(keyword='{self.wake_word}', active={self.wake_word_active}, paused={self.wake_word_paused})" 