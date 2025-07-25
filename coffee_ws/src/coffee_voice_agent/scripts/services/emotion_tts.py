"""Emotion-aware TTS service that processes emotional responses before speech synthesis"""

import logging
from typing import AsyncIterator

from livekit.plugins import openai
from services.emotion_service import EmotionStateManager

logger = logging.getLogger(__name__)


class EmotionTTS:
    """Clean TTS wrapper that handles emotion processing
    
    This is much simpler than the complex streaming TTS override in the original.
    It processes complete text, extracts emotions, and delegates to standard TTS.
    """
    
    def __init__(self, base_tts=None, emotion_manager=None):
        # Use OpenAI TTS as the base implementation
        self.base_tts = base_tts or openai.TTS(
            model="tts-1",
            voice="nova"  # Default voice, can be overridden via env vars
        )
        
        # Use provided emotion manager or create a new one
        self.emotion_manager = emotion_manager or EmotionStateManager()
        
        logger.info("EmotionTTS initialized with base TTS and emotion processing")
    
    async def synthesize(self, text: str) -> AsyncIterator[bytes]:
        """Synthesize speech with emotion processing
        
        Args:
            text: The text to synthesize (may contain emotion:text format)
            
        Yields:
            Audio frames from the base TTS
        """
        try:
            # Process the text for emotions (this handles the emotion:text format)
            emotion, clean_text = self.emotion_manager.process_emotional_response(text)
            
            logger.info(f"🎭 Synthesizing with emotion: {emotion}")
            logger.info(f"💬 Clean text: {clean_text[:50]}{'...' if len(clean_text) > 50 else ''}")
            
            # Use the base TTS to synthesize the clean text
            async for audio_frame in self.base_tts.synthesize(clean_text):
                yield audio_frame
                
        except Exception as e:
            logger.error(f"Error in emotion TTS: {e}")
            # Fallback: synthesize the original text without emotion processing
            async for audio_frame in self.base_tts.synthesize(text):
                yield audio_frame
    
    def get_current_emotion(self) -> str:
        """Get the current emotional state"""
        return self.emotion_manager.get_current_emotion()
    
    def get_emotion_history(self):
        """Get the emotion history"""
        return self.emotion_manager.get_emotion_history()
    
    def reset_emotion_state(self):
        """Reset the emotional state"""
        self.emotion_manager.reset_emotion_state()
    
    def __repr__(self):
        return f"EmotionTTS(base_tts={type(self.base_tts).__name__}, current_emotion={self.get_current_emotion()})" 