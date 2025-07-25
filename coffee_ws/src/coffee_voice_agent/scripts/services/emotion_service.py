"""Emotion processing service for parsing and managing emotional states"""

import json
import logging
from datetime import datetime
from typing import Tuple, List, Dict, Any

from config.settings import VALID_EMOTIONS
from utils.animation_data import log_animated_eyes

logger = logging.getLogger(__name__)


def parse_emotional_response(llm_response: str) -> Tuple[str, str]:
    """Parse LLM response and extract emotion + text from delimiter format (emotion:text)
    
    This is a pure function that doesn't manage state - just parses the response.
    
    Args:
        llm_response: The response from the LLM to parse
        
    Returns:
        Tuple of (emotion, text) where emotion is validated against VALID_EMOTIONS
    """
    try:
        # Check if response uses delimiter format (emotion:text)
        if ":" in llm_response:
            # Split on first colon
            parts = llm_response.split(":", 1)
            emotion = parts[0].strip()
            text = parts[1].strip() if len(parts) > 1 else ""
            
            logger.info(f"🔍 DEBUG: Delimiter format detected - emotion: '{emotion}', text: '{text[:50]}{'...' if len(text) > 50 else ''}'")
            
        else:
            # Try to parse as JSON (legacy format)
            try:
                response_data = json.loads(llm_response)
                emotion = response_data.get("emotion", "friendly")
                text = response_data.get("text", "")
                logger.info("🔍 DEBUG: JSON format detected (legacy)")
            except json.JSONDecodeError:
                # Fallback: treat entire response as text with default emotion
                logger.warning("No delimiter or JSON format found, using fallback")
                emotion = "friendly"
                text = llm_response
        
        # Validate emotion is in our supported set
        if emotion not in VALID_EMOTIONS:
            logger.warning(f"Unknown emotion '{emotion}', defaulting to 'friendly'")
            emotion = "friendly"
        
        return emotion, text
        
    except Exception as e:
        logger.error(f"Error parsing emotional response: {e}")
        return "friendly", llm_response


class EmotionStateManager:
    """Manages emotional state transitions and history"""
    
    def __init__(self, initial_emotion: str = "waiting"):
        self.current_emotion = initial_emotion
        self.emotion_history: List[Dict[str, Any]] = []
    
    def process_emotional_response(self, llm_response: str) -> Tuple[str, str]:
        """Process LLM response with state management - same interface as original method"""
        # Parse the emotion and text
        emotion, text = parse_emotional_response(llm_response)
        
        # Handle state transitions (same logic as original)
        if emotion != self.current_emotion:
            logger.info(f"🎭 Emotion transition: {self.current_emotion} → {emotion}")
            log_animated_eyes(emotion)  # Use our existing utility
            self.current_emotion = emotion
            
            # Store in emotion history (same logic as original)
            self.emotion_history.append({
                'timestamp': datetime.now(),
                'emotion': emotion,
                'text_preview': text[:50] + "..." if len(text) > 50 else text
            })
        
        return emotion, text
    
    def get_current_emotion(self) -> str:
        """Get the current emotional state"""
        return self.current_emotion
    
    def get_emotion_history(self) -> List[Dict[str, Any]]:
        """Get the complete emotion history"""
        return self.emotion_history.copy()
    
    def reset_emotion_state(self):
        """Reset emotional state to initial values"""
        self.current_emotion = "waiting"
        self.emotion_history.clear()
        logger.info("🎭 Emotion state reset to 'waiting'") 