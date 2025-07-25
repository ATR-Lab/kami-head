"""Simple coffee agent with emotion-aware TTS processing"""

import logging
from livekit.agents import Agent, function_tool
from services.emotion_service import EmotionStateManager

from config.instructions import BARISTA_INSTRUCTIONS
from tools.coffee_tools import (
    get_current_time_impl, get_current_date_impl, get_coffee_menu_impl,
    get_ordering_instructions_impl, recommend_drink_impl
)

logger = logging.getLogger(__name__)


class SimpleCoffeeAgent(Agent):
    """Coffee barista agent with emotion-aware TTS processing

    This agent follows the proven pattern from the original implementation:
    - Standard LiveKit Agent with instructions and function tools (programmatic registration)
    - Override tts_node to handle emotion:text format processing
    - Clean separation of concerns with emotion service
    """

    def __init__(self, emotion_manager=None):
        # Use provided emotion manager or create a new one
        self.emotion_manager = emotion_manager or EmotionStateManager()
        
        # Initialize with instructions and programmatically registered tools
        super().__init__(
            instructions=BARISTA_INSTRUCTIONS,
            tools=[
                function_tool(
                    get_current_time_impl,
                    name="get_current_time",
                    description="Get the current time."
                ),
                function_tool(
                    get_current_date_impl,
                    name="get_current_date",
                    description="Get today's date."
                ),
                function_tool(
                    get_coffee_menu_impl,
                    name="get_coffee_menu",
                    description="Get the Sui Hub coffee menu."
                ),
                function_tool(
                    get_ordering_instructions_impl,
                    name="get_ordering_instructions",
                    description="Get instructions on how to order coffee through the Slush wallet and Coffee Hub website."
                ),
                function_tool(
                    recommend_drink_impl,
                    name="recommend_drink",
                    description="Recommend a drink based on user preference."
                ),
            ]
        )
        
        logger.info("SimpleCoffeeAgent initialized with emotion-aware TTS processing and 5 programmatically registered tools")

    async def tts_node(self, text, model_settings=None):
        """Override TTS node to process emotion:text format (same pattern as original)"""
        
        # Process text stream with minimal buffering for emotion extraction
        async def process_text_stream():
            first_chunk_buffer = ""
            emotion_extracted = False
            emotion_check_limit = 50  # Only check first 50 characters for emotion delimiter
            chunks_processed = 0
            
            async for text_chunk in text:
                if not text_chunk:
                    continue

                chunks_processed += 1
                
                # Only buffer and check for emotion in the very first chunk(s)
                if not emotion_extracted and len(first_chunk_buffer) < emotion_check_limit:
                    first_chunk_buffer += text_chunk
                    
                    # Check if we have delimiter in the buffered portion
                    if ":" in first_chunk_buffer:
                        logger.info("🔍 Found delimiter in first chunk(s)! Extracting emotion...")
                        
                        # Process emotion using our emotion service
                        emotion, text_after_delimiter = self.emotion_manager.process_emotional_response(first_chunk_buffer)
                        
                        logger.info(f"🎭 Agent speaking with emotion: {emotion}")
                        
                        # Mark emotion as extracted
                        emotion_extracted = True
                        
                        # Immediately yield the text part (no more buffering)
                        if text_after_delimiter.strip():
                            logger.info(f"💬 TTS streaming text immediately: {text_after_delimiter[:30]}{'...' if len(text_after_delimiter) > 30 else ''}")
                            yield text_after_delimiter
                        
                    elif len(first_chunk_buffer) >= emotion_check_limit:
                        # Reached limit without finding delimiter - give up and stream everything
                        logger.info("🔍 No delimiter found within limit, streaming everything with default emotion")
                        
                        # Process with default emotion
                        emotion, processed_text = self.emotion_manager.process_emotional_response(first_chunk_buffer)
                        
                        emotion_extracted = True
                        
                        # Yield the processed content immediately
                        logger.info(f"💬 TTS fallback streaming: {processed_text[:30]}{'...' if len(processed_text) > 30 else ''}")
                        yield processed_text
                    
                    # If we haven't extracted emotion yet and haven't hit limit, continue buffering
                    # (don't yield anything yet)
                    
                else:
                    # Either emotion already extracted, or we're past the check limit
                    # Stream everything immediately
                    yield text_chunk
        
        # Process the text stream and pass clean text to default TTS
        processed_text = process_text_stream()
        
        # Use default TTS implementation with processed text
        async for audio_frame in Agent.default.tts_node(self, processed_text, model_settings):
            yield audio_frame

    # Function tools are automatically discovered by LiveKit from the imported functions
    # No need to manually register them - the @function_tool decorators handle this

    def get_emotion_manager(self):
        """Get the emotion manager for external access"""
        return self.emotion_manager

    def __repr__(self):
        return f"SimpleCoffeeAgent(tools=5, emotion={self.emotion_manager.get_current_emotion()}, registration=programmatic)" 