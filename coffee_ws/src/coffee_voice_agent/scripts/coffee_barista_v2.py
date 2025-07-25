#!/usr/bin/env python3
"""Coffee Barista Voice Agent v2 - Clean modular implementation

This is a refactored version of the original livekit_voice_agent.py that:
- Uses extracted services for wake word detection and order notifications  
- Uses a simple agent with function tools
- Uses emotion-aware TTS processing
- Maintains the same functionality with cleaner architecture
"""

import asyncio
import logging
import os
from datetime import datetime

from livekit import agents
from livekit.agents import AgentSession, JobContext, WorkerOptions
from livekit.plugins import openai, silero

# Import our extracted components
from config.settings import REQUIRED_ENV_VARS, WEBSOCKET_HOST, WEBSOCKET_PORT
from config.instructions import BARISTA_INSTRUCTIONS
from agents.simple_coffee_agent import SimpleCoffeeAgent
from services.emotion_service import EmotionStateManager
from services.wake_word_service import WakeWordService
from services.order_service import OrderNotificationService
from utils.greeting_data import get_random_greeting
from utils.announcement_data import format_virtual_request_announcement

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class CoffeeBaristaV2:
    """Clean coffee barista implementation using extracted services
    
    This version composes the extracted services to provide the same functionality
    as the original but with much cleaner architecture and separation of concerns.
    """
    
    def __init__(self):
        # Core components - emotion manager is handled by the agent now
        self.emotion_manager = EmotionStateManager()
        self.agent = SimpleCoffeeAgent(emotion_manager=self.emotion_manager)
        
        # I/O Services
        self.wake_word_service = WakeWordService(on_wake_word_detected=self.on_wake_word_detected)
        self.order_service = OrderNotificationService(on_order_received=self.on_order_received)
        
        # Session management
        self.current_session = None
        self.room = None
        
        logger.info("CoffeeBaristaV2 initialized with modular architecture")
    
    async def start(self, ctx: JobContext):
        """Start the coffee barista with all services"""
        # Connect to the room
        await ctx.connect()
        self.room = ctx.room
        logger.info(f"Connected to room: {ctx.room.name}")
        
        # Start I/O services
        await self.order_service.start()
        wake_word_started = await self.wake_word_service.start(ctx.room)
        
        # Handle wake word vs always-on mode
        if not wake_word_started:
            logger.info("🔍 Starting in always-on mode (no wake word detection)")
            await self.start_conversation()
        else:
            logger.info("Started in wake word mode - say 'hey barista' to activate")
    
    async def on_wake_word_detected(self, room):
        """Handle wake word detection - start a conversation"""
        logger.info("🔍 Wake word detected - starting conversation")
        await self.start_conversation()
    
    async def start_conversation(self):
        """Start a conversation session with emotion-aware TTS"""
        try:
            # Create session with standard TTS (emotion processing happens in agent.tts_node)
            self.current_session = AgentSession(
                stt=openai.STT(model="whisper-1"),
                llm=openai.LLM(
                    model="gpt-4o-mini",
                    temperature=float(os.getenv("VOICE_AGENT_TEMPERATURE", "0.7"))
                ),
                tts=openai.TTS(
                    model="tts-1",
                    voice=os.getenv("VOICE_AGENT_VOICE", "nova")
                ),
                vad=silero.VAD.load(),
            )
            
            # Start the session with our simple agent
            await self.current_session.start(
                room=self.room,
                agent=self.agent
            )
            
            # Pause wake word detection during conversation
            if self.wake_word_service.is_active():
                self.wake_word_service.pause()
            
            # Start with a random greeting
            greeting = get_random_greeting()
            emotion, text = self.emotion_manager.process_emotional_response(greeting)
            
            # Use the session to say the greeting
            await self.current_session.say(text)
            
            logger.info("🎉 Conversation started successfully")
            
        except Exception as e:
            logger.error(f"Error starting conversation: {e}")
            await self.end_conversation()
    
    async def on_order_received(self, order_info):
        """Handle order notifications from WebSocket"""
        try:
            # Format the order announcement using our utility
            announcement = format_virtual_request_announcement({
                "type": order_info["type"],
                "content": order_info["content"]
            })
            
            # Process emotion and announce if we have an active session
            if self.current_session:
                emotion, text = self.emotion_manager.process_emotional_response(announcement)
                await self.current_session.say(text)
                logger.info(f"📢 Announced order: {order_info['coffee_type']}")
            else:
                logger.info(f"📋 Order received but no active session: {order_info['coffee_type']}")
                
        except Exception as e:
            logger.error(f"Error processing order notification: {e}")
    
    async def end_conversation(self):
        """End the current conversation and return to dormant state"""
        if self.current_session:
            try:
                await self.current_session.aclose()
            except Exception as e:
                logger.error(f"Error closing session: {e}")
            finally:
                self.current_session = None
        
        # Resume wake word detection
        if self.wake_word_service.is_active():
            self.wake_word_service.resume()
        
        # Reset emotion state for next conversation
        self.emotion_manager.reset_emotion_state()
        
        logger.info("🔍 Conversation ended - returned to dormant state")
    
    def stop(self):
        """Stop all services and clean up resources"""
        logger.info("🛑 Stopping Coffee Barista v2...")
        
        # Stop I/O services
        self.wake_word_service.stop()
        self.order_service.stop()
        
        # Close any active session
        if self.current_session:
            # Note: In a real implementation, this would need proper async cleanup
            logger.info("Closing active session...")
        
        logger.info("✅ Coffee Barista v2 stopped")
    
    def __repr__(self):
        return f"CoffeeBaristaV2(session_active={self.current_session is not None}, emotion={self.emotion_manager.get_current_emotion()})"


async def entrypoint(ctx: JobContext):
    """Main entrypoint for the coffee barista agent v2"""
    barista = CoffeeBaristaV2()
    await barista.start(ctx)


def main():
    """Main function with environment validation and startup"""
    # Validate required environment variables
    missing_vars = [var for var in REQUIRED_ENV_VARS if not os.getenv(var)]
    
    if missing_vars:
        logger.error(f"Missing required environment variables: {missing_vars}")
        logger.error("Please check your .env file and ensure OPENAI_API_KEY is set.")
        exit(1)
    
    # Log configuration
    logger.info("☕ Starting Coffee Barista Voice Agent v2...")
    logger.info(f"Wake Word Detection: {'✅ Enabled' if os.getenv('PORCUPINE_ACCESS_KEY') else '❌ Disabled (always-on mode)'}")
    logger.info(f"WebSocket Server: ✅ Enabled on {WEBSOCKET_HOST}:{WEBSOCKET_PORT}")
    logger.info(f"OpenAI Model: gpt-4o-mini")
    logger.info(f"Voice: {os.getenv('VOICE_AGENT_VOICE', 'nova')}")
    logger.info(f"Temperature: {os.getenv('VOICE_AGENT_TEMPERATURE', '0.7')}")
    logger.info(f"Architecture: 🏗️ Modular (extracted services)")
    
    logger.info("\n📋 Available CLI modes:")
    logger.info("  python coffee_barista_v2.py console  - Terminal mode (local testing)")
    logger.info("  python coffee_barista_v2.py dev      - Development mode (connect to LiveKit)")
    logger.info("  python coffee_barista_v2.py start    - Production mode")
    
    # Run the agent
    agents.cli.run_app(
        WorkerOptions(
            entrypoint_fnc=entrypoint,
            agent_name="coffee-barista-v2"
        )
    )


if __name__ == "__main__":
    main() 