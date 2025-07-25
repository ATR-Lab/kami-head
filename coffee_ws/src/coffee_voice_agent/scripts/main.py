#!/usr/bin/env python3
"""Coffee Barista Voice Agent - Main Entry Point

This is the refactored entry point for the coffee barista agent.
It uses the extracted StateManager and CoffeeBaristaAgent classes.
"""

import logging
import os

from livekit import agents
from livekit.agents import JobContext, WorkerOptions

from config.settings import REQUIRED_ENV_VARS, WEBSOCKET_HOST, WEBSOCKET_PORT
from agents.simple_coffee_agent import CoffeeBaristaAgent
from state.state_manager import AgentState

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


async def entrypoint(ctx: JobContext):
    """Main entrypoint for the coffee barista agent"""
    
    # Connect to the room
    await ctx.connect()
    logger.info(f"Connected to room: {ctx.room.name}")
    
    # Create the coffee barista agent
    agent = CoffeeBaristaAgent()
    
    # Set context in state manager
    agent.state_manager.ctx = ctx
    
    # Start WebSocket server for order notifications
    await agent.start_websocket_server()
    
    # Start wake word detection
    await agent.start_wake_word_detection(ctx.room)
    
    # If no wake word detection, start with always-on mode
    if not agent.porcupine_access_key:
        logger.info("🔍 DEBUG: Starting in always-on mode")
        # Create session immediately for always-on mode
        await agent.state_manager.transition_to_state(AgentState.CONNECTING)
        session = await agent.state_manager.create_session(agent)
        await agent.state_manager.transition_to_state(AgentState.ACTIVE)
        
        # Get random greeting for always-on mode
        greeting = agent.state_manager.get_random_greeting()
        
        logger.info("🔍 DEBUG: About to call process_emotional_response and say_with_emotion (ALWAYS-ON MANUAL TTS)")
        # Process the emotional response
        emotion, text = agent.state_manager.process_emotional_response(greeting)
        await agent.state_manager.say_with_emotion(text, emotion)
        logger.info("🔍 DEBUG: Always-on manual TTS call completed")
    else:
        logger.info("Started in wake word mode - say 'hey barista' to activate")
        # Stay in dormant state, waiting for wake word


def main():
    """Main function with environment validation and startup"""
    # Validate required environment variables
    missing_vars = [var for var in REQUIRED_ENV_VARS if not os.getenv(var)]
    
    if missing_vars:
        logger.error(f"Missing required environment variables: {missing_vars}")
        logger.error("Please check your .env file and ensure OPENAI_API_KEY is set.")
        exit(1)
    
    # Log configuration
    logger.info("☕ Starting Coffee Barista Voice Agent (Refactored)...")
    logger.info(f"Wake Word Detection: {'✅ Enabled' if os.getenv('PORCUPINE_ACCESS_KEY') else '❌ Disabled (always-on mode)'}")
    logger.info(f"WebSocket Server: ✅ Enabled on {WEBSOCKET_HOST}:{WEBSOCKET_PORT}")
    logger.info(f"OpenAI Model: gpt-4o-mini")
    logger.info(f"Voice: {os.getenv('VOICE_AGENT_VOICE', 'nova')}")
    logger.info(f"Temperature: {os.getenv('VOICE_AGENT_TEMPERATURE', '0.7')}")
    logger.info(f"Architecture: 🏗️ File-based modular (StateManager + CoffeeBaristaAgent)")
    
    logger.info("\n📋 Available CLI modes:")
    logger.info("  python main.py console  - Terminal mode (local testing)")
    logger.info("  python main.py dev      - Development mode (connect to LiveKit)")
    logger.info("  python main.py start    - Production mode")
    
    # Run the agent
    agents.cli.run_app(
        WorkerOptions(
            entrypoint_fnc=entrypoint,
            agent_name="coffee-barista-refactored"
        )
    )


if __name__ == "__main__":
    main() 