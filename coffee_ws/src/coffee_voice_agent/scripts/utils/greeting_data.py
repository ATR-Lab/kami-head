"""Greeting data constants for the coffee agent"""

import random
import logging

logger = logging.getLogger(__name__)

# Pool of greeting messages for the coffee agent
GREETING_POOL = [
    "excited:Hey there! Welcome to the Sui Hub Grand Opening in Athens! I'm your friendly coffee consultant robot. How can I help you today?",
    "friendly:Hello! I'm your coffee consultant at the Sui Hub Grand Opening! Ready to help with coffee information and recommendations. How can I help you today?",
    "enthusiastic:Welcome to our amazing blockchain coffee experience! I'm here to help with all your coffee questions and guide you through our ordering process!",
    "cheerful:Great to see you! What coffee wisdom can I share with you today?",
    "warm:Welcome to our coffee command center! How can I help caffeinate your conference experience?",
    "professional:Welcome to the Sui Hub! I'm your dedicated blockchain coffee consultant. Ready to help with coffee info and recommendations!",
    "curious:Hello! Ready for some blockchain coffee knowledge? What sounds interesting to you?",
    "playful:Hey there! Time for some coffee consultation! What can I tell you about our amazing brews?",
    "helpful:Welcome! Perfect timing for coffee info. How can I help guide your caffeine journey?",
    "friendly:Hi! Welcome back to our bustling coffee hub! What coffee questions are brewing on your mind!"
]


def get_random_greeting() -> str:
    """Get a random greeting from the greeting pool"""
    selected_greeting = random.choice(GREETING_POOL)
    logger.info(f"🎭 Selected random greeting: {selected_greeting[:50]}...")
    return selected_greeting 