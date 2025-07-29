"""Coffee-related function implementations for programmatic tool registration"""

import asyncio
import logging
from datetime import datetime
from livekit.agents import RunContext

logger = logging.getLogger(__name__)

# Global reference to the agent for tool event sending
_agent_instance = None

def set_agent_instance(agent):
    """Set the agent instance for tool event sending"""
    global _agent_instance
    _agent_instance = agent

async def send_tool_event(tool_name: str, status: str, parameters: list = None, result: str = ""):
    """Send tool event through the agent's state manager"""
    if _agent_instance and hasattr(_agent_instance, 'state_manager'):
        try:
            await _agent_instance.state_manager._send_tool_event(tool_name, status, parameters, result)
        except Exception as e:
            logger.error(f"Error sending tool event: {e}")
    else:
        logger.debug(f"Cannot send tool event - no agent instance available")


async def get_current_time_impl(context: RunContext) -> str:
    """Get the current time."""
    await send_tool_event("get_current_time", "started")
    
    current_time = datetime.now().strftime("%I:%M %p")
    result = f"The current time is {current_time}"
    logger.info(f"Time requested: {current_time}")
    
    await send_tool_event("get_current_time", "completed", [], result)
    return result


async def get_current_date_impl(context: RunContext) -> str:
    """Get today's date."""
    await send_tool_event("get_current_date", "started")
    
    current_date = datetime.now().strftime("%A, %B %d, %Y")
    result = f"Today's date is {current_date}"
    logger.info(f"Date requested: {current_date}")
    
    await send_tool_event("get_current_date", "completed", [], result)
    return result


async def get_coffee_menu_impl(context: RunContext) -> str:
    """Get the Sui Hub coffee menu."""
    await send_tool_event("get_coffee_menu", "started")
    
    menu = """🚀 SUI HUB COFFEE MENU ☕

    ☕ CLASSIC BREWS:
    - Espresso
    - Black Coffee
    - Americano
    - Long Black
    
    📱 TO ORDER: Open your Slush wallet and go to the Coffee Hub website to place your order!
    
    All drinks come with complimentary blockchain wisdom! 🤖"""
    
    logger.info("Coffee menu requested")
    
    await send_tool_event("get_coffee_menu", "completed", [], menu)
    return menu


async def get_ordering_instructions_impl(context: RunContext) -> str:
    """Get instructions on how to order coffee through the Slush wallet and Coffee Hub website."""
    await send_tool_event("get_ordering_instructions", "started")
    
    instructions = """📱 HOW TO ORDER COFFEE:
    
    1. 📲 Open your Slush wallet
    2. 🌐 Navigate to the Coffee Hub website
    3. ☕ Browse our amazing coffee menu
    4. 🛒 Select your desired drinks
    5. 💳 Complete your order
    6. ⏰ We'll notify you when it's ready!
    
    🎉 It's that easy! Your blockchain-powered coffee experience awaits!
    
    Need help with your Slush wallet? Just ask John or George for assistance! 🤖"""
    
    logger.info("Ordering instructions requested")
    
    await send_tool_event("get_ordering_instructions", "completed", [], instructions)
    return instructions


async def recommend_drink_impl(context: RunContext, preference: str = "energizing") -> str:
    """Recommend a drink based on user preference.
    
    Args:
        preference: Type of drink preference (energizing, smooth, sweet, cold, etc.)
    """
    await send_tool_event("recommend_drink", "started", [preference])
    
    recommendations = {
        "energizing": "I recommend our Espresso! It's a strong shot that'll keep you alert during those blockchain presentations. ⚡",
        "smooth": "Try our Long Black! It's smooth and bold, perfect for networking sessions. 🔥",
        "sweet": "Our Black Coffee is perfect for you! It's rich and comforting, great for savoring the conference atmosphere. ☕",
        "cold": "How about our Americano? It's refreshing and energizing, perfect for staying sharp! 💪",
        "classic": "You can't go wrong with our Espresso - it's the foundation of great coffee! ☕",
        "default": "I'd recommend our Americano - it's popular and reliable, just like the blockchain! Strong and dependable. 💪"
    }
    
    base_recommendation = recommendations.get(preference.lower(), recommendations["default"])
    
    # Add ordering instructions to all recommendations
    full_recommendation = f"{base_recommendation}\n\n📱 To order: Open your Slush wallet and visit the Coffee Hub website!"
    
    logger.info(f"Drink recommendation for '{preference}': {base_recommendation}")
    
    await send_tool_event("recommend_drink", "completed", [preference], full_recommendation)
    return full_recommendation 