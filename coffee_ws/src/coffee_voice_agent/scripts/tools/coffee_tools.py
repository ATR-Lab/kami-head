"""Coffee-related function implementations for programmatic tool registration"""

import logging
from datetime import datetime
from livekit.agents import RunContext

logger = logging.getLogger(__name__)


async def get_current_time_impl(context: RunContext) -> str:
    """Get the current time."""
    current_time = datetime.now().strftime("%I:%M %p")
    logger.info(f"Time requested: {current_time}")
    return f"The current time is {current_time}"


async def get_current_date_impl(context: RunContext) -> str:
    """Get today's date."""
    current_date = datetime.now().strftime("%A, %B %d, %Y")
    logger.info(f"Date requested: {current_date}")
    return f"Today's date is {current_date}"


async def get_coffee_menu_impl(context: RunContext) -> str:
    """Get the Sui Hub coffee menu."""
    menu = """🚀 SUI HUB COFFEE MENU ☕

    ☕ CLASSIC BREWS:
    - Espresso
    - Black Coffee
    - Americano
    - Long Black
    
    📱 TO ORDER: Open your Slush wallet and go to the Coffee Hub website to place your order!
    
    All drinks come with complimentary blockchain wisdom! 🤖"""
    
    logger.info("Coffee menu requested")
    return menu


async def get_ordering_instructions_impl(context: RunContext) -> str:
    """Get instructions on how to order coffee through the Slush wallet and Coffee Hub website."""
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
    return instructions


async def recommend_drink_impl(context: RunContext, preference: str = "energizing") -> str:
    """Recommend a drink based on user preference.
    
    Args:
        preference: Type of drink preference (energizing, smooth, sweet, cold, etc.)
    """
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
    return full_recommendation 