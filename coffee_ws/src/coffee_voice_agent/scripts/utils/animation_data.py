"""Animation data constants for emotional expressions"""

import logging

logger = logging.getLogger(__name__)

# Eye animation descriptions for different emotions
EYE_ANIMATIONS = {
    "excited": "👀 EXCITED: Eyes wide open, rapid blinking, pupils dilated, eyebrows raised high",
    "helpful": "🤓 HELPFUL: Focused gaze, slight squint, eyebrows slightly furrowed in concentration",
    "friendly": "😊 FRIENDLY: Soft, warm gaze, gentle blinking, slightly curved 'smile' shape",
    "curious": "🤔 CURIOUS: One eyebrow raised, eyes tracking side to side, pupils moving inquisitively",
    "empathetic": "🥺 EMPATHETIC: Soft, caring gaze, slower blinking, eyebrows slightly angled down",
    "sleepy": "😴 SLEEPY: Half-closed eyes, very slow blinking, drooping eyelids, occasional yawn animation",
    "waiting": "⏳ WAITING: Steady gaze, regular blinking, eyes occasionally looking around patiently",
    "confused": "😕 CONFUSED: Eyes darting around, irregular blinking, eyebrows furrowed, head tilt effect",
    "proud": "😌 PROUD: Eyes slightly narrowed with satisfaction, confident gaze, subtle sparkle effect",
    "playful": "😄 PLAYFUL: Bright, animated eyes, quick winks, eyebrows dancing, mischievous glint",
    "focused": "🎯 FOCUSED: Intense stare, minimal blinking, laser-focused pupils, determined expression",
    "surprised": "😲 SURPRISED: Eyes suddenly wide, rapid blinking, eyebrows shot up, pupils contracted",
    "excuse": "😅 EXCUSE: Apologetic gaze, slight head tilt, gentle blinking, eyebrows raised politely"
}


def log_animated_eyes(emotion: str):
    """Log how this emotion would appear as animated eyes"""
    animation_desc = EYE_ANIMATIONS.get(emotion, "😐 NEUTRAL: Standard eye animation")
    logger.info(f"🎨 Eye Animation: {animation_desc}") 