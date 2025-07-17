#!/usr/bin/env python3
"""
Constants for Coffee Speech Processing package.

This module contains topic names, service names, and intent mappings used 
throughout the Coffee Buddy speech processing system. These constants were 
previously defined in the shared_configs package but are now local to 
maintain package independence.
"""

# Service endpoints
GENERATE_BEHAVIOR_RESPONSE_SERVICE = "/system/behavior/language_model_processor/generate_behavior_response"
TTS_SERVICE = "/coffee/voice/tts/query"  # Updated to match refactored coffee_voice_service

# Topic endpoints
VOICE_INTENT_RESPONSE_TOPIC = "/system/perception/sensor/voice/intent/response"
"""
VOICE_INTENT_RESPONSE_TOPIC: Intended for pub/sub architecture (not currently implemented)

This topic was designed for the voice intent node to publish LLM responses, allowing 
multiple consumers to process the same response data. The intended architecture was:

  VoiceIntentNode → LanguageProcessor (service) → Publish Response → Multiple Subscribers
                                                        ↓
                                           ┌─────────────┼─────────────┐
                                           ▼             ▼             ▼
                                      TTS Node    State Manager   UI/Analytics

Current implementation uses direct service calls instead:
  VoiceIntentNode → LanguageProcessor (service) → TTS (service)

This topic is kept for future enhancement to restore the decoupled pub/sub pattern.
"""

# Intent classification mappings
INTENT_MAPPING_BYTE_TO_STRING = {
    1: "Agree",
    2: "Disagree",
    3: "Joke",
    4: "Question",
    5: "None"
}

INTENT_MAPPING_STRING_TO_BYTE = {
    "Agree": 1,
    "Disagree": 2,
    "Joke": 3,
    "Question": 4,
    "None": 5
}

DEFAULT_INTENT = 5

# QoS settings (from original shared_configs)
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

DEFAULT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
) 