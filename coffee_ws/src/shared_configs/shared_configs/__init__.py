#!/usr/bin/env python3

# Root
SYSTEM = "/system"

# Packages
PERCEPTION = f"{SYSTEM}/perception"
SENSOR = f"{PERCEPTION}/sensor"
BEHAVIOR = f"{SYSTEM}/behavior"
EFFECTOR = f"{SYSTEM}/effector"

# Topics
VOICE_INTENT_TOPIC = f"{SENSOR}/voice/intent"
VOICE_INTENT_RESPONSE_TOPIC = f"{VOICE_INTENT_TOPIC}/response"
VOICE_INTENT_CLASSIFIER_TOPIC = f"{VOICE_INTENT_TOPIC}/classifier"
EMOTION_TOPIC = f"{EFFECTOR}/emotion"
EMOTION_CLASSIFIER_TOPIC = f"{EMOTION_TOPIC}/classifier"
FACE_POSITION_TOPIC = f"{SENSOR}/face_position"
CAMERA_TOPIC = f"{SENSOR}/camera"
MICROPHONE_TOPIC = f"{SENSOR}/microphone"
LANGUAGE_MODEL_PROCESSOR_TOPIC = f"{BEHAVIOR}/language_model_processor"
STATE_MANAGER_TOPIC = f"{BEHAVIOR}/state_manager"
LANGUAGE_MODEL_PROCESSOR_STATUS_TOPIC = f"{LANGUAGE_MODEL_PROCESSOR_TOPIC}/status"
TTS_TOPIC = f"{EFFECTOR}/tts"
TTS_STATUS_TOPIC = f"{TTS_TOPIC}/status"

# Services
GENERATE_BEHAVIOR_RESPONSE_SERVICE = f"{LANGUAGE_MODEL_PROCESSOR_TOPIC}/generate_behavior_response"
TTS_SERVICE = f"{TTS_TOPIC}/tts_query"

# Intent mapping used for determining the intent of the user's voice command
INTENT_MAPPING_BYTE_TO_STRING = {
    1: "Agree",
    2: "Disagree",
    3: "Joke",
    4: "Question",
    5: "None"
}

# Intent mapping used for determining the intent of the user's voice command
INTENT_MAPPING_STRING_TO_BYTE = {
    "Agree": 1,
    "Disagree": 2,
    "Joke": 3,
    "Question": 4,
    "None": 5
}

DEFAULT_INTENT = len(INTENT_MAPPING_BYTE_TO_STRING)

NULL_VALUE = "NULL"

# QoS settings
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

DEFAULT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)