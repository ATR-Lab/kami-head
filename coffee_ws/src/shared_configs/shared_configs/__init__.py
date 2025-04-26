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
VOICE_INTENT_CLASSIFIER_TOPIC = f"{VOICE_INTENT_TOPIC}/classifier"
EMOTION_TOPIC = f"{EFFECTOR}/emotion"
EMOTION_CLASSIFIER_TOPIC = f"{EMOTION_TOPIC}/classifier"
FACE_POSITION_TOPIC = f"{SENSOR}/face_position"
CAMERA_TOPIC = f"{SENSOR}/camera"
MICROPHONE_TOPIC = f"{SENSOR}/microphone"
LANGUAGE_MODEL_PROCESSOR_TOPIC = f"{BEHAVIOR}/language_model_processor"
STATE_MANAGER_TOPIC = f"{BEHAVIOR}/state_manager"

# Intent mapping used for determining the intent of the user's voice command
INTENT_MAPPING = {
    0: "Greeting",
    1: "Goodbye",
    2: "Thank",
    3: "Apologize",
    4: "Affirm",
    5: "Deny",
    6: "Inform",
    7: "Request",
    8: "Question",
    9: "Confirm",
    10: "Disconfirm",
    11: "Clarify",
    12: "Suggest",
    13: "Complaint",
    14: "Praise",
    15: "Joke",
    16: "SmallTalk",
    17: "Fallback",
    18: "Agree",
    19: "Disagree"
}

# QoS settings
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

DEFAULT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)