# Emotion State Message
# Published when the voice agent changes emotional expression

# Current emotion: "friendly", "excited", "curious", "sleepy", etc.
string emotion

# Previous emotion for smooth transition tracking
string previous_emotion

# Timestamp when the emotion change occurred
builtin_interfaces/Time timestamp 