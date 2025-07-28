# TTS Event Message
# Published when the voice agent starts or stops speaking

# Event type: "started" or "finished"
string event

# Current emotion being expressed
string emotion

# Preview of the text being spoken (truncated for large text)
string text

# Source of the TTS: "manual", "llm", "announcement", etc.
string source

# Timestamp when the event occurred
builtin_interfaces/Time timestamp 