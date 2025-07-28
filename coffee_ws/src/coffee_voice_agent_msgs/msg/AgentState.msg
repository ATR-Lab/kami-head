# Agent State Message
# Published when the voice agent changes conversation states

# Current state: "dormant", "connecting", "active", "speaking", "disconnecting"
string current_state

# Previous state for transition tracking
string previous_state

# Timestamp when the state change occurred
builtin_interfaces/Time timestamp 