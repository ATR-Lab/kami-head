# Agent Status Message
# Published for unified robot coordination - replaces separate TTS and state events

# Overall behavioral mode
string behavioral_mode     # "dormant", "active", "connecting", "disconnecting"

# Current speech activity  
string speech_status       # "idle", "speaking"

# Current emotion to display (always populated)
string emotion            # "waiting", "friendly", "excited", "curious", "sleepy", etc.

# Speech content (empty when not speaking)
string speech_text        # Preview on start, full text on finish, empty when idle

# Previous emotion for smooth visual transitions
string previous_emotion   # Previous emotion state

# Type of interaction context
string conversation_phase # "", "greeting", "discussion", "announcement", "goodbye"

# Last function tool used (for context)
string last_tool_used    # "get_current_time", "get_coffee_menu", "", etc.

# When this status occurred
builtin_interfaces/Time timestamp 