# Configuration update message for propagating agent settings to UI components
#
# This message is published when:
# - Agent connects to bridge and shares its configuration
# - Configuration values change during runtime
# - Connection status changes (to republish current config)

# Standard ROS header with timestamp
std_msgs/Header header

# Configuration validity and source
bool configuration_valid      # True if config data is valid and from connected agent
string config_source         # Source of config: "agent", "fallback", "parameter_override"

# Core timing configuration (from config.settings)
float64 user_response_timeout    # Seconds to wait for user response before prompting
float64 final_timeout           # Additional seconds before ending conversation
float64 max_conversation_time   # Maximum total conversation duration in seconds

# Emotion configuration
string[] valid_emotions         # List of valid emotion states for the agent

# WebSocket configuration (for debugging/monitoring)
string websocket_host           # WebSocket server host
int32 websocket_port           # WebSocket server port

# Version and metadata
string agent_version           # Version of the voice agent
string config_timestamp        # ISO timestamp when config was generated 