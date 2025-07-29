# Tool Event Message
# Published when function tools are called by the agent

# Name of the tool being called
string tool_name          # "get_current_time", "get_coffee_menu", etc.

# Input parameters passed to the tool (JSON string array)
string[] parameters       # Parameters passed to the tool function

# Result returned by the tool
string result            # Tool output/result text

# Status of the tool execution
string status            # "started", "completed", "failed"

# When this tool event occurred
builtin_interfaces/Time timestamp 