# The robot's current facial expression or emotional state
string expression # "Happy", "Curious"

# What triggered this state: "vision", "audio", "event", or "mock"
string trigger_source # "audio", "vision", "payment", "coffee"

# Where the robot should direct its gaze (e.g. face position)
geometry_msgs/Point gaze_target

# Whether the robot is in idle state (e.g. no one is interacting)
bool is_idle
