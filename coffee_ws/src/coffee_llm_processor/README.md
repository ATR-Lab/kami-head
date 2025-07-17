# Coffee LLM Processor

A ROS2 package providing Large Language Model (LLM) processing capabilities for the Coffee Buddy robot system. This package handles conversational AI, response generation, and intelligent interaction management.

## Overview

The `coffee_llm_processor` package serves as the conversational intelligence hub for the Coffee Buddy robot. It processes user intents and generates contextual, personality-driven responses using state-of-the-art language models while maintaining conversation history and integrating with coffee dispensing functionality.

## Features

- **🤖 Conversational AI**: Advanced language model processing with BrewBot personality
- **🔄 Multi-Provider Support**: Compatible with OpenAI and Atoma API providers
- **💬 Conversation Memory**: Maintains conversation history for contextual responses
- **☕ Coffee Integration**: Can trigger coffee dispensing actions based on conversation
- **⚡ Async Processing**: Non-blocking LLM API calls with timeout handling
- **🛡️ Error Handling**: Robust error handling and fallback responses
- **📊 Status Publishing**: Real-time status and diagnostics

## Architecture

### Core Components

- **`LanguageModelProcessorNode`**: Main ROS2 node handling conversation flow
- **`LLMClient`**: Abstraction layer supporting multiple LLM providers
- **Conversation Management**: History tracking and context maintenance

### Robot Personality

BrewBot is configured as a friendly coffee robot with the following characteristics:
- Warm and conversational tone
- Interested in conference experiences and blockchain technology
- Brief, natural responses (1-2 sentences)
- Coffee-focused but socially engaging

## ROS2 Services

### `/chat` (coffee_interfaces/srv/ChatService)
Primary service for processing conversational requests.

**Request:**
```
string prompt    # User input text
```

**Response:**
```
string response  # Generated AI response
bool success     # Operation success status
```

### Coffee Integration
Automatically triggers coffee dispensing when "coffee" is mentioned in conversation through integration with the coffee machine control system.

## Topics

### Publishers
- `/language_model_processor/status` (String): Node status and diagnostics

## Configuration

### Environment Variables

**Required:**
```bash
export OPENAI_API_KEY="your_openai_api_key"    # For OpenAI provider
# OR
export ATOMA_API_KEY="your_atoma_api_key"      # For Atoma provider
```

**Optional:**
```bash
export LLM_PROVIDER="openai"                   # Options: openai, atoma
export LLM_MODEL="gpt-3.5-turbo"             # Model to use
export LLM_TEMPERATURE="0.7"                  # Response creativity (0.0-1.0)
export LLM_MAX_TOKENS="150"                   # Maximum response length
```

### ROS2 Parameters

- `api_provider` (string, default: 'openai'): LLM API provider to use
- `model_name` (string, default: 'gpt-3.5-turbo'): Specific model name
- `temperature` (float, default: 0.7): Response randomness/creativity
- `max_tokens` (int, default: 150): Maximum response length
- `timeout` (float, default: 10.0): API request timeout in seconds

## Installation

### Dependencies

Add to your `package.xml`:
```xml
<depend>rclpy</depend>
<depend>std_msgs</depend>
<depend>coffee_interfaces</depend>
<depend>coffee_machine_control_msgs</depend>
<depend>shared_configs</depend>
```

Python dependencies (automatically installed):
- `openai` - OpenAI API client
- `asyncio` - Asynchronous processing
- `concurrent.futures` - Thread pool execution

### Build

```bash
# Navigate to workspace
cd ~/coffee_ws

# Build the package
colcon build --packages-select coffee_llm_processor

# Source the workspace
source install/setup.bash
```

## Usage

### Basic Usage

Run the LLM processor node:
```bash
ros2 run coffee_llm_processor language_model_processor_node
```

### Service Call Examples

Basic conversation:
```bash
ros2 service call /chat coffee_interfaces/srv/ChatService "{prompt: 'Hello there, how are you?'}"
```

Coffee-related request:
```bash
ros2 service call /chat coffee_interfaces/srv/ChatService "{prompt: 'I would like a cup of coffee please'}"
```

Conference discussion:
```bash
ros2 service call /chat coffee_interfaces/srv/ChatService "{prompt: 'What do you think about blockchain technology?'}"
```

### Integration with Other Nodes

Typically used in conjunction with:
- `coffee_speech_processing` - Speech recognition and intent classification
- `coffee_voice_service` - Text-to-speech output
- `coffee_machine_control` - Physical coffee dispensing

## Configuration Examples

### OpenAI Configuration
```bash
# Set environment variables
export OPENAI_API_KEY="sk-your-key-here"
export LLM_PROVIDER="openai"
export LLM_MODEL="gpt-3.5-turbo"

# Run with parameters
ros2 run coffee_llm_processor language_model_processor_node --ros-args \
  -p api_provider:=openai \
  -p model_name:=gpt-3.5-turbo \
  -p temperature:=0.7
```

### Atoma Configuration  
```bash
# Set environment variables
export ATOMA_API_KEY="your-atoma-key"
export LLM_PROVIDER="atoma"

# Run with parameters
ros2 run coffee_llm_processor language_model_processor_node --ros-args \
  -p api_provider:=atoma \
  -p timeout:=15.0
```

## Monitoring

### Status Monitoring
```bash
# Monitor node status
ros2 topic echo /language_model_processor/status

# Check service availability
ros2 service list | grep chat

# Test service response time
time ros2 service call /chat coffee_interfaces/srv/ChatService "{prompt: 'test'}"
```

### Debugging
```bash
# Run with debug logging
ros2 run coffee_llm_processor language_model_processor_node --ros-args --log-level DEBUG

# Monitor conversation history (check logs)
ros2 log set_logger_level language_model_processor_node DEBUG
```

## Troubleshooting

### Common Issues

**API Key Not Found:**
```
[ERROR] API key not found for provider: openai
```
**Solution:** Set the appropriate environment variable (`OPENAI_API_KEY` or `ATOMA_API_KEY`)

**Service Call Timeout:**
```
[WARN] LLM API request timed out
```
**Solution:** Increase timeout parameter or check network connectivity

**Model Not Available:**
```
[ERROR] Model 'gpt-4' not available
```
**Solution:** Use a supported model name or check API permissions

### Performance Optimization

- Use faster models (e.g., `gpt-3.5-turbo` vs `gpt-4`) for lower latency
- Reduce `max_tokens` for quicker responses
- Adjust `temperature` based on desired response creativity
- Monitor conversation history size to prevent context overflow

## Development

### Adding New LLM Providers

Extend the `LLMClient` abstraction in `llm_client.py`:

1. Create a new client class inheriting from `LLMClient`
2. Implement the required abstract methods
3. Add provider selection logic in `create_llm_client()`

### Customizing Personality

Modify the system prompt in the node initialization to change BrewBot's personality and conversation style.

## Future Enhancements

- [ ] Multi-turn conversation optimization
- [ ] Context-aware coffee recommendations  
- [ ] Integration with customer preference learning
- [ ] Multi-language support
- [ ] Voice tone/emotion integration
- [ ] Advanced conversation analytics
