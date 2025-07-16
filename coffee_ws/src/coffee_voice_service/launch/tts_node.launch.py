#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for TTS node with configurable parameters."""
    
    # Declare launch arguments for TTS configuration
    voice_id_arg = DeclareLaunchArgument(
        'voice_id',
        default_value='KTPVrSVAEUSJRClDzBw7',
        description='ElevenLabs voice ID to use for TTS'
    )
    
    model_id_arg = DeclareLaunchArgument(
        'model_id',
        default_value='eleven_multilingual_v2',
        description='ElevenLabs model ID to use for TTS'
    )
    
    api_key_arg = DeclareLaunchArgument(
        'api_key',
        default_value='',
        description='ElevenLabs API key (leave empty to use ELEVEN_LABS_API_KEY environment variable)'
    )
    
    cooldown_duration_arg = DeclareLaunchArgument(
        'cooldown_duration',
        default_value='1.0',
        description='Cooldown duration in seconds between TTS requests'
    )
    
    output_format_arg = DeclareLaunchArgument(
        'output_format',
        default_value='pcm_24000',
        description='Audio output format (pcm_16000, pcm_24000, etc.)'
    )
    
    # Declare launch arguments for ROS2 communication configuration
    service_name_arg = DeclareLaunchArgument(
        'service_name',
        default_value='/coffee/voice/tts/query',
        description='ROS2 service name for TTS queries'
    )
    
    status_topic_arg = DeclareLaunchArgument(
        'status_topic',
        default_value='/coffee/voice/tts/status',
        description='ROS2 topic for TTS status updates'
    )
    
    audio_state_topic_arg = DeclareLaunchArgument(
        'audio_state_topic',
        default_value='/coffee/voice/tts/audio_state',
        description='ROS2 topic for audio playback state updates'
    )
    
    # TTS Node
    tts_node = Node(
        package='coffee_voice_service',
        executable='tts_node',
        name='tts_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'voice_id': LaunchConfiguration('voice_id'),
            'model_id': LaunchConfiguration('model_id'),
            'api_key': LaunchConfiguration('api_key'),
            'cooldown_duration': LaunchConfiguration('cooldown_duration'),
            'output_format': LaunchConfiguration('output_format'),
            'service_name': LaunchConfiguration('service_name'),
            'status_topic': LaunchConfiguration('status_topic'),
            'audio_state_topic': LaunchConfiguration('audio_state_topic'),
        }]
    )
    
    # Log info about the launch
    log_info = LogInfo(
        msg=[
            'Starting TTS Node with voice: ',
            LaunchConfiguration('voice_id'),
            ' and model: ',
            LaunchConfiguration('model_id'),
            ' | Service: ',
            LaunchConfiguration('service_name')
        ]
    )
    
    return LaunchDescription([
        voice_id_arg,
        model_id_arg,
        api_key_arg,
        cooldown_duration_arg,
        output_format_arg,
        service_name_arg,
        status_topic_arg,
        audio_state_topic_arg,
        log_info,
        tts_node
    ]) 