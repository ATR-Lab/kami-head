#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for TTS node with configurable parameters."""
    
    # Declare launch arguments
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
    
    # TTS Node
    tts_node = Node(
        package='effector_nodes',
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
        }]
    )
    
    # Log info about the launch
    log_info = LogInfo(
        msg=[
            'Starting TTS Node with voice: ',
            LaunchConfiguration('voice_id'),
            ' and model: ',
            LaunchConfiguration('model_id')
        ]
    )
    
    return LaunchDescription([
        voice_id_arg,
        model_id_arg,
        api_key_arg,
        cooldown_duration_arg,
        output_format_arg,
        log_info,
        tts_node
    ]) 