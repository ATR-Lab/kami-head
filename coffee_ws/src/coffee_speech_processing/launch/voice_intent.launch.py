from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    model_size_arg = DeclareLaunchArgument(
        'model_size',
        default_value='turbo',
        description='Whisper model size (tiny, base, small, medium, large, turbo, large-v3-turbo)'
    )
    
    language_arg = DeclareLaunchArgument(
        'language',
        default_value='en',
        description='Language for transcription (en, auto, etc.)'
    )
    
    device_type_arg = DeclareLaunchArgument(
        'device_type',
        default_value='cuda',
        description='Device type used for inference (cuda, cpu, mps)'
    )
    
    compute_type_arg = DeclareLaunchArgument(
        'compute_type',
        default_value='int8',
        description='Compute type used for inference (float16, int8_float16, int8, float32)'
    )
    
    wake_phrase_arg = DeclareLaunchArgument(
        'wake_phrase',
        default_value='buddy',
        description='Wake phrase to listen for'
    )

    verbose_arg = DeclareLaunchArgument(
        'verbose_logging',
        default_value='true',
        description='Enable verbose logging'
    )

    use_vad_arg = DeclareLaunchArgument(
        'use_vad',
        default_value='false',
        description='Enable Voice Activity Detection for better speech segmentation'
    )

    vad_silence_duration_arg = DeclareLaunchArgument(
        'vad_silence_duration',
        default_value='500',
        description='Milliseconds of silence to consider speech ended'
    )

    chunk_size_arg = DeclareLaunchArgument(
        'chunk_size',
        default_value='1920',
        description='Size of audio chunks for VAD (120ms at 16kHz)'
    )
    
    # Create node
    voice_intent_node = Node(
        package='coffee_speech_processing',
        executable='voice_intent_node',
        name='voice_intent_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            # ASR parameters
            {'model_size': LaunchConfiguration('model_size')},
            {'language': LaunchConfiguration('language')},
            {'device_type': LaunchConfiguration('device_type')},
            {'compute_type': LaunchConfiguration('compute_type')},
            {'wake_phrase': LaunchConfiguration('wake_phrase')},
            {'verbose_logging': LaunchConfiguration('verbose_logging')},
            # VAD configuration
            {'use_vad': LaunchConfiguration('use_vad')},  # Set to True to enable Voice Activity Detection
            {'vad_silence_duration': LaunchConfiguration('vad_silence_duration')},  # Milliseconds of silence to mark end of speech
            {'chunk_size': LaunchConfiguration('chunk_size')},  # Audio chunk size (120ms at 16kHz)
        ]
    )
    
    return LaunchDescription([
        model_size_arg,
        language_arg,
        device_type_arg,
        compute_type_arg,
        wake_phrase_arg,
        verbose_arg,
        use_vad_arg,
        vad_silence_duration_arg,
        chunk_size_arg,
        voice_intent_node
    ]) 