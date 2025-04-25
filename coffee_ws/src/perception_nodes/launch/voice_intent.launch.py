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
    
    # Create node
    voice_intent_node = Node(
        package='perception_nodes',
        executable='voice_intent_node',
        name='voice_intent_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'model_size': LaunchConfiguration('model_size'),
            'language': LaunchConfiguration('language'),
            'device_type': LaunchConfiguration('device_type'),
            'compute_type': LaunchConfiguration('compute_type'),
        }]
    )
    
    return LaunchDescription([
        model_size_arg,
        language_arg,
        device_type_arg,
        compute_type_arg,
        voice_intent_node
    ]) 