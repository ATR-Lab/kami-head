from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description for face recognition node"""
    
    # Face recognition node
    face_recognition_node = Node(
        package='coffee_head',
        executable='face_recognition',
        name='face_recognition_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                'data_dir': '~/.coffee_head/face_recognition'
            }
        ]
    )
    
    # Return launch description
    return LaunchDescription([
        face_recognition_node
    ]) 