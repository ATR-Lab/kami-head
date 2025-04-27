from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description for all coffee_head nodes"""
    
    # Dynamixel SDK read_write_node
    # dynamixel_node = Node(
    #     package='dynamixel_sdk_examples',
    #     executable='read_write_node',
    #     name='dynamixel_node',
    #     output='screen',
    #     emulate_tty=True,
    #     parameters=[
    #         {
    #             'device_name': '/dev/ttyUSB0'
    #         }
    #     ]
    # )

    coffee_dynamixel_node = Node(
        package='coffee_head',
        executable='read_write_coffee',
        name='coffee_dynamixel_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                'device_name': '/dev/ttyUSB0'
            }
        ]
    )
    
    # Camera node
    camera_node = Node(
        package='coffee_head',
        executable='camera_node',
        name='camera_node',
        output='screen'
    )

    # Coffee Buddy State Manager
    coffee_buddy_state_manager_node = Node(
        package='coffee_expressions_state_manager',
        executable='state_manager_node',
        name='coffee_buddy_state_manager_node',
        output='screen'
    )
    
    # Head tracking node
    head_tracking_node = Node(
        package='coffee_head',
        executable='head_tracking',
        name='head_tracking_node',
        output='screen',
        emulate_tty=True
    )
    
    # # Eye tracking node
    # eye_tracking_node = Node(
    #     package='coffee_head',
    #     executable='eye_tracking',
    #     name='eye_tracking_node',
    #     output='screen',
    #     emulate_tty=True
    # )
    
    # # Face recognition node
    # face_recognition_node = Node(
    #     package='coffee_head',
    #     executable='face_recognition',
    #     name='face_recognition_node',
    #     output='screen',
    #     emulate_tty=True,
    #     parameters=[
    #         {
    #             'data_dir': '~/.coffee_head/face_recognition'
    #         }
    #     ]
    # )
    
    # Return launch description
    return LaunchDescription([
        coffee_dynamixel_node,
        coffee_buddy_state_manager_node,
        camera_node,
        head_tracking_node
        # eye_tracking_node,
        # face_recognition_node
    ]) 