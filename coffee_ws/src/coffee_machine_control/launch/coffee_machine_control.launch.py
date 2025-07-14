from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    mac_address_arg = DeclareLaunchArgument(
        'mac_address',
        default_value='9C:95:6E:61:B6:2C',  # Default MAC address from README
        description='MAC address of the Delonghi coffee machine'
    )

    # Create coffee machine control node
    coffee_machine_control_node = Node(
        package='coffee_machine_control',
        executable='coffee_machine_control_node',
        name='coffee_machine_control_node',
        parameters=[{
            'mac_address': LaunchConfiguration('mac_address')
        }],
        output='screen'
    )

    return LaunchDescription([
        mac_address_arg,
        coffee_machine_control_node
    ])
