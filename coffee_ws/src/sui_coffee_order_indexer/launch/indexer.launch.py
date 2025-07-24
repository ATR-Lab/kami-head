from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # Declare all launch arguments
    package_id_arg = DeclareLaunchArgument(
        'package_id',
        description='[Required] The Sui package ID to index',
        default_value='""'  # Empty string as default to ensure string type
    )
    
    network_arg = DeclareLaunchArgument(
        'network',
        default_value='testnet',
        description='Sui network to connect to (testnet, mainnet, devnet)'
    )
    
    polling_interval_arg = DeclareLaunchArgument(
        'polling_interval_ms',
        default_value='1000',
        description='Polling interval in milliseconds'
    )
    
    default_limit_arg = DeclareLaunchArgument(
        'default_limit',
        default_value='50',
        description='Default limit for event queries'
    )
    
    database_url_arg = DeclareLaunchArgument(
        'database_url',
        default_value='',  # Empty string to use default path in package share
        description='Database URL for the indexer. If not provided, will use database in package share directory at "data/sui_indexer.db".'
    )
    
    pkg_share = FindPackageShare('sui_coffee_order_indexer')
    
    # First run Prisma setup
    setup_prisma = ExecuteProcess(
        cmd=['ros2', 'run', 'sui_coffee_order_indexer', 'prisma_setup'],
        name='prisma_setup',
        output='screen'
    )
    
    # Then start the indexer node
    indexer_node = Node(
        package='sui_coffee_order_indexer',
        executable='indexer_node',
        name='coffee_order_indexer',
        output='screen',
        parameters=[{
            'polling_interval_ms': LaunchConfiguration('polling_interval_ms'),
            'network': LaunchConfiguration('network'),
            'default_limit': LaunchConfiguration('default_limit'),
            'database_url': LaunchConfiguration('database_url'),
            'package_id': LaunchConfiguration('package_id', default='""')  # Ensure string type
        }]
    )
    
    # Create a timer action that starts the indexer node only after prisma setup completes
    start_indexer = TimerAction(
        period=0.0,  # Start immediately after the condition is met
        actions=[indexer_node]
    )
    
    return LaunchDescription([
        # Add all argument declarations
        package_id_arg,      # Required - no default value
        network_arg,         # Optional with defaults
        polling_interval_arg,
        default_limit_arg,
        database_url_arg,
        setup_prisma,
        # Only start the indexer after prisma_setup exits successfully
        RegisterEventHandler(
            OnProcessExit(
                target_action=setup_prisma,
                on_exit=[start_indexer]
            )
        )
    ]) 