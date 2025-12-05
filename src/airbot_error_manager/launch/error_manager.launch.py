import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, TimerAction

def generate_launch_description():
    parameter_file = os.path.join(
        get_package_share_directory('airbot_error_manager'),
        'config',
        'error_manager_params.yaml'
    )

    error_monitor_node = Node(
        package='airbot_error_manager',
        executable='error_monitor_node',
        name='airbot_error_monitor',
        output='screen',
        parameters=[parameter_file],
        respawn=True,
        respawn_delay=1.0
    )

    error_manager_node = TimerAction(
        period=0.2,
        actions=[
            Node(
                package='airbot_error_manager',
                executable='error_manager_node',
                name='airbot_error_manager',
                output='screen',
                respawn=True,
                respawn_delay=1.0
            )
        ]
    )

    return LaunchDescription([
        error_monitor_node,
        error_manager_node
    ])
