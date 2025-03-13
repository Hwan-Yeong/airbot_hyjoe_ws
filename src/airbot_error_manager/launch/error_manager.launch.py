from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='airbot_error_manager',
            executable='error_manager_node',
            name='airbot_error_manager',
            output='screen',
        ),
        Node(
            package='airbot_error_manager',
            executable='error_monitor_node',
            name='airbot_error_monitor',
            output='screen',
        )
    ])
