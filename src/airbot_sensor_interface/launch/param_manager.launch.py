from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            name='airbot_param_manager',
            package='airbot_sensor_interface',
            executable='param_manager_node',
            output='screen',
        )
    ])
