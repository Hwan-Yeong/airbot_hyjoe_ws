from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            name='param_manager_node',
            package='sensor_manager',
            executable='param_setter',
            output='screen',
        )
    ])
