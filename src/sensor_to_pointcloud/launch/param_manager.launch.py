from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            name='airbot_param_manager',
            package='sensor_to_pointcloud',
            executable='param_setter',
            output='screen',
        )
    ])
