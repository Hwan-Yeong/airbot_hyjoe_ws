from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            name='airbot_param_setter',
            package='airbot_sensor_manager',
            executable='param_setter',
            output='screen',            
            respawn = True
        )
    ])
