from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='udp_interface',  # Replace with the actual package name
            executable='udp_communication',  # Replace with the actual executable name
            name='udp_interface',
            output='screen',
           
        ),
    ])
