from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='airbot_ai_interface',
            executable='airbot_ai_interface',
            name='airbot_ai_interface_node',
            parameters=[
                {"port": "/dev/ttyAI"},
                {"baudrate": 921600},
                {"use_cam": 1},
                {"use_2LL": 1}
            ]
        ),
    ])
