import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('airbot_teleop_velocity_smoother'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='airbot_teleop_velocity_smoother',
            executable='teleop_velocity_smoother_node',
            name='airbot_teleop_velocity_smoother_node',
            parameters=[config],
            output='screen'
        )
    ])