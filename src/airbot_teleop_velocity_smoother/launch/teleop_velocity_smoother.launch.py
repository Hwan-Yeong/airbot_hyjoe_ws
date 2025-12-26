import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config_file = LaunchConfiguration('params_file')
    config_declare = DeclareLaunchArgument(
        'params_file',
        default_value = os.path.join(
            get_package_share_directory('airbot_teleop_velocity_smoother'),
            'config',
            'params.yaml'
        ),
        description='Path to the Teleoperation Velocity Smoother parameters file to use.'
    )

    return LaunchDescription([
        config_declare,
        Node(
            name='airbot_teleop_velocity_smoother_node',
            package='airbot_teleop_velocity_smoother',
            executable='teleop_velocity_smoother_node',
            output='screen',
            parameters=[config_file],
            respawn=True,
            respawn_delay=1.0
        )
    ])