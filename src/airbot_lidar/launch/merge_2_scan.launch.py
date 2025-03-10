
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    param_file = os.path.join(
        get_package_share_directory('airbot_lidar'),
        'config',
        'airbot_lidar_params.yaml'
    )
    return LaunchDescription([
        
        launch_ros.actions.Node(
            package='airbot_lidar',
            executable='airbot_lidar_merger',
            parameters=[param_file],
            output='screen',
            respawn=True,
            respawn_delay=2,
        ),

        launch_ros.actions.Node(
            name='airbot_pointcloud_to_laserscan',
            package='airbot_lidar',
            executable='pointcloud_to_laserscan_node',
            parameters=[param_file]
        )
    ])
