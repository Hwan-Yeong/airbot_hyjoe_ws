import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            name='sensor_to_pointcloud',
            package='sensor_to_pointcloud',
            executable='sensor_to_pointcloud',
            output='screen',
        )
    ])
