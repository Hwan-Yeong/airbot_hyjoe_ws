import os
import numpy as np
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir



def generate_launch_description():

    return LaunchDescription([
         Node(
             package='airbot_sensor_to_pointcloud',
             executable='sensor_to_pointcloud',
             output='screen',
             name='airbot_sensor_to_pointcloud',
             parameters=[
                {"target_frame": "base_link"}, # "map" or "base_link"

                {"use_tof_map_pointcloud": True},
                {"use_tof_1D": True},
                {"use_tof_left": True},
                {"use_tof_right": True},
                {"tof_debug_mode": False},
                {"use_camera_map_pointcloud": True},
                {"use_line_laser_map_pointcloud": False},

                {"camera_pointcloud_resolution_m": 0.05},
                {"camera_number_of_object": 0},

                {"pointcloud_publish_rate_ms": 100},
            ]
        ),
    ])