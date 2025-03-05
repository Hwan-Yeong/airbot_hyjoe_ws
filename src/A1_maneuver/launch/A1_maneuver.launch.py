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
             package='A1_maneuver',
             executable='A1_maneuver',
             name='A1_maneuver',
             parameters=[
                # {"th_obstacle_dist": 0.6},
                # {"th_emergency_dist": 0.3},
                # {"avoid_back_speed": 0.0}
             ]
        ),
    ])