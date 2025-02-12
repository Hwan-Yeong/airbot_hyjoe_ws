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

##############################
### Camera Object Class ID ###
# 0: cable
# 1: carpet
# 2: clothes
# 3: liquid
# 4: non_obstacle
# 5: obstacle
# 6: poop
# 7: scale
# 8: threshold
# 9: person
# 10: dog
# 11: cat
##############################

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
                {"tof_debug_mode": True},
                {"use_camera_map_pointcloud": True},
                {"use_line_laser_map_pointcloud": False},

                {"camera_pointcloud_resolution_m": 0.05},
                {"camera_target_class_id_list": [1, 5, 6]},
                {"camera_confidence_threshold": 55}, # range:[1,100]
                {"camera_object_direction": True}, # 정방향(CCW+):True, 역방향(CW+):False

                {"pointcloud_publish_rate_ms": 100},
            ]
        ),
    ])