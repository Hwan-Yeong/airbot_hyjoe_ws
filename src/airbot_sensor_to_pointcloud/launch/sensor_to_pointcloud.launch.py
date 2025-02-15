import os
import yaml
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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

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
    parameter_file = LaunchConfiguration('params_file')
    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('airbot_sensor_to_pointcloud'),
            'config',
            'sensor_to_pointcloud_param.yaml'
        ),
        description='Path to the ROS2 parameters file to use.'
    )

    return LaunchDescription([
        params_declare,
        Node(
            name='airbot_sensor_to_pointcloud',
            package='airbot_sensor_to_pointcloud',
            executable='sensor_to_pointcloud',
            output='screen',
            parameters=[parameter_file],
        ),
    ])