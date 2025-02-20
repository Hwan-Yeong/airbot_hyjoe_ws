import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,PythonExpression,ThisLaunchFileDir
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():

    DeclareLaunchArgument(
        'min_particle', default_value='1000', description='Minimum number of particles to stop localization'
    ),
    DeclareLaunchArgument(
        'reqeust_time_ms', default_value='100', description='Time to request nomotion'
    ) 
    DeclareLaunchArgument(
        'print_log', default_value='false', description='log printing'
    )
    return LaunchDescription([
         Node(
             package='A1_localization',
             executable='A1_localization',
             name='A1_localization',
             parameters=[
                 {'min_particle': 200},
                 {'reqeust_time_ms': 10},
                 {'print_log': True}
            ]),
    ])