# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_respawn = LaunchConfiguration('use_respawn')
    autostart = LaunchConfiguration('autostart', default='true')
    slam = LaunchConfiguration('slam')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('airbot_navigation'),
            '/home/airbot/app_rw/map/airbot_map_00.yaml'))

    param_file_name = 'navigation_params.yaml'

    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('airbot_navigation'),
            'params',
            param_file_name))

    nav2_launch_file_dir = os.path.join(
        get_package_share_directory('airbot_navigation'), 'launch')

    DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    rviz_config_dir = os.path.join(
        get_package_share_directory('airbot_navigation'),
        'rviz',
        'navigation.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),

        IncludeLaunchDescription(
           PythonLaunchDescriptionSource([get_package_share_directory(
              'A1_maneuver'), '/launch/A1_maneuver.launch.py'])
        ),

        IncludeLaunchDescription(
           PythonLaunchDescriptionSource([get_package_share_directory(
              'A1_perception'), '/launch/A1_perception.launch.py'])
        ),

        # 2025.02.27 clabil
        # navigation 꺼지면서 연결 끊김 이슈로 이동
        IncludeLaunchDescription(
           PythonLaunchDescriptionSource([get_package_share_directory(
              'A1_localization'), '/launch/localization.launch.py'])
        ),

    ])
