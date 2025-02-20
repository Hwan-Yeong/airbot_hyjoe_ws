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

    param_file_name = 'explore_navigation.yaml'

    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('explore'),
            'params',
            param_file_name))

    nav2_launch_file_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')

    DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    rviz_config_dir = os.path.join(
        get_package_share_directory('airbot_slam'),
        'rviz',
        'slam.rviz')

    return LaunchDescription([

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
                'use_respawn', default_value='True',
                description='Whether to respawn if a node crashes. Applied when composition is disabled.'),
##Platform Driver
        # Node(
        #     package='driver_control',
        #     executable='driver_control',
        #     name='driver_control'),

##Odom Publisher Node

##***Scan merger********
#  IncludeLaunchDescription(
#             PythonLaunchDescriptionSource([get_package_share_directory(
#                 'ira_laser_tools'), '/launch/merge_multi.launch.py'])
#         ),
        
        
##YD Lidar Launch
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         [get_package_share_directory('ydlidar_ros2_driver'), '/launch/ydlidar_launch.py'])
        # ),
###***Ld Lidar Launch file
# IncludeLaunchDescription(
#     PythonLaunchDescriptionSource(
#         [get_package_share_directory('ldlidar_stl_ros2'), '/launch/ld19.launch.py'])
# ),

##********CSPC liDAR***********
# IncludeLaunchDescription(
#     PythonLaunchDescriptionSource(
#         [get_package_share_directory('airbot_lidar'), '/launch/lidar_launch.py'])
# ),
# ##************Slam Toolbox Launch File*************
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         [get_package_share_directory('slam_toolbox'), '/launch/online_async_launch.py'])
        # ),


####*******cartographer mapping*******

 IncludeLaunchDescription(
            PythonLaunchDescriptionSource(  
                [get_package_share_directory('airbot_slam'), '/launch/carto_mapping.launch.py'])
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(nav2_launch_file_dir,
        #                                                'localization_launch.py')),
        # #     condition=IfCondition(PythonExpression(['not ', slam])),
        #     launch_arguments={'map': map_dir,
        #                       'use_sim_time': use_sim_time,
        #                       'autostart': autostart,
        #                       'params_file': param_dir,
        #                       'use_respawn': use_respawn}.items()),

#Navigation Bringup
        # IncludeLaunchDescription(
        #         PythonLaunchDescriptionSource(
        #             [nav2_launch_file_dir, '/navigation_launch.py']),
        #         launch_arguments={
        #             'autostart': autostart,
        #             'params_file': param_dir,
        #             'use_respawn': use_respawn,
        #             'use_sim_time': use_sim_time}.items(),
        # ),

        # Node(package='tf2_ros',
        #             executable='static_transform_publisher',
        #             name='static_tf_pub_laser',
        #             arguments=['0', '0', '0.0','0', '0', '0', '1','map','odom'],
        #             ),
# ## Realsense Camera
        
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([get_package_share_directory(
        #         'realsense2_camera'), '/launch/rs_launch.py'])
        # ),
        
        #  Node(
        #     package='rviz2',
        #    executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config_dir],
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     output='screen'),
    ])
