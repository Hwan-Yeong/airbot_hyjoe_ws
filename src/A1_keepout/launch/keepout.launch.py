import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,PythonExpression,ThisLaunchFileDir
from launch_ros.actions import Node
from launch.conditions import IfCondition



def generate_launch_description():
    json_path = os.path.join(get_package_share_directory('A1_keepout'),'json/')

    DeclareLaunchArgument(
        'json_file_path', default_value=json_path, description='File path of keepout zone json'
    ),
    DeclareLaunchArgument(
        'reqeust_time_ms', default_value='1000', description='Publish TEST'
    ),
    DeclareLaunchArgument(
        'obstacle_point_resolution', default_value='0.05', description='Obstacle point cloud resolution'
    ),
    DeclareLaunchArgument(
        'publish_topic_name', default_value='/A1_keepout/point_cloud', description='PointCloud2 publish topic'
    )
    
    return LaunchDescription([
         Node(
             package='A1_keepout',
             executable='A1_keepout',
             name='A1_keepout_node',
             parameters=[
                 {'reqeust_time_ms'   : 5000},
                #  {'json_file_path'    : '/home/airbot/keepout_json/'}
                {'json_file_path'    : '/home/airbot/app_rw/A1_keepout/'}
            ]
        ),    
    ])

