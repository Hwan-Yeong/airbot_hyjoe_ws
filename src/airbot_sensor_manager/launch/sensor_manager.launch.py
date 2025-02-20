import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    sensor_to_pointcloud_launch = os.path.join(
        get_package_share_directory('airbot_sensor_manager'),
        'launch',
        'sensor_to_pointcloud.launch.py'
    )
    
    param_setter_launch = os.path.join(
        get_package_share_directory('airbot_sensor_manager'),
        'launch',
        'param_setter.launch.py'
    )
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensor_to_pointcloud_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(param_setter_launch)
        )
    ])
