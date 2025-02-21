import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    params_file = LaunchConfiguration('params_file')

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
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                get_package_share_directory('airbot_sensor_manager'),
                'config',
                'sensor_to_pointcloud_param.yaml'
            ),
            description='Path to the ROS2 parameters file to use.'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('airbot_sensor_manager'),
                '/launch/sensor_to_pointcloud.launch.py'
            ]),
            launch_arguments={'params_file': params_file}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(param_setter_launch)
        )
    ])
