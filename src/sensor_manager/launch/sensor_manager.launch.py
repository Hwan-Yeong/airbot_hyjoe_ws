import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    parameter_file = LaunchConfiguration('params_file')
    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('sensor_manager'),
            'config',
            'sensor_param.yaml'
        ),
        description='Path to the ROS2 parameters file to use.'
    )

    param_manager_launch = os.path.join(
        get_package_share_directory('sensor_manager'),
        'launch',
        'param_manager.launch.py'
    )

    return LaunchDescription([
        params_declare,
        Node(
            name='sensor_manager_node',
            package='sensor_manager',
            executable='sensor_manager',
            output='screen',
            parameters=[parameter_file],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(param_manager_launch)
        )
    ])
