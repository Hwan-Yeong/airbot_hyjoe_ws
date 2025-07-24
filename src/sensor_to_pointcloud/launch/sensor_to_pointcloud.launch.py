import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    parameter_file = LaunchConfiguration('params_file')
    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('sensor_to_pointcloud'),
            'config',
            'sensor_param.yaml'
        ),
        description='Path to the ROS2 parameters file to use.'
    )

    return LaunchDescription([
        params_declare,
        Node(
            name='sensor_to_pointcloud',
            package='sensor_to_pointcloud',
            executable='sensor_to_pointcloud',
            output='screen',
            parameters=[parameter_file],
        )
    ])
