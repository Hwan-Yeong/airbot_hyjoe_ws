from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    param_file = LaunchConfiguration('airbot_lidar_params')

    scan_params_declare = DeclareLaunchArgument(
        'airbot_lidar_params',
        default_value=os.path.join(
            get_package_share_directory('airbot_lidar'),
            'config',
            'airbot_lidar_params.yaml'
        ),
        description='Path to airbot_lidar parameters file.'
    )

    scan_front_node = Node(
        package='airbot_lidar',
        executable='airbot_lidar',
        name='airbot_lidar_front',
        output='screen',
        emulate_tty=True,
        parameters=[param_file],
        namespace='/',
        remappings=[
            ('/scan', '/scan_front'),  # '/scan'을 '/scan2'로 remap
            ('/scan_error', '/scan_error_front'),  # '/scan'을 '/scan2'로 remap
            ('/scan_dirty', '/scan_dirty_front')
        ]
    )

    scan_back_node = Node(
        package='airbot_lidar',
        executable='airbot_lidar',
        name='airbot_lidar_back',
        output='screen',
        emulate_tty=True,
        parameters=[param_file],
        namespace='/',
        remappings=[
            ('/scan', '/scan_back'),  # '/scan'을 '/scan2'로 remap
            ('/scan_error', '/scan_error_back'),  # '/scan'을 '/scan2'로 remap
            ('/scan_dirty', '/scan_dirty_back')
        ]
    )

    return LaunchDescription([
        scan_params_declare,
        scan_front_node,
        scan_back_node,
    ])
