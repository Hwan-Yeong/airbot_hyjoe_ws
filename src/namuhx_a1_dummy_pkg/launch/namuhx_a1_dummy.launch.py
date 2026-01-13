import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # parameter_file = LaunchConfiguration('params_file')
    # params_declare = DeclareLaunchArgument(
    #     'params_file',
    #     default_value=os.path.join(
    #         get_package_share_directory('namuhx_a1_dummy_pkg'),
    #         'config',
    #         'params.yaml'
    #     ),
    #     description='Path to the ROS2 parameters file to use.'
    # )

    return LaunchDescription([
        # params_declare,
        Node(
            name='namuhx_a1_dummy',
            package='namuhx_a1_dummy_pkg',
            executable='namuhx_a1_dummy',
            parameters=[
                {'uart_loop_rate_ms': 10},
                {'lidar_loop_rate_ms': 10}
            ],
            output='screen',
            # parameters=[parameter_file],
            respawn=True,
            respawn_delay=1.0
        )
    ])
