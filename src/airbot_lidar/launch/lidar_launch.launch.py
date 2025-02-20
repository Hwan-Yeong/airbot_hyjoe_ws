
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo

import lifecycle_msgs.msg
import os


def generate_launch_description():
    share_dir = get_package_share_directory('airbot_lidar')
    parameter_file = LaunchConfiguration('params_file')
    node_name = 'airbot_lidar'

    params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                               share_dir, 'params', 'airbot_lidar.yaml'),
                                           description='FPath to the ROS2 parameters file to use.')

    driver_node = LifecycleNode(package='airbot_lidar',
                                executable='airbot_lidar',
                                name='airbot_lidar',
                                output='screen',
                                emulate_tty=True,
                                parameters=[parameter_file],
                                namespace='/',
                                )

    base_link_to_laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_laser_ld19',
        arguments=['0.20','0.0','0.0','3.14','0','0','base_link','laser_link']
    )

    return LaunchDescription([
        params_declare,
        driver_node,
        base_link_to_laser_tf_node,
    ])
