from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_name = "airbot_motion_anomaly_detector"
    config_path = os.path.join(get_package_share_directory(package_name), "config", "collision_params.yaml")

    return LaunchDescription([
        Node(
            package=package_name,
            executable="anomaly_detector",
            name="collision_detector",
            parameters=[config_path],  # Use absolute path
            output="screen",
        ),

        Node(
        package=package_name,
        executable="odom_monitor",
        name="odom_monitor",
        output="screen",
        ),
    ])
