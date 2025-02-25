import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable

def generate_launch_description():
    return LaunchDescription([

        # Start the anomaly_detector node
        Node(
            package='airbot_motion_anomaly_detector',
            executable='anomaly_detector',
            name='anomaly_detector',
            output='screen',
        ),
    ])
