from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([

        #ExecuteProcess(
        #    cmd=['/usr/bin/nice', '-n', '-10', 'ros2', 'run', 'airbot_ai_interface', 'airbot_ai_interface_node'], 
        #    output="screen"
        #),
        Node(
            package='airbot_node_manager',
            executable='node_manager',
            name='airbot_node_manager',
            output='screen',
        ),
    ])
