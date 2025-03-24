from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    explore_dir = get_package_share_directory('explore')
    wm_slam_dir = get_package_share_directory('wm_slam_toolbox_launcher')
    wm_nav_launch_dir = get_package_share_directory('wm_nav_launch');

    wm_slam_launch_file = os.path.join(wm_slam_dir, 'launch', 'wm_slam_toolbox_manu.launch.py')
    navigation_launch_file = os.path.join(wm_nav_launch_dir, 'launch', 'navigation_wave.launch.py')


    perception_launch_dir = get_package_share_directory('A1_perception')
    maneuver_launch_dir = get_package_share_directory('A1_maneuver');
    perception_launch_file = os.path.join(perception_launch_dir, 'launch', 'A1_perception.launch.py')
    maneuver_launch_file = os.path.join(maneuver_launch_dir, 'launch', 'A1_maneuver.launch.py')

    rviz_config_file = os.path.join(explore_dir, 'rviz', 'explore.rviz')
    explore_config_file = os.path.join(explore_dir, 'config', 'params_map.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(wm_slam_launch_file),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch_file),
        ),
#        IncludeLaunchDescription(
#            PythonLaunchDescriptionSource(perception_launch_file),
#        ),
#        IncludeLaunchDescription(
#            PythonLaunchDescriptionSource(maneuver_launch_file),
#        ),
        Node(
            package='warmup_server',
            executable='warmup_server_node'
        ),
        Node(
            package='explore',
            executable='explore',
            name='explore_node',
            parameters=[explore_config_file]
        ),
        # Uncomment if you want to start RViz as well
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config_file])
    ])
