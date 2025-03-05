from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # nav2_bringup_dir = get_package_share_directory('navigation_launch')
    # airbot_explore_dir = get_package_share_directory('airbot_explore')
    # airbot_slam_dir = get_package_share_directory('airbot_slam')
    explore_dir = get_package_share_directory('explore')
    wm_slam_dir = get_package_share_directory('wm_slam_toolbox_launcher')
    wm_nav_launch_dir = get_package_share_directory('wm_nav_launch');

    #slam_launch_file = os.path.join(explore_dir, 'launch', 'slam.launch.py')
    wm_slam_launch_file = os.path.join(wm_slam_dir, 'launch', 'wm_slam_toolbox.launch.py')
    navigation_launch_file = os.path.join(wm_nav_launch_dir, 'launch', 'wm_explore_nav2_launch.py')
    # explore_demo_file = os.path.join(airbot_explore_dir, 'launch', 'explore_demo.py')

    rviz_config_file = os.path.join(explore_dir, 'rviz', 'explore.rviz')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(wm_slam_launch_file),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch_file),
        ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(explore_demo_file),
        # ),
        Node(
            package='warmup_server',
            executable='warmup_server_node'
        ),
        Node(
            package='explore',
            executable='explorer',
            name='explorer_node',
            parameters=[
                {"potential_scale": 1.0},
                {"gain_scale": 0.0},    # 1.0
                {"min_frontier_size": 0.1},
                {"orientation_scale": 0.0},
                {"progress_timeout": 30},   # 30
                {"visualize": False}
            ]
        ),
        # Uncomment if you want to start RViz as well
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config_file])
    ])
