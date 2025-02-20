import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   share_dir = get_package_share_directory('wm_slam_toolbox')
   config = os.path.join(share_dir, 'config', 'mapper_params_online_async.yaml')

   return LaunchDescription([
      Node(
         package='slam_toolbox',
         executable='async_slam_toolbox_node',
         namespace='',
         name='slam_toolbox',
         output='screen',
         parameters=[config]
      )
   ])


