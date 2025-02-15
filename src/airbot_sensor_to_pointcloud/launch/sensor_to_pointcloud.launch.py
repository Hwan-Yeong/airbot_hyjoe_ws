import os
import yaml
import numpy as np
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir

##############################
### Camera Object Class ID ###
# 0: cable
# 1: carpet
# 2: clothes
# 3: liquid
# 4: non_obstacle
# 5: obstacle
# 6: poop
# 7: scale
# 8: threshold
# 9: person
# 10: dog
# 11: cat
##############################

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='airbot_sensor_to_pointcloud',
            executable='sensor_to_pointcloud',
            output='screen',
            name='airbot_sensor_to_pointcloud',
            parameters=[
            {"target_frame": "map"}, # "map" or "base_link"

            {"use.tof": True},
            {"use.tof.1D": True},
            {"use.tof.left": True},
            {"use.tof.right": True},
            {"use.tof.row": True},
            {"use.camera": True},
            {"use.cliff": True},

            {"camera.pointcloud_resolutionm": 0.05},
            {"camera.class_id_confidence_th": ["2: 55", "5: 55", "6: 55"]}, # "class_id:score_th"
            {"camera.object_direction": False}, # 정방향(CCW+):True, 역방향(CW+):False

            {"publish.rate_ms.tof_1d": 10},
            {"publish.rate_ms.tof_multi": 50},
            {"publish.rate_ms.tof_row": 50},
            {"publish.rate_ms.camera": 100},
            {"publish.rate_ms.cliff": 10},
        ]
        ),
    ])


    # config_file = os.path.join(
    #     get_package_share_directory('airbot_sensor_to_pointcloud'),
    #     'config',
    #     'airbot_sensor_to_pointcloud.yaml'
    # )

    # # YAML 파일 로드
    # with open(config_file, 'r') as file:
    #     params = yaml.safe_load(file)

    # return LaunchDescription([
    #     Node(
    #         name='airbot_sensor_to_pointcloud',
    #         package='airbot_sensor_to_pointcloud',
    #         executable='sensor_to_pointcloud',
    #         output='screen',
    #         parameters=[params],
    #     ),
    # ])