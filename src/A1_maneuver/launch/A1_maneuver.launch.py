import os
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



def generate_launch_description():
    return LaunchDescription([
         Node(
             package='A1_maneuver',
             executable='A1_maneuver',
             name='A1_maneuver',
             parameters=[
                {"time_ms.timer": 100},                 # 타이머를 구동할 시간
                {"time_ms.wait.normal": 1000},          # 일반적인 후진
                {"time_ms.wait.drop_off": 1000},        # drop off 검출시 wait 시간
                {"time_ms.back.1d": 2000},              # 1D ToF 검출시 뒤로 이동할 시간(ms), 속도는 -0.1
                {"time_ms.back.abort": 1500},           # Abort시 뒤로 이동할 시간(ms), 속도는 -0.1
                {"time_ms.back.collision": 2000},       # 충돌 시 후진 시간

                {"lidar.escape.front.distance": 0.25},  # lidar 전방 탈출 검사 distance 검출
                {"lidar.escape.front.angle": 60.0},     # lidar 전방 탈출 검사 angle 검출
                {"lidar.escape.back.distance": 0.25},   # lidar 후방 이동 distanc 검출
                {"lidar.escape.back.angle": 45.0},      # lidar 후방 이동 angle 검출
                
                {"velocity_scaling_factor": 0.075},     # 가속에 관한 스케일링 팩터
             ]
        ),
    ])