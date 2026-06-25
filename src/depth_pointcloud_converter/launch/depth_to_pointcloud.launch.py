#!/usr/bin/env python3
#
# depth_to_pointcloud 실행 launch.
#
# 이 launch 는 inusensor_ros2_driver 가 "이미 실행 중"이라고 가정한다.
# (드라이버가 /camera/depth/image_raw, /camera/depth/camera_info 를 발행하고
#  robot_state_publisher 가 camera_link -> ... -> inusensor_depth TF 를 발행함)
#
# 여기서 띄우는 것:
#   1) base_link -> camera_link static TF  (로봇 본체에 카메라를 붙임)
#   2) depth_to_pointcloud_node            (depth -> PointCloud2 변환)
#
# 노드 파라미터는 config/depth_to_pointcloud_params.yaml (share 에 설치됨)에서 로드한다.
# RViz는 이 launch에서 띄우지 않는다(별도 PC/세션에서 직접 실행).
#
# 실행 예:
#   ros2 launch depth_pointcloud_converter depth_to_pointcloud.launch.py
#   ros2 launch depth_pointcloud_converter depth_to_pointcloud.launch.py launch_static_tf:=false
#   ros2 launch depth_pointcloud_converter depth_to_pointcloud.launch.py \
#       params_file:=/path/to/custom_params.yaml

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('depth_pointcloud_converter')
    default_params = os.path.join(pkg_share, 'config', 'depth_to_pointcloud_params.yaml')

    # ---------------- launch arguments ----------------
    params_file_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Path to the ROS2 params yaml for depth_to_pointcloud_node')

    launch_static_tf_arg = DeclareLaunchArgument(
        'launch_static_tf', default_value='true',
        description='Publish base_link -> camera_link static TF')

    # base_link -> camera_link 부착 위치 (사용자 제원)
    cam_x_arg = DeclareLaunchArgument('cam_x', default_value='0.13')
    cam_y_arg = DeclareLaunchArgument('cam_y', default_value='0.0')
    cam_z_arg = DeclareLaunchArgument('cam_z', default_value='0.1')
    cam_roll_arg = DeclareLaunchArgument('cam_roll', default_value='0.0')
    cam_pitch_arg = DeclareLaunchArgument('cam_pitch', default_value='0.0')
    cam_yaw_arg = DeclareLaunchArgument('cam_yaw', default_value='0.0')

    # ---------------- nodes ----------------
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_link_static_tf',
        condition=IfCondition(LaunchConfiguration('launch_static_tf')),
        arguments=[
            '--x', LaunchConfiguration('cam_x'),
            '--y', LaunchConfiguration('cam_y'),
            '--z', LaunchConfiguration('cam_z'),
            '--roll', LaunchConfiguration('cam_roll'),
            '--pitch', LaunchConfiguration('cam_pitch'),
            '--yaw', LaunchConfiguration('cam_yaw'),
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link',
        ],
        output='screen',
    )

    converter_node = Node(
        package='depth_pointcloud_converter',
        executable='depth_to_pointcloud_node',
        name='depth_to_pointcloud_node',
        parameters=[LaunchConfiguration('params_file')],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        params_file_arg,
        launch_static_tf_arg,
        cam_x_arg,
        cam_y_arg,
        cam_z_arg,
        cam_roll_arg,
        cam_pitch_arg,
        cam_yaw_arg,
        static_tf_node,
        converter_node,
    ])
