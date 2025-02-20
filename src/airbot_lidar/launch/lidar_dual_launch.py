from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    share_dir = get_package_share_directory('airbot_lidar')
    parameter_file = LaunchConfiguration('params_file')
    scan2_param_file = LaunchConfiguration('scan2_params_file')
    # 기본 노드 파라미터 파일
    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'params', 'airbot_lidar.yaml'),
        description='Path to the ROS2 parameters file to use.'
    )

    # 추가 파라미터 파일
    scan2_params_declare = DeclareLaunchArgument(
        'scan2_params_file',
        default_value=os.path.join(share_dir, 'params', 'airbot_lidar_2.yaml'),
        description='Path to the ROS2 parameters file for scan2.'
    )

    # 첫 번째 노드 (기본 파라미터 파일 사용)
    scan1_node = Node(
        package='airbot_lidar',
        executable='airbot_lidar',
        name='airbot_lidar',
        output='screen',
        emulate_tty=True,
        parameters=[parameter_file],
        namespace='/',
        remappings=[
            ('/scan', '/scan_front'),  # '/scan'을 '/scan2'로 remap
            ('/scan_error', '/scan_error_front'),  # '/scan'을 '/scan2'로 remap
            ('/scan_dirty', '/scan_dirty_front')
        ]
    )

    # 두 번째 노드 (추가 파라미터 파일 사용)
    scan2_node = Node(
        package='airbot_lidar',
        executable='airbot_lidar',
        name='airbot_lidar_2',
        output='screen',
        emulate_tty=True,
        parameters=[scan2_param_file],
        namespace='/',
        remappings=[
            ('/scan', '/scan_back'),  # '/scan'을 '/scan2'로 remap
            ('/scan_error', '/scan_error_back'),  # '/scan'을 '/scan2'로 remap
            ('/scan_dirty', '/scan_dirty_back')
        ]
    )

    # base_link_to_laser_tf_node 노드
    # base_link_to_laser_tf_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='base_scan_to_base_scan_front',
    #     arguments=['0.15','0.0','0.0','3.14','0','0','base_scan','base_scan_front']
    # )

    # base_link_to_laser2_tf_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='base_scan_to_base_scan_back',
    #     arguments=['-0.15', '0.0', '0.0', '0.0', '0', '0', 'base_scan', 'base_scan_back']    )

    return LaunchDescription([
        params_declare,
        scan2_params_declare,
        scan1_node,
        scan2_node,
        # base_link_to_laser_tf_node,
        # base_link_to_laser2_tf_node,
    ])
