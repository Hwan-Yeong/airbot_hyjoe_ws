import os

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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    params_file = LaunchConfiguration('params_file')

    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('airbot_sensor_manager'),
            'config',
            'sensor_to_pointcloud_param.yaml'
        ),
        description='Path to the ROS2 parameters file to use.'
    )

    bringup_launch_file = os.path.join(
        get_package_share_directory('robot_bringup'),
        'launch', 'state_publisher.launch.py'
    )

    lidar_bringup_launch_file = os.path.join(
        get_package_share_directory('airbot_lidar'),
        'launch', 'lidar_dual_launch.py'
    )    

    laser_scan_merger_launch_file = os.path.join(
        get_package_share_directory('airbot_lidar'),
        'launch', 'merge_2_scan.launch.py'
    )

    state_manager_launch_file = os.path.join(
    get_package_share_directory('state_manager'),
        'launch',
        'state_manager.launch.py'
    )    
    
    sensor_to_pointcloud_launch = os.path.join(
        get_package_share_directory('airbot_sensor_manager'),
        'launch',
        'sensor_to_pointcloud.launch.py'
    )


    return LaunchDescription([
        declare_params_file_arg,

	#	#Platform Driver
        Node(
             package='robot_bringup',
             executable='robot_bringup',
             name='robot_bringup'),
        Node(
             package='airbot_log',
             executable='logging_amr',
             name='logging_amr'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([bringup_launch_file])
        ),
             
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_bringup_launch_file])
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([laser_scan_merger_launch_file])
        ),
       IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory(
               'udp_interface'), '/launch/udp_interface.launch.py'])
        ),

        IncludeLaunchDescription(
           PythonLaunchDescriptionSource([state_manager_launch_file])
       ),
       
       IncludeLaunchDescription(
           PythonLaunchDescriptionSource([get_package_share_directory(
              'A1_keepout'), '/launch/keepout.launch.py'])
       ),

       IncludeLaunchDescription(
           PythonLaunchDescriptionSource([get_package_share_directory(
              'A1_localization'), '/launch/localization.launch.py'])
       ),

       IncludeLaunchDescription(
           PythonLaunchDescriptionSource([get_package_share_directory(
              'A1_perception'), '/launch/A1_perception.launch.py'])
       ),


       IncludeLaunchDescription(
           PythonLaunchDescriptionSource([get_package_share_directory(
              'airbot_ai_interface'), '/launch/airbot_ai_interface_launch.py'])
       ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('airbot_sensor_manager'),
                '/launch/sensor_manager.launch.py'
            ]),
            launch_arguments={'params_file': params_file}.items()
        ),

        IncludeLaunchDescription(
           PythonLaunchDescriptionSource([get_package_share_directory(
              'airbot_error_manager'), '/launch/error_manager.launch.py'])
       ),
    ])

