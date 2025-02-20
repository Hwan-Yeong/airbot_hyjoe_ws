from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Get the path to the package's share directory
    share_dir = get_package_share_directory('airbot_lidar')

    # Define the parameter file path (airbot_lidar_2.yaml)
    param_file_name = 'airbot_lidar_2.yaml'
    param_file_path = os.path.join(share_dir, 'params', param_file_name)

    # Declare the launch argument for the parameter file path
    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=param_file_path,
        description='Path to the ROS2 parameters file to use'
    )

    # Define the LifecycleNode for the airbot_lidar_2 node
    driver_node = LifecycleNode(
        package='airbot_lidar',
        executable='airbot_lidar_2',
        name='airbot_lidar_2',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('params_file')],
        namespace='lidar_2' , # Optional namespace for the node
        remappings=[
            ('/lidar_2/scan_2', '/scan_2'),
        ]
    )

    # Define the static transform publisher node
    base_link_to_laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_laser2_ld19',
        arguments=['-0.10', '0.0', '0.0', '0.0', '0', '0', 'base_link', 'laser_link_2']    )

    # Create the LaunchDescription with declared arguments and nodes
    return LaunchDescription([
        params_declare,
        driver_node,
        base_link_to_laser_tf_node,
    ])


