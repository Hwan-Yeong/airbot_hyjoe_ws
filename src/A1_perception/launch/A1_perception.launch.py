from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    ld = LaunchDescription()
    node_kwargs = {
        "package": "A1_perception",
        "executable": "A1_perception",
        "output": "screen",
        "emulate_tty": True,
    }
    ros_distro = os.environ.get("ROS_DISTRO", "humble")

    config_file = os.path.join(
        get_package_share_directory("A1_perception"),
        "params",
        "params.yaml",
    )

    talker_node = Node(**node_kwargs, parameters=[config_file])

    ld.add_action(talker_node)
    return ld
