from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    ld = LaunchDescription()
    node_kwargs = {
        "package": "A1_maneuver",
        "executable": "A1_maneuver",
        "output": "screen",
        "emulate_tty": True,
    }

    config_file = os.path.join(
        get_package_share_directory("A1_maneuver"),
        "params",
        "params.yaml",
    )

    talker_node = Node(**node_kwargs, parameters=[config_file])

    ld.add_action(talker_node)
    return ld