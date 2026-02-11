from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 첫 번째 패키지(jslloc) 경로
    jslloc_pkg = get_package_share_directory('jslloc')
    jslloc_launch = os.path.join(jslloc_pkg, 'launch', 'jslloc.launch.py')

    # 두 번째 패키지(localization) 경로
    localization_pkg = get_package_share_directory('airbot_navigation')
    localization_launch = os.path.join(localization_pkg, 'launch', 'no_amcl_localization_launch.py')

    # IncludeLaunchDescription 으로 각각의 launch 파일 포함
    jslloc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(jslloc_launch)
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch)
    )

    return LaunchDescription([
        jslloc,
        localization
    ])