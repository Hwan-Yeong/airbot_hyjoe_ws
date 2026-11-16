"""kebi follow-me: 검출·추적·제어 3개 컴포넌트를 한 프로세스(DDS 참여자 1개)로 띄운다.

ros2 launch kebi_follow_me follow_me.launch.py [shadow:=true] [use_detector:=true] [use_sim_time:=false] [params:=<override.yaml>]
- shadow:=true 면 /cmd_vel 대신 /kebi_follow/cmd_vel_shadow 로만 발행한다 (실차 1단계 시험용).
- use_sim_time:=true 는 bag 재생(--clock)용이다.
- 파라미터는 각 모듈 패키지의 config/<노드>.advanced.yaml(알고리즘 내부값) → config/<노드>.yaml(핵심·동작·안전·배관)
  → kebi_follow_me/config/robot.yaml(로봇 공통 물리 값: 카메라 숙임·지연, 로봇 반경) → params(덮어쓰기) 순서로 얹는다 (뒤가 이긴다).
"""
import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _share(pkg, *path):
    return os.path.join(get_package_share_directory(pkg), *path)


def _as_bool(text):
    return str(text).strip().lower() in ('1', 'true', 'yes', 'on')


def _setup(context):
    override = LaunchConfiguration('params').perform(context)
    shadow = _as_bool(LaunchConfiguration('shadow').perform(context))
    use_detector = _as_bool(LaunchConfiguration('use_detector').perform(context))
    sim_time = {'use_sim_time': _as_bool(LaunchConfiguration('use_sim_time').perform(context))}

    # 로봇 공통 물리 값 (config/robot.yaml): 노드마다 같은 뜻의 파라미터로 넣는다
    with open(_share('kebi_follow_me', 'config', 'robot.yaml')) as f:
        robot = yaml.safe_load(f)
    shared = {
        'cam_leg_detector': {'camera_pitch_correction_deg': robot['camera_pitch_deg'],
                             'camera_latency_s': robot['camera_latency_s'], 'robot_radius': robot['robot_radius']},
        'leg_tracker': {'scan.robot_radius': robot['robot_radius']},
        'follow_controller': {'camera_pitch_correction_deg': robot['camera_pitch_deg'],
                              'camera_latency_s': robot['camera_latency_s'], 'robot_radius': robot['robot_radius']},
    }

    def params(pkg, yaml_name):
        node = yaml_name.replace('.yaml', '')
        advanced = _share(pkg, 'config', node + '.advanced.yaml')
        files = ([advanced] if os.path.exists(advanced) else []) + [_share(pkg, 'config', yaml_name), shared[node]]
        if override:
            files.append(override)
        return files + [sim_time]

    nodes = []
    if use_detector:
        nodes.append(ComposableNode(
            package='kebi_follow_detector', plugin='kebi_follow::CamLegDetector',
            name='cam_leg_detector',
            parameters=params('kebi_follow_detector', 'cam_leg_detector.yaml')))
    nodes.append(ComposableNode(
        package='kebi_follow_tracker', plugin='kebi_follow::LegTracker',
        name='leg_tracker',
        parameters=params('kebi_follow_tracker', 'leg_tracker.yaml')))
    nodes.append(ComposableNode(
        package='kebi_follow_controller', plugin='kebi_follow::FollowController',
        name='follow_controller',
        parameters=params('kebi_follow_controller', 'follow_controller.yaml') + [{'shadow_mode': shadow}]))

    return [ComposableNodeContainer(
        name='kebi_follow_container', namespace='',
        package='rclcpp_components', executable='component_container_mt',
        composable_node_descriptions=nodes, output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=_share('kebi_follow_me', 'config', 'follow_me.yaml'),
                              description='덮어쓸 파라미터 파일 (빈 문자열이면 모듈 기본값만)'),
        DeclareLaunchArgument('shadow', default_value='false',
                              description='true: /cmd_vel 대신 /kebi_follow/cmd_vel_shadow 로 발행'),
        DeclareLaunchArgument('use_detector', default_value='true',
                              description='false: 카메라 검출 없이 (시험용. leg_tracker acquire.require_camera 가 true 면 락이 걸리지 않는다)'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='true: bag 재생 --clock 시간 사용'),
        OpaqueFunction(function=_setup),
    ])
