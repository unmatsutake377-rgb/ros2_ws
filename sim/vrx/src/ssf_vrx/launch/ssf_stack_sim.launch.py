"""드라이버 없는 대회 스택 — 시뮬 전용.

대회 launch(launch_files.launch.py)에서 **센서 드라이버(rplidar·iahrs·ublox·ntrip·OAK)와 ssf_bridge 만 뺀** 것.
나머지(yaw_mux·north_goal_angle·ship_goal_angle·검출기 3종·임무 노드 4종·ship_direction·
motor_control·blackbox)는
대회 launch 파일의 **같은 함수와 같은 yaml** 로 띄운다. 대회 launch 모듈을 share 경로에서 import 해 쓰므로
대회 배선이 바뀌면 여기도 같이 바뀐다(복사본이 아니다).

  ros2 launch ssf_vrx ssf_stack_sim.launch.py boat:=A [search_behavior:=hold] [record:=true]
"""
import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _load_main_launch():
    """대회 launch 파일을 모듈로 읽는다(설치된 share 경로). 실패하면 이유를 그대로 올린다."""
    path = os.path.join(get_package_share_directory('launch_files'), 'launch',
                        'launch_files.launch.py')
    spec = importlib.util.spec_from_file_location('ssf_main_launch', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    for fn in ('_yaw_mux', '_search_nodes', '_blackbox', '_pkg_yaml'):
        if not hasattr(mod, fn):
            raise RuntimeError(f'{path} 에 {fn} 이 없다 — 대회 launch 가 바뀌었으면 이 파일도 맞출 것')
    return mod


def _stack(context):
    m = _load_main_launch()
    vision_cfg = os.path.join(get_package_share_directory('color_shape_detector'),
                              'config', 'vision.yaml')
    nodes = []
    nodes += m._yaw_mux(context)                       # 1) 헤딩 (boat 인자로 배별 yaml)
    nodes.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('north_goal_angle'), 'launch', 'north_goal_angle_launch.py'))))
    nodes.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('ship_goal_angle'), 'launch',
        'ship_goal_angle_launch_file.launch.py'))))
    for exe, name in (('basic_image_subscribergate', 'image_subscriber_gate'),
                      ('basic_image_subscriberturn', 'image_subscriber_turn'),
                      ('basic_image_subscriberdock', 'image_subscriber_dock')):
        nodes.append(Node(package='color_shape_detector', executable=exe, name=name,
                          output='screen', parameters=[vision_cfg]))
    nodes.append(Node(package='ship_gate', executable='ship_gate', name='ship_gate',
                      output='screen', parameters=[m._pkg_yaml('ship_gate')]))
    nodes.append(Node(package='ship_dock', executable='ship_dock', name='ship_dock',
                      output='screen', parameters=[m._pkg_yaml('ship_dock')]))
    nodes += m._search_nodes(context)                  # ship_back · ship_turn (search_behavior)
    nodes += m._ship_avoid(context)                    # mode 5 담당(11-13 C2, 기본 lidar = 20000)
    nodes.append(Node(package='ship_direction', executable='ship_direction', name='ship_direction',
                      output='screen', parameters=[m._pkg_yaml('ship_direction')]))
    nodes.append(Node(package='motor_control', executable='motor_control', name='motor_control',
                      output='screen', parameters=[m._pkg_yaml('motor_control')]))
    nodes += m._blackbox(context)                      # record 인자
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('boat', choices=['A', 'B'], description='배 (필수)'),
        DeclareLaunchArgument('search_behavior', default_value='hold', choices=['hold', 'orbit']),
        DeclareLaunchArgument('record', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=_stack),
    ])
