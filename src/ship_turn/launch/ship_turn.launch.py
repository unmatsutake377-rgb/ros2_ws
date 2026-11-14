"""
ship_turn 단독 실행 (벤치 확인용). 전체 시스템은 launch_files 가 띄운다.

    ros2 launch ship_turn ship_turn.launch.py boat:=A     # 또는 boat:=B (필수)

🚨 `boat` 는 필수다(기본값 없음). 빠지거나 A/B 가 아니면 launch 가 거부한다.
   배마다 다른 키 9개(scan_* 7개 + orbit_target_deg + orbit_max_sec)는 config/ship_turn_<boat>.yaml 에 있다.
   잰 값은 **그 배의 파일**에 적는다 — 첫 줄 로그 "ship_turn boat=…, 파일=…" 로 어느 배 파일인지 확인할 것
   (계약서_ship_turn배별yaml_20261115). 공통 파일 → 배별 파일 순서(뒤 파일이 이긴다) — 대회 launch 와 같다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def ship_turn_param_files(share_dir, boat):
    """ship_turn 공통 yaml 뒤에 붙일 배별 파일 목록 = [<share>/config/ship_turn_<boat>.yaml].

    launch_files.launch.py 에 같은 함수가 있다 — 바꿀 땐 둘 다 (test_launch_params T9 가 검사).
    🚨 파일이 없으면 RuntimeError: 배별 값 없이 뜨면 코드 기본값으로 조용히 돈다 → launch 를 세운다.
    모르는 boat 는 ValueError — 조용히 다른 배 값으로 떨어지지 않는다.
    """
    if boat not in ('A', 'B'):
        raise ValueError(f"boat={boat!r} — 'A' 또는 'B'")
    path = os.path.join(share_dir, 'config', f'ship_turn_{boat}.yaml')
    if not os.path.isfile(path):
        raise RuntimeError(
            f'ship_turn 배별 파일 없음: {path} — pull 뒤 colcon build 를 했는지 확인')
    return [path]


def _ship_turn(context):
    boat = LaunchConfiguration('boat').perform(context)
    share = get_package_share_directory('ship_turn')
    common = os.path.join(share, 'config', 'ship_turn.yaml')
    boat_files = ship_turn_param_files(share, boat)   # 없으면 raise — Node 를 만들기 전에
    return [
        LogInfo(msg=f'ship_turn boat={boat}, 파일={common} → {boat_files[0]}'),
        Node(
            package='ship_turn',
            executable='ship_turn',
            name='ship_turn',      # config yaml 의 /ship_turn 키와 일치
            output='screen',
            # 🚨 순서가 의미다: 공통 → 배별 (뒤 파일이 이긴다)
            parameters=[common, *boat_files],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'boat', choices=['A', 'B'],
            description='배 (필수, 기본값 없음): A 또는 B — ship_turn 배별 값 파일 선택'),
        OpaqueFunction(function=_ship_turn),
    ])
