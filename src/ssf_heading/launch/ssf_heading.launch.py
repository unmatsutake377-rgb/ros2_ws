"""
yaw_mux 단독 실행 (벤치 확인용).

전체 시스템은 launch_files 가 띄운다. 이건 IMU 부호·mount_offset 을 맞출 때 쓴다:

    ros2 launch ssf_heading ssf_heading.launch.py boat:=A     # 또는 boat:=B (필수)
    ros2 topic echo /imu/yaw          # 목표값·절차는 config/ssf_heading_<boat>.yaml 머리말
    ros2 topic echo /heading_status   # imu_relative:OK / imu_relative:STALE ...

🚨 `boat` 는 필수다(기본값 없음). 빠지거나 A/B 가 아니면 launch 가 거부한다.
   잰 값은 **그 배의 파일**(config/ssf_heading_<boat>.yaml)에 적는다 — 첫 줄 로그
   "boat=…, 파일=…" 로 어느 배 파일인지 확인할 것 (계약서_배별IMU설정_20261029).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def yaw_mux_param_files(share_dir, boat):
    """yaw_mux params 파일 목록. 🚨 순서가 의미다: 공통 → 배별 (뒤 파일이 이긴다).

    launch_files.launch.py 에 같은 함수가 있다 — 바꿀 땐 둘 다 (test_boat_config.py 가 검사).
    """
    cfg = os.path.join(share_dir, 'config')
    return [os.path.join(cfg, 'ssf_heading.yaml'),
            os.path.join(cfg, f'ssf_heading_{boat}.yaml')]


def _yaw_mux(context):
    boat = LaunchConfiguration('boat').perform(context)
    files = yaw_mux_param_files(get_package_share_directory('ssf_heading'), boat)  # 공통 → 배별
    missing = [f for f in files if not os.path.isfile(f)]
    if missing:
        # 일부러 launch 를 세운다 — 배별 값 없이 뜨면 헤딩이 조용히 틀린다(어떤 Node 보다 먼저).
        raise RuntimeError(
            f'yaw_mux params 파일 없음: {missing} — pull 뒤 colcon build 를 했는지 확인')
    return [
        LogInfo(msg=f'boat={boat}, 파일={files[0]} → {files[1]}'),
        Node(
            package='ssf_heading',
            executable='yaw_mux',
            name='yaw_mux',
            output='screen',
            parameters=files,
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'boat', choices=['A', 'B'],
            description='배 (필수, 기본값 없음): A 또는 B — yaw_mux 배별 IMU 값 파일 선택'),
        OpaqueFunction(function=_yaw_mux),
    ])
