"""
yaw_mux 단독 실행 (벤치 확인용).

전체 시스템은 launch_files 가 띄운다. 이건 IMU 부호·mount_offset 을 맞출 때 쓴다:

    ros2 launch ssf_heading ssf_heading.launch.py
    ros2 topic echo /imu/yaw          # 배를 정북에 놓고 0 이 나오게 mount_offset_deg 조정
    ros2 topic echo /heading_status   # imu_relative:OK / imu_relative:STALE ...
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('ssf_heading'), 'config', 'ssf_heading.yaml')

    return LaunchDescription([
        Node(
            package='ssf_heading',
            executable='yaw_mux',
            name='yaw_mux',
            output='screen',
            parameters=[config],
        ),
    ])
