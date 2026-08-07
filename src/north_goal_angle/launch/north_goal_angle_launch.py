"""north_goal_angle launch — 웨이포인트 파일을 인자로 바꿔 끼울 수 있다.

  ros2 launch north_goal_angle north_goal_angle_launch.py
      → config/waypoints.yaml (실전 파일)

  ros2 launch north_goal_angle north_goal_angle_launch.py \
      waypoints_file:=/home/ssfb/ssf_logs/waypoints_recorded.yaml
      → 방금 녹화한 파일을 **복사 없이** 바로 사용

🚨 왜 인자화했나 (2026-08-07 야외 시험에서 나온 필요)
   waypoint_recorder 는 안전을 위해 **새 파일**에만 쓴다(실전 waypoints.yaml 을 안 건드림).
   그런데 그 파일을 쓰려면 지금까지는 "복사 → 재빌드" 를 해야 했다.
   대회장에서 좌표를 찍고 바로 미션을 돌려야 하는데, 손이 많이 가고 실수 여지가 크다.
   노드는 이미 waypoints_file 파라미터를 받고 있었는데(north_goal_angle.py) launch 가
   그걸 노출하지 않아 **쓸 방법이 없었다** — ublox_gps launch 가 c94_m8p 에 하드코딩돼
   F9P 를 못 쓰던 것(c1b0d81)과 같은 유형이다.

   ★ 녹화 파일을 인자로 쓰면 config/waypoints.yaml 은 그대로 남는다 → 오입력해도 복구 가능.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('north_goal_angle')
    params = os.path.join(share, 'config', 'north_goal_angle.yaml')
    default_wp = os.path.join(share, 'config', 'waypoints.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'waypoints_file',
            default_value=default_wp,
            description='웨이포인트 yaml 경로 (녹화본을 바로 쓰려면 그 경로를 준다)'),

        Node(
            package='north_goal_angle',
            executable='north_goal_angle',
            name='north_goal_angle',   # config yaml 의 /north_goal_angle 키와 일치
            output='screen',
            parameters=[
                params,
                # 🚨 순서가 중요하다. 뒤에 오는 것이 이긴다 —
                #    params(yaml) 에 waypoints_file 이 있더라도 인자가 덮어쓴다.
                {'waypoints_file': LaunchConfiguration('waypoints_file')},
            ],
        ),
    ])
