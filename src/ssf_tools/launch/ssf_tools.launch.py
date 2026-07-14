"""ssf_tools 통합 실행: blackbox + healthcheck (둘 다 구독 전용).

사용:
  ros2 launch ssf_tools ssf_tools.launch.py
  ros2 launch ssf_tools ssf_tools.launch.py boat:=b   # boat_b 설정이 생기면
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("ssf_tools")
    params = os.path.join(share, "config", "ssf_tools.yaml")

    return LaunchDescription([
        Node(
            package="ssf_tools",
            executable="blackbox",
            name="blackbox",
            output="screen",
            parameters=[params],
        ),
        Node(
            package="ssf_tools",
            executable="healthcheck",
            name="healthcheck",
            output="screen",
            parameters=[params],
        ),
    ])
