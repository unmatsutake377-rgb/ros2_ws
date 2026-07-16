import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('ship_gate'), 'config', 'ship_gate.yaml')

    return LaunchDescription([
        Node(
            package='ship_gate',
            executable='ship_gate',
            name='ship_gate',      # config yaml 의 /ship_gate 키와 일치
            output='screen',
            parameters=[params],
        ),
    ])
