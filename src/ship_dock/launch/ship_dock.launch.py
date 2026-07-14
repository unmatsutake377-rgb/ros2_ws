import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('ship_dock'), 'config', 'ship_dock.yaml')

    return LaunchDescription([
        Node(
            package='ship_dock',
            executable='ship_dock',
            name='ship_dock',      # config yaml 의 /ship_dock 키와 일치
            output='screen',
            parameters=[params],
        ),
    ])
