import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('ship_back'), 'config', 'ship_back.yaml')

    return LaunchDescription([
        Node(
            package='ship_back',
            executable='ship_back',
            name='ship_back',      # config yaml 의 /ship_back 키와 일치
            output='screen',
            parameters=[params],
        ),
    ])
