import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('ship_turn'), 'config', 'ship_turn.yaml')

    return LaunchDescription([
        Node(
            package='ship_turn',
            executable='ship_turn',
            name='ship_turn',      # config yaml 의 /ship_turn 키와 일치
            output='screen',
            parameters=[params],
        ),
    ])
