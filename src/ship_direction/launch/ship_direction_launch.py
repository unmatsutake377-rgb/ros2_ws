import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('ship_direction'), 'config', 'ship_direction.yaml')

    return LaunchDescription([
        Node(
            package='ship_direction',
            executable='ship_direction',
            name='ship_direction',      # config yaml 의 /ship_direction 키와 일치
            output='screen',
            parameters=[params],
        ),
    ])
