import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('north_goal_angle'), 'config', 'north_goal_angle.yaml')

    return LaunchDescription([
        Node(
            package='north_goal_angle',
            executable='north_goal_angle',
            name='north_goal_angle',   # config yaml 의 /north_goal_angle 키와 일치
            output='screen',
            parameters=[params],
        ),
    ])
