import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('ship_goal_angle'), 'config', 'ship_goal_angle.yaml')

    return LaunchDescription([
        Node(
            package='ship_goal_angle',
            executable='ship_goal_angle_node',
            name='ship_goal_angle',        # config yaml 의 /ship_goal_angle 키와 일치
            output='screen',
            parameters=[params],
        ),
    ])
