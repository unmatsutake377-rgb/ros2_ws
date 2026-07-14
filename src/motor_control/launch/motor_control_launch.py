import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('motor_control'), 'config', 'motor_control.yaml')

    return LaunchDescription([
        Node(
            package='motor_control',
            executable='motor_control',
            name='motor_control',        # config yaml 의 /motor_control 키와 일치
            output='screen',
            parameters=[params],
        ),
    ])
