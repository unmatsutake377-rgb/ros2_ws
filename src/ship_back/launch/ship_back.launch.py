from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ship_back',
            executable='ship_back',  # <= 이 이름이 중요!
            name='ship_back',
            output='screen'
        )
    ])
