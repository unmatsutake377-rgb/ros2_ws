from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ship_turn',
            executable='ship_turn',  # <= 이 이름이 중요!
            name='ship_turn',
            output='screen'
        )
    ])
