from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='north_goal_angle',
            executable='north_goal_angle',
            name='north_goal_angle',
            output='screen',
        ),
    ])
