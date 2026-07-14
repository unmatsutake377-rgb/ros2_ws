from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ship_goal_angle',  # 패키지 이름을 'ship_goal_angle'로 변경
            executable='ship_goal_angle_node',  # 실행할 노드 이름을 'ship_goal_angle'로 변경
            output='screen'),
    ])
