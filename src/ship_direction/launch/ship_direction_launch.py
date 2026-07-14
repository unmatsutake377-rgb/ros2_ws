from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ship_direction',  # Replace with your package name
            executable='ship_direction',  # Replace with your executable/script name if different
            name='ship_direction',
            output='screen',
            parameters=[
                # Add any parameters if needed, e.g., {'param_name': 'param_value'}
            ],
            remappings=[
                # Add any topic remappings if needed, e.g., ('/old_topic', '/new_topic')
            ]
        ),
    ])
