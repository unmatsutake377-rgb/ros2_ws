#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the RPLIDAR A3 node with configurable runtime arguments.

    Notes:
    - ``/dev/ttyLiDAR`` requires a matching udev rule on each computer.
    - ``scan_mode`` remains ``Sensitivity`` by default until comparative
      water-test data supports changing it. It can be overridden at launch.
    - QoS for ``/scan`` subscribers belongs in the subscribing node, not here.
    """

    # Launch arguments are declared with literal defaults and read through
    # LaunchConfiguration. This keeps the runtime override path explicit.
    channel_type = LaunchConfiguration('channel_type')
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    frame_id = LaunchConfiguration('frame_id')
    inverted = LaunchConfiguration('inverted')
    angle_compensate = LaunchConfiguration('angle_compensate')
    scan_mode = LaunchConfiguration('scan_mode')

    return LaunchDescription([
        DeclareLaunchArgument(
            'channel_type',
            default_value='serial',
            description='Communication channel type for the LiDAR',
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyLiDAR',
            description=(
                'Serial device for the LiDAR. /dev/ttyLiDAR requires a '
                'matching udev rule on the host computer.'
            ),
        ),
        DeclareLaunchArgument(
            'serial_baudrate',
            default_value='256000',
            description='Serial baud rate; RPLIDAR A3 uses 256000',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='laser',
            description='TF frame ID assigned to LaserScan messages',
        ),
        DeclareLaunchArgument(
            'inverted',
            default_value='false',
            description='Whether to invert the scan data',
        ),
        DeclareLaunchArgument(
            'angle_compensate',
            default_value='true',
            description='Whether to enable angle compensation',
        ),
        DeclareLaunchArgument(
            'scan_mode',
            default_value='Sensitivity',
            description=(
                'RPLIDAR scan mode. Keep Sensitivity as the baseline and '
                'override this argument during Standard/Boost comparison tests.'
            ),
        ),
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'channel_type': channel_type,
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,
                'inverted': inverted,
                'angle_compensate': angle_compensate,
                'scan_mode': scan_mode,
            }],
        ),
    ])
