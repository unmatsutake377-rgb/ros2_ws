from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node


def generate_launch_description():

    # 패키지 경로
    dir_rplidar = get_package_share_directory('rplidar_ros')
    dir_north = get_package_share_directory('north_goal_angle')
    dir_ntrip = get_package_share_directory('ntrip_client')
    dir_gps = get_package_share_directory('ublox_gps')
    dir_ship_direction = get_package_share_directory('ship_direction')
    dir_ship_goal = get_package_share_directory('ship_goal_angle')
    dir_realsense = get_package_share_directory('realsense2_camera')

    # NTRIP 2초 지연
    ntrip_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dir_ntrip, 'launch', 'ntrip_client_launch.py')
        )
    )

    return LaunchDescription([

        # ============================================================
        # 1) RPLIDAR
        # ============================================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(dir_rplidar, 'launch', 'rplidar_a3_launch.py')
            )
        ),

        # ============================================================
        # 2) AHRS (IMU) — 보정 안 한 상대 yaw 를 /imu/yaw_raw 로만 낸다
        # ============================================================
        # 🚨 CLAUDE.md 3-5. 작년엔 이 노드가 /imu/yaw 를 직접 냈는데 그 값이 절대방위가 아니었다
        #    (부팅 0점화 + GPS COG override). 절대방위 합성은 아래 yaw_mux 가 전담한다.
        #    파라미터는 드라이버 기본값과 같지만 **일부러 명시**한다 —
        #    나중에 누가 yaml 로 되살려도 여기서 눈에 띄게 하려는 것이다.
        Node(
            package='iahrs_driver',
            executable='iahrs_driver',
            name='iahrs_driver',
            output='screen',
            parameters=[{
                'yaw_topic': '/imu/yaw_raw',        # ← /imu/yaw 로 되돌리면 발행자 2개가 된다
                'zero_yaw_on_boot': False,
                'use_gps_heading_override': False,
            }],
        ),

        # ============================================================
        # 2b) yaw_mux — /imu/yaw 의 단독 발행자
        # ============================================================
        # 입력이 끊기거나 소스가 미구현이면 발행을 멈춘다(설계).
        # → ship_goal_angle 이 /yaw_error 를 멈추고, north_goal_angle geofence 가 침묵한다.
        Node(
            package='ssf_heading',
            executable='yaw_mux',
            name='yaw_mux',
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory('ssf_heading'),
                'config', 'ssf_heading.yaml')],
        ),

        # ============================================================
        # 3) U-BLOX GPS
        # ============================================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(dir_gps, 'launch', 'ublox_gps_node-launch.py')
            )
        ),

        # ============================================================
        # 4) NTRIP 2초 딜레이
        # ============================================================
        TimerAction(
            period=2.0,
            actions=[ntrip_launch]
        ),

        # ============================================================
        # 5) NORTH 기준 각도 계산
        # ============================================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(dir_north, 'launch', 'north_goal_angle_launch.py')
            )
        ),

        # ============================================================
        # 6) SHIP goal angle
        # ============================================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ship_goal_angle'),
                             'launch', 'ship_goal_angle_launch_file.launch.py')
            )
        ),

        # ============================================================
        # 7) RealSense Camera (Auto Exposure/WhiteBalance OFF)
        # ============================================================
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            output='screen',
            parameters=[
                {"rgb_camera.enable_auto_white_balance": True},
                {"enable_infra1": False},
                {"enable_infra2": False},

                {"rgb_camera.controls.saturation": 120},


                # 🎥 30FPS, 640x480
                {"rgb_camera.color_profile": "640x480x30"},
                {"depth_module.depth_profile": "640x480x30"},
            ]
        ),

        # ============================================================
        # 8) IMAGE 처리 (부표 색상 & 각도/거리 Publisher)
        # ============================================================
        Node(
            package='color_shape_detector',
            executable='basic_image_subscribermode',
            name='ImageSubscriber',
            output='screen'
        ),

        # ============================================================
        # 9) SHIP mission nodes (WP 루틴)
        # ============================================================
        Node(package='ship_gate', executable='ship_gate', name='ship_gate', output='screen'),
        Node(package='ship_dock', executable='ship_dock', name='ship_dock', output='screen'),
        Node(package='ship_turn', executable='ship_turn', name='ship_turn', output='screen'),
        # ship_last 제거됨(6b): mode 0 을 ship_gate 가 인수. 하던 일은 /candidate_angle 에 20000
        #   폴백을 내는 것뿐이라 중복이었다. (mode 5,8 폴백은 north_goal_angle 이 담당)
        Node(package='ship_back', executable='ship_back', name='ship_back', output='screen'),

        # ============================================================
        # 10) Fusion: 라이다 기반 항로 유지
        # ============================================================
        Node(
            package='ship_direction',
            executable='ship_direction',
            name='ship_direction',
            output='screen'
        ),

        # ============================================================
        # 11) Motor Control
        # ============================================================
        Node(
            package='motor_control',
            executable='motor_control',
            name='motor_control',
            output='screen'
        ),
    ])
