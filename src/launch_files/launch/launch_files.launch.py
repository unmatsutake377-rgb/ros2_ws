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
    dir_bridge = get_package_share_directory('ssf_bridge')

    # 비전 공통 설정 (image_topic, hfov_deg, debug_view).
    # 3-4 이후로는 launch 가 노드를 직접 띄우므로 파라미터가 **그대로 닿는다** —
    # V5 의 vision_* 중계(매니저가 --ros-args 로 넘기던 것)는 3-4 에서 제거됐다.
    VISION_CFG = os.path.join(
        get_package_share_directory('color_shape_detector'), 'config', 'vision.yaml')

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

                # 🚨 카메라 내장 IMU(자이로·가속도)를 끈다 — 안 끄면 노드가 예외로 죽는다.
                #    librealsense 가 D455 의 HID/IIO 센서를 열려고 하는데 udev 규칙이 없어
                #    Permission denied 가 나고, 그 예외가 rs_node_setup 의 센서 설정을 통째로
                #    중단시킨다 (2026-08-07 실기 확인):
                #      Failed to open .../iio:device1/scan_elements/in_anglvel_x_en
                #      → Error updating the sensors
                #    ⚠️ udev 규칙을 까는 방법도 있지만 **우리는 이 IMU 를 쓰지 않는다** —
                #      헤딩은 iAHRS(/imu/yaw_raw → yaw_mux)가 담당한다(CLAUDE.md 3-5).
                #      안 쓰는 센서 때문에 시스템에 규칙을 남기는 대신 명시적으로 끈다.
                #    ※ OAK(PoE=이더넷)로 바꾸면 USB HID 자체가 없어져 이 줄은 불필요해진다.
                {"enable_gyro": False},
                {"enable_accel": False},

                {"rgb_camera.controls.saturation": 120},


                # 🎥 30FPS, 640x480
                {"rgb_camera.color_profile": "640x480x30"},
                {"depth_module.depth_profile": "640x480x30"},
            ]
        ),

        # ============================================================
        # 8) 비전 검출기 — **상주**. 각자 /wp_mode 를 보고 자기 차례일 때만 일한다
        # ============================================================
        # 🚨 3-4: 작년엔 subscriber_mode_manager 가 subprocess 로 이 노드들을 죽였다 살렸다.
        #    모드 전환마다 비전이 몇 초 멈추고, 좀비가 남으면 카메라가 잠기고, 추적기를 못 썼다.
        #    지금은 상주 + 모드 게이팅이다. 매니저는 폐기했다(entry point 도 제거).
        #
        #    담당 모드는 **각 노드가 소유**한다 (미션 노드와 같은 패턴).
        #    권위 출처 = 미션 노드의 active_wp_mode. 바꿀 땐 양쪽을 함께 바꿔라:
        #      mode 0,1 → ship_gate  ← gate 검출기 (/red_angle, /green_angle)
        #      mode 2   → ship_back  ← turn 검출기 (white 부표, /image_angle)
        #      mode 3   → ship_turn  ← turn 검출기 (red 부표)
        #      mode 7   → ship_dock  ← dock 검출기 (/image_angle)
        #      mode 5,8 → 순수 회피(담당 없음) → 검출기 전부 비활성
        #
        #    🚨 dock 과 turn 은 둘 다 /image_angle 을 발행한다. 모드가 겹치면 한 토픽에
        #       발행자 2개가 되어 **에러 없이** 값이 섞인다. 겹침 없음은
        #       test_mode_gate.py 의 check_publisher_conflicts 가 정적으로 검사한다.
        Node(package='color_shape_detector', executable='basic_image_subscribergate',
             name='image_subscriber_gate', output='screen', parameters=[VISION_CFG]),
        Node(package='color_shape_detector', executable='basic_image_subscriberturn',
             name='image_subscriber_turn', output='screen', parameters=[VISION_CFG]),
        Node(package='color_shape_detector', executable='basic_image_subscriberdock',
             name='image_subscriber_dock', output='screen', parameters=[VISION_CFG]),
        # basic_image_subscriberhsv 는 튜닝 전용 — 대회 launch 에 넣지 않는다.

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

        # ============================================================
        # 12) 시리얼 브릿지 (Motor_run → 아두이노 Mega, 상태 → /boat_mode)
        # ============================================================
        # 🚨 이게 없으면 motor_control 이 Motor_run 을 내도 **받는 쪽이 없다.**
        #    사슬의 마지막 조각이다: motor_control → Motor_run → [브릿지] → Mega
        # 아두이노가 아직 없어도 안전하다 — 브릿지 launch 가 포트 존재를 먼저 보고
        # 없으면 노드를 안 띄우고 넘어간다(예외를 던지면 launch 전체가 죽는다.
        # ntrip_client 가 파일 하나 못 찾아 노드 15개를 같이 죽인 사고가 실제로 있었다).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(dir_bridge, 'launch', 'ssf_bridge.launch.py')
            )
        ),
    ])
