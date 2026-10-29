import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    VISION_CFG = os.path.join(
        get_package_share_directory('color_shape_detector'), 'config', 'vision.yaml')

    return LaunchDescription([

        # ================================
        # RealSense Camera Only
        # (자동 화이트밸런스는 아래처럼 켜져 있다 — 옛 머리말 'Auto Exposure/WhiteBalance OFF' 는 틀린 설명.
        #  채도는 대회 launch(launch_files.launch.py)와 같게 `rgb_camera.saturation: 64` — 10-28 Ubuntu 확인:
        #  옛 `rgb_camera.controls.saturation: 120` 은 없는 이름이라 조용히 무시돼 실제로는 늘 64 였다.
        #  HSV 튜닝이 전부 64 기준이라 값은 64 그대로 명시만 한다. 대회 launch 는 이 파일이 아니다.)
        # ================================
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

                {"rgb_camera.saturation": 64},   # 실제로 돌던 값(옛 controls.saturation 120 은 무시됐었다)


                # 🎥 30FPS, 640x480
                # 🚨 [2026-09-10] 뎁스 OFF — 켜면 D455 가 USB 에서 떨어진다.
                #    컬러+뎁스 10.8fps/끊김13회 → 컬러전용 30.0fps/오류0.
                #    구독자가 없어 안전하다(V1 에서 depth 의존 제거, 거리는 LiDAR).
                #    자세한 측정은 launch_files.launch.py 의 같은 자리 주석 참고.
                {"enable_depth": False},
                {"enable_gyro": False},
                {"enable_accel": False},

                {"rgb_camera.color_profile": "640x480x30"},
            ]
        ),

        # ================================
        # 비전 검출기 — 상주 (3-4)
        # ================================
        # subscriber_mode_manager(subprocess 로 죽였다 살리기) 폐기.
        # 검출기는 항상 살아있고 /wp_mode 로 자기 차례를 안다.
        # 담당 모드는 config/vision.yaml 의 active_wp_modes 가 소유한다.
        Node(package='color_shape_detector', executable='basic_image_subscribergate',
             name='image_subscriber_gate', output='screen', parameters=[VISION_CFG]),
        Node(package='color_shape_detector', executable='basic_image_subscriberturn',
             name='image_subscriber_turn', output='screen', parameters=[VISION_CFG]),
        Node(package='color_shape_detector', executable='basic_image_subscriberdock',
             name='image_subscriber_dock', output='screen', parameters=[VISION_CFG]),
    ])
