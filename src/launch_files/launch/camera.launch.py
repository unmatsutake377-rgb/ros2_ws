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
        # (Auto Exposure/WhiteBalance OFF)
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

                {"rgb_camera.controls.saturation": 120},


                # 🎥 30FPS, 640x480
                {"rgb_camera.color_profile": "640x480x30"},
                {"depth_module.depth_profile": "640x480x30"},
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
