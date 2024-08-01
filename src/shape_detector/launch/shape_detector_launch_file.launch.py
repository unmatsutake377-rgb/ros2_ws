from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[{
                'video_device': '/dev/video2',
                'framerate': 30.0,
                'pixel_format': 'yuyv',
                'image_width': 640,
                'image_height': 480,
                'camera_frame_id': 'usb_cam',
                'brightness': 120,  # 밝기 조정
                'contrast': 28,  # 대비 조정
                'exposure_auto': 1,  # 자동 노출 비활성화
                'exposure_absolute': 200  # 노출 값 설정
            }]
        ),
        Node(
            package='shape_detector',
            executable='shape_detector',
            name='shape_detector',
            output='screen',
        )
    ])