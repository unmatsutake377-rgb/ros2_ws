from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

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
        # Your Image Subscriber Node
        # ================================
        Node(
            package='color_shape_detector',
            executable='basic_image_subscribermode',
            name='ImageSubscriber',
            output='screen'
        )
    ])
