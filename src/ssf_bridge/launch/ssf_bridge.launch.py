"""ssf_bridge 실행.

    ros2 launch ssf_bridge ssf_bridge.launch.py
    ros2 launch ssf_bridge ssf_bridge.launch.py port:=/dev/ttyACM0

🚨 **포트가 없으면 노드를 띄우지 않고 그냥 넘어간다.**
   예외를 던지면 launch 전체가 죽는다 — ntrip_client 가 파일 하나를 못 찾아
   **이미 떠 있던 노드 15개까지 같이 죽인** 사고가 실제로 있었다
   (docs/결과분석/실기통합시험_LiDAR_IMU_20260806.md).
   아두이노는 아직 없으므로 이 경로를 반드시 안전하게 둔다.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# udev 로 고정한 이름. /dev/ttyACM* 는 꽂는 순서에 따라 번호가 바뀐다
# (LiDAR·IMU·GPS 에서 이미 겪어 세 장치 모두 심링크로 고정했다).
DEFAULT_PORT = "/dev/ttyMEGA"


def _setup(context, *args, **kwargs):
    port = LaunchConfiguration("port").perform(context)
    baud = LaunchConfiguration("baud").perform(context)

    if not os.path.exists(port):
        return [LogInfo(msg=(
            f"⏭️ ssf_bridge 건너뜀 — 시리얼 포트 {port} 가 없다. "
            f"아두이노를 안 꽂았거나 udev 규칙(SYMLINK+=\"ttyMEGA\")이 없다. "
            f"다른 포트면 port:=/dev/ttyACM0 처럼 지정."))]

    return [Node(
        package="ssf_bridge",
        executable="bridge",
        name="ssf_bridge",
        output="screen",
        parameters=[{"port": port, "baud": int(baud)}],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value=DEFAULT_PORT,
                              description="아두이노 Mega 시리얼 포트"),
        DeclareLaunchArgument("baud", default_value="115200",
                              description="펌웨어 SERIAL_BAUD 와 같아야 한다"),
        OpaqueFunction(function=_setup),
    ])
