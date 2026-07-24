#!/usr/bin/env python3
import os
import signal
import subprocess
import time
from typing import Optional, Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32

# 🚩 wp_mode별 실행 매핑
MODE_TO_EXEC: Dict[int, tuple[str, str]] = {
    1: ("color_shape_detector", "basic_image_subscribergate"),  # Gate mission
    2: ("color_shape_detector", "basic_image_subscriberturn"),  # Turn mission
    9: ("color_shape_detector", "basic_image_subscriberdock"),  # Dock mission
    3: ("color_shape_detector", "basic_image_subscriberturn"),  # Turn mission
    4: ("color_shape_detector", "basic_image_subscriberturn"),  # Turn mission
    5: ("color_shape_detector", "basic_image_subscriberturn"),  # Turn mission
    10: ("color_shape_detector", "basic_image_subscriberhsv") # 원격토너먼트
}

# 🚫 Vision OFF 모드 (비전 노드를 끄는 모드)
NO_VISION_MODES = {0, 6, 7, 8}


class SubscriberModeManager(Node):
    """FSM 모드(/wp_mode)에 따라 비전 구독 노드를 자동으로 전환하는 매니저"""
    def __init__(self):
        super().__init__("subscriber_mode_manager")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # /wp_mode 구독
        self.create_subscription(Int32, "/wp_mode", self._on_mode_change, qos)

        self._current_mode: Optional[int] = None
        self._child: Optional[subprocess.Popen] = None

        # V4(T2-5): 자식 비전 노드에 debug_view 를 전달한다.
        #   자식은 `ros2 run` 으로 뜨므로 launch 파라미터가 안 닿는다 — 안 넘기면 debug_view 가
        #   대회 실행 경로에서 영원히 기본값(false)이라 벤치에서 켤 방법이 없다.
        #   ⚠️ 이 subprocess 구조 자체가 CLAUDE.md 3-4 의 제거 대상이다. 그때 같이 사라진다.
        self.vision_debug_view = bool(
            self.declare_parameter("vision_debug_view", False).value)
        # V5(T2-2): 이미지 토픽도 같이 넘긴다. 안 넘기면 자식이 항상 기본값(RealSense)을 쓰게 되어
        #   OAK 전환 시 yaml 을 고쳐도 대회 실행 경로에는 안 먹는다(죽은 설정).
        self.vision_image_topic = str(self.declare_parameter(
            "vision_image_topic", "/camera/camera/color/image_raw").value)
        self.vision_hfov_deg = float(
            self.declare_parameter("vision_hfov_deg", 71.5).value)

        #self.get_logger().info("🎛️ subscriber_mode_manager 시작됨 — /wp_mode 대기 중")

    # 🧭 /wp_mode 토픽 수신 콜백
    def _on_mode_change(self, msg: Int32):
        mode = int(msg.data)
        if mode == self._current_mode:
            return  # 같은 모드면 무시
        #self.get_logger().info(f"🔁 /wp_mode 변경: {self._current_mode} → {mode}")
        self._switch_to_mode(mode)

    # 🔄 모드 전환 처리
    def _switch_to_mode(self, mode: int):
        # 1️⃣ 기존 실행 중인 노드 종료
        self._stop_child()

        # 2️⃣ 비전 OFF 모드인 경우
        if mode in NO_VISION_MODES:
            #self.get_logger().info(f"🛑 비전 OFF 모드 (mode={mode}) → 모든 비전 노드 종료 유지")
            self._current_mode = mode
            return

        # 3️⃣ 해당 모드 실행
        if mode in MODE_TO_EXEC:
            pkg, exe = MODE_TO_EXEC[mode]
            # ⚠️ 자식은 `ros2 run` 으로 뜬다 → launch/yaml 파라미터가 안 닿는다.
            #    카메라 관련 설정은 전부 여기서 명시적으로 넘겨야 실행 경로에 반영된다.
            cmd = ["ros2", "run", pkg, exe, "--ros-args",
                   "-p", f"debug_view:={'true' if self.vision_debug_view else 'false'}",
                   "-p", f"image_topic:={self.vision_image_topic}",
                   "-p", f"hfov_deg:={self.vision_hfov_deg}"]
            #self.get_logger().info(f"▶️ {mode} 모드 실행: {' '.join(cmd)}")
            try:
                self._child = subprocess.Popen(cmd, preexec_fn=os.setsid)
                self._current_mode = mode
            except Exception as e:
                #self.get_logger().error(f"❌ 실행 실패: {e}")
                self._child = None
                self._current_mode = None
        else:
            #self.get_logger().warn(f"⚠️ 매핑되지 않은 모드: {mode}")
            self._current_mode = mode

    # ⏹️ 자식 프로세스 종료
    def _stop_child(self):
        if not self._child:
            return
        try:
            #self.get_logger().info("⏹️ 이전 비전 노드 종료 중...")
            os.killpg(os.getpgid(self._child.pid), signal.SIGINT)
            time.sleep(0.5)
            if self._child.poll() is None:
                os.killpg(os.getpgid(self._child.pid), signal.SIGTERM)
                time.sleep(0.5)
            if self._child.poll() is None:
                os.killpg(os.getpgid(self._child.pid), signal.SIGKILL)
        except Exception as e:
            self.get_logger().warn(f"종료 중 오류: {e}")
        finally:
            self._child = None

    # 종료 시 안전 정리
    def destroy_node(self):
        self._stop_child()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SubscriberModeManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
