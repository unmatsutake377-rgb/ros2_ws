#!/usr/bin/env python3
"""cam_stream_ros.py — /camera/camera/color/image_raw 를 H.264/RTP/UDP 로 쏜다.

왜:
  자율 모드에서는 realsense2_camera 노드가 D455 를 잡고 있어 v4l2 로 직접 못 연다.
  그럴 땐 토픽을 받아 GStreamer appsrc 에 밀어 넣는다. (수동 모드는 cam_stream.sh v4l2 가 더 싸다.)

  이 파일은 **ROS 패키지가 아니다** — tools/ 의 독립 스크립트다.
  ssf_tools 에 넣지 않은 이유: python3-gi 의존을 패키지에 추가하면 빌드 위험이 는다
  (CLAUDE.md 3-9 말미: 대회 전 패키지 의존 안 늘림). 구독 전용(발행 0)이라 안전하다.

사용:
  python3 tools/cam_stream_ros.py --host <물가IP> [--port 5600] [--kbps 1500] [--fps 30]
  (보통은 cam_stream.sh <IP> ros 로 호출된다)

⚠️ 한 대에서 하나만 띄울 것. 두 개 띄우면 같은 포트로 두 스트림이 섞인다.
"""
import argparse
import sys
import time

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402

ENCODING_TO_GST = {"rgb8": "RGB", "bgr8": "BGR", "mono8": "GRAY8"}


class CamStream(Node):
    def __init__(self, host, port, kbps, fps, topic):
        super().__init__("cam_stream")
        Gst.init(None)
        self.fps = fps
        self.host, self.port, self.kbps = host, port, kbps
        self.pipeline = None
        self.appsrc = None
        self.caps_str = None
        self.n_frames = 0
        self.t_last_log = time.monotonic()
        # 센서 토픽 관례: BEST_EFFORT + depth 1 (묵은 프레임을 쌓아두지 않는다)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, topic, self.cb, qos)
        self.get_logger().info(f"구독 {topic} → udp://{host}:{port} {kbps}kbps")

    def _build(self, w, h, fmt):
        self.caps_str = f"video/x-raw,format={fmt},width={w},height={h},framerate={self.fps}/1"
        desc = (
            f"appsrc name=src is-live=true do-timestamp=true format=time caps={self.caps_str} "
            f"! videoconvert "
            f"! clockoverlay time-format=\"%H:%M:%S\" halignment=left valignment=top font-desc=\"Sans 18\" "
            f"! videoconvert ! video/x-raw,format=I420 "
            f"! x264enc tune=zerolatency speed-preset=ultrafast bitrate={self.kbps} "
            f"key-int-max={max(1, self.fps // 2)} bframes=0 "  # 0.5초 키프레임(벤치 근거: 설계문서 §7-B)
            f"! rtph264pay config-interval=1 pt=96 mtu=1200 "
            f"! udpsink host={self.host} port={self.port} sync=false async=false"
        )
        self.pipeline = Gst.parse_launch(desc)
        self.appsrc = self.pipeline.get_by_name("src")
        self.pipeline.set_state(Gst.State.PLAYING)
        self.get_logger().info(f"파이프라인 시작 {w}x{h} {fmt}")

    def cb(self, msg: Image):
        fmt = ENCODING_TO_GST.get(msg.encoding)
        if fmt is None:
            self.get_logger().warn(f"미지원 인코딩 {msg.encoding} — 프레임 버림", throttle_duration_sec=5.0)
            return
        if self.pipeline is None:
            self._build(msg.width, msg.height, fmt)
        buf = Gst.Buffer.new_wrapped(bytes(msg.data))
        ret = self.appsrc.emit("push-buffer", buf)
        if ret != Gst.FlowReturn.OK:
            self.get_logger().warn(f"push-buffer {ret}", throttle_duration_sec=5.0)
        self.n_frames += 1
        now = time.monotonic()
        if now - self.t_last_log >= 10.0:
            self.get_logger().info(f"송신 중 {self.n_frames / (now - self.t_last_log):.1f} fps")
            self.n_frames, self.t_last_log = 0, now

    def destroy_node(self):
        if self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)
        super().destroy_node()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", required=True)
    ap.add_argument("--port", type=int, default=5600)
    ap.add_argument("--kbps", type=int, default=1500)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--topic", default="/camera/camera/color/image_raw")
    a = ap.parse_args()
    rclpy.init()
    node = CamStream(a.host, a.port, a.kbps, a.fps, a.topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
