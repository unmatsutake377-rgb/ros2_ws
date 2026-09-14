#!/usr/bin/env python3
"""cam_stream_oak.py — OAK-1 W PoE 의 **온보드 H.264 인코더**로 영상을 쏜다 (노트북 CPU 0).

왜:
  OAK 는 USB/v4l2 장치가 아니라 **이더넷(PoE) 장치**다 → cam_stream.sh 의 v4l2 모드가 안 된다.
  OAK 는 카메라 안(Myriad X)에 H.264 인코더가 있어, 노트북은 압축된 바이트를 받아 RTP 로 싸서
  보내기만 하면 된다. x264enc 가 필요 없다.

⚠️ 대가: **배 시각 각인(clockoverlay)을 못 한다.** 영상이 카메라에서 이미 압축돼 오기 때문이다.
  프리즈 판별은 수신측 우하단 시계만으로는 안 된다(마지막 프레임이 그대로 남음).
  → 시각 각인이 필요하면 `cam_stream.sh <IP> ros` (depthai-ros 토픽 → 노트북 인코딩) 를 쓴다.
  이 스크립트는 **노트북 CPU 가 모자랄 때의 대안**이다. 기본은 ros 모드.

⚠️ **미검증 (2026-09-14).** OAK 실물이 없어 API 호출 순서만 depthai v2 문서 기준으로 썼다.
  도착일 `docs/oak_arrival_runbook.md` 절차에 이 스크립트 시험을 한 줄 추가할 것.

사용:
  python3 tools/cam_stream_oak.py --host <물가IP> [--port 5600] [--kbps 1500] [--fps 30] [--ip <카메라고정IP>]
의존:
  pip install depthai   (depthai-ros 를 apt 로 깔았으면 이미 있을 수 있음: python3 -c "import depthai")
  + gstreamer python3-gi (cam_stream.sh 헤더와 동일)
"""
import argparse
import sys
import time

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402

try:
    import depthai as dai
except ImportError:
    print("❌ depthai 가 없다: pip install depthai  (또는 ros-humble-depthai-ros)", file=sys.stderr)
    sys.exit(1)


def build_oak(fps, kbps, w, h):
    p = dai.Pipeline()
    cam = p.create(dai.node.ColorCamera)
    cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam.setFps(fps)
    cam.setVideoSize(w, h)            # 인코더 입력 크기 (센서 해상도 이하)
    enc = p.create(dai.node.VideoEncoder)
    enc.setDefaultProfilePreset(fps, dai.VideoEncoderProperties.Profile.H264_MAIN)
    enc.setBitrateKbps(kbps)
    enc.setKeyframeFrequency(max(1, fps // 2))   # 0.5초 키프레임 — cam_stream.sh 와 동일 근거(설계문서 §7-B)
    enc.setNumBFrames(0)                          # B프레임 0 = 지연 최소
    cam.video.link(enc.input)
    xout = p.create(dai.node.XLinkOut)
    xout.setStreamName("h264")
    enc.bitstream.link(xout.input)
    return p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", required=True)
    ap.add_argument("--port", type=int, default=5600)
    ap.add_argument("--kbps", type=int, default=1500)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--w", type=int, default=640)
    ap.add_argument("--h", type=int, default=480)
    ap.add_argument("--ip", default="", help="카메라 고정 IP (비우면 자동 탐색)")
    a = ap.parse_args()

    Gst.init(None)
    gp = Gst.parse_launch(
        f"appsrc name=src is-live=true do-timestamp=true format=time "
        f"caps=video/x-h264,stream-format=byte-stream,alignment=au,width={a.w},height={a.h},framerate={a.fps}/1 "
        f"! h264parse config-interval=1 ! rtph264pay pt=96 mtu=1200 "
        f"! udpsink host={a.host} port={a.port} sync=false async=false")
    src = gp.get_by_name("src")
    gp.set_state(Gst.State.PLAYING)

    pipeline = build_oak(a.fps, a.kbps, a.w, a.h)
    dev_args = (dai.DeviceInfo(a.ip),) if a.ip else ()
    print(f"[oak] → udp://{a.host}:{a.port}  {a.w}x{a.h}@{a.fps} {a.kbps}kbps  카메라={a.ip or '자동탐색'}", flush=True)
    n, t0 = 0, time.monotonic()
    with dai.Device(pipeline, *dev_args) as device:
        q = device.getOutputQueue(name="h264", maxSize=4, blocking=False)
        while True:
            pkt = q.get()                      # dai.ImgFrame (인코딩된 비트스트림)
            data = bytes(pkt.getData())
            buf = Gst.Buffer.new_wrapped(data)
            if src.emit("push-buffer", buf) != Gst.FlowReturn.OK:
                print("[oak] push-buffer 실패", file=sys.stderr, flush=True)
            n += 1
            now = time.monotonic()
            if now - t0 >= 10.0:
                print(f"[oak] 송신 {n / (now - t0):.1f} fps", flush=True)
                n, t0 = 0, now


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        pass
