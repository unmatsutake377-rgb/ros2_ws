#!/usr/bin/env python3
"""cam_bench_tx.py — 영상 링크 벤치 송신. 실카메라 없이(또는 있어도) 지연·fps 를 **숫자로** 잰다.

왜:
  cam_stream.sh 의 시계 각인은 사람 눈으로 "멈췄나" 보는 용도다. 벤치에는 숫자가 필요하다.
  이 스크립트는 프레임마다 **송신 시각(ms)을 상단 바코드(흑백 블록 24개)로 그려 넣는다.**
  H.264 압축을 거쳐도 32px 블록은 안 깨진다. 수신측(cam_bench_rx.py)이 블록을 읽어
  (수신시각 − 송신시각) = 링크 지연을 프레임마다 계산한다.

  인코더·RTP·UDP 체인은 cam_stream.sh 와 **완전히 같다.** 여기서 나온 수치가 곧 본 스트림의 수치다.

사용:
  python3 tools/cam_bench_tx.py --host 127.0.0.1                 # 가상 소스(공 패턴)로 루프백
  python3 tools/cam_bench_tx.py --host 100.x.y.z --source v4l2   # 실카메라 위에 바코드 덧그림
  옵션: --port 5600 --w 640 --h 480 --fps 30 --kbps 1500 --dev /dev/video4 --secs 30 --keyint 15

시계:
  같은 기계(루프백)면 그냥 맞다. 두 기계면 **양쪽 NTP(chrony/timed) 동기가 전제**다 — 보통 ±5ms 이내.
  그래도 못 믿겠으면 폰 스톱워치 촬영법(설계문서 V2)으로 한 번 교차 확인.
"""
import argparse
import sys
import time

import gi
gi.require_version("Gst", "1.0")
gi.require_version("GstApp", "1.0")
from gi.repository import Gst, GstApp, GLib  # noqa: E402

NBITS = 24            # ms 단위 24비트 = 약 4.6시간 주기. 하루 안 벤치엔 충분
BLOCK_W = 24          # 블록 폭 px (640 폭에 24블록 = 576px). 압축 내성 확보
BLOCK_H = 24


def draw_barcode(buf_bytes, w, h, value):
    """I420 프레임의 Y 평면 상단에 바코드를 그린다. (U/V 는 회색 그대로 = 흑백)"""
    y = bytearray(buf_bytes)
    x0 = (w - NBITS * BLOCK_W) // 2
    for row in range(BLOCK_H):
        base = row * w
        for b in range(NBITS):
            bit = (value >> (NBITS - 1 - b)) & 1
            v = 235 if bit else 16
            s = base + x0 + b * BLOCK_W
            y[s:s + BLOCK_W] = bytes([v]) * BLOCK_W
    # 위아래 검은 테두리 두 줄 — 수신측 위치 찾기용 기준선
    for row in (BLOCK_H, BLOCK_H + 1):
        base = row * w
        y[base + x0: base + x0 + NBITS * BLOCK_W] = bytes([16]) * (NBITS * BLOCK_W)
    return bytes(y)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", required=True)
    ap.add_argument("--port", type=int, default=5600)
    ap.add_argument("--w", type=int, default=640)
    ap.add_argument("--h", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--kbps", type=int, default=1500)
    ap.add_argument("--source", choices=["test", "v4l2"], default="test")
    ap.add_argument("--dev", default="/dev/video4")
    ap.add_argument("--secs", type=int, default=0, help="0=무한")
    ap.add_argument("--keyint", type=int, default=0, help="키프레임 간격(프레임). 0=fps/2(0.5초, cam_stream.sh 와 동일)")
    a = ap.parse_args()

    Gst.init(None)
    w, h = a.w, a.h
    if a.source == "test":
        src = f"videotestsrc is-live=true pattern=ball ! video/x-raw,width={w},height={h},framerate={a.fps}/1"
    else:
        src = (f"v4l2src device={a.dev} io-mode=2 do-timestamp=true "
               f"! video/x-raw,format=YUY2,width={w},height={h},framerate={a.fps}/1")
    # 소스 → I420 → appsink(바코드 그림) → appsrc → [cam_stream.sh 와 동일한 인코더 체인]
    p_in = Gst.parse_launch(
        f"{src} ! videoconvert ! video/x-raw,format=I420 "
        f"! appsink name=sink emit-signals=true max-buffers=1 drop=true sync=false")
    p_out = Gst.parse_launch(
        f"appsrc name=src is-live=true do-timestamp=true format=time "
        f"caps=video/x-raw,format=I420,width={w},height={h},framerate={a.fps}/1 "
        f"! x264enc tune=zerolatency speed-preset=ultrafast bitrate={a.kbps} key-int-max={a.keyint or max(1, a.fps // 2)} bframes=0 "
        f"! rtph264pay config-interval=1 pt=96 mtu=1200 "
        f"! udpsink host={a.host} port={a.port} sync=false async=false")
    sink = p_in.get_by_name("sink")
    asrc = p_out.get_by_name("src")
    stats = {"n": 0, "t0": time.monotonic()}

    def on_sample(s):
        sample = s.emit("pull-sample")
        buf = sample.get_buffer()
        ok, mi = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.OK
        data = bytes(mi.data)
        buf.unmap(mi)
        ts_ms = int(time.time() * 1000) & ((1 << NBITS) - 1)   # 벽시계(NTP 동기 전제)
        out = draw_barcode(data, w, h, ts_ms)
        nb = Gst.Buffer.new_wrapped(out)
        asrc.emit("push-buffer", nb)
        stats["n"] += 1
        return Gst.FlowReturn.OK

    sink.connect("new-sample", on_sample)
    p_out.set_state(Gst.State.PLAYING)
    p_in.set_state(Gst.State.PLAYING)
    print(f"[tx] → udp://{a.host}:{a.port}  {w}x{h}@{a.fps} {a.kbps}kbps  source={a.source}", flush=True)

    loop = GLib.MainLoop()

    def report():
        n, t0 = stats["n"], stats["t0"]
        el = time.monotonic() - t0
        print(f"[tx] 송신 {n / el:.1f} fps (누적 {n})", flush=True)
        return True
    GLib.timeout_add(5000, report)
    if a.secs > 0:
        GLib.timeout_add(a.secs * 1000, lambda: (loop.quit(), False)[1])
    try:
        loop.run()
    except KeyboardInterrupt:
        pass
    p_in.set_state(Gst.State.NULL)
    p_out.set_state(Gst.State.NULL)
    return 0


if __name__ == "__main__":
    sys.exit(main())
