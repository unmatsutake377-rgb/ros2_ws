#!/usr/bin/env python3
"""cam_bench_rx.py — 영상 링크 벤치 수신. 프레임마다 바코드를 읽어 지연·fps·유실을 집계한다.

사용:
  python3 tools/cam_bench_rx.py [--port 5600] [--secs 30] [--jitter 30] [--show]
  --show : 화면도 같이 띄움 (평소 벤치는 끄는 게 숫자가 깨끗하다)

출력(5초마다 + 종료 시 요약):
  fps        : 실제 화면에 도달한 프레임/초
  latency    : 송신 바코드 시각 → 수신 디코드 완료 시각. p50 / p90 / max (ms)
  bad        : 바코드를 못 읽은 프레임(심한 깨짐 = 패킷 유실 후 복구 중)
  gap>200ms  : 프레임 간격이 200ms 넘게 벌어진 횟수 = 조종자가 "멈췄다"고 느낄 구간

⚠️ 지연 값은 양쪽 시계가 맞을 때만 뜻이 있다. 루프백이면 그냥 맞다. 두 기계면 NTP 동기 확인:
   Ubuntu: chronyc tracking / timedatectl   ·  Mac: 시스템 설정 → 날짜와 시간 자동
"""
import argparse
import statistics
import sys
import time

import gi
gi.require_version("Gst", "1.0")
gi.require_version("GstApp", "1.0")
from gi.repository import Gst, GstApp, GLib  # noqa: E402

NBITS, BLOCK_W, BLOCK_H = 24, 24, 24


def read_barcode(y_plane, w):
    """상단 바코드를 읽는다. 각 블록 중앙 8x8 평균으로 판정. 실패 시 None."""
    x0 = (w - NBITS * BLOCK_W) // 2
    val = 0
    rows = range(BLOCK_H // 2 - 4, BLOCK_H // 2 + 4)
    for b in range(NBITS):
        cx = x0 + b * BLOCK_W + BLOCK_W // 2
        acc = 0
        for r in rows:
            base = r * w
            acc += sum(y_plane[base + cx - 4: base + cx + 4])
        mean = acc / 64.0
        if 70 < mean < 180:          # 회색 = 못 믿는다
            return None
        val = (val << 1) | (1 if mean >= 128 else 0)
    return val


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5600)
    ap.add_argument("--secs", type=int, default=0)
    ap.add_argument("--jitter", type=int, default=30)
    ap.add_argument("--show", action="store_true")
    a = ap.parse_args()
    Gst.init(None)

    tail = "! tee name=t t. ! queue ! autovideosink sync=false t. ! queue " if a.show else ""
    p = Gst.parse_launch(
        f"udpsrc port={a.port} caps=application/x-rtp,media=video,encoding-name=H264,payload=96 "
        f"! rtpjitterbuffer latency={a.jitter} drop-on-latency=true "
        f"! rtph264depay ! h264parse ! avdec_h264 max-threads=2 ! videoconvert "
        f"{tail}! video/x-raw,format=I420 ! appsink name=sink emit-signals=true max-buffers=2 drop=true sync=false")
    sink = p.get_by_name("sink")

    S = {"n": 0, "bad": 0, "gaps": 0, "lat": [], "last_t": None, "t0": None, "win": []}

    def on_sample(s):
        sample = s.emit("pull-sample")
        buf = sample.get_buffer()
        caps = sample.get_caps().get_structure(0)
        w, h = caps.get_value("width"), caps.get_value("height")
        ok, mi = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.OK
        y = mi.data[: w * h]
        v = read_barcode(y, w)
        buf.unmap(mi)
        now = time.monotonic()
        if S["t0"] is None:
            S["t0"] = now
        if S["last_t"] is not None and now - S["last_t"] > 0.2:
            S["gaps"] += 1
        S["last_t"] = now
        S["n"] += 1
        if v is None:
            S["bad"] += 1
        else:
            now_ms = int(time.time() * 1000) & ((1 << NBITS) - 1)
            lat = (now_ms - v) & ((1 << NBITS) - 1)
            if lat < 10000:            # 10초 넘으면 시계 불일치로 보고 버림
                S["lat"].append(lat)
                S["win"].append(lat)
        return Gst.FlowReturn.OK

    sink.connect("new-sample", on_sample)
    p.set_state(Gst.State.PLAYING)
    print(f"[rx] UDP {a.port} 대기 (지터버퍼 {a.jitter}ms)", flush=True)
    loop = GLib.MainLoop()
    last = {"n": 0, "t": time.monotonic()}

    def pct(xs, q):
        if not xs:
            return float("nan")
        xs = sorted(xs)
        return xs[min(len(xs) - 1, int(q * len(xs)))]

    def report():
        now = time.monotonic()
        dn = S["n"] - last["n"]
        fps = dn / (now - last["t"])
        last["n"], last["t"] = S["n"], now
        wn = S["win"]
        if wn:
            print(f"[rx] {fps:5.1f} fps | latency p50 {pct(wn, .5):.0f}  p90 {pct(wn, .9):.0f}  "
                  f"max {max(wn):.0f} ms | bad {S['bad']} | gap>200ms {S['gaps']}", flush=True)
        else:
            print(f"[rx] {fps:5.1f} fps | (바코드 없음 — 송신 안 옴?) | bad {S['bad']}", flush=True)
        S["win"] = []
        return True

    GLib.timeout_add(5000, report)
    if a.secs > 0:
        GLib.timeout_add(a.secs * 1000, lambda: (loop.quit(), False)[1])
    try:
        loop.run()
    except KeyboardInterrupt:
        pass
    p.set_state(Gst.State.NULL)
    L = S["lat"]
    el = (S["last_t"] - S["t0"]) if S["t0"] and S["last_t"] else 0   # 첫~마지막 프레임 구간
    print("\n=== 요약 ===")
    print(f"프레임 {S['n']}  평균 {S['n'] / el if el else 0:.1f} fps  깨진 프레임 {S['bad']}  200ms 넘는 공백 {S['gaps']}회")
    if L:
        print(f"지연 ms: p50 {pct(L, .5):.0f}  p90 {pct(L, .9):.0f}  p99 {pct(L, .99):.0f}  "
              f"max {max(L)}  평균 {statistics.mean(L):.0f}  (표본 {len(L)})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
