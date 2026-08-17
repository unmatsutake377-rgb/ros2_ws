#!/usr/bin/env python3
"""rc_watch — RC 채널이 **어느 핀에 꽂혀 있는지** 조작으로 확정한다.

    python3 tools/rc_watch.py --seq        # 순서대로 하나씩 움직이며 확정 (권장)
    python3 tools/rc_watch.py              # 실시간 막대 표시

전제: `PIN_HUNT=1` 로 빌드된 펌웨어가 올라가 있을 것.
      (`arduino-cli compile --build-property compiler.cpp.extra_flags=-DPIN_HUNT=1`)

왜 만들었나 (2026-08-17)
  핀 번호를 **눈으로 따라가는 것**은 사진으로도 배선도로도 안 됐다. 기판의 점퍼는
  보드 밑을 지나가고, 문서의 핀표는 실물과 이미 어긋나 있었다.
  확실한 것은 하나뿐이다 — **스틱을 움직였을 때 값이 변하는 핀이 그 채널이다.**

🚨 값이 잡히는 것과 **펌웨어가 읽을 수 있는 것은 다르다.**
   이 도구는 `pulseIn` 폴링이라 아무 핀이나 읽는다. 그런데 실제 펌웨어는 인터럽트로 읽고,
   Mega 에서 외부 인터럽트가 되는 핀은 **2, 3, 18, 19, 20, 21 뿐**이다.
   여기서 잡혔다고 "그 핀 그대로 쓰면 된다" 가 아니다 — 아래 표시에서 ✅/🚨 로 구분한다.
"""

import argparse
import re
import sys
import time

import serial

PORT = "/dev/ttyMEGA"
INTERRUPT_PINS = {2, 3, 18, 19, 20, 21}      # Mega 2560 외부 인터럽트 — 이게 전부다
LINE = re.compile(r"p(\d+)=(\d+)us")

MOVE_THRESHOLD = 60      # us. 이보다 크게 흔들리면 '움직였다'. 정지 잡음은 ±7us 수준
QUIET_GAP = 1.2          # s. 이만큼 아무 핀도 안 움직이면 한 동작이 끝난 것으로 본다


def read_lines(ser, seconds):
    """(시각, {핀: us}) 스트림."""
    buf = b""
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        buf += ser.read(256)
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            s = raw.decode("utf-8", "replace")
            if not s.startswith("HUNT"):
                continue
            vals = {int(p): int(w) for p, w in LINE.findall(s)}
            if vals:
                yield time.monotonic() - t0, vals


def tag(pin):
    return "✅ 인터럽트 가능" if pin in INTERRUPT_PINS else "🚨 인터럽트 불가 — 옮겨야 함"


def bar(w, lo=1000, hi=2000, width=30):
    frac = max(0.0, min(1.0, (w - lo) / (hi - lo)))
    i = int(frac * (width - 1))
    return "[" + "·" * i + "●" + "·" * (width - 1 - i) + "]"


def live(ser, seconds):
    print(f"실시간 표시 {seconds}초 — 스틱/스위치를 움직여 보십시오.\n")
    seen = {}
    for _t, vals in read_lines(ser, seconds):
        for p, w in vals.items():
            seen.setdefault(p, []).append(w)
        row = "  ".join(f"p{p}={vals[p]:4d}{bar(vals[p])}" for p in sorted(vals))
        print("\r" + row, end="", flush=True)
    print("\n")
    return seen


def sequence(ser, per_step, steps):
    """한 번에 하나씩 움직이게 하고, 움직인 핀을 그 동작에 귀속시킨다."""
    print("=" * 68)
    print("  한 번에 **하나만** 움직이십시오. 각 동작 사이에 2초 쉬면 자동으로 끊깁니다.")
    print("=" * 68)
    for i, s in enumerate(steps, 1):
        print(f"   {i}. {s}")
    total = per_step * len(steps) + 4
    print(f"\n  {total}초 동안 기록합니다. 지금부터 순서대로 하십시오.\n")

    base = {}
    events = []          # (시각, 핀, 최소, 최대)
    active = {}          # 핀 -> [최소, 최대, 마지막 움직인 시각]
    last_move = None

    for t, vals in read_lines(ser, total):
        for p, w in vals.items():
            if p not in base:
                base[p] = w
                continue
            if abs(w - base[p]) >= MOVE_THRESHOLD:
                a = active.setdefault(p, [w, w, t])
                a[0] = min(a[0], w)
                a[1] = max(a[1], w)
                a[2] = t
                last_move = t
                base[p] = w          # 새 위치를 기준으로 삼는다(천천히 움직여도 따라감)
        # 조용해지면 한 동작 종료
        if active and last_move is not None and t - last_move > QUIET_GAP:
            for p, a in sorted(active.items()):
                events.append((a[2], p, a[0], a[1]))
            active.clear()
            print(f"   · {len(events)}번째 동작 감지 — 핀 "
                  f"{', '.join(str(e[1]) for e in events[-1:])}")
    for p, a in sorted(active.items()):
        events.append((a[2], p, a[0], a[1]))

    print("\n" + "=" * 68)
    print("  결과")
    print("=" * 68)
    if not events:
        print("  🚨 움직임이 하나도 안 잡혔습니다.")
        print("     · 조종기가 켜져 있습니까? (FS-i6 는 스위치 전부 위 + 스로틀 최하 여야 송신)")
        print("     · 수신기 LED 가 켜져 있습니까?")
        return
    for i, (t, p, lo, hi) in enumerate(events, 1):
        what = steps[i - 1] if i <= len(steps) else "(순서 밖)"
        print(f"  {i}. {what}")
        print(f"       -> 핀 {p}   {lo}~{hi}us (폭 {hi-lo})   {tag(p)}")
    moved = {e[1] for e in events}
    bad = sorted(moved - INTERRUPT_PINS)
    if bad:
        print(f"\n  🚨 인터럽트 안 되는 핀에 있는 채널: {bad}")
        print(f"     남은 인터럽트 핀: {sorted(INTERRUPT_PINS - moved)}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=PORT)
    ap.add_argument("--seq", action="store_true", help="순서대로 하나씩 움직여 확정")
    ap.add_argument("--sec", type=float, default=8.0, help="동작당 초 (--seq) / 총 초")
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, 115200, timeout=0.05)
    except Exception as e:                       # noqa: BLE001
        print(f"🚨 {args.port} 열기 실패: {e}")
        return 1
    time.sleep(2.5)                              # 부트로더 — 열면 보드가 리셋된다
    ser.reset_input_buffer()

    probe = next(read_lines(ser, 12.0), None)    # 첫 전체 훑기가 2.7초씩 걸린다
    if probe is None:
        print("🚨 HUNT 줄이 안 나옵니다 — PIN_HUNT 빌드가 아닙니다.")
        ser.close()
        return 1

    try:
        if args.seq:
            sequence(ser, args.sec, [
                "스로틀 스틱을 위아래로 (전후진)",
                "조향 스틱을 좌우로",
                "모드 스위치 SWA 를 껐다 켜기",
            ])
        else:
            live(ser, args.sec)
    finally:
        ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
