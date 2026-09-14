#!/usr/bin/env python3
"""cam_bench_netsim.py — 무선 링크 흉내 (UDP 프록시). 지연·지터·유실을 넣어 되쏜다.

왜:
  배가 없어도, 물가에 안 나가도 "5GHz 100m" · "LTE" 같은 링크에서 영상이 어떻게 되는지
  미리 본다. 커널 netem(tc) 없이 파이썬만으로 돈다 — Mac/우분투/컨테이너 공통.

사용:
  python3 tools/cam_bench_netsim.py --in 5601 --out 127.0.0.1:5600 --delay 20 --jitter 15 --loss 2
  → tx 는 5601 로 쏘고, rx 는 5600 에서 받는다.

프리셋(대략치, ⚠️ 실측 대체 아님):
  5GHz 근거리     --delay 5   --jitter 3  --loss 0.2
  5GHz 원거리100m --delay 20  --jitter 15 --loss 2
  LTE+Tailscale   --delay 60  --jitter 25 --loss 1
  LTE 나쁨        --delay 120 --jitter 60 --loss 5
"""
import argparse
import heapq
import random
import select
import socket
import sys
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="inp", type=int, default=5601)
    ap.add_argument("--out", default="127.0.0.1:5600")
    ap.add_argument("--delay", type=float, default=0, help="편도 지연 ms")
    ap.add_argument("--jitter", type=float, default=0, help="지터 ms (±균등)")
    ap.add_argument("--loss", type=float, default=0, help="유실 %")
    ap.add_argument("--reorder", action="store_true", help="지터로 순서 뒤집힘 허용(기본: 순서 유지)")
    a = ap.parse_args()
    host, port = a.out.split(":")
    dst = (host, int(port))

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("0.0.0.0", a.inp))
    rx.setblocking(False)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    q = []               # (send_at, seq, data)
    seq = 0
    last_due = 0.0
    n_in = n_drop = 0
    t_rep = time.monotonic()
    print(f"[netsim] :{a.inp} → {a.out}  delay {a.delay}±{a.jitter}ms  loss {a.loss}%", flush=True)
    while True:
        timeout = max(0.0, q[0][0] - time.monotonic()) if q else 0.05
        r, _, _ = select.select([rx], [], [], min(timeout, 0.05))
        if r:
            while True:
                try:
                    data, _ = rx.recvfrom(65535)
                except BlockingIOError:
                    break
                n_in += 1
                if a.loss > 0 and random.random() * 100 < a.loss:
                    n_drop += 1
                    continue
                d = (a.delay + random.uniform(-a.jitter, a.jitter)) / 1000.0
                due = time.monotonic() + max(0.0, d)
                if not a.reorder:
                    due = max(due, last_due)   # 실제 무선처럼 순서는 유지, 간격만 흔들림
                last_due = due
                heapq.heappush(q, (due, seq, data))
                seq += 1
        now = time.monotonic()
        while q and q[0][0] <= now:
            _, _, data = heapq.heappop(q)
            tx.sendto(data, dst)
        if now - t_rep >= 5.0:
            print(f"[netsim] 패킷 {n_in}  유실 {n_drop}  대기중 {len(q)}", flush=True)
            t_rep = now


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        pass
