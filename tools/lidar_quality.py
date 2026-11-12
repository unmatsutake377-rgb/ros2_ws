#!/usr/bin/env python3
"""LiDAR 데이터 품질 실시간 보기 — 햇빛·장착 각도·스캔 모드 비교용 (읽기 전용, /scan 만 구독).

왜: 드라이버 health 'OK' 는 내부 하드웨어 상태만 본다. 햇빛·수면·앞 숙임으로 반사점이 사라지거나
    튀어도 OK 로 뜬다. 같은 자리에서 조건만 바꿔(해 쪽 ↔ 반대쪽, scan_mode Sensitivity ↔ Stability)
    아래 숫자를 나란히 적으면 원인이 갈린다.

1 초마다 한 줄:
  hz      스캔 주기(실측 약 12 Hz)
  pts     한 바퀴 유효점 / 전체(유효 = 유한하고 range_min 초과)
  F30     정면 ±30° 유효 비율(%)   — 부표·장애물이 보이는 곳
  F90     정면 ±90° 유효 비율(%)
  near    정면 ±30° 최근접 거리(m) — 아무것도 없는데 가까운 값이 튀면 거짓 반사
  iso     고립점 수(앞뒤 이웃이 0.3 m 넘게 다른 유효점) — 햇빛·물보라 잡음의 대리 지표

사용(배 노트북, ROS 환경):  python3 tools/lidar_quality.py   [--front-deg 80]
정면 = LiDAR 원래 각 80°(CLAUDE.md §3 규약). 상대각 = a − 80, 좌 +.
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


def scan_stats(msg, front_deg=80.0):
    """LaserScan 하나 → dict(pts, total, f30, f90, near, iso). 순수 계산(테스트 가능)."""
    n = len(msg.ranges)
    rmin = msg.range_min if msg.range_min > 0 else 0.0
    valid = [math.isfinite(r) and r > rmin for r in msg.ranges]
    f30 = [0, 0]
    f90 = [0, 0]
    near = math.inf
    for i, r in enumerate(msg.ranges):
        a = math.degrees(msg.angle_min + i * msg.angle_increment)
        rel = (a - front_deg + 180.0) % 360.0 - 180.0
        if abs(rel) <= 90.0:
            f90[1] += 1
            f90[0] += valid[i]
            if abs(rel) <= 30.0:
                f30[1] += 1
                f30[0] += valid[i]
                if valid[i]:
                    near = min(near, r)
    iso = 0
    for i in range(1, n - 1):
        if not valid[i]:
            continue
        r = msg.ranges[i]
        left = msg.ranges[i - 1] if valid[i - 1] else math.inf
        right = msg.ranges[i + 1] if valid[i + 1] else math.inf
        if abs(r - left) > 0.3 and abs(r - right) > 0.3:
            iso += 1
    pct = (lambda c: 100.0 * c[0] / c[1] if c[1] else float('nan'))
    return dict(pts=sum(valid), total=n, f30=pct(f30), f90=pct(f90), near=near, iso=iso)


class LidarQuality(Node):
    def __init__(self, front_deg):
        super().__init__('lidar_quality')
        self.front_deg = front_deg
        self.stats = []
        self.t0 = time.monotonic()
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(LaserScan, '/scan', self.cb, qos)
        self.create_timer(1.0, self.report)
        print("  hz | pts/total | F30% | F90% | near m | iso")

    def cb(self, msg):
        self.stats.append(scan_stats(msg, self.front_deg))

    def report(self):
        now = time.monotonic()
        dt = now - self.t0
        self.t0 = now
        s, self.stats = self.stats, []
        if not s:
            print("  /scan 없음 (1 s)")
            return
        avg = (lambda k: sum(x[k] for x in s) / len(s))
        near = min(x['near'] for x in s)
        print(f"{len(s) / dt:4.1f} | {avg('pts'):5.0f}/{s[-1]['total']:<4d} | {avg('f30'):4.0f} | {avg('f90'):4.0f} | "
              f"{near:6.2f} | {avg('iso'):4.0f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--front-deg', type=float, default=80.0)
    a, _ = ap.parse_known_args()
    rclpy.init()
    node = LidarQuality(a.front_deg)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
