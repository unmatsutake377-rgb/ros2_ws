#!/usr/bin/env python3
"""geofence 소비자 규약 고정 테스트 — _merge_geofence / _geofence_blocks (동작 변경 없음).
실행: python3 src/ship_direction/test/test_geofence_merge.py

계약서 docs/문제와작업/계약서_geofence부호_20261104.md §8-②.
규약: **배열 인덱스 0 (rel −80) = 우현**, 끝 인덱스 (+80) = 좌현 — LiDAR a−80·/candidate_angle 과 같은
좌 양수. north 가 이 규약으로 발행하고(geofence_rays.py), 여기 소비자는 부호 변환을 **하지 않는다**.
🚨 누군가 소비자 쪽에서도 부호를 뒤집으면(계약서 §1 (b)) 다시 거울이 된다 — 이 파일이 그걸 잡는다.

ship_direction.py 는 rclpy 를 import 하므로 Mac(ROS 없음)에선 그대로 못 읽는다.
→ test_avoidance_filters.py 와 같은 ROS 스텁 + fake self 로 **실제 메서드**를 언바운드 호출한다.
"""

import math
import os
import sys
import threading
import time
import types

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# ---- ROS 모듈 최소 스텁 (import 통과용, 로직엔 안 쓰임) ----


def _stub(name, **attrs):
    m = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(m, k, v)
    sys.modules[name] = m
    return m


class _Node:                       # class ShipDirection(Node) 상속용
    pass


_pol = types.SimpleNamespace(BEST_EFFORT=1, RELIABLE=2, KEEP_LAST=1)
_stub('rclpy', init=None, ok=lambda: True, shutdown=None)
_stub('rclpy.node', Node=_Node)
_stub('rclpy.executors', MultiThreadedExecutor=object)
_stub('rclpy.qos',
      QoSProfile=lambda **k: types.SimpleNamespace(**k),
      ReliabilityPolicy=_pol, HistoryPolicy=_pol)


def _msg(*a, **k):
    return None


_stub('std_msgs', msg=None)
_stub('std_msgs.msg', Int32=_msg, Float32=_msg, Float32MultiArray=_msg)
_stub('sensor_msgs', msg=None)
_stub('sensor_msgs.msg', LaserScan=_msg)

from ship_direction.ship_direction import ShipDirection, LIDAR_FORWARD_DEG  # noqa: E402

INF = float('inf')
N_RAYS = 161                                   # −80..80 / 1°
I_STARBOARD_END, I_PORT_END = 0, 160           # geofence 배열 인덱스: 0 = 우현(−80), 160 = 좌현(+80)
LIDAR_ANGLES = [float(a) for a in range(161)]  # LiDAR 프레임 0..160 (80 = 정면)

_p = _t = 0


def check(name, fn):
    global _p, _t
    _t += 1
    try:
        fn()
        _p += 1
        print(f"  ✅ {name}")
    except AssertionError as e:
        print(f"  ❌ {name}\n     {e}")


class _Logger:
    def __init__(self):
        self.lines = []

    def warn(self, msg, **kw):
        self.lines.append(msg)

    info = error = warn


def fake_self(gf, age_sec=0.0):
    """_merge_geofence / _geofence_blocks 가 읽는 것만 채운 가짜 self. 파라미터는 코드 기본값(=대회값)."""
    log = _Logger()
    return types.SimpleNamespace(
        lock=threading.Lock(),
        geofence=gf,
        last_geofence_t=time.monotonic() - age_sec,
        geofence_stale_sec=2.0,
        geofence_stop_margin_m=2.0,
        geofence_stop_cone_deg=60.0,
        get_logger=lambda: log,
    )


def gf_wall(idx, dist):
    """인덱스 idx 한 칸만 dist, 나머지 inf 인 geofence 튜플 (a0=−80, inc=1)."""
    gr = [INF] * N_RAYS
    gr[idx] = dist
    return (-80.0, 1.0, gr)


# ---------------- _merge_geofence
def test_merge_index0_lands_on_lidar_0_starboard():
    """gf 인덱스 0(rel −80 = 우현) 3.0 m → LiDAR 0°(= 80−80 = 우현 끝) 셀에 3.0. LiDAR 160°(좌현 끝)는 inf."""
    fs = fake_self(gf_wall(I_STARBOARD_END, 3.0))
    out = ShipDirection._merge_geofence(fs, [INF] * 161, LIDAR_ANGLES)
    assert out[0] == 3.0, f"out[0]={out[0]} — LiDAR 0°(우현)에 병합돼야"
    assert out[160] == INF, f"out[160]={out[160]} — 좌현에 '없는 벽'이 생겼다: 소비자 쪽에서 부호를 뒤집었다"
    assert sum(1 for v in out if math.isfinite(v)) == 1, "한 칸만 병합돼야"
    assert int(LIDAR_ANGLES[0] - LIDAR_FORWARD_DEG) == -80       # 규약 재확인


def test_merge_last_index_lands_on_lidar_160_port():
    """gf 인덱스 160(rel +80 = 좌현) → LiDAR 160° 셀."""
    fs = fake_self(gf_wall(I_PORT_END, 2.0))
    out = ShipDirection._merge_geofence(fs, [INF] * 161, LIDAR_ANGLES)
    assert out[160] == 2.0 and out[0] == INF, (out[0], out[160])


def test_merge_stale_leaves_scan_unchanged():
    """3 s 전 geofence(stale 2.0 s 초과) → 병합 안 함 (묵은 '없는 벽'은 배를 가둔다)."""
    fs = fake_self(gf_wall(I_STARBOARD_END, 3.0), age_sec=3.0)
    scan = [INF] * 161
    out = ShipDirection._merge_geofence(fs, scan, LIDAR_ANGLES)
    assert out == scan, "stale 인데 병합됐다"


def test_merge_keeps_min_when_real_scan_closer():
    """실제 스캔이 더 가까우면 min 이라 스캔값 유지, 더 멀면 경계값."""
    fs = fake_self(gf_wall(I_STARBOARD_END, 3.0))
    scan = [INF] * 161
    scan[0] = 1.0
    out = ShipDirection._merge_geofence(fs, scan, LIDAR_ANGLES)
    assert out[0] == 1.0, out[0]
    scan[0] = 5.0
    out = ShipDirection._merge_geofence(fs, scan, LIDAR_ANGLES)
    assert out[0] == 3.0, out[0]


def test_merge_none_geofence_unchanged():
    fs = fake_self(None)
    scan = [INF] * 161
    assert ShipDirection._merge_geofence(fs, scan, LIDAR_ANGLES) == scan


# ---------------- _geofence_blocks  (intended_rel = /candidate_angle, 좌 양수, 0~360 래핑)
def test_blocks_starboard_wall_vs_starboard_heading():
    """최근접 1.5 m 가 인덱스 0(우현 80°) → intended_rel=300(= 우 60°) → diff −20 → **True**(정지)."""
    fs = fake_self(gf_wall(I_STARBOARD_END, 1.5))
    assert ShipDirection._geofence_blocks(fs, 300.0) is True


def test_blocks_starboard_wall_vs_port_heading_is_false():
    """같은 우현 벽, intended_rel=60(좌 60°) → diff −140 → False. 좌로 트는데 우현 벽 때문에 서면 거짓 STOP_HOLD(거울 증상)."""
    fs = fake_self(gf_wall(I_STARBOARD_END, 1.5))
    assert ShipDirection._geofence_blocks(fs, 60.0) is False


def test_blocks_starboard_wall_vs_straight_is_false():
    """intended_rel=0(정면) → |−80| > 60 → False."""
    fs = fake_self(gf_wall(I_STARBOARD_END, 1.5))
    assert ShipDirection._geofence_blocks(fs, 0.0) is False


def test_blocks_outside_margin_is_false():
    """2.5 m > stop_margin 2.0 → False (방향 무관)."""
    fs = fake_self(gf_wall(I_STARBOARD_END, 2.5))
    assert ShipDirection._geofence_blocks(fs, 300.0) is False


def test_blocks_port_wall_vs_port_heading():
    """대칭 확인: 인덱스 160(좌현 80°) 1.5 m, intended_rel=60(좌 60°) → diff +20 → True; 300 → False."""
    fs = fake_self(gf_wall(I_PORT_END, 1.5))
    assert ShipDirection._geofence_blocks(fs, 60.0) is True
    assert ShipDirection._geofence_blocks(fs, 300.0) is False


def test_blocks_stale_is_false():
    fs = fake_self(gf_wall(I_STARBOARD_END, 1.5), age_sec=3.0)
    assert ShipDirection._geofence_blocks(fs, 300.0) is False


def main():
    print("=== geofence 소비자 규약 테스트 (계약서_geofence부호_20261104) ===")
    for n, f in sorted(globals().items()):
        if n.startswith("test_") and callable(f):
            check(n[5:], f)
    print(f"\n{_p}/{_t} 통과")
    return 0 if _p == _t else 1


if __name__ == "__main__":
    sys.exit(main())
