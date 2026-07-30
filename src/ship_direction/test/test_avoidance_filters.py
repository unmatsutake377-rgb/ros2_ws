#!/usr/bin/env python3
"""장애물 필터 파이프라인 단위 테스트 (팀원 제안 §5.1).

smooth_spikes → suppress_spike_edges → dilate_obstacles 를 거치며 장애물 셀이
각 단계에서 어떻게 남는지 확인한다. 핵심 질문: **작은 부표가 dilate 전에 지워지나?**

ship_direction.py 는 rclpy 를 import 하므로 Mac(ROS 없음)에선 그대로 못 읽는다.
→ ROS 모듈을 최소 스텁으로 넣고 **실제 필터 메서드**를 언바운드로 호출한다(노드 로직 그대로 테스트).
   실행: python3 src/ship_direction/test/test_avoidance_filters.py
"""

import os
import sys
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
_msg = lambda *a, **k: None
_stub('std_msgs', msg=None)
_stub('std_msgs.msg', Int32=_msg, Float32=_msg, Float32MultiArray=_msg)
_stub('sensor_msgs', msg=None)
_stub('sensor_msgs.msg', LaserScan=_msg)

from ship_direction.ship_direction import ShipDirection  # noqa: E402
from ship_direction.failsafe import TemporalVote          # noqa: E402

_p = _t = 0


def check(name, fn):
    global _p, _t
    _t += 1
    try:
        fn(); _p += 1; print(f"  ✅ {name}")
    except AssertionError as e:
        print(f"  ❌ {name}\n     {e}")


def fake_self():
    """필터 메서드가 읽는 파라미터만 채운 가짜 self (기본값)."""
    return types.SimpleNamespace(
        max_spike_ratio=0.01,
        border_margin=2,
        half_width=0.45,
        clearance=0.25,
        min_obstacle_cells=1,
        temporal=TemporalVote(1, 2),   # OFF(frames=1)
    )


def count_ones(b):
    return sum(1 for x in b if x == 1)


def make_binary(n, start, width):
    """길이 n, [start, start+width) 를 1 로."""
    b = [0] * n
    for i in range(start, start + width):
        b[i] = 1
    return b


def pipeline(fs, binary, distance_array, inc_deg=1.0, det=3.0):
    """실제 노드 필터 순서대로. 각 단계 후 1의 개수 반환."""
    b0 = count_ones(binary)
    b = ShipDirection.smooth_spikes(fs, binary)
    b1 = count_ones(b)
    b = ShipDirection.suppress_spike_edges(fs, b)
    b2 = count_ones(b)
    b = ShipDirection.dilate_obstacles(fs, b, distance_array, inc_deg, det)
    b3 = count_ones(b)
    return b0, b1, b2, b3, b


# ---------------- 실제 부표(넓음) 는 살아남아야 한다 (안전 핵심)
def test_wide_obstacle_survives():
    """회피거리(3m)의 1.6m 부표는 ~30셀 폭 → 파이프라인 통과 후에도 남아야 한다."""
    fs = fake_self()
    n = 200
    binary = make_binary(n, 85, 30)                 # 30셀 장애물
    dist = [3.0 if v == 1 else float('inf') for v in binary]
    for i in range(85, 85 + 30):
        dist[i] = 2.5
    b0, b1, b2, b3, _ = pipeline(fs, binary, dist)
    assert b3 > 0, f"넓은 부표가 사라짐! stages={b0}->{b1}->{b2}->{b3}"
    assert b3 >= 20, f"넓은 부표가 과하게 깎임: {b3} (원래 30)"


# ---------------- 작은 셀 장애물의 운명 (문서화 — 팀원 §5.1 질문의 핵심)
def test_small_obstacles_fate():
    """1·2·3셀 고립 장애물이 각 단계에서 어떻게 되는지 실제 확인·문서화.

    발견: smooth_spikes 는 길이 <= int(n*max_spike_ratio) 인 run 을 지운다(n=200,0.01→2).
      → 1·2셀은 smooth_spikes 에서 제거.
    suppress_spike_edges 는 ±border_margin 안에 0 이 있는 셀을 지운다(고립 소형 침식).
      → 3셀도 여기서 제거될 수 있다.
    즉 min_obstacle_cells=1 로 낮춰도 **앞 필터가 소형을 먼저 지운다.**
    실제 부표는 회피거리에서 넓어(위 테스트) 살아남으므로, 이 소형 제거는 원거리/노이즈 억제에 가깝다.
    """
    fs = fake_self()
    n = 200
    results = {}
    for w in (1, 2, 3):
        binary = make_binary(n, 100, w)
        dist = [2.5 if v == 1 else float('inf') for v in binary]
        b0, b1, b2, b3, _ = pipeline(fs, binary, dist)
        results[w] = (b0, b1, b2, b3)
        print(f"     {w}셀: 시작{b0} → smooth{b1} → suppress{b2} → dilate{b3}")

    # 1·2셀은 smooth_spikes(run<=2 제거)에서 사라진다
    assert results[1][1] == 0, f"1셀이 smooth 후 남음: {results[1]}"
    assert results[2][1] == 0, f"2셀이 smooth 후 남음: {results[2]}"
    # 3셀은 smooth 는 통과(3>2)하나 suppress_spike_edges 에서 침식
    assert results[3][1] == 3, f"3셀이 smooth 에서 바뀜: {results[3]}"


# ---------------- smooth_spikes 임계는 배열 길이에 비례
def test_smooth_threshold_scales_with_length():
    """max_spike_ratio=0.01: n=200 이면 run<=2 제거, n=600 이면 run<=6 제거."""
    fs = fake_self()
    # n=600 에서 5셀 장애물 → int(600*0.01)=6 >=5 → smooth 가 지운다
    binary = make_binary(600, 300, 5)
    b = ShipDirection.smooth_spikes(fs, binary)
    assert count_ones(b) == 0, "n=600 에서 5셀이 smooth 를 통과하면 안 됨(임계6)"


# ---------------- dilate 는 살아남은 장애물을 배 폭만큼 넓힌다
def test_dilate_widens_survivor():
    """suppress 를 통과한 넓은 장애물은 dilate 로 양옆이 더 넓어진다."""
    fs = fake_self()
    n = 200
    binary = make_binary(n, 90, 20)
    dist = [2.0 if v == 1 else float('inf') for v in binary]
    b = ShipDirection.suppress_spike_edges(fs, binary)
    after_suppress = count_ones(b)
    b = ShipDirection.dilate_obstacles(fs, b, dist, 1.0, 3.0)
    after_dilate = count_ones(b)
    assert after_dilate > after_suppress, (
        f"dilate 가 안 넓힘: {after_suppress} → {after_dilate}")


def main():
    print("=== 장애물 필터 파이프라인 테스트 (§5.1) ===")
    for n, f in sorted(globals().items()):
        if n.startswith("test_") and callable(f):
            check(n[5:], f)
    print(f"\n{_p}/{_t} 통과")
    return 0 if _p == _t else 1


if __name__ == "__main__":
    sys.exit(main())
