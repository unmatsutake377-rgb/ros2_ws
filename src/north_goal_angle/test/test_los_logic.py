#!/usr/bin/env python3
"""los_logic 순수 로직 테스트. python3 src/north_goal_angle/test/test_los_logic.py"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from north_goal_angle.los_logic import (  # noqa: E402
    calc_dist, calc_angle, get_dynamic_lookahead,
    cross_track_error, calc_los_bearing, get_dynamic_arrive_radius,
)

_p = _t = 0


def check(name, fn):
    global _p, _t
    _t += 1
    try:
        fn(); _p += 1; print(f"  ✅ {name}")
    except AssertionError as e:
        print(f"  ❌ {name}\n     {e}")


def sdiff(a, b):
    """최단 각차 (a-b), [-180,180]."""
    return (a - b + 540.0) % 360.0 - 180.0


# ---------------- 거리/방위 기본
def test_dist_reasonable():
    # 위도 1도 ≈ 111km
    d = calc_dist(35.0, 128.0, 36.0, 128.0)
    assert 110000 < d < 112000, d


def test_angle_north_east():
    assert abs(sdiff(calc_angle(35.0, 128.0, 35.001, 128.0), 0.0)) < 1.0    # 북
    assert abs(sdiff(calc_angle(35.0, 128.0, 35.0, 128.001), 90.0)) < 1.0   # 동


# ---------------- 동적 lookahead
def test_lookahead_clamp():
    assert get_dynamic_lookahead(0.0, 1.2, 2.0, 6.0) == 2.0    # 느림 → min
    assert get_dynamic_lookahead(100.0, 1.2, 2.0, 6.0) == 6.0  # 빠름 → max
    assert abs(get_dynamic_lookahead(3.0, 1.2, 2.0, 6.0) - 3.6) < 1e-9


# ---------------- XTE 부호: 배가 경로 오른쪽 → +
def test_xte_sign_right_positive():
    # 경로 북향: p(0,0) → wp(0.002,0). 배가 동쪽(오른쪽)으로 밀림.
    xte, pb = cross_track_error(0.0, 0.0, 0.002, 0.0, 0.001, 0.0005)
    assert abs(sdiff(pb, 0.0)) < 1.0, f"경로방위 북 아님: {pb}"
    assert xte > 0, f"오른쪽 밀림인데 XTE 음수: {xte}"


def test_xte_sign_left_negative():
    xte, _ = cross_track_error(0.0, 0.0, 0.002, 0.0, 0.001, -0.0005)  # 서쪽(왼쪽)
    assert xte < 0, f"왼쪽 밀림인데 XTE 양수: {xte}"


def test_xte_on_path_zero():
    xte, _ = cross_track_error(0.0, 0.0, 0.002, 0.0, 0.001, 0.0)      # 선 위
    assert abs(xte) < 0.5, f"선 위인데 XTE 큼: {xte}"


# ---------------- LOS 방위: 오른쪽 밀림 → 경로방위보다 '왼쪽'으로 틀어 복귀
def test_los_right_steers_left():
    des, xte = calc_los_bearing(0.0, 0.0, 0.002, 0.0, 0.001, 0.0005, lookahead_m=3.0)
    # 경로 북(0). 오른쪽 밀림이면 왼쪽(서, 음의 방향)으로 → sdiff(des, 0) < 0
    assert sdiff(des, 0.0) < 0, f"오른쪽 밀림인데 왼쪽으로 안 틈: des={des}"


def test_los_left_steers_right():
    des, _ = calc_los_bearing(0.0, 0.0, 0.002, 0.0, 0.001, -0.0005, lookahead_m=3.0)
    assert sdiff(des, 0.0) > 0, f"왼쪽 밀림인데 오른쪽으로 안 틈: des={des}"


def test_los_on_path_equals_path_bearing():
    des, _ = calc_los_bearing(0.0, 0.0, 0.002, 0.0, 0.001, 0.0, lookahead_m=3.0)
    assert abs(sdiff(des, 0.0)) < 1.0, f"선 위인데 경로방위와 다름: {des}"


def test_los_larger_lookahead_gentler():
    """전방주시가 멀수록 보정이 완만(각이 작다)."""
    d_near, _ = calc_los_bearing(0.0, 0.0, 0.002, 0.0, 0.001, 0.0005, lookahead_m=2.0)
    d_far, _ = calc_los_bearing(0.0, 0.0, 0.002, 0.0, 0.001, 0.0005, lookahead_m=6.0)
    assert abs(sdiff(d_far, 0.0)) < abs(sdiff(d_near, 0.0)), "멀리 봐도 보정이 더 큼"


def test_ilos_adds_correction():
    """적분항이 쌓이면 같은 XTE 라도 보정이 커진다(외란 상쇄)."""
    d0, _ = calc_los_bearing(0.0, 0.0, 0.002, 0.0, 0.001, 0.0005, 3.0, integral_xte=0.0, ki=0.02)
    d1, _ = calc_los_bearing(0.0, 0.0, 0.002, 0.0, 0.001, 0.0005, 3.0, integral_xte=100.0, ki=0.02)
    assert abs(sdiff(d1, 0.0)) > abs(sdiff(d0, 0.0)), "적분항이 보정을 안 키움"


# ---------------- 동적 도착반경
def test_arrive_radius():
    assert get_dynamic_arrive_radius(7, 5.0) == 2.0     # 도킹은 좁게
    assert get_dynamic_arrive_radius(0, 1.0) == 2.5
    assert get_dynamic_arrive_radius(0, 3.0) == 3.5
    assert get_dynamic_arrive_radius(0, 6.0) == 4.5


def main():
    print("=== los_logic 테스트 ===")
    for n, f in sorted(globals().items()):
        if n.startswith("test_") and callable(f):
            check(n[5:], f)
    print(f"\n{_p}/{_t} 통과")
    return 0 if _p == _t else 1


if __name__ == "__main__":
    sys.exit(main())
