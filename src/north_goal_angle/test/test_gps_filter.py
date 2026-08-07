#!/usr/bin/env python3
"""gps_filter 순수 로직 테스트. python3 src/north_goal_angle/test/test_gps_filter.py"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from north_goal_angle.gps_filter import GPSFilter  # noqa: E402

_p = _t = 0


def check(name, fn):
    global _p, _t
    _t += 1
    try:
        fn(); _p += 1; print(f"  ✅ {name}")
    except AssertionError as e:
        print(f"  ❌ {name}\n     {e}")


# ---------------- 첫 fix 통과 + 스무딩 시작
def test_first_fix_passthrough():
    f = GPSFilter()
    ok, _ = f.update(35.0, 128.0, cov=0.5, now=0.0)
    assert ok and f.have_fix
    assert f.filtered_lat == 35.0 and f.filtered_lon == 128.0


# ---------------- 공분산 필터
def test_cov_reject():
    f = GPSFilter(cov_threshold=2.0)
    ok, reason = f.update(35.0, 128.0, cov=5.0, now=0.0)
    assert not ok and "cov" in reason, reason
    assert not f.have_fix   # 버려졌으니 fix 없음


def test_cov_zero_means_no_info():
    """cov 0/음수는 '정보 없음'이라 통과시킨다(임계로 안 버린다)."""
    f = GPSFilter(cov_threshold=2.0)
    ok, _ = f.update(35.0, 128.0, cov=0.0, now=0.0)
    assert ok


# ---------------- 점프 필터
def test_jump_reject():
    f = GPSFilter(max_speed_mps=15.0)
    f.update(35.0, 128.0, cov=0.5, now=0.0)
    # 다음 fix 가 0.1초 만에 위도 0.01도(~1.1km) 점프 → 수천 m/s
    ok, reason = f.update(35.01, 128.0, cov=0.5, now=0.1)
    assert not ok and "jump" in reason, reason


def test_normal_speed_ok_and_estimated():
    f = GPSFilter(max_speed_mps=15.0)
    f.update(35.0, 128.0, cov=0.5, now=0.0)
    # 1초에 위도 0.00001도(~1.1m) → ~1.1 m/s (정상)
    ok, _ = f.update(35.00001, 128.0, cov=0.5, now=1.0)
    assert ok
    assert 0.5 < f.estimated_speed_mps < 2.0, f.estimated_speed_mps


# ---------------- 스무딩: 작은 오차는 강하게 억제
def test_smoothing_small_error_strong():
    f = GPSFilter()
    f.update(35.0, 128.0, cov=0.5, now=0.0)      # filtered=(35,128)
    # 아주 작은 흔들림(<0.5m) → alpha 0.15 라 raw 쪽으로 조금만
    f.update(35.000001, 128.0, cov=0.5, now=1.0)
    # filtered 는 raw(35.000001)와 이전(35.0) 사이, 이전에 가깝다(alpha 0.15)
    assert 35.0 < f.filtered_lat < 35.000001
    assert (f.filtered_lat - 35.0) < 0.15 * 0.000001 + 1e-12 + (0.15 * 0.000001)


def test_smoothing_large_error_follows():
    f = GPSFilter()
    f.update(35.0, 128.0, cov=0.5, now=0.0)
    # 큰 오차(>1.5m, 위도 0.00003도~3.3m) → alpha 0.80 라 raw 를 강하게 따라감
    f.update(35.00003, 128.0, cov=0.5, now=1.0)
    frac = (f.filtered_lat - 35.0) / 0.00003
    assert frac > 0.7, f"큰 오차인데 안 따라감: frac={frac}"


def main():
    print("=== gps_filter 테스트 ===")
    for n, f in sorted(globals().items()):
        if n.startswith("test_") and callable(f):
            check(n[5:], f)
    print(f"\n{_p}/{_t} 통과")
    return 0 if _p == _t else 1


if __name__ == "__main__":
    sys.exit(main())
