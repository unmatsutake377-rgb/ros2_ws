"""lidar_track 단위 테스트 — ROS 없이 돈다.

    python3 src/ship_turn/test/test_lidar_track.py

프레임: 빔 각 a = angle_min + i·inc, rel = a − 80 (좌 양수). 시뮬 /scan 은 −180~180·0.5° (ssf_sim.py).
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from ship_turn.lidar_track import lidar_track  # noqa: E402

FWD = 80.0
A_MIN = -180.0
INC = 0.5
N = int(360.0 / INC)
fails = 0


def check(name, cond, extra=""):
    global fails
    print(f"  {'✅' if cond else '❌'} {name} {extra}")
    if not cond:
        fails += 1
    assert cond, name


def scan_with(points, a_min=A_MIN, inc=INC, n=N):
    """points = [(rel_deg, dist_m), ...] 를 빔 1개씩 심은 ranges."""
    ranges = [math.inf] * n
    for rel, d in points:
        a = FWD + rel
        i = int(round(((a - a_min) % 360.0) / inc)) % n
        ranges[i] = d
    return ranges


def test_single_buoy_starboard():
    """부표 1개 −90°(우현) 3 m, 예상 −90 → (−90, 3.0)."""
    r = lidar_track(scan_with([(-90.0, 3.0)]), A_MIN, INC, -90.0, 25.0, 5.0, FWD)
    check("(−90, 3.0)", r is not None and abs(r[0] + 90.0) < 1e-6 and abs(r[1] - 3.0) < 1e-9, f"r={r}")


def test_outside_window_is_none():
    """부표가 창 밖(−140°, 예상 −90 ± 25) → None. 창 밖 최근접 금지."""
    r = lidar_track(scan_with([(-140.0, 3.0)]), A_MIN, INC, -90.0, 25.0, 5.0, FWD)
    check("창 밖 → None", r is None, f"r={r}")


def test_wall_beyond_max_is_none():
    """창 안이지만 6 m(max 5) → None(벽 배제)."""
    r = lidar_track(scan_with([(-90.0, 6.0)]), A_MIN, INC, -90.0, 25.0, 5.0, FWD)
    check("6 m > max 5 → None", r is None, f"r={r}")


def test_window_edge_inclusive():
    """창 경계(−115 = −90 − 25) 는 포함."""
    r = lidar_track(scan_with([(-115.0, 3.0)]), A_MIN, INC, -90.0, 25.0, 5.0, FWD)
    check("경계 −115 포함", r is not None and abs(r[0] + 115.0) < 1e-6, f"r={r}")


def test_two_buoys_picks_nearest_documented_limit():
    """🚩 한계 문서화: 부표 2개(−90° 3 m, −70° 2.5 m), 예상 −90 → **가까운 −70 쪽**을 고른다.
    예상 방위에 더 가까운 쪽이 아니라 거리 최근접이다. 완화는 창 25°·직전 rel 연속성뿐(계약서 §5-B 위험 ②)."""
    r = lidar_track(scan_with([(-90.0, 3.0), (-70.0, 2.5)]), A_MIN, INC, -90.0, 25.0, 5.0, FWD)
    check("가까운 −70 선택(현재 한계)", r is not None and abs(r[0] + 70.0) < 1e-6 and abs(r[1] - 2.5) < 1e-9,
          f"r={r}")


def test_nearest_within_window_not_outside():
    """창 밖에 더 가까운 반사(−140° 1 m)가 있어도 창 안(−90° 3 m)을 고른다."""
    r = lidar_track(scan_with([(-90.0, 3.0), (-140.0, 1.0)]), A_MIN, INC, -90.0, 25.0, 5.0, FWD)
    check("창 밖 1 m 무시, 창 안 3 m", r is not None and abs(r[0] + 90.0) < 1e-6, f"r={r}")


def test_invalid_ranges_ignored():
    """inf / nan / 0 / 음수는 무시."""
    ranges = scan_with([(-90.0, 3.0)])
    i_nan = int(round(((FWD - 95.0 - A_MIN) % 360.0) / INC))
    i_zero = int(round(((FWD - 85.0 - A_MIN) % 360.0) / INC))
    ranges[i_nan] = math.nan
    ranges[i_zero] = 0.0
    r = lidar_track(ranges, A_MIN, INC, -90.0, 25.0, 5.0, FWD)
    check("nan·0 무시 → (−90, 3.0)", r is not None and abs(r[0] + 90.0) < 1e-6, f"r={r}")


def test_empty_or_bad_input():
    """빈 스캔 / inc 0 / 예상 None → None."""
    check("빈 스캔", lidar_track([], A_MIN, INC, -90.0, 25.0, 5.0, FWD) is None)
    check("inc 0", lidar_track(scan_with([(-90.0, 3.0)]), A_MIN, 0.0, -90.0, 25.0, 5.0, FWD) is None)
    check("예상 None", lidar_track(scan_with([(-90.0, 3.0)]), A_MIN, INC, None, 25.0, 5.0, FWD) is None)
    check("ranges None", lidar_track(None, A_MIN, INC, -90.0, 25.0, 5.0, FWD) is None)


def test_angle_min_zero_frame_same_result():
    """실 드라이버가 0~360 프레임(angle_min 0)이어도 a−80 규약은 같다: 빔 a=350 → rel −90."""
    ranges = scan_with([(-90.0, 3.0)], a_min=0.0)
    r = lidar_track(ranges, 0.0, INC, -90.0, 25.0, 5.0, FWD)
    check("angle_min 0 프레임 → (−90, 3.0)", r is not None and abs(r[0] + 90.0) < 1e-6, f"r={r}")


def test_window_wraps_pm180():
    """예상 +170, 창 25 → −175(=185) 도 창 안. ±180 경계에서 각도 차를 wrap 한다."""
    r = lidar_track(scan_with([(-175.0, 3.0)]), A_MIN, INC, 170.0, 25.0, 5.0, FWD)
    check("±180 경계 wrap", r is not None and abs(abs(r[0]) - 175.0) < 1e-6, f"r={r}")


def test_ccw_port_side():
    """CCW 설계 자세: 부표 좌현 +90 → (+90, 3.0)."""
    r = lidar_track(scan_with([(90.0, 3.0)]), A_MIN, INC, 90.0, 25.0, 5.0, FWD)
    check("(+90, 3.0)", r is not None and abs(r[0] - 90.0) < 1e-6, f"r={r}")


if __name__ == '__main__':
    tests = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for fn in tests:
        try:
            fn()
        except AssertionError as e:
            fails += 1
            print(f"  ❌ {fn.__name__}: {e}")
    print(f"\n{'PASS' if fails == 0 else str(fails) + ' FAIL'}")
    sys.exit(1 if fails else 0)
