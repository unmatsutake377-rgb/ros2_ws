#!/usr/bin/env python3
"""geofence_rays 순수 로직 테스트 — /geofence_state 좌우 부호 고정.
실행: python3 src/north_goal_angle/test/test_geofence_rays.py

계약서 docs/문제와작업/계약서_geofence부호_20261104.md §8-①.
규약: **배열 인덱스 0 (rel −80) = 우현**, 끝 인덱스 (+80) = 좌현 — LiDAR a−80·/candidate_angle 과 같은
좌 양수. /imu/yaw 는 나침반 CW+ 라 절대 방위 = yaw − rel.
🚨 수정 전 코드(`yaw + rel`)에서는 아래 비대칭 케이스가 **실패**해야 한다 (우현 벽이 끝 인덱스에 찍힌다).
   `yaw + rel` 로 되돌리면 이 파일이 잡는다 — "오타 수정"이 아니다.

north_goal_angle.py 는 rclpy·geopy 를 import 하므로(Mac 에 없음) 노드 모듈이 아니라 순수 모듈만 읽는다.
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from north_goal_angle.geofence_rays import (  # noqa: E402
    cast_rays, _ray_polygon_dist, _point_in_polygon)

INF = float('inf')
RAY_MIN, RAY_MAX, INC, MAX_RANGE = -80.0, 80.0, 1.0, 30.0   # yaml 기본값과 같은 격자
I_RIGHT_END, I_FRONT, I_LEFT_END = 0, 80, 160                  # 인덱스 0 = 우현 끝, 160 = 좌현 끝
R_WALL_3M_AT_80 = 3.0 / math.sin(math.radians(80.0))           # ≈ 3.046 m

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


def box(east, west, north, south):
    """축 정렬 사각형 폴리곤, 보트 기준 로컬 (east, north) m. 반시계 순."""
    return [(west, south), (east, south), (east, north), (west, north)]


def rays(yaw, pts):
    return cast_rays(yaw, pts, RAY_MIN, RAY_MAX, INC, MAX_RANGE)


# ---------------- 비대칭 케이스 — 거울을 잡는다
def test_starboard_wall_lands_on_index_0():
    """yaw=0(북향), 동벽 +3 m(우현) → 인덱스 0(rel −80 = 우현) ≈ 3.046 m, 인덱스 160(좌현) inf, 정면 inf.
    수정 전(`yaw + rel`)은 r[160]≈3.046 / r[0] inf 로 뒤집혀 실패한다."""
    r = rays(0.0, box(east=3.0, west=-47.0, north=40.0, south=-40.0))
    assert len(r) == 161, len(r)
    assert math.isclose(r[I_RIGHT_END], R_WALL_3M_AT_80, rel_tol=1e-6), f"r[0]={r[0]} (우현 벽이어야)"
    assert r[I_LEFT_END] == INF, f"r[160]={r[160]} — 좌현에 벽이 없는데 유한값: 거울"
    assert r[I_FRONT] == INF, f"r[80]={r[80]} (북벽 40 m > 30 m)"


def test_port_wall_lands_on_last_index():
    """yaw=90(동향), 북벽 +3 m(좌현) → 인덱스 160(rel +80 = 좌현) ≈ 3.046 m, 인덱스 0 inf, 정면 inf(동벽 40 m)."""
    r = rays(90.0, box(east=40.0, west=-40.0, north=3.0, south=-47.0))
    assert math.isclose(r[I_LEFT_END], R_WALL_3M_AT_80, rel_tol=1e-6), f"r[160]={r[160]} (좌현 벽이어야)"
    assert r[I_RIGHT_END] == INF, f"r[0]={r[0]} — 우현에 벽이 없는데 유한값: 거울"
    assert r[I_FRONT] == INF, f"r[80]={r[80]}"


def test_starboard_wall_yaw_wraps_past_360():
    """yaw=350(래핑), 우현 벽 → 같은 인덱스 0 에 유한값, 좌현 끝 inf. (절대 방위 350−(−80)=430→70°)"""
    r = rays(350.0, box(east=3.0, west=-47.0, north=40.0, south=-40.0))
    expect = 3.0 / math.sin(math.radians(70.0))
    assert math.isclose(r[I_RIGHT_END], expect, rel_tol=1e-6), f"r[0]={r[0]} expected {expect}"
    assert r[I_LEFT_END] == INF, f"r[160]={r[160]}"


def test_finite_cells_are_on_starboard_half_only():
    """우현 벽일 때 유한값은 전부 인덱스 < 80(우현 반쪽)에만 있어야 한다 — 부호가 통째로 뒤집히면 좌반쪽에 몰린다."""
    r = rays(0.0, box(east=3.0, west=-47.0, north=40.0, south=-40.0))
    finite_idx = [k for k, v in enumerate(r) if math.isfinite(v)]
    assert finite_idx, "유한값이 하나도 없다"
    assert max(finite_idx) < I_FRONT, f"좌현 반쪽에 유한값: {[k for k in finite_idx if k >= I_FRONT][:5]}"


# ---------------- 대칭 케이스 — 거울을 **못 잡는다** (문서화용)
def test_front_wall_symmetric_cannot_detect_mirror():
    """yaw=90, 동벽 +5 m 정면 → r[80]=5.0. 좌우 대칭이라 `yaw ± rel` 어느 쪽이든 같다.
    이 케이스만으로는 부호 오류를 못 잡는다 — 위 비대칭 케이스가 방어선이다."""
    r = rays(90.0, box(east=5.0, west=-40.0, north=40.0, south=-40.0))
    assert math.isclose(r[I_FRONT], 5.0, rel_tol=1e-9), f"r[80]={r[80]}"
    assert math.isclose(r[I_FRONT - 10], r[I_FRONT + 10], rel_tol=1e-9), "정면 벽은 좌우 대칭이어야"


# ---------------- 격자·범위 규약
def test_beyond_max_range_is_inf_and_grid_count():
    """모든 벽이 30 m 밖 → 전부 inf, 길이 161."""
    r = rays(0.0, box(east=50.0, west=-50.0, north=50.0, south=-50.0))
    assert len(r) == 161 and all(v == INF for v in r), r[:3]


def test_invalid_grid_returns_empty():
    """inc ≤ 0 / max < min 이면 빈 리스트 — 노드는 이를 '없음'(빈 배열 발행)으로 다룬다."""
    pts = box(3.0, -3.0, 3.0, -3.0)
    assert cast_rays(0.0, pts, -80.0, 80.0, 0.0, 30.0) == []
    assert cast_rays(0.0, pts, 80.0, -80.0, 1.0, 30.0) == []


# ---------------- 이동한 기하 함수 (동작 불변 확인)
def test_ray_polygon_dist_and_point_in_polygon():
    pts = box(3.0, -3.0, 3.0, -3.0)
    assert math.isclose(_ray_polygon_dist((0.0, 0.0), (1.0, 0.0), pts), 3.0)
    assert _ray_polygon_dist((0.0, 0.0), (0.0, 1.0), [(10.0, 5.0), (12.0, 5.0), (11.0, 7.0)]) is None
    assert _point_in_polygon((0.0, 0.0), pts) is True
    assert _point_in_polygon((4.0, 0.0), pts) is False


def main():
    print("=== geofence_rays 부호 테스트 (계약서_geofence부호_20261104) ===")
    for n, f in sorted(globals().items()):
        if n.startswith("test_") and callable(f):
            check(n[5:], f)
    print(f"\n{_p}/{_t} 통과")
    return 0 if _p == _t else 1


if __name__ == "__main__":
    sys.exit(main())
