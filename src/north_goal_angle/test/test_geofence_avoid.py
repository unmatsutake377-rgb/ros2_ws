#!/usr/bin/env python3
"""장애물 구간 경계(geofence_avoid_polygon) 순수 로직 + 노드 배선 정적 검사.

실행: python3 src/north_goal_angle/test/test_geofence_avoid.py

규정 종합임무 (4): 「장애물 구간의 가상경계선을 확연하게 벗어나 경기를 진행하는 경우 … 실격」.
  · 켜는 WP = mode 5 이고 WP 좌표가 폴리곤 안 — mode 5 만으로 켜지 않는다(도킹 접근점도 mode 5).
  · 두 벽은 방향마다 가까운 쪽(merge_min).
north_goal_angle.py 는 rclpy·geopy 를 import 하므로 노드는 소스를 ast·문자열로 본다.
"""

import ast
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from north_goal_angle.geofence_rays import (  # noqa: E402
    avoid_fence_wps, edge_dist_m, latlon_in_polygon, local_xy, merge_min,
    M_PER_DEG_LAT, M_PER_DEG_LON_EQ)

_NODE_SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..',
                         'north_goal_angle', 'north_goal_angle.py')
LAT0, LON0 = 35.1866, 128.5655
KX = math.cos(math.radians(LAT0)) * M_PER_DEG_LON_EQ
INF = float('inf')

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


def ll(east, north):
    """(LAT0, LON0) 기준 로컬 m → (lat, lon)."""
    return LAT0 + north / M_PER_DEG_LAT, LON0 + east / KX


def poly_box(w, e, s, n):
    out = []
    for x, y in ((w, s), (e, s), (e, n), (w, n)):
        out += list(ll(x, y))
    return out


ZONE = poly_box(-5.0, 5.0, 0.0, 30.0)          # 폭 10 m, 길이 30 m


def wp(east, north, mode):
    return (*ll(east, north), mode, 1.0)


def test_flag_needs_mode5_and_inside():
    wps = [wp(0, -10, 2),      # 구역 밖 mode 2
           wp(0, 4, 5),        # 회피 시작 — 안
           wp(0, 26, 5),       # 회피 끝 — 안
           wp(0, 40, 5),       # 도킹 접근 — 밖 mode 5 → 꺼짐
           wp(0, 15, 7)]       # 안이지만 mode 7 → 꺼짐
    assert avoid_fence_wps(wps, ZONE) == [False, True, True, False, False]


def test_empty_polygon_all_off():
    wps = [wp(0, 4, 5), wp(0, 26, 5)]
    assert avoid_fence_wps(wps, []) == [False, False]
    assert latlon_in_polygon(*ll(0, 4), []) is False
    assert edge_dist_m(*ll(0, 4), []) is None


def test_edge_distance_m():
    d = edge_dist_m(*ll(0, 4), ZONE)              # 남쪽 변까지 4 m, 동·서 5 m
    assert math.isclose(d, 4.0, abs_tol=0.05), d
    d = edge_dist_m(*ll(3, 15), ZONE)             # 동쪽 변 2 m
    assert math.isclose(d, 2.0, abs_tol=0.05), d


def test_local_xy_matches_ll():
    x, y = local_xy(LAT0, LON0, *ll(7.0, -3.0))
    assert math.isclose(x, 7.0, abs_tol=1e-6) and math.isclose(y, -3.0, abs_tol=1e-6), (x, y)


def test_merge_min_takes_nearer_wall():
    assert merge_min([3.0, INF, 5.0], [INF, 2.0, 6.0]) == [3.0, 2.0, 5.0]


# ---------------- 노드 배선 (정적)
def _src():
    return open(_NODE_SRC, encoding='utf-8').read()


def _method(name):
    tree = ast.parse(_src())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'NorthGoalAngle')
    return next(f for f in cls.body if isinstance(f, ast.FunctionDef) and f.name == name)


def test_param_dynamic_typing_and_sanitized():
    src = _src()
    assert ("'geofence_avoid_polygon', [], ParameterDescriptor(dynamic_typing=True)" in src)
    assert "sanitize_geofence_polygon(raw_avoid)" in src


def test_flags_after_waypoints_loaded():
    src = _src()
    i_load = src.index("self.waypoints = self._load_waypoints(self.waypoints_file)")
    i_flag = src.index("self.avoid_wps = avoid_fence_wps(self.waypoints, self.avoid_polygon)")
    i_timer = src.index("self.create_timer(self.period_sec")
    assert i_load < i_flag < i_timer


def test_ranges_merges_and_guards():
    seg = ast.get_source_segment(_src(), _method('_geofence_ranges'))
    assert "merge_min(ranges, ar)" in seg
    assert "self._avoid_fence_on()" in seg
    # 구역 밖이면 벽을 세우지 않는다(입구·복귀로) — cast_rays 는 안일 때만
    i_in = seg.index("if _point_in_polygon((0.0, 0.0), apts):")
    i_cast = seg.index("ar = cast_rays(")
    i_else = seg.index("elif self.avoid_entered:")
    assert i_in < i_cast < i_else
    # 켜는 구간을 벗어나면 리셋
    assert "self.avoid_entered = False" in seg


def test_avoid_on_uses_wp_idx_bounds():
    seg = ast.get_source_segment(_src(), _method('_avoid_fence_on'))
    assert "self.wp_idx < len(self.avoid_wps)" in seg


def test_geofence_on_start_log():
    """설명서 §3-1 이 확인하라는 'geofence ON(N점)' 시작 로그가 실제로 있다."""
    assert 'f"🧱 geofence ON({len(self.geofence_polygon) // 2}점)' in _src()
    assert 'f"🚧 장애물 구간 경계 ON({n_pts}점)' in _src()


def test_yaml_keys_present():
    cfg = os.path.join(os.path.dirname(_NODE_SRC), '..', 'config')
    for f in ('north_goal_angle.yaml', 'north_goal_angle_main.yaml'):
        s = open(os.path.join(cfg, f), encoding='utf-8').read()
        assert "geofence_avoid_polygon: []" in s, f


def main():
    print("=== 장애물 구간 경계 (규정 종합임무 (4)) ===")
    for n, f in sorted(globals().items()):
        if n.startswith("test_") and callable(f):
            check(n[5:], f)
    print(f"\n{_p}/{_t} 통과")
    return 0 if _p == _t else 1


if __name__ == "__main__":
    sys.exit(main())
