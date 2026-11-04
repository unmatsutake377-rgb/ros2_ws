#!/usr/bin/env python3
"""los_logic 순수 로직 테스트. python3 src/north_goal_angle/test/test_los_logic.py"""

import os
import re
import sys

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from north_goal_angle.los_logic import (  # noqa: E402
    calc_dist, calc_angle, get_dynamic_lookahead,
    cross_track_error, calc_los_bearing, get_dynamic_arrive_radius,
    along_track, calc_guidance_bearing,
    DEFAULT_ARRIVE_RADIUS_HOLD_MODES_M, HOLD_MODES, get_arrive_radius, sanitize_hold_radius,
    SHIP_BACK_HOLD_OUTER_M, hold_radius_inside_hold_dist,
    HOLD_DWELL_WARN_SEC, stale_hold_dwells,
)
from north_goal_angle.waypoint_loader import parse_waypoints, waypoint_labels  # noqa: E402
from north_goal_angle.wp_advance import wp_step  # noqa: E402

_PKG = os.path.join(os.path.dirname(__file__), '..')
_PARAMS_YAML = os.path.join(_PKG, 'config', 'north_goal_angle.yaml')
_NODE_SRC = os.path.join(_PKG, 'north_goal_angle', 'north_goal_angle.py')
_SHIP_BACK_YAML = os.path.join(_PKG, '..', 'ship_back', 'config', 'ship_back.yaml')
_WAYPOINTS_YAML = os.path.join(_PKG, 'config', 'waypoints.yaml')

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


# ---------------- 지나침 복귀 (계약서_전체코스_20261104 §2-1)
# 경로: p(0,0) → wp(0.002,0) 북향, 길이 ≈ 221 m. 위도 0.0001° ≈ 11 m.
def test_along_track_before_and_after():
    s0, L = along_track(0.0, 0.0, 0.002, 0.0, 0.001, 0.0)
    assert abs(L - 222.4) < 1.0, L
    assert abs(s0 - L / 2) < 1.0, s0
    s1, _ = along_track(0.0, 0.0, 0.002, 0.0, 0.0022, 0.0001)   # WP 를 22 m 지나침
    assert s1 > L, (s1, L)
    s2, _ = along_track(0.0, 0.0, 0.002, 0.0, -0.0001, 0.0)     # 출발점 뒤
    assert s2 < 0, s2


def test_passed_outside_radius_direct_bearing():
    """WP 를 북쪽으로 지나쳐 동쪽에 있으면 → 직접 방위(남서쪽), xte 0."""
    boat = (0.0022, 0.0002)
    des, xte, direct = calc_guidance_bearing(0.0, 0.0, 0.002, 0.0, *boat, 3.0, 2.5,
                                             integral_xte=40.0, ki=0.02)
    assert direct is True
    assert xte == 0.0, f"지나친 뒤 xte 가 0 이 아님(ILOS 적분이 쌓인다): {xte}"
    assert abs(sdiff(des, calc_angle(*boat, 0.002, 0.0))) < 1e-6, des
    assert 180.0 < des < 270.0, f"WP 쪽(남서)으로 안 돎: {des}"


def test_passed_old_los_runs_away():
    """기존 LOS 는 같은 위치에서 경로 연장선(북)을 따라 멀어진다 — 고친 이유."""
    des, _ = calc_los_bearing(0.0, 0.0, 0.002, 0.0, 0.0022, 0.0002, lookahead_m=3.0)
    assert abs(sdiff(des, 0.0)) < 90.0, f"옛 LOS 가 북쪽(WP 반대)이 아님: {des}"
    des_new, _, _ = calc_guidance_bearing(0.0, 0.0, 0.002, 0.0, 0.0022, 0.0002, 3.0, 2.5)
    assert abs(sdiff(des_new, 180.0)) < 90.0, f"새 방위가 남쪽(WP)으로 안 돎: {des_new}"


def test_passed_inside_radius_keeps_los():
    """지나쳤어도 반경 안이면 기존 LOS 그대로(dwell 중 방위가 튀지 않게)."""
    boat = (0.002 + 0.00001, 0.000005)     # WP 북쪽 1.1 m, 동쪽 0.6 m
    args = (0.0, 0.0, 0.002, 0.0, *boat, 3.0)
    des, xte, direct = calc_guidance_bearing(*args, 2.5, integral_xte=5.0, ki=0.02)
    los, los_xte = calc_los_bearing(*args, integral_xte=5.0, ki=0.02)
    assert direct is False
    assert des == los and xte == los_xte, (des, los, xte, los_xte)


def test_not_passed_keeps_los():
    """지나치기 전(s < L)이면 반경 밖이라도 기존 LOS 와 같다."""
    args = (0.0, 0.0, 0.002, 0.0, 0.001, 0.0005, 3.0)
    des, xte, direct = calc_guidance_bearing(*args, 2.5, integral_xte=10.0, ki=0.02)
    los, los_xte = calc_los_bearing(*args, integral_xte=10.0, ki=0.02)
    assert direct is False
    assert des == los and xte == los_xte


def test_radius_boundary():
    """WP 거리 ≥ 반경이면 직접, < 반경이면 LOS — 반경 인자를 그대로 쓴다."""
    boat = (0.002 + 0.00003, 0.0)          # WP 북쪽 ≈ 3.3 m
    d = calc_dist(*boat, 0.002, 0.0)
    _, _, direct_small = calc_guidance_bearing(0.0, 0.0, 0.002, 0.0, *boat, 3.0, d - 0.01)
    _, _, direct_big = calc_guidance_bearing(0.0, 0.0, 0.002, 0.0, *boat, 3.0, d + 0.01)
    assert direct_small is True and direct_big is False


def test_zero_length_path_direct():
    """경로 길이 0(WP8·WP9 같은 좌표처럼 prev == wp) → 직접 방위, xte 0."""
    boat = (-0.0001, 0.0001)               # WP 남동쪽
    des, xte, direct = calc_guidance_bearing(0.002, 0.0, 0.002, 0.0, *boat, 3.0, 2.5,
                                             integral_xte=10.0, ki=0.02)
    assert direct is True and xte == 0.0
    assert abs(sdiff(des, calc_angle(*boat, 0.002, 0.0))) < 1e-6
    # 반경 안이어도 경로선이 없으니 직접(옛 코드는 경로 방위를 북 0° 로 잘못 잡았다)
    near = (0.002 - 0.00001, 0.0)
    _, xte2, direct2 = calc_guidance_bearing(0.002, 0.0, 0.002, 0.0, *near, 3.0, 2.5)
    assert direct2 is True and xte2 == 0.0


# ---------------- 동적 도착반경
def test_arrive_radius():
    assert get_dynamic_arrive_radius(7, 5.0) == 2.0     # 도킹은 좁게
    assert get_dynamic_arrive_radius(0, 1.0) == 2.5
    assert get_dynamic_arrive_radius(0, 3.0) == 3.5
    assert get_dynamic_arrive_radius(0, 6.0) == 4.5


# ---------------- mode 2·3 고정 도착 반경 (계약서_전체코스_20261104 §10 추가 결정 3)
def _arrives(dist, arrive_r):
    """반경 안 dwell 0.5 s 를 두 틱(0.5 s 주기)에 채우면 'arrive' 인가."""
    _, ts, _ = wp_step(0.0, 0.0, 120.0, dist, arrive_r, None, 0.5, True)
    ev, _, _ = wp_step(0.5, 0.0, 120.0, dist, arrive_r, ts, 0.5, True)
    return ev == 'arrive'


def test_hold_radius_default_value():
    assert DEFAULT_ARRIVE_RADIUS_HOLD_MODES_M == 3.5
    assert type(DEFAULT_ARRIVE_RADIUS_HOLD_MODES_M) is float
    assert DEFAULT_ARRIVE_RADIUS_HOLD_MODES_M > 2.5 + 0.4, "ship_back 유지 거리 밖이어야 한다"
    assert tuple(HOLD_MODES) == (2, 3)


def test_hold_modes_fixed_radius_any_speed():
    for m in (2, 3):
        for v in (0.0, 1.0, 3.0, 6.0):
            assert get_arrive_radius(m, v) == 3.5, (m, v)
            assert get_arrive_radius(m, v, 4.0) == 4.0, (m, v)


def test_hold_modes_boundary_349_arrive_351_not():
    for m in (2, 3):
        r = get_arrive_radius(m, 1.0)
        assert _arrives(3.49, r) is True, m
        assert _arrives(3.51, r) is False, m
    # 옛 반경(저속 2.5 m)이면 3.49 도 미도착이었다 — 바뀐 것은 mode 2·3 뿐
    assert _arrives(3.49, get_dynamic_arrive_radius(2, 1.0)) is False


def test_other_modes_keep_old_radius():
    for m in (0, 1, 5, 7, 8):
        for v in (0.0, 1.0, 3.0, 6.0):
            assert get_arrive_radius(m, v) == get_dynamic_arrive_radius(m, v), (m, v)
            assert get_arrive_radius(m, v, 9.0) == get_dynamic_arrive_radius(m, v), (m, v)
    assert get_arrive_radius(7, 1.0) == 2.0                 # 도킹 무관
    assert get_arrive_radius(0, 1.0) == 2.5
    assert _arrives(3.49, get_arrive_radius(0, 1.0)) is False


def test_sanitize_hold_radius():
    assert sanitize_hold_radius(3.5) == (3.5, True)
    assert sanitize_hold_radius(4.0) == (4.0, True)
    for bad in (0.0, -1.0, float('nan'), float('inf'), None, True, '3.5'):
        v, ok = sanitize_hold_radius(bad)
        assert ok is False and v == DEFAULT_ARRIVE_RADIUS_HOLD_MODES_M, bad


def test_hold_radius_small_warn_threshold():
    """리뷰 권고: 양수지만 ≤ 2.9 m(ship_back 유지 거리 바깥 끝)면 시작 WARN — 값은 그대로 쓴다."""
    assert abs(SHIP_BACK_HOLD_OUTER_M - (2.5 + 0.4)) < 1e-9
    for r in (0.5, 2.5, 2.9):
        assert hold_radius_inside_hold_dist(r) is True, r
        assert sanitize_hold_radius(r) == (r, True), "작아도 정상값 — 그대로 쓴다"
    for r in (2.91, 3.5, 4.0):
        assert hold_radius_inside_hold_dist(r) is False, r
    assert hold_radius_inside_hold_dist(DEFAULT_ARRIVE_RADIUS_HOLD_MODES_M) is False


def test_hold_outer_matches_ship_back_yaml():
    """SHIP_BACK_HOLD_OUTER_M = ship_back.yaml hold_dist_m + dist_tol_m (읽기만 한다)."""
    if not os.path.exists(_SHIP_BACK_YAML):
        return
    with open(_SHIP_BACK_YAML, encoding='utf-8') as f:
        raw = yaml.safe_load(f)
    p = next(iter(raw.values()))['ros__parameters']
    outer = float(p['hold_dist_m']) + float(p['dist_tol_m'])
    assert abs(outer - SHIP_BACK_HOLD_OUTER_M) < 1e-9, (outer, SHIP_BACK_HOLD_OUTER_M)


def test_node_warns_small_hold_radius_once():
    """노드: 잘못된 값이면 ERROR(기본값), 정상이지만 ≤ 2.9 면 WARN — 둘 중 하나만, 시작 때 1회."""
    src = open(_NODE_SRC, encoding='utf-8').read()
    i_init = src.index("'arrive_radius_hold_modes_m'")
    seg = src[i_init:src.index("self.create_timer(self.period_sec")]
    assert re.search(r"if not hold_r_ok:\s*\n\s*self\._log_hold_radius_invalid\(raw_hold_r\)"
                     r"\s*\n\s*elif los_logic\.hold_radius_inside_hold_dist\("
                     r"self\.arrive_radius_hold_m\):\s*\n\s*self\._log_hold_radius_small\(\)",
                     seg), "선언 직후 if/elif 로 ERROR·WARN 을 나눠야 한다"
    assert src.count("self._log_hold_radius_small()") == 1, "WARN 은 시작 때 한 곳에서만"
    body = src[src.index("def _log_hold_radius_small"):]
    body = body[:body.index("\n    def ", 1)]
    assert "get_logger().warn(" in body
    assert "위치유지 거리 안이라 DONE 뒤" in body and "부표 쪽으로 파고들 수 있음" in body


def test_hold_radius_yaml_matches_code_default():
    with open(_PARAMS_YAML, encoding='utf-8') as f:
        p = yaml.safe_load(f)['/north_goal_angle']['ros__parameters']
    v = p['arrive_radius_hold_modes_m']
    assert type(v) is float, "arrive_radius_hold_modes_m 는 소수(.0)로"
    assert v == DEFAULT_ARRIVE_RADIUS_HOLD_MODES_M, v


def test_node_declares_hold_radius_before_timer():
    src = open(_NODE_SRC, encoding='utf-8').read()
    assert re.search(
        r"declare_parameter\(\s*'arrive_radius_hold_modes_m',\s*"
        r"los_logic\.DEFAULT_ARRIVE_RADIUS_HOLD_MODES_M\)", src)
    assert src.index("'arrive_radius_hold_modes_m'") < src.index(
        "self.create_timer(self.period_sec")
    assert "los_logic.get_arrive_radius(" in src
    assert "los_logic.get_dynamic_arrive_radius(" not in src, "노드는 get_arrive_radius 만 쓴다"


# ---------------- 옛 녹화 파일 dwell 경고 (결정 6 보충 ③)
def _wp(mode, dwell):
    return [35.1, 128.5, mode, dwell]


def test_stale_hold_dwell_threshold():
    """1.0 은 경고 없음(경계 포함 안 함), 1.01 은 경고. 기본 문턱 1.0."""
    assert HOLD_DWELL_WARN_SEC == 1.0
    for m in (2, 3):
        assert stale_hold_dwells([_wp(m, 1.0)]) == [], m
        assert stale_hold_dwells([_wp(m, 0.5)]) == [], m
        assert stale_hold_dwells([_wp(m, 1.01)]) == [(0, "", m, 1.01)], m
        assert stale_hold_dwells([_wp(m, 3.0)]) == [(0, "", m, 3.0)], m


def test_stale_hold_dwell_other_modes_ignored():
    """mode 5·7·8(그리고 0·1)은 dwell 이 커도 무관."""
    wps = [_wp(m, 60.0) for m in (0, 1, 5, 7, 8)]
    assert stale_hold_dwells(wps) == []


def test_stale_hold_dwell_reports_index_and_label():
    raw = {"waypoints": [
        {"구역": "WP0 게이트 시작", "lat": 35.1, "lon": 128.5, "mode": 0, "dwell": 3.0},
        {"구역": "WP2 위치유지", "lat": 35.1, "lon": 128.5, "mode": 2, "dwell": 3.0},
        {"구역": "WP3 부표(초록)", "lat": 35.1, "lon": 128.5, "mode": 3, "dwell": 0.5},
        [35.1, 128.5, 3, 3.0],
    ]}
    hits = stale_hold_dwells(parse_waypoints(raw), waypoint_labels(raw))
    assert hits == [(1, "WP2 위치유지", 2, 3.0), (3, "", 3, 3.0)], hits
    assert stale_hold_dwells(parse_waypoints(raw)) == [(1, "", 2, 3.0), (3, "", 3, 3.0)]


def test_default_waypoints_no_stale_dwell():
    """기본 config/waypoints.yaml 은 경고가 나지 않는다(mode 2·3 dwell 0.5)."""
    with open(_WAYPOINTS_YAML, encoding='utf-8') as f:
        raw = yaml.safe_load(f)
    assert stale_hold_dwells(parse_waypoints(raw), waypoint_labels(raw)) == []


def test_node_warns_stale_hold_dwell_once_after_load():
    """노드: 웨이포인트 로드(parse_waypoints) 직후 WARN 1회, 값은 그대로 반환."""
    src = open(_NODE_SRC, encoding='utf-8').read()
    body = src[src.index("    def _load_waypoints"):]
    body = body[:body.index("\n    def ", 1)]
    i_parse = body.index("wps = parse_waypoints(raw)")
    i_stale = body.index("los_logic.stale_hold_dwells(wps, waypoint_labels(raw))")
    assert i_parse < i_stale < body.index("return wps")
    assert re.search(r"if stale:\s*\n\s*self\._log_stale_hold_dwells\(stale\)", body)
    assert src.count("self._log_stale_hold_dwells(") == 1, "WARN 은 로드 때 한 곳에서만"
    assert src.index("self.waypoints = self._load_waypoints(") < src.index(
        "self.create_timer(self.period_sec")
    log = src[src.index("def _log_stale_hold_dwells"):]
    log = log[:log.index("\n    def ", 1)]
    assert log.count("get_logger().warn(") == 1 and "for " not in log.split("warn(")[1]
    assert "옛 녹화 파일일 수 있음" in log and "DONE 뒤 dwell 동안 부표 쪽으로 감" in log
    assert "0.5 권장" in log


def main():
    print("=== los_logic 테스트 ===")
    for n, f in sorted(globals().items()):
        if n.startswith("test_") and callable(f):
            check(n[5:], f)
    print(f"\n{_p}/{_t} 통과")
    return 0 if _p == _t else 1


if __name__ == "__main__":
    sys.exit(main())
