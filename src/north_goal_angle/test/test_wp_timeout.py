#!/usr/bin/env python3
"""wp_timeout 순수 로직 테스트. python3 src/north_goal_angle/test/test_wp_timeout.py

계약서 docs/문제와작업/계약서_WP타임아웃모드기준_20261029.md — WP 타임아웃을 순번 → wp_mode 기준으로.
"""

import math
import os
import re
import sys

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from north_goal_angle.waypoint_loader import parse_waypoints, VALID_MODES  # noqa: E402
from north_goal_angle.wp_timeout import (  # noqa: E402
    DEFAULT_WP_TIMEOUT_MODES, DEFAULT_WP_TIMEOUT_SEC, WpTimeoutError,
    build_timeout_table, dwell_conflicts, per_wp_timeouts, timeout_for_mode,
)

_PKG = os.path.join(os.path.dirname(__file__), '..')
_PARAMS_YAML = os.path.join(_PKG, 'config', 'north_goal_angle.yaml')
_WAYPOINTS_YAML = os.path.join(_PKG, 'config', 'waypoints.yaml')
_NODE_SRC = os.path.join(_PKG, 'north_goal_angle', 'north_goal_angle.py')

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


def _raises(modes, secs):
    """build_timeout_table 이 WpTimeoutError 를 던지면 True."""
    try:
        build_timeout_table(modes, secs)
        return False
    except WpTimeoutError:
        return True


def _default_table():
    return build_timeout_table(DEFAULT_WP_TIMEOUT_MODES, DEFAULT_WP_TIMEOUT_SEC)


def _real_waypoints():
    with open(_WAYPOINTS_YAML, encoding='utf-8') as f:
        return parse_waypoints(yaml.safe_load(f))


def _yaml_params():
    with open(_PARAMS_YAML, encoding='utf-8') as f:
        return yaml.safe_load(f)['/north_goal_angle']['ros__parameters']


def _old_rule(idx):
    """옛 순번 규칙 (north_goal_angle.py 옛 :418-422) — 회귀 비교용."""
    if idx in (3, 4, 5):
        return 90.0
    if idx <= 6:
        return 120.0
    return None


# ---------------- 표 생성: 정상
def test_default_table():
    t = _default_table()
    assert t == {0: 120.0, 1: 120.0, 2: 120.0, 3: 90.0, 5: 120.0, 7: 150.0}, t


def test_accepts_tuple_and_int_secs():
    t = build_timeout_table((3,), (90,))
    assert t == {3: 90.0} and isinstance(t[3], float), t


def test_reject_empty_table():
    """빈 표 = 모든 모드 타임아웃 없음 → GPS 없으면 영구 정체. 출발 전 거부."""
    assert _raises([], [])
    assert _raises((), ())


def test_reject_none():
    """None 을 조용히 빈 표로 바꾸지 않는다."""
    assert _raises(None, None)
    assert _raises([3], None)
    assert _raises(None, [90.0])


# ---------------- 표 생성: 거부 (Davidson 결정 4 — 출발 전 raise)
def test_reject_length_mismatch():
    assert _raises([0, 1], [120.0])
    assert _raises([0], [120.0, 90.0])


def test_reject_duplicate_mode():
    assert _raises([3, 3], [90.0, 120.0])


def test_reject_mode_outside_valid():
    assert 4 not in VALID_MODES
    assert _raises([4], [120.0])
    assert _raises([70], [120.0])


def test_reject_non_int_mode():
    assert _raises([3.0], [90.0])
    assert _raises([True], [90.0])
    assert _raises(["3"], [90.0])


def test_reject_zero_and_negative():
    assert _raises([3], [0.0])
    assert _raises([3], [-1.0])


def test_reject_nan_and_inf():
    assert _raises([3], [float('nan')])
    assert _raises([3], [math.inf])


def test_reject_non_number_sec():
    assert _raises([3], ["90"])
    assert _raises([3], [None])
    assert _raises([3], [True])


# ---------------- timeout_for_mode
def test_timeout_for_mode_present():
    t = _default_table()
    assert timeout_for_mode(t, 3) == 90.0
    assert timeout_for_mode(t, 7) == 150.0


def test_timeout_for_mode_absent_is_none():
    """표에 없는 모드 = 타임아웃 없음(None) — 0 이 아니다 (0 이면 틱마다 WP 를 건너뛴다)."""
    t = _default_table()
    assert timeout_for_mode(t, 8) is None
    assert timeout_for_mode({}, 0) is None


# ---------------- 회귀: 현재 waypoints.yaml + 기본값
def test_regression_real_waypoints():
    """2026 규정 순서 16개: 게이트 2 → 위치유지 1 → 도킹 네 줄(접근 10m·6m mode 5, 입구·이탈 mode 7)
    → 탐색 접근(5) → 탐색 3 → 위치유지 2 접근(5) → 위치유지 2 → 회피 시작·끝 → WP10 종료(mode 8)."""
    wps = _real_waypoints()
    assert len(wps) == 16, f"waypoints.yaml 가 16개가 아니다 ({len(wps)}) — 회귀 기준 재확인 필요"
    assert [w[2] for w in wps] == [0, 1, 2, 5, 5, 7, 7, 5, 3, 3, 3, 5, 2, 5, 5, 8], [w[2] for w in wps]
    got = per_wp_timeouts(wps, _default_table())
    assert got[:3] == [120.0, 120.0, 120.0], got           # 게이트 0·1, 위치유지 1
    assert got[3] == 120.0 and got[4] == 120.0, got        # 도킹 접근 10m·6m(mode 5)
    assert got[5] == 150.0 and got[6] == 150.0, got        # WP8 입구·WP9 이탈(mode 7)
    assert got[7] == 120.0, got                            # 탐색 접근(mode 5)
    assert got[8:11] == [90.0, 90.0, 90.0], got            # 탐색 3(mode 3)
    assert got[11] == 120.0 and got[12] == 120.0, got      # 위치유지 2 접근(5)·위치유지 2(2)
    assert got[13] == 120.0 and got[14] == 120.0, got      # 회피 시작·끝(mode 5)
    assert got[15] is None, got                            # WP10 종료(mode 8): 없음


def test_real_waypoints_dock_exit_is_near_approach():
    """WP9 이탈점 = 가까운 접근점(6 m) 좌표, WP8 입구와는 다른 좌표(같으면 도킹 뒤 칸으로 재진입)."""
    wps = _real_waypoints()
    j = _first_dock(wps)
    near, entr, exit_ = wps[j - 1], wps[j], wps[j + 1]
    assert exit_[:2] == near[:2], (exit_, near)
    assert exit_[:2] != entr[:2], (exit_, entr)


def _first_dock(wps):
    """첫 mode 7(WP8 입구) 순번 — 도킹 네 줄은 그 앞 두 줄(접근 10m·6m)과 뒤 한 줄(WP9 이탈). 순서가 바뀌어도 따라간다."""
    return next(i for i, w in enumerate(wps) if w[2] == 7)


def _xy(lat0, lon0, lat, lon):
    """기준점 기준 동(x)·북(y) [m] (tools/dock_wp.py to_xy 와 같은 평면 근사)."""
    r = 6371000.0
    return (math.radians(lon - lon0) * r * math.cos(math.radians(lat0)),
            math.radians(lat - lat0) * r)


def _dock_geometry_errors(wps, far=10.0, near=6.0, tol_m=0.2, tol_deg=1.0):
    """도킹 네 줄(첫 mode 7 앞 두 줄 접근점, 첫 mode 7 WP8 입구, 그 뒤 WP9 이탈) 기하 불변식 위반 목록(빈 목록 = 통과).

    dock_wp.py 가 만든 줄이면 현장 교체 뒤에도 성립한다:
      · 접근점이 WP8 입구에서 far·near ± tol_m
      · WP8→접근 10m 과 WP8→접근 6m 방위 차 ≤ tol_deg (도크 법선 한 직선 위)
      · 접근점이 물 쪽: WP8→접근점 방위와 WP8→도킹 직전 WP(배가 지나온 물 위 점 — 2026 순서는 위치유지 1) 방위 차 < 90°
        (L·R 을 뒤집어 접근점이 육지 쪽으로 나온 경우를 잡는다)
    """
    errs = []
    j = _first_dock(wps)
    w7, wf, wn, w8 = wps[j - 3], wps[j - 2], wps[j - 1], wps[j]
    lat0, lon0 = w8[0], w8[1]
    pts = {k: _xy(lat0, lon0, w[0], w[1]) for k, w in (('wp7', w7), ('far', wf), ('near', wn))}
    for k, want in (('far', far), ('near', near)):
        d = math.hypot(*pts[k])
        if abs(d - want) > tol_m:
            errs.append(f"접근 {k} 거리 {d:.2f} m ≠ {want} ± {tol_m}")

    def brg(k):
        return math.degrees(math.atan2(pts[k][0], pts[k][1]))

    def diff(a, b):
        return abs((a - b + 180.0) % 360.0 - 180.0)
    if diff(brg('far'), brg('near')) > tol_deg:
        errs.append(f"접근점 방위 차 {diff(brg('far'), brg('near')):.2f}° > {tol_deg}°")
    if diff(brg('near'), brg('wp7')) >= 90.0:
        errs.append(f"접근점이 도킹 직전 WP 반대(육지) 쪽: 방위 차 {diff(brg('near'), brg('wp7')):.1f}°")
    return errs


def test_real_waypoints_dock_geometry():
    """기본 yaml(자리표시값)의 도킹 네 줄이 dock_wp.py 기하를 만족한다."""
    assert _dock_geometry_errors(_real_waypoints()) == []


def test_dock_geometry_catches_bad_rows():
    """불변식이 잘못된 좌표를 실제로 잡는지: 육지 쪽(방향 180° 뒤집힘)·거리 틀림·한 직선 밖."""
    wps = [list(w) for w in _real_waypoints()]
    j = _first_dock(wps)                     # WP8 입구. 접근 10m = j-2, 6m = j-1, WP9 이탈 = j+1
    F, N, X = j - 2, j - 1, j + 1
    w8 = wps[j]

    def mirror(w):          # WP8 기준 점대칭 = L·R 뒤집어 법선이 180° 돈 경우
        return [2 * w8[0] - w[0], 2 * w8[1] - w[1]] + list(w[2:])
    flipped = list(wps)
    flipped[F], flipped[N], flipped[X] = mirror(wps[F]), mirror(wps[N]), mirror(wps[X])
    assert any("육지" in e for e in _dock_geometry_errors(flipped)), _dock_geometry_errors(flipped)

    far_wrong = list(wps)
    f = wps[F]                              # 10 m 점을 WP8 에서 1.5 배(15 m)로
    far_wrong[F] = [w8[0] + (f[0] - w8[0]) * 1.5, w8[1] + (f[1] - w8[1]) * 1.5] + f[2:]
    assert any("far" in e for e in _dock_geometry_errors(far_wrong))

    swapped = list(wps)                     # 10m·6m 순서 뒤바뀜
    swapped[F], swapped[N] = wps[N], wps[F]
    errs = _dock_geometry_errors(swapped)
    assert any("far" in e for e in errs) and any("near" in e for e in errs), errs

    skew = list(wps)                        # 6 m 점을 옆으로 1 m(≈ 9.5°) 밀기
    dlon_1m = math.degrees(1.0 / (6371000.0 * math.cos(math.radians(w8[0]))))
    skew[N] = [wps[N][0], wps[N][1] + dlon_1m] + wps[N][2:]
    assert any("방위 차" in e for e in _dock_geometry_errors(skew)), _dock_geometry_errors(skew)


def test_follows_mode_not_index():
    """순서를 섞어도 타임아웃은 모드를 따라간다."""
    wps = [[35.1, 128.5, m, 3.0] for m in (8, 3, 0, 7, 5, 3, 2, 1)]
    got = per_wp_timeouts(wps, _default_table())
    assert got == [None, 90.0, 120.0, 150.0, 120.0, 90.0, 120.0, 120.0], got


# ---------------- timeout ≤ dwell 검출 (부팅 WARN 대상)
def test_dwell_conflicts_none_for_real():
    assert dwell_conflicts(_real_waypoints(), _default_table()) == []


def test_dwell_conflicts_detected():
    table = build_timeout_table([5, 7], [50.0, 150.0])
    wps = [[35.1, 128.5, 5, 50.0],     # timeout == dwell → 충돌
           [35.1, 128.5, 5, 10.0],     # 여유 있음
           [35.1, 128.5, 7, 200.0],    # timeout < dwell → 충돌
           [35.1, 128.5, 8, 999.0]]    # 타임아웃 없음 → 충돌 아님
    assert dwell_conflicts(wps, table) == [(0, 5, 50.0, 50.0), (2, 7, 150.0, 200.0)]


# ---------------- yaml: 타입·길이·코드 기본값 일치 (Ubuntu 10-29 실측: 정수·섞임이면 노드 사망)
def test_yaml_sec_all_float():
    secs = _yaml_params()['wp_timeout_sec']
    bad = [v for v in secs if type(v) is not float]
    assert not bad, f"wp_timeout_sec 에 float 아닌 원소 {bad} — 전부 .0 으로 적어라: {secs}"


def test_yaml_modes_all_int():
    modes = _yaml_params()['wp_timeout_modes']
    bad = [v for v in modes if type(v) is not int]
    assert not bad, f"wp_timeout_modes 에 int 아닌 원소 {bad}: {modes}"


def test_yaml_valid_and_matches_code_default():
    p = _yaml_params()
    build_timeout_table(p['wp_timeout_modes'], p['wp_timeout_sec'])   # 거부되면 예외
    assert p['wp_timeout_modes'] == DEFAULT_WP_TIMEOUT_MODES, p['wp_timeout_modes']
    assert p['wp_timeout_sec'] == DEFAULT_WP_TIMEOUT_SEC, p['wp_timeout_sec']


def test_code_default_types():
    assert all(type(v) is int for v in DEFAULT_WP_TIMEOUT_MODES)
    assert all(type(v) is float for v in DEFAULT_WP_TIMEOUT_SEC)


def test_node_declares_with_code_defaults():
    """노드가 이 모듈의 기본값으로 선언하는지 (대회 launch 가 yaml 을 못 실은 경우 대비)."""
    src = open(_NODE_SRC, encoding='utf-8').read()
    assert re.search(r"declare_parameter\('wp_timeout_modes',\s*list\(DEFAULT_WP_TIMEOUT_MODES\)\)", src)
    assert re.search(r"declare_parameter\('wp_timeout_sec',\s*list\(DEFAULT_WP_TIMEOUT_SEC\)\)", src)
    assert 'TURN_TIMEOUT' not in src and 'DEFAULT_TIMEOUT' not in src, "옛 순번 규칙 상수가 남아 있다"


def main():
    print("=== wp_timeout 테스트 ===")
    for n, f in sorted(globals().items()):
        if n.startswith("test_") and callable(f):
            check(n[5:], f)
    print(f"\n{_p}/{_t} 통과")
    return 0 if _p == _t else 1


if __name__ == "__main__":
    sys.exit(main())
