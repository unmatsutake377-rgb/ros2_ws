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
    wps = _real_waypoints()
    assert len(wps) == 11, f"waypoints.yaml 가 11개가 아니다 ({len(wps)}) — 회귀 기준 재확인 필요"
    got = per_wp_timeouts(wps, _default_table())
    # WP0~6 은 옛 순번 규칙과 같아야 한다
    assert got[:7] == [_old_rule(i) for i in range(7)] == [120.0, 120.0, 120.0, 90.0, 90.0, 90.0, 120.0], got
    assert got[7] == 120.0, got          # WP7 회피 끝(mode 5): 옛 규칙 None → 120
    assert got[8] == 150.0 and got[9] == 150.0, got   # WP8·9 도킹(mode 7)
    assert got[10] is None, got          # WP10 토너먼트(mode 8): 없음


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
