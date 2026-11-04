#!/usr/bin/env python3
"""wp_advance 순수 로직 테스트. python3 src/north_goal_angle/test/test_wp_advance.py

계약서 docs/문제와작업/계약서_전체코스_20261104.md C2 §2-2 — dwell 은 담당 노드가 손 뗀 동안만,
시간 초과 진행은 그대로, 코스 종료 후 candidate.
"""

import math
import os
import re
import sys

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from north_goal_angle.wp_advance import (  # noqa: E402
    CANDIDATE_RELEASED, DEFAULT_DWELL_WAIT_ENABLED, DEFAULT_DWELL_WAIT_MODES,
    DEFAULT_DWELL_WAIT_STALE_SEC, candidate_after_course, dwell_may_count,
    sanitize_wait_modes, wp_step,
)

_PKG = os.path.join(os.path.dirname(__file__), '..')
_PARAMS_YAML = os.path.join(_PKG, 'config', 'north_goal_angle.yaml')
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


def _may(mode, cand, cand_t, now, enabled=True, modes=(2, 3), stale=1.0):
    return dwell_may_count(enabled, mode, modes, cand, cand_t, now, stale)


def _yaml_params():
    with open(_PARAMS_YAML, encoding='utf-8') as f:
        return yaml.safe_load(f)['/north_goal_angle']['ros__parameters']


# ---------------- dwell_may_count
def test_fresh_20000_counts():
    assert _may(2, 20000.0, 10.0, 10.5) is True
    assert _may(3, 20000.0, 10.0, 11.0) is True          # 경계 1.0 s = 신선


def test_fresh_non_20000_waits():
    """ship_back HOLD 조향(30°)·STOP_HOLD(50000)·후진(180) → 세지 않음."""
    for v in (30.0, 50000.0, 180.0, 5000.0, 0.0):
        assert _may(2, v, 10.0, 10.5) is False, v
        assert _may(3, v, 10.0, 11.0) is False, v


def test_stale_counts():
    """1.0 s 넘게 무수신 = 담당 노드 사망 → 기다려도 소용없다 → 센다."""
    assert _may(2, 50000.0, 10.0, 11.01) is True
    assert _may(3, 30.0, 10.0, 30.0) is True


def test_never_received_counts():
    assert _may(2, None, None, 5.0) is True


def test_other_modes_unaffected():
    """mode 5(north 담당)·7(도킹)·0·1·8 은 대상 아님 → 신선한 비-20000 이어도 센다."""
    for m in (0, 1, 5, 7, 8):
        assert _may(m, 50000.0, 10.0, 10.1) is True, m


def test_disabled_counts():
    assert _may(2, 50000.0, 10.0, 10.1, enabled=False) is True


def test_nan_waits():
    """NaN 을 '손 뗌'으로 보지 않는다(모르면 대기, 시간 초과가 진행시킨다)."""
    assert _may(2, float('nan'), 10.0, 10.1) is False


# ---------------- wp_step: 기존 동작 유지
def test_step_arrive_after_dwell():
    ev, ts, rem = wp_step(0.0, 0.0, 120.0, 1.0, 2.5, None, 3.0, True)
    assert ev is None and ts == 0.0 and rem == 120.0
    ev, ts, _ = wp_step(2.0, 0.0, 120.0, 1.0, 2.5, ts, 3.0, True)
    assert ev is None and ts == 0.0
    ev, ts, _ = wp_step(3.0, 0.0, 120.0, 1.0, 2.5, ts, 3.0, True)
    assert ev == 'arrive' and ts is None


def test_step_outside_or_no_fix_resets():
    assert wp_step(5.0, 0.0, 120.0, 3.0, 2.5, 1.0, 3.0, True)[:2] == (None, None)
    assert wp_step(5.0, 0.0, 120.0, None, 2.5, 1.0, 3.0, True)[:2] == (None, None)


def test_step_no_timeout_mode():
    ev, ts, rem = wp_step(9999.0, 0.0, None, 10.0, 2.5, None, 60.0, True)
    assert ev is None and rem is None


def test_step_wait_resets_t_start():
    """대기 중에는 t_start 를 비운다 → 손 뗀 뒤부터 dwell 연속 시간을 다시 잰다."""
    ev, ts, _ = wp_step(10.0, 0.0, 120.0, 1.0, 2.5, 4.0, 3.0, False)
    assert ev is None and ts is None


# ---------------- 시나리오: mode 2 위치유지 중 north 가 넘기지 않음
def _run(cands, t_end, timeout=120.0, dwell=3.0, mode=2, period=0.5, dist=2.4):
    """cands: [(t_from, value or None)] — 그 시각부터 10 Hz 로 그 값 수신(None=발행 정지).
    north 틱(period)마다 판정. → (event, t) 또는 (None, t_end)."""
    last, last_t, ts = None, None, None
    k = 0
    while True:
        now = k * 0.1
        if now > t_end:
            return None, t_end
        cur = None
        for t0, v in cands:
            if now >= t0:
                cur = v
        if cur is not None:
            last, last_t = cur, now
        if k % int(round(period / 0.1)) == 0:
            may = _may(mode, last, last_t, now)
            ev, ts, _ = wp_step(now, 0.0, timeout, dist, 2.5, ts, dwell, may)
            if ev is not None:
                return ev, round(now, 1)
        k += 1


def test_scenario_hold_then_done():
    """INIT·SEARCH 20000(0~1 s) → HOLD 비-20000(1~40 s) → DONE 20000: DONE 뒤 dwell 3 s 에 도착."""
    ev, t = _run([(0.0, 20000.0), (1.0, 50000.0), (40.0, 20000.0)], 200.0)
    assert ev == 'arrive', ev
    assert 43.0 <= t <= 44.0, f"HOLD 도중(<40 s) 넘기면 안 되고 DONE 뒤 3 s 쯤 도착: {t}"


def test_scenario_old_behaviour_would_cut():
    """같은 입력을 대기 없이(enabled False) 판정하면 HOLD 도중 3 s 만에 넘긴다(결함 재현)."""
    ts, t_arr = None, None
    for k in range(0, 400, 5):
        now = k * 0.1
        ev, ts, _ = wp_step(now, 0.0, 120.0, 2.4, 2.5, ts, 3.0,
                            dwell_may_count(False, 2, (2, 3), 50000.0, now, now, 1.0))
        if ev == 'arrive':
            t_arr = now
            break
    assert t_arr is not None and t_arr < 40.0, t_arr


def test_scenario_node_dies_counts_after_stale():
    """HOLD 중 담당 노드가 죽으면(발행 정지 10 s) 1.0 s 뒤부터 dwell → 약 14 s 에 도착."""
    ev, t = _run([(0.0, 50000.0), (10.0, None)], 100.0)
    assert ev == 'arrive', ev
    assert 13.0 <= t <= 15.0, t


# ---------------- 시간 초과는 dwell 대기와 무관 (계약서 C2-3)
def test_timeout_not_blocked_by_wait():
    """담당 노드가 끝까지 손을 안 떼도(신선한 50000) 시간 초과 120 s 에 넘어간다."""
    ev, t = _run([(0.0, 50000.0)], 300.0, timeout=120.0)
    assert ev == 'timeout', ev
    assert abs(t - 120.0) < 0.6, t


def test_timeout_wins_same_tick():
    """dwell 완료와 시간 초과가 같은 틱이면 시간 초과(옛 timer_cb 순서)."""
    ev, ts, rem = wp_step(120.0, 0.0, 120.0, 1.0, 2.5, 100.0, 3.0, True)
    assert ev == 'timeout' and ts is None and rem == 0.0


def test_timeout_without_fix_and_waiting():
    ev, _, _ = wp_step(150.0, 0.0, 150.0, None, 2.5, None, 60.0, False)
    assert ev == 'timeout'


# ---------------- 파라미터 검증
def test_sanitize_wait_modes():
    assert sanitize_wait_modes([2, 3]) == ((2, 3), True)
    assert sanitize_wait_modes((3,)) == ((3,), True)
    for bad in ([], None, [2, 4], [2, 'x'], [2.0], [True], 5):
        modes, ok = sanitize_wait_modes(bad)
        assert ok is False and modes == tuple(DEFAULT_DWELL_WAIT_MODES), bad


# ---------------- 코스 종료 후 candidate (C1 리뷰 권고)
def test_after_course_fallback_last_mode():
    assert candidate_after_course(8, (5, 8)) == CANDIDATE_RELEASED
    assert candidate_after_course(5, (5, 8)) == CANDIDATE_RELEASED


def test_after_course_owned_last_mode_silent():
    """마지막 모드에 담당 노드가 있으면 north 는 내지 않는다(발행자 둘 금지)."""
    for m in (0, 1, 2, 3, 7):
        assert candidate_after_course(m, (5, 8)) is None, m


# ---------------- 코드 기본값 = yaml, 타입
def test_code_default_types():
    assert type(DEFAULT_DWELL_WAIT_ENABLED) is bool
    assert DEFAULT_DWELL_WAIT_MODES and all(type(v) is int for v in DEFAULT_DWELL_WAIT_MODES)
    assert type(DEFAULT_DWELL_WAIT_STALE_SEC) is float
    assert math.isfinite(DEFAULT_DWELL_WAIT_STALE_SEC) and DEFAULT_DWELL_WAIT_STALE_SEC > 0


def test_yaml_matches_code_default():
    p = _yaml_params()
    assert p['dwell_wait_enabled'] is DEFAULT_DWELL_WAIT_ENABLED, p['dwell_wait_enabled']
    assert p['dwell_wait_modes'] == DEFAULT_DWELL_WAIT_MODES, p['dwell_wait_modes']
    assert all(type(v) is int for v in p['dwell_wait_modes'])
    assert type(p['dwell_wait_stale_sec']) is float, "dwell_wait_stale_sec 는 소수(.0)로"
    assert p['dwell_wait_stale_sec'] == DEFAULT_DWELL_WAIT_STALE_SEC
    assert 7 not in p['dwell_wait_modes'], "mode 7(도킹)은 대상 아님"


def test_node_declares_with_code_defaults():
    src = open(_NODE_SRC, encoding='utf-8').read()
    assert re.search(
        r"declare_parameter\('dwell_wait_enabled',\s*DEFAULT_DWELL_WAIT_ENABLED\)", src)
    assert re.search(
        r"declare_parameter\(\s*'dwell_wait_modes',\s*list\(DEFAULT_DWELL_WAIT_MODES\)\)", src)
    assert re.search(
        r"declare_parameter\(\s*'dwell_wait_stale_sec',\s*DEFAULT_DWELL_WAIT_STALE_SEC\)", src)
    assert "'/candidate_angle', self.candidate_cb" in src, "north 가 /candidate_angle 을 구독 안 함"


def test_node_declares_before_timer_and_subscription():
    """파라미터 선언이 그 값을 쓰는 구독·타이머 생성보다 먼저."""
    src = open(_NODE_SRC, encoding='utf-8').read()
    i_decl = src.index("'dwell_wait_stale_sec'")
    assert i_decl < src.index("'/candidate_angle', self.candidate_cb")
    assert i_decl < src.index("self.create_timer(self.period_sec")


def main():
    print("=== wp_advance 테스트 ===")
    for n, f in sorted(globals().items()):
        if n.startswith("test_") and callable(f):
            check(n[5:], f)
    print(f"\n{_p}/{_t} 통과")
    return 0 if _p == _t else 1


if __name__ == "__main__":
    sys.exit(main())
