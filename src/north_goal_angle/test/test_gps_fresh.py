#!/usr/bin/env python3
"""GPS fix 신선도 순수 로직 테스트. python3 src/north_goal_angle/test/test_gps_fresh.py

계약서 docs/문제와작업/계약서_GPS묶음_20261029.md C1 — /fix 가 끊기거나 GPSFilter 가 계속 거부해도
have_fix 가 True 로 남아 옛 위치로 tp·goal_distance·geofence 를 내던 구멍을 막는다.
(test_gps_guard.py 는 스크립트 방식 n=18 고정이라 건드리지 않고 이 파일에 둔다.)
"""

import ast
import math
import os
import sys

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from north_goal_angle.gps_guard import (  # noqa: E402
    DEFAULT_GPS_STALE_SEC, fix_is_fresh, sanitize_stale_sec)

_PKG = os.path.join(os.path.dirname(__file__), '..')
_PARAMS_YAML = os.path.join(_PKG, 'config', 'north_goal_angle.yaml')
_NODE_SRC = os.path.join(_PKG, 'north_goal_angle', 'north_goal_angle.py')

T0 = 1000.0          # 임의의 monotonic 기준 시각
S = DEFAULT_GPS_STALE_SEC


# ---------------- fix_is_fresh
def test_default_is_2_5_float():
    assert DEFAULT_GPS_STALE_SEC == 2.5 and type(DEFAULT_GPS_STALE_SEC) is float


def test_never_had_fix_is_not_fresh():
    assert fix_is_fresh(None, T0, S) is False


def test_just_received_is_fresh():
    assert fix_is_fresh(T0, T0, S) is True


def test_boundary_equal_is_fresh():
    assert fix_is_fresh(T0, T0 + S, S) is True


def test_boundary_plus_eps_is_stale():
    assert fix_is_fresh(T0, T0 + S + 1e-6, S) is False


def test_one_missed_frame_at_1hz_is_fresh():
    # 1 Hz 에서 한 프레임 빠짐 → 마지막 수용 2.0 s 전. 견뎌야 한다(거짓 정지 금지).
    assert fix_is_fresh(T0, T0 + 2.0, S) is True


def test_two_missed_frames_at_1hz_is_stale():
    assert fix_is_fresh(T0, T0 + 3.0, S) is False


def test_long_gap_is_stale():
    assert fix_is_fresh(T0, T0 + 60.0, S) is False


def test_nan_inputs_are_not_fresh():
    assert fix_is_fresh(float('nan'), T0, S) is False
    assert fix_is_fresh(T0, float('nan'), S) is False
    assert fix_is_fresh(T0, T0, float('nan')) is False


def test_garbage_inputs_are_not_fresh():
    assert fix_is_fresh('x', T0, S) is False
    assert fix_is_fresh(T0, None, S) is False


# ---------------- sanitize_stale_sec
def test_sanitize_normal_passes_through():
    assert sanitize_stale_sec(2.5) == (2.5, True)
    assert sanitize_stale_sec(4.0) == (4.0, True)
    assert sanitize_stale_sec(0.1) == (0.1, True)


def test_sanitize_int_ok_as_float():
    # 🚨 순수 함수 호출만 본다. 노드에선 정수(0, 3)를 주면 DOUBLE 선언과 타입이 달라 Humble rclpy 가
    #    InvalidParameterTypeException 으로 노드를 세운다 — sanitize 까지 오지 않는다. yaml 은 반드시 소수(.0).
    sec, ok = sanitize_stale_sec(3)
    assert ok and sec == 3.0 and type(sec) is float


def test_sanitize_rejects_inf():
    # Davidson 10-29: inf("검사 끔")도 잘못된 값 → ERROR + 2.5
    assert sanitize_stale_sec(float('inf')) == (2.5, False)
    assert sanitize_stale_sec(float('-inf')) == (2.5, False)


def test_sanitize_rejects_zero_and_negative():
    assert sanitize_stale_sec(0.0) == (2.5, False)
    assert sanitize_stale_sec(0) == (2.5, False)
    assert sanitize_stale_sec(-1.0) == (2.5, False)


def test_sanitize_rejects_nan():
    sec, ok = sanitize_stale_sec(float('nan'))
    assert sec == 2.5 and ok is False


def test_sanitize_rejects_non_number():
    assert sanitize_stale_sec('2.5') == (2.5, False)
    assert sanitize_stale_sec('abc') == (2.5, False)
    assert sanitize_stale_sec(None) == (2.5, False)
    assert sanitize_stale_sec(True) == (2.5, False)


def test_sanitize_result_always_usable():
    for v in (float('inf'), 0.0, -3.0, float('nan'), 'x', None, 2.5, 7):
        sec, _ = sanitize_stale_sec(v)
        assert math.isfinite(sec) and sec > 0.0, (v, sec)


# ---------------- yaml ↔ 코드 기본값 (대회 launch 는 parameters= 없이 띄운다 → 코드 기본값이 대회값)
def test_yaml_value_is_float_and_matches_code_default():
    with open(_PARAMS_YAML, encoding='utf-8') as f:
        p = yaml.safe_load(f)['/north_goal_angle']['ros__parameters']
    v = p['gps_stale_sec']
    assert type(v) is float, f"gps_stale_sec 는 소수로 적어라(2.5): {v!r}"
    assert v == DEFAULT_GPS_STALE_SEC, (v, DEFAULT_GPS_STALE_SEC)
    assert sanitize_stale_sec(v) == (v, True)


# ---------------- 노드 소스 정적 검사 (rclpy 없이 ast 로)
def _node_methods():
    tree = ast.parse(open(_NODE_SRC, encoding='utf-8').read())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'NorthGoalAngle')
    return {f.name: f for f in cls.body if isinstance(f, ast.FunctionDef)}


def _self_assigns(fn, attr):
    """fn 안에서 self.<attr> = ... 하는 줄 번호들."""
    lines = []
    for node in ast.walk(fn):
        targets = []
        if isinstance(node, ast.Assign):
            targets = node.targets
        elif isinstance(node, (ast.AugAssign, ast.AnnAssign)):
            targets = [node.target]
        for t in targets:
            for e in ast.walk(t):
                if (isinstance(e, ast.Attribute) and e.attr == attr
                        and isinstance(e.value, ast.Name) and e.value.id == 'self'):
                    lines.append(node.lineno)
    return lines


def test_have_fix_only_changed_in_gps_cb():
    # 🚨 have_fix 를 타이머에서 바꾸지 말 것(계약 §6-4). stale 은 별도 상태.
    bad = {name: ln for name, fn in _node_methods().items()
           if name not in ('__init__', 'gps_cb') and (ln := _self_assigns(fn, 'have_fix'))}
    assert not bad, f"have_fix 를 gps_cb 밖에서 바꾼다: {bad}"


def test_last_fix_t_only_set_on_accepted_frame():
    methods = _node_methods()
    bad = {name: ln for name, fn in methods.items()
           if name not in ('__init__', 'gps_cb') and (ln := _self_assigns(fn, 'last_fix_t'))}
    assert not bad, f"last_fix_t 를 gps_cb 밖에서 바꾼다: {bad}"
    cb = methods['gps_cb']
    sets = _self_assigns(cb, 'last_fix_t')
    assert len(sets) == 1, sets
    # GPSFilter 거부 분기(if not accepted: ... return) 뒤에서만 갱신해야 한다.
    reject = [n.lineno for n in ast.walk(cb) if isinstance(n, ast.If)
              and isinstance(n.test, ast.UnaryOp) and isinstance(n.test.op, ast.Not)
              and isinstance(n.test.operand, ast.Name) and n.test.operand.id == 'accepted']
    assert len(reject) == 1, reject
    assert sets[0] > reject[0], (sets, reject)


def test_param_declared_with_code_default_before_timer():
    src = open(_NODE_SRC, encoding='utf-8').read()
    decl = src.find("declare_parameter('gps_stale_sec', DEFAULT_GPS_STALE_SEC)")
    timer = src.find("self.create_timer(")
    assert decl != -1, "gps_stale_sec 를 DEFAULT_GPS_STALE_SEC 로 선언하지 않는다"
    assert timer != -1 and decl < timer, "gps_stale_sec 선언이 타이머 생성보다 뒤다"
    assert 'sanitize_stale_sec(' in src


def test_timer_uses_freshness_not_bare_have_fix():
    # 위치에서 나오는 값(tp·dist·ILOS·geofence)이 have_fix 단독 조건으로 돌면 stale 이 샌다.
    m = _node_methods()
    for name in ('timer_cb', '_geofence_ranges'):
        for node in ast.walk(m[name]):
            if isinstance(node, ast.If):
                t = node.test
                bare = (isinstance(t, ast.Attribute) and t.attr == 'have_fix')
                assert not bare, f"{name}:{node.lineno} 가 'if self.have_fix:' 로 위치를 쓴다"


def main():
    print("=== gps_fresh 테스트 ===")
    p = t = 0
    for n, f in sorted(globals().items()):
        if n.startswith("test_") and callable(f):
            t += 1
            try:
                f()
                p += 1
                print(f"  ✅ {n[5:]}")
            except AssertionError as e:
                print(f"  ❌ {n[5:]}\n     {e}")
    print(f"\n{p}/{t} 통과")
    return 0 if p == t else 1


if __name__ == "__main__":
    sys.exit(main())
