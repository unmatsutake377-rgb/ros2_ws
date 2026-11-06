#!/usr/bin/env python3
"""(C) 갭 후보 정렬 — 편 유지 히스테리시스·가로지름 벌점 테스트.
실행: python3 src/ship_direction/test/test_gap_select.py

근거: docs/문제와작업/시뮬3자유도_발견_20261106.md 발견 8 / 계약서_회피편선택_20261106.md 검증 계획 ①~⑩.
순수 함수(gap_select.py)는 ROS 없이 바로 부르고, _compute·control_cb 는 test_candidate_fresh.py 와
같은 ROS 스텁 + 실제 __init__(Node 메서드만 잠깐 끼움)으로 만든 노드에서 부른다.
"""

import ast
import math
import os
import re
import sys
import types

import yaml

HERE = os.path.dirname(__file__)
PKG = os.path.join(HERE, '..')
sys.path.insert(0, PKG)

from ship_direction.gap_select import (  # noqa: E402
    obstacle_blobs, rank_candidates, next_side_memory)

# ---- ROS 모듈 최소 스텁 (import 통과용, 로직엔 안 쓰임) ----


def _stub(name, **attrs):
    m = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(m, k, v)
    sys.modules[name] = m
    return m


class _Node:                       # class ShipDirection(Node) 상속용
    pass


def _msg(*a, **k):
    return types.SimpleNamespace(data=k.get('data', a[0] if a else None))


if 'ship_direction.ship_direction' not in sys.modules:
    _pol = types.SimpleNamespace(BEST_EFFORT=1, RELIABLE=2, KEEP_LAST=1)
    _stub('rclpy', init=None, ok=lambda: True, shutdown=None)
    _stub('rclpy.node', Node=_Node)
    _stub('rclpy.executors', MultiThreadedExecutor=object)
    _stub('rclpy.qos',
          QoSProfile=lambda **k: types.SimpleNamespace(**k),
          ReliabilityPolicy=_pol, HistoryPolicy=_pol)
    _stub('std_msgs', msg=None)
    _stub('std_msgs.msg', Int32=_msg, Float32=_msg, Float32MultiArray=_msg)
    _stub('sensor_msgs', msg=None)
    _stub('sensor_msgs.msg', LaserScan=_msg)

import ship_direction.ship_direction as SD  # noqa: E402
from ship_direction.ship_direction import (  # noqa: E402
    ShipDirection, CANDIDATE_INVALID, STOP_HOLD, LIDAR_FORWARD_DEG)

FWD = LIDAR_FORWARD_DEG
HYS = 40.0           # side_hysteresis_deg 코드 기본값(=yaml)
CROSS = 90.0         # cross_penalty_deg 코드 기본값(=yaml)

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


def C(diff, angle, arc=1.0, s=0, e=0):
    """_compute 후보 튜플 형식 (angle_diff, -arc_len, angle, s, e)."""
    return (diff, -arc, angle, s, e)


def angles(ranked):
    return [x[2] for x in ranked]


# ======================================================================
# 순수 함수
# ======================================================================
def test_blobs_center_and_nearest():
    binary = [0, 1, 1, 1, 0, 0, 1, 0]
    ang = [10.0 * i for i in range(8)]
    dist = [9.0, 2.5, 1.9, math.nan, 9.0, 9.0, math.inf, 9.0]
    b = obstacle_blobs(binary, ang, dist)
    assert len(b) == 2, b
    assert b[0] == (20.0, 1.9), b                      # (10+30)/2, nan 무시
    assert b[1][0] == 60.0 and math.isinf(b[1][1]), b  # 전부 비유한 → inf (지어내지 않음)
    assert obstacle_blobs([], [], []) == []
    assert obstacle_blobs([1, 1], [0.0, 2.0], [1.0, 2.0]) == [(1.0, 1.0)]   # 끝까지 이어진 구간


def test_zero_penalty_matches_old_sort():
    """① 벌점 0/0 → 기존 (x[0], x[1]) 정렬과 같은 순서(동점·arc 둘째 키 포함)."""
    cands = [C(5.0, 100.0, 2.0), C(3.0, 60.0, 1.0), C(5.0, 110.0, 3.0), C(0.0, 85.0, 0.5),
             C(3.0, 40.0, 1.0, 1, 1)]
    old = sorted(cands, key=lambda x: (x[0], x[1]))
    obs = [(90.0, 1.0), (70.0, 2.0)]
    for prev in (None, 50.0, 120.0):
        got = rank_candidates(cands, prev, obs, 0.0, 0.0, FWD)
        assert got == old, (prev, got, old)


def test_input_not_mutated():
    cands = [C(5.0, 100.0), C(1.0, 60.0)]
    snap = list(cands)
    out = rank_candidates(cands, 60.0, [(90.0, 1.0)], HYS, CROSS, FWD)
    assert cands == snap and out is not cands


def test_cross_left_obstacle_picks_right():
    """② 정면 7° 왼쪽(중심 87, 1.9 m) + 목표 왼쪽(95) → 오른쪽 후보(60)."""
    yaw = 95.0
    cands = [C(abs(100.0 - yaw), 100.0), C(abs(60.0 - yaw), 60.0)]
    obs = [(87.0, 1.9)]
    assert angles(rank_candidates(cands, None, obs, 0.0, 0.0, FWD))[0] == 100.0   # 기존: 왼쪽
    assert angles(rank_candidates(cands, None, obs, HYS, CROSS, FWD))[0] == 60.0


def test_hysteresis_keeps_side():
    """③ 반대편이 0.2° 더 가까워도 직전 편(오른쪽) 유지."""
    cands = [C(22.4, 102.4), C(22.6, 57.4)]
    assert angles(rank_candidates(cands, None, [], HYS, CROSS, FWD))[0] == 102.4
    assert angles(rank_candidates(cands, 60.0, [], HYS, CROSS, FWD))[0] == 57.4
    assert angles(rank_candidates(cands, 100.0, [], HYS, CROSS, FWD))[0] == 102.4
    # 히스테리시스보다 크게 차이 나면 편을 바꾼다
    far = [C(1.0, 102.4), C(45.0, 57.4)]
    assert angles(rank_candidates(far, 60.0, [], HYS, 0.0, FWD))[0] == 102.4


def test_single_candidate_no_memory():
    """④ 후보 1개 → 미적용, 다음 기억 None. 후보 0개 → 빈 리스트·None."""
    one = [C(3.0, 100.0)]
    assert rank_candidates(one, 60.0, [], HYS, CROSS, FWD) == one
    assert next_side_memory(100.0, 1) is None
    assert next_side_memory(100.0, 0) is None
    assert next_side_memory(100.0, 2) == 100.0
    assert rank_candidates([], 60.0, [], HYS, CROSS, FWD) == []


def _first(cands, prev, obs):
    return rank_candidates(cands, prev, obs, HYS, CROSS, FWD)[0][2]


def test_boundaries_open_interval():
    """⑤ 경계: 후보 == 80, 중심 == 후보각, 중심 == 80, prev == 80 → 모두 무벌점."""
    ref = C(1.0, FWD)                         # 정면 기준 후보 — 언제나 무벌점
    # 후보 == 80 (prev 반대편·중심 80 이어도 곱 0·빈 구간)
    assert _first([C(0.0, FWD), C(10.0, 120.0)], 120.0, [(FWD, 1.0)]) == FWD
    # 중심 == 후보각 (열린 구간 밖)
    assert _first([C(0.0, 100.0), ref], None, [(100.0, 1.0)]) == 100.0
    # 중심 == 80 (열린 구간 밖)
    assert _first([C(0.0, 100.0), ref], None, [(FWD, 1.0)]) == 100.0
    # prev == 80 (곱 0)
    assert _first([C(0.0, 100.0), ref], FWD, []) == 100.0
    # 대조: 살짝 안쪽이면 벌점
    assert _first([C(0.0, 100.0), ref], None, [(99.9, 1.0)]) == FWD
    assert _first([C(0.0, 100.0), ref], None, [(80.1, 1.0)]) == FWD
    assert _first([C(0.0, 100.0), ref], 79.9, []) == FWD
    # 오른쪽 후보도 대칭
    assert _first([C(0.0, 60.0), ref], None, [(70.0, 1.0)]) == FWD
    assert _first([C(0.0, 60.0), ref], None, [(60.0, 1.0)]) == 60.0


def test_both_penalized_keeps_old_order():
    """⑥ 두 후보 모두 가로지름 벌점 → 기존 순서."""
    cands = [C(5.0, 110.0), C(2.0, 50.0)]
    obs = [(95.0, 1.0), (65.0, 1.0)]
    old = sorted(cands, key=lambda x: (x[0], x[1]))
    assert rank_candidates(cands, None, obs, HYS, CROSS, FWD) == old


def test_len_inside_sort_key_is_zero():
    """⑦ 결함 주입: 키 함수 안에서 len(candidates) 를 읽으면 정렬 중 0 → 히스테리시스가 꺼진다."""
    seen = []
    lst = [C(2.0, 100.0), C(1.0, 60.0)]
    lst.sort(key=lambda x: (seen.append(len(lst)), x[0])[1])
    assert seen and all(n == 0 for n in seen), seen     # CPython: 정렬 중 리스트는 비어 보인다

    def buggy_rank(cands, prev, hys):
        out = list(cands)
        out.sort(key=lambda x: (x[0] + (hys if len(out) >= 2 and prev is not None and
                                        (x[2] - FWD) * (prev - FWD) < 0 else 0.0), x[1]))
        return out

    cands = [C(22.4, 102.4), C(22.6, 57.4)]
    assert angles(buggy_rank(cands, 60.0, HYS))[0] == 102.4             # 결함: 편 바꿈
    assert angles(rank_candidates(cands, 60.0, [], HYS, 0.0, FWD))[0] == 57.4   # 올바른 구현


# ======================================================================
# 노드 수준: _compute 통합(⑩)·리셋 규칙(⑧)·파라미터(⑨)·AST
# ======================================================================
class _Logger:
    def __init__(self):
        self.lines = []

    def warn(self, msg, **kw):
        self.lines.append(('WARN', msg))

    def info(self, msg, **kw):
        self.lines.append(('INFO', msg))

    def error(self, msg, **kw):
        self.lines.append(('ERROR', msg))


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


class _Clock:
    def __init__(self, t=1000.0):
        self.t = t

    def monotonic(self):
        return self.t


def _ns(*a, **k):
    return types.SimpleNamespace(data=k.get('data', a[0] if a else None))


def _construct(overrides):
    """ShipDirection.__init__ 을 실제로 돌린다 — Node 메서드만 잠깐 끼운다."""
    base = ShipDirection.__bases__[0]
    log = _Logger()
    names = ('__init__', 'declare_parameter', 'create_subscription', 'create_publisher',
             'create_timer', 'get_logger')
    saved = {n: base.__dict__[n] for n in names if n in base.__dict__}
    base.__init__ = lambda self, name: None
    base.declare_parameter = lambda self, n, d: types.SimpleNamespace(value=overrides.get(n, d))
    base.create_subscription = lambda self, *a, **k: None
    base.create_publisher = lambda self, *a, **k: _Pub()
    base.create_timer = lambda self, *a, **k: None
    base.get_logger = lambda self: log
    try:
        node = ShipDirection()
    finally:
        for n in names:
            if n in saved:
                setattr(base, n, saved[n])
            elif n in base.__dict__:
                delattr(base, n)
    node.get_logger = lambda: log      # 실행 중 경고 경로용(인스턴스 속성)
    return node, log


def with_env(fn):
    """SD.time·메시지 생성자를 값 보존 스텁으로 바꿔 fn(clock) 실행 후 되돌린다.
    (pytest 한 프로세스에선 먼저 import 한 다른 테스트의 스텁이 모듈에 남아 있을 수 있다)"""
    keys = ('time', 'Float32', 'Float32MultiArray', 'Int32')
    real = {k: getattr(SD, k) for k in keys}
    clk = _Clock()
    SD.time = clk
    SD.Float32 = SD.Float32MultiArray = SD.Int32 = _ns
    try:
        return fn(clk)
    finally:
        for k, v in real.items():
            setattr(SD, k, v)


INC_DEG = 0.5


def scan(obstacles):
    """0~179.5° (0.5° 간격) 스캔. obstacles = [(lo_deg, hi_deg, r)] 구간 안 셀만 r, 나머지 inf."""
    n = 360
    ranges = [math.inf] * n
    for lo, hi, r in obstacles:
        for i in range(n):
            if lo <= i * INC_DEG <= hi:
                ranges[i] = r
    return types.SimpleNamespace(angle_min=0.0, angle_increment=math.radians(INC_DEG),
                                 ranges=ranges)


LEFT7 = scan([(82.5, 91.5, 1.9)])      # 정면 7° 왼쪽(중심 87) 부표 1.9 m
YAW_LEFT = 20.0                         # 목표 20° 왼쪽 → LiDAR 100


def test_compute_cross_picks_right():
    """⑩ _compute 통합: 정면 7° 왼쪽 부표 + 목표 왼쪽 → < 80 (오른쪽). 벌점 0/0 이면 왼쪽(기존)."""
    def run(clk):
        node, _ = _construct({})
        a, _ = node._compute(LEFT7, None, CANDIDATE_INVALID, YAW_LEFT, 3.0, None)
        assert a < FWD, a
        assert node._side_prev == a, (node._side_prev, a)       # 후보 2개 → 기억
        off, _ = _construct({'side_hysteresis_deg': 0.0, 'cross_penalty_deg': 0.0})
        b, _ = off._compute(LEFT7, None, CANDIDATE_INVALID, YAW_LEFT, 3.0, None)
        assert b > FWD, b
    with_env(run)


def _tick(node, clk, candidate=CANDIDATE_INVALID, yaw=YAW_LEFT, sc=LEFT7):
    clk.t += 0.1
    node.latest_scan = sc
    node.yaw_error = yaw
    node.candidate_angle = candidate
    node.candidate_t = clk.t
    ShipDirection.control_cb(node)
    return node.pub_desired_angle.msgs[-1].data if node.pub_desired_angle.msgs else None


def test_reset_rules_control_cb():
    """⑧ 리셋: (B)·(A)·STOP_HOLD·[N]·레벨2·후보 1개 틱 → None. (C) 를 타는 틱에서만 유지."""
    def run(clk):
        node, _ = _construct({})
        _tick(node, clk)
        right = node._side_prev
        assert right is not None and right < FWD, right
        for cand in (10.0, SD.SPIN_RIGHT, STOP_HOLD):           # (B)·(A)·(0)
            _tick(node, clk)
            assert node._side_prev == right
            _tick(node, clk, candidate=cand)
            assert node._side_prev is None, cand
        _tick(node, clk)
        assert node._side_prev is not None
        _tick(node, clk, yaw=None)                              # [N] → STOP_HOLD → (0)
        assert node._side_prev is None
        _tick(node, clk)
        node.failsafe_level = 2                                 # 레벨2 정지
        _tick(node, clk)
        assert node._side_prev is None
        node.failsafe_level = 0
        _tick(node, clk)
        assert node._side_prev is not None
        _tick(node, clk, sc=scan([]))                           # 장애물 없음 → 후보 1개
        assert node._side_prev is None
        _tick(node, clk)
        assert node._side_prev is not None
        _tick(node, clk, sc=None)                               # 스캔 없음(부팅 중) 틱
        assert node._side_prev is None
    with_env(run)


def test_reset_on_wp_mode_change():
    """⑧ wp_mode 가 바뀐 틱엔 _compute 가 기억 없이(None) 시작한다. 같은 모드면 유지."""
    def run(clk):
        node, _ = _construct({})
        seen = []
        real = node._compute

        def spy(*a):
            seen.append(node._side_prev)
            return real(*a)
        node._compute = spy
        _tick(node, clk)
        _tick(node, clk)
        assert seen[-1] is not None, seen                       # 같은 모드 → 직전 편 전달
        node.wp_mode = 5
        _tick(node, clk)
        assert seen[-1] is None, seen                           # 모드 변경 틱 → None
        _tick(node, clk)
        assert seen[-1] is not None, seen
    with_env(run)


def test_memory_is_before_rate_limit():
    """기억은 레이트 리미터 **전** 선택각."""
    def run(clk):
        node, _ = _construct({})
        node.prev_final_angle = 120.0
        out = _tick(node, clk)
        assert out == 120.0 - node.rate_limit_max_deg_per_tick, out
        assert node._side_prev < FWD and node._side_prev != out, (node._side_prev, out)
    with_env(run)


def _yaml():
    with open(os.path.join(PKG, 'config', 'ship_direction.yaml'), encoding='utf-8') as f:
        return yaml.safe_load(f)['/ship_direction']['ros__parameters']


def _src():
    with open(os.path.join(PKG, 'ship_direction', 'ship_direction.py'), encoding='utf-8') as f:
        return f.read()


def test_code_default_equals_yaml():
    """⑨ 코드 기본값 == yaml (켬 40.0/90.0, float)."""
    p = _yaml()
    node, log = _construct({})
    src = _src()
    for name, want in (('side_hysteresis_deg', HYS), ('cross_penalty_deg', CROSS)):
        assert isinstance(p[name], float), (name, p[name])
        assert getattr(node, name) == want == p[name], (name, getattr(node, name), p[name])
        m = re.search(r"declare_parameter\('%s', ([0-9.]+)\)" % name, src)
        assert m and float(m.group(1)) == p[name], name
    assert not [m for sv, m in log.lines if sv == 'WARN'], log.lines


def test_bad_param_falls_back():
    """⑨ nan/inf/음수 → 기본값 + WARN 1회. 0.0(끔)·양수는 그대로, WARN 없음."""
    for name, default in (('side_hysteresis_deg', HYS), ('cross_penalty_deg', CROSS)):
        for bad in (math.nan, math.inf, -math.inf, -1.0):
            node, log = _construct({name: bad})
            assert getattr(node, name) == default, (name, bad)
            w = [m for s, m in log.lines if s == 'WARN' and name in m]
            assert len(w) == 1, (name, bad, log.lines)
        for ok in (0.0, 25.0):
            node, log = _construct({name: ok})
            assert getattr(node, name) == ok, (name, ok)
            assert not [m for s, m in log.lines if s == 'WARN'], (name, ok, log.lines)


def test_source_uses_rank_candidates():
    """ship_direction.py 가 rank_candidates 를 실제로 호출하고, cross_dist 는 어디에도 없다(결정 1)."""
    src = _src()
    calls = [n for n in ast.walk(ast.parse(src))
             if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and
             n.func.id == 'rank_candidates']
    assert calls, "rank_candidates 호출 없음"
    with open(os.path.join(PKG, 'config', 'ship_direction.yaml'), encoding='utf-8') as f:
        y = f.read()
    with open(os.path.join(PKG, 'ship_direction', 'gap_select.py'), encoding='utf-8') as f:
        g = f.read()
    assert 'cross_dist' not in src and 'cross_dist' not in y
    assert 'def rank_candidates' in g
    assert 'candidates.sort(' not in src


def main():
    print("=== (C) 후보 정렬 편 유지·가로지름 테스트 (발견 8) ===")
    for n, f in sorted(globals().items()):
        if n.startswith("test_") and callable(f):
            check(n[5:], f)
    print(f"\n{_p}/{_t} 통과")
    return 0 if _p == _t else 1


if __name__ == "__main__":
    sys.exit(main())
