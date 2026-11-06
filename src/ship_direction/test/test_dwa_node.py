#!/usr/bin/env python3
"""(C) DWA 노드 통합 테스트 — 파라미터·가드·훅·로그 (계약서_DWA_20261106 검증 계획).

실행: tools/run_tests.sh ship_direction

test_side_escape.py 와 같은 ROS 스텁 + 실제 __init__(Node 메서드만 잠깐 끼움)으로 만든 노드의 control_cb 틱.
"""

import math
import os
import re
import sys
import types

import yaml

HERE = os.path.dirname(__file__)
PKG = os.path.join(HERE, '..')
sys.path.insert(0, PKG)

from ship_direction.dwa import DwaPlanner, INIT_STATE  # noqa: E402

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
    CANDIDATE_INVALID, REVERSE_ANGLE, ShipDirection, STOP_HOLD)


class _Logger:
    def __init__(self):
        self.lines = []

    def warn(self, msg, **kw):
        self.lines.append(('WARN', msg, kw))

    def info(self, msg, **kw):
        self.lines.append(('INFO', msg, kw))

    def error(self, msg, **kw):
        self.lines.append(('ERROR', msg, kw))

    def of(self, sev, word=''):
        return [(m, kw) for s, m, kw in self.lines if s == sev and word in m]


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
    """ShipDirection.__init__ 을 실제로 돌린다 — Node 메서드만 잠깐 끼운다 (test_side_escape 와 같음)."""
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
    node.get_logger = lambda: log      # 실행 중 로그 경로용(인스턴스 속성)
    return node, log


def with_env(fn):
    """SD.time·메시지 생성자를 값 보존 스텁으로 바꿔 fn(clock) 실행 후 되돌린다."""
    keys = ('time', 'Float32', 'Float32MultiArray', 'Int32', 'DwaPlanner')
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


def arrays(sc):
    """노드 (C) 가 만드는 angle_array·distance_array (0~160, 경계 없음)."""
    return [i * INC_DEG for i in range(321)], list(sc.ranges[:321])


BUOY_SC = scan([(82.0, 86.0, 2.8)])             # 정면 약간 왼쪽 부표 → 갭 71.5, DWA 0(크게 우)
CLOSE_SC = scan([(0.0, 160.0, 0.3)])            # 사방 0.3 m → 갭 0 개(후진), DWA None(폴백)
# 부표 6개 2.8 m(폭 2°): 팽창이 사이를 다 막아 갭 0 개(후진), DWA 는 정면 ±15° 사이로 직진(80)
NOGAP_SC = scan([(c - 1.0, c + 1.0, 2.8) for c in (10.0, 35.0, 65.0, 95.0, 125.0, 150.0)])
OPEN_SC = scan([])
YAW = 0.0                                        # 목표 정면(yaw_mapped 80)


def _tick(node, clk, sc=BUOY_SC, yaw=YAW, candidate=CANDIDATE_INVALID):
    clk.t += 0.1
    node.latest_scan = sc
    node.yaw_error = yaw
    node.candidate_angle = candidate
    node.candidate_t = clk.t
    ShipDirection.control_cb(node)
    return node.pub_desired_angle.msgs[-1].data


ON = {'dwa_enabled': True, 'dwa_modes': [5, 8]}


def _node(over=None, wp_mode=8):
    node, log = _construct(dict(over or {}))
    if wp_mode is not None:
        node.wp_mode_cb(_ns(data=wp_mode))
    return node, log


def _state(node):
    return (node._side_prev, node.reverse_state, node.reverse_start_time,
            node.reverse_cooldown_start_time, node.prev_final_angle, node._side_escape_active)


SEQ = ([BUOY_SC] * 5 + [CLOSE_SC] * 8 + [NOGAP_SC] * 4 + [OPEN_SC] * 3 + [CLOSE_SC] * 40
       + [BUOY_SC] * 3)


def _run(node, clk, seq=SEQ, yaw=YAW):
    outs, states = [], []
    for sc in seq:
        outs.append(_tick(node, clk, sc=sc, yaw=yaw))
        states.append(_state(node))
    return outs, states


# ======================================================================
# 끔 = 기존과 같다
# ======================================================================
def test_off_no_planner_and_same_as_disallowed_mode():
    """기본(끔) → DwaPlanner 생성 안 함, DWA 로그 없음. 켜고 mode 7 → 출력·상태·로그 비트 동일."""
    def run(clk):
        def boom(*a, **k):
            raise AssertionError('끔인데 DwaPlanner 생성')
        SD.DwaPlanner = boom
        off, log_off = _node(wp_mode=7)
        assert off._dwa is None and off.dwa_enabled is False
        o1, s1 = _run(off, clk)
        assert not [ln for ln in log_off.lines if 'DWA' in ln[1]], log_off.lines
        assert REVERSE_ANGLE in o1 and STOP_HOLD in o1           # 후진 상태기계가 돈다
    with_env(run)

    def run2(clk):
        off, log_off = _node(wp_mode=7)
        on, log_on = _node(ON, wp_mode=7)
        calls = []
        on._dwa.choose = lambda *a: calls.append(a)
        clk.t = 1000.0
        o1, s1 = _run(off, clk)
        clk.t = 1000.0
        o2, s2 = _run(on, clk)
        assert o1 == o2 and s1 == s2
        assert not calls
        start = [ln for ln in log_on.lines if 'DWA 켬' in ln[1]]
        assert len(start) == 1
        assert [ln for ln in log_on.lines if ln not in start] == log_off.lines
    with_env(run2)


def test_disallowed_and_boot_modes_never_call():
    """dwa_modes 에 0·1·2·3·7·−1 을 넣어도 걸러져 그 모드에선 choose 안 부름. 부팅 wp_mode −1 도."""
    def run(clk):
        for m in (0, 1, 2, 3, 7, None):
            node, log = _node({'dwa_enabled': True, 'dwa_modes': [0, 1, 2, 3, 7, -1, 8]},
                              wp_mode=m)
            assert node.dwa_modes == [8]
            calls = []
            node._dwa.choose = lambda *a: calls.append(a)
            ref, _ = _node(wp_mode=m)
            clk.t = 1000.0
            o1, _ = _run(node, clk, seq=[BUOY_SC, CLOSE_SC, BUOY_SC])
            clk.t = 1000.0
            o2, _ = _run(ref, clk, seq=[BUOY_SC, CLOSE_SC, BUOY_SC])
            assert not calls and o1 == o2, m
            assert not log.of('INFO', 'DWA 사용')
    with_env(run)


def test_mode8_only_when_listed():
    """dwa_modes [8] → mode 5 에선 안 씀, mode 8 에선 씀."""
    def run(clk):
        node, _ = _node({'dwa_enabled': True}, wp_mode=5)
        assert node.dwa_modes == [8]
        calls = []
        real = node._dwa.choose
        node._dwa.choose = lambda *a: calls.append(a) or real(*a)
        _tick(node, clk)
        assert not calls
        node.wp_mode_cb(_ns(data=8))
        _tick(node, clk)
        assert len(calls) == 1
    with_env(run)


# ======================================================================
# 켬 — mode 8·5 동작
# ======================================================================
def test_on_mode8_and_mode5_use_dwa():
    """첫 틱 출력 == 새 DwaPlanner.choose(같은 배열) — 리미터 첫 값 통과. 갭 결과와 다르다."""
    def run(clk):
        ang, dist = arrays(BUOY_SC)
        want = DwaPlanner('A').choose(ang, dist, 80.0)
        assert want is not None
        gap, _ = _node(wp_mode=8)
        gap_out = _tick(gap, clk)
        assert gap_out != want
        for m in (8, 5):
            node, log = _node(ON, wp_mode=m)
            assert _tick(node, clk) == want, m
            assert len(log.of('INFO', f'DWA 사용(mode {m})')) == 1
            # 편 기억은 갭 기준(갭 노드와 같은 값)
            assert node._side_prev == gap._side_prev
            assert node.reverse_state == 'READY'
    with_env(run)


def test_observe_every_dwa_tick_including_fallback():
    def run(clk):
        node, _ = _node(ON)
        seen = []
        real = node._dwa.observe
        node._dwa.observe = lambda a: seen.append(a) or real(a)
        outs = [_tick(node, clk, sc=sc) for sc in (BUOY_SC, CLOSE_SC, CLOSE_SC)]
        assert len(seen) == 3
        assert seen[1:] == [REVERSE_ANGLE, REVERSE_ANGLE]           # 폴백 = 기존 후진(리미터 전)
        assert outs[1:] == [REVERSE_ANGLE, REVERSE_ANGLE]
    with_env(run)


def test_pick_resets_reverse_and_no_false_warn():
    """후진 중(REVERSING)에 DWA 가 고르면 READY. 후보 0 개 틱이라도 '후진 시작' WARN 안 남."""
    def run(clk):
        node, log = _node(ON)
        assert _tick(node, clk, sc=CLOSE_SC) == REVERSE_ANGLE       # DWA None → 기존 후진
        assert node.reverse_state == 'REVERSING'
        n_warn = len(log.of('WARN'))
        ang, dist = arrays(NOGAP_SC)
        assert DwaPlanner('A').choose(ang, dist, 80.0) is not None
        out = _tick(node, clk, sc=NOGAP_SC)
        assert 0.0 <= out <= 160.0 and node.reverse_state == 'READY'
        assert node._side_prev is None                              # 갭 후보 0 개 → None
        for _ in range(5):
            _tick(node, clk, sc=NOGAP_SC)
        assert len(log.of('WARN')) == n_warn, log.lines
        ref, _ = _node(wp_mode=8)
        assert _tick(ref, clk, sc=NOGAP_SC) == REVERSE_ANGLE        # 끄면 같은 장면에서 후진
    with_env(run)


def test_fallback_same_as_off_and_info_throttled():
    """DWA None → 기존 결과와 같다. DWA 선택 뒤 첫 폴백에 INFO(throttle) 1회, 계속 폴백이면 안 늘어남."""
    def run(clk):
        node, log = _node(ON)
        ref, _ = _node(wp_mode=8)
        _tick(node, clk)
        _tick(ref, clk)
        node.prev_final_angle = ref.prev_final_angle = None
        clk0 = clk.t
        o1 = [_tick(node, clk, sc=CLOSE_SC) for _ in range(10)]
        clk.t = clk0
        o2 = [_tick(ref, clk, sc=CLOSE_SC) for _ in range(10)]
        assert o1 == o2
        fb = log.of('INFO', '폴백')
        assert len(fb) == 1 and fb[0][1].get('throttle_duration_sec', 0) > 0
    with_env(run)


def test_entry_resets_estimate_and_logs_once():
    """mode 8 → 7 → 8: 진입마다 reset(추정 0.9,0,0) + 'DWA 사용' INFO 1회. 같은 구간 안에선 1회."""
    def run(clk):
        node, log = _node(ON)
        for _ in range(5):
            _tick(node, clk, sc=OPEN_SC, yaw=60.0)
        assert node._dwa.state != INIT_STATE
        assert len(log.of('INFO', 'DWA 사용')) == 1
        node.wp_mode_cb(_ns(data=7))
        _tick(node, clk)
        assert node._dwa_active is False
        node.wp_mode_cb(_ns(data=8))
        seen = []
        real = node._dwa.choose
        node._dwa.choose = lambda *a: seen.append(node._dwa.state) or real(*a)
        _tick(node, clk)
        assert seen == [INIT_STATE]
        assert len(log.of('INFO', 'DWA 사용')) == 2
    with_env(run)


def test_level2_tick_with_mode_change_still_resets():
    """레벨 2(_compute 안 탐) 동안 mode 가 8 → 7 → 8 이어도 진입 리셋은 일어난다(control_cb 판정)."""
    def run(clk):
        node, log = _node(ON)
        _tick(node, clk)
        node.failsafe_level = 2
        node.wp_mode_cb(_ns(data=7))
        assert _tick(node, clk) == STOP_HOLD
        node.wp_mode_cb(_ns(data=8))
        _tick(node, clk)
        node.failsafe_level = 0
        _tick(node, clk)
        assert len(log.of('INFO', 'DWA 사용')) == 2
    with_env(run)


def test_nan_yaw_falls_back():
    def run(clk):
        node, _ = _node(ON)
        ref, _ = _node(wp_mode=8)
        a = _tick(node, clk, yaw=math.nan)
        b = _tick(ref, clk, yaw=math.nan)
        assert a == b or (math.isnan(a) and math.isnan(b))
    with_env(run)


def test_nan_yaw_then_finite_not_hard_right():
    """리뷰 차단 1: NaN 틱(폴백 → observe(NaN)) 다음 유한 틱 == 같은 상태 새 planner 의 choose, 0° 아님."""
    def run(clk):
        node, _ = _node(ON)
        _tick(node, clk, sc=OPEN_SC)
        last0 = node._dwa.last
        _tick(node, clk, sc=OPEN_SC, yaw=math.nan)
        assert node._dwa.last == last0 and math.isfinite(node._dwa.last)
        ref = DwaPlanner('A')
        ref.state, ref.last = node._dwa.state, node._dwa.last
        ang, dist = arrays(OPEN_SC)
        want = ref.choose(ang, dist, 80.0)
        out = _tick(node, clk, sc=OPEN_SC)
        assert out == want and out != 0.0, (out, want)
    with_env(run)


def _idle_case(gap_ticks, how):
    """같은 mode 8 안에서 (C) 아닌 틱 gap_ticks 개(0.1 s 씩) 뒤 (C) 재개 → choose 직전 추정 상태."""
    out = {}

    def run(clk):
        node, log = _node(ON)
        for _ in range(5):
            _tick(node, clk, sc=OPEN_SC, yaw=60.0)
        before = node._dwa.state
        assert before != INIT_STATE
        for _ in range(gap_ticks):
            if how == 'level2':
                node.failsafe_level = 2
                assert _tick(node, clk) == STOP_HOLD
            else:
                assert _tick(node, clk, candidate=STOP_HOLD) == STOP_HOLD
        node.failsafe_level = 0
        seen = []
        real = node._dwa.choose
        node._dwa.choose = lambda *a: seen.append(node._dwa.state) or real(*a)
        _tick(node, clk, sc=OPEN_SC, yaw=60.0)
        out.update(seen=seen[0], before=before,
                   entry=len(log.of('INFO', 'DWA 사용')), idle=len(log.of('INFO', '공백')))
    with_env(run)
    return out


def test_idle_gap_resets_estimate():
    """권고 1: 레벨2·(0) 정지가 1.0 s 넘게 이어진 뒤 (C) → reset(INFO throttle). 0.3 s 면 그대로."""
    for how in ('level2', 'stop'):
        r = _idle_case(20, how)                       # 2.0 s
        assert r['seen'] == INIT_STATE and r['entry'] == 1 and r['idle'] == 1, (how, r)
        r = _idle_case(3, how)                        # 0.3 s
        assert r['seen'] == r['before'] and r['idle'] == 0, (how, r)


def test_observe_exception_quiet_reset():
    """권고 2: observe 예외가 이어져도 'DWA 사용' INFO 는 진입 1회뿐. 다음 DWA 틱에 추정 초기화."""
    def run(clk):
        node, log = _node(ON)

        def boom(a):
            raise RuntimeError('obs')
        real_obs = node._dwa.observe
        node._dwa.observe = boom
        for _ in range(5):
            _tick(node, clk, sc=OPEN_SC, yaw=60.0)
        assert len(log.of('ERROR', 'DWA 추정 갱신 예외')) == 5
        assert len(log.of('INFO', 'DWA 사용')) == 1
        node._dwa.observe = real_obs
        node._dwa.state = (0.1, 0.0, 0.0)
        seen = []
        real = node._dwa.choose
        node._dwa.choose = lambda *a: seen.append(node._dwa.state) or real(*a)
        _tick(node, clk, sc=OPEN_SC, yaw=60.0)
        _tick(node, clk, sc=OPEN_SC, yaw=60.0)
        assert seen[0] == INIT_STATE and seen[1] != INIT_STATE
        assert len(log.of('INFO', 'DWA 사용')) == 1
    with_env(run)


def test_choose_exception_keeps_old_result_and_publishes():
    def run(clk):
        node, log = _node(ON)
        ref, _ = _node(wp_mode=8)

        def boom(*a):
            raise RuntimeError('x')
        node._dwa.choose = boom
        n0 = len(node.pub_desired_angle.msgs)
        clk0 = clk.t
        o1 = [_tick(node, clk, sc=sc) for sc in (BUOY_SC, CLOSE_SC, BUOY_SC)]
        clk.t = clk0
        o2 = [_tick(ref, clk, sc=sc) for sc in (BUOY_SC, CLOSE_SC, BUOY_SC)]
        assert o1 == o2
        assert len(node.pub_desired_angle.msgs) == n0 + 3
        err = log.of('ERROR', 'DWA 계산 예외')
        assert len(err) == 3 and all(kw.get('throttle_duration_sec', 0) > 0 for _, kw in err)
    with_env(run)


def test_slow_choose_warns_throttled():
    def run(clk):
        node, log = _node(ON)
        real = node._dwa.choose

        def slow(*a):
            clk.t += 0.06
            return real(*a)
        node._dwa.choose = slow
        _tick(node, clk)
        w = log.of('WARN', 'DWA 1회')
        assert len(w) == 1 and '60 ms' in w[0][0] and w[0][1].get('throttle_duration_sec') == 5.0
        node._dwa.choose = real
        _tick(node, clk)
        assert len(log.of('WARN', 'DWA 1회')) == 1
    with_env(run)


# ======================================================================
# 파라미터 기본값·가드
# ======================================================================
def _yaml():
    with open(os.path.join(PKG, 'config', 'ship_direction.yaml'), encoding='utf-8') as f:
        return yaml.safe_load(f)['/ship_direction']['ros__parameters']


def _src():
    with open(os.path.join(PKG, 'ship_direction', 'ship_direction.py'), encoding='utf-8') as f:
        return f.read()


def test_defaults_equal_yaml():
    p = _yaml()
    want = {'dwa_enabled': False, 'dwa_modes': [8], 'dwa_hull': 'A',
            'dwa_model_agility': 1.0, 'dwa_lidar_x_m': 0.0}
    for k, v in want.items():
        assert p[k] == v and type(p[k]) is type(v), (k, p[k])
    src = _src()
    for k, lit in (('dwa_enabled', 'False'), ('dwa_modes', r'\[8\]'), ('dwa_hull', "'A'"),
                   ('dwa_model_agility', r'1\.0'), ('dwa_lidar_x_m', r'0\.0')):
        assert re.search(rf"declare_parameter\('{k}', {lit}\)", src), k
    node, log = _construct({})
    assert (node.dwa_enabled, node.dwa_modes, node.dwa_hull, node.dwa_model_agility,
            node.dwa_lidar_x_m) == (False, [8], 'A', 1.0, 0.0)
    assert node._dwa is None
    assert not [ln for ln in log.lines if 'dwa' in ln[1].lower()], log.lines


def test_start_info_when_on():
    node, log = _construct({'dwa_enabled': True, 'dwa_hull': 'B', 'dwa_modes': [8, 5],
                            'dwa_model_agility': 1.4, 'dwa_lidar_x_m': 0.3})
    i = log.of('INFO', 'DWA 켬')
    assert len(i) == 1
    for word in ('선체 B', '[8, 5]', '1.4', '0.3'):
        assert word in i[0][0], (word, i)
    assert node._dwa.hull == 'B' and node._dwa.agility == 1.4 and node._dwa.lidar_x_m == 0.3


def test_guard_modes():
    node, log = _construct({'dwa_enabled': True, 'dwa_modes': [0, 8, 7, 8]})
    assert node.dwa_modes == [8] and node.dwa_enabled
    assert len(log.of('WARN', 'dwa_modes')) == 1
    node, log = _construct({'dwa_enabled': True, 'dwa_modes': [7, 3]})
    assert node.dwa_modes == [] and node.dwa_enabled is False and node._dwa is None
    assert log.of('WARN', '하나도 없음')
    node, log = _construct({'dwa_enabled': True, 'dwa_modes': [-1]})
    assert node.dwa_enabled is False


def test_guard_hull():
    for bad in ('C', 'a', '', None):
        node, log = _construct({'dwa_enabled': True, 'dwa_hull': bad})
        assert node.dwa_enabled is False and node._dwa is None, bad
        assert len(log.of('ERROR', 'dwa_hull')) == 1, bad
    node, log = _construct({'dwa_enabled': True, 'dwa_hull': 'B'})
    assert node.dwa_enabled and not log.of('ERROR')


def test_guard_agility_and_lidar_x():
    for bad in (0.49, 2.01, math.nan, math.inf, -1.0):
        node, log = _construct({'dwa_model_agility': bad})
        assert node.dwa_model_agility == 1.0 and len(log.of('WARN', 'dwa_model_agility')) == 1
    for ok in (0.5, 2.0, 1.3):
        node, log = _construct({'dwa_model_agility': ok})
        assert node.dwa_model_agility == ok and not log.of('WARN')
    for bad in (1.01, -1.01, math.nan, math.inf):
        node, log = _construct({'dwa_lidar_x_m': bad})
        assert node.dwa_lidar_x_m == 0.0 and len(log.of('WARN', 'dwa_lidar_x_m')) == 1
    for ok in (1.0, -1.0, 0.65):
        node, log = _construct({'dwa_lidar_x_m': ok})
        assert node.dwa_lidar_x_m == ok and not log.of('WARN')


def test_guard_enabled_type():
    node, log = _construct({'dwa_enabled': 1})
    assert node.dwa_enabled is False and log.of('WARN', 'dwa_enabled')
