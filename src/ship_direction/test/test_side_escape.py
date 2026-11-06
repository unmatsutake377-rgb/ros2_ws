#!/usr/bin/env python3
"""(C) 후보 0 개 + 옆 장애물 → 후진 대신 비킴 테스트 (side_escape_deg).
실행: python3 src/ship_direction/test/test_side_escape.py

근거: docs/문제와작업/시뮬3자유도_발견_20261106.md 발견 9 / 계약서_옆비킴_현장조정_20261106.md 검증 계획.
순수 함수(gap_select.side_escape)는 ROS 없이 바로 부르고, 노드 통합은 test_gap_select.py 와 같은
ROS 스텁 + 실제 __init__(Node 메서드만 잠깐 끼움)으로 만든 노드의 control_cb 로 본다.
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

from ship_direction.gap_select import side_escape  # noqa: E402

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
    ShipDirection, CANDIDATE_INVALID, STOP_HOLD, REVERSE_ANGLE, LIDAR_FORWARD_DEG)

FWD = LIDAR_FORWARD_DEG
AWAY = 60.0          # 실배 확인 뒤 배별 yaml 값(결정 ②)

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


# ======================================================================
# 순수 함수 — LiDAR 프레임 0~160 (0.5° 간격), 80 = 정면, > 80 = 좌현
# ======================================================================
STEP = 0.5
ANG = [i * STEP for i in range(321)]


def dist(cells=()):
    """cells = [(lo, hi, r)] 구간 셀만 r, 나머지 inf."""
    d = [math.inf] * len(ANG)
    for lo, hi, r in cells:
        for i, a in enumerate(ANG):
            if lo <= a <= hi:
                d[i] = r
    return d


LEFT = [(110.0, 0.45)]
LEFT_D = dist([(100.0, 120.0, 0.45)])


def esc(blobs, d, away=AWAY):
    return side_escape(blobs, ANG, d, away, FWD)


def test_off_returns_none():
    """끔(0.0)·음수·비유한 → None."""
    for away in (0.0, -1.0, math.nan, math.inf):
        assert esc(LEFT, LEFT_D, away) is None, away


def test_left_obstacle_goes_right():
    """좌 110°·0.45 m → 80 − 60 = 20 (우)."""
    assert esc(LEFT, LEFT_D) == 20.0


def test_mirror_goes_left():
    """거울: 우 50°·0.45 m → 80 + 60 = 140 (좌)."""
    assert esc([(50.0, 0.45)], dist([(40.0, 60.0, 0.45)])) == 140.0


def test_front_lower_bound():
    """|중심 − 80| 14.9 → None(정면 장애물 = 후진), 15.0 → 발동. 오른쪽도 같다."""
    assert esc([(94.9, 0.45)], dist()) is None
    assert esc([(95.0, 0.45)], dist([(95.0, 95.0, 0.45)])) == 20.0
    assert esc([(65.1, 0.45)], dist()) is None
    assert esc([(65.0, 0.45)], dist([(65.0, 65.0, 0.45)])) == 140.0


def test_beam_upper_bound():
    """정횡 상한(결정 ③): |중심 − 80| 60.0 → 발동, 60.1 → None(선미 휘두름). 오른쪽도 같다."""
    assert esc([(140.0, 0.3)], dist([(140.0, 140.0, 0.3)])) == 20.0
    assert esc([(140.1, 0.3)], dist()) is None
    assert esc([(20.0, 0.3)], dist([(20.0, 20.0, 0.3)])) == 140.0
    assert esc([(19.9, 0.3)], dist()) is None


def test_front_window_exact():
    """정면 ±10° 최솟값 0.8 정확 → None (> 0.8 이어야 함). 0.81 → 발동."""
    assert esc(LEFT, dist([(100.0, 120.0, 0.45), (80.0, 80.0, 0.8)])) is None
    assert esc(LEFT, dist([(100.0, 120.0, 0.45), (80.0, 80.0, 0.81)])) == 20.0
    assert esc(LEFT, dist([(100.0, 120.0, 0.45), (90.0, 90.0, 0.5)])) is None   # 창 끝 포함


def test_arc_whole_checked():
    """호 전체(결정 ③): tgt 20 일 때 45°(정면 창·tgt 창 밖, 호 안)에 0.5 m → None. 0.81 → 발동."""
    assert esc(LEFT, dist([(100.0, 120.0, 0.45), (45.0, 45.0, 0.5)])) is None
    assert esc(LEFT, dist([(100.0, 120.0, 0.45), (45.0, 45.0, 0.81)])) == 20.0
    # 호 반대편(정면보다 왼쪽, 정면 창 밖)은 안 본다
    assert esc(LEFT, dist([(100.0, 120.0, 0.45), (95.0, 95.0, 0.5)])) == 20.0


def test_target_window():
    """tgt ±15° 최솟값 1.0 정확 → None (> 1.0). 경계값: 창 끝 35.0 포함, 4.5 는 창·호 밖."""
    assert esc(LEFT, dist([(100.0, 120.0, 0.45), (30.0, 30.0, 1.0)])) is None
    assert esc(LEFT, dist([(100.0, 120.0, 0.45), (30.0, 30.0, 1.01)])) == 20.0
    assert esc(LEFT, dist([(100.0, 120.0, 0.45), (35.0, 35.0, 0.9)])) is None
    assert esc(LEFT, dist([(100.0, 120.0, 0.45), (5.0, 5.0, 0.9)])) is None
    assert esc(LEFT, dist([(100.0, 120.0, 0.45), (4.5, 4.5, 0.5)])) == 20.0


def test_no_finite_cells_pass():
    """inf·nan 만 → 창이 비었다(99) → 통과."""
    d = [math.inf if i % 2 else math.nan for i in range(len(ANG))]
    assert esc(LEFT, d) == 20.0
    assert side_escape(LEFT, [], [], AWAY, FWD) == 20.0          # 셀 자체가 없음


def test_nearest_blob_decides_side():
    """덩어리 둘 → 가까운 쪽의 반대편."""
    d = dist([(110.0, 110.0, 1.5), (50.0, 50.0, 0.4)])
    assert esc([(110.0, 1.5), (50.0, 0.4)], d) == 140.0
    d = dist([(110.0, 110.0, 0.4), (50.0, 50.0, 1.5)])
    assert esc([(110.0, 0.4), (50.0, 1.5)], d) == 20.0


def test_empty_blobs():
    assert esc([], LEFT_D) is None


def test_away_80_limits():
    """away 80 → 0 / 160 (0~160 안). 80 초과는 목표각이 범위 밖이라 None."""
    assert esc(LEFT, LEFT_D, 80.0) == 0.0
    assert esc([(50.0, 0.45)], dist([(40.0, 60.0, 0.45)]), 80.0) == 160.0
    assert esc(LEFT, LEFT_D, 80.1) is None


def test_inputs_not_mutated():
    blobs = [(110.0, 0.45), (40.0, 2.0)]
    ang = list(ANG)
    d = dist([(100.0, 120.0, 0.45), (30.0, 30.0, math.nan), (60.0, 60.0, -1.0)])
    snap = (list(blobs), list(ang), list(d))
    side_escape(blobs, ang, d, AWAY, FWD)
    assert (blobs, ang, d) == snap


def test_zero_or_negative_range_is_blocked():
    """r = 0.0(·음수) 는 측정 없음이 아니라 막힘."""
    for a in (80.0, 45.0, 30.0):
        for r in (0.0, -1.0):
            assert esc(LEFT, dist([(100.0, 120.0, 0.45), (a, a, r)])) is None, (a, r)


# ======================================================================
# 노드 통합 — 실제 __init__ 로 만든 노드, control_cb 틱
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
    """ShipDirection.__init__ 을 실제로 돌린다 — Node 메서드만 잠깐 끼운다 (test_gap_select 와 같음)."""
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
    keys = ('time', 'Float32', 'Float32MultiArray', 'Int32', 'side_escape')
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


# 후보 0 개 장면: 오른쪽~정면 벽 1.5 m(팽창 ±20°) + 틈 + 좌 110° 부표 0.45 m(팽창 ±50°) → 전부 막힘.
# 가장 가까운 덩어리 = 부표(중심 110, 0.45 m). 정면·호·tgt 창은 1.5 m 라 통과 → 비킴 20.
LEFT_SC = scan([(0.0, 89.5, 1.5), (100.0, 120.0, 0.45)])
RIGHT_SC = scan([(40.0, 60.0, 0.45), (70.5, 160.0, 1.5)])    # 거울 → 140
WALL_SC = scan([(0.0, 160.0, 0.5)])                          # 정면 벽 → 후진


def _tick(node, clk, sc=LEFT_SC, yaw=20.0, candidate=CANDIDATE_INVALID):
    clk.t += 0.1
    node.latest_scan = sc
    node.yaw_error = yaw
    node.candidate_angle = candidate
    node.candidate_t = clk.t
    ShipDirection.control_cb(node)
    return node.pub_desired_angle.msgs[-1].data


def _node(away, wp_mode=5):
    node, log = _construct({'side_escape_deg': away})
    node.wp_mode_cb(_ns(data=wp_mode))
    return node, log


def test_node_off_keeps_reverse_sequence():
    """끔(기본 0.0) → 기존 후진 시퀀스 그대로(260 × 5 → 50000 × 30 → 260 …), side_escape 안 부름."""
    def run(clk):
        calls = []
        SD.side_escape = lambda *a, **k: calls.append(a)
        node, _ = _node(0.0)
        outs = [_tick(node, clk) for _ in range(40)]
        assert not calls, calls
        assert outs[:5] == [REVERSE_ANGLE] * 5, outs[:6]
        assert outs[5:35] == [STOP_HOLD] * 30, outs[5:36]
        assert outs[35] == REVERSE_ANGLE, outs[30:]
        # 켜도 side_escape 가 None 이면 같은 출력 (분기 바깥은 그대로)
        SD.side_escape = lambda *a, **k: None
        on, _ = _node(AWAY)
        clk.t = 1000.0
        assert [_tick(on, clk) for _ in range(40)] == outs
    with_env(run)


def test_node_on_left_escapes_right():
    """켬 60 → 좌 부표에서 20. 편 기억 None(결정 ⑥), 후진 상태기계 READY."""
    def run(clk):
        node, _ = _node(AWAY)
        assert _tick(node, clk) == 20.0
        assert node._side_prev is None and node.reverse_state == 'READY'
        mirror, _ = _node(AWAY)
        assert _tick(mirror, clk, sc=RIGHT_SC) == 140.0
    with_env(run)


def test_node_rate_limiter_applies():
    """비킴 각은 0~160 일반값이라 리미터를 거친다. 직전 260 이면 리미터 상태 None → 바로 20."""
    def run(clk):
        node, _ = _node(AWAY)
        node.prev_final_angle = 60.0
        lim = node.rate_limit_max_deg_per_tick
        assert _tick(node, clk) == 60.0 - lim
        assert _tick(node, clk) == 60.0 - 2 * lim
        node2, _ = _node(AWAY)
        assert _tick(node2, clk, sc=WALL_SC) == REVERSE_ANGLE
        assert node2.reverse_state == 'REVERSING'
        assert _tick(node2, clk) == 20.0                     # 후진 0.5 s 도중 끊음(결정 ④)
        assert node2.reverse_state == 'READY'
    with_env(run)


def test_node_breaks_cooldown():
    """쿨다운(50000) 도중에도 옆 장애물이면 비킴."""
    def run(clk):
        node, _ = _node(AWAY)
        outs = [_tick(node, clk, sc=WALL_SC) for _ in range(8)]
        assert outs[-1] == STOP_HOLD and node.reverse_state == 'COOLDOWN', outs
        assert _tick(node, clk) == 20.0 and node.reverse_state == 'READY'
    with_env(run)


def test_node_front_wall_reverses():
    """켜도 정면 벽(중심 80) → 260."""
    def run(clk):
        node, _ = _node(AWAY)
        assert _tick(node, clk, sc=WALL_SC) == REVERSE_ANGLE
    with_env(run)


def test_node_gate_modes_keep_reverse():
    """게이트 구간 wp_mode 0·1 → 켜도 260(결정 ③). 다른 모드는 20."""
    def run(clk):
        for m in (0, 1):
            node, _ = _node(AWAY, wp_mode=m)
            assert node.detection_distance == node.detection_distance_gate
            assert _tick(node, clk) == REVERSE_ANGLE, m
        for m in (2, 5, 7):
            node, _ = _node(AWAY, wp_mode=m)
            assert _tick(node, clk) == 20.0, m
    with_env(run)


def test_node_level2_stop():
    """레벨 2 → 켜도 50000."""
    def run(clk):
        node, _ = _node(AWAY)
        node.failsafe_level = 2
        assert _tick(node, clk) == STOP_HOLD
        node.failsafe_level = 0
        assert _tick(node, clk) == 20.0
    with_env(run)


def test_node_enter_log_once():
    """비킴 진입 INFO 1회(틱마다 아님). 후진 틱을 거쳐 다시 들어오면 한 번 더."""
    def run(clk):
        node, log = _node(AWAY)

        def esc_infos():
            return [m for s, m in log.lines if s == 'INFO' and '비킴' in m]
        for _ in range(10):
            _tick(node, clk)
        assert len(esc_infos()) == 1, log.lines
        assert '110°' in esc_infos()[0] and '0.45 m' in esc_infos()[0] and '20°' in esc_infos()[0]
        assert not [m for s, m in log.lines if s == 'WARN'], log.lines   # 후진 시작 WARN 없음
        _tick(node, clk, sc=WALL_SC)
        _tick(node, clk)
        assert len(esc_infos()) == 2, log.lines
    with_env(run)


def _yaml():
    with open(os.path.join(PKG, 'config', 'ship_direction.yaml'), encoding='utf-8') as f:
        return yaml.safe_load(f)['/ship_direction']['ros__parameters']


def _src():
    with open(os.path.join(PKG, 'ship_direction', 'ship_direction.py'), encoding='utf-8') as f:
        return f.read()


def test_code_default_equals_yaml():
    """코드 기본값 == yaml == 0.0(끔, 결정 ②), float."""
    p = _yaml()
    assert isinstance(p['side_escape_deg'], float) and p['side_escape_deg'] == 0.0, p
    node, log = _construct({})
    assert node.side_escape_deg == 0.0
    m = re.search(r"declare_parameter\('side_escape_deg', ([0-9.]+)\)", _src())
    assert m and float(m.group(1)) == p['side_escape_deg']
    assert not [m for s, m in log.lines if s == 'WARN'], log.lines


def test_bad_param_turns_off():
    """nan/inf/음수/81 → 0.0(끔) + WARN 1회. 0.0·60.0·80.0 은 그대로, WARN 없음."""
    for bad in (math.nan, math.inf, -math.inf, -1.0, 81.0):
        node, log = _construct({'side_escape_deg': bad})
        assert node.side_escape_deg == 0.0, bad
        w = [m for s, m in log.lines if s == 'WARN' and 'side_escape_deg' in m]
        assert len(w) == 1, (bad, log.lines)
    for ok in (0.0, 60.0, 80.0):
        node, log = _construct({'side_escape_deg': ok})
        assert node.side_escape_deg == ok
        assert not [m for s, m in log.lines if s == 'WARN'], (ok, log.lines)


def main():
    print("=== (C) 후보 0 개 옆 장애물 비킴 테스트 (발견 9) ===")
    for n, f in sorted(globals().items()):
        if n.startswith("test_") and callable(f):
            check(n[5:], f)
    print(f"\n{_p}/{_t} 통과")
    return 0 if _p == _t else 1


if __name__ == "__main__":
    sys.exit(main())
