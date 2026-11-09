#!/usr/bin/env python3
"""(C) 옆 잠금 테스트 — side_lock.py 순수 로직 + 노드 훅·파라미터·꺼짐 불변 (T2-1).

실행: tools/run_tests.sh ship_direction

근거: docs/문제와작업/계약서_토너먼트분리_20261109.md T2-1·§5 위험 4·§6·§9-6·§9-7.
노드 부분은 test_dwa_node.py 와 같은 ROS 스텁 + 실제 __init__(Node 메서드만 잠깐 끼움)의 control_cb 틱.

부호 규약(CLAUDE.md §3): LiDAR 프레임 80° = 정면, 상대방위 rel = a − 80 은 좌 양수. 조향각 > 80 = 좌.
  고정 근거 3개 — ① DwaPlanner.points: LiDAR 170° → 선체 y > 0(왼쪽) ② pwm_from_cmd(motor_control
  사본, test_dwa 가 원본과 대조): 조향 > 80 이면 오른쪽 추진기가 더 세다(좌회전) ③ gap_select.side_escape:
  중심 > 80 인 장애물 → 80 보다 작은 각(우)으로 비킴. 이 셋과 옆 구역(좌 = rel +45~+135)이 맞는지 본다.

꺼짐 불변 골든: GOLDEN 은 **변경 전 코드**(옆 잠금 추가 전 작업 트리)로 같은 묶음을 돌려 뽑은 값이다.
  DWA 켬 조합은 numpy 삼각함수의 플랫폼 차이로 동률이 갈릴 수 있어 숫자로 박지 않고, 같은 기계에서
  '옆 잠금 끔(기본) == 옆 잠금 켬이지만 옆 구역 모름·비어 있음' 과 choose 호출 인자 불변으로 본다.
"""

import ast
import importlib.util
import math
import os
import re
import sys
import types

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, '..')
WS = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, PKG)

from ship_direction.dwa import DwaPlanner, pwm_from_cmd  # noqa: E402
from ship_direction.gap_select import side_escape  # noqa: E402
from ship_direction.side_lock import (  # noqa: E402
    CLEAR, LOCKED, UNKNOWN, SideLockFilter, SideLockWait, allowed_range, angle_allowed,
    check_params, clip_candidates, side_states, zone_min)

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
        return [m for s, m, _kw in self.lines if s == sev and word in m]


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
    """ShipDirection.__init__ 을 실제로 돌린다 — Node 메서드만 잠깐 끼운다 (test_dwa_node 와 같음)."""
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
    node.get_logger = lambda: log
    return node, log


def with_env(fn):
    """SD.time·메시지 생성자를 값 보존 스텁으로 바꿔 fn(clock) 실행 후 되돌린다."""
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


def scan180(obstacles):
    """test_dwa_node·test_side_escape 와 같은 0~179.5° 스캔(LiDAR 각). 옆 구역은 덮지 않는다."""
    n = 360
    ranges = [math.inf] * n
    for lo, hi, r in obstacles:
        for i in range(n):
            if lo <= i * INC_DEG <= hi:
                ranges[i] = r
    return types.SimpleNamespace(angle_min=0.0, angle_increment=math.radians(INC_DEG),
                                 ranges=ranges)


def _rel(a):
    """LiDAR 각 → 상대방위(0 = 정면, 좌 양수, −180~180)."""
    return ((a - 80.0 + 180.0) % 360.0) - 180.0


def scan360(obstacles, angle_min_deg=-180.0):
    """한 바퀴 스캔(시뮬 LiDAR 규격: −180° 부터 0.5° × 720). obstacles = [(lo_rel, hi_rel, r)]
    — **상대방위**(좌 양수) 구간."""
    n = 720
    ranges = [math.inf] * n
    for i in range(n):
        rel = _rel(angle_min_deg + i * INC_DEG)
        for lo, hi, r in obstacles:
            if lo <= rel <= hi:
                ranges[i] = r
    return types.SimpleNamespace(angle_min=math.radians(angle_min_deg),
                                 angle_increment=math.radians(INC_DEG), ranges=ranges)


# 기존 테스트의 스캔 묶음 (test_dwa_node · test_side_escape) — 0~179.5° 라 옆 구역 '모름'
BUOY_SC = scan180([(82.0, 86.0, 2.8)])
CLOSE_SC = scan180([(0.0, 160.0, 0.3)])
NOGAP_SC = scan180([(c - 1.0, c + 1.0, 2.8) for c in (10.0, 35.0, 65.0, 95.0, 125.0, 150.0)])
OPEN_SC = scan180([])
LEFT_SC = scan180([(0.0, 89.5, 1.5), (100.0, 120.0, 0.45)])
WALL_SC = scan180([(0.0, 160.0, 0.5)])
# 한 바퀴 스캔 — 켜면 잠금이 걸리는 장면들 (상대방위, 좌 양수)
SIDE_L = scan360([(70.0, 110.0, 1.0), (-2.0, 6.0, 2.8)])       # 좌현 옆 1.0 m + 정면 부표
SIDE_R = scan360([(-110.0, -70.0, 1.0), (-6.0, 2.0, 2.8)])     # 거울
BOTH = scan360([(60.0, 120.0, 1.0), (-120.0, -60.0, 1.0)])     # 양현 옆 1.0 m, 정면 열림
FAR_L = scan360([(60.0, 120.0, 2.0)])                          # 좌현 옆 2.0 m(> 1.5, 안 잠김)
BLOCK_L = scan360([(-80.0, 80.0, 0.5), (85.0, 120.0, 1.0)])    # 정면 벽 + 좌현 옆 → 갭 0 개
ESC_L = scan360([(-80.0, 9.5, 1.5), (20.0, 40.0, 0.45), (95.0, 120.0, 1.0)])  # 비킴 장면 + 좌현 옆

GOLDEN_SEQ = ([(BUOY_SC, 0.0)] * 3 + [(SIDE_L, 40.0)] * 4 + [(SIDE_R, 320.0)] * 4
              + [(BOTH, 40.0)] * 3 + [(BLOCK_L, 0.0)] * 6 + [(OPEN_SC, 0.0)] * 2
              + [(LEFT_SC, 20.0)] * 3 + [(WALL_SC, 20.0)] * 5 + [(FAR_L, 40.0)] * 3
              + [(ESC_L, 20.0)] * 3 + [(CLOSE_SC, 0.0)] * 5 + [(NOGAP_SC, 0.0)] * 3
              + [(SIDE_L, 40.0)] * 2)
GOLDEN_CONFIGS = (('off', {}), ('escape', {'side_escape_deg': 60.0}))
GOLDEN_MODES = (8, 5, 0)


def _tick(node, clk, sc, yaw, candidate=CANDIDATE_INVALID, fresh=True):
    """제어 한 틱. fresh = 새 스캔 도착(메시지 객체 새로 — 실제 scan_callback 처럼). False = 같은 스캔 재처리."""
    clk.t += 0.1
    if fresh:
        sc = types.SimpleNamespace(**vars(sc))
    node.latest_scan = sc if fresh else getattr(node, '_test_last_scan', sc)
    node._test_last_scan = node.latest_scan
    node.yaw_error = yaw
    node.candidate_angle = candidate
    node.candidate_t = clk.t
    ShipDirection.control_cb(node)
    return node.pub_desired_angle.msgs[-1].data


def golden_run(over, mode, seq=GOLDEN_SEQ):
    """→ (출력 목록, reverse_state 목록). 같은 시계 1000.0 에서 시작."""
    def run(clk):
        node, _log = _construct(dict(over))
        node.wp_mode_cb(_ns(data=mode))
        outs, rev = [], []
        for sc, yaw in seq:
            outs.append(_tick(node, clk, sc, yaw))
            rev.append(node.reverse_state)
        return outs, rev
    return with_env(run)


def rle(xs):
    """[a, a, b] → [(a, 2), (b, 1)]."""
    out = []
    for x in xs:
        if out and out[-1][0] == x:
            out[-1] = (x, out[-1][1] + 1)
        else:
            out.append((x, 1))
    return out


# 변경 전 코드로 뽑은 값 (위 머리말). (설정, wp_mode) → (출력 RLE, reverse_state RLE)
GOLDEN = {
    ('off', 8): (
        [(71.5, 3), (67.5, 4), (52.5, 1), (40.0, 3), (55.0, 1), (70.0, 1), (85.0, 1), (260.0, 5),
         (50000.0, 1), (80.0, 2), (260.0, 5), (50000.0, 3), (120.0, 3), (260.0, 5), (50000.0, 6),
         (67.5, 2)],
        [('READY', 14), ('REVERSING', 5), ('COOLDOWN', 1), ('READY', 2), ('REVERSING', 5),
         ('COOLDOWN', 3), ('READY', 3), ('REVERSING', 5), ('COOLDOWN', 6), ('READY', 2)]),
    ('off', 5): (
        [(71.5, 3), (67.5, 4), (52.5, 1), (40.0, 3), (55.0, 1), (70.0, 1), (85.0, 1), (260.0, 5),
         (50000.0, 1), (80.0, 2), (260.0, 5), (50000.0, 3), (120.0, 3), (260.0, 5), (50000.0, 6),
         (67.5, 2)],
        [('READY', 14), ('REVERSING', 5), ('COOLDOWN', 1), ('READY', 2), ('REVERSING', 5),
         ('COOLDOWN', 3), ('READY', 3), ('REVERSING', 5), ('COOLDOWN', 6), ('READY', 2)]),
    ('off', 0): (
        [(80.0, 3), (95.0, 1), (110.0, 1), (120.0, 2), (105.0, 1), (90.0, 1), (75.0, 1), (60.0, 1),
         (75.0, 1), (90.0, 1), (105.0, 1), (260.0, 5), (50000.0, 1), (80.0, 2), (260.0, 5),
         (50000.0, 3), (120.0, 3), (260.0, 5), (50000.0, 3), (80.0, 3), (95.0, 1), (110.0, 1)],
        [('READY', 14), ('REVERSING', 5), ('COOLDOWN', 1), ('READY', 2), ('REVERSING', 5),
         ('COOLDOWN', 3), ('READY', 3), ('REVERSING', 5), ('COOLDOWN', 3), ('READY', 5)]),
    ('escape', 8): (
        [(71.5, 3), (67.5, 4), (52.5, 1), (40.0, 3), (55.0, 1), (70.0, 1), (85.0, 1), (260.0, 5),
         (50000.0, 1), (80.0, 2), (65.0, 1), (50.0, 1), (35.0, 1), (260.0, 5), (120.0, 3),
         (105.0, 1), (90.0, 1), (75.0, 1), (260.0, 5), (50000.0, 3), (67.5, 2)],
        [('READY', 14), ('REVERSING', 5), ('COOLDOWN', 1), ('READY', 5), ('REVERSING', 5),
         ('READY', 6), ('REVERSING', 5), ('COOLDOWN', 3), ('READY', 2)]),
    ('escape', 5): (
        [(71.5, 3), (67.5, 4), (52.5, 1), (40.0, 3), (55.0, 1), (70.0, 1), (85.0, 1), (260.0, 5),
         (50000.0, 1), (80.0, 2), (65.0, 1), (50.0, 1), (35.0, 1), (260.0, 5), (120.0, 3),
         (105.0, 1), (90.0, 1), (75.0, 1), (260.0, 5), (50000.0, 3), (67.5, 2)],
        [('READY', 14), ('REVERSING', 5), ('COOLDOWN', 1), ('READY', 5), ('REVERSING', 5),
         ('READY', 6), ('REVERSING', 5), ('COOLDOWN', 3), ('READY', 2)]),
    ('escape', 0): (
        [(80.0, 3), (95.0, 1), (110.0, 1), (120.0, 2), (105.0, 1), (90.0, 1), (75.0, 1), (60.0, 1),
         (75.0, 1), (90.0, 1), (105.0, 1), (260.0, 5), (50000.0, 1), (80.0, 2), (260.0, 5),
         (50000.0, 3), (120.0, 3), (260.0, 5), (50000.0, 3), (80.0, 3), (95.0, 1), (110.0, 1)],
        [('READY', 14), ('REVERSING', 5), ('COOLDOWN', 1), ('READY', 2), ('REVERSING', 5),
         ('COOLDOWN', 3), ('READY', 3), ('REVERSING', 5), ('COOLDOWN', 3), ('READY', 5)]),
}

ON3 = {'side_lock_enabled': True}                     # 기본값(확인 3프레임)
ON = dict(ON3, side_lock_confirm_frames=1)           # 동작 테스트용: 첫 프레임에 잠금
DWA = {'dwa_enabled': True, 'dwa_modes': [5, 8]}
ESC = {'side_escape_deg': 60.0}
# 켜면 잠금이 걸리는 장면 (상대방위, 좌 양수)
BESIDE_L = scan360([(90.0, 120.0, 1.0)])             # 좌현 옆 1.0 m — 0~160 창 밖(갭엔 안 보임)
BESIDE_R = scan360([(-120.0, -90.0, 1.0)])           # 거울
GAP_L = scan360([(-80.0, 0.0, 1.6), (90.0, 120.0, 1.0)])   # 정면~우 벽 → 틈은 좌 98° 뿐 + 좌현 옆
ESC_R = scan360([(-80.0, 9.5, 1.6), (20.0, 40.0, 0.45), (-120.0, -95.0, 1.0)])  # 비킴 20(우) + 우현 옆
YAW_L, YAW_R, YAW_0 = 40.0, 320.0, 0.0               # 목표 yaw_mapped 120(좌) / 40(우) / 80
ANG160 = [i * INC_DEG for i in range(321)]


def _node(over=None, wp_mode=8):
    node, log = _construct(dict(over or {}))
    if wp_mode is not None:
        node.wp_mode_cb(_ns(data=wp_mode))
    return node, log


def _rel_cells(sc):
    return sc.ranges, math.degrees(sc.angle_min), math.degrees(sc.angle_increment)


def _states(sc, dist=1.5, lo=45.0, hi=135.0, range_min=0.0):
    r, a0, inc = _rel_cells(sc)
    (ls, _ld), (rs, _rd) = side_states(r, a0, inc, 80.0, dist, lo, hi, range_min)
    return ls, rs


# ======================================================================
# 옆 구역 판정 (순수)
# ======================================================================
def test_zone_left_right_both():
    assert _states(BESIDE_L) == (LOCKED, CLEAR)
    assert _states(BESIDE_R) == (CLEAR, LOCKED)
    assert _states(BOTH) == (LOCKED, LOCKED)
    assert _states(scan360([])) == (CLEAR, CLEAR)
    assert _states(FAR_L) == (CLEAR, CLEAR)                      # 2.0 m ≥ 1.5 → 안 잠김
    r, a0, inc = _rel_cells(BESIDE_L)
    (ls, ld), (rs, rd) = side_states(r, a0, inc, 80.0, 1.5, 45.0, 135.0)
    assert ld == 1.0 and rd == math.inf


def test_zone_boundaries_and_dist_strict():
    """구역 양끝 포함(45·135), 밖(44·136)은 안 봄, 정면·정후방 안 봄. dist 는 '<'(같으면 안 잠김)."""
    for rel, want in ((45.0, LOCKED), (135.0, LOCKED), (44.0, CLEAR), (136.0, CLEAR),
                      (0.0, CLEAR), (180.0, CLEAR), (-180.0, CLEAR)):
        sc = scan360([(rel, rel, 1.0)])
        assert _states(sc)[0] == want, rel
        mirror = scan360([(-rel, -rel, 1.0)])
        assert _states(mirror)[1] == want, rel
    assert _states(scan360([(90.0, 90.0, 1.5)]))[0] == CLEAR
    assert _states(scan360([(90.0, 90.0, 1.499)]))[0] == LOCKED


def test_zone_unknown_when_scan_does_not_cover():
    """0~179.5° 스캔(기존 테스트 묶음): 좌 구역(LiDAR 125~215)·우 구역(−55~35) 둘 다 '모름'."""
    for sc in (OPEN_SC, CLOSE_SC, BUOY_SC, WALL_SC):
        assert _states(sc) == (UNKNOWN, UNKNOWN)
    # 0~270° 반쪽 스캔: 좌 구역은 덮임(잠김 판정 가능), 우 구역은 모름
    n = 541
    ranges = [math.inf] * n
    for i in range(n):
        if abs(_rel(i * INC_DEG) - 90.0) <= 5.0:
            ranges[i] = 1.0
    assert side_states(ranges, 0.0, INC_DEG, 80.0, 1.5, 45.0, 135.0)[0][0] == LOCKED
    assert side_states(ranges, 0.0, INC_DEG, 80.0, 1.5, 45.0, 135.0)[1][0] == UNKNOWN
    assert zone_min([], 0.0, INC_DEG, 80.0, 45.0, 135.0) == (False, math.inf)
    assert zone_min([1.0], 0.0, 0.0, 80.0, 45.0, 135.0) == (False, math.inf)
    assert zone_min([1.0], 0.0, math.nan, 80.0, 45.0, 135.0) == (False, math.inf)


def test_zone_angle_min_independent():
    """같은 장면을 angle_min −180 / 0 / +37 로 내도 같은 판정(한 바퀴는 돌려서 본다)."""
    for a0 in (-180.0, 0.0, 37.0, -90.25):
        for obst, want in (([(90.0, 120.0, 1.0)], (LOCKED, CLEAR)),
                           ([(-120.0, -90.0, 1.0)], (CLEAR, LOCKED)),
                           ([(130.0, 135.0, 1.0), (-50.0, -45.0, 0.8)], (LOCKED, LOCKED))):
            assert _states(scan360(obst, angle_min_deg=a0)) == want, (a0, obst)
    # 구역이 −180/180 을 넘는 경우(max 180): 정후방 셀도 본다
    sc = scan360([(179.0, 180.0, 1.0)])
    r, a0, inc = _rel_cells(sc)
    assert side_states(r, a0, inc, 80.0, 1.5, 100.0, 180.0)[0][0] == LOCKED


def test_zone_invalid_cells_ignored():
    """inf·nan·0·음수·range_min 미만은 '측정 없음'(안 잠금)."""
    for bad in (math.inf, math.nan, 0.0, -1.0):
        assert _states(scan360([(90.0, 100.0, bad)])) == (CLEAR, CLEAR), bad
    sc = scan360([(90.0, 100.0, 0.1)])
    assert _states(sc)[0] == LOCKED
    assert _states(sc, range_min=0.15)[0] == CLEAR
    assert _states(sc, range_min=math.nan)[0] == LOCKED         # range_min 비유한 → 0 으로 본다


# ======================================================================
# 부호 (좌 양수) — 옆 구역 ↔ 조향 방향
# ======================================================================
def test_sign_left_zone_is_port_and_steer_above_80_is_left():
    # ① LiDAR 170°(rel +90) 는 선체 왼쪽(y > 0) — DWA 좌표 규약
    P = DwaPlanner('A').points([170.0, -10.0], [1.0, 1.0])
    assert P[0][1] > 0.5 and P[1][1] < -0.5
    assert _states(scan360([(90.0, 90.0, 1.0)])) == (LOCKED, CLEAR)
    # ② 조향 > 80 → 오른쪽 추진기가 더 세다(ROS < 1500 = 전진) → 좌회전. motor_control 사본
    pl, pr = pwm_from_cmd(120.0)
    assert pr < pl
    pl, pr = pwm_from_cmd(40.0)
    assert pl < pr
    # ③ 좌현(중심 > 80) 장애물 → 비킴은 80 보다 작은 각(우)
    assert side_escape([(110.0, 0.45)], ANG160, [math.inf] * 321, 60.0) == 20.0
    # → 좌 잠김은 80 보다 큰 각을, 우 잠김은 작은 각을 자른다
    assert allowed_range(True, False, 80.0, 15.0) == (0.0, 95.0)
    assert allowed_range(False, True, 80.0, 15.0) == (65.0, 160.0)
    assert allowed_range(True, True, 80.0, 15.0) == (65.0, 95.0)
    assert allowed_range(False, False, 80.0, 15.0) == (0.0, 160.0)


def test_angle_allowed():
    assert angle_allowed(95.0, 0.0, 95.0) and angle_allowed(0.0, 0.0, 95.0)
    assert not angle_allowed(95.5, 0.0, 95.0)
    assert not angle_allowed(None, 0.0, 95.0) and not angle_allowed(math.nan, 0.0, 160.0)


# ======================================================================
# 갭 후보 자르기
# ======================================================================
def test_clip_candidates():
    ang = ANG160
    # zone 0~160(s 0, e 320) 목표 120 → 좌 잠김 95 로 clamp, -arc·s·e 그대로
    c = [(0.0, -5.0, 120.0, 0, 320)]
    assert clip_candidates(c, ang, 120.0, 0.0, 95.0) == [(25.0, -5.0, 95.0, 0, 320)]
    # zone 100~160 은 좌 잠김이면 빠짐, zone 20~60 은 그대로(같은 튜플)
    c2 = [(0.0, -1.0, 120.0, 200, 320), (60.0, -2.0, 60.0, 40, 120)]
    assert clip_candidates(c2, ang, 120.0, 0.0, 95.0) == [(60.0, -2.0, 60.0, 40, 120)]
    # 잠김 없음 구간(0~160) → 원본과 같다, 원본 리스트는 안 바뀐다
    keep = list(c2)
    assert clip_candidates(c2, ang, 120.0, 0.0, 160.0) == c2 and c2 == keep
    # 우 잠김: 목표 40 → 65
    assert clip_candidates(c, ang, 40.0, 65.0, 160.0) == [(25.0, -5.0, 65.0, 0, 320)]
    assert clip_candidates([], ang, 80.0, 0.0, 95.0) == []


# ======================================================================
# 후보 0 개 대기 상태기계 (§9-7)
# ======================================================================
def test_wait_start_hold_timeout_fallback():
    """update(blocked, now): blocked = 잠금이 후보를 지움."""
    w = SideLockWait(10.0)
    assert w.update(False, 0.0) == ('go', None)                 # 안 막힘(잠금 없음·막다른 곳 포함)
    assert w.update(True, 1.0) == ('wait', 'wait_start')
    assert w.waiting
    assert w.update(True, 5.0) == ('wait', None)
    assert w.update(True, 11.0) == ('wait', None)               # 정확히 10.0 s 는 아직 대기
    assert w.update(True, 11.05) == ('fallback', 'timeout')
    assert w.update(True, 30.0) == ('fallback', None)           # 폴백 유지(이벤트 1회)
    assert w.update(False, 30.1) == ('go', 'rearm')             # 잠금이 더는 후보를 안 지움
    assert w.update(True, 31.0) == ('wait', 'wait_start')
    assert w.update(False, 31.1) == ('go', 'resume')            # 길 열림 → 바로 재출발
    w.update(True, 32.0)
    w.reset()
    assert not w.waiting and w.update(True, 60.0) == ('wait', 'wait_start')


def test_wait_zero_means_immediate_fallback():
    w = SideLockWait(0.0)
    assert w.update(True, 1.0) == ('fallback', 'timeout')
    assert not w.waiting
    assert w.update(True, 1.1) == ('fallback', None)
    assert w.update(False, 1.2) == ('go', 'rearm')


# ======================================================================
# 잠금 필터 — 연속 프레임 확인 + 풀림 히스테리시스 (순수)
# ======================================================================
def _st(d):
    return (LOCKED if d < 1.5 else CLEAR, d)


def test_filter_confirm_frames():
    f = SideLockFilter(3, 0.3)
    near, far = _st(1.0), _st(math.inf)
    assert f.update((near, far), 1.5, True) == (False, False)   # 1프레임
    assert f.update((near, far), 1.5, False) == (False, False)  # 같은 스캔 재처리는 안 셈
    assert f.update((near, far), 1.5, True) == (False, False)   # 2프레임
    assert f.update((near, far), 1.5, True) == (True, False)    # 3프레임 → 잠김
    g = SideLockFilter(3, 0.3)
    for d in (1.0, 1.0, 1.6, 1.0, 1.0):                          # 중간에 끊기면 처음부터
        out = g.update((far, _st(d)), 1.5, True)
    assert out == (False, False)
    assert g.update((far, _st(1.0)), 1.5, True) == (False, True)


def test_filter_release_hysteresis_and_unknown():
    f = SideLockFilter(1, 0.3)
    far = _st(math.inf)
    assert f.update((_st(1.0), far), 1.5, True) == (True, False)
    for d in (1.5, 1.6, 1.79):                                   # dist 이상이어도 1.8 미만이면 유지
        assert f.update((_st(d), far), 1.5, True) == (True, False), d
    assert f.update((_st(1.6), far), 1.5, False) == (True, False)
    assert f.update((_st(1.8), far), 1.5, True) == (False, False)   # 1.5 + 0.3 이상 → 풀림
    assert f.update((_st(1.0), far), 1.5, True) == (True, False)
    assert f.update(((UNKNOWN, math.inf), far), 1.5, False) == (False, False)   # 모름 → 바로 초기화
    f3 = SideLockFilter(3, 0.3)
    f3.update((_st(1.0), far), 1.5, True)
    f3.update((_st(1.0), far), 1.5, True)
    f3.update(((UNKNOWN, math.inf), far), 1.5, True)
    assert f3.update((_st(1.0), far), 1.5, True) == (False, False)   # 카운터도 초기화됨
    f.reset()
    assert f.locked == [False, False] and f.count == [0, 0]


def test_filter_flicker_does_not_toggle():
    """1.4 ↔ 1.6 깜빡임: 확인 3프레임이면 안 잠김. 잠긴 뒤면 1.6 은 안 풀림(히스테리시스) → 계속 잠김."""
    f = SideLockFilter(3, 0.3)
    far = _st(math.inf)
    outs = [f.update((_st(d), far), 1.5, True)[0] for d in (1.4, 1.6) * 10]
    assert not any(outs)
    for _ in range(3):
        f.update((_st(1.4), far), 1.5, True)
    outs = [f.update((_st(d), far), 1.5, True)[0] for d in (1.6, 1.4) * 10]
    assert all(outs)


# ======================================================================
# 파라미터 가드 (순수)
# ======================================================================
GOOD = dict(enabled=True, dist_m=1.5, min_deg=45.0, max_deg=135.0, turn_deg=15.0,
            wait_max_sec=10.0, confirm_frames=3, release_margin_m=0.3)


def test_check_params():
    assert check_params(**GOOD) == []
    for k, v in (('confirm_frames', 0), ('confirm_frames', -1), ('confirm_frames', 2.5),
                 ('confirm_frames', 3.0), ('confirm_frames', True), ('confirm_frames', None),
                 ('release_margin_m', -0.1), ('release_margin_m', math.nan),
                 ('release_margin_m', math.inf), ('release_margin_m', True)):
        assert check_params(**dict(GOOD, **{k: v})), (k, v)
    for k, v in (('confirm_frames', 1), ('confirm_frames', 10), ('release_margin_m', 0.0),
                 ('release_margin_m', 1.0)):
        assert check_params(**dict(GOOD, **{k: v})) == [], (k, v)
    assert check_params(**dict(GOOD, enabled=False)) == []
    for k, v in (('enabled', 'true'), ('enabled', 1), ('dist_m', 0.0), ('dist_m', -1.0),
                 ('dist_m', 5.01), ('dist_m', math.nan), ('dist_m', math.inf),
                 ('min_deg', -1.0), ('max_deg', 181.0), ('min_deg', 135.0), ('min_deg', 140.0),
                 ('turn_deg', -0.1), ('turn_deg', 80.5), ('turn_deg', math.nan),
                 ('wait_max_sec', -1.0), ('wait_max_sec', 60.5), ('wait_max_sec', math.inf),
                 ('dist_m', True), ('dist_m', None)):
        assert check_params(**dict(GOOD, **{k: v})), (k, v)
    for k, v in (('dist_m', 5.0), ('min_deg', 0.0), ('max_deg', 180.0), ('turn_deg', 0.0),
                 ('turn_deg', 80.0), ('wait_max_sec', 0.0), ('wait_max_sec', 60.0)):
        assert check_params(**dict(GOOD, **{k: v})) == [], (k, v)


# ======================================================================
# 노드 — 파라미터·yaml
# ======================================================================
DEFAULTS = {'side_lock_enabled': False, 'side_lock_dist_m': 1.5, 'side_lock_min_deg': 45.0,
            'side_lock_max_deg': 135.0, 'side_lock_turn_deg': 15.0,
            'side_lock_wait_max_sec': 10.0, 'side_lock_confirm_frames': 3,
            'side_lock_release_margin_m': 0.3}


def _yaml(name):
    with open(os.path.join(PKG, 'config', name), encoding='utf-8') as f:
        return yaml.safe_load(f)['/ship_direction']['ros__parameters']


def _src():
    with open(os.path.join(PKG, 'ship_direction', 'ship_direction.py'), encoding='utf-8') as f:
        return f.read()


def test_code_default_equals_common_yaml_and_types():
    """코드 기본값 리터럴 == 공통 yaml == 기대값, 타입(bool/float) 같음(§1-4)."""
    src = _src()
    common = _yaml('ship_direction.yaml')
    for k, v in DEFAULTS.items():
        m = re.search(r"declare_parameter\(\s*'" + k + r"', ([^)]+)\)", src)
        assert m, k
        lit = ast.literal_eval(m.group(1))
        assert type(lit) is type(v) and lit == v, (k, lit)
        assert type(common[k]) is type(v) and common[k] == v, (k, common.get(k))


def test_tournament_yaml_has_keys_off():
    """tournament yaml: 8 키 모두, 타입 같음(confirm_frames 정수), 처음 값 false(§9-6)."""
    t = _yaml('ship_direction_tournament.yaml')
    for k, v in DEFAULTS.items():
        assert k in t, k
        assert type(t[k]) is type(v), (k, t[k])
    assert t['side_lock_enabled'] is False
    assert check_params(True, t['side_lock_dist_m'], t['side_lock_min_deg'],
                        t['side_lock_max_deg'], t['side_lock_turn_deg'],
                        t['side_lock_wait_max_sec'], t['side_lock_confirm_frames'],
                        t['side_lock_release_margin_m']) == []


def test_node_default_off_no_logs():
    node, log = _construct({})
    for k, v in DEFAULTS.items():
        assert getattr(node, k) == v, k
    assert node.side_lock_enabled is False and node._side_lock_wait is None
    assert not [ln for ln in log.lines if '옆 잠금' in ln[1]], log.lines


def test_node_on_start_info():
    node, log = _construct(ON3)
    assert node.side_lock_enabled is True and isinstance(node._side_lock_wait, SideLockWait)
    assert isinstance(node._side_lock_filter, SideLockFilter)
    assert node._side_lock_filter.confirm_frames == 3
    start = log.of('INFO', '옆 잠금 켬')
    assert len(start) == 1
    for word in ('1.5 m', '45.0~135.0°', 'turn 15.0°', 'wait 10.0 s', '확인 3프레임',
                 '풀림 여유 0.3 m'):
        assert word in start[0], (word, start[0])
    assert not log.of('ERROR') and not log.of('WARN')


@pytest.mark.parametrize('bad', [
    {'side_lock_enabled': 'true'}, {'side_lock_dist_m': math.nan}, {'side_lock_dist_m': 0.0},
    {'side_lock_min_deg': 140.0}, {'side_lock_max_deg': 200.0}, {'side_lock_turn_deg': -5.0},
    {'side_lock_turn_deg': math.inf}, {'side_lock_wait_max_sec': -1.0},
    {'side_lock_wait_max_sec': math.nan}, {'side_lock_confirm_frames': 0},
    {'side_lock_confirm_frames': 3.0}, {'side_lock_confirm_frames': True},
    {'side_lock_release_margin_m': -0.1}, {'side_lock_release_margin_m': math.nan}])
def test_node_bad_param_error_once_and_off(bad):
    node, log = _construct(dict(ON, **bad))
    assert node.side_lock_enabled is False and node._side_lock_wait is None
    errs = log.of('ERROR', '옆 잠금')
    assert len(errs) == 1 and '끔' in errs[0], log.lines
    assert not log.of('INFO', '옆 잠금 켬')
    # 끈 노드는 꺼짐 노드와 같은 출력
    ref, _ = _node()
    node.wp_mode_cb(_ns(data=8))

    def run(clk):
        out = [_tick(node, clk, BESIDE_L, YAW_L) for _ in range(3)]
        clk.t = 1000.0
        assert out == [_tick(ref, clk, BESIDE_L, YAW_L) for _ in range(3)] == [120.0] * 3
    with_env(run)


# ======================================================================
# 꺼짐 불변 (골든)
# ======================================================================
@pytest.mark.parametrize('cname,over', GOLDEN_CONFIGS)
@pytest.mark.parametrize('mode', GOLDEN_MODES)
def test_off_golden_same_as_before(cname, over, mode):
    """옆 잠금 끔(기본) — 기존 스캔 묶음 + 잠금이 걸릴 한 바퀴 장면에서 출력·후진 상태가 변경 전과 같다."""
    outs, rev = golden_run(over, mode)
    want_o, want_r = GOLDEN[(cname, mode)]
    assert rle(outs) == want_o
    assert rle(rev) == want_r


@pytest.mark.parametrize('over', [{}, ESC, DWA, dict(DWA, **ESC)])
@pytest.mark.parametrize('mode', [8, 5, 0])
def test_off_explicit_equals_default_and_no_side_lock_logs(over, mode):
    """side_lock_enabled: false 를 직접 줘도 기본과 비트 동일, '옆 잠금' 로그 없음, 잠금 함수 안 부름."""
    real = {k: getattr(SD, k) for k in ('side_states', 'clip_candidates')}

    def boom(*a, **k):
        raise AssertionError('끔인데 옆 잠금 계산')
    SD.side_states = SD.clip_candidates = boom
    try:
        a = golden_run(over, mode)
        b = golden_run(dict(over, side_lock_enabled=False), mode)
    finally:
        for k, v in real.items():
            setattr(SD, k, v)
    assert a == b


def test_off_dwa_choose_called_exactly_as_before():
    """끔 + DWA 켬: choose 는 예전처럼 위치 인자 3개로만(allowed 없음) 불린다."""
    def run(clk):
        node, log = _node(DWA, wp_mode=8)
        calls = []
        real = node._dwa.choose

        def spy(*a, **k):
            calls.append((len(a), dict(k)))
            return real(*a, **k)
        node._dwa.choose = spy
        for sc, yaw in GOLDEN_SEQ:
            _tick(node, clk, sc, yaw)
        assert calls and all(c == (3, {}) for c in calls), calls[:3]
        assert not [ln for ln in log.lines if '옆 잠금' in ln[1]]
    with_env(run)


@pytest.mark.parametrize('over', [{}, ESC, DWA, dict(DWA, **ESC)])
@pytest.mark.parametrize('mode', [8, 0])
def test_on_but_unknown_or_clear_equals_off(over, mode):
    """켜도 옆 구역 '모름'(0~179.5° 스캔)·비어 있음·2.0 m 면 꺼짐과 출력·상태 같다(같은 기계)."""
    seq = ([(BUOY_SC, 0.0)] * 3 + [(CLOSE_SC, 0.0)] * 8 + [(NOGAP_SC, 0.0)] * 4
           + [(OPEN_SC, 40.0)] * 3 + [(LEFT_SC, 20.0)] * 3 + [(WALL_SC, 20.0)] * 40
           + [(FAR_L, 40.0)] * 4 + [(scan360([]), 320.0)] * 3 + [(CLOSE_SC, 0.0)] * 5)
    assert golden_run(over, mode, seq) == golden_run(dict(over, **ON), mode, seq)


def test_on_unknown_warns_once():
    def run(clk):
        node, log = _node(ON)
        for _ in range(5):
            _tick(node, clk, OPEN_SC, YAW_0)
        w = log.of('WARN', '옆 잠금')
        assert len(w) == 1 and '안 잠금' in w[0], log.lines
    with_env(run)


# ======================================================================
# 켬 — 잠긴 쪽으로 안 꺾음 (갭·DWA·비킴), 좌우 부호
# ======================================================================
@pytest.mark.parametrize('mode', [8, 5, 2, 7])
def test_on_gap_does_not_turn_into_locked_side(mode):
    """좌현 옆 1.0 m + 목표 좌 120: 끔 120(옆 배 쪽으로 꺾음) → 켬 95. 거울: 40 → 65.
    wp_mode 로 가르지 않는다(2·5·7·8 같음)."""
    def run(clk):
        off, _ = _node(wp_mode=mode)
        on, log = _node(ON, wp_mode=mode)
        assert _tick(off, clk, BESIDE_L, YAW_L) == 120.0
        assert _tick(on, clk, BESIDE_L, YAW_L) == 95.0
        assert on._side_prev is None              # 후보 1 개 → 편 기억 없음(기존 규칙)
        off2, _ = _node(wp_mode=mode)
        on2, _ = _node(ON, wp_mode=mode)
        assert _tick(off2, clk, BESIDE_R, YAW_R) == 40.0
        assert _tick(on2, clk, BESIDE_R, YAW_R) == 65.0
        # 잠긴 쪽 반대로 꺾는 건 그대로: 좌 잠김 + 목표 우 → 40
        on3, _ = _node(ON, wp_mode=mode)
        assert _tick(on3, clk, BESIDE_L, YAW_R) == 40.0
        assert len(log.of('INFO', '옆 잠금 걸림: 좌 1.00 m')) == 1
    with_env(run)


def test_on_both_sides_locked_keeps_heading_band():
    def run(clk):
        on, _ = _node(ON)
        assert _tick(on, clk, BOTH, YAW_L) == 95.0
        on2, _ = _node(ON)
        assert _tick(on2, clk, BOTH, YAW_R) == 65.0
    with_env(run)


def test_on_dwa_picks_inside_lock():
    """DWA 켬: 끄면 좌현 옆 배 쪽 160(최대 좌), 켜면 잠금 구간 안 — 같은 상태 새 planner 의 choose(allowed)."""
    def run(clk):
        off, _ = _node(DWA)
        on, _ = _node(dict(DWA, **ON))
        a_off = _tick(off, clk, BESIDE_L, YAW_L)
        a_on = _tick(on, clk, BESIDE_L, YAW_L)
        ang = ANG160
        dist = list(BESIDE_L.ranges[360:681])
        assert a_off == DwaPlanner('A').choose(ang, dist, 120.0) and a_off > 95.0
        assert a_on == DwaPlanner('A').choose(ang, dist, 120.0, allowed=(0.0, 95.0))
        assert a_on <= 95.0
        on_r, _ = _node(dict(DWA, **ON))
        a_r = _tick(on_r, clk, BESIDE_R, YAW_R)
        assert 65.0 <= a_r == DwaPlanner('A').choose(ang, list(BESIDE_R.ranges[360:681]), 40.0,
                                                     allowed=(65.0, 160.0))
    with_env(run)


def test_dwa_allowed_none_is_old_choose():
    """dwa.choose(allowed=None) == 인자 없는 호출. allowed 는 구간 밖 후보만 버린다."""
    p = DwaPlanner('A')
    inf = [math.inf] * 321
    for y in (40.0, 80.0, 120.0):
        assert p.choose(ANG160, inf, y, allowed=None) == p.choose(ANG160, inf, y)
    assert p.choose(ANG160, inf, 120.0, allowed=(0.0, 95.0)) == 95.0
    assert p.choose(ANG160, inf, 40.0, allowed=(65.0, 160.0)) == 65.0
    assert p.choose(ANG160, inf, 80.0, allowed=(81.0, 84.0)) is None    # 5° 격자에 후보 없음


def test_on_escape_respects_lock():
    """비킴이 잠긴 쪽이면 뺀다(→ 대기), 반대쪽이면 그대로 비킴."""
    def run(clk):
        off, _ = _node(ESC)
        assert _tick(off, clk, ESC_R, 20.0) == 20.0          # 끔: 우로 비킴(우현 옆 배 쪽)
        on, log = _node(dict(ESC, **ON))
        assert _tick(on, clk, ESC_R, 20.0) == STOP_HOLD      # 켬: 우 잠김 → 대기
        assert on.reverse_state == 'READY'
        on2, _ = _node(dict(ESC, **ON))
        assert _tick(on2, clk, ESC_L, 20.0) == 20.0          # 좌 잠김 + 우로 비킴 = 허용
    with_env(run)


# ======================================================================
# 켬 — 후보 0 개: 후진 대신 대기 → 길 열리면 재출발 / 상한 뒤 폴백
# ======================================================================
def test_on_no_option_waits_instead_of_reverse_then_resumes():
    def run(clk):
        off, _ = _node()
        assert _tick(off, clk, GAP_L, YAW_0) == 98.0         # 끔: 좌 틈으로 꺾음(옆 배 쪽)
        assert _tick(off, clk, BLOCK_L, YAW_0) == REVERSE_ANGLE
        on, log = _node(ON)
        outs = [_tick(on, clk, GAP_L, YAW_0) for _ in range(20)]
        assert outs == [STOP_HOLD] * 20
        assert on.reverse_state == 'READY'
        assert len(log.of('INFO', '50000 대기')) == 1
        assert not log.of('WARN', '폴백')
        # 상대 배가 지나감(옆 비고 앞 열림) → 바로 재출발
        a = _tick(on, clk, scan360([]), YAW_0)
        assert a == 80.0
        rs = log.of('INFO', '재출발')
        assert len(rs) == 1 and '2.0 s' in rs[0], rs
        # 다시 막힘 → 다시 대기(INFO 한 번 더)
        assert _tick(on, clk, GAP_L, YAW_0) == STOP_HOLD
        assert len(log.of('INFO', '50000 대기')) == 2
    with_env(run)


def test_on_wait_max_then_fallback_to_existing():
    """잠금이 지운 장면: 상한 10.0 s 까지 50000, 넘으면 WARN 1회 + 기존 동작(끈 노드와 같은 98)."""
    def run(clk):
        on, log = _node(ON)
        outs = [_tick(on, clk, GAP_L, YAW_0) for _ in range(100)]   # t0 … t0 + 9.9 s
        assert outs == [STOP_HOLD] * 100
        assert not log.of('WARN', '폴백')
        first, extra = STOP_HOLD, 0
        while first == STOP_HOLD and extra < 3:   # 10.0 s 언저리(0.1 s 누적 오차) 뒤 첫 폴백
            first = _tick(on, clk, GAP_L, YAW_0)
            extra += 1
        assert extra <= 2, extra
        fb = [first] + [_tick(on, clk, GAP_L, YAW_0) for _ in range(19)]
        clk.t = 1000.0
        ref, _ = _node()
        assert fb == [_tick(ref, clk, GAP_L, YAW_0) for _ in range(20)] == [98.0] * 20
        w = log.of('WARN', '폴백')
        assert len(w) == 1 and '10.0 s' in w[0]
        # 폴백 중 잠금이 더는 후보를 안 지움 → 다시 잠금(INFO 1회), 그 각은 잠금 구간 안
        assert _tick(on, clk, BESIDE_L, YAW_L) == 95.0
        assert len(log.of('INFO', '다시 잠금 지킴')) == 1
    with_env(run)


def test_on_fallback_turns_into_gap_like_before():
    """대기 상한 0.0 이면 바로 기존 동작 — 좌 틈 98 로 꺾는다(잠금 없을 때와 같다)."""
    def run(clk):
        on, log = _node(dict(ON, side_lock_wait_max_sec=0.0))
        assert _tick(on, clk, GAP_L, YAW_0) == 98.0
        assert len(log.of('WARN', '폴백')) == 1
        assert not log.of('INFO', '50000 대기')
    with_env(run)


@pytest.mark.parametrize('over', [{}, ESC, DWA, dict(DWA, **ESC)])
def test_on_dead_end_unrelated_to_lock_keeps_reverse(over):
    """정면 벽 + 옆 부표(잠금 없이도 후보 0 개) → 대기 안 함, 끈 노드와 같은 후진 시퀀스·상태."""
    seq = [(BLOCK_L, YAW_0)] * 45 + [(WALL_SC, 20.0)] * 3 + [(BLOCK_L, YAW_0)] * 10
    a = golden_run(over, 8, seq)
    b = golden_run(dict(over, **ON), 8, seq)
    assert a == b
    assert REVERSE_ANGLE in a[0] and STOP_HOLD in a[0]

    def run(clk):
        on, log = _node(dict(over, **ON))
        for sc, yaw in seq:
            _tick(on, clk, sc, yaw)
        assert log.of('INFO', '옆 잠금 걸림')            # 잠기긴 했다
        assert not log.of('INFO', '50000 대기') and not log.of('WARN', '폴백')
    with_env(run)


def test_on_confirm_frames_new_scans_only():
    """기본 3프레임: 새 스캔 1·2장은 안 잠김(120), 3장째 잠김(리미터로 105 → 95). 같은 스캔 재처리는 안 셈."""
    def run(clk):
        on, log = _node(ON3)
        outs = [_tick(on, clk, BESIDE_L, YAW_L) for _ in range(4)]
        assert outs == [120.0, 120.0, 105.0, 95.0], outs
        on2, _ = _node(ON3)
        _tick(on2, clk, BESIDE_L, YAW_L)
        outs = [_tick(on2, clk, BESIDE_L, YAW_L, fresh=False) for _ in range(5)]
        assert outs == [120.0] * 5                         # 같은 스캔 5틱 = 여전히 1프레임
        assert _tick(on2, clk, BESIDE_L, YAW_L) == 120.0   # 2프레임
        assert _tick(on2, clk, BESIDE_L, YAW_L) == 105.0   # 3프레임 → 잠김
    with_env(run)


def _beside(d):
    return scan360([(90.0, 120.0, d)])


def _gap(d):
    return scan360([(-80.0, 0.0, 1.6), (90.0, 120.0, d)])


def _dead(d):
    return scan360([(-80.0, 80.0, 0.5), (90.0, 120.0, d)])


def test_on_release_hysteresis():
    """잠긴 뒤 1.6 m(< 1.5 + 0.3) 은 안 풀림(95 유지), 1.8 m 에서 풀림(리미터로 110 → 120)."""
    def run(clk):
        on, _ = _node(ON3)
        for _ in range(4):
            _tick(on, clk, _beside(1.0), YAW_L)
        assert [_tick(on, clk, _beside(1.6), YAW_L) for _ in range(5)] == [95.0] * 5
        assert [_tick(on, clk, _beside(1.8), YAW_L) for _ in range(2)] == [110.0, 120.0]
    with_env(run)


def test_on_flicker_no_toggling():
    """1.4 ↔ 1.6 깜빡임: ① 잠금 전이면 끝까지 안 잠김(끈 노드와 같은 98)
    ② 잠긴 뒤면 계속 잠김(50000 만) ③ 막다른 곳이면 끈 노드와 같은 후진 시퀀스 — 260↔50000 번갈음 없음."""
    def run(clk):
        flick = [1.4, 1.6] * 10
        on, _ = _node(ON3)
        assert [_tick(on, clk, _gap(d), YAW_0) for d in flick] == [98.0] * 20
        on2, _ = _node(ON3)
        for _ in range(3):
            _tick(on2, clk, _gap(1.4), YAW_0)
        assert [_tick(on2, clk, _gap(d), YAW_0) for d in flick] == [STOP_HOLD] * 20
    with_env(run)
    seq = [(_dead(1.4), YAW_0), (_dead(1.6), YAW_0)] * 25
    assert golden_run(ON3, 8, seq) == golden_run({}, 8, seq)


def test_on_dwa_picked_matches_actual_choice():
    """잠금 중 DWA: 대기 틱은 '안 고름'(폴백 INFO 없음), 폴백 틱에 잠금 없는 DWA 가 고르면 picked True·INFO 없음."""
    def spy(real):
        def choose(*a, **k):
            if k.get('allowed') is not None:
                return None                    # 잠금 구간 안엔 없음
            return 120.0                       # 잠금 없으면 있음
        return choose

    def run(clk):
        for wait, want in ((10.0, STOP_HOLD), (0.0, 120.0)):
            on, log = _node(dict(DWA, **ON, side_lock_wait_max_sec=wait))
            assert _tick(on, clk, OPEN_SC, YAW_0) == 80.0 and on._dwa_picked
            on._dwa.choose = spy(on._dwa.choose)
            outs = [_tick(on, clk, GAP_L, YAW_0) for _ in range(3)]
            assert outs[-1] == want, (wait, outs)
            assert on._dwa_picked is (want == 120.0)
            assert not log.of('INFO', 'DWA 가능한 후보 없음'), log.lines
    with_env(run)


def test_on_level2_and_stop_hold_priority_unchanged():
    """레벨 2 → 50000(대기와 무관), 끝나면 대기를 처음부터. candidate 50000·(B) 각도는 잠금 안 탐."""
    def run(clk):
        on, log = _node(ON)
        for _ in range(50):
            _tick(on, clk, GAP_L, YAW_0)
        on.failsafe_level = 2
        assert _tick(on, clk, GAP_L, YAW_0) == STOP_HOLD
        on.failsafe_level = 0
        for _ in range(95):                                  # 대기 시간을 이어 세지 않는다
            assert _tick(on, clk, GAP_L, YAW_0) == STOP_HOLD
        assert len(log.of('INFO', '50000 대기')) == 2
        assert not log.of('WARN', '폴백')
        # (B) candidate 30°(좌) 는 옆 잠금 대상 아님 — 미션 노드 지시 그대로
        on2, _ = _node(ON, wp_mode=7)
        assert _tick(on2, clk, BESIDE_L, YAW_0, candidate=30.0) == 110.0
        assert _tick(on2, clk, BESIDE_L, YAW_0, candidate=STOP_HOLD) == STOP_HOLD
    with_env(run)


def test_logger_call_sites_one_severity_each():
    """§1-7: 옆 잠금 로그 호출 줄마다 심각도 하나(같은 줄에서 info/warn 섞지 않음, 변수에 담지 않음)."""
    src = _src()
    assert not re.search(r"=\s*self\.get_logger\(\)\.(info|warn|error)\b", src)
    body = src[src.index('def _side_lock_step'):src.index('def _dwa_leave')]
    calls = re.findall(r"get_logger\(\)\.(info|warn|error)\(", body)
    assert sorted(calls) == ['info', 'info', 'info', 'warn'], calls


# ======================================================================
# tools/sim/ssf_sim.py 수동 동기화 사본
# ======================================================================
def _load_sim():
    path = os.path.join(WS, 'tools', 'sim', 'ssf_sim.py')
    if not os.path.exists(path):
        pytest.skip(f'{path} 없음')
    try:
        spec = importlib.util.spec_from_file_location('ssf_sim_side_lock', path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    except ImportError as e:            # matplotlib·numpy 없는 환경
        pytest.skip(f'ssf_sim import 실패: {e}')
    return mod


def test_sim_copy_matches_node():
    """사본 상수 == 노드 기본값, 순간 판정 == zone_min, 연속 프레임 필터 == SideLockFilter."""
    S = _load_sim()
    assert S.SIDE_LOCK_ENABLED is False                     # 끔 = 노드 기본값
    got = (S.SIDE_LOCK_DIST_M, S.SIDE_LOCK_MIN_DEG, S.SIDE_LOCK_MAX_DEG, S.SIDE_LOCK_TURN_DEG,
           S.SIDE_LOCK_WAIT_MAX_SEC, S.SIDE_LOCK_CONFIRM_FRAMES, S.SIDE_LOCK_RELEASE_MARGIN_M)
    want = tuple(DEFAULTS[k] for k in list(DEFAULTS)[1:])
    assert got == want and type(S.SIDE_LOCK_CONFIRM_FRAMES) is int
    assert S.SCAN_ANGLE_MIN_DEG == -180.0 and S.SCAN_INC_DEG == INC_DEG
    a0s, incs = S.SCAN_ANGLE_MIN_DEG, S.SCAN_INC_DEG
    scenes = [BESIDE_L, BESIDE_R, BOTH, FAR_L, GAP_L, ESC_R, BLOCK_L, scan360([])]
    for sc in scenes:
        r, a0, inc = _rel_cells(sc)
        assert S.side_lock_raw(r, a0s, incs) == [
            zone_min(r, a0, inc, 80.0, 45.0, 135.0), zone_min(r, a0, inc, 80.0, -135.0, -45.0)]
    # 같은 장면 열(깜빡임·히스테리시스 포함)에서 필터 결과·허용 구간이 노드와 같다
    S.side_lock_reset()
    f = SideLockFilter(3, 0.3)
    seq = ([BESIDE_L] * 4 + [_beside(1.6)] * 2 + [_beside(1.8)] + [_beside(d) for d in
           (1.4, 1.6) * 4] + [_beside(1.4)] * 3 + [BOTH] * 3 + [BESIDE_R] * 4 + [scan360([])])
    try:
        for sc in seq:
            r, a0, inc = _rel_cells(sc)
            left, right = f.update(side_states(r, a0, inc, 80.0, 1.5, 45.0, 135.0), 1.5, True)
            want = (left or right,) + allowed_range(left, right, 80.0, 15.0)
            assert S.side_lock_range(r, a0s, incs) == want
    finally:
        S.side_lock_reset()


def _sim_on(S, **over):
    saved = {k: getattr(S, k) for k in ('SIDE_LOCK_ENABLED', 'SIDE_LOCK_CONFIRM_FRAMES',
                                        'SIDE_LOCK_WAIT_MAX_SEC')}
    S.SIDE_LOCK_ENABLED = True
    for k, v in over.items():
        setattr(S, k, v)
    S.side_lock_reset()
    return saved


def test_sim_copy_on_off():
    S = _load_sim()
    a0, inc = S.SCAN_ANGLE_MIN_DEG, S.SCAN_INC_DEG
    r = list(BESIDE_L.ranges)
    g = list(GAP_L.ranges)
    d = list(BLOCK_L.ranges)
    assert S.ship_direction_step(r, a0, inc, YAW_L, 3.0)[0] == 120.0
    saved = _sim_on(S)
    try:
        # 확인 3프레임: 1·2장째는 그대로, 3장째 잠김
        assert [S.ship_direction_step(r, a0, inc, YAW_L, 3.0)[0] for _ in range(3)] == \
            [120.0, 120.0, 95.0]
        # 잠금이 지운 장면 → 대기, 상한 뒤 기존(98)
        S.side_lock_reset()
        outs = [S.ship_direction_step(g, a0, inc, YAW_0, 3.0, now=t)[0]
                for t in (0.0, 0.1, 0.2, 10.2, 10.3)]
        assert outs[:2] == [98.0, 98.0] and outs[2:4] == [STOP_HOLD, STOP_HOLD], outs
        assert outs[4] == 98.0
        # 막다른 곳(잠금 무관) → 대기 없이 260
        S.side_lock_reset()
        assert [S.ship_direction_step(d, a0, inc, YAW_0, 3.0, now=0.1 * k)[0]
                for k in range(5)] == [260.0] * 5
        # 상한 0 → 바로 기존(노드와 같음, now 없어도)
        S.SIDE_LOCK_WAIT_MAX_SEC = 0.0
        S.side_lock_reset()
        assert [S.ship_direction_step(g, a0, inc, YAW_0, 3.0)[0] for _ in range(4)] == [98.0] * 4
    finally:
        for k, v in saved.items():
            setattr(S, k, v)
        S.side_lock_reset()
