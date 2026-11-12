#!/usr/bin/env python3
"""ship_back 위치유지 hold_source(gps|vision) 테스트 — ROS 없이 돈다.

실행: python3 src/ship_back/test/test_gps_hold.py
      tools/run_tests.sh ship_back

계약서 docs/문제와작업/계약서_GPS위치유지_20261113.md (사용자 결정 11-13):
  · 기본 hold_source = gps(코드 기본값 == yaml). vision 은 옵션 — 명시하면 지금과 출력이 비트 동일.
  · gps(mode 2 만): INIT 20000 1 s → SEARCH 20000 → /goal_distance(쓸 수 있는 값) < gps_enter_m 이면 HOLD
    → HOLD 는 50000(정지) / 거리 > gps_resume_m 이면 < gps_enter_m 까지 20000(재접근, 히스테리시스)
    → gps_keep_radius_m(4.5) 안 연속 gps_hold_time_sec(6) 이면 DONE(20000). 180·조향각은 절대 안 낸다.
  · 쓸 수 있는 거리 = 유한 ≥ 0, goal_dist_stale_sec 안, 모드 진입 1.0 s 뒤에 받은 값.
  · 거리 없음(HOLD) → 50000, unknown_giveup_sec 뒤 포기 DONE. hold_max_sec 그대로.
  · search_behavior:=hold 의 mode 3 은 vision 경로 그대로. vision 이면 /goal_distance 구독 없음.
  · 시작 가드: gps_enter_m ≥ north 3.5, resume ≥ enter, keep > resume, hold_source 값 → ERROR + 기본값.
노드 쪽은 test_fused_match.py 와 같은 rclpy 스텁으로 실제 ShipBack 을 돈다.
"""

import importlib.util
import math
import os
import re
import sys
import types

PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
sys.path.insert(0, PKG)
NORTH_YAML = os.path.join(PKG, '..', 'north_goal_angle', 'config', 'north_goal_angle.yaml')
NODE_SRC = os.path.join(PKG, 'ship_back', 'ship_back.py')

from ship_back.hold_logic import (  # noqa: E402
    check_gps_hold_params, check_hold_source, goal_dist_usable, gps_hold_action, gps_hold_applies,
    DEFAULT_HOLD_SOURCE, DEFAULT_GPS_ENTER_M, DEFAULT_GPS_RESUME_M, DEFAULT_GPS_KEEP_RADIUS_M,
    DEFAULT_GPS_HOLD_TIME_SEC, DEFAULT_GOAL_DIST_STALE_SEC, GOAL_DIST_ENTRY_GUARD_SEC,
    NORTH_ARRIVE_HOLD_M)

NAN, INF = float('nan'), float('inf')
STOP, REL = 50000.0, 20000.0


def _levels(issues):
    return [lv for lv, _ in issues]


# ─────────────── 순수 함수 ───────────────
def test_constants():
    assert DEFAULT_HOLD_SOURCE == 'gps'
    assert (DEFAULT_GPS_ENTER_M, DEFAULT_GPS_RESUME_M, DEFAULT_GPS_KEEP_RADIUS_M) == (3.5, 4.0, 4.5)
    assert (DEFAULT_GPS_HOLD_TIME_SEC, DEFAULT_GOAL_DIST_STALE_SEC) == (6.0, 2.0)
    assert GOAL_DIST_ENTRY_GUARD_SEC == 1.0 and NORTH_ARRIVE_HOLD_M == 3.5


def test_hold_source_values():
    assert check_hold_source('gps') == ('gps', None)
    assert check_hold_source('vision') == ('vision', None)
    assert check_hold_source(' Vision ') == ('vision', None)
    for bad in ('lidar', '', None, 3, ['gps']):
        v, err = check_hold_source(bad)
        assert v == 'gps' and err is not None and 'vision|gps' in err, (bad, v, err)


def test_gps_applies_mode_2_only():
    assert gps_hold_applies('gps', 2) is True
    for m in (3, 0, 1, 5, 7, 8, -1):
        assert gps_hold_applies('gps', m) is False, m
    assert gps_hold_applies('vision', 2) is False


def test_action_hysteresis_boundaries():
    # 정지 중: resume(4.0) 초과일 때만 재접근. 4.0 정확히 = 정지 유지.
    assert gps_hold_action(3.0, False, 3.5, 4.0) == 'stop'
    assert gps_hold_action(4.0, False, 3.5, 4.0) == 'stop'
    assert gps_hold_action(4.0001, False, 3.5, 4.0) == 'approach'
    # 재접근 중: enter(3.5) 미만일 때만 정지. 3.5 정확히 = 재접근 유지.
    assert gps_hold_action(3.9, True, 3.5, 4.0) == 'approach'
    assert gps_hold_action(3.5, True, 3.5, 4.0) == 'approach'
    assert gps_hold_action(3.4999, True, 3.5, 4.0) == 'stop'
    assert gps_hold_action(None, True, 3.5, 4.0) == 'unknown'
    assert gps_hold_action(None, False, 3.5, 4.0) == 'unknown'


def test_usable_guard_stale_nonfinite():
    enter = 100.0
    assert goal_dist_usable(3.0, 101.0, enter, 101.5, 2.0) == 3.0          # guard 경계(== 1.0) 사용
    assert goal_dist_usable(3.0, 100.99, enter, 101.5, 2.0) is None       # guard 안 → 무시
    assert goal_dist_usable(0.0, 100.2, enter, 100.3, 2.0) is None        # 직전 WP ≈0 m 오판 방지
    assert goal_dist_usable(3.0, 101.0, enter, 103.0, 2.0) == 3.0          # stale 경계(== 2.0) 신선
    assert goal_dist_usable(3.0, 101.0, enter, 103.01, 2.0) is None
    assert goal_dist_usable(0.0, 101.0, enter, 101.0, 2.0) == 0.0          # 0 은 유효
    for bad in (NAN, INF, -INF, -0.1, None):
        assert goal_dist_usable(bad, 101.0, enter, 101.5, 2.0) is None, bad
    assert goal_dist_usable(3.0, None, enter, 101.5, 2.0) is None
    assert goal_dist_usable(3.0, 101.0, None, 101.5, 2.0) is None


def test_param_guard_defaults_silent():
    out = check_gps_hold_params(3.5, 4.0, 4.5, 6.0, 2.0)
    assert out[:5] == (3.5, 4.0, 4.5, 6.0, 2.0) and out[5] == [], out


def test_param_guard_band_errors_restore_all_three():
    for e, r, k in ((3.4, 4.0, 4.5),     # enter < north 3.5
                    (3.5, 3.4, 4.5),     # resume < enter
                    (3.5, 4.0, 4.0),     # keep == resume
                    (3.5, 4.0, 3.9),     # keep < resume
                    (NAN, 4.0, 4.5), (3.5, INF, 4.5), (3.5, 4.0, 'x'), (None, 4.0, 4.5)):
        out = check_gps_hold_params(e, r, k, 6.0, 2.0)
        assert out[:3] == (3.5, 4.0, 4.5), (e, r, k, out)
        assert _levels(out[5]) == ['ERROR'], (e, r, k, out[5])
        assert 'gps_enter_m' in out[5][0][1], out[5]
    # 경계: enter == north, resume == enter, keep 이 resume 보다 조금 큼 → 정상
    out = check_gps_hold_params(3.5, 3.5, 3.51, 6.0, 2.0)
    assert out[:3] == (3.5, 3.5, 3.51) and out[5] == [], out
    out = check_gps_hold_params(4.0, 4.5, 5.0, 6.0, 2.0)
    assert out[:3] == (4.0, 4.5, 5.0) and out[5] == [], out


def test_param_guard_time_and_stale():
    for bad in (0.0, -1.0, NAN, INF, 'x'):
        out = check_gps_hold_params(3.5, 4.0, 4.5, bad, 2.0)
        assert out[3] == 6.0 and _levels(out[5]) == ['ERROR'], (bad, out)
        out = check_gps_hold_params(3.5, 4.0, 4.5, 6.0, bad)
        assert out[4] == 2.0 and _levels(out[5]) == ['ERROR'], (bad, out)


# ─────────────── 노드(스텁) ───────────────
OVERRIDE = {}
DECLARED = {}
EVENTS = []      # ('param', 이름) / ('sub', 토픽)


class _Param:
    def __init__(self, v):
        self.value = v


class _Logger:
    def __init__(self):
        self.lines = []

    def info(self, m, **kw):
        self.lines.append(('INFO', m))

    def warn(self, m, **kw):
        self.lines.append(('WARN', m))

    def error(self, m, **kw):
        self.lines.append(('ERROR', m))


class _Pub:
    def __init__(self):
        self.values = []

    def publish(self, msg):
        self.values.append(msg.data)


class _Node:
    def __init__(self, name):
        self._logger = _Logger()
        self.subs = []
        EVENTS.clear()

    def declare_parameter(self, name, default):
        DECLARED[name] = default
        EVENTS.append(('param', name))
        return _Param(OVERRIDE.get(name, default))

    def create_publisher(self, typ, topic, qos):
        return _Pub()

    def create_subscription(self, typ, topic, cb, qos):
        self.subs.append((typ, topic, cb, qos))
        EVENTS.append(('sub', topic))
        return None

    def create_timer(self, *a, **k):
        return None

    def get_logger(self):
        return self._logger


class _Msg:
    def __init__(self, data=0.0):
        self.data = data


class _F32(_Msg):
    pass


class _QoS:
    def __init__(self, **k):
        self.kw = k


def _enum(n):
    return type(n, (), {'BEST_EFFORT': 'BEST_EFFORT', 'RELIABLE': 'RELIABLE', 'KEEP_LAST': 'KEEP_LAST'})


def _load_node_module():
    mods = {
        'rclpy': dict(init=lambda *a, **k: None, ok=lambda: True, shutdown=lambda: None),
        'rclpy.node': dict(Node=_Node),
        'rclpy.executors': dict(MultiThreadedExecutor=object),
        'rclpy.qos': dict(QoSProfile=_QoS, ReliabilityPolicy=_enum('R'), HistoryPolicy=_enum('H')),
        'std_msgs': {},
        'std_msgs.msg': dict(Float32=_F32, Int32=_Msg, Float32MultiArray=_Msg),
        'sensor_msgs': {},
        'sensor_msgs.msg': dict(LaserScan=object),
    }
    saved = {k: sys.modules.get(k) for k in mods}
    try:
        for name, attrs in mods.items():
            m = types.ModuleType(name)
            for k, v in attrs.items():
                setattr(m, k, v)
            sys.modules[name] = m
        spec = importlib.util.spec_from_file_location('ship_back_gps_hold_under_test', NODE_SRC)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    finally:
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v
    return mod


SB = _load_node_module()
SB.Float32 = _F32


class _Clock:
    def __init__(self, t=1000.0):
        self.t = t

    def monotonic(self):
        return self.t


CLOCK = _Clock()
SB.time = CLOCK
T0 = 1000.0


class _Scan:
    """1° 간격 −180~179°. beams = {LiDAR 각: 거리}, ring = 전 방위 거리(None = 없음)."""

    def __init__(self, beams=None, ring=None):
        self.angle_min = math.radians(-180.0)
        self.angle_increment = math.radians(1.0)
        self.ranges = [float(ring) if ring is not None else INF] * 360
        for a, d in (beams or {}).items():
            self.ranges[int(round(a + 180.0))] = float(d)


def _make(**over):
    OVERRIDE.clear()
    OVERRIDE.update(over)
    try:
        return SB.ShipBack()
    finally:
        OVERRIDE.clear()


def _lines(n, lv):
    return [m for x, m in n.get_logger().lines if x == lv]


class _Run:
    """시각 k × 0.1 s 로 한 틱씩. 입력(모드·비전·스캔·GPS 거리)을 실제 콜백으로 넣고 발행·상태를 기록."""

    def __init__(self, node):
        self.n = node
        self.k = 0
        self.out = []          # 틱마다 그 틱에 낸 값 목록
        self.states = []

    def t(self):
        return T0 + self.k * 0.1

    def tick(self, mode=None, angle=None, scan=None, goal=None):
        CLOCK.t = self.t()
        if goal is not None and hasattr(self.n, 'goal_distance_cb'):
            self.n.goal_distance_cb(_F32(float(goal)))
        if mode is not None:
            self.n.wp_cb(_Msg(mode))
        self.n.angle_cb(_F32(10000.0 if angle is None else float(angle)))
        if scan is not None:
            self.n.scan_cb(scan)
        before = len(self.n.pub_candidate.values)
        self.n.timer_cb()
        self.out.append(list(self.n.pub_candidate.values[before:]))
        self.states.append(self.n.state)
        self.k += 1

    def run(self, n_ticks, **kw):
        for _ in range(n_ticks):
            self.tick(**kw)

    def flat(self):
        return [v for o in self.out for v in o]


def _rle(seq):
    out = []
    for v in seq:
        if out and out[-1][0] == v:
            out[-1][1] += 1
        else:
            out.append([v, 1])
    return [tuple(x) for x in out]


# ─────────── vision 명시 = 지금(수정 전) 동작과 출력 비트 동일 ───────────
def _vision_scenario(node):
    """mode 2 → 3 를 오가는 vision 경로 전 분기(전진·클램프·후진·데드밴드·LiDAR 보조·놓침 창·성공·
    2→2 래치·2→3 리셋·unknown 포기·3→3 래치). GPS 거리도 같이 넣는다 — vision 이면 무시돼야 한다."""
    r = _Run(node)
    ring6 = _Scan(ring=6.0)
    r.tick(mode=2, scan=ring6, goal=0.0)                      # 진입(직전 WP 0 m 가 와도)
    r.run(9, scan=ring6, goal=1.0)                            # INIT 1 s
    r.run(20, scan=ring6, goal=2.0)                           # SEARCH 비전 없음 → 20000
    r.run(10, angle=10.0, scan=_Scan({90.0: 6.0}), goal=2.0)  # 전진 조향 10
    r.run(10, angle=95.0, scan=_Scan({175.0: 6.0}), goal=2.0)   # 클램프 80
    r.run(10, angle=-20.0, scan=_Scan({60.0: 1.5}), goal=2.0)   # 후진 180
    r.run(10, scan=ring6, goal=2.0)                           # 비전 끊김 → LiDAR 보조 → 놓침
    r.run(15, scan=ring6, goal=2.0)                           # 재획득 정지 창 50000
    r.run(60, angle=0.0, scan=_Scan({80.0: 2.5}), goal=2.0)   # 재획득 → 데드밴드 → 5 s 성공 DONE
    r.tick(mode=2, angle=0.0, scan=_Scan({80.0: 2.5}), goal=2.0)   # 2→2 래치
    r.run(5, angle=0.0, scan=_Scan({80.0: 2.5}), goal=2.0)
    r.tick(mode=3, angle=0.0, scan=_Scan({80.0: 3.5}), goal=2.0)   # 2→3 리셋
    r.run(15, angle=0.0, scan=_Scan({80.0: 3.5}), goal=2.0)
    r.run(110, angle=0.0, scan=_Scan({170.0: 3.5}), goal=2.0)  # 거리 unknown → 10 s 포기
    r.tick(mode=3, angle=0.0, scan=_Scan({80.0: 2.5}), goal=2.0)   # 3→3 래치
    r.run(5, angle=0.0, scan=_Scan({80.0: 2.5}), goal=2.0)
    return r


# 수정 전 코드(5dd737a)로 같은 시나리오를 돌려 얻은 값(발행값 RLE, 상태 RLE, 로그 심각도 수).
# None = 그 틱 발행 없음(상태 전이 틱).
VISION_GOLDEN_OUT = [
    ((20000.0,), 30), (None, 1), ((10.0,), 9), ((80.0,), 10), ((180.0,), 10), ((330.0,), 20), (None, 1),
    ((50000.0,), 4), (None, 1), ((50000.0,), 50), (None, 1), ((20000.0,), 25), (None, 1), ((0.0,), 4),
    ((50000.0,), 100), (None, 1), ((20000.0,), 15)]
VISION_GOLDEN_STATES = [
    ('INIT', 10), ('SEARCH', 20), ('HOLD', 50), ('SEARCH', 5), ('HOLD', 51), ('DONE', 15), ('INIT', 10),
    ('SEARCH', 1), ('HOLD', 105), ('DONE', 16)]
VISION_GOLDEN_LEVELS = {'INFO': 10, 'WARN': 121, 'ERROR': 1}


def _vision_signature(r):
    outs = [tuple(o) if o else None for o in r.out]
    lv = {x: sum(1 for y, _ in r.n.get_logger().lines if y == x) for x in ('INFO', 'WARN', 'ERROR')}
    return _rle(outs), _rle(r.states), lv


def test_vision_explicit_bit_identical_to_before():
    r = _vision_scenario(_make(hold_source='vision', active_wp_modes=[2, 3]))
    out, states, lv = _vision_signature(r)
    assert out == VISION_GOLDEN_OUT, out
    assert states == VISION_GOLDEN_STATES, states
    # 시작 INFO 한 줄 내용만 늘었다(hold_source 표시) — 줄 수·심각도는 그대로
    assert lv == VISION_GOLDEN_LEVELS, lv


def test_vision_no_goal_distance_subscription():
    n = _make(hold_source='vision')
    assert n.hold_source == 'vision'
    assert [s for s in n.subs if s[1] == '/goal_distance'] == [], n.subs
    assert _lines(n, 'ERROR') == []


def test_gps_default_subscribes_goal_distance_reliable_d5():
    n = _make()
    assert n.hold_source == 'gps'
    subs = [s for s in n.subs if s[1] == '/goal_distance']
    assert len(subs) == 1 and subs[0][0] is _F32, n.subs
    assert subs[0][3].kw == dict(depth=5, reliability='RELIABLE'), subs[0][3].kw
    assert subs[0][2] == n.goal_distance_cb


def test_params_declared_before_subscriptions():
    _make()
    first_sub = next(i for i, e in enumerate(EVENTS) if e[0] == 'sub')
    for p in ('hold_source', 'gps_enter_m', 'gps_resume_m', 'gps_keep_radius_m',
              'gps_hold_time_sec', 'goal_dist_stale_sec', 'unknown_giveup_sec', 'hold_max_sec'):
        assert EVENTS.index(('param', p)) < first_sub, (p, EVENTS)


def test_defaults_equal_yaml_and_hold_logic():
    import yaml
    _make()
    with open(os.path.join(PKG, 'config', 'ship_back.yaml'), encoding='utf-8') as f:
        y = yaml.safe_load(f)['/ship_back']['ros__parameters']
    want = {'hold_source': DEFAULT_HOLD_SOURCE, 'gps_enter_m': DEFAULT_GPS_ENTER_M,
            'gps_resume_m': DEFAULT_GPS_RESUME_M, 'gps_keep_radius_m': DEFAULT_GPS_KEEP_RADIUS_M,
            'gps_hold_time_sec': DEFAULT_GPS_HOLD_TIME_SEC, 'goal_dist_stale_sec': DEFAULT_GOAL_DIST_STALE_SEC}
    for k, v in want.items():
        assert DECLARED[k] == v and y[k] == v, (k, DECLARED[k], y.get(k), v)
        assert type(DECLARED[k]) is type(y[k]) is type(v), (k, type(DECLARED[k]), type(y[k]))  # noqa: E721
    # 기존 vision·포기 값 불변
    for k, v in (('keep_radius_m', 5.0), ('hold_time_sec', 5.0), ('unknown_giveup_sec', 10.0),
                 ('hold_max_sec', 60.0)):
        assert DECLARED[k] == v and y[k] == v, k


def test_cross_package_north_radius_le_gps_enter():
    """north arrive_radius_hold_modes_m ≤ ship_back gps_enter_m (그래야 정지 전에 dwell 로 넘어가지 않는다)."""
    import yaml
    with open(os.path.join(PKG, 'config', 'ship_back.yaml'), encoding='utf-8') as f:
        y = yaml.safe_load(f)['/ship_back']['ros__parameters']
    if not os.path.exists(NORTH_YAML):
        return
    with open(NORTH_YAML, encoding='utf-8') as f:
        raw = yaml.safe_load(f)
    north_r = float(next(iter(raw.values()))['ros__parameters']['arrive_radius_hold_modes_m'])
    assert north_r == NORTH_ARRIVE_HOLD_M, (north_r, NORTH_ARRIVE_HOLD_M)
    assert north_r <= y['gps_enter_m'] <= y['gps_resume_m'] < y['gps_keep_radius_m'], (north_r, y)


# ─────────── gps 노드 동작 ───────────
def _gps_run(**over):
    return _Run(_make(**over))


def test_gps_normal_flow_init_search_stop_6s_done():
    r = _gps_run()
    vis = dict(angle=15.0, scan=_Scan(ring=1.2))          # 비전·LiDAR 가 있어도 mode 2 는 GPS 만
    r.tick(mode=2, goal=0.1, **vis)                       # 진입 틱 직전 WP 0.1 m
    r.run(4, goal=0.1, **vis)                             # 가드 안(0.4 s) — 무시
    r.run(7, goal=20.0, **vis)                            # INIT 1 s
    assert set(r.flat()) == {REL} and r.n.state == 'SEARCH', (r.flat(), r.n.state)
    r.run(30, goal=10.0, **vis)                           # 접근
    assert r.n.state == 'SEARCH' and set(r.flat()) == {REL}
    k_hold = r.k
    r.tick(goal=3.4, **vis)                               # < 3.5 → HOLD, 같은 틱에 50000
    assert r.n.state == 'HOLD' and r.out[-1] == [STOP], (r.n.state, r.out[-1])
    r.run(58, goal=3.9, **vis)                            # 5.8 s — resume(4.0) 안이라 정지 유지
    assert r.n.state == 'HOLD' and set(sum(r.out[k_hold:], [])) == {STOP}
    r.run(3, goal=3.9, **vis)                             # 6.1 s 까지 → DONE, 20000 (정확한 경계는 순수 테스트)
    assert r.n.state == 'DONE' and r.out[-1] == [REL], (r.n.state, r.out[-1])
    assert any('달성' in m for m in _lines(r.n, 'INFO'))
    r.run(10, goal=4.4, **vis)
    assert r.n.state == 'DONE' and set(sum(r.out[-10:], [])) == {REL}
    assert all(len(o) == 1 for o in r.out), "GPS 경로는 틱마다 한 번 발행"
    assert set(r.flat()) <= {REL, STOP}
    assert _lines(r.n, 'ERROR') == [] and _lines(r.n, 'WARN') == []


def test_gps_entry_guard_ignores_early_values():
    """진입 뒤 1.0 s 안에 받은 0 m 는 HOLD 로 안 넘긴다(INIT 이 끝나도 새 값 전까지 SEARCH 20000)."""
    r = _gps_run()
    r.tick(mode=2, goal=0.0)
    r.run(9, goal=0.0)                                    # 0.9 s 까지 0 m (가드 안)
    r.run(5)                                              # 값 없음 — 마지막 0 m 는 가드 안 수신
    assert r.n.state == 'SEARCH' and set(r.flat()) == {REL}, (r.n.state, r.flat())
    r.tick(goal=0.0)                                      # 1.4 s 수신 → 사용
    assert r.n.state == 'HOLD' and r.out[-1] == [STOP]


def test_gps_guard_boundary_exactly_1s_used():
    r = _gps_run()
    r.tick(mode=2, goal=9.0)
    r.run(9, goal=9.0)                                    # INIT 0.0~0.9
    r.tick(goal=3.0)                                      # t = 진입 + 1.0 → 사용, INIT→SEARCH 틱
    assert r.n.state == 'SEARCH'
    r.tick()                                              # SEARCH 에서 그 값 → HOLD
    assert r.n.state == 'HOLD' and r.out[-1] == [STOP]


def _in_hold(**over):
    r = _gps_run(**over)
    r.tick(mode=2, goal=9.0)
    r.run(14, goal=9.0)
    r.tick(goal=3.0)
    assert r.n.state == 'HOLD'
    return r


def test_gps_hysteresis_resume_and_reenter():
    r = _in_hold()
    seq = [(3.9, STOP), (4.0, STOP), (4.1, REL), (4.4, REL), (3.6, REL), (3.5, REL), (3.4, STOP),
           (3.9, STOP), (4.2, REL), (3.0, STOP)]
    for d, want in seq:
        r.tick(goal=d)
        assert r.out[-1] == [want], (d, r.out[-1])
    assert r.n.state == 'HOLD'
    assert sum(m.startswith('WP') and '재접근' in m for m in _lines(r.n, 'INFO')) == 2, r.n.get_logger().lines


def test_gps_keep_radius_resets_timer():
    r = _in_hold()
    r.run(58, goal=3.0)                                   # 5.8 s
    assert r.n.state == 'HOLD'
    r.tick(goal=4.6)                                      # 반경 밖 → 0 부터 (재접근 20000)
    assert r.n.state == 'HOLD' and r.out[-1] == [REL]
    r.run(58, goal=3.0)                                   # 새로 5.7 s
    assert r.n.state == 'HOLD' and r.out[-1] == [STOP]
    r.run(3, goal=3.0)                                    # 6 s 넘김
    assert r.n.state == 'DONE'


def test_gps_keep_radius_boundary_inside_while_approaching():
    """4.5 m = 반경 안(경계 포함) — resume(4.0) 밖이라 재접근(20000) 중이어도 판정 시간은 쌓인다."""
    r = _in_hold()
    r.run(58, goal=4.5)
    assert r.n.state == 'HOLD' and r.out[-1] == [REL]
    r.run(4, goal=4.5)
    assert r.n.state == 'DONE'


def test_gps_stale_stop_then_giveup_10s():
    r = _in_hold()
    r.run(20)                                             # 수신 없음 → 2.0 s 뒤 stale
    k = r.k
    r.run(99)
    assert r.n.state == 'HOLD' and set(sum(r.out[k:], [])) == {STOP}
    assert _lines(r.n, 'ERROR') == []
    assert len([m for m in _lines(r.n, 'WARN') if '/goal_distance' in m]) == 1, r.n.get_logger().lines
    r.run(2)
    assert r.n.state == 'DONE' and r.out[-1] == [REL]
    e = _lines(r.n, 'ERROR')
    assert len(e) == 1 and '포기' in e[0], e
    r.run(5, goal=3.0)                                    # 거리가 돌아와도 DONE 래치
    assert r.n.state == 'DONE' and len(_lines(r.n, 'ERROR')) == 1


def test_gps_nonfinite_negative_distance_is_unknown():
    for bad in (NAN, INF, -1.0):
        r = _in_hold()
        r.run(10, goal=bad)
        assert r.n.state == 'HOLD' and r.n.unknown_t0 is not None, bad
        assert set(r.flat()) <= {REL, STOP}


def test_gps_unknown_then_back_resumes_from_stop():
    """unknown 동안 정지했으므로 거리가 돌아오면 '정지' 기준(> resume 이어야 재접근)."""
    r = _in_hold()
    r.tick(goal=4.2)
    assert r.out[-1] == [REL]
    r.run(25)                                             # stale → 50000
    assert r.out[-1] == [STOP]
    r.tick(goal=3.8)
    assert r.out[-1] == [STOP] and r.n.unknown_t0 is None


def test_gps_hold_max_giveup():
    r = _in_hold()
    for _ in range(60):
        r.run(9, goal=3.0)
        r.tick(goal=4.7)                                  # 1 s 마다 반경 밖 → 성공 못 함
    assert r.n.state == 'DONE'
    e = _lines(r.n, 'ERROR')
    assert len(e) == 1 and 'HOLD 총' in e[0] and '포기' in e[0], e


def test_gps_never_emits_reverse_or_angle():
    r = _gps_run()
    lidar_close = _Scan(ring=1.0)                         # vision 경로였다면 후진(180)
    r.tick(mode=2, angle=0.0, scan=lidar_close, goal=0.0)
    for k in range(400):
        g = None if (k // 37) % 3 == 2 else 2.0 + 3.0 * abs(math.sin(k * 0.05))
        r.tick(angle=(k % 7) * 30.0 - 90.0, scan=lidar_close, goal=g)
    assert set(r.flat()) <= {REL, STOP}, sorted(set(r.flat()))
    assert 180.0 not in r.flat()


def test_gps_search_does_not_use_vision_or_lidar():
    r = _gps_run()
    r.tick(mode=2, angle=0.0, scan=_Scan(ring=2.0))
    r.run(30, angle=0.0, scan=_Scan(ring=2.0))           # 거리 없음 → SEARCH 20000
    assert r.n.state == 'SEARCH' and set(r.flat()) == {REL}
    assert r.n.last_seen_t is None


def test_gps_2_to_2_latch_and_2_to_3_reset_vision_path():
    r = _gps_run(active_wp_modes=[2, 3])
    r.tick(mode=2, goal=9.0)
    r.run(14, goal=9.0)
    r.run(63, goal=3.0)
    assert r.n.state == 'DONE'
    r.tick(mode=2, goal=3.0)                              # 2→2 재수신 = 래치
    assert r.n.state == 'DONE' and r.out[-1] == [REL]
    beam = _Scan({80.0 + 20.0: 3.5})
    r.tick(mode=3, angle=20.0, scan=beam, goal=3.0)      # 2→3 리셋
    assert r.n.state == 'INIT'
    r.run(11, angle=20.0, scan=beam, goal=1.0)           # INIT 1 s → SEARCH
    r.run(3, angle=20.0, scan=beam, goal=1.0)            # 비전으로 HOLD
    assert r.n.state == 'HOLD' and r.out[-1] == [20.0], (r.n.state, r.out[-1])   # mode 3 = vision 전진 조향
    r.n.hold_timer.reset()
    r.n.hold_timer.update(3.5, CLOCK.t - 5.0)
    r.tick(angle=20.0, scan=beam)
    assert r.n.state == 'DONE'
    r.tick(mode=3, angle=20.0, scan=beam)                 # 3→3 래치
    assert r.n.state == 'DONE' and r.out[-1] == [REL]


def test_gps_mode_3_after_new_entry_back_to_2_resets_guard():
    """3→2 재진입이면 가드·HoldTimer·재접근 기억이 새로 시작."""
    r = _gps_run(active_wp_modes=[2, 3])
    r.tick(mode=3)
    r.run(5)
    r.tick(mode=2, goal=0.0)
    assert r.n.state == 'INIT' and r.n.mode_enter_t == CLOCK.t
    r.run(14)
    assert r.n.state == 'SEARCH'                          # 진입 틱의 0 m 는 가드 안


# ─────────── 시작 가드(노드) ───────────
def _start_line(n):
    return [m for m in _lines(n, 'INFO') if '위치유지: hold_dist=' in m]


def test_node_defaults_silent_single_start_line():
    n = _make()
    assert n.get_logger().lines and len(n.get_logger().lines) == 1, n.get_logger().lines
    s = _start_line(n)[0]
    assert 'hold_source=gps' in s and '3.5' in s and '4.5' in s and '6.0' in s, s


def test_node_band_error_once_defaults():
    for over in (dict(gps_enter_m=3.0), dict(gps_resume_m=3.0), dict(gps_keep_radius_m=4.0)):
        n = _make(**over)
        assert (n.gps_enter_m, n.gps_resume_m, n.gps_keep_radius_m) == (3.5, 4.0, 4.5), over
        assert len(_lines(n, 'ERROR')) == 1 and _lines(n, 'WARN') == [], (over, n.get_logger().lines)
        assert n.gps_hold_timer.keep_radius == 4.5 and n.gps_hold_timer.hold_time == 6.0


def test_node_bad_hold_source_error_gps():
    n = _make(hold_source='lidar')
    assert n.hold_source == 'gps'
    e = _lines(n, 'ERROR')
    assert len(e) == 1 and 'lidar' in e[0], e
    assert len([s for s in n.subs if s[1] == '/goal_distance']) == 1
    assert 'hold_source=gps' in _start_line(n)[0]


def test_node_vision_skips_gps_guard_lines_but_keeps_values():
    """vision 이어도 gps_* 값은 검사·보관(롤백 뒤 다시 gps 로 바꿀 때 같은 기준)."""
    n = _make(hold_source='vision', gps_enter_m=3.0)
    assert n.gps_enter_m == 3.5 and len(_lines(n, 'ERROR')) == 1


def test_logger_not_stored_in_variable():
    src = open(NODE_SRC, encoding='utf-8').read()
    assert re.search(r'=\s*self\.get_logger\(\)\s*$', src, re.M) is None
    assert 'time.time()' not in src


if __name__ == '__main__':
    if '--golden' in sys.argv:
        r = _vision_scenario(_make(hold_source='vision', active_wp_modes=[2, 3]))
        o, s, lv = _vision_signature(r)
        print('VISION_GOLDEN_OUT =', o)
        print('VISION_GOLDEN_STATES =', s)
        print('VISION_GOLDEN_LEVELS =', lv)
        sys.exit(0)
    fails = 0
    for name, fn in list(globals().items()):
        if name.startswith('test_') and callable(fn):
            try:
                fn()
                print(f"  ok  {name}")
            except (AssertionError, AttributeError, TypeError, ValueError, KeyError) as e:
                fails += 1
                print(f"  FAIL {name}: {e!r}")
    sys.exit(1 if fails else 0)
