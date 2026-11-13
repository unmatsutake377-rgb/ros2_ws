#!/usr/bin/env python3
"""ship_turn 거리 출처(distance_source: lidar|fill|fused) · /fused_objects 매칭 테스트 — ROS 없이 돈다.

실행: python3 src/ship_turn/test/test_fused_match.py
      tools/run_tests.sh ship_turn

계약서 docs/문제와작업/계약서_인식융합_20261112.md §4·§10:
  /fused_objects = Float32MultiArray [VER=1, STRIDE=8, N, N × (color_id, bearing_deg, range_m, conf, src_id,
  age_s, track_id, flags)]. 매칭 = 부표 방위 ±fused_match_deg 안, 색 알면 같은 색(/image_color) → 방위 최근접
  (동점 거리 짧은 것) → conf ≥ fused_min_conf · src ∈ fused_allowed_sources.
  fill = LiDAR 먼저, None 일 때만 fused / fused = fused 먼저, 없으면 LiDAR / lidar = 지금 그대로(구독도 없음).
  기본 = fused(계약서_흰부표_거리융합_v3 G-9, 11-14). perception_fusion 없음·묵음이면 lidar 와 같은 출력.
  🚨 적용은 APPROACH 의 카메라 분기(_lidar_dist_at) 한 곳 — ORBIT 의 LiDAR 추적·거리는 그대로(계약 위험 4).
순수 함수 부분은 ship_back/test/test_fused_match.py 와 같은 내용(사본 — 공유 모듈 금지 §1-8).
노드 쪽은 test_turn_fsm.py 와 같은 rclpy 스텁으로 실제 ShipTurn 을 돈다.
"""

import importlib.util
import math
import os
import sys
import types

PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
SRC = os.path.join(PKG, '..')
sys.path.insert(0, PKG)

from ship_turn import orbit_logic as FL  # noqa: E402
from ship_turn.orbit_logic import (  # noqa: E402
    FUSED_VER, FUSED_STRIDE, FUSED_COLOR_IDS, FUSED_SRC_IDS, DISTANCE_SOURCES,
    check_distance_source, fused_allowed_src_ids, fused_color_id, parse_fused_objects,
    fused_fresh, match_fused, choose_distance)

ALLOWED = fused_allowed_src_ids(['lidar', 'fused', 'depth'])[0]
RED, GREEN = FUSED_COLOR_IDS['red'], FUSED_COLOR_IDS['green']
S_LIDAR, S_DEPTH, S_FUSED, S_ROW = (FUSED_SRC_IDS[k] for k in ('lidar', 'depth', 'fused', 'row'))


def obj(bearing, rng, color=0, conf=0.9, src=None, age=0.0, track=-1.0, flags=0.0):
    src = S_FUSED if src is None else src
    return [float(color), float(bearing), float(rng), float(conf), float(src), age, track, flags]


def data(*objs, ver=FUSED_VER, stride=FUSED_STRIDE, n=None):
    out = [float(ver), float(stride), float(len(objs) if n is None else n)]
    for o in objs:
        out += o
    return out


def objs_of(*o):
    parsed, err = parse_fused_objects(data(*o))
    assert err is None, err
    return parsed


# ─────────────── 형식 (parse_fused_objects) ───────────────
def test_parse_ok():
    p, err = parse_fused_objects(data(obj(10.0, 3.0, RED, 0.8, S_DEPTH)))
    assert err is None and len(p) == 1, (p, err)
    o = p[0]
    assert (o.color_id, o.bearing_deg, o.range_m, o.src_id) == (RED, 10.0, 3.0, S_DEPTH), o
    assert abs(o.conf - 0.8) < 1e-9


def test_parse_empty_list_is_alive():
    """N=0 = 살아 있고 물체 없음 → 빈 목록(오류 아님)."""
    p, err = parse_fused_objects(data())
    assert p == [] and err is None, (p, err)


def test_parse_ver_mismatch():
    p, err = parse_fused_objects(data(obj(0.0, 3.0), ver=2))
    assert p is None and err and 'VER' in err, (p, err)


def test_parse_stride_mismatch():
    p, err = parse_fused_objects(data(obj(0.0, 3.0), stride=7))
    assert p is None and err and 'STRIDE' in err, (p, err)


def test_parse_length_mismatch():
    for d in (data(obj(0.0, 3.0), n=2), data(obj(0.0, 3.0), obj(5.0, 4.0), n=1),
              data(obj(0.0, 3.0))[:-1], [1.0, 8.0]):
        p, err = parse_fused_objects(d)
        assert p is None and err, (d, p, err)


def test_parse_bad_n():
    for n in (-1, 1.5, float('nan')):
        p, err = parse_fused_objects(data(obj(0.0, 3.0), n=n))
        assert p is None and err, (n, p, err)


def test_parse_non_numeric_and_none():
    for d in (None, ['a', 8.0, 0.0]):
        p, err = parse_fused_objects(d)
        assert p is None and err, (d, p, err)


def test_parse_skips_object_without_range():
    """거리 없는(0·음수·NaN) 물체는 거리로 쓰지 않는다 — 버리고 나머지는 남긴다."""
    p, err = parse_fused_objects(data(obj(0.0, 0.0), obj(1.0, -2.0), obj(2.0, float('nan')),
                                      obj(float('inf'), 3.0), obj(3.0, 4.0)))
    assert err is None and [o.range_m for o in p] == [4.0], (p, err)


def test_parse_bearing_wrapped():
    p, _ = parse_fused_objects(data(obj(190.0, 3.0)))
    assert abs(p[0].bearing_deg - (-170.0)) < 1e-9, p


# ─────────────── 매칭 (match_fused) ───────────────
def test_match_within_window_inclusive():
    o = objs_of(obj(5.0, 3.0))
    assert match_fused(o, 0.0, 0, 5.0, 0.5, ALLOWED) == (3.0, S_FUSED)
    assert match_fused(objs_of(obj(5.01, 3.0)), 0.0, 0, 5.0, 0.5, ALLOWED) is None


def test_match_nearest_bearing():
    o = objs_of(obj(4.0, 2.0), obj(1.0, 6.0), obj(-3.0, 1.0))
    assert match_fused(o, 0.0, 0, 5.0, 0.5, ALLOWED) == (6.0, S_FUSED)


def test_match_tie_shorter_range():
    o = objs_of(obj(2.0, 6.0), obj(-2.0, 4.0))
    assert match_fused(o, 0.0, 0, 5.0, 0.5, ALLOWED) == (4.0, S_FUSED)


def test_match_color_filter():
    o = objs_of(obj(0.0, 3.0, GREEN), obj(3.0, 5.0, RED), obj(1.0, 2.0, 0))
    assert match_fused(o, 0.0, RED, 5.0, 0.5, ALLOWED) == (5.0, S_FUSED), "색 알면 같은 색만"
    assert match_fused(o, 0.0, GREEN, 5.0, 0.5, ALLOWED) == (3.0, S_FUSED)
    assert match_fused(o, 0.0, 0, 5.0, 0.5, ALLOWED) == (3.0, S_FUSED), "색 모름 → 방위만"
    assert match_fused(o, 0.0, FUSED_COLOR_IDS['yellow'], 5.0, 0.5, ALLOWED) is None


def test_match_conf_gate_after_nearest():
    """최근접 물체의 conf 가 모자라면 None — 더 먼 방위의 다른 물체로 넘어가지 않는다."""
    o = objs_of(obj(0.5, 3.0, conf=0.3), obj(4.0, 5.0, conf=0.9))
    assert match_fused(o, 0.0, 0, 5.0, 0.5, ALLOWED) is None
    assert match_fused(objs_of(obj(0.5, 3.0, conf=0.5)), 0.0, 0, 5.0, 0.5, ALLOWED) == (3.0, S_FUSED)


def test_match_src_gate():
    assert match_fused(objs_of(obj(0.0, 3.0, src=S_ROW)), 0.0, 0, 5.0, 0.5, ALLOWED) is None
    for s in (S_LIDAR, S_DEPTH, S_FUSED):
        assert match_fused(objs_of(obj(0.0, 3.0, src=s)), 0.0, 0, 5.0, 0.5, ALLOWED) == (3.0, s)
    o = objs_of(obj(0.0, 3.0, src=S_ROW), obj(3.0, 5.0, src=S_DEPTH))
    assert match_fused(o, 0.0, 0, 5.0, 0.5, ALLOWED) is None, "최근접이 row 면 None"


def test_match_empty_and_none():
    assert match_fused([], 0.0, 0, 5.0, 0.5, ALLOWED) is None
    assert match_fused(None, 0.0, 0, 5.0, 0.5, ALLOWED) is None
    assert match_fused(objs_of(obj(0.0, 3.0)), None, 0, 5.0, 0.5, ALLOWED) is None


def test_match_wraps_180():
    o = objs_of(obj(-178.0, 3.0))
    assert match_fused(o, 179.0, 0, 5.0, 0.5, ALLOWED) == (3.0, S_FUSED)


def test_match_nan_params_fail_closed():
    o = objs_of(obj(0.0, 3.0))
    assert match_fused(o, 0.0, 0, float('nan'), 0.5, ALLOWED) is None
    assert match_fused(o, 0.0, 0, 5.0, float('nan'), ALLOWED) is None


# ─────────────── 파라미터 해석 ───────────────
def test_check_distance_source():
    for v in DISTANCE_SOURCES:
        assert check_distance_source(v) == (v, None)
    assert check_distance_source(' Fill ') == ('fill', None)
    for bad in ('', 'depth', None, 3, 'lidar,fill'):
        mode, err = check_distance_source(bad)
        assert mode == 'lidar' and err and 'lidar' in err, (bad, mode, err)


def test_allowed_src_ids():
    ids, bad = fused_allowed_src_ids(['lidar', 'fused', 'depth'])
    assert ids == {S_LIDAR, S_FUSED, S_DEPTH} and bad == []
    ids, bad = fused_allowed_src_ids(['depth', 'camera'])
    assert ids == {S_DEPTH} and bad == ['camera']
    ids, bad = fused_allowed_src_ids('depth')
    assert ids == set() and bad, "문자열 하나는 배열 아님 → 오류"


def test_color_id():
    assert fused_color_id('red') == RED and fused_color_id(' Green ') == GREEN
    assert fused_color_id(None) == 0 and fused_color_id('') == 0 and fused_color_id('purple') == 0


def test_fused_fresh():
    assert not fused_fresh(None, 10.0, 0.5)
    assert fused_fresh(9.5, 10.0, 0.5) and not fused_fresh(9.49, 10.0, 0.5)
    assert not fused_fresh(9.9, 10.0, float('nan'))


# ─────────────── 분기 (choose_distance): mode × LiDAR 있음/None × fused 있음/없음 ───────────────
def test_choose_distance_matrix():
    hit = (2.0, S_DEPTH)
    cases = {
        ('lidar', 3.0, hit): (3.0, 'lidar'), ('lidar', None, hit): (None, None),
        ('lidar', 3.0, None): (3.0, 'lidar'), ('lidar', None, None): (None, None),
        ('fill', 3.0, hit): (3.0, 'lidar'), ('fill', None, hit): (2.0, 'fused:depth'),
        ('fill', 3.0, None): (3.0, 'lidar'), ('fill', None, None): (None, None),
        ('fused', 3.0, hit): (3.0, 'lidar'), ('fused', None, hit): (2.0, 'fused:depth'),   # depth = 채움 전용(G-5)
        ('fused', 3.0, None): (3.0, 'lidar'), ('fused', None, None): (None, None),
    }
    for (mode, ld, h), want in cases.items():
        assert choose_distance(mode, ld, h) == want, (mode, ld, h, choose_distance(mode, ld, h))


def test_choose_distance_fused_lidar_priority_over_depth_only():
    """fused: 고른 물체가 depth(D455 단독)이고 노드 LiDAR 값이 있으면 LiDAR(계약서_흰부표_거리융합_v3 G-5).
    depth 는 LiDAR 가 못 볼 때 채우기 전용. src lidar·fused(합의) 는 지금처럼 fused 먼저. 출처 이름 = 실제로 쓴 값."""
    assert choose_distance('fused', 3.0, (2.0, S_DEPTH)) == (3.0, 'lidar')
    assert choose_distance('fused', None, (2.0, S_DEPTH)) == (2.0, 'fused:depth')
    assert choose_distance('fused', 3.0, (2.5, S_FUSED)) == (2.5, 'fused:fused')
    assert choose_distance('fused', 3.0, (2.5, S_LIDAR)) == (2.5, 'fused:lidar')
    assert choose_distance('fused', 3.0, None) == (3.0, 'lidar')
    assert choose_distance('fill', 3.0, (2.0, S_DEPTH)) == (3.0, 'lidar')
    assert choose_distance('fill', None, (2.0, S_DEPTH)) == (2.0, 'fused:depth')


# ─────────────── 상수 사본 일치 (공유 모듈 금지 §1-8 → 세 미션 패키지 사본) ───────────────
def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


COPIES = {
    'ship_back': os.path.join(SRC, 'ship_back', 'ship_back', 'hold_logic.py'),
    'ship_gate': os.path.join(SRC, 'ship_gate', 'ship_gate', 'gate_logic.py'),
}
CONST_NAMES = ('FUSED_VER', 'FUSED_STRIDE', 'FUSED_HEAD', 'FUSED_I_COLOR', 'FUSED_I_BEARING', 'FUSED_I_RANGE',
               'FUSED_I_CONF', 'FUSED_I_SRC', 'FUSED_I_AGE', 'FUSED_I_TRACK', 'FUSED_I_FLAGS',
               'FUSED_COLOR_IDS', 'FUSED_SRC_IDS', 'DISTANCE_SOURCES')


def test_constants_match_spec():
    assert (FUSED_VER, FUSED_STRIDE, FL.FUSED_HEAD) == (1, 8, 3)
    assert [FL.FUSED_I_COLOR, FL.FUSED_I_BEARING, FL.FUSED_I_RANGE, FL.FUSED_I_CONF, FL.FUSED_I_SRC,
            FL.FUSED_I_AGE, FL.FUSED_I_TRACK, FL.FUSED_I_FLAGS] == list(range(8))
    assert FUSED_COLOR_IDS == {'unknown': 0, 'red': 1, 'green': 2, 'white': 3, 'yellow': 4,
                               'black': 5, 'orange': 6, 'blue': 7}
    assert FUSED_SRC_IDS == {'lidar': 0, 'depth': 1, 'fused': 2, 'row': 3}
    assert DISTANCE_SOURCES == ('lidar', 'fill', 'fused')


def test_constant_copies_equal():
    for pkg, path in COPIES.items():
        other = _load(path, f'fused_copy_{pkg}_from_ship_turn')
        for c in CONST_NAMES:
            assert getattr(other, c) == getattr(FL, c), (pkg, c, getattr(other, c), getattr(FL, c))
        # 같은 입력 → 같은 결과(함수 사본도 어긋나지 않게)
        d = data(obj(1.0, 3.0, RED, 0.9, S_DEPTH), obj(-2.0, 2.0, GREEN, 0.4), obj(1.0, 4.0, RED))
        a, _ = parse_fused_objects(d)
        b, _ = other.parse_fused_objects(d)
        assert [tuple(x) for x in a] == [tuple(x) for x in b], pkg
        for args in ((0.0, RED, 5.0, 0.5), (-2.0, 0, 5.0, 0.5), (-2.0, GREEN, 5.0, 0.3)):
            assert match_fused(a, *args, ALLOWED) == other.match_fused(b, *args, ALLOWED), (pkg, args)
        for v in ('fill', 'x'):
            assert check_distance_source(v)[0] == other.check_distance_source(v)[0]
        # choose_distance 사본도 같은 결과(LiDAR 우선 G-5 포함). src 4·5 = P1 단안·삼각측량(노드는 이름 모름)
        for mode in ('lidar', 'fill', 'fused', 'x'):
            for ld in (None, 3.0):
                for h in (None, (2.0, S_LIDAR), (2.0, S_DEPTH), (2.0, S_FUSED), (2.0, S_ROW), (2.0, 4), (2.0, 5)):
                    assert choose_distance(mode, ld, h) == other.choose_distance(mode, ld, h), (pkg, mode, ld, h)


PF_LOGIC = os.path.join(SRC, 'perception_fusion', 'perception_fusion', 'fusion_logic.py')


def test_constants_match_perception_fusion():
    """발행자 perception_fusion.fusion_logic 형식 사본과 같은지(계약 위험 1). 그 패키지·numpy 가 없으면 건너뜀."""
    if not os.path.exists(PF_LOGIC):
        return
    try:
        pf = _load(PF_LOGIC, 'pf_logic_from_ship_turn')
    except ImportError:
        return
    assert (pf.VER, pf.STRIDE) == (FUSED_VER, FUSED_STRIDE)
    assert {'lidar': pf.SRC_LIDAR, 'depth': pf.SRC_DEPTH, 'fused': pf.SRC_FUSED, 'row': pf.SRC_ROW} == FUSED_SRC_IDS
    assert dict(pf.COLOR_IDS, unknown=0) == FUSED_COLOR_IDS
    cols = {'color_id': FL.FUSED_I_COLOR, 'bearing_deg': FL.FUSED_I_BEARING, 'range_m': FL.FUSED_I_RANGE,
            'conf': FL.FUSED_I_CONF, 'src_id': FL.FUSED_I_SRC, 'age_s': FL.FUSED_I_AGE,
            'track_id': FL.FUSED_I_TRACK, 'flags': FL.FUSED_I_FLAGS}
    assert {k: i for i, k in enumerate(pf.FIELDS)} == cols, pf.FIELDS
    # 왕복: 발행자 encode → 이 노드 parse
    o = dict(color_id=RED, bearing_deg=3.0, range_m=2.5, conf=0.75, src_id=S_DEPTH, age_s=0.1,
             track_id=-1, flags=0)
    p, err = parse_fused_objects(pf.encode([o]))
    assert err is None and [tuple(x) for x in p] == [(RED, 3.0, 2.5, 0.75, S_DEPTH)], (p, err)


# ─────────────── 노드(스텁) ───────────────
OVERRIDE = {}
DECLARED = {}
EVENTS = []      # ('param', 이름) / ('sub', 토픽) — 선언이 구독보다 먼저인지


class _Param:
    def __init__(self, v):
        self.value = v


class _Logger:
    def __init__(self):
        self.lines = []

    def info(self, m, **kw):
        self.lines.append(('INFO', m, kw))

    def warn(self, m, **kw):
        self.lines.append(('WARN', m, kw))

    def error(self, m, **kw):
        self.lines.append(('ERROR', m, kw))


class _Pub:
    def __init__(self):
        self.values = []

    def publish(self, msg):
        self.values.append(msg.data)


class _Node:
    def __init__(self, name):
        self._logger = _Logger()
        self.pubs = []
        self.subs = []
        EVENTS.clear()

    def declare_parameter(self, name, default):
        DECLARED[name] = default
        EVENTS.append(('param', name))
        return _Param(OVERRIDE.get(name, default))

    def create_publisher(self, typ, topic, qos):
        p = _Pub()
        self.pubs.append(p)
        return p

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


class _F32MA(_Msg):
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
        'std_msgs.msg': dict(Float32=_Msg, Int32=_Msg, String=_Msg, Float64=_Msg, Float32MultiArray=_F32MA),
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
        spec = importlib.util.spec_from_file_location(
            'ship_turn_fused_under_test', os.path.join(PKG, 'ship_turn', 'ship_turn.py'))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    finally:
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v
    return mod


ST = _load_node_module()
ST.Float32 = _Msg


class _Clock:
    def __init__(self, t=1000.0):
        self.t = t

    def monotonic(self):
        return self.t


CLOCK = _Clock()
ST.time = CLOCK


class _Scan:
    """0.5° 간격 −180~179.5°. points = [(rel 좌 양수, dist)] — 없으면 전부 inf(LiDAR 못 봄)."""

    def __init__(self, points=()):
        self.angle_min = math.radians(-180.0)
        self.angle_increment = math.radians(0.5)
        self.ranges = [math.inf] * 720
        for rel, d in points:
            i = int(round(((80.0 + rel + 180.0) % 360.0) / 0.5)) % 720
            self.ranges[i] = float(d)


def _make(**over):
    OVERRIDE.clear()
    OVERRIDE.update({'enabled': True})
    OVERRIDE.update(over)
    try:
        return ST.ShipTurn()
    finally:
        OVERRIDE.clear()


def _lines(n, lv):
    return [ln for ln in n.get_logger().lines if ln[0] == lv]


def _fused_subs(n):
    return [s for s in n.subs if s[1] == '/fused_objects']


def test_node_default_is_fused_and_subscribes():
    n = _make()
    assert n.distance_source == 'fused'
    subs = _fused_subs(n)
    assert len(subs) == 1 and subs[0][0] is _F32MA, n.subs
    assert subs[0][3].kw == dict(reliability='BEST_EFFORT', history='KEEP_LAST', depth=1), subs[0][3].kw
    assert any('distance_source=fused' in m for _, m, _ in _lines(n, 'INFO')), n.get_logger().lines


def test_node_lidar_no_subscription():
    n = _make(distance_source='lidar')
    assert n.distance_source == 'lidar' and _fused_subs(n) == [], n.subs
    assert _lines(n, 'ERROR') == []


def test_node_bad_source_error_once_then_lidar():
    n = _make(distance_source='both')
    assert n.distance_source == 'lidar' and _fused_subs(n) == []
    errs = _lines(n, 'ERROR')
    assert len(errs) == 1 and 'both' in errs[0][1], errs


def test_node_bad_allowed_source_error():
    n = _make(fused_allowed_sources=['row', 'sonar'])
    errs = _lines(n, 'ERROR')
    assert len(errs) == 1 and 'sonar' in errs[0][1], errs
    assert n.fused_allowed_ids == {S_ROW}


def test_node_params_declared_before_subscriptions():
    _make()
    first_sub = next(i for i, e in enumerate(EVENTS) if e[0] == 'sub')
    for p in ('distance_source', 'fused_topic', 'fused_stale_sec', 'fused_match_deg', 'fused_min_conf',
              'fused_allowed_sources'):
        assert EVENTS.index(('param', p)) < first_sub, (p, EVENTS)


def test_node_defaults_equal_yaml():
    """코드 기본값 = config/ship_turn.yaml (대회 launch 가 이 yaml 을 넘긴다)."""
    import yaml
    _make()
    with open(os.path.join(PKG, 'config', 'ship_turn.yaml'), encoding='utf-8') as f:
        y = yaml.safe_load(f)['/ship_turn']['ros__parameters']
    assert set(DECLARED) == set(y), set(DECLARED) ^ set(y)
    diff = {k: (DECLARED[k], y[k]) for k in y if DECLARED[k] != y[k] or type(DECLARED[k]) is not type(y[k])}
    assert not diff, diff
    assert y['distance_source'] == 'fused' and y['fused_allowed_sources'] == ['lidar', 'fused', 'depth']


def _approach(mode, lidar_dist, rel=0.0, color='red'):
    """APPROACH(CW) 중, 카메라 방위 rel·색 신선, LiDAR 는 그 방위에 lidar_dist(None = 못 봄).
    mode None = 코드 기본값."""
    n = _make() if mode is None else _make(distance_source=mode)
    n.wp_cb(_Msg(3))
    n.state = 'APPROACH'
    n.state_t = CLOCK.t
    n.dir_cw = True
    n.last_seen_t = CLOCK.t
    n.last_rel = rel
    n.angle_cb(_Msg(float(rel)))
    if color is not None:
        n.color_cb(_Msg(color))
    n.latest_scan = _Scan(() if lidar_dist is None else ((rel, lidar_dist),))
    n.t_scan = CLOCK.t
    return n


def _feed(n, *objs, age=0.0):
    n.fused_cb(_F32MA(data(*objs)))
    n.t_fused = CLOCK.t - age


def _adist(n, rel=0.0, color='red'):
    return n._approach_dist(rel, color, CLOCK.t)


def test_node_branch_matrix():
    """mode × LiDAR(6.0 / None) × fused(빨강 3.0 있음 / 없음 / stale) → APPROACH 거리."""
    want = {
        ('lidar', 6.0): (6.0, 6.0, 6.0), ('lidar', None): (None, None, None),
        ('fill', 6.0): (6.0, 6.0, 6.0), ('fill', None): (3.0, None, None),
        ('fused', 6.0): (6.0, 6.0, 6.0), ('fused', None): (3.0, None, None),   # depth 3.0 은 LiDAR 없을 때만(G-5)
    }
    for (mode, ld), (w_hit, w_none, w_stale) in want.items():
        n = _approach(mode, ld)
        _feed(n, obj(1.0, 3.0, RED, src=S_DEPTH))
        assert _adist(n) == w_hit, (mode, ld, 'hit', _adist(n))
        n = _approach(mode, ld)
        _feed(n, obj(1.0, 3.0, GREEN, src=S_DEPTH))         # 다른 색 = 맞는 물체 없음
        assert _adist(n) == w_none, (mode, ld, 'none', _adist(n))
        n = _approach(mode, ld)
        _feed(n, obj(1.0, 3.0, RED, src=S_DEPTH), age=0.6)  # stale(> 0.5 s)
        assert _adist(n) == w_stale, (mode, ld, 'stale', _adist(n))


def test_node_default_without_fusion_equals_lidar():
    """기본(fused) + perception_fusion 없음(미수신)·묵음(stale) → APPROACH 거리·/candidate_angle·상태가
    lidar 와 같다(계약 v3 N). LiDAR 6.0(APPROACH 유지) / 3.0(ORBIT 진입) / 없음. 차이는 5 s 주기 WARN 뿐."""
    t0 = CLOCK.t
    for ld in (6.0, 3.0, None):
        for feed in ('absent', 'stale'):
            got = {}
            for mode in (None, 'lidar'):
                CLOCK.t = t0
                n = _approach(mode, ld)
                if mode is None and feed == 'stale':
                    _feed(n, obj(0.0, 2.0, RED, src=S_DEPTH), age=0.6)
                trace = [_adist(n)]
                for _ in range(5):
                    _tick(n)
                    trace.append(n.state)
                got[mode] = (trace, list(n.pubs[0].values))
                if mode is None:
                    assert n.distance_source == 'fused'
                    w = [x for x in _lines(n, 'WARN') if '/fused_objects' in x[1]]
                    assert w and all(x[2].get('throttle_duration_sec') == 5.0 for x in w), w
            assert got[None] == got['lidar'], (ld, feed, got)
    CLOCK.t = t0 + 1.0


def test_node_fused_lidar_priority_over_depth_only():
    """검증 S4c·S7 재현: 노드 LiDAR 3.0, /fused_objects 에 빨강 depth 2.0(0°)·빨강 lidar 물체(3°) → 3.0.
    LiDAR None 이면 depth 2.0 으로 채움. PF LiDAR stale(flags bit1) depth 물체도 같은 규칙(G-5)."""
    for other in (3.0, 3.5):
        n = _approach(None, 3.0)
        _feed(n, obj(0.0, 2.0, RED, src=S_DEPTH), obj(3.0, other, RED, src=S_LIDAR))
        assert _adist(n) == 3.0, other
    n = _approach(None, None)
    _feed(n, obj(0.0, 2.0, RED, src=S_DEPTH), obj(3.0, 3.0, RED, src=S_LIDAR))
    assert _adist(n) == 2.0
    for ld, want in ((3.0, 3.0), (None, 2.0)):
        n = _approach(None, ld)
        _feed(n, obj(0.0, 2.0, RED, src=S_DEPTH, flags=2.0))
        assert _adist(n) == want, (ld, _adist(n))


def test_node_unknown_color_bearing_only():
    n = _approach('fill', None)
    _feed(n, obj(1.0, 3.0, GREEN, src=S_DEPTH))
    assert n._approach_dist(0.0, None, CLOCK.t) == 3.0
    assert n._approach_dist(0.0, 'purple', CLOCK.t) == 3.0
    assert n._approach_dist(0.0, 'red', CLOCK.t) is None


def _tick(n, dt=0.1):
    CLOCK.t += dt
    n.timer_cb()


def test_node_fill_enters_orbit_from_depth_when_lidar_blind():
    """APPROACH: LiDAR 못 봄 + depth 빨강 3.0 m(≤ approach_dist 4.0) → ORBIT. lidar 모드면 APPROACH 유지."""
    n = _approach('fill', None)
    _feed(n, obj(0.0, 3.0, RED, src=S_DEPTH))
    _tick(n)
    assert n.state == 'ORBIT', n.state
    n = _approach('lidar', None)
    n.fused_objs = objs_of(obj(0.0, 3.0, RED, src=S_DEPTH))
    n.t_fused = CLOCK.t
    _tick(n)
    assert n.state == 'APPROACH', n.state


def test_node_fill_lidar_first_when_present():
    n = _approach('fill', 6.0)
    _feed(n, obj(0.0, 3.0, RED, src=S_DEPTH))
    _tick(n)
    assert n.state == 'APPROACH', "fill: LiDAR 6.0 이 있으면 fused 3.0 을 안 본다"
    n = _approach('fused', 6.0)
    _feed(n, obj(0.0, 3.0, RED, src=S_FUSED))
    _tick(n)
    assert n.state == 'ORBIT', "fused: 합의(src fused) 3.0 먼저"
    n = _approach('fused', 6.0)
    _feed(n, obj(0.0, 3.0, RED, src=S_DEPTH))
    _tick(n)
    assert n.state == 'APPROACH', "fused: depth 단독 3.0 은 LiDAR 6.0 이 있으면 안 씀(G-5, 조기 ORBIT 없음)"


def test_node_orbit_never_consults_fused():
    """ORBIT 의 카메라 분기·LiDAR 추적은 fused 를 안 본다(계약 위험 4)."""
    n = _approach('fused', None)
    n.state = 'ORBIT'
    n.state_t = CLOCK.t
    n.last_rel = 60.0          # LiDAR 추적 창(±25) 밖 → lidar_hit None → 카메라 분기
    n.t_angle = CLOCK.t + 1.0

    def boom(*a, **k):
        raise AssertionError('ORBIT 에서 fused 를 봤다')

    n._approach_dist = boom
    n._fused_hit = boom
    _feed(n, obj(0.0, 1.0, RED, src=S_DEPTH))
    _tick(n)
    assert n.state == 'ORBIT', n.state
    assert _lines(n, 'WARN') == [] or all('/fused_objects' not in m for _, m, _ in _lines(n, 'WARN'))


def test_node_bad_format_error_throttled_and_ignored():
    n = _approach('fill', None)
    _feed(n, obj(0.0, 2.0, RED, src=S_DEPTH))
    t0 = n.t_fused
    n.fused_cb(_F32MA(data(obj(0.0, 1.0), ver=0)))
    n.fused_cb(_F32MA(data(obj(0.0, 1.0), stride=16)))
    n.fused_cb(_F32MA(data(obj(0.0, 1.0))[:-2]))
    errs = _lines(n, 'ERROR')
    assert len(errs) == 3 and all(e[2].get('throttle_duration_sec') == 5.0 for e in errs), errs
    assert n.t_fused == t0 and [o.range_m for o in n.fused_objs] == [2.0]


def test_node_stale_warn_only_when_consulted():
    n = _approach('fill', 6.0)
    _adist(n)
    assert _lines(n, 'WARN') == []
    n = _approach('fill', None)
    _adist(n)
    w = _lines(n, 'WARN')
    assert len(w) == 1 and w[0][2].get('throttle_duration_sec') == 5.0 and '미수신' in w[0][1], w


def test_node_source_change_info():
    n = _approach('fill', 6.0)
    _feed(n, obj(0.0, 3.0, RED, src=S_FUSED))
    _adist(n)
    n.latest_scan = _Scan()
    _adist(n)
    info = [m for _, m, _ in _lines(n, 'INFO') if 'dist src' in m]
    assert info == ['dist src lidar→fused:fused'], info


if __name__ == '__main__':
    fails = 0
    for name, fn in list(globals().items()):
        if name.startswith('test_') and callable(fn):
            try:
                fn()
                print(f"  ok  {name}")
            except AssertionError as e:
                fails += 1
                print(f"  FAIL {name}: {e}")
    sys.exit(1 if fails else 0)
