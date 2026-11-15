#!/usr/bin/env python3
"""ship_back 노드 hold_source=square 배선 테스트 — rclpy 스텁으로 실제 ShipBack 을 돈다(ROS 없이).

실행: tools/run_tests.sh ship_back

지키는 것 (계약서 v2 위치유지 사각형 (b))
  · 기본 hold_source = square. square 일 때만 /ublox_gps_node/fix·/imu/yaw·/imu/yaw_raw 구독과 /hold_done 발행을 만든다
    (gps·vision 은 DDS 그래프 그대로 — 롤백 스위치). 새 파라미터는 구독보다 먼저 선언.
  · /buoy_bearings 침묵 → 고정주기 타이머에서 50000 + ERROR, /hold_done 은 mode 2 틱마다 false, DONE 이면 true.
  · 센서 부호·단위(상대방위 좌 +, LiDAR 80° = 정면, 부표 반지름 보정, 지역 평면)가 맞아 끝까지 DONE.
  · INIT·DONE 밖 20000 없음, 출력 집합 밖 없음. mode 3(search_behavior:=hold)은 vision 경로(/hold_done 없음).
"""

import importlib.util
import math
import os
import sys
import types

PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
sys.path.insert(0, PKG)
NODE_SRC = os.path.join(PKG, 'ship_back', 'ship_back.py')

from ship_back.square_logic import DEFAULTS, output_allowed, xy_to_ll, bearing_to, wrap180, order_ccw  # noqa: E402
from ship_back.square_logic import hull_inside  # noqa: E402

INV, STOP, SR, SL, REV = 20000.0, 50000.0, 5000.0, 6000.0, 180.0
LAT0, LON0 = 35.18690, 128.56600

OVERRIDE = {}
EVENTS = []


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
    def __init__(self, topic):
        self.topic = topic
        self.values = []

    def publish(self, msg):
        self.values.append(msg.data)


class _Node:
    def __init__(self, name):
        self._logger = _Logger()
        self.subs = []
        self.pubs = {}
        EVENTS.clear()

    def declare_parameter(self, name, default):
        EVENTS.append(('param', name))
        return _Param(OVERRIDE.get(name, default))

    def create_publisher(self, typ, topic, qos):
        p = _Pub(topic)
        self.pubs[topic] = p
        EVENTS.append(('pub', topic))
        return p

    def create_subscription(self, typ, topic, cb, qos):
        self.subs.append((typ, topic, cb, qos))
        EVENTS.append(('sub', topic))

    def create_timer(self, *a, **k):
        EVENTS.append(('timer', None))

    def get_logger(self):
        return self._logger


class _Msg:
    def __init__(self, data=0.0):
        self.data = data


class _F64(_Msg):
    pass


class _Bool(_Msg):
    pass


class _Fix:
    def __init__(self, lat, lon, status=2):
        self.latitude, self.longitude = lat, lon
        self.status = types.SimpleNamespace(status=status)


class _QoS:
    def __init__(self, **k):
        self.kw = k


def _enum(n):
    return type(n, (), {'BEST_EFFORT': 'BEST_EFFORT', 'RELIABLE': 'RELIABLE', 'KEEP_LAST': 'KEEP_LAST'})


def _load():
    mods = {
        'rclpy': dict(init=lambda *a, **k: None, ok=lambda: True, shutdown=lambda: None),
        'rclpy.node': dict(Node=_Node),
        'rclpy.executors': dict(MultiThreadedExecutor=object),
        'rclpy.qos': dict(QoSProfile=_QoS, ReliabilityPolicy=_enum('R'), HistoryPolicy=_enum('H')),
        'std_msgs': {},
        'std_msgs.msg': dict(Float32=_Msg, Float64=_F64, Int32=_Msg, Bool=_Bool, Float32MultiArray=_Msg),
        'sensor_msgs': {},
        'sensor_msgs.msg': dict(LaserScan=object, NavSatFix=_Fix),
    }
    saved = {k: sys.modules.get(k) for k in mods}
    try:
        for name, attrs in mods.items():
            m = types.ModuleType(name)
            for k, v in attrs.items():
                setattr(m, k, v)
            sys.modules[name] = m
        spec = importlib.util.spec_from_file_location('ship_back_square_under_test', NODE_SRC)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    finally:
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v
    return mod


SB = _load()


class _Clock:
    t = 1000.0

    def monotonic(self):
        return self.t


CLOCK = _Clock()
SB.time = CLOCK


def _make(**over):
    OVERRIDE.clear()
    OVERRIDE.update(over)
    try:
        return SB.ShipBack()
    finally:
        OVERRIDE.clear()


def _subs(n, topic):
    return [s for s in n.subs if s[1] == topic]


def _err(n):
    return [m for lv, m in n.get_logger().lines if lv == 'ERROR']


# ─────────────── 배선 ───────────────
def test_square_default_creates_square_io_with_qos():
    n = _make()
    assert n.hold_source == 'square' and _err(n) == []
    rel5 = dict(depth=5, reliability='RELIABLE')
    for topic, typ in (('/ublox_gps_node/fix', _Fix), ('/imu/yaw', _F64), ('/imu/yaw_raw', _F64),
                       ('/goal_distance', _Msg)):
        s = _subs(n, topic)
        assert len(s) == 1 and s[0][0] is typ and s[0][3].kw == rel5, (topic, n.subs)
    bb = _subs(n, '/buoy_bearings')
    assert len(bb) == 1 and bb[0][3].kw == dict(reliability='BEST_EFFORT', history='KEEP_LAST', depth=1)
    assert '/hold_done' in n.pubs and '/candidate_angle' in n.pubs


def test_gps_and_vision_have_no_square_io():
    for src in ('gps', 'vision'):
        n = _make(hold_source=src)
        for topic in ('/ublox_gps_node/fix', '/imu/yaw', '/imu/yaw_raw'):
            assert _subs(n, topic) == [], (src, topic)
        assert '/hold_done' not in n.pubs, src
    assert _subs(_make(hold_source='vision'), '/goal_distance') == []


def test_square_params_declared_before_any_subscription_or_timer():
    _make()
    first = next(i for i, e in enumerate(EVENTS) if e[0] in ('sub', 'timer', 'pub'))
    for k in DEFAULTS:
        assert EVENTS.index(('param', k)) < first, k


def test_no_wallclock_and_no_logger_variable():
    import re
    src = open(NODE_SRC, encoding='utf-8').read()
    assert 'time.time(' not in src
    assert re.search(r'=\s*self\.get_logger\(\)\s*$', src, re.M) is None
    lsrc = open(os.path.join(PKG, 'ship_back', 'square_logic.py'), encoding='utf-8').read()
    assert 'import rclpy' not in lsrc and 'time.time(' not in lsrc


# ─────────────── 합성 세계 ───────────────
class _Scan:
    """1° 간격 −180~179°(LiDAR 각, 80° = 정면, 좌 +)."""

    def __init__(self, ranges):
        self.angle_min = math.radians(-180.0)
        self.angle_increment = math.radians(1.0)
        self.ranges = ranges


class World:
    def __init__(self, start=(0.0, 11.0), heading=180.0, buoy_r=0.25):
        self.buoys = [(-2.5, -2.5), (2.5, -2.5), (2.5, 2.5), (-2.5, 2.5)]
        self.x, self.y = start
        self.h = heading
        self.br = buoy_r
        self.imu0 = 77.0

    def scan(self):
        r = [float('inf')] * 360
        for bx, by in self.buoys:
            d = math.hypot(bx - self.x, by - self.y)
            rel = wrap180(self.h - bearing_to(self.x, self.y, bx, by))
            half = math.degrees(math.asin(min(1.0, self.br / d)))
            for a in range(int(math.floor(80 + rel - half)), int(math.ceil(80 + rel + half)) + 1):
                i = int(wrap180(a) + 180) % 360
                r[i] = min(r[i], d - self.br)
        return _Scan(r)

    def bearings(self):
        data = [1.0, 4.0, 0.0]
        for bx, by in self.buoys:
            d = math.hypot(bx - self.x, by - self.y)
            rel = wrap180(self.h - bearing_to(self.x, self.y, bx, by))
            if abs(rel) <= 31.9 and 0.5 <= d <= 12.0:
                data += [rel, 400.0 * (5.0 / d) ** 2, 0.8, 0.05]
                data[2] += 1.0
        return data

    def act(self, v, dt=0.1):
        if v == INV:
            spd, w = 1.0, max(-40.0, min(40.0, wrap180(bearing_to(self.x, self.y, 0, 0) - self.h) / dt))
        elif v == STOP:
            spd, w = 0.0, 0.0
        elif v in (SR, SL):
            spd, w = 0.0, 30.0 if v == SR else -30.0
        elif v == REV:
            spd, w = -0.4, 0.0
        else:
            rel = v if v <= 180.0 else v - 360.0
            spd, w = 0.6, max(-40.0, min(40.0, -rel))
        self.h = (self.h + w * dt) % 360.0
        self.x += spd * math.sin(math.radians(self.h)) * dt
        self.y += spd * math.cos(math.radians(self.h)) * dt


def _run(n, w, ticks, mode=2, bearings=True, fix_every=10, stop_after_done=True):
    """틱 하나 = 0.1 s. DONE 이 되면(그 틱의 세계 위치를 w.at_done 에 남기고) 3 틱 더 돌고 멈춘다
    (DONE 뒤 20000 은 이 세계에선 'north LOS' 라 배가 움직인다 — 판정은 DONE 순간)."""
    out = []
    w.at_done = None
    for k in range(ticks):
        CLOCK.t += 0.1
        if k == 0:
            n.wp_cb(_Msg(mode))
        n.yaw_raw_cb(_F64((w.h - w.imu0) % 360.0))            # IMU 상대: 임의 기준
        n.yaw_cb(_F64(w.h))                                   # 참값(이 테스트는 배선만 본다)
        if k % fix_every == 0:
            n.fix_cb(_Fix(*xy_to_ll(w.x, w.y, LAT0, LON0)))
        n.scan_cb(w.scan())
        if bearings:
            n.buoy_bearings_cb(_Msg(w.bearings()))
        n.goal_distance_cb(_Msg(math.hypot(w.x, w.y)))
        before = len(n.pub_candidate.values)
        hd_before = len(n.pubs['/hold_done'].values) if '/hold_done' in n.pubs else 0
        n.timer_cb()
        vals = n.pub_candidate.values[before:]
        hd = n.pubs['/hold_done'].values[hd_before:] if '/hold_done' in n.pubs else []
        assert len(vals) == 1, vals
        out.append((n.state, vals[0], hd))
        if n.state == 'DONE' and w.at_done is None:
            w.at_done = (w.x, w.y, w.h)
        if stop_after_done and w.at_done is not None and len(out) - 1 - next(
                i for i, o in enumerate(out) if o[0] == 'DONE') >= 3:
            break
        w.act(vals[0])
    return out


def test_node_square_end_to_end_done_and_hold_done():
    n = _make()
    w = World()
    out = _run(n, w, 900)
    states = [s for s, _, _ in out]
    assert 'DONE' in states, (states[-1], n.get_logger().lines[-6:])
    k_done = states.index('DONE')
    for s, v, hd in out:
        assert output_allowed(v, s), (s, v)
        assert hd == [s == 'DONE'], (s, hd)
    eng = states.index(next(s for s in states if s != 'INIT'))
    assert all(v != INV for s, v, _ in out[eng:k_done]), 'DONE 전 20000'
    assert n.sq_fsm.success is True and _err(n) == []
    assert hull_inside(order_ccw(w.buoys), *w.at_done, 1.77, 0.57, 0.25), w.at_done
    c = n.sq_fsm.center
    cx, cy = c[0] + (n.sq_origin and 0.0), c[1]
    # 중심(지역 평면, 원점 = 첫 fix) → 세계: 원점 fix 의 세계 좌표를 더한다
    from ship_back.square_logic import ll_to_xy
    ox, oy = ll_to_xy(n.sq_origin[0], n.sq_origin[1], LAT0, LON0)
    assert math.hypot(cx + ox, cy + oy) < 0.15, (cx + ox, cy + oy)   # 부표 반지름 보정이 맞아야 중심이 맞는다
    # 2→2 재수신은 래치, 다른 모드 갔다 오면 새로
    n.wp_cb(_Msg(2))
    CLOCK.t += 0.1
    n.timer_cb()
    assert n.state == 'DONE' and n.pub_candidate.values[-1] == INV and n.pubs['/hold_done'].values[-1] is True
    n.wp_cb(_Msg(5))
    n.wp_cb(_Msg(2))
    assert n.state == 'INIT' and n.sq_fsm.map.lms == [] and n.sq_origin is None


def test_node_square_stale_bearings_stop_error_hold_done_false():
    n = _make()
    w = World(start=(0.0, 7.0))
    out = _run(n, w, 60, bearings=False)
    after = [(s, v, hd) for s, v, hd in out if s != 'INIT']
    assert after and all(v == STOP and hd == [False] for _, v, hd in after), after[:3]
    assert any('/buoy_bearings' in m for m in _err(n))


def test_node_square_bad_bearings_format_is_stale_error():
    n = _make()
    w = World(start=(0.0, 7.0))
    for k in range(40):
        CLOCK.t += 0.1
        if k == 0:
            n.wp_cb(_Msg(2))
        n.yaw_raw_cb(_F64(0.0))
        n.yaw_cb(_F64(180.0))
        n.fix_cb(_Fix(*xy_to_ll(w.x, w.y, LAT0, LON0)))
        n.buoy_bearings_cb(_Msg([7.0, 7.0, 0.0]))
        n.goal_distance_cb(_Msg(7.0))
        n.timer_cb()
    assert n.pub_candidate.values[-1] == STOP
    assert any('형식 불일치' in m for m in _err(n)) and any('/buoy_bearings' in m for m in _err(n))


def test_node_mode_3_hold_option_is_vision_path():
    n = _make(active_wp_modes=[2, 3])
    w = World(start=(0.0, 7.0))
    n.wp_cb(_Msg(3))
    for k in range(20):
        CLOCK.t += 0.1
        n.angle_cb(_Msg(10000.0))
        n.buoy_bearings_cb(_Msg(w.bearings()))
        n.timer_cb()
    assert set(n.pub_candidate.values) == {INV}           # vision SEARCH 20000(옛 동작)
    assert n.pubs['/hold_done'].values == []              # square 아님 → /hold_done 안 냄
