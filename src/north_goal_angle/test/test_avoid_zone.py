#!/usr/bin/env python3
"""/avoid_zone (계약서_도리도리회피_v2_20261114 U2, 명세 8) — 순수 로직 + 스텁 노드 테스트. ROS 없이 돈다.

    tools/run_tests.sh north_goal_angle

지키는 것
  · avoid_leg_on(avoid_wps, wp_idx) = wp_idx ≥ 1 and avoid_wps[wp_idx] and avoid_wps[wp_idx − 1]
    — 폴리곤 비면 false, WP6 로 가는 중 false·WP7 로 가는 중 true, 연속 구간, wp_idx 0, 코스 끝 false
  · 노드: timer_cb 매 틱 /avoid_zone 한 번(= /wp_mode 와 같은 주기), 값 = avoid_leg_on, GPS 없음·AUTO 대기에도 냄,
    코스 끝은 false 계속. 발행 QoS = RELIABLE depth 10. 시작 로그 한 줄('없음' 포함). 로거 호출 지점 심각도 섞지 않음.
north_goal_angle.py 는 rclpy·geopy 를 import 하므로 노드는 sys.modules 스텁으로 잠시 불러 실제 timer_cb 를 돌린다.
"""

import importlib
import math
import os
import sys
import types

import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from north_goal_angle.geofence_rays import (  # noqa: E402
    M_PER_DEG_LAT, M_PER_DEG_LON_EQ, avoid_fence_wps, avoid_leg_indices, avoid_leg_on)
from north_goal_angle.waypoint_loader import parse_waypoints  # noqa: E402

_PKG = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
_WP_YAML = os.path.join(_PKG, 'config', 'waypoints.yaml')
_NODE_MOD = 'north_goal_angle.north_goal_angle'

LAT0, LON0 = 35.1866, 128.5655
KX = math.cos(math.radians(LAT0)) * M_PER_DEG_LON_EQ


def ll(east, north):
    return LAT0 + north / M_PER_DEG_LAT, LON0 + east / KX


def poly_box(w, e, s, n):
    out = []
    for x, y in ((w, s), (e, s), (e, n), (w, n)):
        out += list(ll(x, y))
    return out


ZONE = poly_box(-5.0, 5.0, 0.0, 30.0)          # 폭 10 m, 길이 30 m


def wp(east, north, mode):
    return (*ll(east, north), mode, 1.0)


# 접근(구역 밖 mode 5) → 회피 시작(안) → 회피 끝(안) → 종료(mode 8)
COURSE = [wp(0.0, -20.0, 0), wp(0.0, -8.0, 5), wp(0.0, 5.0, 5), wp(0.0, 25.0, 5), wp(0.0, 40.0, 8)]


# ------------------------------------------------------------ 순수: avoid_leg_on

def test_empty_polygon_always_false():
    flags = avoid_fence_wps(COURSE, [])
    assert flags == [False] * len(COURSE)
    for i in range(-1, len(COURSE) + 2):
        assert avoid_leg_on(flags, i) is False, i
    assert avoid_leg_indices(flags) == []
    assert avoid_leg_on([], 0) is False and avoid_leg_on([], 3) is False


def test_leg_needs_both_ends_inside():
    """출발 WP·도착 WP 둘 다 켜는 WP 인 다리만 — 접근 다리(밖→안)는 false, 회피 다리(안→안)는 true."""
    flags = avoid_fence_wps(COURSE, ZONE)
    assert flags == [False, False, True, True, False], flags
    assert avoid_leg_on(flags, 1) is False          # WP0 → 접근
    assert avoid_leg_on(flags, 2) is False          # 접근(밖) → 회피 시작 = 'WP6 로 가는 중'
    assert avoid_leg_on(flags, 3) is True           # 회피 시작 → 회피 끝 = 'WP7 로 가는 중'
    assert avoid_leg_on(flags, 4) is False          # 회피 끝 → 종료(mode 8)
    assert avoid_leg_indices(flags) == [3]


def test_continuous_zone():
    """구역 안 mode 5 WP 가 셋 연달아면 다리 둘(2→3, 3→4) 모두 true, 첫 진입 다리는 false."""
    flags = [False, True, True, True, False]
    assert [avoid_leg_on(flags, i) for i in range(5)] == [False, False, True, True, False]
    assert avoid_leg_indices(flags) == [2, 3]
    # 사이에 구역 밖 WP 가 끼면 끊긴다
    flags = [True, True, False, True, True]
    assert avoid_leg_indices(flags) == [1, 4]


def test_wp_idx_zero_false():
    """첫 WP 로 가는 중(출발 WP 없음)은 첫 WP 가 구역 안이어도 false — 인덱스 −1(마지막 WP)을 보지 않는다."""
    flags = [True, True]
    assert avoid_leg_on(flags, 0) is False
    assert avoid_leg_on(flags, 1) is True
    assert avoid_leg_on([True, False, True], 0) is False      # flags[-1] 이 True 여도


def test_course_end_false():
    flags = [False, True, True]
    assert avoid_leg_on(flags, 3) is False
    assert avoid_leg_on(flags, 10) is False


def test_returns_plain_bool():
    """std_msgs/Bool data 에 그대로 넣는다 — numpy bool 등이 아니라 bool."""
    for flags, i in (([True, True], 1), ([True, False], 1), ([], 0)):
        assert type(avoid_leg_on(flags, i)) is bool


def _course_wps():
    with open(_WP_YAML, encoding='utf-8') as f:
        return parse_waypoints(yaml.safe_load(f))


def test_default_course_wp6_wp7():
    """실제 waypoints.yaml: WP6·WP7 둘레 폴리곤이면 WP6 로 가는 중(idx 10) false, WP7 로 가는 중(idx 11) true.

    도킹 정렬·탐색 접근(구역 밖 mode 5)은 켜지지 않는다. 폴리곤이 비면(yaml 기본 []) 늘 false.
    """
    wps = _course_wps()
    i6, i7 = 10, 11
    assert wps[i6][2] == 5 and wps[i7][2] == 5 and wps[i6 - 1][2] != 5
    lats = (wps[i6][0], wps[i7][0])
    lons = (wps[i6][1], wps[i7][1])
    m_lat, m_lon = 3.0 / M_PER_DEG_LAT, 3.0 / KX         # 두 WP 둘레 3 m 여유
    box = [min(lats) - m_lat, min(lons) - m_lon, min(lats) - m_lat, max(lons) + m_lon,
           max(lats) + m_lat, max(lons) + m_lon, max(lats) + m_lat, min(lons) - m_lon]
    flags = avoid_fence_wps(wps, box)
    assert [i for i, f in enumerate(flags) if f] == [i6, i7], flags
    assert avoid_leg_on(flags, i6) is False
    assert avoid_leg_on(flags, i7) is True
    assert avoid_leg_indices(flags) == [i7]
    assert avoid_leg_indices(avoid_fence_wps(wps, [])) == []


def test_yaml_polygons_empty_by_default():
    """폴리곤 기본 [] (공통·main yaml) → 오늘 배는 /avoid_zone 늘 false (계약서 v2 C U2 '안전한 이유')."""
    for name in ('north_goal_angle.yaml', 'north_goal_angle_main.yaml'):
        with open(os.path.join(_PKG, 'config', name), encoding='utf-8') as f:
            p = yaml.safe_load(f)['/north_goal_angle']['ros__parameters']
        assert p['geofence_avoid_polygon'] == [], name


# ------------------------------------------------------------ 스텁 노드

class _Logger:
    """심각도별 호출 지점 기록 — 같은 줄을 다른 심각도로 부르면 실패(§1-7)."""

    def __init__(self):
        self.lines = []
        self.sites = {}

    def _log(self, sev, msg):
        f = sys._getframe(2)
        site = (f.f_code.co_filename, f.f_lineno)
        old = self.sites.setdefault(site, sev)
        assert old == sev, f'{site} 가 {old} 와 {sev} 를 같이 씀 (§1-7)'
        self.lines.append((sev, str(msg)))

    def info(self, msg, **kw):
        self._log('INFO', msg)

    def warn(self, msg, **kw):
        self._log('WARN', msg)

    def warning(self, msg, **kw):
        self._log('WARN', msg)

    def error(self, msg, **kw):
        self._log('ERROR', msg)


class _Pub:
    def __init__(self, typ, topic, qos):
        self.typ, self.topic, self.qos = typ, topic, qos
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)

    def get_subscription_count(self):
        return 1


class _Param:
    def __init__(self, v):
        self.value = v


class _Msg:
    def __init__(self, **kw):
        self.__dict__.update(kw)


def _msg_type(name):
    return type(name, (_Msg,), {})


OVERRIDES = {}


class _Node:
    def __init__(self, name):
        self.name = name
        self.logger = _Logger()
        self.subs = {}
        self.pubs = {}
        self.timers = []

    def declare_parameter(self, key, default, descriptor=None):
        return _Param(OVERRIDES.get(key, default))

    def get_logger(self):
        return self.logger

    def create_subscription(self, typ, topic, cb, qos):
        self.subs[topic] = cb

    def create_publisher(self, typ, topic, qos):
        self.pubs[topic] = _Pub(typ, topic, qos)
        return self.pubs[topic]

    def create_timer(self, period, cb):
        self.timers.append((period, cb))


class _Clock:
    t = 100.0

    def monotonic(self):
        return self.t


@pytest.fixture
def node_mod(monkeypatch):
    """rclpy·msgs·geopy·ament 스텁을 sys.modules 에 잠시 넣고 노드 모듈을 새로 import (테스트 뒤 원래대로)."""
    def stub(name, **attrs):
        m = types.ModuleType(name)
        for k, v in attrs.items():
            setattr(m, k, v)
        monkeypatch.setitem(sys.modules, name, m)
        return m

    rclpy = stub('rclpy', init=lambda **k: None, spin=lambda n: None, ok=lambda: False,
                 shutdown=lambda: None)
    rclpy.node = stub('rclpy.node', Node=_Node)
    rclpy.qos = stub('rclpy.qos', QoSProfile=lambda **k: dict(k),
                     ReliabilityPolicy=types.SimpleNamespace(BEST_EFFORT='BE', RELIABLE='R'))
    stub('rcl_interfaces')
    stub('rcl_interfaces.msg', ParameterDescriptor=lambda **k: dict(k))
    stub('std_msgs')
    stub('std_msgs.msg', Bool=_msg_type('Bool'), Float32=_msg_type('Float32'),
         Float32MultiArray=_msg_type('Float32MultiArray'), Float64=_msg_type('Float64'),
         Int32=_msg_type('Int32'))
    stub('sensor_msgs')
    stub('sensor_msgs.msg', NavSatFix=_msg_type('NavSatFix'))
    stub('geopy', distance=types.SimpleNamespace(distance=None))
    stub('ament_index_python')
    stub('ament_index_python.packages', get_package_share_directory=lambda pkg: _PKG)
    monkeypatch.delitem(sys.modules, _NODE_MOD, raising=False)
    mod = importlib.import_module(_NODE_MOD)
    clock = _Clock()
    monkeypatch.setattr(mod, 'time', clock)
    OVERRIDES.clear()
    yield mod, clock
    OVERRIDES.clear()
    sys.modules.pop(_NODE_MOD, None)


def _write_course(tmp_path, wps):
    path = tmp_path / 'wps.yaml'
    rows = [{'구역': f'T{i}', 'lat': float(w[0]), 'lon': float(w[1]), 'mode': int(w[2]), 'dwell': float(w[3])}
            for i, w in enumerate(wps)]
    path.write_text(yaml.safe_dump({'waypoints': rows}, allow_unicode=True), encoding='utf-8')
    return str(path)


def _make(mod, tmp_path, polygon):
    OVERRIDES['waypoints_file'] = _write_course(tmp_path, COURSE)
    OVERRIDES['geofence_avoid_polygon'] = polygon
    return mod.NorthGoalAngle()


def _tick(node, clock, dt=0.5):
    assert len(node.timers) == 1
    clock.t += dt
    node.timers[0][1]()


def _zone_vals(node):
    return [m.data for m in node.pubs['/avoid_zone'].msgs]


def _infos(node, text):
    return [m for s, m in node.logger.lines if s == 'INFO' and text in m]


def test_node_publisher_qos_reliable_depth10(node_mod, tmp_path):
    mod, clock = node_mod
    node = _make(mod, tmp_path, [])
    p = node.pubs['/avoid_zone']
    assert p.typ.__name__ == 'Bool'
    assert p.qos == {'depth': 10, 'reliability': 'R'}, p.qos      # ship_avoid 구독(RELIABLE 10)과 같은 프로필


def test_node_empty_polygon_false_every_tick_and_log_none(node_mod, tmp_path):
    """폴리곤 비면 모든 wp_idx 에서 false, 매 틱 한 번(= /wp_mode 와 같은 주기). 시작 로그 '없음'."""
    mod, clock = node_mod
    node = _make(mod, tmp_path, [])
    assert node.timers[0][0] == node.period_sec == 0.5
    logs = _infos(node, '/avoid_zone')
    assert len(logs) == 1 and '없음' in logs[0] and '비었거나' in logs[0], logs
    for i in range(len(COURSE)):
        node.wp_idx = i
        n_mode, n_zone = len(node.pubs['/wp_mode'].msgs), len(_zone_vals(node))
        _tick(node, clock)
        assert len(node.pubs['/wp_mode'].msgs) == n_mode + 1
        assert len(_zone_vals(node)) == n_zone + 1, '틱마다 /avoid_zone 정확히 한 번'
        assert _zone_vals(node)[-1] is False, i


def test_node_filled_polygon_leg_values(node_mod, tmp_path):
    """폴리곤을 채우면 enable 스위치 없이 바로: 접근 다리 false, 회피 다리 true. GPS 없음·AUTO 대기에도 낸다."""
    mod, clock = node_mod
    node = _make(mod, tmp_path, ZONE)
    logs = _infos(node, '/avoid_zone')
    assert len(logs) == 1 and 'idx 2→3' in logs[0] and '없음' not in logs[0], logs
    assert node.have_fix is False and node.boat_mode is None      # GPS 없음 + AUTO 아님(미션 전진 안 함)
    got = []
    for i in range(len(COURSE)):
        node.wp_idx = i
        _tick(node, clock)
        got.append(_zone_vals(node)[-1])
        assert node.wp_idx == i, '스텁 틱에서 WP 가 전진하면 안 된다(AUTO 대기)'
    assert got == [False, False, False, True, False], got


def test_node_filled_polygon_no_leg_logs_none(node_mod, tmp_path):
    """폴리곤 안 mode 5 WP 가 하나뿐이면 다리 없음 → 늘 false + 시작 로그 '없음'(이유 다름)."""
    mod, clock = node_mod
    one = poly_box(-5.0, 5.0, 0.0, 10.0)        # 회피 시작(5 m)만 안
    node = _make(mod, tmp_path, one)
    logs = _infos(node, '/avoid_zone')
    assert len(logs) == 1 and '없음' in logs[0] and '연달아' in logs[0], logs
    for i in range(len(COURSE)):
        node.wp_idx = i
        _tick(node, clock)
        assert _zone_vals(node)[-1] is False, i


def test_node_course_end_false_every_tick(node_mod, tmp_path):
    """코스 끝: /wp_mode 는 멈추고 /avoid_zone 은 false 를 매 틱 계속 낸다."""
    mod, clock = node_mod
    node = _make(mod, tmp_path, ZONE)
    node.wp_idx = 3
    _tick(node, clock)
    assert _zone_vals(node)[-1] is True
    node.wp_idx = len(COURSE)
    n_mode = len(node.pubs['/wp_mode'].msgs)
    for _ in range(3):
        n_zone = len(_zone_vals(node))
        _tick(node, clock)
        assert len(_zone_vals(node)) == n_zone + 1 and _zone_vals(node)[-1] is False
    assert len(node.pubs['/wp_mode'].msgs) == n_mode, '코스 끝에는 /wp_mode 를 내지 않는다(기존 동작)'


def test_node_zone_after_wp_mode_same_tick(node_mod, tmp_path):
    """한 틱 안 순서: /wp_mode 다음 /avoid_zone(같은 wp_idx 기준). 전진 판정 전에 낸다."""
    mod, clock = node_mod
    src = open(mod.__file__, encoding='utf-8').read()
    i_mode = src.index("self.pub_mode.publish(Int32(data=wp_mode))")
    i_zone = src.index("self.pub_avoid_zone.publish(Bool(data=avoid_leg_on(self.avoid_wps, self.wp_idx)))")
    i_adv = src.index("event, self.t_start, remain = wp_step(")
    assert i_mode < i_zone < i_adv
    assert src.count("'/avoid_zone'") == 1, '발행자 생성은 한 곳'
