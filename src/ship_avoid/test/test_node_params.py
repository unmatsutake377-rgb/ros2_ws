#!/usr/bin/env python3
"""ship_avoid 노드 래퍼 정적·스텁 테스트 (계약서_도리도리회피_launch_20261113 C1). ROS 없이 돈다.

    tools/run_tests.sh ship_avoid

지키는 것
  · declare_parameter 가 전부 `self.declare_parameter('키', 리터럴)` 꼴(test_launch_params._declared 가 읽는 꼴)
  · 선언 리터럴 == core.AvoidParams 기본값 == config/ship_avoid.yaml (키 집합·값·타입 — 3.0 ↔ 3)
  · avoid_mode_logic 의 복귀 기본값 == 선언 리터럴 == yaml
  · 파라미터 선언이 구독·타이머·발행자 생성보다 먼저, time.time() 금지, 로거 변수 저장 금지(§1-7)
  · plan_tick 판정: lidar 는 GPS·yaw 없어도 20000, doridori + 검출 침묵이면 20000(코어 안 켬),
    /wp_mode stale 이면 발행 안 함, 이상값 → ERROR + lidar
  · 스텁 rclpy 로 실제 노드를 돌려 위 동작을 끝까지 확인
"""

import ast
import dataclasses
import importlib
import math
import os
import re
import sys
import types

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from ship_avoid import avoid_mode_logic as ml  # noqa: E402
from ship_avoid.core import AvoidParams  # noqa: E402

PKG = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
NODE_SRC = os.path.join(PKG, 'ship_avoid', 'ship_avoid.py')
YAML = os.path.join(PKG, 'config', 'ship_avoid.yaml')
NODE_NAME = 'ship_avoid'
# AvoidParams 가 아닌 노드 자체 파라미터
NODE_ONLY = {'active_wp_mode', 'period_sec', 'gps_stale_sec', 'yaw_stale_sec', 'detections_quiet_warn_sec',
             'lidar_forward_deg', 'pitch_sign', 'virtual_extra_m', 'avoid_mode', 'wp_mode_stale_sec'}


def _tree():
    with open(NODE_SRC, encoding='utf-8') as f:
        return ast.parse(f.read(), filename=NODE_SRC)


def _declares():
    """{키: (기본값, 줄)}. 모든 declare_parameter 호출이 self.declare_parameter('키', 리터럴) 이어야 한다."""
    out = {}
    for n in ast.walk(_tree()):
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == 'declare_parameter':
            assert isinstance(n.func.value, ast.Name) and n.func.value.id == 'self', f'{n.lineno}: self. 로 부를 것'
            assert len(n.args) >= 2 and isinstance(n.args[0], ast.Constant), f'{n.lineno}: 키·기본값 리터럴 아님'
            assert n.args[0].value not in out, f'{n.args[0].value} 중복 선언'
            out[n.args[0].value] = (ast.literal_eval(n.args[1]), n.lineno)
    assert out, 'declare_parameter 0개'
    return out


def _yaml_params():
    """`/ship_avoid:` → `ros__parameters:` 아래 평평한 key: value (pyyaml 없이)."""
    out, state = {}, 'seek'
    with open(YAML, encoding='utf-8') as f:
        for raw in f:
            line = re.sub(r'\s+#.*$', '', raw.rstrip('\n')).rstrip()
            if not line.strip() or line.lstrip().startswith('#'):
                continue
            if not line.startswith(' '):
                if state != 'seek':
                    break
                state = 'section' if line == f'/{NODE_NAME}:' else 'seek'
                continue
            if state == 'section' and line.strip() == 'ros__parameters:':
                state = 'params'
                continue
            if state == 'params':
                m = re.match(r'^\s+([A-Za-z_]\w*):\s*(\S.*)$', line)
                assert m, f'해석 못 하는 줄 {raw!r}'
                v = m.group(2).strip()
                if v[0] in '"\'':
                    assert v[-1] == v[0], f'따옴표 짝 {raw!r}'
                    val = v[1:-1]
                elif re.match(r'^[-+]?\d+$', v):
                    val = int(v)
                else:
                    val = float(v)
                assert m.group(1) not in out, f'yaml {m.group(1)} 중복'
                out[m.group(1)] = val
    assert out, 'yaml 파라미터를 못 읽음'
    return out


# ------------------------------------------------------------ 기본값 == AvoidParams == yaml

def test_declared_literals_cover_avoid_params():
    dec = {k: v for k, (v, _) in _declares().items()}
    p = AvoidParams()
    names = {f.name for f in dataclasses.fields(AvoidParams)}
    assert set(dec) == names | NODE_ONLY, f'선언 키 차이 {set(dec) ^ (names | NODE_ONLY)}'
    for k in names:
        d = getattr(p, k)
        assert dec[k] == d and type(dec[k]) is type(d), f'{k}: 선언 {dec[k]!r} ≠ AvoidParams {d!r}'


def test_yaml_equals_declared_with_types():
    dec = {k: v for k, (v, _) in _declares().items()}
    y = _yaml_params()
    assert set(y) == set(dec), f'yaml 키 차이 {set(y) ^ set(dec)}'
    for k, v in y.items():
        assert v == dec[k] and type(v) is type(dec[k]), f'{k}: yaml {v!r} ≠ 선언 {dec[k]!r}'  # noqa: E721


def test_logic_fallback_defaults_equal_declared():
    dec = {k: v for k, (v, _) in _declares().items()}
    assert dec['avoid_mode'] == ml.DEFAULT_AVOID_MODE == 'lidar'
    assert dec['wp_mode_stale_sec'] == ml.DEFAULT_WP_MODE_STALE_SEC == 1.0
    assert dec['detections_quiet_warn_sec'] == ml.DEFAULT_DET_QUIET_SEC
    assert ml.CANDIDATE_INVALID == 20000.0
    from ship_avoid import core
    assert ml.CANDIDATE_INVALID == core.CANDIDATE_INVALID


def test_node_name():
    names = [n.args[0].value for n in ast.walk(_tree())
             if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == '__init__'
             and isinstance(n.func.value, ast.Call) and getattr(n.func.value.func, 'id', None) == 'super'
             and n.args and isinstance(n.args[0], ast.Constant)]
    assert names == [NODE_NAME]


def test_declare_before_subscriptions_and_timers():
    last_decl = max(line for _, line in _declares().values())
    uses = [n.lineno for n in ast.walk(_tree()) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
            and n.func.attr in ('create_subscription', 'create_timer', 'create_publisher')]
    assert uses and last_decl < min(uses), '파라미터 선언이 구독·타이머·발행자 생성보다 먼저여야 한다'


def test_no_time_time_and_logger_not_stored():
    src = open(NODE_SRC, encoding='utf-8').read()
    assert 'time.time(' not in src, '§1-5: time.monotonic() 사용'
    levels = {'debug', 'info', 'warn', 'warning', 'error', 'fatal'}
    tree = _tree()
    called = {id(n.func) for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)}
    for n in ast.walk(tree):
        if (isinstance(n, ast.Attribute) and n.attr in levels and isinstance(n.value, ast.Call)
                and getattr(n.value.func, 'attr', None) == 'get_logger'):
            assert id(n) in called, f'{n.lineno}: 로거 메서드를 호출하지 않고 담음(§1-7)'
        if isinstance(n, ast.Assign) and isinstance(n.value, ast.Call) \
                and getattr(n.value.func, 'attr', None) == 'get_logger':
            pytest.fail(f'{n.lineno}: get_logger() 를 변수에 담음(§1-7)')


# ------------------------------------------------------------ 순수 판정 (avoid_mode_logic)

def _plan(mode='lidar', wp=5, wp_t=99.9, now=100.0, stale=1.0, det_t=None, since=None, quiet=5.0):
    return ml.plan_tick(mode, wp, wp_t, 5, now, stale, det_t, since, quiet)


def test_check_avoid_mode():
    assert ml.check_avoid_mode('lidar') == ('lidar', None)
    assert ml.check_avoid_mode('doridori') == ('doridori', None)
    for bad in ('Lidar', 'DORIDORI', '', 'dori', None, 1):
        mode, err = ml.check_avoid_mode(bad)
        assert mode == 'lidar' and err and 'avoid_mode' in err


def test_check_stale_sec():
    assert ml.check_stale_sec('x', 1.5, 1.0) == (1.5, None)
    for bad in (0.0, -1.0, float('nan'), float('inf'), 'a', None):
        v, err = ml.check_stale_sec('x', bad, 1.0)
        assert v == 1.0 and err


def test_plan_lidar_is_20000_regardless_of_gps_yaw_and_detections():
    s = _plan('lidar')
    assert s == ml.LIDAR and ml.fixed_candidate(s) == 20000.0 and ml.is_active(s)
    assert _plan('lidar', det_t=99.0) == ml.LIDAR


def test_plan_not_our_mode_or_never_received_is_idle():
    for s in (_plan(wp=3), _plan(wp=None, wp_t=None), _plan(wp=8, wp_t=50.0)):
        assert s == ml.IDLE and ml.fixed_candidate(s) is None and not ml.is_active(s)


def test_plan_wp_mode_stale_stops():
    s = _plan('lidar', wp_t=98.9)                      # 1.1 s 묵음
    assert s == ml.WP_STALE and ml.fixed_candidate(s) is None and not ml.is_active(s)
    assert _plan('doridori', wp_t=98.9, det_t=99.9) == ml.WP_STALE
    assert _plan('lidar', wp_t=99.0) == ml.LIDAR       # 정확히 1.0 s 는 아직 신선


def test_plan_doridori_detection_silence_falls_back():
    assert _plan('doridori', det_t=None, since=100.0) == ml.DET_WAIT       # 진입 직후, 한 번도 못 받음
    assert _plan('doridori', det_t=None, since=94.0) == ml.DET_DEAD        # 진입 후 6 s 동안 한 번도 없음
    assert _plan('doridori', det_t=94.0, since=80.0) == ml.DET_DEAD        # 6 s 조용
    assert _plan('doridori', det_t=None, since=None) == ml.DET_WAIT        # 이번 틱 진입
    for s in (ml.DET_WAIT, ml.DET_DEAD):
        assert ml.fixed_candidate(s) == 20000.0 and ml.is_active(s)
    s = _plan('doridori', det_t=96.0, since=80.0)                          # 4 s 전 수신 → 정상
    assert s == ml.DORI and ml.fixed_candidate(s) is None and ml.is_active(s)


# ------------------------------------------------------------ 스텁 rclpy 로 노드 돌리기

class _Logger:
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
    def __init__(self, topic):
        self.topic = topic
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


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

    def declare_parameter(self, key, default):
        return _Param(OVERRIDES.get(key, default))

    def get_logger(self):
        return self.logger

    def create_subscription(self, typ, topic, cb, qos):
        self.subs[topic] = cb

    def create_publisher(self, typ, topic, qos):
        self.pubs[topic] = _Pub(topic)
        return self.pubs[topic]

    def create_timer(self, period, cb):
        self.timers.append((period, cb))


class _Clock:
    t = 100.0

    def monotonic(self):
        return self.t


@pytest.fixture
def node_mod(monkeypatch):
    """rclpy·msgs 스텁을 sys.modules 에 잠시 넣고 노드 모듈을 새로 import (테스트 뒤 원래대로)."""
    def stub(name, **attrs):
        m = types.ModuleType(name)
        for k, v in attrs.items():
            setattr(m, k, v)
        monkeypatch.setitem(sys.modules, name, m)
        return m

    rclpy = stub('rclpy', init=lambda **k: None, spin=lambda n: None, try_shutdown=lambda: None)
    rclpy.node = stub('rclpy.node', Node=_Node)
    rclpy.qos = stub('rclpy.qos', QoSProfile=lambda **k: dict(k),
                     ReliabilityPolicy=types.SimpleNamespace(BEST_EFFORT='BE', RELIABLE='R'),
                     HistoryPolicy=types.SimpleNamespace(KEEP_LAST='KL'))
    stub('std_msgs')
    stub('std_msgs.msg', Float32=_msg_type('Float32'), Float32MultiArray=_msg_type('Float32MultiArray'),
         Float64=_msg_type('Float64'), Int32=_msg_type('Int32'), String=_msg_type('String'))
    stub('sensor_msgs')
    stub('sensor_msgs.msg', Imu=_msg_type('Imu'), LaserScan=_msg_type('LaserScan'),
         NavSatFix=_msg_type('NavSatFix'))
    monkeypatch.delitem(sys.modules, 'ship_avoid.ship_avoid', raising=False)
    mod = importlib.import_module('ship_avoid.ship_avoid')
    clock = _Clock()
    monkeypatch.setattr(mod, 'time', clock)
    OVERRIDES.clear()
    yield mod, clock
    OVERRIDES.clear()
    sys.modules.pop('ship_avoid.ship_avoid', None)


def _tick(node):
    assert len(node.timers) == 1
    node.timers[0][1]()


def _cands(node):
    return [m.data for m in node.pubs['/candidate_angle'].msgs]


def _errors(node, text):
    return [m for s, m in node.logger.lines if s == 'ERROR' and text in m]


def _wp(node, clock, mode):
    node.subs['/wp_mode'](_Msg(data=mode))


def _pose(node, clock):
    node.subs['/ublox_gps_node/fix'](_Msg(status=_Msg(status=0), latitude=35.0, longitude=129.0))
    node.subs['/imu/yaw'](_Msg(data=0.0))


def test_node_lidar_default_publishes_20000_without_gps_yaw(node_mod):
    mod, clock = node_mod
    node = mod.ShipAvoid()
    assert node.name == NODE_NAME and node.avoid_mode == 'lidar' and node.wp_stale == 1.0
    _tick(node)
    assert _cands(node) == [], '/wp_mode 못 받음 → 발행 안 함'
    _wp(node, clock, 3)
    _tick(node)
    assert _cands(node) == [], '담당 모드 아님'
    _wp(node, clock, 5)
    for k in range(5):                         # GPS·yaw 없음
        clock.t = 100.0 + 0.1 * k
        _tick(node)
    assert _cands(node) == [20000.0] * 5
    assert [m.data for m in node.pubs['/avoid_state'].msgs] == ['LIDAR'] * 5
    assert node.pubs['/virtual_obstacle_state'].msgs == [], 'lidar 는 지도가 없으니 침묵'
    assert not node.core.active
    assert not [m for s, m in node.logger.lines if s == 'ERROR']


def test_node_wp_mode_stale_stops_and_errors(node_mod):
    mod, clock = node_mod
    node = mod.ShipAvoid()
    _wp(node, clock, 5)
    _tick(node)
    assert _cands(node) == [20000.0]
    clock.t = 101.05                           # 1.05 s 묵음
    _tick(node)
    assert _cands(node) == [20000.0], 'stale → 발행 중지'
    assert _errors(node, '/wp_mode')
    _wp(node, clock, 5)                        # 다시 들어오면 재개
    _tick(node)
    assert _cands(node) == [20000.0, 20000.0]


def test_node_stale_in_other_mode_is_silent_without_error(node_mod):
    mod, clock = node_mod
    node = mod.ShipAvoid()
    _wp(node, clock, 8)                        # 코스 끝(north 가 /wp_mode 를 멈춤) — 담당 아님
    clock.t = 105.0
    _tick(node)
    assert _cands(node) == [] and not _errors(node, '/wp_mode')


def test_node_invalid_avoid_mode_error_and_lidar(node_mod):
    mod, clock = node_mod
    OVERRIDES.update(avoid_mode='Doridori', wp_mode_stale_sec=0.0)
    node = mod.ShipAvoid()
    assert node.avoid_mode == 'lidar' and node.wp_stale == ml.DEFAULT_WP_MODE_STALE_SEC
    assert _errors(node, 'avoid_mode') and _errors(node, 'wp_mode_stale_sec')
    _wp(node, clock, 5)
    _tick(node)
    assert _cands(node) == [20000.0]


def test_node_overrides_reach_avoid_params(node_mod):
    mod, _ = node_mod
    OVERRIDES.update(half_l=0.885, confirm_n=4, dori_angle_deg=45.0)
    node = mod.ShipAvoid()
    assert node.core.p.half_l == 0.885 and node.core.p.confirm_n == 4 and node.core.p.dori_angle_deg == 45.0
    assert node.core.p.half_w == AvoidParams().half_w


def test_node_doridori_detection_silence_fallback_then_core(node_mod):
    mod, clock = node_mod
    OVERRIDES.update(avoid_mode='doridori')
    node = mod.ShipAvoid()
    assert node.avoid_mode == 'doridori'
    _wp(node, clock, 5)
    _pose(node, clock)
    _tick(node)                                # 검출 한 번도 없음 → 20000, 코어 안 켬, 아직 유예(ERROR 없음)
    assert _cands(node) == [20000.0] and not node.core.active
    assert node.pubs['/avoid_state'].msgs[-1].data.startswith('DORIDORI_NO_DET')
    assert not _errors(node, '/avoid_detections')
    for k in range(1, 61):                     # 6 s 동안 계속 침묵
        clock.t = 100.0 + 0.1 * k
        _wp(node, clock, 5)
        _pose(node, clock)
        _tick(node)
    assert set(_cands(node)) == {20000.0} and not node.core.active
    assert _errors(node, '/avoid_detections'), '진입 후 quiet 초 넘게 침묵 → ERROR'

    node.subs['/avoid_detections'](_Msg(data=[]))      # 검출기 하트비트(빈 배열) → 정상 경로
    n = len(_cands(node))
    clock.t += 0.1
    _wp(node, clock, 5)
    _pose(node, clock)
    _tick(node)
    assert node.core.active, '검출이 들어오면 도리도리 코어 활성'
    new = _cands(node)[n:]
    assert len(new) == 1 and math.isfinite(new[0])
    assert node.pubs['/virtual_obstacle_state'].msgs, '코어 경로에선 가상 장애물 배열을 낸다'

    clock.t += 5.5                             # 다시 침묵 → 20000 + 코어 비활성
    _wp(node, clock, 5)
    _pose(node, clock)
    _tick(node)
    assert _cands(node)[-1] == 20000.0 and not node.core.active


def test_node_doridori_gps_missing_with_detections_stays_silent(node_mod):
    mod, clock = node_mod
    OVERRIDES.update(avoid_mode='doridori')
    node = mod.ShipAvoid()
    _wp(node, clock, 5)
    node.subs['/avoid_detections'](_Msg(data=[]))
    _tick(node)                                # 검출 살아 있음, GPS·yaw 없음 → 지금처럼 발행 중지 + ERROR
    assert _cands(node) == [] and _errors(node, 'GPS')
