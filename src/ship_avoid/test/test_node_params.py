#!/usr/bin/env python3
"""ship_avoid 노드 래퍼 정적·스텁 테스트 (계약서_도리도리회피_launch_20261113 C1, v2_20261114 U1). ROS 없이 돈다.

    tools/run_tests.sh ship_avoid

지키는 것
  · declare_parameter 가 전부 `self.declare_parameter('키', 리터럴)` 꼴(test_launch_params._declared 가 읽는 꼴)
  · 선언 리터럴 == core.AvoidParams 기본값 == config/ship_avoid.yaml (키 집합·값·타입 — 3.0 ↔ 3)
  · avoid_mode_logic 의 복귀 기본값 == 선언 리터럴 == yaml
  · 파라미터 선언이 구독·타이머·발행자 생성보다 먼저, time.time() 금지, 로거 변수 저장 금지(§1-7)
  · plan_tick 판정: lidar 는 GPS·yaw·zone 없어도 20000, fused + zone 미수신·stale·false 면 20000(ZONE_OFF),
    fused + zone 안 + 검출 침묵이면 20000(코어 멈춤), /wp_mode stale 이면 발행 안 함, 이상값 → ERROR + lidar,
    옛 이름 doridori → fused + WARN
  · 기본 fused + zone·검출 발행자 없음(오늘 배) → mode 5 매 틱 20000, ERROR 0, WARN 1회
  · 지도 수명 = zone 구간(검출 침묵 뒤 복귀 시 지도·ψ0 유지, zone false·mode 이탈이면 초기화)
  · /avoid_detections VER 1 / STRIDE 6 파서 경계(형식 오류 → 버림 + ERROR, N=0 하트비트, age_s 초과 검출 버림)
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
             'lidar_forward_deg', 'pitch_sign', 'roll_sign', 'virtual_extra_m', 'avoid_mode', 'wp_mode_stale_sec',
             'avoid_zone_stale_sec', 'det_max_age_s', 'imu_stale_sec'}


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
                elif v in ('true', 'false'):
                    val = v == 'true'
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
    assert dec['avoid_mode'] == ml.DEFAULT_AVOID_MODE == 'fused'        # 사용자 결정 11-14
    assert ml.INVALID_AVOID_MODE == 'lidar'                              # 이상값은 안전 쪽
    assert dec['wp_mode_stale_sec'] == ml.DEFAULT_WP_MODE_STALE_SEC == 1.0
    assert dec['detections_quiet_warn_sec'] == ml.DEFAULT_DET_QUIET_SEC
    assert dec['avoid_zone_stale_sec'] == ml.DEFAULT_ZONE_STALE_SEC == 1.0
    assert dec['det_max_age_s'] == ml.DEFAULT_DET_MAX_AGE_S
    assert dec['imu_stale_sec'] == ml.DEFAULT_IMU_STALE_SEC == 1.0
    assert _yaml_params()['avoid_mode'] == 'fused'
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

def _plan(mode='lidar', wp=5, wp_t=99.9, now=100.0, stale=1.0, det_t=None, since=None, quiet=5.0,
          zone=True, zone_t=99.9, zstale=1.0):
    return ml.plan_tick(mode, wp, wp_t, 5, now, stale, det_t, since, quiet, zone, zone_t, zstale)


def test_check_avoid_mode():
    assert ml.check_avoid_mode('lidar') == ('lidar', None, None)
    assert ml.check_avoid_mode('fused') == ('fused', None, None)
    mode, err, note = ml.check_avoid_mode('doridori')              # 옛 이름 — 이상값 아님
    assert mode == 'fused' and err is None and note and '이름 바뀜' in note
    for bad in ('Lidar', 'DORIDORI', 'Fused', '', 'dori', None, 1):
        mode, err, note = ml.check_avoid_mode(bad)
        assert mode == 'lidar' and err and 'avoid_mode' in err and note is None


def test_check_stale_sec():
    assert ml.check_stale_sec('x', 1.5, 1.0) == (1.5, None)
    for bad in (0.0, -1.0, float('nan'), float('inf'), 'a', None):
        v, err = ml.check_stale_sec('x', bad, 1.0)
        assert v == 1.0 and err


def test_plan_lidar_is_20000_regardless_of_gps_yaw_and_detections():
    s = _plan('lidar')
    assert s == ml.LIDAR and ml.fixed_candidate(s) == 20000.0 and ml.is_active(s) and not ml.keeps_map(s)
    assert _plan('lidar', det_t=99.0) == ml.LIDAR


def test_plan_lidar_ignores_zone():
    for z, zt in ((None, None), (False, 99.9), (True, 99.9), (True, 90.0)):
        assert _plan('lidar', det_t=99.9, zone=z, zone_t=zt) == ml.LIDAR


def test_plan_fused_zone_never_stale_false_is_zone_off():
    for z, zt in ((None, None), (True, 98.9), (False, 99.9), (False, 98.0)):
        s = _plan('fused', det_t=99.9, since=80.0, zone=z, zone_t=zt)
        assert s == ml.ZONE_OFF and ml.fixed_candidate(s) == 20000.0 and ml.is_active(s) and not ml.keeps_map(s)
    assert _plan('fused', det_t=99.9, zone=True, zone_t=99.0) == ml.DORI      # 정확히 1.0 s 는 아직 신선
    # zone 판정이 검출 침묵 판정보다 먼저 — zone 없음이면 DET_DEAD(ERROR) 가 아니라 ZONE_OFF
    assert _plan('fused', det_t=None, since=50.0, zone=None, zone_t=None) == ml.ZONE_OFF
    assert _plan('fused', det_t=99.9, zone=True, zone_t=99.9) == ml.DORI
    # /wp_mode 판정이 zone 보다 먼저
    assert _plan('fused', wp=3, zone=True) == ml.IDLE
    assert _plan('fused', wp_t=98.0, zone=True) == ml.WP_STALE


def test_zone_status():
    assert ml.zone_status(None, None, 100.0, 1.0) == ml.ZONE_NEVER
    assert ml.zone_status(True, 98.9, 100.0, 1.0) == ml.ZONE_STALE
    assert ml.zone_status(False, 98.9, 100.0, 1.0) == ml.ZONE_STALE
    assert ml.zone_status(False, 99.5, 100.0, 1.0) == ml.ZONE_FALSE
    assert ml.zone_status(True, 99.5, 100.0, 1.0) == ml.ZONE_TRUE


def test_plan_not_our_mode_or_never_received_is_idle():
    for s in (_plan(wp=3), _plan(wp=None, wp_t=None), _plan(wp=8, wp_t=50.0)):
        assert s == ml.IDLE and ml.fixed_candidate(s) is None and not ml.is_active(s)


def test_plan_wp_mode_stale_stops():
    s = _plan('lidar', wp_t=98.9)                      # 1.1 s 묵음
    assert s == ml.WP_STALE and ml.fixed_candidate(s) is None and not ml.is_active(s)
    assert _plan('doridori', wp_t=98.9, det_t=99.9) == ml.WP_STALE
    assert _plan('lidar', wp_t=99.0) == ml.LIDAR       # 정확히 1.0 s 는 아직 신선


def test_plan_fused_detection_silence_falls_back():
    assert _plan('fused', det_t=None, since=100.0) == ml.DET_WAIT       # 진입 직후, 한 번도 못 받음
    assert _plan('fused', det_t=None, since=94.0) == ml.DET_DEAD        # 진입 후 6 s 동안 한 번도 없음
    assert _plan('fused', det_t=94.0, since=80.0) == ml.DET_DEAD        # 6 s 조용
    assert _plan('fused', det_t=None, since=None) == ml.DET_WAIT        # 이번 틱 진입
    for s in (ml.DET_WAIT, ml.DET_DEAD):
        assert ml.fixed_candidate(s) == 20000.0 and ml.is_active(s) and ml.keeps_map(s)
    s = _plan('fused', det_t=96.0, since=80.0)                          # 4 s 전 수신 → 정상
    assert s == ml.DORI and ml.fixed_candidate(s) is None and ml.is_active(s) and ml.keeps_map(s)
    for s in (ml.IDLE, ml.WP_STALE, ml.LIDAR, ml.ZONE_OFF):
        assert not ml.keeps_map(s)


# ------------------------------------------------------------ /avoid_detections 파서 (G1 (a))

def _det(*rows, ver=1, stride=6):
    out = [ver, stride, len(rows)]
    for r in rows:
        out += list(r)
    return out


RED = (0, 10.0, 0.2, float('nan'), 0.9, 0.1)
YEL = (2, -5.0, 0.15, 2.5, 0.8, 0.2)


def test_parse_heartbeat_and_rows():
    assert ml.parse_detections([1, 6, 0], 0.5) == ([], 0, None)              # N=0 = 살아 있음
    dets, n_old, err = ml.parse_detections(_det(RED, YEL), 0.5)
    assert err is None and n_old == 0 and len(dets) == 2
    c, b, v, r, conf, age = dets[0]
    assert (c, b, v, conf, age) == (0, 10.0, 0.2, 0.9, 0.1) and type(c) is int and math.isnan(r)  # noqa: E721
    assert dets[1] == (2, -5.0, 0.15, 2.5, 0.8, 0.2)
    dets, _, _ = ml.parse_detections(_det((0, 1.0, 0.2, float('inf'), 0.9, 0.1)), 0.5)
    assert math.isnan(dets[0][3]), 'range 무한 = 모름 → NaN'


def test_parse_format_errors_drop_message():
    bad = [
        [],                                    # 예전 빈 배열 하트비트 — 머리 없음
        [0, 10.0, 0.2, 3.0, 0.9],              # 예전 stride 5 형식
        [1, 6],                                # 머리 짧음
        _det(RED, ver=2), _det(RED, stride=5),
        [1, 6, 1] + list(RED)[:5],             # 길이 부족
        [1, 6, 1] + list(RED) + [0.0],         # 길이 초과
        [1, 6, 2] + list(RED),                 # N 과 길이 불일치
        [1, 6, -1], [1, 6, 0.5], [1, 6, float('nan')],
        _det((float('nan'), 1.0, 0.2, 3.0, 0.9, 0.1)),        # cls NaN
        _det((0.5, 1.0, 0.2, 3.0, 0.9, 0.1)),                 # cls 정수 아님
        _det((0, float('nan'), 0.2, 3.0, 0.9, 0.1)),          # bearing NaN
        _det((0, 1.0, float('inf'), float('nan'), 0.9, 0.1)),  # v_norm 무한
        _det((0, 1.0, 0.2, 3.0, 0.9, float('nan'))),          # age NaN
        _det((0, 1.0, 0.2, 3.0, 0.9, -0.1)),                  # age 음수
        ['a', 6, 0],
    ]
    for d in bad:
        dets, n_old, err = ml.parse_detections(d, 0.5)
        assert dets is None and err, d


def test_parse_age_limit_drops_only_old():
    rows = (RED, (0, 1.0, 0.2, 3.0, 0.9, 0.5), (0, 2.0, 0.2, 3.0, 0.9, 0.51))
    dets, n_old, err = ml.parse_detections(_det(*rows), 0.5)
    assert err is None and n_old == 1 and [d[1] for d in dets] == [10.0, 1.0]   # 정확히 0.5 는 받음
    dets, n_old, err = ml.parse_detections(_det((0, 2.0, 0.2, 3.0, 0.9, 9.0)), 0.5)
    assert dets == [] and n_old == 1 and err is None, '전부 묵어도 메시지는 살아 있음'


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
    stub('std_msgs.msg', Bool=_msg_type('Bool'), Float32=_msg_type('Float32'),
         Float32MultiArray=_msg_type('Float32MultiArray'),
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


def _warns(node, text):
    return [m for s, m in node.logger.lines if s == 'WARN' and text in m]


def _all_errors(node):
    return [m for s, m in node.logger.lines if s == 'ERROR']


def _zone(node, on):
    node.subs['/avoid_zone'](_Msg(data=on))


def _hb(node, *rows):
    node.subs['/avoid_detections'](_Msg(data=[1, 6, len(rows)] + [v for r in rows for v in r]))


def _step(node, clock, dt=0.1, zone=True, det=True, pose=True):
    """한 틱: 시각 진행 + /wp_mode 5 (+ zone·검출 하트비트·자세) + 타이머."""
    clock.t += dt
    _wp(node, clock, 5)
    if zone is not None:
        _zone(node, zone)
    if det:
        _hb(node)
    if pose:
        _pose(node, clock)
    _tick(node)


def test_node_default_fused_without_zone_and_detector_is_20000_no_error(node_mod):
    """오늘 배: 기본 fused, /avoid_zone·/avoid_detections 발행자 0 → mode 5 매 틱 20000, ERROR 0, WARN 1회."""
    mod, clock = node_mod
    node = mod.ShipAvoid()
    assert node.name == NODE_NAME and node.avoid_mode == 'fused' and node.wp_stale == 1.0
    assert node.zone_stale == 1.0 and node.det_max_age == ml.DEFAULT_DET_MAX_AGE_S
    assert '/avoid_zone' in node.subs
    _tick(node)
    assert _cands(node) == [], '/wp_mode 못 받음 → 발행 안 함'
    _wp(node, clock, 3)
    _tick(node)
    assert _cands(node) == [], '담당 모드 아님'
    for _ in range(100):                       # 10 s — 검출 quiet 5 s 를 훨씬 넘김, GPS·yaw 도 없음
        _step(node, clock, zone=None, det=False, pose=False)
    assert _cands(node) == [20000.0] * 100
    assert set(m.data for m in node.pubs['/avoid_state'].msgs) == {'FUSED_ZONE_OFF'}
    assert node.pubs['/virtual_obstacle_state'].msgs == []
    assert not node.core.active
    assert not _all_errors(node), '검출 침묵 DET_DEAD ERROR 가 찍히면 안 된다(zone 판정이 먼저)'
    assert len(_warns(node, '/avoid_zone')) == 1, 'zone 미수신은 WARN 1회만'
    _wp(node, clock, 8)                        # 코스 끝 쪽 — 담당 모드 밖
    clock.t += 0.1
    _tick(node)
    clock.t += 5.0                             # /wp_mode 도 멈춤(마지막 값 8)
    _tick(node)
    assert len(_cands(node)) == 100 and not _all_errors(node), '담당 모드 밖·코스 끝 거짓 ERROR 없음'


def test_node_lidar_publishes_20000_without_gps_yaw_and_ignores_zone(node_mod):
    mod, clock = node_mod
    OVERRIDES.update(avoid_mode='lidar')
    node = mod.ShipAvoid()
    assert node.avoid_mode == 'lidar'
    _wp(node, clock, 5)
    for k in range(5):                         # GPS·yaw·zone 없음
        clock.t = 100.0 + 0.1 * k
        _tick(node)
    for z in (True, False):                    # zone 이 와도 무관
        _zone(node, z)
        _hb(node)
        _tick(node)
    assert _cands(node) == [20000.0] * 7
    assert [m.data for m in node.pubs['/avoid_state'].msgs] == ['LIDAR'] * 7
    assert node.pubs['/virtual_obstacle_state'].msgs == [], 'lidar 는 지도가 없으니 침묵'
    assert not node.core.active
    assert not _all_errors(node) and not _warns(node, '/avoid_zone')


def test_node_doridori_alias_warns_and_runs_fused(node_mod):
    mod, _ = node_mod
    OVERRIDES.update(avoid_mode='doridori')
    node = mod.ShipAvoid()
    assert node.avoid_mode == 'fused'
    assert len(_warns(node, '이름 바뀜')) == 1 and not _all_errors(node)


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


def test_node_invalid_params_error_and_fallback(node_mod):
    mod, clock = node_mod
    OVERRIDES.update(avoid_mode='Fused', wp_mode_stale_sec=0.0, avoid_zone_stale_sec=-1.0, det_max_age_s='x')
    node = mod.ShipAvoid()
    assert node.avoid_mode == 'lidar' and node.wp_stale == ml.DEFAULT_WP_MODE_STALE_SEC
    assert node.zone_stale == ml.DEFAULT_ZONE_STALE_SEC and node.det_max_age == ml.DEFAULT_DET_MAX_AGE_S
    for k in ('avoid_mode', 'wp_mode_stale_sec', 'avoid_zone_stale_sec', 'det_max_age_s'):
        assert _errors(node, k), k
    _wp(node, clock, 5)
    _tick(node)
    assert _cands(node) == [20000.0]


def test_node_overrides_reach_avoid_params(node_mod):
    mod, _ = node_mod
    OVERRIDES.update(half_l=0.885, confirm_n=4, dori_angle_deg=45.0)
    node = mod.ShipAvoid()
    assert node.core.p.half_l == 0.885 and node.core.p.confirm_n == 4 and node.core.p.dori_angle_deg == 45.0
    assert node.core.p.half_w == AvoidParams().half_w


def test_node_zone_false_and_stale_fall_back(node_mod):
    mod, clock = node_mod
    node = mod.ShipAvoid()
    _step(node, clock, zone=False)             # zone false → 20000, 로그 없음
    assert _cands(node) == [20000.0] and not node.core.active
    assert not _all_errors(node) and not _warns(node, '/avoid_zone')
    _step(node, clock, zone=True)              # zone true + 검출 살아 있음 → 도리도리
    assert node.core.active and math.isfinite(_cands(node)[-1]) and _cands(node)[-1] != 20000.0
    clock.t += 1.05                            # zone 만 묵음(/wp_mode·검출은 옴)
    _step(node, clock, dt=0.0, zone=None)
    assert _cands(node)[-1] == 20000.0 and not node.core.active
    assert _errors(node, '/avoid_zone'), 'zone 받다가 묵으면 ERROR'
    assert node.pubs['/avoid_state'].msgs[-1].data == 'FUSED_ZONE_OFF'


def test_node_fused_detection_silence_fallback_then_core(node_mod):
    mod, clock = node_mod
    node = mod.ShipAvoid()
    _step(node, clock, dt=0.0, det=False)      # zone 안, 검출 한 번도 없음 → 20000, 코어 안 켬, 아직 유예
    assert _cands(node) == [20000.0] and not node.core.active
    assert node.pubs['/avoid_state'].msgs[-1].data.startswith('FUSED_NO_DET')
    assert not _errors(node, '/avoid_detections')
    for _ in range(60):                        # 6 s 동안 계속 침묵
        _step(node, clock, det=False)
    assert set(_cands(node)) == {20000.0} and not node.core.active
    assert _errors(node, '/avoid_detections'), '진입 후 quiet 초 넘게 침묵 → ERROR'

    n = len(_cands(node))
    _step(node, clock)                         # 하트비트(N=0) → 정상 경로
    assert node.core.active, '검출이 들어오면 도리도리 코어 활성'
    new = _cands(node)[n:]
    assert len(new) == 1 and math.isfinite(new[0])
    assert node.pubs['/virtual_obstacle_state'].msgs, '코어 경로에선 가상 장애물 배열을 낸다'

    _step(node, clock, dt=5.5, det=False)      # 다시 침묵 → 20000 + 코어 멈춤
    assert _cands(node)[-1] == 20000.0 and not node.core.active


def test_node_map_kept_through_detection_silence_in_zone(node_mod):
    """zone 구간 안 검출 침묵 → 코어 멈춤, 지도·ψ0 유지 → 검출 복귀 시 이어 감(C1 리뷰 경고 1)."""
    mod, clock = node_mod
    node = mod.ShipAvoid()
    _step(node, clock, dt=0.0)
    assert node.core.active and node.core.psi0 == 0.0
    node.core.map.update(1.0, 3.0, [[0.01, 0.0], [0.0, 0.01]], True, clock.t)
    tracks = list(node.core.map.tracks)
    _step(node, clock, dt=5.5, det=False)      # 검출 침묵(zone 은 true)
    assert _cands(node)[-1] == 20000.0 and not node.core.active
    assert node.core.map.tracks == tracks, '침묵만으로 지도를 버리면 안 된다'
    clock.t += 0.1                             # 복귀 — 그사이 배가 돌았다(yaw 30°)
    _wp(node, clock, 5)
    _zone(node, True)
    _hb(node)
    _pose(node, clock)
    node.subs['/imu/yaw'](_Msg(data=30.0))
    _tick(node)
    assert node.core.active and node.core.map.tracks == tracks
    assert node.core.psi0 == 0.0, '재개는 코스 축을 다시 잡지 않는다'
    assert [m for s, m in node.logger.lines if s == 'INFO' and '재개' in m]


def test_node_map_reset_when_zone_false_or_mode_leaves(node_mod):
    mod, clock = node_mod
    node = mod.ShipAvoid()
    _step(node, clock, dt=0.0)
    node.core.map.update(1.0, 3.0, [[0.01, 0.0], [0.0, 0.01]], True, clock.t)
    _step(node, clock, zone=False)             # zone false → 지도 초기화
    assert _cands(node)[-1] == 20000.0 and not node.core.active and node.core.map.tracks == []
    _step(node, clock, zone=True)              # 새 zone 구간 = 새로 활성(코스 축 다시 잡음)
    assert node.core.active
    node.core.map.update(1.0, 3.0, [[0.01, 0.0], [0.0, 0.01]], True, clock.t)
    _step(node, clock, dt=5.5, det=False)      # 침묵으로 멈춘 상태에서
    assert node.core.map.tracks
    clock.t += 0.1
    _wp(node, clock, 8)                        # 담당 모드 이탈 → 초기화
    _tick(node)
    assert not node.core.active and node.core.map.tracks == []


def test_node_detection_format_error_is_not_heartbeat(node_mod):
    mod, clock = node_mod
    node = mod.ShipAvoid()
    node.subs['/avoid_detections'](_Msg(data=[0, 10.0, 0.2, 3.0, 0.9]))     # 예전 stride 5
    node.subs['/avoid_detections'](_Msg(data=[]))
    assert node.last_det_t is None and _errors(node, '형식 오류')
    _step(node, clock, dt=0.0, det=False)
    assert _cands(node) == [20000.0] and not node.core.active
    _hb(node, (0, 1.0, 0.2, 3.0, 0.9, 5.0))                                  # 묵은 검출만 — 살아 있음 + WARN
    assert node.last_det_t == clock.t and _warns(node, 'age_s')


def test_node_doridori_gps_missing_with_detections_stays_silent(node_mod):
    mod, clock = node_mod
    node = mod.ShipAvoid()
    _wp(node, clock, 5)
    _zone(node, True)
    _hb(node)
    _tick(node)                                # zone·검출 살아 있음, GPS·yaw 없음 → 지금처럼 발행 중지 + ERROR
    assert _cands(node) == [] and _errors(node, 'GPS')


def _quat(roll_deg, pitch_deg):
    r, p = math.radians(roll_deg) / 2.0, math.radians(pitch_deg) / 2.0
    return _Msg(w=math.cos(r) * math.cos(p), x=math.sin(r) * math.cos(p),
                y=math.cos(r) * math.sin(p), z=-math.sin(r) * math.sin(p))


def test_node_imu_feeds_pitch_and_roll_with_signs(node_mod):
    """W2 ⑥: /imu/data 에서 피치·롤 둘 다 읽어 부호 파라미터를 곱해 코어로."""
    mod, _ = node_mod
    node = mod.ShipAvoid()
    node.subs['/imu/data'](_Msg(orientation=_quat(3.0, 2.0)))
    assert abs(node.core.pitch - 2.0) < 1e-6 and abs(node.core.roll - 3.0) < 1e-6 and node.imu_t is not None
    OVERRIDES.update(pitch_sign=-1.0, roll_sign=-1.0)
    node = mod.ShipAvoid()
    node.subs['/imu/data'](_Msg(orientation=_quat(3.0, 2.0)))
    assert abs(node.core.pitch + 2.0) < 1e-6 and abs(node.core.roll + 3.0) < 1e-6


def test_node_activation_without_imu_warns_mount_only(node_mod):
    mod, clock = node_mod
    node = mod.ShipAvoid()
    _step(node, clock, dt=0.0)
    assert node.core.active and len(_warns(node, '/imu/data')) == 1
    assert node.core.pitch == 0.0 and node.core.roll == 0.0       # 장착값(cam_pitch_deg·cam_roll_deg)만
    node = mod.ShipAvoid()
    node.subs['/imu/data'](_Msg(orientation=_quat(0.0, 1.0)))
    _step(node, clock)
    assert node.core.active and not _warns(node, '/imu/data')


def test_node_scan_width_check_uses_obstacle_r(node_mod):
    """W2 ③: 폭 검사 반경 = max(buoy_r, obstacle_r) — 넓은 흰 부표 군집도 LiDAR 부표로."""
    mod, clock = node_mod
    rng = [math.inf] * 720
    for a in range(520, 553):                     # 16° 폭 @ 3 m
        rng[a] = 3.0
    scan = _Msg(ranges=rng, angle_min=math.radians(-180.0), angle_increment=math.radians(0.5))
    OVERRIDES.update(obstacle_r=0.12)
    node = mod.ShipAvoid()
    _pose(node, clock)
    node.subs['/scan'](scan)
    assert node.core.buoys == []
    OVERRIDES.clear()
    node = mod.ShipAvoid()
    _pose(node, clock)
    node.subs['/scan'](scan)
    assert len(node.core.buoys) == 1


# ------------------------------------------------------------ W2b (계약서_흰부표_거리융합_v3 'W2 리뷰·검증 메모')

def test_camera_mount_defaults_are_mast_design_values():
    """⑧ 사용자 결정(11-14): OAK 수면 위 0.415 m·아래로 11° = 마스트 설계값(mast v2), 두 배 공통, 실측 아님.
    AvoidParams == 선언 리터럴 == yaml 은 위 테스트가 묶고, 여기선 값과 '실측 아님' 표시."""
    dec = {k: v for k, (v, _) in _declares().items()}
    y = _yaml_params()
    for key, want in (('cam_h', 0.415), ('cam_pitch_deg', 11.0)):
        assert getattr(AvoidParams(), key) == dec[key] == y[key] == want, key
    raw = open(YAML, encoding='utf-8').read().splitlines()
    src = open(NODE_SRC, encoding='utf-8').read().splitlines()
    for key in ('cam_h', 'cam_pitch_deg'):
        assert '설계값(mast v2), 실측 아님' in next(ln for ln in raw if ln.strip().startswith(key + ':'))
        assert '설계값(mast v2), 실측 아님' in next(ln for ln in src if f"declare_parameter('{key}'" in ln)


def test_imu_status():
    assert ml.imu_status(None, 100.0, 1.0) == ml.IMU_NEVER
    assert ml.imu_status(99.5, 100.0, 1.0) == ml.IMU_OK
    assert ml.imu_status(98.5, 100.0, 1.0) == ml.IMU_STALE


def test_node_imu_stale_zeroes_attitude_and_warns_periodically(node_mod):
    """⑦ 받다가 imu_stale_sec(1 s) 넘게 끊기면 피치·롤 0(장착값만) + 주기 WARN. 한 번도 못 받은 경우는 1회 WARN 만."""
    mod, clock = node_mod
    node = mod.ShipAvoid()
    node.subs['/imu/data'](_Msg(orientation=_quat(3.0, 2.0)))
    _step(node, clock)
    assert node.core.active and abs(node.core.pitch - 2.0) < 1e-6 and not _warns(node, '묵음')
    _step(node, clock, dt=0.5)
    assert abs(node.core.pitch - 2.0) < 1e-6                         # 1 s 안 = 그대로
    _step(node, clock, dt=0.7)
    assert node.core.pitch == 0.0 and node.core.roll == 0.0
    assert len(_warns(node, '/imu/data')) == 1 and _warns(node, '묵음')
    node.subs['/imu/data'](_Msg(orientation=_quat(1.0, -1.0)))       # 돌아오면 다시 씀
    _step(node, clock)
    assert abs(node.core.pitch + 1.0) < 1e-6 and abs(node.core.roll - 1.0) < 1e-6
    node = mod.ShipAvoid()                                           # 한 번도 못 받음(지금 배)
    for _ in range(30):
        _step(node, clock)
    assert node.core.active and len(_warns(node, '/imu/data')) == 1 and not _warns(node, '묵음')


def test_node_scan_marks_wide_clusters(node_mod):
    """⑤ scan_cb 가 cluster_buoys(classify=True) 의 넓은 군집 표시를 코어로 넘긴다."""
    mod, clock = node_mod
    rng = [math.inf] * 720
    for a in range(520, 553):                     # 16° @ 3 m — 흰 폭으로만 통과
        rng[a] = 3.0
    for a in range(600, 605):                     # 2° @ 2 m — 노랑 폭
        rng[a] = 2.0
    node = mod.ShipAvoid()
    _pose(node, clock)
    node.subs['/scan'](_Msg(ranges=rng, angle_min=math.radians(-180.0), angle_increment=math.radians(0.5)))
    assert node.core.buoy_wide == [True, False]
