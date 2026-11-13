#!/usr/bin/env python3
"""perception_fusion 노드 래퍼 정적·스텁 테스트 (계약서_인식융합_20261112 §6·§8). ROS 없이 돈다.

    tools/run_tests.sh perception_fusion

지키는 것
  · 기본값 == yaml == fusion_logic.DEFAULTS (코드 기본값·yaml·순수 로직 셋이 같은 값, 타입까지 — 3.0 ↔ 3)
  · super().__init__('perception_fusion') = yaml 절 이름 = launch name=
  · 파라미터 선언이 구독·타이머 생성보다 먼저
  · time.time() 금지, 로거 호출을 변수에 담지 않음(§1-7)
  · 스텁 rclpy 로 실제 노드를 돌려: 스캔 → /fused_objects [1, 8, N, ...], 둘 다 stale → 발행 안 함,
    LiDAR stale WARN 은 5 s 주기, 한 호출 줄이 두 심각도를 쓰지 않음, 10000 센티넬 무시, 무효 파라미터 → ERROR
"""

import ast
import importlib
import math
import os
import re
import sys
import types

import pytest

from perception_fusion import fusion_logic as fl

PKG = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
NODE_SRC = os.path.join(PKG, 'perception_fusion', 'perception_fusion.py')
YAML = os.path.join(PKG, 'config', 'perception_fusion.yaml')
NODE_NAME = 'perception_fusion'


def _tree():
    with open(NODE_SRC, encoding='utf-8') as f:
        return ast.parse(f.read(), filename=NODE_SRC)


def _declares():
    out = {}
    for n in ast.walk(_tree()):
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == 'declare_parameter':
            assert len(n.args) >= 2 and isinstance(n.args[0], ast.Constant), f'{n.lineno}: 키·기본값 리터럴 아님'
            assert n.args[0].value not in out, f'{n.args[0].value} 중복 선언'
            out[n.args[0].value] = (ast.literal_eval(n.args[1]), n.lineno)
    return out


def _yaml_params():
    """`/perception_fusion:` → `ros__parameters:` 아래 평평한 key: value (pyyaml 없이)."""
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
                    val = v[1:-1]
                elif re.match(r'^[-+]?\d+$', v):
                    val = int(v)
                else:
                    val = float(v)
                out[m.group(1)] = val
    assert out, 'yaml 파라미터를 못 읽음'
    return out


# ------------------------------------------------------------ 기본값 == yaml == DEFAULTS

def test_declared_defaults_equal_logic_defaults():
    dec = {k: v for k, (v, _) in _declares().items()}
    assert dec == fl.DEFAULTS
    for k, v in dec.items():
        assert type(v) is type(fl.DEFAULTS[k]), f'{k}: 타입 {type(v)} ≠ {type(fl.DEFAULTS[k])}'


def test_yaml_equals_defaults_with_types():
    y = _yaml_params()
    assert set(y) == set(fl.DEFAULTS), f'yaml 키 차이 {set(y) ^ set(fl.DEFAULTS)}'
    for k, v in y.items():
        assert v == fl.DEFAULTS[k] and type(v) is type(fl.DEFAULTS[k]), f'{k}: yaml {v!r} ≠ 기본값 {fl.DEFAULTS[k]!r}'


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
    called = set()
    for n in ast.walk(tree):
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute):
            called.add(id(n.func))
    for n in ast.walk(tree):
        if (isinstance(n, ast.Attribute) and n.attr in levels and isinstance(n.value, ast.Call)
                and getattr(n.value.func, 'attr', None) == 'get_logger'):
            assert id(n) in called, f'{n.lineno}: 로거 메서드를 호출하지 않고 담음(§1-7)'
        if isinstance(n, ast.Assign) and isinstance(n.value, ast.Call) \
                and getattr(n.value.func, 'attr', None) == 'get_logger':
            pytest.fail(f'{n.lineno}: get_logger() 를 변수에 담음(§1-7) — 호출 지점마다 self.get_logger().<level>')


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
        self.subs[topic] = (cb, qos)

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

    rclpy = stub('rclpy', init=lambda **k: None, spin=lambda n: None, ok=lambda: False, shutdown=lambda: None)
    rclpy.node = stub('rclpy.node', Node=_Node)
    rclpy.qos = stub('rclpy.qos', QoSProfile=lambda **k: dict(k),
                     ReliabilityPolicy=types.SimpleNamespace(BEST_EFFORT='BE', RELIABLE='R'),
                     HistoryPolicy=types.SimpleNamespace(KEEP_LAST='KL'))
    stub('std_msgs')
    stub('std_msgs.msg', Float32=_msg_type('Float32'), Float32MultiArray=_msg_type('Float32MultiArray'),
         Int32=_msg_type('Int32'), String=_msg_type('String'))
    stub('sensor_msgs')
    stub('sensor_msgs.msg', LaserScan=_msg_type('LaserScan'), Image=_msg_type('Image'),
         CameraInfo=_msg_type('CameraInfo'))
    monkeypatch.delitem(sys.modules, 'perception_fusion.perception_fusion', raising=False)
    mod = importlib.import_module('perception_fusion.perception_fusion')
    clock = _Clock()
    monkeypatch.setattr(mod, 'time', clock)
    OVERRIDES.clear()
    yield mod, clock
    OVERRIDES.clear()
    sys.modules.pop('perception_fusion.perception_fusion', None)


def _scan_msg(rel_r, inc_deg=0.25):
    n = int(round(360.0 / inc_deg))
    ranges = [float('inf')] * n
    for rel, r in rel_r.items():
        ranges[int(round((80.0 + rel + 180.0) / inc_deg)) % n] = r
    return _Msg(ranges=ranges, angle_min=math.radians(-180.0), angle_increment=math.radians(inc_deg),
                range_min=0.15, range_max=25.0)


def _timer(node, hz):
    cbs = [cb for period, cb in node.timers if abs(period - 1.0 / hz) < 1e-9]
    assert len(cbs) == 1, f'{hz} Hz 타이머 {len(cbs)}개'
    return cbs[0]


def test_node_publishes_and_stops(node_mod):
    mod, clock = node_mod
    node = mod.PerceptionFusion()
    assert node.name == NODE_NAME
    pub = node.pubs['/fused_objects']
    status = node.pubs['/fusion_status']
    tick = _timer(node, 10.0)
    stick = _timer(node, 1.0)
    for topic in ('/scan', '/camera/camera/depth/image_rect_raw', '/camera/camera/depth/camera_info'):
        assert node.subs[topic][1]['reliability'] == 'BE' and node.subs[topic][1]['depth'] == 1, topic
    for topic in ('/red_angle', '/green_angle', '/image_angle', '/image_color', '/wp_mode'):
        assert node.subs[topic][1]['reliability'] == 'R', topic

    tick()                                     # 아무것도 못 받음 → 발행 안 함
    assert pub.msgs == []
    scan_cb = node.subs['/scan'][0]
    for k in range(10):
        clock.t = 100.0 + 0.1 * k
        scan_cb(_scan_msg({-1.0 + 0.25 * i: 3.0 for i in range(9)}))
    node.subs['/red_angle'][0](_Msg(data=0.5))
    tick()
    data = list(pub.msgs[-1].data)
    assert data[:3] == [1.0, 8.0, 1.0] and len(data) == 11
    assert data[3] == 1.0 and data[5] == 3.0 and data[7] == fl.SRC_LIDAR
    node.subs['/red_angle'][0](_Msg(data=10000.0))   # 센티넬 → 색 지움
    tick()
    assert list(pub.msgs[-1].data)[3] == 0.0
    stick()
    assert status.msgs[-1].data.startswith('L1.00 D- agree- lidar_ok depth_stale n=1')

    n = len(pub.msgs)
    clock.t = 101.5                            # LiDAR 0.6 s 묵음, 뎁스 없음 → 둘 다 stale
    tick()
    assert len(pub.msgs) == n, '둘 다 stale → 발행 중단'
    stick()
    assert status.msgs[-1].data.endswith('(발행 중단)')
    warns = [m for s, m in node.logger.lines if s == 'WARN' and 'LiDAR /scan' in m]
    clock.t = 102.0
    tick()
    assert len([m for s, m in node.logger.lines if s == 'WARN' and 'LiDAR /scan' in m]) == len(warns), '5 s 주기'
    clock.t = 106.6
    tick()
    assert len([m for s, m in node.logger.lines if s == 'WARN' and 'LiDAR /scan' in m]) == len(warns) + 1


def test_node_depth_path(node_mod):
    mod, clock = node_mod
    import numpy as np
    node = mod.PerceptionFusion()
    pub = node.pubs['/fused_objects']
    tick = _timer(node, 10.0)
    info_cb = node.subs['/camera/camera/depth/camera_info'][0]
    depth_cb = node.subs['/camera/camera/depth/image_rect_raw'][0]
    w, h = 40, 20
    img = np.zeros((h, w), dtype='<u2')
    img[:, 18:23] = 2500                        # 가운데 열 → 정면 2.5 m
    raw = img.tobytes()
    msg = _Msg(height=h, width=w, step=2 * w, encoding='16UC1', is_bigendian=0, data=raw)
    depth_cb(msg)                               # camera_info 전 → 안 씀(WARN)
    tick()
    assert pub.msgs == []
    info_cb(_Msg(k=[100.0, 0.0, 20.0, 0.0, 100.0, 10.0, 0.0, 0.0, 1.0], width=w, height=h))
    for k in range(10):
        clock.t = 100.0 + 0.07 * k
        depth_cb(msg)
    tick()
    data = list(pub.msgs[-1].data)
    assert data[:3] == [1.0, 8.0, 1.0]
    assert data[5] == pytest.approx(2.5, abs=0.01) and data[7] == fl.SRC_DEPTH
    assert int(data[10]) & fl.FLAG_LIDAR_STALE, 'LiDAR 없음 → bit1'
    bad = _Msg(height=h, width=w, step=2 * w, encoding='rgb8', is_bigendian=0, data=raw)
    depth_cb(bad)
    assert any(s == 'ERROR' and 'rgb8' in m for s, m in node.logger.lines)


def test_node_invalid_param_error_and_default(node_mod):
    mod, _ = node_mod
    OVERRIDES.update(lidar_stale_sec=-1.0, weight_min=0.9)
    node = mod.PerceptionFusion()
    errs = [m for s, m in node.logger.lines if s == 'ERROR']
    assert any('lidar_stale_sec' in m for m in errs) and any('weight_min' in m for m in errs)
    assert node.p['lidar_stale_sec'] == 0.5 and node.p['weight_min'] == 0.3


def test_node_disagree_policy_default_and_invalid(node_mod):
    """기본 = lidar(코드 리터럴 == yaml == DEFAULTS 는 위 두 테스트가 묶음). 이상값 → ERROR 한 줄 + lidar."""
    mod, _ = node_mod
    assert _declares()['disagree_policy'][0] == 'lidar' == _yaml_params()['disagree_policy']
    node = mod.PerceptionFusion()
    assert node.p['disagree_policy'] == 'lidar'
    assert not [m for s, m in node.logger.lines if s == 'ERROR']
    OVERRIDES.update(disagree_policy='average')
    node = mod.PerceptionFusion()
    errs = [m for s, m in node.logger.lines if s == 'ERROR' and 'disagree_policy' in m]
    assert len(errs) == 1 and node.p['disagree_policy'] == 'lidar'
    OVERRIDES.update(disagree_policy='min')
    assert mod.PerceptionFusion().p['disagree_policy'] == 'min'
