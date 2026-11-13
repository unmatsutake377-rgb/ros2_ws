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
                elif v in ('true', 'false'):
                    val = v == 'true'
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


def test_oak_mount_defaults_are_mast_design_values():
    """사용자 결정(11-14): OAK 수면 위 0.415 m·아래로 11° = 마스트 설계값(mast v2), 두 배 공통, 실측 아님.
    코드 리터럴 == yaml == DEFAULTS 는 위 두 테스트가 묶고, 여기선 값과 '실측 아님' 표시를 묶는다."""
    dec = {k: v for k, (v, _) in _declares().items()}
    y = _yaml_params()
    for key, want in (('oak_cam_h_m', 0.415), ('oak_pitch_deg', 11.0)):
        assert fl.DEFAULTS[key] == dec[key] == y[key] == want, key
        assert type(dec[key]) is float and type(y[key]) is float, key
    raw = open(YAML, encoding='utf-8').read().splitlines()
    for key in ('oak_cam_h_m', 'oak_pitch_deg'):
        line = next(ln for ln in raw if ln.strip().startswith(key + ':'))
        assert '설계값(mast v2), 실측 아님' in line, line
    src = open(NODE_SRC, encoding='utf-8').read().splitlines()
    for key in ('oak_cam_h_m', 'oak_pitch_deg'):
        line = next(ln for ln in src if f"declare_parameter('{key}'" in ln)
        assert '설계값(mast v2), 실측 아님' in line, line


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
         Float64=_msg_type('Float64'), Int32=_msg_type('Int32'), String=_msg_type('String'))
    stub('sensor_msgs')
    stub('sensor_msgs.msg', LaserScan=_msg_type('LaserScan'), Image=_msg_type('Image'),
         CameraInfo=_msg_type('CameraInfo'), Imu=_msg_type('Imu'), NavSatFix=_msg_type('NavSatFix'))
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


# ------------------------------------------------------------ P1 카메라 경로 (스텁 노드)

def _oak_msg(rows):
    data = [1.0, 6.0, float(len(rows))]
    for r in rows:
        data += [float(v) for v in r]
    return _Msg(data=data)


def _feed_scan(node, clock, t0, rel_r):
    scan_cb = node.subs['/scan'][0]
    for k in range(10):
        clock.t = t0 + 0.05 * k
        scan_cb(_scan_msg(rel_r))


# _mono_row·삼각측량 장면은 높이 0.6 m·피치 0 기하로 합성 → 그 기하를 파라미터로 고정(설계값 기본과 무관).
GEOM = {'oak_cam_h_m': 0.6, 'oak_pitch_deg': 0.0}


def _mono_row(D, beta_deg, h=0.6, color=3):
    """피치·롤 0, 수면 점 (D, β) → /oak_detections 한 줄 (bearing_left, v_norm_bottom)."""
    x = -math.sin(math.radians(beta_deg)) * D
    z = math.cos(math.radians(beta_deg)) * D
    return (color, -math.degrees(math.atan2(x, z)), h / z, 2.0, 0.9, 0.0)


def test_node_p1_subscriptions_qos(node_mod):
    mod, _ = node_mod
    node = mod.PerceptionFusion()
    assert node.subs['/oak_detections'][1]['reliability'] == 'BE' and node.subs['/oak_detections'][1]['depth'] == 1
    for topic in ('/ublox_gps_node/fix', '/imu/yaw', '/imu/data'):
        assert node.subs[topic][1]['reliability'] == 'R' and node.subs[topic][1]['depth'] == 10, topic
    assert node.p['use_mono'] is False and node.p['use_triangulation'] is False


def _run_scene(mod, clock, overrides, feed_oak):
    """같은 LiDAR 장면 + (선택) /oak_detections → 발행 data 목록."""
    OVERRIDES.clear()
    OVERRIDES.update(overrides)
    clock.t = 100.0
    node = mod.PerceptionFusion()
    tick = _timer(node, 10.0)
    _feed_scan(node, clock, 100.0, {**{-1.0 + 0.25 * i: 3.0 for i in range(9)},
                                    **{19.0 + 0.25 * i: 5.0 for i in range(9)}})
    node.subs['/red_angle'][0](_Msg(data=0.5))
    if feed_oak:
        node.subs['/oak_detections'][0](_oak_msg([_mono_row(3.0, 0.0), _mono_row(4.0, -30.0)]))
    tick()
    clock.t += 0.1
    tick()
    return [list(m.data) for m in node.pubs['/fused_objects'].msgs], node


@pytest.mark.parametrize('overrides,feed', [
    ({'use_mono': True, 'use_triangulation': True}, False),       # 켜짐 + /oak_detections 없음
    ({}, True),                                                    # 꺼짐 + /oak_detections 들어옴
    ({'use_mono': False, 'use_triangulation': False}, True),
])
def test_node_bit_identical_without_camera_path(node_mod, overrides, feed):
    mod, clock = node_mod
    want, _ = _run_scene(mod, clock, {}, False)
    got, _ = _run_scene(mod, clock, overrides, feed)
    assert want and got == want


def test_node_mount_geometry_change_is_invisible_while_camera_off(node_mod):
    """use_mono·use_triangulation 기본 false → 장착 기본값(0.6/0° → 0.415/11°)을 바꿔도 /fused_objects 비트 동일."""
    mod, clock = node_mod
    want, _ = _run_scene(mod, clock, {'oak_cam_h_m': 0.6, 'oak_pitch_deg': 0.0}, True)
    got, _ = _run_scene(mod, clock, {}, True)
    assert fl.DEFAULTS['use_mono'] is False and fl.DEFAULTS['use_triangulation'] is False
    assert want and got == want


def test_node_mono_on_adds_object_and_bits(node_mod):
    mod, clock = node_mod
    base, _ = _run_scene(mod, clock, {}, False)
    got, node = _run_scene(mod, clock, {'use_mono': True, **GEOM}, True)
    objs = fl.decode(got[-1])
    old = fl.decode(base[-1])
    assert len(objs) == len(old) + 1, '−30° 4 m 단안 물체 하나 더(LiDAR 없음 → src 4)'
    near = min(objs, key=lambda o: o['range_m'])
    assert near['range_m'] == 3.0 and near['src_id'] == fl.SRC_LIDAR, 'LiDAR 우선 — 거리는 LiDAR'
    assert int(near['flags']) == fl.FLAG_SRC_LIDAR | fl.FLAG_SRC_MONO and near['color_id'] == 3.0
    mono = [o for o in objs if o['src_id'] == fl.SRC_MONO]
    assert len(mono) == 1 and mono[0]['range_m'] == pytest.approx(4.0, abs=1e-4)
    assert mono[0]['bearing_deg'] == pytest.approx(-30.0, abs=1e-4)
    assert any(s == 'WARN' and '/imu/data' in m for s, m in node.logger.lines), 'IMU 없음 WARN'
    _timer(node, 1.0)()
    st = node.pubs['/fusion_status'].msgs[-1].data
    assert 'oak_ok mono=2/2 tri=off imu_mount rtk_no' in st, st


def test_node_imu_warn_once_and_imu_used(node_mod):
    mod, clock = node_mod
    OVERRIDES.update(use_mono=True)
    node = mod.PerceptionFusion()
    tick = _timer(node, 10.0)
    for k in range(5):
        clock.t = 100.0 + 0.1 * k
        tick()
    warns = [m for s, m in node.logger.lines if s == 'WARN' and '/imu/data' in m]
    assert len(warns) == 1, '1회'
    s5 = math.sin(math.radians(2.5))
    node.subs['/imu/data'][0](_Msg(orientation=_Msg(x=0.0, y=s5, z=0.0, w=math.cos(math.radians(2.5)))))
    assert node.att.buf[-1][1] == pytest.approx((5.0, 0.0))
    assert fl.camera_attitude(node.att, clock.t, node.p)[2] is True


def test_node_oak_bad_format_error_periodic(node_mod):
    mod, clock = node_mod
    node = mod.PerceptionFusion()
    cb = node.subs['/oak_detections'][0]
    cb(_Msg(data=[2.0, 6.0, 0.0]))
    cb(_Msg(data=[1.0, 5.0, 0.0]))
    errs = [m for s, m in node.logger.lines if s == 'ERROR' and '/oak_detections' in m]
    assert len(errs) == 1 and 'VER' in errs[0] and node.oak is None
    clock.t += 6.0
    cb(_Msg(data=[1.0, 6.0, 2.0, 0.0]))
    assert len([m for s, m in node.logger.lines if s == 'ERROR' and '/oak_detections' in m]) == 2
    cb(_oak_msg([(3, 0.0, 0.1, 2.0, 0.9, -1.0)]))
    assert node.oak is not None and node.oak[1] == [] and any(s == 'WARN' and '무효 줄' in m
                                                              for s, m in node.logger.lines)


def _fix(lat, lon, status=2, sigma=0.015):
    return _Msg(latitude=lat, longitude=lon, status=_Msg(status=status),
                position_covariance=[sigma ** 2, 0.0, 0.0, 0.0, sigma ** 2, 0.0, 0.0, 0.0, 0.04],
                position_covariance_type=2)


@pytest.mark.parametrize('status,want_tri', [(2, True), (0, False)])
def test_node_triangulation_end_to_end(node_mod, status, want_tri):
    """동쪽 1 m/s 로 지나가며 부표 (6, 4) m 를 봄 → src 5 물체가 참 거리. GPS 비FIXED(status 0) 면 안 나옴."""
    mod, clock = node_mod
    OVERRIDES.update(use_triangulation=True, **GEOM)
    node = mod.PerceptionFusion()
    tick = _timer(node, 10.0)
    gps_cb, yaw_cb, oak_cb = (node.subs[t][0] for t in ('/ublox_gps_node/fix', '/imu/yaw', '/oak_detections'))
    scan_cb = node.subs['/scan'][0]
    lat0, lon0 = 35.2, 128.6
    k_m = math.pi / 180.0 * fl.EARTH_R_M
    for k in range(16):
        clock.t = 100.0 + 0.2 * k
        x = 0.2 * k
        gps_cb(_fix(lat0, lon0 + x / (k_m * math.cos(math.radians(lat0))), status=status))
        yaw_cb(_Msg(data=90.0))
        scan_cb(_scan_msg({}))
        beta = math.degrees(math.atan2(6.0 - x, 4.0))
        oak_cb(_oak_msg([(3, 90.0 - beta, 0.1, 2.0, 0.9, 0.0)]))
    tick()
    objs = fl.decode(node.pubs['/fused_objects'].msgs[-1].data)
    tri = [o for o in objs if o['src_id'] == fl.SRC_TRI]
    if not want_tri:
        assert tri == []
        return
    (o,) = tri
    assert o['range_m'] == pytest.approx(math.hypot(6.0 - 3.0, 4.0), abs=1e-3)
    assert o['bearing_deg'] == pytest.approx(90.0 - math.degrees(math.atan2(3.0, 4.0)), abs=0.05)
    assert int(o['flags']) == fl.FLAG_SRC_TRI and o['color_id'] == 3.0


# ------------------------------------------------------------ P1b-6 진단 (스텁 노드)

@pytest.mark.parametrize('pitch,warn', [(None, True), (3.0, False)])
def test_node_mono_default_pitch_start_warn(node_mod, pitch, warn):
    mod, _ = node_mod
    OVERRIDES.update(use_mono=True)
    if pitch is not None:
        OVERRIDES.update(oak_pitch_deg=pitch)
    node = mod.PerceptionFusion()
    got = [m for s, m in node.logger.lines if s == 'WARN' and 'oak_pitch_deg' in m]
    assert bool(got) is warn
    OVERRIDES.clear()
    node = mod.PerceptionFusion()
    assert not [m for s, m in node.logger.lines if 'oak_pitch_deg' in m], 'use_mono 꺼짐이면 경고 없음'


def _status_tokens(node):
    _timer(node, 1.0)()
    return node.pubs['/fusion_status'].msgs[-1].data


def test_node_rtk_fixed_freshness(node_mod):
    """rtk_fixed 는 신선한 FIXED 일 때만: NO_FIX 가 오면 바로, 1.5·gps_period_s 넘게 끊기면 rtk_no."""
    mod, clock = node_mod
    OVERRIDES.update(use_triangulation=True)
    node = mod.PerceptionFusion()
    tick = _timer(node, 10.0)
    gps_cb = node.subs['/ublox_gps_node/fix'][0]
    clock.t = 100.0
    gps_cb(_fix(35.2, 128.6))
    tick()
    assert ' rtk_fixed' in _status_tokens(node)
    clock.t = 100.5
    gps_cb(_fix(35.2, 128.6, status=-1))
    tick()
    assert ' rtk_no' in _status_tokens(node), 'NO_FIX → 바로'
    clock.t = 101.0
    gps_cb(_fix(35.2, 128.6))
    tick()
    assert ' rtk_fixed' in _status_tokens(node)
    clock.t = 102.6
    tick()
    assert ' rtk_no' in _status_tokens(node), '1.6 s 끊김'


def test_node_ray_drop_warn_periodic(node_mod):
    """GPS 없이 /oak_detections 만 → 대기 만료 광선이 'gps' 로 버려지고 warn_period_sec 주기 WARN 한 줄."""
    mod, clock = node_mod
    OVERRIDES.update(use_triangulation=True)
    node = mod.PerceptionFusion()
    tick = _timer(node, 10.0)
    oak_cb = node.subs['/oak_detections'][0]
    for k in range(40):
        clock.t = 100.0 + 0.1 * k
        node.subs['/imu/yaw'][0](_Msg(data=90.0))
        oak_cb(_oak_msg([(3, 10.0, 0.1, 2.0, 0.9, 0.0)]))
        tick()
    warns = [m for s, m in node.logger.lines if s == 'WARN' and '광선 버림' in m]
    assert len(warns) == 1 and 'gps ' in warns[0], warns
    n0 = int(warns[0].split('gps ')[1].split('(')[0])
    assert n0 >= 1
    clock.t = 106.0
    tick()
    assert len([m for s, m in node.logger.lines if s == 'WARN' and '광선 버림' in m]) == 1, '5 s 안 = 더 안 냄'
    clock.t = 108.0
    tick()
    warns = [m for s, m in node.logger.lines if s == 'WARN' and '광선 버림' in m]
    assert len(warns) == 2 and int(warns[1].split('gps ')[1].split('(')[0]) >= 20, '5 s 뒤 그동안 센 수'
