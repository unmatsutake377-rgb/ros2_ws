#!/usr/bin/env python3
"""[P] 카메라 장애물 병합 테스트 — camera_merge.py(순수) + ship_direction 래퍼 (계약서_카메라장애물_20261111 U1).

실행: tools/run_tests.sh ship_direction

규약: /camera_obstacle_state = [angle_min_deg, angle_inc_deg, r0, ...] 좌 양수(인덱스 0 = 우현), 멀면 inf,
정보 없음 빈 배열. (C) 에서 geofence 병합 **바로 뒤** _merge_geofence 와 같은 매핑으로 min 병합.
끔(기본) = 비트 동일. 감속 신호·SensorWatch 엔 안 들어간다. 침묵은 ERROR 진입 1회 + 5 s 주기 + 복구 INFO.

래퍼는 test_dwa_node.py 와 같은 ROS 스텁 + 실제 __init__(Node 메서드만 잠깐 끼움)으로 만든 노드의
control_cb 틱으로 본다.
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

from ship_direction.camera_merge import (  # noqa: E402
    CAMERA_ALLOWED_MODES, ST_EMPTY, ST_NONE, ST_OK, ST_STALE, camera_active, camera_silent,
    camera_status, check_enabled, check_modes, check_stale_sec, merge_camera, parse_state)

# ---- ROS 모듈 최소 스텁 (import 통과용, 로직엔 안 쓰임) ----


def _stub(name, **attrs):
    m = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(m, k, v)
    sys.modules[name] = m
    return m


class _Node:                       # class ShipDirection(Node) 상속용
    pass


def _ns(*a, **k):
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
    _stub('std_msgs.msg', Int32=_ns, Float32=_ns, Float32MultiArray=_ns)
    _stub('sensor_msgs', msg=None)
    _stub('sensor_msgs.msg', LaserScan=_ns)

import ship_direction.ship_direction as SD  # noqa: E402
from ship_direction.ship_direction import (  # noqa: E402
    CAMERA_SILENT_LOG_PERIOD_SEC, CANDIDATE_INVALID, LIDAR_FORWARD_DEG, ShipDirection)

INF = math.inf
SD_FILE = os.path.abspath(SD.__file__)


class _Logger:
    """심각도와 **호출한 소스 줄**을 함께 적는다 — rclpy 는 줄마다 심각도를 캐싱한다(§1-7)."""

    def __init__(self):
        self.lines = []

    def _add(self, sev, msg, kw):
        f = sys._getframe(2)
        self.lines.append((sev, msg, kw, (os.path.abspath(f.f_code.co_filename), f.f_lineno)))

    def warn(self, msg, **kw):
        self._add('WARN', msg, kw)

    def info(self, msg, **kw):
        self._add('INFO', msg, kw)

    def error(self, msg, **kw):
        self._add('ERROR', msg, kw)

    def of(self, sev, word=''):
        return [m for s, m, kw, src in self.lines if s == sev and word in m]

    def texts(self):
        return [(s, m) for s, m, kw, src in self.lines]


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


def _construct(overrides):
    """ShipDirection.__init__ 을 실제로 돌린다 — Node 메서드만 잠깐 끼운다 (test_dwa_node 와 같음)."""
    base = ShipDirection.__bases__[0]
    log = _Logger()
    subs = []
    names = ('__init__', 'declare_parameter', 'create_subscription', 'create_publisher',
             'create_timer', 'get_logger')
    saved = {n: base.__dict__[n] for n in names if n in base.__dict__}
    base.__init__ = lambda self, name: None
    base.declare_parameter = lambda self, n, d: types.SimpleNamespace(value=overrides.get(n, d))
    base.create_subscription = lambda self, typ, topic, cb, qos: subs.append((topic, cb, qos))
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
    node.subs = subs
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


def scan(obstacles):
    """0~179.5° (0.5° 간격) 스캔. obstacles = [(lo_deg, hi_deg, r)] 구간 안 셀만 r, 나머지 inf."""
    n = 360
    ranges = [INF] * n
    for lo, hi, r in obstacles:
        for i in range(n):
            if lo <= i * INC_DEG <= hi:
                ranges[i] = r
    return types.SimpleNamespace(angle_min=0.0, angle_increment=math.radians(INC_DEG),
                                 ranges=ranges)


OPEN_SC = scan([])
BUOY_SC = scan([(40.0, 44.0, 2.5)])          # 우현 앞 부표 하나 — 카메라 없이도 (C) 가 돈다


def cam_data(cells, a0=-31.5, inc=0.5, n=127):
    """카메라 가짜 LiDAR msg.data. cells = {상대방위(deg, 좌 +): r}."""
    rs = [INF] * n
    for rel, r in cells.items():
        rs[int(round((rel - a0) / inc))] = r
    return [a0, inc] + rs


CAM_FRONT = cam_data({rel / 2.0: 2.0 for rel in range(-10, 11)})   # 정면 ±5° 2.0 m 원통

ON = {'camera_obstacle_enabled': True}


def _node(over=None, wp_mode=8):
    node, log = _construct(dict(over or {}))
    if wp_mode is not None:
        node.wp_mode_cb(_ns(data=wp_mode))
    return node, log


def _tick(node, clk, sc=OPEN_SC, cam=None, yaw=0.0, dt=0.1):
    """dt 진행 → (cam 이 있으면 카메라 수신) → control_cb 1회 → 발행한 desired."""
    clk.t += dt
    if cam is not None:
        node.camera_obstacle_cb(_ns(data=cam))
    node.latest_scan = sc
    node.yaw_error = yaw
    node.candidate_angle = CANDIDATE_INVALID
    node.candidate_t = clk.t
    ShipDirection.control_cb(node)
    return node.pub_desired_angle.msgs[-1].data


def _state(node):
    return (node._side_prev, node.reverse_state, node.reverse_start_time,
            node.reverse_cooldown_start_time, node.prev_final_angle, node._side_escape_active)


LIDAR_ANGLES = [i * 1.0 for i in range(161)]   # LiDAR 프레임 0..160 (80 = 정면)


# ======================================================================
# 순수 함수 — 인덱스 매핑·min·건너뜀 (camera_merge.merge_camera)
# ======================================================================
def _cam(cells, a0=-31.5, inc=0.5, n=127):
    d = cam_data(cells, a0, inc, n)
    return parse_state(d)


def test_index_map_left20_to_lidar100():
    """카메라 rel +20°(좌) → LiDAR 100°. 좌 양수 그대로(부호 변환 없음)."""
    cam = _cam({20.0: 2.0})
    out, st, hits, near = merge_camera([INF] * 161, LIDAR_ANGLES, cam, 0.0, 0.1, 0.5)
    assert st == ST_OK and hits == 1 and near == 2.0
    assert out[100] == 2.0, out[100]
    assert out[60] == INF, "우현 60°에 '없는 원통' — 소비자 쪽에서 부호를 뒤집었다"


def test_index0_is_starboard():
    """카메라 배열 인덱스 0(rel = a0 = −31.5 → 반올림 셀) → LiDAR 80−31.5 쪽(우현). 끝 인덱스 → 좌현."""
    cam = (-30.0, 1.0, [3.0] + [INF] * 59 + [4.0])           # 인덱스 0 = −30, 끝 60 = +30
    out, st, _, _ = merge_camera([INF] * 161, LIDAR_ANGLES, cam, 0.0, 0.1, 0.5)
    assert out[50] == 3.0 and out[110] == 4.0, (out[50], out[110])
    assert sum(1 for v in out if math.isfinite(v)) == 2


def test_same_mapping_as_geofence_merge():
    """같은 배열을 geofence 로 넣은 _merge_geofence 결과와 칸마다 같다(같은 매핑)."""
    cam = _cam({-25.0: 1.5, 0.0: 2.0, 12.5: 3.5, 31.0: 4.0})
    out_c, _, _, _ = merge_camera([INF] * 161, LIDAR_ANGLES, cam, 0.0, 0.1, 0.5)
    fs = types.SimpleNamespace(lock=_NoLock(), geofence=cam, last_geofence_t=SD.time.monotonic(),
                               geofence_stale_sec=2.0, get_logger=lambda: _Logger())
    out_g = ShipDirection._merge_geofence(fs, [INF] * 161, LIDAR_ANGLES)
    assert out_c == out_g


class _NoLock:
    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False


def test_real_scan_closer_is_kept():
    cam = _cam({0.0: 2.0})
    scan_ = [INF] * 161
    scan_[80] = 1.0
    out, _, hits, _ = merge_camera(scan_, LIDAR_ANGLES, cam, 0.0, 0.1, 0.5)
    assert out[80] == 1.0 and hits == 0
    scan_[80] = 5.0
    out, _, hits, _ = merge_camera(scan_, LIDAR_ANGLES, cam, 0.0, 0.1, 0.5)
    assert out[80] == 2.0 and hits == 1
    scan_[80] = math.nan                                     # 실제가 nan 이면 카메라 값
    out, _, _, _ = merge_camera(scan_, LIDAR_ANGLES, cam, 0.0, 0.1, 0.5)
    assert out[80] == 2.0


def test_nonfinite_and_nonpositive_cells_skipped():
    cam = (-1.0, 1.0, [math.nan, -math.inf, 0.0])            # rel −1, 0, +1
    scan_ = [INF] * 161
    scan_[81] = 4.0
    out, st, hits, _ = merge_camera(scan_, LIDAR_ANGLES, cam, 0.0, 0.1, 0.5)
    assert st == ST_OK and hits == 0
    assert out == scan_


def test_stale_empty_none_leave_unchanged():
    """못 받음·stale(초과)·빈 배열 → 그 객체 그대로. age == stale 은 신선."""
    scan_ = [INF] * 161
    cam = _cam({0.0: 2.0})
    for c, t, now, want in ((cam, None, 1.0, ST_NONE), (cam, 0.0, 0.51, ST_STALE),
                            (None, 0.0, 0.1, ST_EMPTY), ((0.0, 1.0, []), 0.0, 0.1, ST_EMPTY)):
        out, st, hits, _ = merge_camera(scan_, LIDAR_ANGLES, c, t, now, 0.5)
        assert out is scan_ and st == want and hits == 0, (st, want)
    _, st, _, _ = merge_camera(scan_, LIDAR_ANGLES, cam, 0.0, 0.5, 0.5)
    assert st == ST_OK
    assert camera_status(cam, 0.0, 0.5, 0.5) == ST_OK


def test_out_of_range_k_skipped():
    """카메라 배열이 덮지 않는 LiDAR 각(±31.5° 밖)은 안 건드린다."""
    cam = (-31.5, 0.5, [1.0] * 127)
    out, _, hits, _ = merge_camera([INF] * 161, LIDAR_ANGLES, cam, 0.0, 0.1, 0.5)
    assert all(out[i] == INF for i in range(0, 48)) and all(out[i] == INF for i in range(112, 161))
    assert all(out[i] == 1.0 for i in range(49, 112)), out[45:115]
    assert hits == sum(1 for v in out if v == 1.0)


def test_parse_state():
    assert parse_state([]) is None
    assert parse_state([-30.0, 1.0]) is None                 # 거리 칸 없음
    assert parse_state([-30.0, 0.0, 1.0]) is None            # inc 0
    assert parse_state([-30.0, -1.0, 1.0]) is None
    assert parse_state([math.nan, 1.0, 1.0]) is None
    assert parse_state([-30.0, math.inf, 1.0]) is None
    assert parse_state(None) is None
    assert parse_state([-30.0, 1.0, 2.0, INF]) == (-30.0, 1.0, [2.0, INF])


def test_camera_silent_grace_from_entry():
    """기준 = max(마지막 수신, 진입). 진입 전 묵은 수신은 진입부터 stale_sec 기다린다."""
    assert camera_silent(None, None, 100.0, 0.5) is False
    assert camera_silent(None, 10.0, 10.5, 0.5) is False
    assert camera_silent(None, 10.0, 10.51, 0.5) is True
    assert camera_silent(1.0, 10.0, 10.4, 0.5) is False        # 수신은 묵었지만 진입 직후
    assert camera_silent(1.0, 10.0, 10.6, 0.5) is True
    assert camera_silent(10.3, 10.0, 10.7, 0.5) is False


def test_camera_active():
    assert camera_active(True, 8, [8])
    assert not camera_active(False, 8, [8])
    assert not camera_active(1, 8, [8])                       # True 정확히만
    assert not camera_active(True, 7, [8])
    assert not camera_active(True, None, [8])
    assert not camera_active(True, -1, [8])


def test_param_checks():
    assert check_enabled(True) == (True, None) and check_enabled(False) == (False, None)
    for bad in (1, 'true', None, 0.0):
        v, err = check_enabled(bad)
        assert v is False and err, bad
    assert check_modes([8]) == ([8], [])
    assert check_modes([0, 8, 7, 8, 5, True]) == ([8, 5], [0, 7, True])
    assert check_modes([]) == ([], [])
    assert check_modes(8) == ([], [8])
    assert set(CAMERA_ALLOWED_MODES) == {5, 8}
    assert check_stale_sec(0.5) == (0.5, None) and check_stale_sec(1) == (1.0, None)
    for bad in (0.0, -1.0, math.nan, math.inf, None, 'x', True):
        v, err = check_stale_sec(bad)
        assert v == 0.5 and err, bad


# ======================================================================
# 노드 — 파라미터·구독·기본값
# ======================================================================
def _yaml(name):
    with open(os.path.join(PKG, 'config', name), encoding='utf-8') as f:
        return yaml.safe_load(f)['/ship_direction']['ros__parameters']


def _src():
    with open(os.path.join(PKG, 'ship_direction', 'ship_direction.py'), encoding='utf-8') as f:
        return f.read()


WANT = {'camera_obstacle_enabled': False, 'camera_obstacle_modes': [8],
        'camera_obstacle_stale_sec': 0.5}


def test_defaults_equal_yaml():
    """코드 리터럴 기본값 == 공통 yaml == 토너먼트 yaml(처음 값 false), 타입까지."""
    for name in ('ship_direction.yaml', 'ship_direction_tournament.yaml'):
        p = _yaml(name)
        for k, v in WANT.items():
            assert p[k] == v and type(p[k]) is type(v), (name, k, p.get(k))
        assert all(type(m) is int for m in p['camera_obstacle_modes'])
    src = _src()
    for k, lit in (('camera_obstacle_enabled', 'False'), ('camera_obstacle_modes', r'\[8\]'),
                   ('camera_obstacle_stale_sec', r'0\.5')):
        assert re.search(rf"declare_parameter\('{k}', {lit}\)", src), k
    node, log = _construct({})
    assert (node.camera_obstacle_enabled, node.camera_obstacle_modes,
            node.camera_obstacle_stale_sec) == (False, [8], 0.5)
    assert not [m for s, m in log.texts() if '카메라' in m or 'camera' in m], log.texts()


def test_params_declared_before_subscriptions():
    """파라미터 선언이 구독 생성보다 먼저(소스 순서)."""
    src = _src()
    first_sub = src.index("self.create_subscription(")
    for k in WANT:
        assert src.index(f"declare_parameter('{k}'") < first_sub, k


def test_subscription_qos_even_when_off():
    """끔이어도 구독은 만든다. BEST_EFFORT + KEEP_LAST + depth 1, 콜백은 camera_obstacle_cb."""
    node, _ = _construct({})
    subs = [s for s in node.subs if s[0] == '/camera_obstacle_state']
    assert len(subs) == 1, node.subs
    topic, cb, qos = subs[0]
    assert cb == node.camera_obstacle_cb
    pol = SD.ReliabilityPolicy
    assert qos.reliability == pol.BEST_EFFORT and qos.history == SD.HistoryPolicy.KEEP_LAST
    assert qos.depth == 1


def test_callback_stores_only():
    """콜백은 파싱·저장·monotonic 시각만. 빈 배열 → None(시각은 기록)."""
    def run(clk):
        node, log = _node(ON)
        n_pub = len(node.pub_desired_angle.msgs)
        node.camera_obstacle_cb(_ns(data=CAM_FRONT))
        assert node.camera_obstacle[0] == -31.5 and node.camera_obstacle_t == clk.t
        clk.t += 1.0
        node.camera_obstacle_cb(_ns(data=[]))
        assert node.camera_obstacle is None and node.camera_obstacle_t == clk.t
        assert len(node.pub_desired_angle.msgs) == n_pub
    with_env(run)


def test_guards():
    node, log = _construct({'camera_obstacle_enabled': True, 'camera_obstacle_modes': [0, 8, 7]})
    assert node.camera_obstacle_enabled and node.camera_obstacle_modes == [8]
    assert len(log.of('WARN', 'camera_obstacle_modes')) == 1
    assert len(log.of('INFO', '카메라 장애물 병합 켬')) == 1
    node, log = _construct({'camera_obstacle_enabled': True, 'camera_obstacle_modes': [7, 3]})
    assert node.camera_obstacle_enabled is False and log.of('WARN', '하나도 없음')
    assert not log.of('INFO', '카메라 장애물 병합 켬')
    node, log = _construct({'camera_obstacle_enabled': 1})
    assert node.camera_obstacle_enabled is False and log.of('WARN', 'camera_obstacle_enabled')
    for bad in (0.0, -0.5, math.nan, math.inf):
        node, log = _construct({'camera_obstacle_stale_sec': bad})
        assert node.camera_obstacle_stale_sec == 0.5
        assert len(log.of('WARN', 'camera_obstacle_stale_sec')) == 1


# ======================================================================
# 노드 — (C) 동작
# ======================================================================
SEQ = [OPEN_SC] * 3 + [BUOY_SC] * 10 + [OPEN_SC] * 5


def _run(node, clk, cam=None, seq=SEQ):
    outs, states, obsts = [], [], []
    for sc in seq:
        outs.append(_tick(node, clk, sc=sc, cam=cam))
        states.append(_state(node))
        obsts.append(repr(node.pub_obstacle_distance.msgs[-1].data))
    return outs, states, obsts


def test_off_bit_identical_with_camera_data():
    """끔(기본): 정면 2 m 원통을 매 틱 받아도 결과·상태·감속 신호·로그가 카메라 없음과 같다.
    켜고 모드 밖(7)도 같다(시작 INFO 한 줄만 다름)."""
    def run(clk):
        base, log_b = _node(wp_mode=8)
        clk.t = 1000.0
        r_base = _run(base, clk)
        off, log_off = _node(wp_mode=8)
        clk.t = 1000.0
        r_off = _run(off, clk, cam=CAM_FRONT)
        assert r_off == r_base
        assert log_off.texts() == log_b.texts()

        base7, log_b7 = _node(wp_mode=7)
        clk.t = 1000.0
        r_b7 = _run(base7, clk)
        on7, log_on7 = _node(ON, wp_mode=7)
        calls = []
        on7._merge_camera = lambda d, a, now: calls.append(1) or d
        clk.t = 1000.0
        r_on7 = _run(on7, clk, cam=CAM_FRONT)
        assert r_on7 == r_b7 and calls == [1] * len(SEQ)     # 불렸지만 모드 밖 → 그대로
        start = [x for x in log_on7.texts() if '병합 켬' in x[1]]
        assert len(start) == 1 and [x for x in log_on7.texts() if x not in start] == log_b7.texts()
    with_env(run)


def test_off_never_calls_merge():
    """끔이면 _merge_camera 를 부르지도 않는다."""
    def run(clk):
        node, _ = _node(wp_mode=8)
        node._merge_camera = lambda *a: (_ for _ in ()).throw(AssertionError('끔인데 호출'))
        _run(node, clk, cam=CAM_FRONT)
    with_env(run)


def test_mode_outside_unchanged_real_method():
    """켬 + 모드 7: 실제 _merge_camera 가 배열 객체 그대로, 로그 없음."""
    def run(clk):
        node, log = _node(ON, wp_mode=7)
        node._side_wp_mode = 7
        node.camera_obstacle_cb(_ns(data=CAM_FRONT))
        arr = [INF] * 321
        assert node._merge_camera(arr, [i * 0.5 for i in range(321)], clk.t) is arr
        assert not [x for x in log.texts() if '📷' in x[1]]
    with_env(run)


def test_on_mode8_avoids_camera_obstacle():
    """켬 + 모드 8 + 열린 스캔 + 정면 2 m 카메라 원통 → 정면(80)이 아닌 쪽으로 튼다. 끄면 80."""
    def run(clk):
        off, _ = _node(wp_mode=8)
        on, log = _node(ON, wp_mode=8)
        a_off = [_tick(off, clk, cam=CAM_FRONT) for _ in range(5)]
        a_on = [_tick(on, clk, cam=CAM_FRONT) for _ in range(5)]
        assert all(a == 80.0 for a in a_off), a_off
        assert abs(a_on[-1] - 80.0) > 10.0, a_on
        assert log.of('INFO', '카메라 장애물이 스캔에 병합됨')
        assert not log.of('ERROR')
    with_env(run)


def test_on_stale_camera_same_as_off():
    """켬 + 모드 8, 카메라가 0.5 s 넘게 묵음 → 병합 안 함(끔과 같은 각)."""
    def run(clk):
        off, _ = _node(wp_mode=8)
        on, _ = _node(ON, wp_mode=8)
        on.camera_obstacle_cb(_ns(data=CAM_FRONT))
        clk.t += 0.6
        a_on = [_tick(on, clk, dt=0.0)]
        a_off = [_tick(off, clk, dt=0.0)]
        assert a_on == a_off == [80.0]
    with_env(run)


def test_geofence_then_camera_both_applied():
    """병합 순서: geofence → camera(geofence 결과를 입력으로). 둘 다 DWA choose 배열에 들어간다(스파이)."""
    def run(clk):
        node, _ = _node(dict(ON, dwa_enabled=True, dwa_modes=[8]), wp_mode=8)
        node.geofence = (-80.0, 1.0, [INF] * 161)
        node.geofence[2][30] = 3.0                         # rel −50 → LiDAR 30° (우현)
        node.last_geofence_t = clk.t + 0.1
        order = []
        real_g, real_c = node._merge_geofence, node._merge_camera

        def spy_g(d, a):
            out = real_g(d, a)
            order.append(('g', out))
            return out

        def spy_c(d, a, now):
            assert order and d is order[-1][1], "카메라 병합 입력이 geofence 결과가 아니다"
            out = real_c(d, a, now)
            order.append(('c', out))
            return out
        node._merge_geofence, node._merge_camera = spy_g, spy_c
        seen = []
        real_choose = node._dwa.choose
        node._dwa.choose = lambda aa, dd, y, **k: seen.append(list(dd)) or real_choose(aa, dd, y, **k)
        _tick(node, clk, cam=cam_data({30.0: 2.0}))        # 카메라 rel +30 → LiDAR 110° (좌현)
        assert [o[0] for o in order] == ['g', 'c']
        dd = seen[-1]
        assert dd[60] == 3.0, dd[60]                        # LiDAR 30° = 인덱스 60 (0.5° 간격)
        assert dd[220] == 2.0, dd[220]                      # LiDAR 110° = 인덱스 220
    with_env(run)


def test_closest_obstacle_unchanged():
    """감속 신호(/obstacle_distance_array)는 원 스캔만 — 카메라 원통 0.8 m 가 있어도 [inf, nan]."""
    def run(clk):
        node, _ = _node(ON, wp_mode=8)
        for _ in range(3):
            _tick(node, clk, cam=cam_data({0.0: 0.8}))
        d = node.pub_obstacle_distance.msgs[-1].data
        assert d[0] == INF and math.isnan(d[1]), d
    with_env(run)


def test_camera_silence_keeps_failsafe_zero():
    """켬 + 모드 8, 카메라 한 번도 안 옴(10 s) → /failsafe_level 은 계속 0. 감시 키 불변."""
    def run(clk):
        node, log = _node(ON, wp_mode=8)
        assert set(node.watch.last_seen) == {'scan', 'yaw_error'}     # 카메라는 감시 대상 아님
        for _ in range(100):
            clk.t += 0.1
            node.scan_callback(OPEN_SC)
            node.yaw_error_callback(_ns(data=0.0))
            node.candidate_angle, node.candidate_t = CANDIDATE_INVALID, clk.t
            ShipDirection.watchdog_cb(node)
            ShipDirection.control_cb(node)
        assert {m.data for m in node.pub_failsafe.msgs} == {0}
        assert log.of('ERROR', '한 번도 못 받음')
    with_env(run)


def test_silence_log_enter_repeat_recover_and_lines_split():
    """침묵 ERROR 진입 1회 + 5 s 주기 ERROR + 복구 INFO 1회. 진입 직후 stale_sec 안은 ERROR 없음.
    호출한 소스 줄마다 심각도는 하나(§1-7 — rclpy 가 줄별로 캐싱)."""
    def run(clk):
        node, log = _node(ON, wp_mode=8)
        for _ in range(5):                                   # 0.5 s: 진입 유예 안
            _tick(node, clk)
        assert not log.of('ERROR'), log.texts()
        _tick(node, clk)                                     # 0.6 s → 진입 ERROR
        assert len(log.of('ERROR', '한 번도 못 받음')) == 1
        for _ in range(int(CAMERA_SILENT_LOG_PERIOD_SEC / 0.1) + 1):
            _tick(node, clk)
        assert len(log.of('ERROR', '침묵 지속')) == 1
        _tick(node, clk, cam=CAM_FRONT)                      # 복구
        _tick(node, clk, cam=CAM_FRONT)
        assert len(log.of('INFO', '수신 재개')) == 1
        for _ in range(7):                                   # 다시 침묵(0.7 s) → 진입 ERROR(갱신 없음)
            _tick(node, clk)
        assert len(log.of('ERROR', '갱신 없음')) == 1
        _tick(node, clk, cam=[])                             # 빈 배열 → 복구 INFO + WARN
        _tick(node, clk, cam=[])
        assert len(log.of('INFO', '수신 재개')) == 2
        assert len(log.of('INFO', '→ 카메라 병합')) == 1             # 첫 복구(데이터 있음)만 '병합'
        assert len(log.of('INFO', '수신 재개(빈 배열 — 병합 안 함)')) == 1   # 빈 배열 복구는 사실대로
        assert log.of('WARN', '빈 배열')
        sev_by_src = {}
        for sev, m, kw, src in log.lines:
            if src[0] == SD_FILE:
                sev_by_src.setdefault(src, set()).add(sev)
        bad = {k: v for k, v in sev_by_src.items() if len(v) > 1}
        assert not bad, bad
    with_env(run)


def test_mode_reentry_no_false_error_or_recover():
    """mode 8 → 7 → 8: 재진입 순간 묵은 수신이어도 stale_sec 안은 ERROR 없고, 거짓 '복구' INFO 도 없다."""
    def run(clk):
        node, log = _node(ON, wp_mode=8)
        for _ in range(10):                                  # 침묵 ERROR 상태로 만든다
            _tick(node, clk)
        assert len(log.of('ERROR')) == 1
        node.wp_mode_cb(_ns(data=7))
        for _ in range(5):
            _tick(node, clk)
        node.camera_obstacle_cb(_ns(data=CAM_FRONT))         # 모드 7 동안 마지막 수신(곧 묵음)
        for _ in range(10):
            _tick(node, clk)
        node.wp_mode_cb(_ns(data=8))
        _tick(node, clk, cam=CAM_FRONT)                      # 진입 첫 틱에 수신
        _tick(node, clk, cam=CAM_FRONT)
        assert len(log.of('ERROR')) == 1, log.texts()
        assert not log.of('INFO', '수신 재개'), log.texts()
    with_env(run)


def test_mode_reentry_grace_then_error_without_camera():
    """mode 8(수신 정상) → 7 → 8 재진입, 카메라 데이터 없음: 재진입 뒤 0.5 s 이내 틱들은 ERROR 0,
    그 뒤 첫 틱에 ERROR 1(노드 수준 진입 유예 — _camera_track_active 가 재진입 시각을 기준으로 삼는다)."""
    def run(clk):
        node, log = _node(ON, wp_mode=8)
        for _ in range(5):
            _tick(node, clk, cam=CAM_FRONT)                  # 정상 수신 → ERROR 없음
        node.wp_mode_cb(_ns(data=7))
        for _ in range(10):                                  # 모드 7 에서 1 s(카메라 노드는 발행 안 함)
            _tick(node, clk)
        assert not log.of('ERROR'), log.texts()
        node.wp_mode_cb(_ns(data=8))
        t_enter = clk.t + 0.1
        for dt in (0.1, 0.1, 0.1, 0.1, 0.05):                # 재진입 첫 틱 ~ +0.35 s: 마지막 수신은 1 s 넘게 묵었다
            _tick(node, clk, dt=dt)
        assert clk.t - t_enter <= node.camera_obstacle_stale_sec
        assert not log.of('ERROR'), log.texts()
        _tick(node, clk, dt=0.2)                             # 진입 +0.55 s > 0.5 s → 진입 ERROR 1회
        assert clk.t - t_enter > node.camera_obstacle_stale_sec
        assert len(log.of('ERROR')) == 1, log.texts()
        assert len(log.of('ERROR', '갱신 없음')) == 1
    with_env(run)


def test_lidar_forward_constant():
    assert LIDAR_FORWARD_DEG == 80.0
