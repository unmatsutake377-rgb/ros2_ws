#!/usr/bin/env python3
"""위치유지 남색 부표 검출기(image_subscriber_stop) 테스트 — 계약서 v2 위치유지 사각형 변경 단위 (a).

    tools/run_tests.sh color_shape_detector

지키는 것
  · /buoy_bearings 형식 [VER 1, STRIDE 4, N, (bearing_left_deg, area_px, conf, age_s)×N] — 머리 규약은
    /avoid_detections 와 같음, pack → unpack 왕복(float32 반올림 포함), N=0 하트비트, 나쁜 행만 뺌
  · 윤곽·방위는 avoid_detect_logic 재사용(detect_blobs·bearing_left_deg — camera_info fx·cx)
  · 합성 이미지 + 가짜 camera_info: 남색 부표 4개 → 4행(좌 +), 하늘색(blue 범위 안·navy 밖)·흰색 무시,
    작은 점 버림, 빈 프레임 → 0개, max_detections
  · 노드 선언 리터럴 == stop_detect_logic.DEFAULT_PARAMS == vision.yaml image_subscriber_stop 절(키·값·타입),
    §7-2 active_wp_modes [2]·debug_view false, HSV 는 `/**` 단일 출처
  · 스텁 rclpy 로 실제 노드: 게이트 밖 침묵, camera_info 없음·K 무효·해상도 불일치 → 발행 안 함 + ERROR,
    mode 2 첫 프레임부터 하트비트, 로거 심각도 고정(§1-7), 선언이 구독보다 먼저, 발행 RELIABLE, time.time() 금지
  참고: local-stale-20261115 의 test_stop_logic.py 시나리오(N개 → N개, 빈 프레임, 다른 색 무시, 작은 노이즈, 좌우 부호).
"""

import ast
import importlib
import math
import os
import re
import sys
import types

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.normpath(os.path.join(HERE, '..'))
sys.path.insert(0, PKG)

from color_shape_detector import avoid_detect_logic as adl  # noqa: E402
from color_shape_detector import hsv_ranges  # noqa: E402
from color_shape_detector import stop_detect_logic as sdl  # noqa: E402

NODE_SRC = os.path.join(PKG, 'color_shape_detector', 'basic_image_subscriberstop.py')
VISION_YAML = os.path.join(PKG, 'config', 'vision.yaml')
NODE_NAME = 'image_subscriber_stop'

# OAK-1 PoE 640×360 camera_info(런북 10-31: fx 513.5 · cx 326.8). fy·cy 는 시험용 값(⚠️ 실물 미확인).
W, H = 640, 360
FX, FY, CX, CY = 513.5, 513.5, 326.8, 180.0
K_OAK = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
INTR = (FX, FY, CX, CY)


def _f32(data):
    np = pytest.importorskip('numpy')
    return [float(v) for v in np.asarray(data, dtype=np.float32)]


# ================================================================ 형식
def test_format_header_follows_multiarray_convention():
    assert (sdl.BB_VER, sdl.BB_STRIDE, sdl.BB_HEADER) == (1, 4, 3)
    assert sdl.BB_HEADER == adl.DET_HEADER, '머리 길이 = /avoid_detections 와 같은 규약 [VER, STRIDE, N]'


def test_pack_unpack_roundtrip_float32():
    rows = [(12.5, 400.0, 0.9, 0.07), (-20.25, 95.0, 0.2, 0.07), (0.0, 1234.0, 1.0, 0.0)]
    data, dropped = sdl.pack_bearings(rows)
    assert dropped == 0 and data[:3] == [1.0, 4.0, 3.0] and len(data) == 3 + 3 * 4
    got, err = sdl.unpack_bearings(_f32(data))
    assert err is None and len(got) == 3
    for g, r in zip(got, rows):
        for a, b in zip(g, r):
            assert abs(a - b) < 1e-3, (g, r)


def test_heartbeat_empty():
    data, dropped = sdl.pack_bearings([])
    assert data == [1.0, 4.0, 0.0] and dropped == 0, '빈 배열 [] 이 아니라 N=0 머리를 낸다'
    assert sdl.unpack_bearings(data) == ([], None)


def test_pack_drops_only_bad_rows():
    good = (5.0, 100.0, 0.5, 0.02)
    bad = [(float('nan'), 100.0, 0.5, 0.0), (1.0, float('inf'), 0.5, 0.0), (1.0, 0.0, 0.5, 0.0),
           (1.0, 100.0, 1.5, 0.0), (1.0, 100.0, 0.5, -0.01), (True, 100.0, 0.5, 0.0)]
    data, dropped = sdl.pack_bearings(bad[:3] + [good] + bad[3:])
    assert dropped == len(bad) and data[:3] == [1.0, 4.0, 1.0]
    assert data[3:] == list(good)


@pytest.mark.parametrize('data,why', [
    ([], 'short'), ([1.0, 4.0], 'short'), ([2.0, 4.0, 0.0], 'VER'), ([1.0, 6.0, 0.0], 'STRIDE'),
    ([1.0, 4.0, 1.0], 'length'), ([1.0, 4.0, -1.0], 'N'), ([1.0, 4.0, 0.5], 'N'), (['x'], 'not numeric'),
])
def test_unpack_rejects_bad_format(data, why):
    rows, err = sdl.unpack_bearings(data)
    assert rows == [] and err is not None and why in err


def test_avoid_detections_not_parsable_as_buoy_bearings():
    """다른 토픽을 잘못 물려도(STRIDE 6) 엉뚱한 칸을 방위로 읽지 않는다."""
    data, _ = adl.pack_detections([(0, 1.0, 0.1, float('nan'), 0.5, 0.0)])
    rows, err = sdl.unpack_bearings(data)
    assert rows == [] and 'STRIDE' in err


# ================================================================ 부팅 파라미터 검사
def test_check_params_defaults_pass():
    vals, errs = sdl.check_params(dict(sdl.DEFAULT_PARAMS))
    assert errs == [] and vals == sdl.DEFAULT_PARAMS
    assert sdl.check_params({}) == (sdl.DEFAULT_PARAMS, []), '빠진 키는 기본값(오류 아님)'
    assert sdl.DEFAULT_PARAMS['buoy_color'] == 'navy' and 'navy' in sdl.BUOY_COLORS


@pytest.mark.parametrize('name,value', [
    ('buoy_color', 'purple'), ('buoy_color', ''), ('buoy_color', None), ('buoy_color', 3),
    ('min_area_px', 0.0), ('min_area_px', float('nan')), ('min_area_px', True),
    ('conf_full_area_px', 0.0), ('conf_full_area_px', float('inf')),
    ('min_solidity', -0.01), ('min_solidity', 1.01), ('min_solidity', None),
    ('max_detections', 0), ('max_detections', 2.0), ('max_detections', True),
])
def test_check_params_bad_value_falls_back_with_error(name, value):
    vals, errs = sdl.check_params(dict(sdl.DEFAULT_PARAMS, **{name: value}))
    assert vals == sdl.DEFAULT_PARAMS, f'{name}={value!r} → 기본값으로'
    assert len(errs) == 1 and name in errs[0] and NODE_NAME in errs[0]


def test_check_params_color_normalized():
    vals, errs = sdl.check_params({'buoy_color': ' Navy '})
    assert errs == [] and vals['buoy_color'] == 'navy'
    assert sdl.color_cls('navy') == {'navy': sdl.CLS_BUOY}


# ================================================================ 합성 이미지 (cv2) + 가짜 camera_info
def _hsv_bgr(h, s=255, v=255):
    cv2 = pytest.importorskip('cv2')
    np = pytest.importorskip('numpy')
    px = np.uint8([[[h, s, v]]])
    return tuple(int(c) for c in cv2.cvtColor(px, cv2.COLOR_HSV2BGR)[0, 0])


NAVY_HSV = (115, 200, 100)      # hsv.navy [100,100,30]-[130,255,140] 안
SKY_HSV = (108, 144, 230)       # 맑은 하늘 — hsv.blue 안, navy 밖(V 230 > 140)
BUOY_U = (60, 250, 400, 560)    # 남색 원 중심 x(px)


def _scene(extra=True):
    """남색 원 4개(가로로) + (extra) 하늘색 큰 사각·흰 원·작은 남색 점."""
    cv2 = pytest.importorskip('cv2')
    np = pytest.importorskip('numpy')
    img = np.zeros((H, W, 3), dtype=np.uint8)
    navy = _hsv_bgr(*NAVY_HSV)
    for i, u in enumerate(BUOY_U):
        cv2.circle(img, (u, 200), 14 + 3 * i, navy, -1)       # 면적이 다르게 — 큰 순 정렬 확인
    if extra:
        cv2.rectangle(img, (0, 0), (639, 60), _hsv_bgr(*SKY_HSV), -1)
        cv2.circle(img, (320, 300), 20, (255, 255, 255), -1)
        cv2.rectangle(img, (150, 120), (153, 123), navy, -1)   # 16 px² < min_area 40
    return img


def _ranges(color='navy'):
    return {color: hsv_ranges.DEFAULT_RANGES[color]}


def _detect(img, max_det=8):
    blobs = adl.detect_blobs(img, _ranges(), 40.0, 0.7, max_det, sdl.color_cls('navy'))
    return blobs, sdl.blobs_to_rows(blobs, INTR, 0.05, 400.0)


def test_synthetic_four_navy_buoys_multi_detection():
    blobs, rows = _detect(_scene())
    assert len(rows) == 4, f'남색 원 4개 — 하늘·흰 원·작은 점은 버림: {[(b["u"], b["area"]) for b in blobs]}'
    assert all(b['color'] == 'navy' for b in blobs)
    assert [r[1] for r in rows] == sorted((r[1] for r in rows), reverse=True), '면적 큰 순'
    by_u = sorted(blobs, key=lambda b: b['u'])
    for b, u in zip(by_u, BUOY_U):
        assert abs(b['u'] - u) < 1.0
    for r, b in zip(rows, blobs):
        assert r[0] == adl.bearing_left_deg(b['u'], FX, CX), '방위 = camera_info fx·cx'
        assert r[1] == b['area'] and 0.0 < r[2] <= 1.0 and r[3] == 0.05
    brg = sorted(r[0] for r in rows)
    assert brg[0] < 0.0 < brg[-1], '화면 왼쪽 = 좌 +, 오른쪽 = −'
    # 화면 끝(u 60)의 방위 ≈ atan((326.8−60)/513.5) ≈ 27.5° — OAK 반화각 31.95° 안
    assert abs(max(brg) - math.degrees(math.atan((CX - 60) / FX))) < 0.5


def test_synthetic_roundtrip_through_pack():
    _blobs, rows = _detect(_scene())
    data, dropped = sdl.pack_bearings(rows)
    assert dropped == 0 and data[:3] == [1.0, 4.0, 4.0]
    got, err = sdl.unpack_bearings(_f32(data))
    assert err is None and len(got) == 4
    for g, r in zip(got, rows):
        assert all(abs(a - b) < 1e-2 for a, b in zip(g, r))


def test_sky_is_blue_but_not_navy():
    """hsv.blue 는 맑은 하늘을 잡지만 hsv.navy(V≤140)는 안 잡는다 — navy 를 따로 둔 이유."""
    np = pytest.importorskip('numpy')
    cv2 = pytest.importorskip('cv2')
    img = np.zeros((H, W, 3), dtype=np.uint8)
    cv2.rectangle(img, (0, 0), (639, 60), _hsv_bgr(*SKY_HSV), -1)
    assert adl.detect_blobs(img, _ranges('navy'), 40.0, 0.7, 8, {'navy': 0}) == []
    assert len(adl.detect_blobs(img, _ranges('blue'), 40.0, 0.7, 8, {'blue': 0})) == 1


def test_empty_frame_gives_heartbeat():
    np = pytest.importorskip('numpy')
    _blobs, rows = _detect(np.zeros((H, W, 3), dtype=np.uint8))
    assert rows == []
    assert sdl.pack_bearings(rows)[0] == [1.0, 4.0, 0.0]


def test_max_detections_keeps_largest():
    blobs, rows = _detect(_scene(), max_det=2)
    full, _ = _detect(_scene())
    assert len(rows) == 2 and [b['area'] for b in blobs] == [b['area'] for b in full[:2]]


# ================================================================ 선언 == DEFAULT_PARAMS == vision.yaml
def _declares():
    tree = ast.parse(open(NODE_SRC, encoding='utf-8').read(), filename=NODE_SRC)
    out = {}
    for n in ast.walk(tree):
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == 'declare_parameter':
            if n.args and isinstance(n.args[0], ast.Constant):
                out[n.args[0].value] = ast.literal_eval(n.args[1])
    return out


def _yaml_section(name):
    txt = open(VISION_YAML, encoding='utf-8').read()
    m = re.search(r'^' + re.escape(name) + r':\s*$(.*?)(?=^\S|\Z)', txt, re.M | re.S)
    assert m, f'vision.yaml 에 {name}: 절이 없다'
    out = {}
    for line in m.group(1).splitlines():
        line = re.sub(r'\s+#.*$', '', line).rstrip()
        mm = re.match(r'^\s{4}([A-Za-z_][\w.]*):\s*(\S.*)$', line)
        if not mm:
            continue
        v = mm.group(2)
        if v.startswith('['):
            val = [int(x) for x in v.strip('[]').split(',') if x.strip()]
        elif v in ('true', 'false'):
            val = v == 'true'
        elif v[0] in '"\'':
            val = v[1:-1]
        elif re.match(r'^[-+]?\d+$', v):
            val = int(v)
        else:
            val = float(v)
        out[mm.group(1)] = val
    return out


def test_declared_literals_equal_logic_defaults():
    dec = _declares()
    for k, v in sdl.DEFAULT_PARAMS.items():
        assert dec[k] == v and type(dec[k]) is type(v), f'{k}: 선언 {dec[k]!r} ≠ DEFAULT_PARAMS {v!r}'  # noqa: E721


def test_yaml_section_equals_declared_literals_and_types():
    dec = _declares()
    y = _yaml_section(NODE_NAME)
    assert set(y) <= set(dec), f'선언 안 한 yaml 키(조용히 버려짐): {set(y) - set(dec)}'
    for k, v in y.items():
        assert v == dec[k] and type(v) is type(dec[k]), f'{k}: yaml {v!r} ≠ 선언 {dec[k]!r}'  # noqa: E721
    for k in ('active_wp_modes', 'debug_view', 'camera_info_topic', 'buoy_color', 'min_area_px', 'min_solidity',
              'conf_full_area_px', 'max_detections'):
        assert k in y, f'vision.yaml {NODE_NAME} 절에 {k} 없음'


def test_safety_switch_defaults():
    """🚨 §7-2: active_wp_modes [2] · debug_view false — 선언과 yaml 둘 다."""
    dec, y = _declares(), _yaml_section(NODE_NAME)
    assert dec['active_wp_modes'] == y['active_wp_modes'] == [2]
    assert dec['debug_view'] is False and y['debug_view'] is False


def test_hsv_single_source_not_overridden_in_section():
    y = _yaml_section(NODE_NAME)
    assert not [k for k in y if k.startswith('hsv.')], y
    assert _declares().get('image_topic') == '/oak/rgb/image_raw'
    txt = open(VISION_YAML, encoding='utf-8').read()
    assert re.search(r'^\s{4}hsv\.%s:\s*\[' % y['buoy_color'], txt, re.M), 'vision.yaml `/**` 에 hsv.navy 없음'


def test_turn_mode2_unchanged():
    """계약서 v2: image_subscriber_turn active_wp_modes [2, 3] 불변."""
    assert _yaml_section('image_subscriber_turn')['active_wp_modes'] == [2, 3]


def test_no_wall_clock_and_publisher_reliable():
    src = open(NODE_SRC, encoding='utf-8').read()
    assert 'time.time(' not in src, '§1-5 time.monotonic()'
    m = re.search(r"create_publisher\(\s*Float32MultiArray,\s*'/buoy_bearings',\s*(\w+)\s*\)", src)
    assert m, '/buoy_bearings 발행자 없음'
    prof = re.search(r'^%s = QoSProfile\(([^)]*)\)' % m.group(1), src, re.M)
    assert prof and 'ReliabilityPolicy.RELIABLE' in prof.group(1), '관찰 토픽도 발행은 RELIABLE(§3 QoS)'
    img = re.search(r"create_subscription\(Image, self\.image_topic, self\.color_callback, (\w+)\)", src)
    assert img, '이미지 구독 없음'
    prof = re.search(r'^%s = QoSProfile\(([^)]*)\)' % img.group(1), src, re.M)
    assert prof and 'BEST_EFFORT' in prof.group(1) and 'depth=1' in prof.group(1), '이미지 구독 BEST_EFFORT depth 1'


# ================================================================ 스텁 rclpy 로 실제 노드
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


class _Msg:
    def __init__(self, **kw):
        self.__dict__.update(kw)


def _msg_type(name):
    return type(name, (_Msg,), {})


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


class _Param:
    def __init__(self, v):
        self.value = v


class _RosClock:
    ns = 0

    def now(self):
        return types.SimpleNamespace(nanoseconds=self.ns)


OVERRIDES = {}
ORDER = []


class _Node:
    def __init__(self, name):
        self.name = name
        self.logger = _Logger()
        self.subs = {}
        self.pubs = {}
        self.ros_clock = _RosClock()

    def declare_parameter(self, key, default):
        ORDER.append(('declare', key))
        return _Param(OVERRIDES.get(key, default))

    def get_logger(self):
        return self.logger

    def get_clock(self):
        return self.ros_clock

    def create_subscription(self, typ, topic, cb, qos):
        ORDER.append(('sub', topic))
        self.subs[topic] = cb

    def create_publisher(self, typ, topic, qos):
        ORDER.append(('pub', topic))
        self.pubs[topic] = _Pub()
        return self.pubs[topic]


class _Mono:
    t = 100.0

    def monotonic(self):
        return self.t


class _Bridge:
    def imgmsg_to_cv2(self, msg, desired_encoding=None):
        assert desired_encoding == 'bgr8'
        return msg.frame


@pytest.fixture
def node_mod(monkeypatch):
    pytest.importorskip('cv2')

    def stub(name, **attrs):
        m = types.ModuleType(name)
        for k, v in attrs.items():
            setattr(m, k, v)
        monkeypatch.setitem(sys.modules, name, m)
        return m

    rclpy = stub('rclpy', init=lambda **k: None, spin=lambda n: None, shutdown=lambda: None)
    rclpy.node = stub('rclpy.node', Node=_Node)
    rclpy.qos = stub('rclpy.qos', QoSProfile=lambda **k: dict(k),
                     ReliabilityPolicy=types.SimpleNamespace(BEST_EFFORT='BE', RELIABLE='R'),
                     HistoryPolicy=types.SimpleNamespace(KEEP_LAST='KL'))
    stub('cv_bridge', CvBridge=_Bridge)
    stub('std_msgs')
    stub('std_msgs.msg', Float32MultiArray=_msg_type('Float32MultiArray'), Int32=_msg_type('Int32'))
    stub('sensor_msgs')
    stub('sensor_msgs.msg', Image=_msg_type('Image'), CameraInfo=_msg_type('CameraInfo'))
    name = 'color_shape_detector.basic_image_subscriberstop'
    monkeypatch.delitem(sys.modules, name, raising=False)
    mod = importlib.import_module(name)
    mono = _Mono()
    monkeypatch.setattr(mod, 'time', mono)
    OVERRIDES.clear()
    ORDER.clear()
    yield mod, mono
    OVERRIDES.clear()
    sys.modules.pop(name, None)


def _img(frame, stamp_s=None):
    sec = int(stamp_s) if stamp_s else 0
    nsec = int(round((stamp_s - sec) * 1e9)) if stamp_s else 0
    return _Msg(frame=frame, width=frame.shape[1], height=frame.shape[0],
                header=_Msg(stamp=_Msg(sec=sec, nanosec=nsec)))


def _info(k=None, w=W, h=H):
    return _Msg(k=list(K_OAK if k is None else k), width=w, height=h)


def _out(node):
    return [m.data for m in node.pubs['/buoy_bearings'].msgs]


def _sev(node, sev, text=''):
    return [m for s, m in node.logger.lines if s == sev and text in m]


def test_node_declares_before_subscriptions(node_mod):
    mod, _ = node_mod
    node = mod.ImageSubscriberStop()
    assert node.name == NODE_NAME
    first_io = min(i for i, (kind, _k) in enumerate(ORDER) if kind in ('sub', 'pub'))
    last_decl = max(i for i, (kind, _k) in enumerate(ORDER) if kind == 'declare')
    assert last_decl < first_io, f'파라미터 선언이 구독·발행자 생성보다 먼저여야 한다: {ORDER}'
    assert set(node.subs) == {'/wp_mode', '/oak/rgb/image_raw', '/oak/rgb/camera_info'}
    assert set(node.pubs) == {'/buoy_bearings'}
    assert node.debug_view is False and node.mode_gate.active_modes == {2}
    assert node.buoy_color == 'navy' and set(node.color_ranges) == {'navy'}
    decl = [k for kind, k in ORDER if kind == 'declare']
    assert 'hsv.navy' in decl and 'hsv.blue' not in decl, '부표 색의 hsv.<색> 만 읽는다'
    assert decl.index('buoy_color') < decl.index('hsv.navy'), '색을 정한 뒤 HSV 를 읽는다'
    assert not _sev(node, 'ERROR')


def test_node_bad_color_falls_back_to_navy_with_error(node_mod):
    mod, _ = node_mod
    OVERRIDES['buoy_color'] = 'purple'
    node = mod.ImageSubscriberStop()
    assert node.buoy_color == 'navy' and _sev(node, 'ERROR', 'buoy_color')


def test_node_silent_outside_mode2(node_mod):
    mod, mono = node_mod
    node = mod.ImageSubscriberStop()
    node.subs['/oak/rgb/camera_info'](_info())
    node.subs['/oak/rgb/image_raw'](_img(_scene()))
    assert _out(node) == [], '/wp_mode 미수신 → 발행 안 함'
    assert _sev(node, 'WARN', '비활성')
    for m in (0, 1, 3, 5, 7, 8):
        node.subs['/wp_mode'](_Msg(data=m))
        node.subs['/oak/rgb/image_raw'](_img(_scene()))
    assert _out(node) == [], '담당 모드(2)가 아니면 발행 안 함'
    assert not _sev(node, 'ERROR')


def test_node_stale_wp_mode_silent(node_mod):
    mod, mono = node_mod
    node = mod.ImageSubscriberStop()
    node.subs['/oak/rgb/camera_info'](_info())
    node.subs['/wp_mode'](_Msg(data=2))
    mono.t += 2.5
    node.subs['/oak/rgb/image_raw'](_img(_scene()))
    assert _out(node) == [] and _sev(node, 'WARN', 'wp_mode_stale')


def test_node_heartbeat_from_first_mode2_frame_and_detections(node_mod):
    np = pytest.importorskip('numpy')
    mod, mono = node_mod
    node = mod.ImageSubscriberStop()
    node.subs['/oak/rgb/camera_info'](_info())
    node.subs['/wp_mode'](_Msg(data=2))
    assert _sev(node, 'INFO', '발행 시작')
    node.ros_clock.ns = int(50.1 * 1e9)
    node.subs['/oak/rgb/image_raw'](_img(np.zeros((H, W, 3), dtype=np.uint8), stamp_s=50.0))
    assert _out(node) == [[1.0, 4.0, 0.0]], 'mode 2 첫 프레임부터 N=0 하트비트'
    node.subs['/oak/rgb/image_raw'](_img(_scene(), stamp_s=50.0))
    rows, err = sdl.unpack_bearings(_f32(_out(node)[-1]))
    assert err is None and len(rows) == 4
    assert all(abs(r[3] - 0.1) < 1e-4 for r in rows), 'age_s = ROS 지금 − header.stamp'
    assert max(r[0] for r in rows) > 0.0 > min(r[0] for r in rows)
    assert not _sev(node, 'ERROR')


def test_node_no_camera_info_silent_with_error(node_mod):
    mod, mono = node_mod
    node = mod.ImageSubscriberStop()
    node.subs['/wp_mode'](_Msg(data=2))
    node.subs['/oak/rgb/image_raw'](_img(_scene()))
    assert _out(node) == [], 'camera_info 없으면 방위를 지어내지 않는다'
    assert _sev(node, 'ERROR', '미수신')


def test_node_invalid_k_or_size_mismatch_silent_with_error(node_mod):
    mod, mono = node_mod
    node = mod.ImageSubscriberStop()
    node.subs['/wp_mode'](_Msg(data=2))
    node.subs['/oak/rgb/camera_info'](_info(k=[0.0] * 9))
    node.subs['/oak/rgb/image_raw'](_img(_scene()))
    assert _out(node) == [] and _sev(node, 'ERROR', 'K 무효')
    node.subs['/oak/rgb/camera_info'](_info(w=1280, h=720))
    node.subs['/oak/rgb/image_raw'](_img(_scene()))
    assert _out(node) == [] and _sev(node, 'ERROR', '1280x720')
    node.subs['/oak/rgb/camera_info'](_info())
    node.subs['/oak/rgb/image_raw'](_img(_scene()))
    assert len(_out(node)) == 1, 'camera_info 가 맞으면 바로 복구'


def test_node_future_stamp_age_zero_with_warn(node_mod):
    mod, mono = node_mod
    node = mod.ImageSubscriberStop()
    node.subs['/wp_mode'](_Msg(data=2))
    node.subs['/oak/rgb/camera_info'](_info())
    node.ros_clock.ns = int(50.0 * 1e9)
    node.subs['/oak/rgb/image_raw'](_img(_scene(), stamp_s=50.4))
    rows, err = sdl.unpack_bearings(_out(node)[-1])
    assert err is None and rows and all(r[3] == 0.0 for r in rows)
    assert _sev(node, 'WARN', '앞섬')
