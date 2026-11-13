#!/usr/bin/env python3
"""도리도리 회피 검출기(image_subscriber_avoid) 테스트 — 계약서_도리도리회피_v2_20261114 U3.

    tools/run_tests.sh color_shape_detector

지키는 것
  · 형식 사본(VER·STRIDE·머리·cls) == ship_avoid avoid_mode_logic (파일 경로로 읽기만 — 다른 패키지 수정 없음)
  · 검출기 pack → ship_avoid parse_detections 왕복(float32 반올림 포함, NaN range, N=0 하트비트)
  · camera_info K 검사, 방위 좌 +, v_norm 아래 +, age_s ≥ 0 clamp(W3), conf 0~1
  · 합성 이미지: 빨강(두 H 구간)·노랑 다중 윤곽, 작은 덩어리·들쭉날쭉한 조각 버림, 빈 프레임 → 0개, 최대 개수
  · 노드 선언 리터럴 == vision.yaml image_subscriber_avoid 절(키·값·타입), HSV 는 `/**` 단일 출처
  · 스텁 rclpy 로 실제 노드: 게이트 밖 침묵, camera_info 없음·K 무효·해상도 불일치 → 발행 안 함 + ERROR,
    mode 5 첫 프레임부터 하트비트, 미래 stamp → age 0 + WARN, 로거 심각도 고정(§1-7), 선언이 구독보다 먼저
  · /avoid_detections 발행 QoS = RELIABLE depth 10 (ship_avoid 구독과 호환), time.time() 금지
"""

import ast
import importlib
import importlib.util
import math
import os
import re
import sys
import types

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.normpath(os.path.join(HERE, '..'))
SRC = os.path.dirname(PKG)
sys.path.insert(0, PKG)

from color_shape_detector import avoid_detect_logic as adl  # noqa: E402
from color_shape_detector import hsv_ranges  # noqa: E402
from color_shape_detector.vision_geom import angle_from_pixel  # noqa: E402

NODE_SRC = os.path.join(PKG, 'color_shape_detector', 'basic_image_subscriberavoid.py')
VISION_YAML = os.path.join(PKG, 'config', 'vision.yaml')
SA_LOGIC = os.path.join(SRC, 'ship_avoid', 'ship_avoid', 'avoid_mode_logic.py')
SA_NODE = os.path.join(SRC, 'ship_avoid', 'ship_avoid', 'ship_avoid.py')
NODE_NAME = 'image_subscriber_avoid'

# OAK-1 PoE 640×360 camera_info(런북 10-31: fx 513.5 · cx 326.8). fy·cy 는 시험용 값(⚠️ 실물 미확인).
W, H = 640, 360
FX, FY, CX, CY = 513.5, 513.5, 326.8, 180.0
K_OAK = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
INTR = (FX, FY, CX, CY)


def _ship_avoid_logic():
    """ship_avoid avoid_mode_logic 을 파일 경로로 읽는다(math 만 import — 다른 패키지를 고치지 않는다)."""
    spec = importlib.util.spec_from_file_location('sa_avoid_mode_logic_for_detector_test', SA_LOGIC)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _f32(data):
    """Float32MultiArray 를 거친 값(float32 반올림)."""
    np = pytest.importorskip('numpy')
    return [float(v) for v in np.asarray(data, dtype=np.float32)]


# ================================================================ 형식 사본 == ship_avoid
def test_format_constants_match_ship_avoid():
    ml = _ship_avoid_logic()
    assert (adl.DET_VER, adl.DET_STRIDE, adl.DET_HEADER) == (ml.DET_VER, ml.DET_STRIDE, ml.DET_HEADER)
    # cls 번호 — ship_avoid core.add_detections: 0 = 빨강(낮은 장애물), 2 = 노랑, 그 외 무시
    assert adl.CLS_RED == 0 and adl.CLS_YELLOW == 2
    assert adl.COLOR_CLS == {'red': 0, 'yellow': 2}
    assert set(adl.COLOR_CLS) <= set(hsv_ranges.VALID_COLORS)


def test_roundtrip_through_ship_avoid_parser():
    ml = _ship_avoid_logic()
    dets = [(0, 12.5, 0.21, float('nan'), 0.8, 0.07),
            (2, -20.25, -0.05, float('nan'), 0.35, 0.07),
            (0, 0.0, 0.4, 3.2, 1.0, 0.0)]
    data, dropped = adl.pack_detections(dets)
    assert dropped == 0
    assert data[:3] == [1.0, 6.0, 3.0] and len(data) == 3 + 3 * 6
    got, n_old, err = ml.parse_detections(_f32(data), 0.5)
    assert err is None and n_old == 0 and len(got) == 3
    for g, d in zip(got, dets):
        assert g[0] == d[0] and isinstance(g[0], int)
        for a, b in zip(g[1:], d[1:]):
            if math.isnan(b):
                assert math.isnan(a)
            else:
                assert abs(a - b) < 1e-5, (g, d)


def test_heartbeat_empty_is_alive_for_ship_avoid():
    ml = _ship_avoid_logic()
    data, dropped = adl.pack_detections([])
    assert data == [1.0, 6.0, 0.0] and dropped == 0, '빈 배열 [] 이 아니라 N=0 머리를 낸다'
    got, n_old, err = ml.parse_detections(_f32(data), 0.5)
    assert got == [] and n_old == 0 and err is None, 'N=0 = 살아 있음'


def test_pack_drops_only_bad_rows_so_message_survives():
    """비유한·음수 age·모르는 cls 가 섞여도 나머지는 ship_avoid 가 받는다(메시지 통째 버림 방지)."""
    ml = _ship_avoid_logic()
    good = (2, 5.0, 0.1, float('inf'), 0.5, 0.02)
    bad = [(1, 0.0, 0.0, float('nan'), 0.5, 0.0),            # 모르는 cls
           (0, float('nan'), 0.1, float('nan'), 0.5, 0.0),   # 방위 비유한
           (0, 1.0, 0.1, float('nan'), 0.5, -0.01)]          # age < 0
    data, dropped = adl.pack_detections([bad[0], good, bad[1], bad[2]])
    assert dropped == 3 and data[2] == 1.0
    assert math.isnan(data[3 + 3]), 'range inf → NaN(모름)'
    got, _n_old, err = ml.parse_detections(_f32(data), 0.5)
    assert err is None and len(got) == 1 and got[0][0] == 2


# ================================================================ camera_info · 기하
def test_intrinsics_valid_and_invalid():
    assert adl.intrinsics_from_camera_info(K_OAK, W, H) == INTR
    assert adl.intrinsics_from_camera_info([0.0] * 9, W, H) is None, 'K 전부 0(캘리브 없음) → 못 씀'
    assert adl.intrinsics_from_camera_info([-1.0, 0, CX, 0, FY, CY, 0, 0, 1], W, H) is None
    assert adl.intrinsics_from_camera_info([FX, 0, float('nan'), 0, FY, CY, 0, 0, 1], W, H) is None
    assert adl.intrinsics_from_camera_info(K_OAK[:8], W, H) is None
    assert adl.intrinsics_from_camera_info(K_OAK, 0, H) is None
    assert adl.intrinsics_from_camera_info(None, W, H) is None


def test_bearing_left_positive_and_matches_vision_geom():
    assert adl.bearing_left_deg(CX, FX, CX) == 0.0
    assert adl.bearing_left_deg(CX - 100.0, FX, CX) > 0.0, '화면 왼쪽 = 좌 +'
    assert adl.bearing_left_deg(CX + 100.0, FX, CX) < 0.0
    # cx = w/2, fx = hfov 에서 나온 값이면 기존 검출기(vision_geom.angle_from_pixel)와 같은 값
    hfov = 63.9
    fx = (W / 2.0) / math.tan(math.radians(hfov / 2.0))
    for u in (0, 100, 320, 500, 639):
        assert abs(adl.bearing_left_deg(u, fx, W / 2.0) - angle_from_pixel(u, W, hfov)) < 1e-9


def test_v_norm_below_center_positive():
    assert adl.v_norm_of(CY, FY, CY) == 0.0
    assert abs(adl.v_norm_of(CY + 51.35, FY, CY) - 0.1) < 1e-12, '아래 + (수면 기준 거리 입력)'
    assert adl.v_norm_of(CY - 10.0, FY, CY) < 0.0


def test_age_stamp_proc_and_clamp():
    assert adl.frame_age_s(100.25, 100.0, 0.01) == (0.25, 0.25, 'stamp')
    a, raw, src = adl.frame_age_s(100.0, 100.3, 0.01)
    assert a == 0.0 and raw < 0.0 and src == 'stamp', '카메라 시각이 앞서면 0 으로 clamp(W3)'
    assert adl.frame_age_s(100.0, 0.0, 0.02) == (0.02, 0.02, 'proc'), 'stamp 0(미설정) → 처리 시간'
    assert adl.frame_age_s(100.0, None, 0.02)[2] == 'proc'
    assert adl.frame_age_s(float('nan'), 99.0, 0.02)[2] == 'proc'
    assert adl.frame_age_s(None, None, -0.001)[0] == 0.0
    a, _raw, _src = adl.frame_age_s(None, None, float('nan'))
    assert a == 0.0 and math.isfinite(a)


def test_conf_bounded_and_monotone():
    assert adl.conf_score(0.0, 1.0, 400.0) == 0.0
    assert adl.conf_score(200.0, 1.0, 400.0) == 0.5
    assert adl.conf_score(4000.0, 1.0, 400.0) == 1.0
    assert adl.conf_score(4000.0, 0.8, 400.0) == 0.8
    assert adl.conf_score(100.0, 0.9, 400.0) < adl.conf_score(300.0, 0.9, 400.0)
    assert adl.conf_score(100.0, None, 400.0) == 0.0
    assert adl.conf_score(100.0, 1.0, 0.0) == 0.0


# ================================================================ 부팅 파라미터 검사 (W2)
def test_check_params_defaults_pass():
    vals, errs = adl.check_params(dict(adl.DEFAULT_PARAMS))
    assert errs == [] and vals == adl.DEFAULT_PARAMS
    assert adl.check_params({}) == (adl.DEFAULT_PARAMS, []), '빠진 키는 기본값(오류 아님)'


@pytest.mark.parametrize('name,value', [
    ('min_area_px', 0.0), ('min_area_px', -5.0), ('min_area_px', float('nan')), ('min_area_px', float('inf')),
    ('min_area_px', 'x'), ('min_area_px', True),
    ('conf_full_area_px', 0.0), ('conf_full_area_px', float('inf')), ('conf_full_area_px', -1.0),
    ('min_solidity', -0.01), ('min_solidity', 1.01), ('min_solidity', float('nan')), ('min_solidity', None),
    ('max_detections', 0), ('max_detections', -3), ('max_detections', 2.0), ('max_detections', True),
])
def test_check_params_bad_value_falls_back_with_error(name, value):
    raw = dict(adl.DEFAULT_PARAMS, **{name: value})
    vals, errs = adl.check_params(raw)
    assert vals == adl.DEFAULT_PARAMS, f'{name}={value!r} → 기본값으로'
    assert len(errs) == 1 and name in errs[0] and 'image_subscriber_avoid' in errs[0]


def test_check_params_edges_accepted():
    vals, errs = adl.check_params({'min_area_px': 1e-3, 'min_solidity': 0.0, 'conf_full_area_px': 1.0,
                                   'max_detections': 1})
    assert errs == [] and vals['min_solidity'] == 0.0 and vals['max_detections'] == 1
    vals, errs = adl.check_params({'min_solidity': 1.0, 'min_area_px': 3})
    assert errs == [] and vals['min_solidity'] == 1.0 and vals['min_area_px'] == 3.0


def test_check_params_only_bad_one_replaced():
    vals, errs = adl.check_params({'min_area_px': 80.0, 'max_detections': 0})
    assert vals['min_area_px'] == 80.0 and vals['max_detections'] == 16 and len(errs) == 1


def test_declared_literals_equal_logic_defaults():
    dec = _declares()
    for k, v in adl.DEFAULT_PARAMS.items():
        assert dec[k] == v and type(dec[k]) is type(v), f'{k}: 선언 {dec[k]!r} ≠ DEFAULT_PARAMS {v!r}'  # noqa: E721


# ================================================================ 합성 이미지 (cv2)
def _hsv_bgr(h, s=255, v=255):
    cv2 = pytest.importorskip('cv2')
    np = pytest.importorskip('numpy')
    px = np.uint8([[[h, s, v]]])
    return tuple(int(c) for c in cv2.cvtColor(px, cv2.COLOR_HSV2BGR)[0, 0])


def _scene():
    """빨강 H0 원(왼쪽) · 빨강 H175 원(오른쪽) · 노랑 사각(가운데 아래) · 작은 빨강 점 · 빨강 십자(들쭉날쭉)."""
    cv2 = pytest.importorskip('cv2')
    np = pytest.importorskip('numpy')
    img = np.zeros((H, W, 3), dtype=np.uint8)
    cv2.circle(img, (120, 200), 25, _hsv_bgr(0), -1)
    cv2.circle(img, (540, 160), 20, _hsv_bgr(175), -1)
    cv2.rectangle(img, (300, 240), (340, 299), _hsv_bgr(30), -1)       # 노랑, 맨 아래 행 299
    cv2.rectangle(img, (60, 40), (63, 43), _hsv_bgr(0), -1)            # 16 px² < min_area 40
    cv2.rectangle(img, (420, 30), (480, 39), _hsv_bgr(0), -1)          # 십자 가로
    cv2.rectangle(img, (445, 5), (454, 64), _hsv_bgr(0), -1)           # 십자 세로 → solidity ≈ 0.5
    return img


def _ranges():
    return {c: hsv_ranges.DEFAULT_RANGES[c] for c in adl.COLOR_CLS}


def test_synthetic_multi_contours_red_two_bands_and_yellow():
    blobs = adl.detect_blobs(_scene(), _ranges(), 40.0, 0.7, 16)
    reds = sorted((b for b in blobs if b['cls'] == adl.CLS_RED), key=lambda b: b['u'])
    yellows = [b for b in blobs if b['cls'] == adl.CLS_YELLOW]
    assert len(reds) == 2, f'빨강 두 구간(H0·H175) 원 2개 — 작은 점·십자는 버림: {[(b["u"], b["area"]) for b in reds]}'
    assert len(yellows) == 1
    assert abs(reds[0]['u'] - 120) < 1.0 and abs(reds[1]['u'] - 540) < 1.0
    assert abs(yellows[0]['u'] - 320) < 1.0 and yellows[0]['v_bottom'] == 299.0
    assert [b['area'] for b in blobs] == sorted((b['area'] for b in blobs), reverse=True), '면적 큰 순'

    dets = adl.blobs_to_detections(blobs, INTR, 0.05, 400.0)
    left = max(dets, key=lambda d: d[1])
    right = min(dets, key=lambda d: d[1])
    assert left[0] == adl.CLS_RED and left[1] > 0.0, '왼쪽 빨강 = 좌 +'
    assert right[0] == adl.CLS_RED and right[1] < 0.0
    y = [d for d in dets if d[0] == adl.CLS_YELLOW][0]
    assert abs(y[2] - (299.0 - CY) / FY) < 1e-12, 'v_norm = (맨 아래 행 − cy)/fy'
    assert all(math.isnan(d[3]) for d in dets), 'U3: range_m 은 늘 NaN'
    assert all(0.0 < d[4] <= 1.0 and d[5] == 0.05 for d in dets)


def test_empty_frame_gives_zero_detections():
    np = pytest.importorskip('numpy')
    blobs = adl.detect_blobs(np.zeros((H, W, 3), dtype=np.uint8), _ranges(), 40.0, 0.7, 16)
    assert blobs == []
    data, _ = adl.pack_detections(adl.blobs_to_detections(blobs, INTR, 0.0, 400.0))
    assert data == [1.0, 6.0, 0.0]


def test_max_detections_keeps_largest():
    blobs = adl.detect_blobs(_scene(), _ranges(), 40.0, 0.7, 2)
    assert len(blobs) == 2
    full = adl.detect_blobs(_scene(), _ranges(), 40.0, 0.7, 16)
    assert [b['area'] for b in blobs] == [b['area'] for b in full[:2]]
    assert adl.detect_blobs(_scene(), _ranges(), 40.0, 0.7, 0) == []


def test_low_solidity_cross_kept_when_threshold_lowered():
    """십자를 버린 이유가 solidity 인지(면적 아님) — 문턱을 내리면 잡힌다."""
    n_hi = len(adl.detect_blobs(_scene(), _ranges(), 40.0, 0.7, 16))
    n_lo = len(adl.detect_blobs(_scene(), _ranges(), 40.0, 0.3, 16))
    assert n_lo == n_hi + 1


def test_unknown_color_ignored():
    r = _ranges()
    r['green'] = hsv_ranges.DEFAULT_RANGES['green']
    assert len(adl.detect_blobs(_scene(), r, 40.0, 0.7, 16)) == 3


# ================================================================ 선언 == vision.yaml
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


def test_yaml_section_equals_declared_literals_and_types():
    dec = _declares()
    y = _yaml_section(NODE_NAME)
    assert set(y) <= set(dec), f'선언 안 한 yaml 키(조용히 버려짐): {set(y) - set(dec)}'
    for k, v in y.items():
        assert v == dec[k] and type(v) is type(dec[k]), f'{k}: yaml {v!r} ≠ 선언 {dec[k]!r}'  # noqa: E721
    for k in ('active_wp_modes', 'debug_view', 'camera_info_topic', 'min_area_px', 'min_solidity',
              'conf_full_area_px', 'max_detections'):
        assert k in y, f'vision.yaml {NODE_NAME} 절에 {k} 없음'


def test_safety_switch_defaults():
    """🚨 §7-2: active_wp_modes [5] · debug_view false — 선언과 yaml 둘 다."""
    dec, y = _declares(), _yaml_section(NODE_NAME)
    assert dec['active_wp_modes'] == y['active_wp_modes'] == [5]
    assert dec['debug_view'] is False and y['debug_view'] is False


def test_hsv_single_source_not_overridden_in_section():
    """HSV 는 `/**` 공통값(gate·turn·dock 과 같은 부표 색). 이 절에 hsv.* 가 있으면 이 노드만 다른 색을 본다."""
    y = _yaml_section(NODE_NAME)
    assert not [k for k in y if k.startswith('hsv.')], y
    assert _declares().get('image_topic') == '/oak/rgb/image_raw'


def test_no_wall_clock_and_qos_reliable():
    src = open(NODE_SRC, encoding='utf-8').read()
    assert 'time.time(' not in src, '§1-5 time.monotonic()'
    m = re.search(r"create_publisher\(\s*Float32MultiArray,\s*'/avoid_detections',\s*(\w+)\s*\)", src)
    assert m, '/avoid_detections 발행자 없음'
    prof = re.search(r'^%s = QoSProfile\(([^)]*)\)' % m.group(1), src, re.M)
    assert prof and 'ReliabilityPolicy.RELIABLE' in prof.group(1) and 'depth=10' in prof.group(1), \
        'ship_avoid 구독(RELIABLE 10)과 맞춘다 — BEST_EFFORT 발행이면 RELIABLE 구독자는 아무것도 못 받는다'
    sa = open(SA_NODE, encoding='utf-8').read()
    assert "create_subscription(Float32MultiArray, '/avoid_detections', self.det_cb, qos)" in sa
    assert re.search(r'qos = QoSProfile\(depth=10, reliability=ReliabilityPolicy.RELIABLE\)', sa)


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
    name = 'color_shape_detector.basic_image_subscriberavoid'
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
    return [m.data for m in node.pubs['/avoid_detections'].msgs]


def _sev(node, sev, text=''):
    return [m for s, m in node.logger.lines if s == sev and text in m]


def test_node_declares_before_subscriptions(node_mod):
    mod, _ = node_mod
    node = mod.ImageSubscriberAvoid()
    assert node.name == NODE_NAME
    first_io = min(i for i, (kind, _k) in enumerate(ORDER) if kind in ('sub', 'pub'))
    last_decl = max(i for i, (kind, _k) in enumerate(ORDER) if kind == 'declare')
    assert last_decl < first_io, f'파라미터 선언이 구독·발행자 생성보다 먼저여야 한다: {ORDER}'
    assert set(node.subs) == {'/wp_mode', '/oak/rgb/image_raw', '/oak/rgb/camera_info'}
    assert node.debug_view is False and node.mode_gate.active_modes == {5}
    assert set(node.color_ranges) == {'red', 'yellow'}


def test_node_silent_outside_mode5(node_mod):
    mod, mono = node_mod
    node = mod.ImageSubscriberAvoid()
    node.subs['/oak/rgb/camera_info'](_info())
    node.subs['/oak/rgb/image_raw'](_img(_scene()))
    assert _out(node) == [], '/wp_mode 미수신 → 발행 안 함'
    assert _sev(node, 'WARN', '비활성')
    for m in (0, 1, 2, 3, 7, 8):
        node.subs['/wp_mode'](_Msg(data=m))
        node.subs['/oak/rgb/image_raw'](_img(_scene()))
    assert _out(node) == [], '담당 모드(5)가 아니면 발행 안 함(발행자 충돌·CPU)'
    assert not _sev(node, 'ERROR')


def test_node_heartbeat_from_first_mode5_frame_and_detections(node_mod):
    ml = _ship_avoid_logic()
    np = pytest.importorskip('numpy')
    mod, mono = node_mod
    node = mod.ImageSubscriberAvoid()
    node.subs['/oak/rgb/camera_info'](_info())
    node.subs['/wp_mode'](_Msg(data=5))
    assert _sev(node, 'INFO', '발행 시작')
    node.ros_clock.ns = int(50.1 * 1e9)
    node.subs['/oak/rgb/image_raw'](_img(np.zeros((H, W, 3), dtype=np.uint8), stamp_s=50.0))
    assert _out(node) == [[1.0, 6.0, 0.0]], 'mode 5 첫 프레임부터 N=0 하트비트'
    node.subs['/oak/rgb/image_raw'](_img(_scene(), stamp_s=50.0))
    got, n_old, err = ml.parse_detections(_f32(_out(node)[-1]), 0.5)
    assert err is None and n_old == 0
    assert sorted(d[0] for d in got) == [0, 0, 2]
    assert all(abs(d[5] - 0.1) < 1e-4 for d in got), 'age_s = ROS 지금 − header.stamp'
    assert max(got, key=lambda d: d[1])[1] > 0.0
    assert not _sev(node, 'ERROR')


def test_node_no_camera_info_silent_with_error(node_mod):
    mod, mono = node_mod
    node = mod.ImageSubscriberAvoid()
    node.subs['/wp_mode'](_Msg(data=5))
    node.subs['/oak/rgb/image_raw'](_img(_scene()))
    assert _out(node) == [], 'camera_info 없으면 방위를 지어내지 않는다'
    assert _sev(node, 'ERROR', '미수신')


def test_node_invalid_k_or_size_mismatch_silent_with_error(node_mod):
    mod, mono = node_mod
    node = mod.ImageSubscriberAvoid()
    node.subs['/wp_mode'](_Msg(data=5))
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
    ml = _ship_avoid_logic()
    mod, mono = node_mod
    node = mod.ImageSubscriberAvoid()
    node.subs['/wp_mode'](_Msg(data=5))
    node.subs['/oak/rgb/camera_info'](_info())
    node.ros_clock.ns = int(50.0 * 1e9)
    node.subs['/oak/rgb/image_raw'](_img(_scene(), stamp_s=50.4))
    data = _out(node)[-1]
    got, _n_old, err = ml.parse_detections(_f32(data), 0.5)
    assert err is None and got and all(d[5] == 0.0 for d in got), 'age_s 0 으로 clamp — ship_avoid 가 버리지 않음'
    assert _sev(node, 'WARN', '앞섬')


def test_node_stamp_missing_uses_processing_time(node_mod):
    mod, mono = node_mod
    node = mod.ImageSubscriberAvoid()
    node.subs['/wp_mode'](_Msg(data=5))
    node.subs['/oak/rgb/camera_info'](_info())
    node.ros_clock.ns = int(50.0 * 1e9)
    node.subs['/oak/rgb/image_raw'](_img(_scene(), stamp_s=None))
    data = _out(node)[-1]
    assert data[2] == 3.0 and all(data[3 + i * 6 + 5] == 0.0 for i in range(3)), '스텁 monotonic 고정 → 처리 시간 0'


def test_node_wp_mode_stale_goes_silent(node_mod):
    mod, mono = node_mod
    node = mod.ImageSubscriberAvoid()
    node.subs['/wp_mode'](_Msg(data=5))
    node.subs['/oak/rgb/camera_info'](_info())
    node.subs['/oak/rgb/image_raw'](_img(_scene()))
    assert len(_out(node)) == 1
    mono.t += 2.5
    node.subs['/oak/rgb/image_raw'](_img(_scene()))
    assert len(_out(node)) == 1, '/wp_mode 2 s 넘게 묵으면 발행 안 함'
    assert _sev(node, 'WARN', 'wp_mode_stale')


def test_node_bad_params_error_once_and_defaults(node_mod):
    mod, mono = node_mod
    OVERRIDES.update({'max_detections': 0, 'min_solidity': 1.5})
    node = mod.ImageSubscriberAvoid()
    errs = _sev(node, 'ERROR')
    assert len(errs) == 2 and any('max_detections' in e for e in errs) and any('min_solidity' in e for e in errs)
    assert node.max_detections == 16 and node.min_solidity == 0.7
    node.subs['/wp_mode'](_Msg(data=5))
    node.subs['/oak/rgb/camera_info'](_info())
    node.subs['/oak/rgb/image_raw'](_img(_scene()))
    assert _out(node)[-1][2] == 3.0, '기본값으로 돌아 검출이 나간다'
    assert len(_sev(node, 'ERROR')) == 2, '부팅 1회뿐(프레임마다 안 냄)'
