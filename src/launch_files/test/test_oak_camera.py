#!/usr/bin/env python3
"""
대회 launch 카메라 = OAK-1 PoE(depthai_ros_driver) 정적 테스트 (계약서_OAK전환_20261104 §6-b, 결정 §10).
ROS 없이 돈다 — launch 파일은 ast 로, yaml 은 정규식으로 읽는다.

    tools/run_tests.sh launch_files

지키는 것
  ① depthai_ros_driver Node 정확히 1개, realsense2_camera Node 0개 (결정 1: 대회 launch 에서 D455 제거).
     camera.launch.py(벤치)의 realsense 는 그대로 — CLAUDE.md §1-8 "RealSense 경로 삭제 금지".
  ② 그 Node 는 name='oak', namespace 없음 → 토픽 /oak/rgb/image_raw (런북 10-31 실측 = vision.yaml image_topic).
  ③ parameters= 로 설치 yaml(config/oak_640x360_full.yaml)을 넘기고, setup.py 가 config/*.yaml 을 설치한다.
  ④ 설치 yaml 의 값: i_ip 169.254.1.222, 640×360, **i_isp_num 1 / i_isp_den 3**, i_low_bandwidth true,
     1080P, 30 fps, RGB 파이프라인 — ISP 1/3 이 빠지면 화각 40.3° 인데 camera_info 로는 안 드러난다(계약 위험 1).
  ⑤ 설치 yaml 의 키·값 == tools/oak/oak_640x360_full.yaml (런북 원본).
  ⑥ respawn=True, respawn_delay ≥ 10 s(계약) — 런북은 20 s 이상 권고.
  ⑦ 패키지 미설치 시 건너뛰기: PackageNotFoundError 를 잡고 LogInfo 를 낸다(결정 5, 계약 위험 3).
  ⑧ HFOV 63.9 가 640×360 ISP 1/3 설정과 일치: camera_info fx 513.5 ↔ 2·atan(320/513.5) = 63.87°.
"""

import ast
import math
import os
import re

import pytest

PKG = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
SRC = os.path.dirname(PKG)
WS = os.path.dirname(SRC)
LAUNCH = os.path.join(PKG, 'launch', 'launch_files.launch.py')
SETUP = os.path.join(PKG, 'setup.py')
OAK_YAML = os.path.join(PKG, 'config', 'oak_640x360_full.yaml')
OAK_YAML_ORIGIN = os.path.join(WS, 'tools', 'oak', 'oak_640x360_full.yaml')
VISION_YAML = os.path.join(SRC, 'color_shape_detector', 'config', 'vision.yaml')

OAK_IP = '169.254.1.222'
IMAGE_TOPIC = '/oak/rgb/image_raw'
MIN_RESPAWN_DELAY_S = 10.0
# 런북 10-31 저녁 화각 실측: camera_info fx 513.5(640 px) → 63.87°, 원 간격 실측 63.9°.
OAK_FX_PX = 513.5
OAK_WIDTH = 640


def _tree():
    with open(LAUNCH, encoding='utf-8') as f:
        return ast.parse(f.read(), filename=LAUNCH)


def _call_name(node):
    f = node.func
    return f.attr if isinstance(f, ast.Attribute) else getattr(f, 'id', None)


def _kw(call, name):
    for k in call.keywords:
        if k.arg == name:
            return k.value
    return None


def _nodes_of_package(tree, package):
    return [n for n in ast.walk(tree)
            if isinstance(n, ast.Call) and _call_name(n) == 'Node'
            and isinstance(_kw(n, 'package'), ast.Constant)
            and _kw(n, 'package').value == package]


def _yaml_scalars(path):
    """`key: value` 줄을 전부 {key: value} 로 (pyyaml 없이, 들여쓰기 무시). 주석·블록 헤더는 뺀다."""
    out = {}
    with open(path, encoding='utf-8') as f:
        for line in f:
            line = line.split('#', 1)[0].rstrip()
            m = re.match(r'^\s*([A-Za-z_][\w]*):\s*(\S.*)$', line)
            if m:
                out[m.group(1)] = m.group(2).strip().strip('"').strip("'")
    return out


@pytest.fixture(scope='module')
def depthai_node():
    nodes = _nodes_of_package(_tree(), 'depthai_ros_driver')
    assert len(nodes) == 1, f'depthai_ros_driver Node {len(nodes)}개 — 정확히 1개여야 한다'
    return nodes[0]


@pytest.fixture(scope='module')
def oak_params():
    assert os.path.isfile(OAK_YAML), f'{OAK_YAML} 없음 — launch 가 넘길 설치 yaml'
    return _yaml_scalars(OAK_YAML)


# ------------------------------------------------------------ ①  노드 수

def test_exactly_one_depthai_and_no_realsense_node(depthai_node):
    tree = _tree()
    assert len(_nodes_of_package(tree, 'realsense2_camera')) == 0, (
        '대회 launch 에 realsense2_camera Node 가 남아 있다 — 결정 1(D455 제거). 벤치는 camera.launch.py')
    assert not re.search(r"IncludeLaunchDescription[^)]*realsense", open(LAUNCH, encoding='utf-8').read())


# ------------------------------------------------------------ ②  이름 → 토픽

def test_node_name_oak_without_namespace(depthai_node):
    name = _kw(depthai_node, 'name')
    assert isinstance(name, ast.Constant) and name.value == 'oak', (
        "name='oak' 이어야 토픽이 /oak/rgb/image_raw (런북 실측) 가 된다")
    ns = _kw(depthai_node, 'namespace')
    assert ns is None or (isinstance(ns, ast.Constant) and ns.value in ('', '/')), (
        '네임스페이스를 주면 토픽이 /<ns>/oak/rgb/image_raw 로 바뀌어 검출기가 침묵한다')
    vision = _yaml_scalars(VISION_YAML)
    assert vision.get('image_topic') == IMAGE_TOPIC, (
        f"vision.yaml image_topic={vision.get('image_topic')!r} ≠ {IMAGE_TOPIC} — 검출기가 다른 토픽을 본다")


# ------------------------------------------------------------ ③  yaml 전달·설치

def test_parameters_point_to_installed_yaml(depthai_node):
    params = _kw(depthai_node, 'parameters')
    assert params is not None, 'parameters= 없음 — i_ip·ISP 1/3 이 안 닿아 자동탐색 실패/화각 40.3°'
    src = ast.get_source_segment(open(LAUNCH, encoding='utf-8').read(), params) or ''
    body = open(LAUNCH, encoding='utf-8').read()
    # parameters=[oak_cfg] 변수 또는 경로 리터럴 — 어느 쪽이든 oak_640x360_full.yaml 이 launch 에 등장해야 한다
    assert 'oak_640x360_full.yaml' in body, 'launch 가 oak_640x360_full.yaml 을 참조하지 않는다'
    assert "get_package_share_directory('launch_files')" in body, '설치 경로(share/launch_files/config)로 찾아야 한다'
    assert src, 'parameters 소스 세그먼트를 못 읽음'


def test_setup_installs_config_yaml():
    setup = open(SETUP, encoding='utf-8').read()
    assert re.search(r"glob\(\s*['\"]config/\*\.yaml['\"]\s*\)", setup), (
        "setup.py data_files 에 config/*.yaml 이 없다 — colcon build 뒤 share/launch_files/config 가 비어 launch 가 죽는다")
    assert re.search(r"os\.path\.join\(\s*'share'\s*,\s*package_name\s*,\s*'config'\s*\)", setup)


# ------------------------------------------------------------ ④  값

def test_oak_yaml_values(oak_params):
    p = oak_params
    assert p.get('i_ip') == OAK_IP, f"i_ip={p.get('i_ip')!r} — 자동탐색은 실패한다(런북), 폴백 IP 고정"
    assert p.get('i_width') == '640' and p.get('i_height') == '360', f"{p.get('i_width')}x{p.get('i_height')}"
    assert p.get('i_isp_num') == '1' and p.get('i_isp_den') == '3', (
        f"ISP {p.get('i_isp_num')}/{p.get('i_isp_den')} — 1/3 이 아니면 가운데만 잘려 화각 40.3° (hfov 63.9 와 8° 오차)")
    assert p.get('i_low_bandwidth') == 'true', '100 Mb 링크에서 raw 는 8 fps/포화 — MJPEG 필수'
    assert p.get('i_resolution') == '1080P'
    assert float(p.get('i_fps')) == 30.0
    assert p.get('i_pipeline_type') == 'RGB', 'OAK-1 은 모노 카메라 — 뎁스 파이프라인이면 기동 실패'
    assert p.get('i_nn_type') == 'none'


def test_oak_yaml_block_is_node_oak():
    text = open(OAK_YAML, encoding='utf-8').read()
    assert re.search(r'^/oak:\s*$', text, re.M), 'yaml 최상위 블록이 /oak 가 아니면 name=oak 노드에 안 닿는다'


# ------------------------------------------------------------ ⑤  원본과 일치

def test_oak_yaml_matches_tools_origin(oak_params):
    assert os.path.isfile(OAK_YAML_ORIGIN), f'{OAK_YAML_ORIGIN} 없음'
    origin = _yaml_scalars(OAK_YAML_ORIGIN)
    assert oak_params == origin, (
        f'설치 yaml 과 tools/oak 원본의 키·값이 다르다:\n  설치={oak_params}\n  원본={origin}')


# ------------------------------------------------------------ ⑥  respawn

def test_respawn_enabled_with_delay(depthai_node):
    respawn = _kw(depthai_node, 'respawn')
    assert isinstance(respawn, ast.Constant) and respawn.value is True, 'respawn=True 여야 드라이버가 죽어도 되살아난다'
    delay = _kw(depthai_node, 'respawn_delay')
    assert delay is not None, 'respawn_delay 없음 — 즉시 재기동은 카메라 재부팅(~14 s)과 겹쳐 연결 실패'
    if isinstance(delay, ast.Constant):
        value = float(delay.value)
    else:
        # 모듈 상수(OAK_RESPAWN_DELAY_S 등)로 넘긴 경우: 그 대입을 찾는다
        assert isinstance(delay, ast.Name), f'respawn_delay 표현식을 못 읽음: {ast.dump(delay)}'
        assigns = [n for n in ast.walk(_tree()) if isinstance(n, ast.Assign)
                   and any(isinstance(t, ast.Name) and t.id == delay.id for t in n.targets)]
        assert len(assigns) == 1, f'{delay.id} 대입 {len(assigns)}개'
        value = float(ast.literal_eval(assigns[0].value))
    assert value >= MIN_RESPAWN_DELAY_S, f'respawn_delay {value} < {MIN_RESPAWN_DELAY_S} s (런북: 20 s 이상 권고)'


# ------------------------------------------------------------ ⑦  미설치 시 건너뛰기

def test_skips_when_depthai_package_missing():
    tree = _tree()
    fns = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == '_oak_camera']
    assert len(fns) == 1, '_oak_camera 함수(OpaqueFunction)가 없다'
    fn = fns[0]
    handlers = [h for h in ast.walk(fn) if isinstance(h, ast.ExceptHandler)]
    names = {getattr(h.type, 'id', getattr(h.type, 'attr', None)) for h in handlers}
    assert 'PackageNotFoundError' in names, (
        'get_package_share_directory 를 try/except PackageNotFoundError 로 감싸야 미설치 노트북에서 launch 가 안 죽는다')
    # except 블록 안에 LogInfo 가 있고 Node 는 없어야 한다
    for h in handlers:
        calls = {_call_name(c) for c in ast.walk(h) if isinstance(c, ast.Call)}
        if 'LogInfo' in calls:
            assert 'Node' not in calls, '미설치인데 Node 를 띄우면 그 자리에서 죽는다'
            break
    else:
        pytest.fail('PackageNotFoundError 처리에 LogInfo 가 없다 — 조용히 건너뛰면 아무도 모른다')
    uses = [n for n in ast.walk(tree) if isinstance(n, ast.Call) and _call_name(n) == 'OpaqueFunction'
            and isinstance(_kw(n, 'function'), ast.Name) and _kw(n, 'function').id == '_oak_camera']
    assert len(uses) == 1, f'OpaqueFunction(function=_oak_camera) {len(uses)}개'


# ------------------------------------------------------------ ⑧  화각 ↔ 설정

def test_hfov_consistent_with_isp_one_third():
    vision = _yaml_scalars(VISION_YAML)
    hfov = float(vision['hfov_deg'])
    hfov_from_fx = 2.0 * math.degrees(math.atan((OAK_WIDTH / 2.0) / OAK_FX_PX))
    assert abs(hfov - hfov_from_fx) < 0.1, (
        f'vision.yaml hfov_deg={hfov} vs camera_info fx {OAK_FX_PX} → {hfov_from_fx:.2f}° '
        f'(640×360 ISP 1/3 기준). 40.3° 면 ISP 2/3 잘림 상태 값이다')
