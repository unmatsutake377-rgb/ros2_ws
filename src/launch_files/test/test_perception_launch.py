#!/usr/bin/env python3
"""
대회 launch 의 인식 융합 인자 `d455_depth`·`perception_fusion` 정적 테스트 (계약서_인식융합_20261112 §10 Q8·Q9).
ROS 없이 돈다 — launch 파일에서 ast 로 함수만 떼어 가짜 LaunchConfiguration·Node 로 실행한다.

    tools/run_tests.sh launch_files

지키는 것
  ① 두 인자 각 1개, 기본 'true'(대회에서 켬), choices ['true', 'false'] — ast·정규식 두 경로
  ② false 면 Node 0개(LogInfo 만) → 둘 다 false = 이 계약 전과 같은 노드 구성
  ③ true 면 Node 1개: realsense2_camera name=camera ns=camera / perception_fusion parameters=[_pkg_yaml]
  ④ 패키지가 없으면 Node 없이 LogInfo(launch 전체를 세우지 않는다)
  ⑤ D455 = 뎁스 전용: 컬러·IR·IMU·pointcloud·align·hole_filling·initial_reset 끔(Python bool), 뎁스 켬,
     depth_module.depth_profile 이 래퍼 정규식([xX,] 구분, profile_manager.cpp:453)에 맞는 480x270x15
     (424x240 은 2번배 D455 USB2 목록에 없음 — 계약서_도리도리회피_v2_20261114 U0)
  ⑥ 드라이버 이름(ns/name) → 토픽 = perception_fusion.yaml depth_topic·depth_info_topic
"""

import ast
import os
import re

import pytest

PKG = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
SRC = os.path.dirname(PKG)
LAUNCH = os.path.join(PKG, 'launch', 'launch_files.launch.py')
FUSION_YAML = os.path.join(SRC, 'perception_fusion', 'config', 'perception_fusion.yaml')
ARGS = ('d455_depth', 'perception_fusion')
# realsense-ros 4.56.4 profile_manager.cpp:453 의 정규식 그대로
RS_PROFILE_RE = re.compile(r'\s*([0-9]+)\s*[xX,]\s*([0-9]+)\s*[xX,]\s*([0-9]+)\s*')


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


# ------------------------------------------------------------ 가짜 launch 객체

class _LC:
    VALUES = {}

    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return self.VALUES[self.name]


class _Node:
    def __init__(self, **kw):
        self.kw = kw


class _LogInfo:
    def __init__(self, msg):
        self.msg = msg


class _NotFound(Exception):
    pass


def _load(missing=()):
    """launch 의 d455_depth_params·_d455_depth·_perception_fusion 을 떼어 가짜 의존성으로 실행 가능하게."""
    names = ('d455_depth_params', '_d455_depth', '_perception_fusion')
    fns = [n for n in _tree().body if isinstance(n, ast.FunctionDef) and n.name in names]
    assert sorted(f.name for f in fns) == sorted(names)

    def share(pkg):
        if pkg in missing:
            raise _NotFound(pkg)
        return f'/install/{pkg}/share/{pkg}'

    ns = {'LaunchConfiguration': _LC, 'Node': _Node, 'LogInfo': _LogInfo,
          'PackageNotFoundError': _NotFound, 'get_package_share_directory': share,
          '_pkg_yaml': lambda pkg: f'/install/{pkg}/share/{pkg}/config/{pkg}.yaml'}
    exec(compile(ast.Module(body=fns, type_ignores=[]), '<launch>', 'exec'), ns)
    return ns


def _run(fn_name, values, missing=()):
    _LC.VALUES = dict(values)
    out = _load(missing)[fn_name](None)
    return [a for a in out if isinstance(a, _Node)], [a for a in out if isinstance(a, _LogInfo)]


# ------------------------------------------------------------ ① 인자

@pytest.mark.parametrize('arg', ARGS)
def test_arg_default_true_with_choices(arg):
    calls = [n for n in ast.walk(_tree()) if isinstance(n, ast.Call) and _call_name(n) == 'DeclareLaunchArgument'
             and n.args and isinstance(n.args[0], ast.Constant) and n.args[0].value == arg]
    assert len(calls) == 1, f"DeclareLaunchArgument('{arg}') {len(calls)}개"
    assert _kw(calls[0], 'default_value').value == 'true', '§10 Q8: 대회에서 켬 → 기본 true'
    assert ast.literal_eval(_kw(calls[0], 'choices')) == ['true', 'false']


@pytest.mark.parametrize('arg', ARGS)
def test_arg_default_by_regex(arg):
    src = open(LAUNCH, encoding='utf-8').read()
    found = re.findall(r"DeclareLaunchArgument\(\s*['\"]" + arg + r"['\"]\s*,\s*default_value\s*=\s*['\"](\w+)['\"]",
                       src)
    assert found == ['true']


# ------------------------------------------------------------ ② false → Node 없음 / ③ true → Node 1개

@pytest.mark.parametrize('fn,arg', [('_d455_depth', 'd455_depth'), ('_perception_fusion', 'perception_fusion')])
def test_false_means_no_node(fn, arg):
    nodes, logs = _run(fn, {arg: 'false'})
    assert nodes == [] and logs, f'{arg}=false 인데 Node 를 띄움(또는 LogInfo 없음)'


def test_both_false_same_as_before():
    """둘 다 false → 두 진입점이 Node 를 하나도 안 낸다 = 이 계약 전 노드 구성."""
    vals = {'d455_depth': 'false', 'perception_fusion': 'false'}
    assert _run('_d455_depth', vals)[0] == [] and _run('_perception_fusion', vals)[0] == []


def test_d455_true_node():
    nodes, _ = _run('_d455_depth', {'d455_depth': 'true'})
    assert len(nodes) == 1
    kw = nodes[0].kw
    assert kw['package'] == 'realsense2_camera' and kw['executable'] == 'realsense2_camera_node'
    assert kw['name'] == 'camera' and kw['namespace'] == 'camera', 'camera.launch.py 와 같은 이름 → /camera/camera/...'
    assert kw['parameters'] == [_load()['d455_depth_params']()]


def test_perception_fusion_true_node():
    nodes, _ = _run('_perception_fusion', {'perception_fusion': 'true'})
    assert len(nodes) == 1
    kw = nodes[0].kw
    assert kw['package'] == kw['executable'] == kw['name'] == 'perception_fusion'
    assert kw['parameters'] == ['/install/perception_fusion/share/perception_fusion/config/perception_fusion.yaml']


# ------------------------------------------------------------ ④ 패키지 없음 → 건너뜀

@pytest.mark.parametrize('fn,arg,pkg', [('_d455_depth', 'd455_depth', 'realsense2_camera'),
                                        ('_perception_fusion', 'perception_fusion', 'perception_fusion')])
def test_missing_package_skips(fn, arg, pkg):
    nodes, logs = _run(fn, {arg: 'true'}, missing=(pkg,))
    assert nodes == [] and logs and '⏭️' in logs[0].msg


# ------------------------------------------------------------ ⑤ D455 뎁스 전용 파라미터

def test_d455_depth_only_params():
    p = _load()['d455_depth_params']()
    want_false = ('enable_color', 'enable_infra1', 'enable_infra2', 'enable_gyro', 'enable_accel',
                  'pointcloud.enable', 'align_depth.enable', 'hole_filling_filter.enable', 'initial_reset')
    for k in want_false:
        assert p[k] is False, f'{k} = {p[k]!r} — Python False 여야 한다'
    assert p['enable_depth'] is True
    prof = p['depth_module.depth_profile']
    m = RS_PROFILE_RE.fullmatch(prof)
    assert m and m.groups() == ('480', '270', '15'), f'뎁스 프로필 {prof!r}'
    assert 'depth_module.profile' not in p, '옛 이름(4.56 에선 depth_module.depth_profile)'


# ------------------------------------------------------------ ⑥ 토픽 이름 = perception_fusion.yaml

def test_driver_topics_match_fusion_yaml():
    nodes, _ = _run('_d455_depth', {'d455_depth': 'true'})
    kw = nodes[0].kw
    base = f"/{kw['namespace']}/{kw['name']}/depth"
    text = open(FUSION_YAML, encoding='utf-8').read()
    topic = re.search(r'^\s*depth_topic:\s*"([^"]+)"', text, re.M).group(1)
    info = re.search(r'^\s*depth_info_topic:\s*"([^"]+)"', text, re.M).group(1)
    assert topic == f'{base}/image_rect_raw'
    assert info == f'{base}/camera_info'
