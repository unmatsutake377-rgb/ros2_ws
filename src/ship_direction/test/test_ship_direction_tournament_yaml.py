#!/usr/bin/env python3
"""ship_direction 토너먼트 yaml 정적 테스트 (계약서_토너먼트분리_20261109 T1-2, §9-3).

실행: tools/run_tests.sh ship_direction

ROS 없이 파일만 읽는다. 값 자체(dwa_enabled true, dwa_modes [8])는 현장 스위치라 고정하지 않고
형식·타입·범위만 본다(motor_control test_motor_tournament_yaml 과 같은 방침). 강제하는 것:
  Y1 새 yaml 이 파싱되고 최상위가 공통 yaml 과 같다(`/ship_direction:` → `ros__parameters:`), 키 1개 이상
  Y2 키 ⊆ ship_direction.py declare_parameter 키 (미선언 키 = 조용히 버려지는 죽은 설정)
  Y3 값 타입 = 코드 기본값 타입, 공통 yaml 에도 있는 키면 그 타입과도 같다 (리스트는 원소 타입까지)
  Y4 dwa_enabled 는 bool, dwa_modes 는 비지 않은 int 리스트이고 노드 DWA_ALLOWED_MODES 안
  Y5 dwa_hull 은 없다 (launch 가 boat 인자에서 넘긴다, T1-4)
  Y6 setup.py data_files 가 새 yaml 을 share/ship_direction/config 에 설치한다
"""

import ast
import os

import yaml

HERE = os.path.dirname(__file__)
PKG = os.path.join(HERE, '..')
NODE_KEY = '/ship_direction'
TOURN = 'ship_direction_tournament.yaml'


def _read(*parts):
    with open(os.path.join(PKG, *parts), encoding='utf-8') as f:
        return f.read()


def _params(name):
    doc = yaml.safe_load(_read('config', name))
    assert list(doc) == [NODE_KEY], list(doc)
    assert list(doc[NODE_KEY]) == ['ros__parameters'], list(doc[NODE_KEY])
    return doc[NODE_KEY]['ros__parameters']


def _node_tree():
    return ast.parse(_read('ship_direction', 'ship_direction.py'))


def _declared_defaults():
    """declare_parameter('이름', 기본값) → {이름: 기본값}. 기본값이 리터럴이 아니면 실패."""
    out = {}
    for node in ast.walk(_node_tree()):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) \
                and node.func.attr == 'declare_parameter':
            name = ast.literal_eval(node.args[0])
            out[name] = ast.literal_eval(node.args[1])
    return out


def _allowed_modes():
    for node in ast.walk(_node_tree()):
        if isinstance(node, ast.Assign) and len(node.targets) == 1 \
                and getattr(node.targets[0], 'id', None) == 'DWA_ALLOWED_MODES':
            return set(ast.literal_eval(node.value))
    raise AssertionError('ship_direction.py 에 DWA_ALLOWED_MODES 없음')


def _same_type(a, b, key):
    assert type(a) is type(b), (key, type(a), type(b))
    if isinstance(a, list):
        assert a and b, key   # 빈 리스트는 원소 타입이 정해지지 않는다
        assert {type(x) for x in a} == {type(x) for x in b}, (key, a, b)


def test_y1_parses():
    assert _params(TOURN)


def test_y2_keys_are_declared():
    p = _params(TOURN)
    declared = _declared_defaults()
    assert set(p) <= set(declared), set(p) - set(declared)


def test_y3_types_match_default_and_common():
    p = _params(TOURN)
    defaults = _declared_defaults()
    common = _params('ship_direction.yaml')
    for k, v in p.items():
        _same_type(v, defaults[k], k)
        if k in common:
            _same_type(v, common[k], k)


def test_y4_dwa_switch_range():
    p = _params(TOURN)
    allowed = _allowed_modes()
    if 'dwa_enabled' in p:
        assert type(p['dwa_enabled']) is bool, p['dwa_enabled']
    if 'dwa_modes' in p:
        modes = p['dwa_modes']
        assert isinstance(modes, list) and modes, modes
        assert all(type(m) is int for m in modes), modes
        assert set(modes) <= allowed, (modes, allowed)


def test_y5_no_hull():
    assert 'dwa_hull' not in _params(TOURN)


def _config_install_list():
    for node in ast.walk(ast.parse(_read('setup.py'))):
        if isinstance(node, ast.Tuple) and len(node.elts) == 2 \
                and isinstance(node.elts[1], ast.List) \
                and ast.unparse(node.elts[0]).endswith("'/config'"):
            return [ast.literal_eval(e) for e in node.elts[1].elts]
    raise AssertionError('setup.py 에 config data_files 없음')


def test_y6_setup_installs():
    files = _config_install_list()
    assert 'config/ship_direction.yaml' in files, files
    assert 'config/' + TOURN in files, files
    for f in files:
        assert os.path.isfile(os.path.join(PKG, f)), f
