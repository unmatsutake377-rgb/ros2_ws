#!/usr/bin/env python3
"""
배별 IMU 설정 정적 테스트 (계약서_배별IMU설정_20261029). ROS 없이 돈다 — pyyaml 과 ast 만 쓴다.

    tools/run_tests.sh ssf_heading

지키는 것
  ① ssf_heading_A/B.yaml 이 있고, yaw_mux 절의 키가 **정확히** {mount_offset_deg, invert_yaw}
     (안전 스위치 cog_require_reverse_gate 같은 게 배별 파일로 새어 들어가면 한 배에서만 꺼진다)
  ② mount_offset_deg 는 float, invert_yaw 는 bool
     — 10-29 Ubuntu 실측: `mount_offset_deg: 37`(정수) → yaw_mux 부팅 즉시 사망
  ③ 공통 파일(ssf_heading.yaml)엔 두 키가 없다 (있으면 배별 값에 덮여 고쳐도 안 먹는다)
  ④ 공통 파일에 cog_require_reverse_gate: true (CLAUDE.md 7-2 대회값)
  ⑤ 두 launch(벤치·대회)에 `boat` 인자 — choices A,B · 기본값 없음 — 와 공통 → 배별 순서
"""

import ast
import os

import pytest
import yaml

PKG = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
SRC = os.path.dirname(PKG)
CFG = os.path.join(PKG, 'config')
COMMON = os.path.join(CFG, 'ssf_heading.yaml')
BOATS = ('A', 'B')
BOAT_KEYS = {'mount_offset_deg', 'invert_yaw'}

LAUNCHES = {
    'bench': os.path.join(PKG, 'launch', 'ssf_heading.launch.py'),
    'competition': os.path.join(SRC, 'launch_files', 'launch', 'launch_files.launch.py'),
}


def _yaw_mux_params(path):
    with open(path, encoding='utf-8') as f:
        doc = yaml.safe_load(f)
    assert isinstance(doc, dict) and 'yaw_mux' in doc, f'{path}: yaw_mux 절 없음'
    params = doc['yaw_mux'].get('ros__parameters')
    assert isinstance(params, dict), f'{path}: yaw_mux.ros__parameters 없음'
    return params


def _boat_file(boat):
    return os.path.join(CFG, f'ssf_heading_{boat}.yaml')


# ------------------------------------------------------------ ①②  배별 파일

@pytest.mark.parametrize('boat', BOATS)
def test_boat_file_has_exactly_two_keys(boat):
    path = _boat_file(boat)
    assert os.path.isfile(path), f'{path} 없음'
    with open(path, encoding='utf-8') as f:
        doc = yaml.safe_load(f)
    assert set(doc) == {'yaw_mux'}, f'{boat}: yaw_mux 외 절이 있다 {set(doc)}'
    assert set(doc['yaw_mux']) == {'ros__parameters'}
    keys = set(_yaw_mux_params(path))
    assert keys == BOAT_KEYS, f'{boat}: 키가 {sorted(keys)} — {sorted(BOAT_KEYS)} 만 둔다'


@pytest.mark.parametrize('boat', BOATS)
def test_boat_file_types(boat):
    p = _yaw_mux_params(_boat_file(boat))
    # bool 은 int 의 하위형이라 isinstance 로는 못 거른다 → type is
    assert type(p['mount_offset_deg']) is float, (
        f"{boat}: mount_offset_deg={p['mount_offset_deg']!r} — 소수점으로 (37 ❌ → 37.0 ✅). "
        '정수면 yaw_mux 가 부팅 즉시 죽는다')
    assert type(p['invert_yaw']) is bool, (
        f"{boat}: invert_yaw={p['invert_yaw']!r} — true/false 로")


# ------------------------------------------------------------ ③④  공통 파일

def test_common_file_has_no_boat_keys():
    leaked = BOAT_KEYS & set(_yaw_mux_params(COMMON))
    assert not leaked, f'공통 파일에 배별 키 {sorted(leaked)} — 배별 파일로'


def test_common_file_keeps_reverse_gate_on():
    p = _yaw_mux_params(COMMON)
    assert p.get('cog_require_reverse_gate') is True, 'CLAUDE.md 7-2: 대회값 true'


# ------------------------------------------------------------ ⑤  launch (정적)

def _tree(path):
    with open(path, encoding='utf-8') as f:
        return ast.parse(f.read(), filename=path)


def _call_name(node):
    f = node.func
    return f.attr if isinstance(f, ast.Attribute) else getattr(f, 'id', None)


def _kw(call, name):
    for k in call.keywords:
        if k.arg == name:
            return k.value
    return None


def _boat_arg_calls(tree):
    out = []
    for n in ast.walk(tree):
        if (isinstance(n, ast.Call) and _call_name(n) == 'DeclareLaunchArgument'
                and n.args and isinstance(n.args[0], ast.Constant)
                and n.args[0].value == 'boat'):
            out.append(n)
    return out


@pytest.mark.parametrize('which', sorted(LAUNCHES))
def test_launch_declares_required_boat_arg(which):
    calls = _boat_arg_calls(_tree(LAUNCHES[which]))
    assert len(calls) == 1, f'{which}: boat DeclareLaunchArgument {len(calls)}개'
    call = calls[0]
    choices = _kw(call, 'choices')
    assert choices is not None, f'{which}: choices 없음 → 오타(boat:=a)가 통과한다'
    assert ast.literal_eval(choices) == ['A', 'B'], f'{which}: choices 는 대문자 A,B'
    # 기본값이 있으면 다른 배에서 조용히 그 배 값으로 뜬다 (Davidson 결정 1: 필수)
    assert _kw(call, 'default_value') is None, f'{which}: boat 에 기본값 금지'
    assert len(call.args) == 1, f'{which}: 위치 인자로 기본값을 주지 말 것'


def _load_func(tree, name):
    """launch 파일에서 함수 하나만 떼어 실행 가능하게 만든다 (launch·ament 없이)."""
    fns = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == name]
    assert len(fns) == 1, f'{name} 정의 {len(fns)}개'
    mod = ast.Module(body=fns, type_ignores=[])
    ns = {'os': os}
    exec(compile(mod, '<launch>', 'exec'), ns)
    return ns[name]


@pytest.mark.parametrize('which', sorted(LAUNCHES))
@pytest.mark.parametrize('boat', BOATS)
def test_launch_param_order_common_then_boat(which, boat):
    fn = _load_func(_tree(LAUNCHES[which]), 'yaw_mux_param_files')
    files = fn('/share', boat)
    # 🚨 순서가 의미다 — 뒤 파일이 이긴다 (10-29 Ubuntu 확인). 뒤집히면 배별 값이 공통 파일에 진다.
    assert files == [os.path.join('/share', 'config', 'ssf_heading.yaml'),
                     os.path.join('/share', 'config', f'ssf_heading_{boat}.yaml')]
    # 그 이름의 파일이 실제로 패키지에 있어야 setup.py glob 이 설치한다
    for f in files:
        assert os.path.isfile(os.path.join(CFG, os.path.basename(f)))


@pytest.mark.parametrize('which', sorted(LAUNCHES))
def test_launch_yaw_mux_uses_param_files(which):
    """yaw_mux Node 의 parameters= 가 yaw_mux_param_files(...) 결과 그대로인지."""
    tree = _tree(LAUNCHES[which])
    nodes = [n for n in ast.walk(tree)
             if isinstance(n, ast.Call) and _call_name(n) == 'Node'
             and isinstance(_kw(n, 'executable'), ast.Constant)
             and _kw(n, 'executable').value == 'yaw_mux']
    assert len(nodes) == 1, f'{which}: yaw_mux Node {len(nodes)}개'
    params = _kw(nodes[0], 'parameters')
    assert isinstance(params, ast.Name), f'{which}: parameters= 는 yaw_mux_param_files 결과 변수'
    # 그 변수가 yaw_mux_param_files(...) 호출에서 왔는지, 다시 대입되지 않는지
    assigns = [n for n in ast.walk(tree) if isinstance(n, ast.Assign)
               and any(isinstance(t, ast.Name) and t.id == params.id for t in n.targets)]
    assert len(assigns) == 1, f'{which}: {params.id} 대입 {len(assigns)}번'
    value = assigns[0].value
    assert isinstance(value, ast.Call) and _call_name(value) == 'yaw_mux_param_files'


def test_two_launch_helpers_identical():
    """두 launch 의 yaw_mux_param_files 가 어긋나면 벤치와 대회가 다른 파일을 읽는다."""
    bodies = []
    for path in LAUNCHES.values():
        fn = [n for n in _tree(path).body
              if isinstance(n, ast.FunctionDef) and n.name == 'yaw_mux_param_files'][0]
        bodies.append(ast.dump(ast.Module(body=fn.body[1:], type_ignores=[])))  # docstring 제외
    assert bodies[0] == bodies[1]
