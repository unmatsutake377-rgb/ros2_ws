#!/usr/bin/env python3
"""motor_control 토너먼트 yaml 정적 테스트 (계약서_토너먼트분리_20261109 T1-1).

실행: python3 src/motor_control/test/test_motor_tournament_yaml.py
      tools/run_tests.sh motor_control

ROS·pyyaml 없이 돈다 — 노드 소스는 ast, yaml 은 정규식(launch_files test_launch_params 와 같은 방식).
값 자체(1.0)는 현장에서 바뀔 수 있어 범위만 본다. 강제하는 것:
  Y1 새 yaml 이 파싱되고 절 이름이 공통 yaml 과 같다(`/motor_control:`), 키가 1개 이상
  Y2 키 ⊆ motor_control.py declare_parameter 키 (미선언 키 = 조용히 버려지는 죽은 설정)
  Y3 값 타입 = 코드 기본값 타입 = 공통 yaml 타입 (1 ↔ 1.0 오타는 노드를 부팅 때 죽인다)
  Y4 덮어쓰는 키만 — 모든 키가 공통 yaml 에도 있다
  Y5 goal_slow_min_ratio 는 코드 가드 범위 (0, 1] 안 (밖이면 코드가 조용히 0.5 로 되돌린다)
  Y6 setup.py data_files 가 새 yaml 을 share/motor_control/config 에 설치한다
"""

import ast
import math
import os
import re
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(_HERE, '..')
NODE_SRC = os.path.join(PKG, 'motor_control', 'motor_control.py')
COMMON_YAML = os.path.join(PKG, 'config', 'motor_control.yaml')
TOURN_YAML = os.path.join(PKG, 'config', 'motor_control_tournament.yaml')
SETUP_PY = os.path.join(PKG, 'setup.py')
NODE_NAME = 'motor_control'

_INT = re.compile(r'^[-+]?\d+$')
_FLOAT = re.compile(r'^[-+]?(\d+\.\d*|\.\d+|\d+\.?\d*[eE][-+]?\d+)$')


def _strip_comment(line):
    """따옴표 밖의 # 부터 지운다."""
    quote = None
    for i, ch in enumerate(line):
        if quote:
            if ch == quote:
                quote = None
        elif ch in '"\'':
            quote = ch
        elif ch == '#' and (i == 0 or line[i - 1] in ' \t'):
            return line[:i]
    return line


def _scalar(text):
    """Yaml 스칼라 하나를 Python 값으로. 모르는 꼴이면 AssertionError."""
    t = text.strip()
    if len(t) >= 2 and t[0] == t[-1] and t[0] in '"\'':
        return t[1:-1]
    if t in ('true', 'True'):
        return True
    if t in ('false', 'False'):
        return False
    if _INT.match(t):
        return int(t)
    if _FLOAT.match(t):
        return float(t)
    raise AssertionError(f'해석 못 하는 yaml 값 {text!r}')


def _yaml_sections(path):
    with open(path, encoding='utf-8') as f:
        return re.findall(r'^/([\w*]+):\s*(?:#.*)?$', f.read(), re.M)


def _yaml_params(path, node_name):
    """`/<node_name>:` 아래 `ros__parameters:` 의 key: value 를 dict 로. 못 읽는 줄은 실패."""
    out = {}
    state = 'seek'
    param_indent = None
    with open(path, encoding='utf-8') as f:
        for raw in f:
            line = _strip_comment(raw.rstrip('\n')).rstrip()
            if not line.strip():
                continue
            indent = len(line) - len(line.lstrip(' '))
            if indent == 0:
                if state in ('section', 'params'):
                    break
                if line == f'/{node_name}:':
                    state = 'section'
                continue
            if state == 'section':
                if line.strip() == 'ros__parameters:':
                    state = 'params'
                continue
            if state != 'params':
                continue
            if param_indent is None:
                param_indent = indent
            assert indent == param_indent, f'{path}: 중첩 파라미터는 지원 안 함: {raw!r}'
            m = re.match(r'^\s*([A-Za-z_]\w*):\s*(\S.*)$', line)
            assert m, f'{path}: 해석 못 하는 줄 {raw!r}'
            assert m.group(1) not in out, f'{path}: {m.group(1)} 중복'
            out[m.group(1)] = _scalar(m.group(2))
    return out


def _declared():
    """motor_control.py 의 declare_parameter('키', 기본값) 을 dict 로."""
    with open(NODE_SRC, encoding='utf-8') as f:
        tree = ast.parse(f.read(), filename=NODE_SRC)
    out = {}
    for n in ast.walk(tree):
        if (isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                and n.func.attr == 'declare_parameter'):
            assert len(n.args) >= 2 and isinstance(n.args[0], ast.Constant), (
                f'{NODE_SRC}:{n.lineno} declare_parameter 의 키·기본값을 못 읽음')
            out[n.args[0].value] = ast.literal_eval(n.args[1])
    assert out, 'declare_parameter 0개'
    return out


def _ros_type(v):
    """Rclpy 파라미터 타입 이름. bool 은 int 의 하위형이라 먼저 본다."""
    if isinstance(v, bool):
        return 'bool'
    if isinstance(v, int):
        return 'int'
    if isinstance(v, float):
        return 'float'
    if isinstance(v, str):
        return 'str'
    raise AssertionError(f'모르는 타입 {v!r}')


def _tourn():
    return _yaml_params(TOURN_YAML, NODE_NAME)


# ------------------------------------------------------------ Y1 파싱·절 이름

def test_y1_parses_with_same_section():
    assert os.path.isfile(TOURN_YAML), f'{TOURN_YAML} 없음'
    assert _yaml_sections(TOURN_YAML) == [NODE_NAME], _yaml_sections(TOURN_YAML)
    assert _yaml_sections(COMMON_YAML) == [NODE_NAME], _yaml_sections(COMMON_YAML)
    params = _tourn()
    assert params, '토너먼트 yaml 에 덮어쓰는 키 0개 — 파서가 놓쳤거나 빈 파일'


# ------------------------------------------------------------ Y2 키 ⊆ 선언

def test_y2_keys_declared_by_node():
    undeclared = sorted(set(_tourn()) - set(_declared()))
    assert not undeclared, f'motor_control.py 가 선언하지 않는 키(죽은 설정): {undeclared}'


# ------------------------------------------------------------ Y3 타입 = 코드 기본값 = 공통 yaml

def test_y3_types_match_code_and_common_yaml():
    decl = _declared()
    common = _yaml_params(COMMON_YAML, NODE_NAME)
    for k, v in _tourn().items():
        assert _ros_type(v) == _ros_type(decl[k]), (
            f'{k}: 토너먼트 {v!r}({_ros_type(v)}) ≠ 코드 기본값 {decl[k]!r}({_ros_type(decl[k])})')
        assert _ros_type(v) == _ros_type(common[k]), (
            f'{k}: 토너먼트 {v!r} ≠ 공통 yaml {common[k]!r} 타입')


def test_y3_goal_slow_min_ratio_is_float():
    v = _tourn()['goal_slow_min_ratio']
    assert _ros_type(v) == 'float', f'goal_slow_min_ratio={v!r} — 소수점 필수(1.0)'


# ------------------------------------------------------------ Y4 덮어쓰는 키만

def test_y4_only_overrides_common_keys():
    extra = sorted(set(_tourn()) - set(_yaml_params(COMMON_YAML, NODE_NAME)))
    assert not extra, f'공통 yaml 에 없는 키 {extra} — 이 파일은 덮어쓰기 전용'


# ------------------------------------------------------------ Y5 가드 범위

def test_y5_goal_slow_min_ratio_in_guard_range():
    v = _tourn()['goal_slow_min_ratio']
    assert math.isfinite(v) and 0.0 < v <= 1.0, (
        f'goal_slow_min_ratio={v} — (0, 1] 밖이면 코드가 0.5 로 되돌려 감속이 다시 켜진다')


# ------------------------------------------------------------ Y6 setup.py 설치

def _setup_data_files():
    with open(SETUP_PY, encoding='utf-8') as f:
        tree = ast.parse(f.read(), filename=SETUP_PY)
    calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
             and getattr(n.func, 'id', None) == 'setup']
    assert len(calls) == 1, f'setup() 호출 {len(calls)}개'
    kws = [k.value for k in calls[0].keywords if k.arg == 'data_files']
    assert len(kws) == 1 and isinstance(kws[0], ast.List), 'data_files 리스트를 못 찾음'
    return kws[0].elts


def _dest_is_config(node):
    """`'share/' + package_name + '/config'` 꼴인지 (상수 조각을 이어 본다)."""
    parts = []

    def walk(n):
        if isinstance(n, ast.BinOp) and isinstance(n.op, ast.Add):
            walk(n.left)
            walk(n.right)
        elif isinstance(n, ast.Constant):
            parts.append(n.value)
        elif isinstance(n, ast.Name) and n.id == 'package_name':
            parts.append(NODE_NAME)
        else:
            parts.append(None)
    walk(node)
    return None not in parts and ''.join(parts) == f'share/{NODE_NAME}/config'


def test_y6_setup_installs_tournament_yaml():
    files = []
    for e in _setup_data_files():
        if isinstance(e, ast.Tuple) and len(e.elts) == 2 and _dest_is_config(e.elts[0]):
            assert isinstance(e.elts[1], ast.List), 'config 설치 목록이 리스트가 아니다'
            files += [ast.literal_eval(x) for x in e.elts[1].elts]
    assert 'config/motor_control.yaml' in files, files
    assert 'config/motor_control_tournament.yaml' in files, (
        f'share/{NODE_NAME}/config 설치 목록에 토너먼트 yaml 없음 — launch 가 못 찾는다: {files}')


def main():
    p = t = 0
    for n, f in sorted(globals().items()):
        if n.startswith('test_') and callable(f):
            t += 1
            try:
                f()
                p += 1
                print(f'  ok   {n}')
            except AssertionError as e:
                print(f'  FAIL {n}: {e}')
    print(f'\n{p}/{t} 통과')
    return 0 if p == t else 1


if __name__ == '__main__':
    sys.exit(main())
