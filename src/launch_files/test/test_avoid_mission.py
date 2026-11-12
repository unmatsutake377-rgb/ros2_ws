#!/usr/bin/env python3
"""
mode 5 담당을 north_goal_angle 폴백에서 ship_avoid 로 옮긴 원자 묶음 정적 테스트
(계약서_도리도리회피_launch_20261113 C2). ROS·pyyaml 없이 돈다 — launch·노드 소스는 ast, yaml 은 정규식.

    tools/run_tests.sh launch_files

지키는 것
  ① 대회 launch 에 ship_avoid Node 정확히 1개(_ship_avoid 안), package = executable = name = 'ship_avoid',
     parameters = [_pkg_yaml('ship_avoid')] — yaml 없이 뜨면 코드 기본값이 대회값이 된다
  ② ship_avoid 패키지가 없으면 _ship_avoid 가 Node 없이 LogInfo 만 낸다(launch 전체가 서지 않는다, 위험 4)
  ③ 실제 /wp_mode 값 {0,1,2,3,5,7,8} 각각 /candidate_angle 담당이 **정확히 하나**(search_behavior 두 값 모두)
     — 담당 표를 손으로 적지 않고 각 노드 yaml(없으면 선언 기본값)·north FALLBACK_MODES 소스에서 읽는다.
       mode 5 가 north 와 ship_avoid 둘이면 에러 없이 값이 섞인다. 0이면 침묵.
  ④ north FALLBACK_MODES == (8,), COURSE_END_STOP_MODES == (5, 8) — 코스 끝 정지 집합은 따로(위험 6)
  ⑤ ssf_tools(healthcheck) 의 mode 5 담당 = ship_avoid — launch 와 같은 커밋이어야 거짓 경보가 없다
"""

import ast
import os
import re

import pytest

PKG = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
SRC = os.path.dirname(PKG)
LAUNCH = os.path.join(PKG, 'launch', 'launch_files.launch.py')
NORTH_SRC = os.path.join(SRC, 'north_goal_angle', 'north_goal_angle', 'north_goal_angle.py')
TOOLS_YAML = os.path.join(SRC, 'ssf_tools', 'config', 'ssf_tools.yaml')
WP_MODES = (0, 1, 2, 3, 5, 7, 8)          # 실제 /wp_mode 값 (CLAUDE.md §3)
BEHAVIORS = ('orbit', 'hold')

# 모드 담당 노드 → 담당 모드를 정하는 파라미터 (이름, 리스트인가). ship_back·ship_turn 은 search_behavior 가 덮는다.
OWNER_PARAMS = {
    'ship_gate': ('active_wp_modes', True),
    'ship_dock': ('active_wp_mode', False),
    'ship_avoid': ('active_wp_mode', False),
    'ship_back': ('active_wp_modes', True),
    'ship_turn': ('active_wp_mode', False),
}


# ------------------------------------------------------------ ast 도우미

def _tree(path=LAUNCH):
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


def _const_kw(call, name):
    v = _kw(call, name)
    return v.value if isinstance(v, ast.Constant) else None


def _func(tree, name):
    fns = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == name]
    assert len(fns) == 1, f'{name} 정의 {len(fns)}개'
    return fns[0]


def _nodes(tree, executable):
    return [n for n in ast.walk(tree)
            if isinstance(n, ast.Call) and _call_name(n) == 'Node'
            and _const_kw(n, 'executable') == executable]


def _declared(path):
    """노드 소스의 declare_parameter('키', 리터럴) → {키: 기본값}."""
    out = {}
    for n in ast.walk(_tree(path)):
        if (isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                and n.func.attr == 'declare_parameter' and len(n.args) >= 2
                and isinstance(n.args[0], ast.Constant)):
            try:
                out[n.args[0].value] = ast.literal_eval(n.args[1])
            except ValueError:
                pass
    return out


def _yaml_value(path, key):
    """`    key: 값` 한 줄(주석 제외) → int 또는 int 리스트. 없으면 None."""
    with open(path, encoding='utf-8') as f:
        m = re.search(r'^\s+%s:\s*([^#\n]+)' % re.escape(key), f.read(), re.M)
    if not m:
        return None
    t = m.group(1).strip()
    if t.startswith('['):
        return [int(x) for x in t[1:-1].split(',') if x.strip()]
    return int(t)


def _node_tuple(name):
    src = open(NORTH_SRC, encoding='utf-8').read()
    m = re.search(r'^%s = \(([0-9, ]+)\)' % name, src, re.M)
    assert m, f'north_goal_angle.py 에 {name} = (...) 정의가 없다'
    return tuple(int(v) for v in m.group(1).split(',') if v.strip())


# ------------------------------------------------------------ ① Node 1개 + yaml

def test_ship_avoid_node_once_with_pkg_yaml():
    nodes = _nodes(_tree(), 'ship_avoid')
    assert len(nodes) == 1, f'ship_avoid Node {len(nodes)}개 — 둘이면 mode 5 발행자 2개, 0이면 north 폴백도 없어 담당 0'
    node = nodes[0]
    assert _const_kw(node, 'package') == 'ship_avoid'
    assert _const_kw(node, 'name') == 'ship_avoid', '이름이 다르면 ship_avoid.yaml /ship_avoid 절이 안 닿는다'
    params = _kw(node, 'parameters')
    assert isinstance(params, ast.List) and len(params.elts) == 1, 'parameters=[_pkg_yaml(\'ship_avoid\')] 이어야 한다'
    first = params.elts[0]
    assert (isinstance(first, ast.Call) and _call_name(first) == '_pkg_yaml' and len(first.args) == 1
            and isinstance(first.args[0], ast.Constant) and first.args[0].value == 'ship_avoid'), (
        "parameters 첫 항목이 _pkg_yaml('ship_avoid') 가 아니다")
    assert len(_nodes(_func(_tree(), '_ship_avoid'), 'ship_avoid')) == 1, 'ship_avoid Node 가 _ship_avoid 밖에 있다'


# ------------------------------------------------------------ ② 패키지 없으면 건너뜀

class _Node:
    def __init__(self, **kw):
        self.kw = kw


class _LogInfo:
    def __init__(self, msg):
        self.msg = msg


class _NotFound(Exception):
    pass


def _run_ship_avoid(missing=()):
    """launch 의 _pkg_yaml·_ship_avoid 를 떼어 가짜 의존성으로 실행 (진짜 _pkg_yaml 이 가짜 share 를 부른다)."""
    tree = _tree()
    fns = [_func(tree, '_pkg_yaml'), _func(tree, '_ship_avoid')]

    def share(pkg):
        if pkg in missing:
            raise _NotFound(pkg)
        return f'/install/{pkg}/share/{pkg}'

    ns = {'os': os, 'Node': _Node, 'LogInfo': _LogInfo,
          'PackageNotFoundError': _NotFound, 'get_package_share_directory': share}
    exec(compile(ast.Module(body=fns, type_ignores=[]), '<launch>', 'exec'), ns)
    out = ns['_ship_avoid'](None)
    return [a for a in out if isinstance(a, _Node)], [a for a in out if isinstance(a, _LogInfo)]


def test_ship_avoid_present_gives_one_node():
    nodes, _ = _run_ship_avoid()
    assert len(nodes) == 1
    kw = nodes[0].kw
    assert kw['package'] == kw['executable'] == kw['name'] == 'ship_avoid'
    assert kw['parameters'] == ['/install/ship_avoid/share/ship_avoid/config/ship_avoid.yaml']


def test_ship_avoid_missing_package_skips():
    """위험 4: 빌드에서 빠져도 launch 전체가 죽지 않는다 — 예외 없이 LogInfo 만."""
    nodes, logs = _run_ship_avoid(missing=('ship_avoid',))
    assert nodes == [], '패키지가 없는데 Node 를 냈다'
    assert logs and '⏭️' in logs[0].msg and 'ship_avoid' in logs[0].msg


def test_ship_avoid_takes_no_launch_arg():
    """스위치는 yaml avoid_mode(앱에서 바꿈, 결정 2) — launch 인자로 만들지 않는다(위험 7: 앱 버튼은 boat·course 만)."""
    fn = _func(_tree(), '_ship_avoid')
    lcs = [n for n in ast.walk(fn) if isinstance(n, ast.Call) and _call_name(n) == 'LaunchConfiguration']
    assert lcs == [], 'ship_avoid 가 launch 인자를 읽는다 — 앱 launch 버튼에서 못 바꾼다'


# ------------------------------------------------------------ ③ 모드별 담당 정확히 하나

def _launch_value(pkg, key, list_):
    """launch 가 넘기는 yaml 값(있으면) — 없으면 노드 선언 기본값. 둘 다 없으면 실패."""
    v = _yaml_value(os.path.join(SRC, pkg, 'config', f'{pkg}.yaml'), key)
    if v is None:
        v = _declared(os.path.join(SRC, pkg, pkg, f'{pkg}.py')).get(key)
    assert v is not None, f'{pkg}.{key} 값을 못 읽음'
    return set(v) if list_ else {v}


def _search_params(behavior):
    fn = _func(_tree(), 'search_behavior_params')
    ns = {}
    exec(compile(ast.Module(body=[fn], type_ignores=[]), '<launch>', 'exec'), ns)
    return ns['search_behavior_params'](behavior)


def _owners(behavior):
    table = {pkg: _launch_value(pkg, key, list_) for pkg, (key, list_) in OWNER_PARAMS.items()}
    sb = _search_params(behavior)
    table['ship_back'] = set(sb['ship_back']['active_wp_modes'])      # dict 가 yaml 뒤라 이긴다
    if not sb['ship_turn']['enabled']:
        table['ship_turn'] = set()
    table['north_goal_angle'] = set(_node_tuple('FALLBACK_MODES'))
    owners = {m: [] for m in WP_MODES}
    for node, modes in table.items():
        for m in modes:
            if m in owners:
                owners[m].append(node)
    return owners


@pytest.mark.parametrize('behavior', BEHAVIORS)
def test_every_mode_has_exactly_one_owner(behavior):
    owners = _owners(behavior)
    for m in WP_MODES:
        assert len(owners[m]) == 1, f'{behavior}: mode {m} /candidate_angle 담당 {owners[m]} — 정확히 하나여야 한다'


@pytest.mark.parametrize('behavior', BEHAVIORS)
def test_mode5_owner_is_ship_avoid_and_8_is_north(behavior):
    owners = _owners(behavior)
    assert owners[5] == ['ship_avoid'], owners[5]
    assert owners[8] == ['north_goal_angle'], owners[8]


def test_ship_avoid_yaml_equals_code_default_mode():
    """active_wp_mode: yaml = 선언 기본값 = 5 (yaml 이 안 닿는 단독 실행에서도 같은 모드)."""
    src = os.path.join(SRC, 'ship_avoid', 'ship_avoid', 'ship_avoid.py')
    assert _declared(src).get('active_wp_mode') == 5
    assert _yaml_value(os.path.join(SRC, 'ship_avoid', 'config', 'ship_avoid.yaml'), 'active_wp_mode') == 5


# ------------------------------------------------------------ ④ north 상수

def test_north_fallback_only_mode8():
    assert _node_tuple('FALLBACK_MODES') == (8,), 'mode 5 를 north 폴백에 되살리면 ship_avoid 와 발행자 2개'


def test_north_fallback_publish_uses_constant():
    """폴백 20000 발행 판정이 FALLBACK_MODES 상수를 쓰는지(코스 끝 집합이나 (5, 8) 리터럴로 바뀌면 mode 5 발행자 2개)."""
    src = open(NORTH_SRC, encoding='utf-8').read()
    assert 'if wp_mode in FALLBACK_MODES:' in src
    assert 'if wp_mode in COURSE_END_STOP_MODES:' not in src
    assert 'if wp_mode in (5, 8):' not in src


def test_north_course_end_stop_modes_kept():
    """위험 6: 폴백에서 5 를 빼도 코스 끝 50000 은 마지막 mode 5·8 모두 그대로."""
    assert _node_tuple('COURSE_END_STOP_MODES') == (5, 8)
    src = open(NORTH_SRC, encoding='utf-8').read()
    assert 'candidate_after_course(self.waypoints[-1][2], COURSE_END_STOP_MODES)' in src


# ------------------------------------------------------------ ⑤ healthcheck 매핑

def test_healthcheck_mode5_owner_matches_launch():
    text = open(TOOLS_YAML, encoding='utf-8').read()
    modes = re.search(r'^\s+wp_modes:\s*\[([^\]]*)\]', text, re.M)
    nodes = re.search(r'^\s+wp_nodes:\s*\[([^\]]*)\]', text, re.M)
    assert modes and nodes, 'ssf_tools.yaml 에 wp_modes / wp_nodes 가 없다'
    table = dict(zip((int(x) for x in modes.group(1).split(',')),
                     (x.strip().strip('"\'') for x in nodes.group(1).split(','))))
    assert table[5] == 'ship_avoid', f'healthcheck mode 5 담당 {table[5]!r} — launch 의 ship_avoid 와 다르다'
    assert table[8] == 'none'
