#!/usr/bin/env python3
"""
대회 launch 가 미션·제어 노드와 blackbox 에 config/<pkg>.yaml 을 넘기는지 정적 테스트
(계약서_launch_yaml_blackbox_20261105). ROS·pyyaml 없이 돈다 — launch·노드 소스는 ast, yaml 은 정규식.

    tools/run_tests.sh launch_files

왜: 11-05 부터 yaml 이 대회값이다. 위험의 성격이 바뀌었다 —
  · yaml 타입 오타(3.0 ↔ 3)는 그 노드를 부팅 때 죽인다(rclpy InvalidParameterTypeException).
  · 노드가 선언 안 한 키는 에러 없이 버려진다(노드들은 allow_undeclared 미사용).
  · ship_back·ship_turn 은 parameters 순서가 틀리면 mode 3 발행자가 2개가 된다(에러 없음).

지키는 것 (값은 비교하지 않는다 — 결정 ②: yaml 이 이긴다, 키·타입만 강제)
  T1 노드별 Node 정확히 1개 + parameters 첫 항목이 _pkg_yaml('<pkg>') = share/<pkg>/config/<pkg>.yaml
     T1b OpaqueFunction 진입점(_yaw_mux·_course_check·_oak_camera·_d455_depth·_avoid_detector·_perception_fusion·
         _search_nodes·_ship_avoid·_control_nodes·_blackbox)이 launch 목록에 각 1개
     T1~T4·T6 은 perception_fusion(계약서_인식융합_20261112)도 대조한다 — 미션 노드는 아니지만 같은 _pkg_yaml 패턴
     T1~T4·T6 은 ship_avoid(계약서_도리도리회피_launch_20261113 C2)도 대조한다 — _ship_avoid 가 띄운다
  T2 yaml 절 이름 = launch name= = 노드 super().__init__ 인자 (어긋나면 yaml 이 통째로 안 닿는다)
     T2~T4 는 코스 yaml(<pkg>_tournament.yaml)도 대조한다 (계약서_토너먼트분리_20261109 T1-4)
  T3 yaml 키 ⊆ 노드 declare_parameter 키 (미선언 키 = 조용히 버려지는 죽은 설정)
  T4 yaml 값 타입 = 선언 기본값 타입 (bool/int/float/str, 리스트는 원소 타입까지). yaml 빈 리스트 금지
  T5 ship_back·ship_turn parameters = [yaml, search_behavior dict] — dict 가 **맨 뒤**
  T6 setup.py 가 config/<pkg>.yaml 을 share 에 설치한다 (코스 패키지는 <pkg>_tournament.yaml 도)
  T7 blackbox: Node 1개, condition=IfCondition(record), ssf_tools.yaml, record 기본 'true'·choices,
     ssf_tools 없으면 건너뜀(PackageNotFoundError), healthcheck·mission_monitor 는 없음(결정 ④)
  T8 코스(계약서_토너먼트분리_20261109 T1-4): course 필수(choices main/tournament, 기본값 없음),
     waypoints_file 기본 '', north Include 에 둘 다 전달, ship_direction parameters =
     [_pkg_yaml, *course_param_files, {'dwa_hull': boat}] (dict 맨 뒤), motor =
     [_pkg_yaml, *course_param_files], course_param_files: main → [], tournament → 설치 경로,
     없으면 RuntimeError, 모르는 course 는 ValueError, _course_check 가 목록 첫 OpaqueFunction
"""

import ast
import os
import re

import pytest

PKG = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
SRC = os.path.dirname(PKG)
LAUNCH = os.path.join(PKG, 'launch', 'launch_files.launch.py')

# 대회 launch 에서 yaml 을 받는 노드: 패키지 = 실행파일 = 노드 이름 = yaml 절 이름
MISSION_PKGS = ('ship_direction', 'motor_control', 'ship_gate', 'ship_dock',
                'ship_back', 'ship_turn')
# yaml 을 _pkg_yaml 로 받는 노드 전체 = 미션·제어 + perception_fusion(계약서_인식융합_20261112, 패키지 = 실행파일 =
#   노드 이름 = yaml 절 이름). perception_fusion Node 는 _perception_fusion(perception_fusion:=true 일 때)이 띄운다.
#   ship_avoid(계약서_도리도리회피_launch_20261113 C2, mode 5 담당) 도 같은 규칙 — Node 는 _ship_avoid 가 띄운다.
YAML_PKGS = MISSION_PKGS + ('perception_fusion', 'ship_avoid')
SEARCH_PKGS = ('ship_back', 'ship_turn')     # search_behavior dict 를 맨 뒤에 더 받는다
# blackbox 는 패키지(ssf_tools) ≠ 노드 이름(blackbox)
BLACKBOX = {'package': 'ssf_tools', 'executable': 'blackbox', 'name': 'blackbox',
            'yaml': os.path.join(SRC, 'ssf_tools', 'config', 'ssf_tools.yaml'),
            'source': os.path.join(SRC, 'ssf_tools', 'ssf_tools', 'blackbox.py')}


# course:=tournament 때 공통 yaml 뒤에 <pkg>_tournament.yaml 을 받는 패키지 (= launch COURSE_PKGS)
COURSE_PKGS = ('ship_direction', 'motor_control')
COURSES = ['main', 'tournament']


def _tournament_yaml(pkg):
    return os.path.join(SRC, pkg, 'config', f'{pkg}_tournament.yaml')


def _node_cases():
    """(노드 이름, yaml 경로, 노드 소스) — T2·T3·T4 공통. 코스 yaml 도 같은 노드 절로 대조한다."""
    cases = [(p, os.path.join(SRC, p, 'config', f'{p}.yaml'),
              os.path.join(SRC, p, p, f'{p}.py')) for p in YAML_PKGS]
    cases += [(p, _tournament_yaml(p), os.path.join(SRC, p, p, f'{p}.py')) for p in COURSE_PKGS]
    cases.append((BLACKBOX['name'], BLACKBOX['yaml'], BLACKBOX['source']))
    return cases


NODE_CASES = _node_cases()
NODE_IDS = [os.path.splitext(os.path.basename(c[1]))[0] for c in NODE_CASES]   # 파일 이름


# ------------------------------------------------------------ ast 도우미

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


def _const_kw(call, name):
    v = _kw(call, name)
    return v.value if isinstance(v, ast.Constant) else None


def _nodes(tree, executable):
    return [n for n in ast.walk(tree)
            if isinstance(n, ast.Call) and _call_name(n) == 'Node'
            and _const_kw(n, 'executable') == executable]


def _func(tree, name):
    fns = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == name]
    assert len(fns) == 1, f'{name} 정의 {len(fns)}개'
    return fns[0]


def _is_pkg_yaml_call(node, pkg):
    return (isinstance(node, ast.Call) and _call_name(node) == '_pkg_yaml'
            and len(node.args) == 1 and isinstance(node.args[0], ast.Constant)
            and node.args[0].value == pkg)


def _params_list(node):
    params = _kw(node, 'parameters')
    assert isinstance(params, ast.List), (
        f'{_const_kw(node, "executable")}: parameters= 가 리스트 리터럴이 아니다 — '
        'yaml 없이 뜨면 코드 기본값이 대회값이 된다')
    return params.elts


# ------------------------------------------------------------ yaml (pyyaml 없이)

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
    """yaml 스칼라 하나 → Python 값. 모르는 꼴이면 AssertionError (조용히 str 로 넘기지 않는다)."""
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
    assert re.match(r'^[A-Za-z_/][\w/.\-]*$', t), f'해석 못 하는 yaml 값 {text!r}'
    return t


def _value(text):
    t = text.strip()
    if t.startswith('['):
        assert t.endswith(']'), f'한 줄 흐름 리스트만 지원: {text!r}'
        inner = t[1:-1].strip()
        return [_scalar(x) for x in inner.split(',')] if inner else []
    return _scalar(t)


def _yaml_section(path, node_name):
    """`/<node_name>:` → `ros__parameters:` 아래 key: value 를 {key: 값} 으로.

    파라미터 들여쓰기 줄 중 해석 못 하는 줄이 있으면 실패한다 — 파서가 조용히 키를 놓쳐
    T3·T4 가 빈 집합으로 통과하는 것을 막는다.
    """
    out = {}
    state = 'seek'          # seek → section → params
    param_indent = None
    with open(path, encoding='utf-8') as f:
        for raw in f:
            line = _strip_comment(raw.rstrip('\n')).rstrip()
            if not line.strip():
                continue
            indent = len(line) - len(line.lstrip(' '))
            if indent == 0:
                if state == 'params' or state == 'section':
                    break                       # 다음 절
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
            key = m.group(1)
            assert key not in out, f'{path}: {key} 중복'
            out[key] = _value(m.group(2))
    return out


def _yaml_sections(path):
    with open(path, encoding='utf-8') as f:
        return re.findall(r'^/([\w*]+):\s*(?:#.*)?$', f.read(), re.M)


# ------------------------------------------------------------ 노드 소스 (ast)

def _source_tree(path):
    with open(path, encoding='utf-8') as f:
        return ast.parse(f.read(), filename=path)


def _declared(path):
    """노드 소스의 declare_parameter('키', 기본값) → {키: 기본값}. 기본값은 리터럴이어야 한다."""
    out = {}
    for n in ast.walk(_source_tree(path)):
        if (isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                and n.func.attr == 'declare_parameter'):
            assert len(n.args) >= 2 and isinstance(n.args[0], ast.Constant), (
                f'{path}:{n.lineno} declare_parameter 의 키·기본값을 못 읽음')
            out[n.args[0].value] = ast.literal_eval(n.args[1])
    assert out, f'{path}: declare_parameter 0개'
    return out


def _super_init_name(path):
    names = []
    for n in ast.walk(_source_tree(path)):
        if (isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                and n.func.attr == '__init__' and isinstance(n.func.value, ast.Call)
                and _call_name(n.func.value) == 'super'
                and n.args and isinstance(n.args[0], ast.Constant)):
            names.append(n.args[0].value)
    assert len(names) == 1, f'{path}: super().__init__(이름) {names}'
    return names[0]


def _ros_type(v):
    """rclpy 파라미터 타입 이름. bool 은 int 의 하위형이라 먼저 본다."""
    if isinstance(v, bool):
        return 'bool'
    if isinstance(v, int):
        return 'int'
    if isinstance(v, float):
        return 'float'
    if isinstance(v, str):
        return 'str'
    if isinstance(v, list):
        kinds = {_ros_type(x) for x in v}
        assert len(kinds) <= 1, f'원소 타입이 섞인 리스트 {v!r} — rclpy 가 거부한다'
        return f'list[{kinds.pop() if kinds else "?"}]'
    raise AssertionError(f'모르는 타입 {v!r}')


# ------------------------------------------------------------ T1  Node 1개 + yaml 경로

@pytest.mark.parametrize('pkg', YAML_PKGS)
def test_t1_node_once_with_pkg_yaml_first(pkg):
    nodes = _nodes(_tree(), pkg)
    assert len(nodes) == 1, f'{pkg} Node {len(nodes)}개 — 둘이면 발행자 2개, 0이면 침묵'
    node = nodes[0]
    assert _const_kw(node, 'package') == pkg
    assert _const_kw(node, 'name') == pkg
    elts = _params_list(node)
    assert elts and _is_pkg_yaml_call(elts[0], pkg), (
        f"{pkg}: parameters 첫 항목이 _pkg_yaml('{pkg}') 가 아니다")


def test_t1_pkg_yaml_builds_share_config_path():
    """_pkg_yaml 을 떼어 실행: <share>/config/<pkg>.yaml (get_package_share_directory 는 가짜)."""
    fn = _func(_tree(), '_pkg_yaml')
    ns = {'os': os, 'get_package_share_directory': lambda p: f'/install/{p}/share/{p}'}
    exec(compile(ast.Module(body=[fn], type_ignores=[]), '<launch>', 'exec'), ns)
    for pkg in YAML_PKGS + ('ssf_tools',):
        assert ns['_pkg_yaml'](pkg) == f'/install/{pkg}/share/{pkg}/config/{pkg}.yaml'


@pytest.mark.parametrize('pkg', YAML_PKGS)
def test_t1_source_yaml_exists(pkg):
    assert os.path.isfile(os.path.join(SRC, pkg, 'config', f'{pkg}.yaml'))


# ------------------------------------------------------------ T2  절 이름 = name = 노드 이름

@pytest.mark.parametrize('name,yaml_path,source', NODE_CASES, ids=NODE_IDS)
def test_t2_section_matches_node_name(name, yaml_path, source):
    assert name in _yaml_sections(yaml_path), (
        f'{yaml_path} 에 /{name} 절이 없다 — 절 이름이 노드 이름과 다르면 yaml 이 통째로 안 닿는다')
    assert _super_init_name(source) == name, f'{source}: super().__init__ 이름 ≠ {name}'
    executable = BLACKBOX['executable'] if name == BLACKBOX['name'] else name
    nodes = _nodes(_tree(), executable)
    assert len(nodes) == 1
    assert _const_kw(nodes[0], 'name') == name, f'launch name= ≠ {name}'


# ------------------------------------------------------------ T3  yaml 키 ⊆ 선언 키

@pytest.mark.parametrize('name,yaml_path,source', NODE_CASES, ids=NODE_IDS)
def test_t3_yaml_keys_are_declared(name, yaml_path, source):
    keys = _yaml_section(yaml_path, name)
    assert keys, f'{yaml_path}: /{name} 절에서 키를 못 읽음(파서 확인)'
    undeclared = sorted(set(keys) - set(_declared(source)))
    assert not undeclared, (
        f'{name}: yaml 키 {undeclared} 를 노드가 declare_parameter 하지 않는다 — '
        'rclpy 가 에러 없이 버린다(오타 또는 죽은 설정)')


# ------------------------------------------------------------ T4  타입 일치 (값은 안 본다)

@pytest.mark.parametrize('name,yaml_path,source', NODE_CASES, ids=NODE_IDS)
def test_t4_yaml_types_match_declared(name, yaml_path, source):
    declared = _declared(source)
    bad = []
    for key, val in _yaml_section(yaml_path, name).items():
        if key not in declared:
            continue                            # T3 가 잡는다
        if val == []:
            # 빈 리스트는 rcl yaml 파서가 원소 타입을 못 정한다(부팅 실패 가능) — 금지
            bad.append(f'{key}: yaml 빈 리스트 [] — 원소 타입 미정, 빈 리스트 금지')
            continue
        want, got = _ros_type(declared[key]), _ros_type(val)
        if want == 'list[?]':
            continue                            # 기본값이 빈 리스트면 원소 타입을 모른다
        if want != got:
            bad.append(f'{key}: yaml {val!r}({got}) ≠ 기본값 {declared[key]!r}({want})')
    assert not bad, (f'{name}: 타입 불일치·빈 리스트 → 부팅 때 노드가 죽을 수 있다\n  '
                     + '\n  '.join(bad))


# ------------------------------------------------------------ T1b  진입점이 목록에 들어 있는가

# OpaqueFunction 으로 노드를 띄우는 진입점. 함수가 있어도 목록에서 빠지면 그 노드는 안 뜬다
# (에러 없음) — T1·T5·T7 은 함수 안의 Node 만 보므로 따로 검사한다.
OPAQUE_ENTRIES = ('_yaw_mux', '_course_check', '_oak_camera', '_d455_depth', '_avoid_detector', '_perception_fusion',
                  '_search_nodes', '_ship_avoid', '_control_nodes', '_blackbox')


def _launch_list():
    """generate_launch_description 이 return 하는 LaunchDescription([...]) 의 직접 항목."""
    fn = _func(_tree(), 'generate_launch_description')
    rets = [n for n in ast.walk(fn) if isinstance(n, ast.Return)]
    assert len(rets) == 1, f'return {len(rets)}개'
    call = rets[0].value
    assert isinstance(call, ast.Call) and _call_name(call) == 'LaunchDescription'
    assert len(call.args) == 1 and isinstance(call.args[0], ast.List)
    return call.args[0].elts


def _opaque_targets():
    out = []
    for e in _launch_list():
        if isinstance(e, ast.Call) and _call_name(e) == 'OpaqueFunction':
            fn = _kw(e, 'function')
            out.append(fn.id if isinstance(fn, ast.Name) else None)
    return out


@pytest.mark.parametrize('entry', OPAQUE_ENTRIES)
def test_t1b_opaque_entry_in_launch_list_once(entry):
    n = _opaque_targets().count(entry)
    assert n == 1, (f'OpaqueFunction(function={entry}) 가 launch 목록에 {n}개 — '
                    '0이면 그 노드들이 조용히 안 뜨고, 2면 발행자가 2개')


def test_t1b_every_context_function_is_listed():
    """(context) 를 받는 모듈 함수 = 진입점. 새로 생기면 OPAQUE_ENTRIES 와 목록에 같이 넣는다."""
    ctx_fns = sorted(n.name for n in _tree().body if isinstance(n, ast.FunctionDef)
                     and [a.arg for a in n.args.args] == ['context'])
    assert ctx_fns == sorted(OPAQUE_ENTRIES), f'진입점 함수 {ctx_fns}'
    assert sorted(t for t in _opaque_targets()) == sorted(OPAQUE_ENTRIES)


# ------------------------------------------------------------ T5  [yaml, dict] — dict 맨 뒤

@pytest.mark.parametrize('pkg', SEARCH_PKGS)
def test_t5_search_dict_is_last(pkg):
    elts = _params_list(_nodes(_tree(), pkg)[0])
    assert len(elts) == 2, f'{pkg}: parameters 항목 {len(elts)}개 — [yaml, search_behavior dict]'
    assert _is_pkg_yaml_call(elts[0], pkg), f'{pkg}: 첫 항목이 yaml 이 아니다'
    last = elts[-1]
    assert (isinstance(last, ast.Subscript) and isinstance(last.value, ast.Name)
            and last.value.id == 'params' and isinstance(last.slice, ast.Constant)
            and last.slice.value == pkg), (
        f"{pkg}: 마지막 항목이 params['{pkg}'](search_behavior dict) 가 아니다 — "
        'yaml 이 뒤면 orbit 에서 mode 3 발행자 2개(에러 없음)')


def test_t5_dict_comes_from_search_behavior_params():
    """_search_nodes 안의 params 가 search_behavior_params(...) 결과다 (다른 dict 로 바뀌지 않게)."""
    fn = _func(_tree(), '_search_nodes')
    srcs = [n.value for n in ast.walk(fn) if isinstance(n, ast.Assign)
            and any(isinstance(t, ast.Name) and t.id == 'params' for t in n.targets)]
    assert len(srcs) == 1 and isinstance(srcs[0], ast.Call)
    assert _call_name(srcs[0]) == 'search_behavior_params'


# ------------------------------------------------------------ T6  setup.py 가 yaml 설치

@pytest.mark.parametrize('pkg', YAML_PKGS + ('ssf_tools',))
def test_t6_setup_installs_yaml(pkg):
    with open(os.path.join(SRC, pkg, 'setup.py'), encoding='utf-8') as f:
        setup = f.read()
    # share/<pkg>/config 로 가는 항목이 config/<pkg>.yaml 또는 config/*.yaml 을 담는다
    assert re.search(r"""['"]config/(\*|""" + re.escape(pkg) + r""")\.yaml['"]""", setup), (
        f'{pkg}/setup.py 가 config/{pkg}.yaml 을 설치하지 않는다 — share 에 없으면 launch 가 죽는다')
    assert re.search(r"""/config['"]|['"]config['"]\)""", setup), (
        f'{pkg}/setup.py 의 data_files 에 share/{pkg}/config 대상이 없다')


@pytest.mark.parametrize('pkg', COURSE_PKGS)
def test_t6_setup_installs_tournament_yaml(pkg):
    """계약서 위험 1: setup.py 가 파일 이름을 직접 적는 패키지라 새 yaml 은 따로 적어야 설치된다."""
    with open(os.path.join(SRC, pkg, 'setup.py'), encoding='utf-8') as f:
        setup = f.read()
    assert os.path.isfile(_tournament_yaml(pkg)), f'{_tournament_yaml(pkg)} 없음'
    assert re.search(r"""['"]config/(\*|""" + re.escape(pkg) + r"""_tournament)\.yaml['"]""",
                     setup), (
        f'{pkg}/setup.py 가 config/{pkg}_tournament.yaml 을 설치하지 않는다 — '
        'course:=tournament 가 launch 시작에서 RuntimeError')


# ------------------------------------------------------------ T7  blackbox

def test_t7_blackbox_node_once_with_condition_and_yaml():
    tree = _tree()
    nodes = _nodes(tree, BLACKBOX['executable'])
    assert len(nodes) == 1, f'blackbox Node {len(nodes)}개 — 2개면 같은 초 파일명으로 CSV 덮어씀'
    node = nodes[0]
    assert _const_kw(node, 'package') == BLACKBOX['package']
    assert _const_kw(node, 'name') == BLACKBOX['name']
    cond = _kw(node, 'condition')
    assert isinstance(cond, ast.Call) and _call_name(cond) == 'IfCondition', (
        'condition= IfCondition 없음')
    arg = cond.args[0] if cond.args else None
    assert (isinstance(arg, ast.Call) and _call_name(arg) == 'LaunchConfiguration'
            and isinstance(arg.args[0], ast.Constant) and arg.args[0].value == 'record'), (
        "IfCondition(LaunchConfiguration('record')) 가 아니다")
    # parameters=[cfg], cfg = _pkg_yaml('ssf_tools') (try 안에서)
    elts = _params_list(node)
    assert len(elts) == 1 and isinstance(elts[0], ast.Name)
    fn = _func(tree, '_blackbox')
    assigns = [n for n in ast.walk(fn) if isinstance(n, ast.Assign)
               and any(isinstance(t, ast.Name) and t.id == elts[0].id for t in n.targets)]
    assert len(assigns) == 1 and _is_pkg_yaml_call(assigns[0].value, BLACKBOX['package']), (
        "blackbox parameters 가 _pkg_yaml('ssf_tools') 가 아니다")


def test_t7_blackbox_skips_when_package_missing():
    """ssf_tools 가 없어도 launch 전체가 서지 않는다: _pkg_yaml 조회를 try 로 감싸 LogInfo 를 낸다."""
    fn = _func(_tree(), '_blackbox')
    tries = [n for n in ast.walk(fn) if isinstance(n, ast.Try)]
    assert len(tries) == 1
    t = tries[0]
    assert any(_is_pkg_yaml_call(n, 'ssf_tools') for b in t.body for n in ast.walk(b))
    handlers = [h for h in t.handlers
                if isinstance(h.type, ast.Name) and h.type.id == 'PackageNotFoundError']
    assert len(handlers) == 1, 'PackageNotFoundError 를 잡지 않는다'
    calls = [_call_name(n) for b in handlers[0].body for n in ast.walk(b)
             if isinstance(n, ast.Call)]
    assert 'LogInfo' in calls, '건너뛸 때 LogInfo 를 안 낸다(조용히 기록이 빠진다)'
    assert 'Node' not in calls


def test_t7_record_arg_default_true():
    calls = [n for n in ast.walk(_tree())
             if isinstance(n, ast.Call) and _call_name(n) == 'DeclareLaunchArgument'
             and n.args and isinstance(n.args[0], ast.Constant) and n.args[0].value == 'record']
    assert len(calls) == 1, f"DeclareLaunchArgument('record') {len(calls)}개"
    assert _const_kw(calls[0], 'default_value') == 'true', 'record 기본값은 true(결정 ③)'
    assert ast.literal_eval(_kw(calls[0], 'choices')) == ['true', 'false']


def test_t7_record_default_by_regex():
    """ast 와 독립된 경로: 원문에서 record 기본값을 정규식으로."""
    with open(LAUNCH, encoding='utf-8') as f:
        src = f.read()
    found = re.findall(
        r"DeclareLaunchArgument\(\s*['\"]record['\"]\s*,\s*default_value\s*=\s*['\"](\w+)['\"]",
        src)
    assert found == ['true']


@pytest.mark.parametrize('executable', ['healthcheck', 'mission_monitor'])
def test_t7_no_healthcheck_or_monitor(executable):
    """결정 ④: 대회 launch 에 넣지 않는다 (/health_ok 발행자 중복 방지)."""
    assert _nodes(_tree(), executable) == []


def test_t7_blackbox_yaml_section_has_no_healthcheck_mixup():
    """blackbox 절만 읽는다 — 같은 파일의 /healthcheck 키가 섞이지 않게(파서 확인)."""
    keys = _yaml_section(BLACKBOX['yaml'], 'blackbox')
    assert 'rate_hz' in keys and 'health_period_sec' not in keys


# ------------------------------------------------------------ T8  코스 (course·waypoints_file)

def _declare(tree, name):
    calls = [n for n in ast.walk(tree)
             if isinstance(n, ast.Call) and _call_name(n) == 'DeclareLaunchArgument'
             and n.args and isinstance(n.args[0], ast.Constant) and n.args[0].value == name]
    assert len(calls) == 1, f"DeclareLaunchArgument('{name}') {len(calls)}개"
    return calls[0]


def _is_launch_config(node, name):
    return (isinstance(node, ast.Call) and _call_name(node) == 'LaunchConfiguration'
            and len(node.args) == 1 and isinstance(node.args[0], ast.Constant)
            and node.args[0].value == name)


def test_t8_course_arg_required_with_choices():
    """기본값이 있으면 토너먼트 날 인자를 빠뜨려도 main 값으로 조용히 뜬다(결정 ③ — boat 와 같다)."""
    call = _declare(_tree(), 'course')
    assert _kw(call, 'default_value') is None, 'course 는 기본값 없음(필수)'
    assert ast.literal_eval(_kw(call, 'choices')) == COURSES


def test_t8_waypoints_file_arg_default_empty():
    """비면 north launch 가 코스 기본 파일을 고른다(waypoints.yaml / waypoints_tournament.yaml)."""
    call = _declare(_tree(), 'waypoints_file')
    assert _const_kw(call, 'default_value') == ''
    assert _kw(call, 'choices') is None


def test_t8_courses_match_north_launch():
    """대회 launch choices = north_goal_angle_launch.COURSES, north 도 두 인자를 선언한다."""
    path = os.path.join(SRC, 'north_goal_angle', 'launch', 'north_goal_angle_launch.py')
    north = _source_tree(path)
    vals = [ast.literal_eval(n.value) for n in north.body if isinstance(n, ast.Assign)
            and any(isinstance(t, ast.Name) and t.id == 'COURSES' for t in n.targets)]
    assert vals == [COURSES], f'north COURSES {vals}'
    for name in ('course', 'waypoints_file'):
        _declare(north, name)


def test_t8_north_include_passes_course_and_waypoints_file():
    """안 넘기면 north 단독 기본값(course=main)으로 떠 토너먼트에서도 main 웨이포인트를 쓴다."""
    incs = [n for n in ast.walk(_tree())
            if isinstance(n, ast.Call) and _call_name(n) == 'IncludeLaunchDescription'
            and any(isinstance(c, ast.Constant) and c.value == 'north_goal_angle_launch.py'
                    for c in ast.walk(n))]
    assert len(incs) == 1, f'north Include {len(incs)}개'
    la = _kw(incs[0], 'launch_arguments')
    assert (isinstance(la, ast.Call) and isinstance(la.func, ast.Attribute)
            and la.func.attr == 'items' and isinstance(la.func.value, ast.Dict)), (
        'launch_arguments={...}.items() 꼴이 아니다')
    d = la.func.value
    got = {k.value: v for k, v in zip(d.keys, d.values) if isinstance(k, ast.Constant)}
    assert sorted(got) == ['course', 'waypoints_file'], f'north 로 넘기는 인자 {sorted(got)}'
    for name, v in got.items():
        assert _is_launch_config(v, name), f"{name} 가 LaunchConfiguration('{name}') 가 아니다"


def _is_course_files_starred(node, pkg):
    """*course_param_files('<pkg>', course)"""
    if not isinstance(node, ast.Starred):
        return False
    c = node.value
    return (isinstance(c, ast.Call) and _call_name(c) == 'course_param_files'
            and len(c.args) == 2 and isinstance(c.args[0], ast.Constant)
            and c.args[0].value == pkg
            and isinstance(c.args[1], ast.Name) and c.args[1].id == 'course')


def _performed(fn, var, arg):
    """fn 안에서 var = LaunchConfiguration('<arg>').perform(context) 가 정확히 한 번."""
    vals = [n.value for n in ast.walk(fn) if isinstance(n, ast.Assign)
            and any(isinstance(t, ast.Name) and t.id == var for t in n.targets)]
    assert len(vals) == 1, f'{fn.name}: {var} 대입 {len(vals)}개'
    v = vals[0]
    assert (isinstance(v, ast.Call) and isinstance(v.func, ast.Attribute)
            and v.func.attr == 'perform' and _is_launch_config(v.func.value, arg)), (
        f"{fn.name}: {var} 가 LaunchConfiguration('{arg}').perform(context) 가 아니다")


def test_t8_ship_direction_params_order():
    """🚨 뒤가 이긴다: 공통 yaml → 코스 yaml → dwa_hull dict **맨 뒤**(§9-5, main·tournament 모두).

    dwa_hull 은 공통 yaml 에 'A' 로 있고(T3·T4 는 키·타입만 봄 — 값은 안 봄) 여기 dict 가 boat 로
    덮는다. 그래서 T3·T4 와 충돌하지 않는다. dict 가 yaml 앞이면 B배도 'A' 로 뜬다(에러 없음).
    """
    nodes = _nodes(_tree(), 'ship_direction')
    assert len(nodes) == 1
    elts = _params_list(nodes[0])
    assert len(elts) == 3, f'ship_direction parameters 항목 {len(elts)}개'
    assert _is_pkg_yaml_call(elts[0], 'ship_direction')
    assert _is_course_files_starred(elts[1], 'ship_direction'), (
        "두 번째가 *course_param_files('ship_direction', course) 가 아니다")
    last = elts[2]
    assert isinstance(last, ast.Dict) and len(last.keys) == 1, '마지막이 dict 1키가 아니다'
    assert isinstance(last.keys[0], ast.Constant) and last.keys[0].value == 'dwa_hull'
    assert isinstance(last.values[0], ast.Name) and last.values[0].id == 'boat'
    fn = _func(_tree(), '_control_nodes')
    _performed(fn, 'boat', 'boat')
    _performed(fn, 'course', 'course')


def test_t8_motor_control_params_order():
    elts = _params_list(_nodes(_tree(), 'motor_control')[0])
    assert len(elts) == 2, f'motor_control parameters 항목 {len(elts)}개'
    assert _is_pkg_yaml_call(elts[0], 'motor_control')
    assert _is_course_files_starred(elts[1], 'motor_control'), (
        "두 번째가 *course_param_files('motor_control', course) 가 아니다")


def test_t8_course_nodes_live_in_control_nodes():
    """ship_direction·motor_control Node 는 _control_nodes 안에만 있다 (course 를 perform 하는 곳)."""
    fn = _func(_tree(), '_control_nodes')
    for pkg in COURSE_PKGS:
        assert len(_nodes(fn, pkg)) == 1, f'{pkg} Node 가 _control_nodes 밖에 있다'


def test_t8_dwa_hull_declared_and_boat_values_are_hulls():
    """dict 키 dwa_hull 을 노드가 str 로 선언하고, boat choices = DWA 선체 표 키(A·B)."""
    src = os.path.join(SRC, 'ship_direction', 'ship_direction', 'ship_direction.py')
    assert _ros_type(_declared(src)['dwa_hull']) == 'str'
    boat = ast.literal_eval(_kw(_declare(_tree(), 'boat'), 'choices'))
    dwa = _source_tree(os.path.join(SRC, 'ship_direction', 'ship_direction', 'dwa.py'))
    hulls = [n.value for n in dwa.body if isinstance(n, ast.Assign)
             and any(isinstance(t, ast.Name) and t.id == 'HULLS' for t in n.targets)]
    assert len(hulls) == 1 and isinstance(hulls[0], ast.Dict)
    assert sorted(k.value for k in hulls[0].keys) == sorted(boat)


def _course_param_files(share):
    """launch 의 course_param_files 를 떼어 실행 (get_package_share_directory 는 share(pkg))."""
    fn = _func(_tree(), 'course_param_files')
    ns = {'os': os, 'get_package_share_directory': share}
    exec(compile(ast.Module(body=[fn], type_ignores=[]), '<launch>', 'exec'), ns)
    return ns['course_param_files']


@pytest.mark.parametrize('pkg', COURSE_PKGS)
def test_t8_course_param_files_main_is_empty(pkg):
    """main 은 공통 yaml 그대로 — main 동작 불변."""
    calls = []
    fn = _course_param_files(lambda p: calls.append(p) or '/nonexistent')
    assert fn(pkg, 'main') == []
    assert calls == [], 'main 에서 share 를 조회할 이유가 없다'


@pytest.mark.parametrize('pkg', COURSE_PKGS)
def test_t8_course_param_files_tournament_path(pkg):
    # 소스 패키지 폴더를 share 로 둔다 — setup.py 가 config/ 를 share/<pkg>/config 로 그대로 옮긴다
    fn = _course_param_files(lambda p: os.path.join(SRC, p))
    got = fn(pkg, 'tournament')
    assert got == [_tournament_yaml(pkg)]
    assert os.path.isfile(got[0])
    # 설치 경로 꼴: <share>/config/<pkg>_tournament.yaml (파일 확인은 실패해도 경로는 이 꼴)
    fake = _course_param_files(lambda p: f'/install/{p}/share/{p}')
    with pytest.raises(RuntimeError, match=re.escape(
            f'/install/{pkg}/share/{pkg}/config/{pkg}_tournament.yaml')):
        fake(pkg, 'tournament')


def test_t8_course_param_files_missing_raises():
    """코스 파일이 없는 패키지(설치 누락과 같은 상황) → RuntimeError, 조용히 main 값으로 안 뜬다."""
    fn = _course_param_files(lambda p: os.path.join(SRC, p))
    assert not os.path.isfile(_tournament_yaml('ship_gate'))
    with pytest.raises(RuntimeError):
        fn('ship_gate', 'tournament')


@pytest.mark.parametrize('course', ['', 'Main', 'TOURNAMENT', 'tourney', 'main '])
def test_t8_course_param_files_unknown_course_raises(course):
    fn = _course_param_files(lambda p: os.path.join(SRC, p))
    with pytest.raises(ValueError):
        fn('ship_direction', course)


def test_t8_launch_course_pkgs_match():
    tree = _tree()
    vals = [ast.literal_eval(n.value) for n in tree.body if isinstance(n, ast.Assign)
            and any(isinstance(t, ast.Name) and t.id == 'COURSE_PKGS' for t in n.targets)]
    assert vals == [COURSE_PKGS], f'launch COURSE_PKGS {vals}'


def test_t8_course_check_before_any_node():
    """_course_check = launch 목록의 첫 OpaqueFunction, 첫 Node·Include·TimerAction 보다 앞.

    launch 는 깊이 우선 방문이다 — 뒤에 두면 앞 OpaqueFunction(_yaw_mux)의 Node 가 먼저 예약된다.
    코스 파일이 없으면 아무 노드도 안 뜬 채 멈춰야 한다. _course_check 자신은 Node 를 안 만든다.
    """
    items = _launch_list()
    calls = [(i, _call_name(e)) for i, e in enumerate(items) if isinstance(e, ast.Call)]
    opaque = [i for i, name in calls if name == 'OpaqueFunction']
    assert opaque, 'OpaqueFunction 없음'
    first = items[opaque[0]]
    assert _kw(first, 'function').id == '_course_check', (
        f"첫 OpaqueFunction 이 {_kw(first, 'function').id} — _course_check 여야 한다")
    others = [i for i, name in calls
              if name in ('Node', 'IncludeLaunchDescription', 'TimerAction', 'OpaqueFunction')
              and i != opaque[0]]
    assert opaque[0] < min(others), '_course_check 앞에 Node·Include·TimerAction·OpaqueFunction 이 있다'
    fn = _func(_tree(), '_course_check')
    _performed(fn, 'course', 'course')
    names = [_call_name(n) for n in ast.walk(fn) if isinstance(n, ast.Call)]
    assert 'course_param_files' in names and 'LogInfo' in names
    assert 'Node' not in names
