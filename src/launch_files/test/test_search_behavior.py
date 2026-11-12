#!/usr/bin/env python3
"""
탐색(wp_mode 3) 행동 스위치 `search_behavior:=orbit|hold` 정적 테스트
(계약서_탐색hold스위치_20261102). ROS 없이 돈다 — launch 파일에서 ast 로 함수만 떼어 실행한다.

    tools/run_tests.sh launch_files

지키는 것
  ① DeclareLaunchArgument('search_behavior') 정확히 1개, 기본값 'orbit', choices ['orbit','hold']
     — 기본 orbit = 2026-11 경기규정(탐색 = 공지 색 부표 원 선회): 3→ship_turn
       (계약서_탐색선회_부표색_20261111 C3 — 11-04 의 hold 기본을 뒤집음).
       hold 는 옵션(탐색 = 부표 앞 위치유지, 2,3→ship_back). 오타(hold 대신 holf)는 choices 가 막는다.
       ast 와 정규식 두 경로로 읽어 같은 값인지도 본다(하나가 깨져도 다른 쪽이 잡는다).
  ② orbit → ship_back [2], ship_turn enabled **is True** (Python bool — 문자열 'false' 금지)
  ③ hold  → ship_back [2, 3], ship_turn enabled **is False**
  ④ 모르는 값 → ValueError (조용히 orbit 으로 떨어지지 않는다)
  ⑤ 두 행동 모두에서 실제 /wp_mode 값 {0,1,2,3,5,7,8} 각각 /candidate_angle 담당이 **정확히 하나**
     — 둘이면 발행자 충돌(에러 없이 값이 섞임), 0이면 침묵(3-1 사고).
  ⑥ launch 가 넘기는 파라미터 키를 노드가 declare_parameter 로 선언한다
     — 미선언 키는 rclpy 가 조용히 버려 hold 가 겉으로만 켜진다(계약서 "반드시 마지막" 이유).
  ⑦ ship_turn·ship_back Node 가 launch 에 각 1개뿐이고 둘 다 parameters= 를 받는다
     — parameters 없는 맨 Node 를 누가 되살리면 스위치가 그 노드엔 안 닿는다.
"""

import ast
import os
import re

import pytest

PKG = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
SRC = os.path.dirname(PKG)
LAUNCH = os.path.join(PKG, 'launch', 'launch_files.launch.py')
BEHAVIORS = ('orbit', 'hold')
DEFAULT_BEHAVIOR = 'orbit'  # 2026-11 경기규정: 탐색 = 원 선회 (계약서_탐색선회_부표색_20261111, 승인 11-11)
# 실제 /wp_mode 값 (CLAUDE.md §3). 8 은 미션 노드 없음 = north_goal_angle 의 20000 폴백이 담당.
#   5 는 11-13 부터 ship_avoid 담당(계약서_도리도리회피_launch_20261113 C2) — north FALLBACK_MODES (8,).
WP_MODES = {0, 1, 2, 3, 5, 7, 8}
# 이 계약이 건드리지 않는 발행자 (각 노드의 active_wp_mode(s) 기본값·FALLBACK_MODES).
#   소스와 맞는지는 test_avoid_mission.py 가 본다(ship_avoid active_wp_mode·north FALLBACK_MODES).
FIXED_OWNERS = {
    'ship_gate': {0, 1},
    'ship_dock': {7},
    'ship_avoid': {5},
    'north_goal_angle': {8},
}
# launch 가 넘기는 파라미터 키 → 그 키를 선언해야 하는 노드 소스
NODE_SOURCES = {
    'ship_back': os.path.join(SRC, 'ship_back', 'ship_back', 'ship_back.py'),
    'ship_turn': os.path.join(SRC, 'ship_turn', 'ship_turn', 'ship_turn.py'),
}


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


def _load_func(tree, name):
    """launch 파일에서 함수 하나만 떼어 실행 가능하게 만든다 (launch·ament 없이)."""
    fns = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == name]
    assert len(fns) == 1, f'{name} 정의 {len(fns)}개'
    mod = ast.Module(body=fns, type_ignores=[])
    ns = {'os': os}
    exec(compile(mod, '<launch>', 'exec'), ns)
    return ns[name]


@pytest.fixture(scope='module')
def params_fn():
    return _load_func(_tree(), 'search_behavior_params')


def _owners(params):
    """행동별 파라미터 → 모드별 /candidate_angle 담당 노드 목록."""
    owners = {m: [] for m in WP_MODES}
    table = dict(FIXED_OWNERS)
    table['ship_back'] = set(params['ship_back']['active_wp_modes'])
    # ship_turn 은 active_wp_mode 3 그대로 두고 enabled 로만 끈다 (-1 센티널 금지, 계약 위험 1)
    table['ship_turn'] = {3} if params['ship_turn']['enabled'] else set()
    for node, modes in table.items():
        for m in modes:
            owners[m].append(node)
    return owners


# ------------------------------------------------------------ ①  인자 선언

def test_declares_search_behavior_arg():
    calls = [n for n in ast.walk(_tree())
             if isinstance(n, ast.Call) and _call_name(n) == 'DeclareLaunchArgument'
             and n.args and isinstance(n.args[0], ast.Constant)
             and n.args[0].value == 'search_behavior']
    assert len(calls) == 1, f'search_behavior DeclareLaunchArgument {len(calls)}개'
    call = calls[0]
    default = _kw(call, 'default_value')
    assert default is not None, '기본값 없으면 인자 없이 launch 가 거부된다 — 기본 = orbit(2026-11 규정)'
    assert ast.literal_eval(default) == DEFAULT_BEHAVIOR, '기본값은 orbit = 2026-11 경기규정(11-11 승인)'
    choices = _kw(call, 'choices')
    assert choices is not None, 'choices 없음 → 오타가 ValueError 로 launch 전체를 세운다'
    assert ast.literal_eval(choices) == ['orbit', 'hold']


def _default_by_regex():
    """launch 원문에서 search_behavior 의 default_value 를 정규식으로 읽는다 (ast 와 독립된 경로)."""
    with open(LAUNCH, encoding='utf-8') as f:
        src = f.read()
    pat = r"DeclareLaunchArgument\(\s*['\"]search_behavior['\"]\s*,\s*default_value\s*=\s*['\"]([a-z]+)['\"]"
    found = re.findall(pat, src)
    assert len(found) == 1, f'정규식으로 찾은 search_behavior default_value {found}'
    return found[0]


def test_default_is_orbit_by_regex():
    assert _default_by_regex() == DEFAULT_BEHAVIOR


def test_default_is_orbit_and_mode3_owner_is_ship_turn_alone(params_fn):
    """기본값으로 launch 하면 mode 3 /candidate_angle 담당이 ship_turn 하나다 (ship_back 은 mode 2 만)."""
    default = _default_by_regex()
    assert default == 'orbit'
    owners = _owners(params_fn(default))
    assert owners[3] == ['ship_turn'], f'기본({default}) mode 3 담당 {owners[3]}'
    assert owners[2] == ['ship_back']


# ------------------------------------------------------------ ②③④  순수 함수

def test_orbit_is_current_behavior(params_fn):
    p = params_fn('orbit')
    assert p['ship_back'] == {'active_wp_modes': [2]}
    # bool 은 int 의 하위형 — == True 는 1 도 통과한다 → is
    assert p['ship_turn']['enabled'] is True
    assert set(p) == {'ship_back', 'ship_turn'}


def test_hold_moves_mode3_to_ship_back(params_fn):
    p = params_fn('hold')
    assert p['ship_back'] == {'active_wp_modes': [2, 3]}
    assert p['ship_turn']['enabled'] is False, "문자열 'false' 금지 — bool('false') 는 True"
    assert set(p) == {'ship_back', 'ship_turn'}


@pytest.mark.parametrize('behavior', BEHAVIORS)
def test_no_empty_list_and_no_sentinel(params_fn, behavior):
    p = params_fn(behavior)
    # 빈 리스트는 rclpy 타입 추론 실패 가능(계약 위험 5), -1 은 ship_turn 부팅 초기값(위험 1)
    assert p['ship_back']['active_wp_modes'], '빈 리스트 금지'
    assert 'active_wp_mode' not in p['ship_turn'], 'ship_turn 은 enabled 로만 끈다'
    assert type(p['ship_turn']['enabled']) is bool


@pytest.mark.parametrize('bad', ['', 'ORBIT', 'Hold', 'circle', 'none'])
def test_unknown_behavior_raises(params_fn, bad):
    with pytest.raises(ValueError):
        params_fn(bad)


# ------------------------------------------------------------ ⑤  담당 표

@pytest.mark.parametrize('behavior', BEHAVIORS)
def test_every_mode_has_exactly_one_candidate_owner(params_fn, behavior):
    owners = _owners(params_fn(behavior))
    for mode in sorted(WP_MODES):
        assert len(owners[mode]) == 1, (
            f'{behavior}: mode {mode} /candidate_angle 담당 {owners[mode]} — 정확히 하나여야 한다')


def test_owner_table_by_behavior(params_fn):
    """표를 통째로 고정한다 — 누가 바뀌면 여기서 눈에 띄게."""
    orbit = {m: o[0] for m, o in _owners(params_fn('orbit')).items()}
    hold = {m: o[0] for m, o in _owners(params_fn('hold')).items()}
    assert orbit[3] == 'ship_turn' and hold[3] == 'ship_back'
    for m in WP_MODES - {3}:
        assert orbit[m] == hold[m], f'mode {m} 담당이 행동에 따라 바뀌면 안 된다'


# ------------------------------------------------------------ ⑥  노드가 그 키를 선언하는가

@pytest.mark.parametrize('behavior', BEHAVIORS)
def test_nodes_declare_passed_params(params_fn, behavior):
    for node, kv in params_fn(behavior).items():
        with open(NODE_SOURCES[node], encoding='utf-8') as f:
            src = f.read()
        for key in kv:
            pat = r"declare_parameter\(\s*['\"]" + re.escape(key) + r"['\"]"
            assert re.search(pat, src), (
                f'{node} 가 {key!r} 를 declare_parameter 하지 않는다 — '
                f'rclpy 가 조용히 버려 {behavior} 가 겉으로만 켜진다')


# ------------------------------------------------------------ ⑦  Node 가 parameters= 를 받는가

@pytest.mark.parametrize('executable', ['ship_turn', 'ship_back'])
def test_mission_node_receives_parameters(executable):
    nodes = [n for n in ast.walk(_tree())
             if isinstance(n, ast.Call) and _call_name(n) == 'Node'
             and isinstance(_kw(n, 'executable'), ast.Constant)
             and _kw(n, 'executable').value == executable]
    assert len(nodes) == 1, f'{executable} Node {len(nodes)}개 — 둘이면 하나는 스위치를 못 받는다'
    assert _kw(nodes[0], 'parameters') is not None, f'{executable}: parameters= 없음'
