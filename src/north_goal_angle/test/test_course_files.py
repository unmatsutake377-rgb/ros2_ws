#!/usr/bin/env python3
"""코스 분리(course = main | tournament) 테스트. python3 src/north_goal_angle/test/test_course_files.py

계약서 docs/문제와작업/계약서_토너먼트분리_20261109.md T1-3 (§9 팀장 결정 우선).
ROS 없이 돈다 — launch 파일은 ast 로 함수만 떼어 실행한다(launch_files/test_search_behavior.py 와 같은 방식).

지키는 것
  ① 코스 검증: tournament 에 mode 8 아닌 WP → WaypointError(몇 번째·무슨 mode), main 은 mode 8 포함 전부 허용,
     모르는 course → WaypointError
  ② 실제 config/waypoints_tournament.yaml 이 parse + tournament 검증 통과, main 파일은 tournament 에서 거부
  ③ launch north_goal_files: course → (공통, 코스 yaml) 순서, waypoints_file '' → 코스 기본 파일,
     모르는 코스 ValueError
  ④ launch 인자: course 기본 'main'·choices, waypoints_file 기본 '', Node parameters 맨 뒤 dict
  ⑤ 코스 yaml 키 ⊆ 노드 declare 키, 타입 = 공통 yaml 타입, 절 이름 /north_goal_angle
  ⑥ setup.py 가 config/*.yaml glob 으로 새 yaml 을 설치
  ⑦ 노드가 course 를 웨이포인트 로드보다 먼저 선언
"""

import ast
import glob
import os
import re
import sys

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from north_goal_angle.waypoint_loader import (  # noqa: E402
    COURSE_MODES, DEFAULT_COURSE, VALID_MODES, WaypointError, check_course_name,
    parse_waypoints, validate_course,
)

_PKG = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
_CFG = os.path.join(_PKG, 'config')
_LAUNCH = os.path.join(_PKG, 'launch', 'north_goal_angle_launch.py')
_NODE_SRC = os.path.join(_PKG, 'north_goal_angle', 'north_goal_angle.py')
_SETUP = os.path.join(_PKG, 'setup.py')
COURSES = ('main', 'tournament')

_p = _t = 0


def check(name, fn):
    global _p, _t
    _t += 1
    try:
        fn()
        _p += 1
        print(f"  ✅ {name}")
    except AssertionError as e:
        print(f"  ❌ {name}\n     {e}")


def _raises(fn, *a):
    try:
        fn(*a)
    except WaypointError as e:
        return str(e)
    return None


def _wps(*modes):
    return [[35.1862, 128.5655, m, 0.5] for m in modes]


def _load_wp(name):
    with open(os.path.join(_CFG, name), encoding='utf-8') as f:
        return parse_waypoints(yaml.safe_load(f))


def _params(name):
    with open(os.path.join(_CFG, name), encoding='utf-8') as f:
        raw = yaml.safe_load(f)
    assert list(raw) == ['/north_goal_angle'], f"{name}: 절 이름 {list(raw)}"
    return raw['/north_goal_angle']['ros__parameters']


def _launch_tree():
    with open(_LAUNCH, encoding='utf-8') as f:
        return ast.parse(f.read(), filename=_LAUNCH)


def _load_func(tree, name):
    """launch 파일에서 함수 하나만 떼어 실행 가능하게 만든다 (launch·ament 없이)."""
    fns = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == name]
    assert len(fns) == 1, f'{name} 정의 {len(fns)}개'
    ns = {'os': os}
    exec(compile(ast.Module(body=fns, type_ignores=[]), '<launch>', 'exec'), ns)
    return ns[name]


# ---------------- ① 코스 검증 (waypoint_loader)
def test_course_names():
    assert DEFAULT_COURSE == 'main'
    assert set(COURSE_MODES) == set(COURSES)
    assert COURSE_MODES['main'] == VALID_MODES
    assert COURSE_MODES['tournament'] == {8}
    for c in COURSES:
        assert _raises(check_course_name, c) is None, c


def test_unknown_course_rejected():
    for bad in ('Main', 'tournment', '', ' main', None, 8, 'final'):
        assert _raises(check_course_name, bad) is not None, bad
        assert _raises(validate_course, bad, _wps(8)) is not None, bad


def test_tournament_rejects_non8():
    """tournament 에 mode 8 아닌 WP 가 하나라도 있으면 거부 — 메시지에 몇 번째·무슨 mode."""
    msg = _raises(validate_course, 'tournament', _wps(8, 8, 0))
    assert msg is not None
    assert '2번째' in msg and 'mode 0' in msg, msg
    for m in VALID_MODES - {8}:
        assert _raises(validate_course, 'tournament', _wps(m)) is not None, m


def test_tournament_all8_ok():
    assert _raises(validate_course, 'tournament', _wps(8)) is None
    assert _raises(validate_course, 'tournament', _wps(8, 8, 8)) is None


def test_main_allows_all_valid_modes_including_8():
    assert _raises(validate_course, 'main', _wps(*sorted(VALID_MODES))) is None
    assert _raises(validate_course, 'main', _wps(0, 1, 2, 3, 5, 7, 8)) is None


# ---------------- ② 실제 파일
def test_tournament_template_passes():
    wps = _load_wp('waypoints_tournament.yaml')
    assert len(wps) >= 2, len(wps)
    assert all(w[2] == 8 for w in wps), [w[2] for w in wps]
    assert all(w[3] == 0.5 for w in wps), [w[3] for w in wps]   # §9-8 dwell 0.5
    assert _raises(validate_course, 'tournament', wps) is None


def test_tournament_template_marked_placeholder():
    src = open(os.path.join(_CFG, 'waypoints_tournament.yaml'), encoding='utf-8').read()
    rows = [ln for ln in src.splitlines() if ln.strip().startswith('- {')]
    assert rows and all('⚠️ 현장 레인별 녹화본으로 교체(waypoints_file:=)' in ln for ln in rows)


def test_main_file_ok_for_main_rejected_for_tournament():
    wps = _load_wp('waypoints.yaml')
    assert _raises(validate_course, 'main', wps) is None
    assert wps[-1][2] == 8, "main 마지막 WP10 은 mode 8 그대로(결정 ②)"
    assert _raises(validate_course, 'tournament', wps) is not None


# ---------------- ③ launch: course → 파일 해석
def test_north_goal_files_resolution():
    fn = _load_func(_launch_tree(), 'north_goal_files')
    share = '/s'
    for c, wp in (('main', 'waypoints.yaml'), ('tournament', 'waypoints_tournament.yaml')):
        params, path = fn(share, c, '')
        assert params == ['/s/config/north_goal_angle.yaml',
                          f'/s/config/north_goal_angle_{c}.yaml'], params  # 순서: 공통 → 코스
        assert path == f'/s/config/{wp}', path
        assert fn(share, c, '   ')[1] == f'/s/config/{wp}'
        assert fn(share, c, '/x/lane1.yaml')[1] == '/x/lane1.yaml'
        assert fn(share, c, '/x/lane1.yaml')[0] == params


def test_north_goal_files_unknown_course():
    fn = _load_func(_launch_tree(), 'north_goal_files')
    for bad in ('Main', '', 'tournment'):
        try:
            fn('/s', bad, '')
        except ValueError:
            continue
        raise AssertionError(f'{bad!r} 가 거부되지 않음')


def test_launch_files_exist_for_every_course():
    """해석 결과 파일이 소스 config 에 실제로 있다(설치는 setup.py glob)."""
    fn = _load_func(_launch_tree(), 'north_goal_files')
    for c in COURSES:
        params, wp = fn(_PKG, c, '')
        for f in params + [wp]:
            assert os.path.isfile(f), f


# ---------------- ④ launch 인자
def _calls(tree, name):
    out = []
    for n in ast.walk(tree):
        if isinstance(n, ast.Call):
            f = n.func
            if (f.attr if isinstance(f, ast.Attribute) else getattr(f, 'id', None)) == name:
                out.append(n)
    return out


def _kw(call, k):
    for kw in call.keywords:
        if kw.arg == k:
            return kw.value
    return None


def test_launch_arguments():
    tree = _launch_tree()
    ns = {}
    for n in tree.body:
        if isinstance(n, ast.Assign) and n.targets[0].id in ('COURSES', 'DEFAULT_COURSE'):
            ns[n.targets[0].id] = ast.literal_eval(n.value)
    assert ns['DEFAULT_COURSE'] == 'main', "§9-1: 단독 launch 기본 main"
    assert ns['COURSES'] == list(COURSES)
    assert set(ns['COURSES']) == set(COURSE_MODES), "launch choices ≠ 노드 허용 코스"
    args = {ast.literal_eval(c.args[0]): c for c in _calls(tree, 'DeclareLaunchArgument')}
    assert set(args) == {'course', 'waypoints_file'}, set(args)
    assert isinstance(_kw(args['course'], 'default_value'), ast.Name)    # DEFAULT_COURSE
    assert isinstance(_kw(args['course'], 'choices'), ast.Name)          # COURSES
    assert ast.literal_eval(_kw(args['waypoints_file'], 'default_value')) == ''


def test_launch_node_params_dict_last():
    src = open(_LAUNCH, encoding='utf-8').read()
    assert "parameters=params + [{'course': course, 'waypoints_file': wp}]" in src
    assert len(_calls(_launch_tree(), 'Node')) == 1


# ---------------- ⑤ 코스 yaml 키·타입
def _declared_keys():
    src = open(_NODE_SRC, encoding='utf-8').read()
    return set(re.findall(r"declare_parameter\(\s*'([a-z_]+)'", src))


def test_course_yaml_keys_declared_and_types_match():
    common = _params('north_goal_angle.yaml')
    declared = _declared_keys()
    for c in COURSES:
        p = _params(f'north_goal_angle_{c}.yaml')
        assert p, f'{c}: 빈 ros__parameters'
        for k, v in p.items():
            assert k in declared, f'{c}: {k} 를 노드가 선언 안 함(조용히 버려진다)'
            assert k in common, f'{c}: {k} 가 공통 yaml 에 없다'
            # bool/int 구분이 목적이라 isinstance 가 아니라 정확한 타입 비교
            assert type(v) is type(common[k]), f'{c}: {k} 타입 {type(v)} ≠ 공통 {type(common[k])}'  # noqa: E721


def test_course_geofence_polygon_empty_like_common():
    """지금은 둘 다 [] (현장 값) — main 은 공통과 같아 main 동작 불변."""
    assert _params('north_goal_angle.yaml')['geofence_polygon'] == []
    assert _params('north_goal_angle_main.yaml')['geofence_polygon'] == []
    assert _params('north_goal_angle_tournament.yaml')['geofence_polygon'] == []
    src = open(os.path.join(_CFG, 'north_goal_angle_tournament.yaml'), encoding='utf-8').read()
    assert '현장 2경기장 좌표, 결승선 너머까지 포함(하드 정지 2 m)' in src


def test_course_yaml_not_setting_course():
    """course 는 launch dict 로만 — yaml 에 적으면 코스 파일과 값이 엇갈릴 수 있다."""
    for name in ('north_goal_angle.yaml', 'north_goal_angle_main.yaml',
                 'north_goal_angle_tournament.yaml'):
        assert 'course' not in _params(name), name


# ---------------- ⑥ setup.py 설치
def test_setup_glob_installs_new_yaml():
    src = open(_SETUP, encoding='utf-8').read()
    assert "glob('config/*.yaml')" in src
    here = os.getcwd()
    try:
        os.chdir(_PKG)
        got = set(glob.glob('config/*.yaml'))
    finally:
        os.chdir(here)
    for f in ('waypoints_tournament.yaml', 'north_goal_angle_main.yaml',
              'north_goal_angle_tournament.yaml'):
        assert f'config/{f}' in got, f


# ---------------- ⑦ 노드 선언 순서
def test_node_declares_course_before_load():
    src = open(_NODE_SRC, encoding='utf-8').read()
    assert re.search(r"declare_parameter\('course',\s*DEFAULT_COURSE\)", src)
    i_course = src.index("declare_parameter('course'")
    assert i_course < src.index("self._load_waypoints(self.waypoints_file)")
    assert i_course < src.index("self.create_timer(self.period_sec")
    assert "validate_course(self.course, wps)" in src


def main():
    print("=== 코스 분리 테스트 ===")
    for n, f in sorted(globals().items()):
        if n.startswith("test_") and callable(f):
            check(n[5:], f)
    print(f"\n{_p}/{_t} 통과")
    return 0 if _p == _t else 1


if __name__ == "__main__":
    sys.exit(main())
