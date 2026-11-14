#!/usr/bin/env python3
"""
yaw_mux yaml ↔ declare_parameter 정적 검사 (계약서_GPS방위_20261114 검증 계획 12). ROS 없이 돈다.

    tools/run_tests.sh ssf_heading

왜: yaw_mux 는 launch_files 의 _pkg_yaml 이 아니라서 test_launch_params.py 의 "yaml 키 ⊆ 선언 키"
    검사를 안 받는다. yaml 키 오타는 **조용히 버려지고**(코드 기본값으로 뜸), 타입이 어긋나면
    (예: float 키에 정수 37) yaw_mux 가 부팅 즉시 죽는다(10-29 실측).

지키는 것
  ① ssf_heading*.yaml 의 yaw_mux 키는 전부 yaw_mux.py 에 declare_parameter 로 있다
  ② 값 타입이 코드 기본값 타입과 같다 (bool·int·float·str 엄격 — 37 ≠ 37.0)
  ③ 공통 파일 값 = 코드 기본값 (CLAUDE.md 1-4: 코드 기본값과 yaml 을 같이 맞춘다)
"""

import ast
import glob
import os
import sys

import pytest
import yaml

PKG = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
sys.path.insert(0, PKG)

from ssf_heading import heading_logic  # noqa: E402

YAW_MUX = os.path.join(PKG, 'ssf_heading', 'yaw_mux.py')
CFG = os.path.join(PKG, 'config')
COMMON = os.path.join(CFG, 'ssf_heading.yaml')
YAMLS = sorted(glob.glob(os.path.join(CFG, 'ssf_heading*.yaml')))


def _declared():
    """yaw_mux.py 의 declare_parameter('이름', 기본값) → {이름: 기본값}."""
    with open(YAW_MUX, encoding='utf-8') as f:
        tree = ast.parse(f.read(), filename=YAW_MUX)
    out = {}
    for n in ast.walk(tree):
        if not (isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                and n.func.attr == 'declare_parameter'):
            continue
        assert len(n.args) == 2 and isinstance(n.args[0], ast.Constant), \
            f'yaw_mux.py:{n.lineno} declare_parameter 는 (이름 상수, 기본값) 로'
        name = n.args[0].value
        d = n.args[1]
        if isinstance(d, ast.Constant):
            val = d.value
        elif isinstance(d, ast.Name) and hasattr(heading_logic, d.id):
            val = getattr(heading_logic, d.id)       # 예: SRC_IMU_RELATIVE
        else:
            raise AssertionError(f'yaw_mux.py:{n.lineno} {name} 기본값을 정적으로 못 읽는다')
        assert name not in out, f'{name} 두 번 선언 (ParameterAlreadyDeclaredException)'
        out[name] = val
    return out


def _params(path):
    with open(path, encoding='utf-8') as f:
        doc = yaml.safe_load(f)
    return doc['yaw_mux']['ros__parameters']


def test_found_files():
    assert COMMON in YAMLS and len(YAMLS) >= 3, YAMLS
    assert len(_declared()) >= 20


@pytest.mark.parametrize('path', YAMLS, ids=os.path.basename)
def test_yaml_keys_are_declared_with_same_type(path):
    decl = _declared()
    for key, val in _params(path).items():
        assert key in decl, (
            f'{os.path.basename(path)}: {key} 는 yaw_mux.py 에 선언이 없다 → 조용히 버려진다')
        want = type(decl[key])
        assert type(val) is want, (
            f'{os.path.basename(path)}: {key}={val!r} ({type(val).__name__}) — 코드 기본값은 '
            f'{want.__name__}. float 면 소수점으로(37 ❌ 37.0 ✅) — 아니면 부팅 즉시 죽는다')


def test_common_yaml_equals_code_defaults():
    decl = _declared()
    for key, val in _params(COMMON).items():
        assert val == decl[key], (
            f'{key}: 공통 yaml {val!r} ≠ 코드 기본값 {decl[key]!r} (CLAUDE.md 1-4)')


def test_common_heading_source_is_gps_cog():
    """두 배 공통 기본값은 gps_cog — 공통 yaml = 코드 기본값 (보완 C, 사용자 11-15).

    C1 의 '공통은 imu_relative 유지(C2 배별 전환)' 계약을 뒤집었다. 배별 파일엔 이 키가 없다
    (test_boat_config.py). 배 하나만 되돌리기는 그 노트북 로컬 공통 yaml 한 줄(커밋 안 함, C-Q3).
    """
    assert _params(COMMON)['heading_source'] == heading_logic.SRC_GPS_COG
    assert _declared()['heading_source'] == heading_logic.SRC_GPS_COG


def _yaw_mux_class():
    with open(YAW_MUX, encoding='utf-8') as f:
        tree = ast.parse(f.read(), filename=YAW_MUX)
    return next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'YawMux')


def test_init_goal_warn_constant_and_reference():
    """C-Q5: INIT_GOAL 장기 지속 WARN — 상수 10.0 s(계약), 기준 = AUTO 진입 시각(boat_mode_cb 에서 기록).

    순수 판정(init_goal_overdue_sec)은 test_heading_logic 이 본다. 여기선 래퍼 배선만 정적으로:
    ① 클래스 상수 INIT_GOAL_WARN_AFTER_SEC = 10.0 (float)  ② boat_mode_cb 가 진입(entered)일 때
    self._auto_enter_t 에 시각을 넣는다 — 소스 조기 return 보다 먼저(재진입마다 재설정).
    """
    cls = _yaw_mux_class()
    consts = {t.id: n.value.value for n in cls.body if isinstance(n, ast.Assign)
              for t in n.targets if isinstance(t, ast.Name) and isinstance(n.value, ast.Constant)}
    assert consts.get('INIT_GOAL_WARN_AFTER_SEC') == 10.0
    assert type(consts['INIT_GOAL_WARN_AFTER_SEC']) is float

    cb = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'boat_mode_cb')
    set_line = ret_line = None
    for n in ast.walk(cb):
        if (isinstance(n, ast.If) and isinstance(n.test, ast.Name) and n.test.id == 'entered'
                and any(isinstance(s, ast.Assign) and isinstance(s.targets[0], ast.Attribute)
                        and s.targets[0].attr == '_auto_enter_t' for s in n.body)):
            set_line = n.lineno
        if isinstance(n, ast.Return) and ret_line is None:
            ret_line = n.lineno
    assert set_line is not None, 'boat_mode_cb 가 진입 때 _auto_enter_t 를 안 넣는다'
    assert ret_line is None or set_line < ret_line, '진입 시각 기록이 조기 return 뒤에 있다'
