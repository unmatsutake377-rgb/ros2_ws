#!/usr/bin/env python3
"""ship_turn 배별 yaml(ship_turn_A/B.yaml) 정적 테스트 — ROS 없이 돈다(pyyaml + ast).

    tools/run_tests.sh ship_turn

계약서 docs/문제와작업/계약서_ship_turn배별yaml_20261115.md (승인 2026-11-15):
  ① 배별 파일 키 = 화이트리스트 9개 **정확히** (scan_* 7개 + orbit_target_deg + orbit_max_sec)
     — 다른 키가 새어 들어오면 한 배에서만 조용히 바뀐다.
  ② 배별 키 ∩ {enabled, active_wp_mode} = ∅ — mode 3 담당이 배마다 갈리면 search_behavior 검사
     (test_search_behavior·test_mode_gate)가 공통 yaml 만 봐서 못 잡는다.
  ③ A·B 키 집합이 같다 — 한 배에만 있는 키는 다른 배에서 코드 기본값으로 뜬다.
  ④ 배별 값은 전부 float(소수점) — 정수면 rclpy 타입 오류로 부팅 사망.
  ⑤ 공통 파일(ship_turn.yaml)엔 배별 키가 없다 — 있으면 배별 값에 덮여 고쳐도 반영되지 않는다.
     묶인 값(orbit_radius_m·lidar_track_max_m·approach_dist_m)은 공통 파일에 함께 있다(계약 위험 5).
  ⑥ 절 이름 '/ship_turn' 하나뿐.
"""

import ast
import os

import pytest
import yaml

PKG = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
CFG = os.path.join(PKG, 'config')
COMMON = os.path.join(CFG, 'ship_turn.yaml')
NODE = os.path.join(PKG, 'ship_turn', 'ship_turn.py')
BOATS = ('A', 'B')
# 🚨 화이트리스트 — 늘리려면 계약서부터(Davidson 승인 2: 9개로 고정)
BOAT_KEYS = {
    'scan_angle_deg', 'scan_tol_deg', 'scan_lead_sec', 'scan_stop_sec', 'scan_hold_sec',
    'scan_timeout_sec', 'scan_clear_m', 'orbit_target_deg', 'orbit_max_sec',
}
FORBIDDEN = {'enabled', 'active_wp_mode'}
COMMON_BOUND = {'orbit_radius_m', 'lidar_track_max_m', 'approach_dist_m'}


def _doc(path):
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


def _params(path):
    doc = _doc(path)
    assert isinstance(doc, dict) and set(doc) == {'/ship_turn'}, f'{path}: 절 {list(doc or {})}'
    assert set(doc['/ship_turn']) == {'ros__parameters'}
    p = doc['/ship_turn']['ros__parameters']
    assert isinstance(p, dict) and p, f'{path}: ros__parameters 가 비었다'
    return p


def _boat_file(boat):
    return os.path.join(CFG, f'ship_turn_{boat}.yaml')


def _declared():
    with open(NODE, encoding='utf-8') as f:
        tree = ast.parse(f.read())
    out = {}
    for n in ast.walk(tree):
        if (isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                and n.func.attr == 'declare_parameter'):
            out[n.args[0].value] = ast.literal_eval(n.args[1])
    return out


@pytest.mark.parametrize('boat', BOATS)
def test_boat_file_keys_are_whitelist(boat):
    keys = set(_params(_boat_file(boat)))
    assert keys == BOAT_KEYS, f'{boat}: 넘침 {sorted(keys - BOAT_KEYS)} / 빠짐 {sorted(BOAT_KEYS - keys)}'


@pytest.mark.parametrize('boat', BOATS)
def test_boat_file_has_no_mode_owner_keys(boat):
    leaked = FORBIDDEN & set(_params(_boat_file(boat)))
    assert not leaked, f'{boat}: {sorted(leaked)} 는 배별 파일 금지(mode 3 담당은 공통 + launch 가 정한다)'


def test_boat_whitelist_excludes_forbidden_and_bound():
    assert not (BOAT_KEYS & FORBIDDEN)
    assert not (BOAT_KEYS & COMMON_BOUND), '묶인 값은 공통 파일에 함께 둔다(계약 위험 5)'


def test_two_boats_same_keys():
    a, b = (set(_params(_boat_file(x))) for x in BOATS)
    assert a == b, f'A 만 {sorted(a - b)} / B 만 {sorted(b - a)}'


@pytest.mark.parametrize('boat', BOATS)
def test_boat_values_are_float(boat):
    bad = {k: v for k, v in _params(_boat_file(boat)).items() if type(v) is not float}
    assert not bad, f'{boat}: 소수점으로 (25 ❌ → 25.0 ✅) — 정수면 부팅 사망: {bad}'


def test_boat_keys_declared_as_float():
    decl = _declared()
    for k in BOAT_KEYS:
        assert k in decl, f'{k} 를 노드가 선언하지 않는다'
        assert type(decl[k]) is float, f'{k} 기본값 {decl[k]!r} 이 float 이 아니다'


def test_common_has_no_boat_keys():
    leaked = BOAT_KEYS & set(_params(COMMON))
    assert not leaked, f'공통 파일에 배별 키 {sorted(leaked)} — 배별 파일 값에 덮여 안 먹는다'


def test_common_keeps_owner_and_bound_keys():
    p = _params(COMMON)
    assert FORBIDDEN <= set(p), '공통 파일에 enabled·active_wp_mode 가 있어야 한다'
    assert COMMON_BOUND <= set(p), f'묶인 값 {sorted(COMMON_BOUND - set(p))} 가 공통 파일에 없다'
