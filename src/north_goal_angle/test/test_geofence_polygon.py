#!/usr/bin/env python3
"""geofence_polygon 선언 타입·값 검증 테스트.

실행: python3 src/north_goal_angle/test/test_geofence_polygon.py

계약서 docs/문제와작업/계약서_지오펜스폴리곤타입_20261109.md C1 (결정 1·2·3 권고안).
  · 11-09 Ubuntu S0 실측: 정적 타입 선언(기본값 [] → BYTE_ARRAY)이면 소수 폴리곤에서 부팅 사망
    → dynamic_typing 선언 + 노드 검증(sanitize_geofence_polygon)으로 바꿨다.
  · Humble dynamic_typing 실측 값: 값 없음 → [], yaml [](NOT_SET) → None, 소수 → list(float),
    정수만 → list(int), 문자열 → list(str), 단일 → float.

지키는 것
  ① 꺼짐(ERROR 없음): None·[]·()·빈 array
  ② 정상: 3점 list·tuple·array('d')·4점 → 새 float 목록
  ③ 거부(ERROR + 꺼짐): 홀수·6개 미만·NaN·inf·문자열/bytes/bool/None 원소·정수 원소·정수만·
     값 전체 문자열·숫자 하나·위도 밖·경도 밖·위경도 뒤바뀜 — 이유에 원소 위치
  ④ 노드 정적 검사: dynamic_typing 선언, 선언 직후 검증 → 그 뒤 tournament 경고,
     ERROR 는 전용 메서드에서만, `.value or []` 제거, 시작 로그 OFF 사유 구분

north_goal_angle.py 는 rclpy·geopy 를 import 하므로(Mac 에 없음) 순수 모듈만 import 하고
노드는 소스를 ast·문자열로 본다.
"""

import array
import ast
import os
import re
import sys

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from north_goal_angle import geofence_rays, waypoint_loader  # noqa: E402
from north_goal_angle.geofence_rays import sanitize_geofence_polygon  # noqa: E402

_PKG = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
_CFG = os.path.join(_PKG, 'config')
_NODE_SRC = os.path.join(_PKG, 'north_goal_angle', 'north_goal_angle.py')

# S0 에서 쓴 폴리곤(3점) — 소수, [위도, 경도] 순서
POLY3 = [35.1862, 128.5655, 35.1866, 128.5655, 35.1866, 128.5660]
POLY4 = [35.1862, 128.5655, 35.1866, 128.5655, 35.1866, 128.5660, 35.1862, 128.5660]

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


def _off(raw):
    got = sanitize_geofence_polygon(raw)
    assert got == ([], True, ''), (raw, got)


def _rejected(raw, *must_contain):
    vals, ok, reason = sanitize_geofence_polygon(raw)
    assert vals == [] and ok is False, (raw, vals, ok, reason)
    assert reason, f"이유가 비었다: {raw!r}"
    for s in must_contain:
        assert s in reason, f"{raw!r}: 이유에 {s!r} 없음 → {reason}"
    return reason


def _accepted(raw, expect):
    vals, ok, reason = sanitize_geofence_polygon(raw)
    assert ok is True and reason == '', (raw, ok, reason)
    assert vals == expect, (vals, expect)
    assert type(vals) is list, type(vals)
    assert all(type(v) is float for v in vals), vals
    assert vals is not raw, "받은 객체를 그대로 돌려준다(새 목록이어야 한다)"
    return vals


# ---------------- ① 꺼짐 (ERROR 없음)
def test_off_none_not_set():
    _off(None)               # yaml `[]` = NOT_SET → rclpy .value None (S0 실측)


def test_off_empty_list():
    _off([])                 # 값 없음 → 기본값 [] (S0 실측)


def test_off_empty_tuple():
    _off(())


def test_off_empty_array():
    _off(array.array('d'))


def test_config_yaml_values_are_off():
    """지금 yaml 세 개는 [] — 검증을 넣어도 꺼짐(ERROR 없음) 그대로."""
    for name in ('north_goal_angle.yaml', 'north_goal_angle_main.yaml',
                 'north_goal_angle_tournament.yaml'):
        with open(os.path.join(_CFG, name), encoding='utf-8') as f:
            v = yaml.safe_load(f)['/north_goal_angle']['ros__parameters']['geofence_polygon']
        _off(v)


# ---------------- ② 정상
def test_ok_three_points_list():
    _accepted(list(POLY3), POLY3)


def test_ok_three_points_tuple():
    _accepted(tuple(POLY3), POLY3)


def test_ok_three_points_array_d():
    _accepted(array.array('d', POLY3), POLY3)


def test_ok_four_points():
    _accepted(list(POLY4), POLY4)


def test_ok_returns_copy_not_alias():
    raw = list(POLY3)
    vals = _accepted(raw, POLY3)
    vals[0] = 0.0
    assert raw[0] == POLY3[0], "반환 목록을 고치면 원본이 바뀐다"


# ---------------- ③ 거부 — 개수
def test_reject_seven_values_odd():
    # 지금 코드는 7개면 마지막을 조용히 버리고 ON(3점) 이었다 → ERROR + 꺼짐
    _rejected(POLY3 + [35.1862], '7개', '홀수', '원소 6')


def test_reject_two_values():
    _rejected([35.1862, 128.5655], '2개', '3점')


def test_reject_four_values():
    _rejected(POLY3[:4], '4개', '3점')


# ---------------- ③ 거부 — 원소 값
def test_reject_nan():
    raw = list(POLY3)
    raw[3] = float('nan')
    _rejected(raw, '원소 3', 'NaN')


def test_reject_pos_inf():
    raw = list(POLY3)
    raw[4] = float('inf')
    _rejected(raw, '원소 4')


def test_reject_neg_inf():
    raw = list(POLY3)
    raw[1] = float('-inf')
    _rejected(raw, '원소 1')


def test_reject_string_element():
    raw = list(POLY3)
    raw[2] = '35.1866'
    _rejected(raw, '원소 2')


def test_reject_bytes_element():
    raw = list(POLY3)
    raw[5] = b'1'
    _rejected(raw, '원소 5')


def test_reject_all_strings():
    # STRING_ARRAY 로 와도(S0 실측) 노드가 판정
    _rejected([str(v) for v in POLY3], '원소 0')


def test_reject_bool_element():
    raw = list(POLY3)
    raw[0] = True
    _rejected(raw, '원소 0')


def test_reject_none_element():
    raw = list(POLY3)
    raw[3] = None
    _rejected(raw, '원소 3')


def test_reject_int_mixed():
    raw = list(POLY3)
    raw[2] = 35
    _rejected(raw, '원소 2', '정수', '소수')


def test_reject_int_only():
    # INTEGER_ARRAY 는 param_check 를 통과한다 → 노드 검증만 잡는다(위험 3·4, 결정 2)
    _rejected([35, 128, 35, 129, 36, 128], '원소 0', '정수')


def test_reject_int_array():
    _rejected(array.array('i', [35, 128, 35, 129, 36, 128]), '원소 0', '정수')


# ---------------- ③ 거부 — 값 전체가 목록이 아님
def test_reject_whole_string():
    _rejected('35.1862, 128.5655, 35.1866, 128.5655, 35.1866, 128.5660', '문자열')


def test_reject_whole_bytes():
    _rejected(b'\x01\x02', '문자열')


def test_reject_single_float():
    _rejected(35.5, '숫자 하나')       # `-p geofence_polygon:=35.5` → DOUBLE (S0 실측)


def test_reject_single_int():
    _rejected(35, '숫자 하나')


def test_reject_single_bool():
    _rejected(True, '숫자 하나')


def test_reject_dict():
    _rejected({'lat': 35.0}, '목록이 아니다')


# ---------------- ③ 거부 — 범위·순서
def test_reject_lat_out_of_range():
    raw = list(POLY3)
    raw[2] = 45.0            # 1번째 점 위도
    _rejected(raw, '1번째 점', '원소 2·3', '위도')


def test_reject_lon_out_of_range():
    raw = list(POLY3)
    raw[5] = 140.0           # 2번째 점 경도
    _rejected(raw, '2번째 점', '원소 4·5', '경도')


def test_reject_swapped_lat_lon():
    raw = list(POLY3)
    raw[0], raw[1] = raw[1], raw[0]
    _rejected(raw, '0번째 점', '원소 0·1', '뒤바뀌')


def test_reject_all_swapped():
    swapped = []
    for k in range(0, len(POLY3), 2):
        swapped += [POLY3[k + 1], POLY3[k]]
    _rejected(swapped, '뒤바뀌')


def test_reject_zero_placeholder():
    # 0.0 을 빈칸으로 둔 실수
    _rejected([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], '0번째 점')


def test_range_is_waypoint_loader_range():
    """범위는 waypoint_loader 상수를 그대로 쓴다(상수 공유 모듈 신설 금지, import 만)."""
    assert geofence_rays.LAT_MIN is waypoint_loader.LAT_MIN
    assert geofence_rays.LAT_MAX is waypoint_loader.LAT_MAX
    assert geofence_rays.LON_MIN is waypoint_loader.LON_MIN
    assert geofence_rays.LON_MAX is waypoint_loader.LON_MAX
    lo = [waypoint_loader.LAT_MIN, waypoint_loader.LON_MIN] * 3
    hi = [waypoint_loader.LAT_MAX, waypoint_loader.LON_MAX] * 3
    assert sanitize_geofence_polygon(lo)[1] is True      # 경계값 포함
    assert sanitize_geofence_polygon(hi)[1] is True


# ---------------- ④ 노드 정적 검사
def _src():
    return open(_NODE_SRC, encoding='utf-8').read()


def _node_methods():
    tree = ast.parse(_src())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'NorthGoalAngle')
    return {f.name: f for f in cls.body if isinstance(f, ast.FunctionDef)}


def _logger_calls(fn):
    """fn 안의 self.get_logger().<level>(...) 호출 (level, 첫 인자 소스) 목록."""
    out = []
    for n in ast.walk(fn):
        if (isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                and isinstance(n.func.value, ast.Call)
                and isinstance(n.func.value.func, ast.Attribute)
                and n.func.value.func.attr == 'get_logger'):
            out.append((n.func.attr, ast.unparse(n.args[0]) if n.args else ''))
    return out


def test_node_imports_parameter_descriptor():
    assert 'from rcl_interfaces.msg import ParameterDescriptor' in _src()


def test_node_declares_geofence_dynamic_typing():
    src = _src()
    assert re.search(
        r"declare_parameter\(\s*'geofence_polygon',\s*\[\],\s*"
        r"ParameterDescriptor\(dynamic_typing=True\)\)", src), \
        "geofence_polygon 을 ParameterDescriptor(dynamic_typing=True) 로 선언하지 않는다"
    assert src.count("'geofence_polygon'") == 1, "geofence_polygon 선언이 하나가 아니다"


def test_node_no_value_or_empty_list():
    assert '.value or []' not in _src(), "`.value or []` 가 남아 있다(검증 함수가 None 을 다룬다)"


def test_node_validates_right_after_declare_then_tournament_warn():
    src = _src()
    i_decl = src.index("'geofence_polygon', [], ParameterDescriptor(dynamic_typing=True)")
    i_san = src.index("sanitize_geofence_polygon(raw_polygon)")
    i_err = src.index("self._log_geofence_invalid(polygon_reason)")
    i_warn = src.index("self._log_tournament_no_geofence()")
    i_timer = src.index("self.create_timer(self.period_sec")
    assert i_decl < i_san < i_err < i_warn < i_timer, (i_decl, i_san, i_err, i_warn, i_timer)
    # 선언과 검증 사이에 다른 선언이 끼지 않는다(선언 직후 검증)
    assert 'declare_parameter' not in src[i_decl:i_san], "선언과 검증 사이에 다른 선언"
    # tournament 경고는 검증 뒤 실효 길이로
    assert re.search(r"if self\.course == 'tournament' and len\(self\.geofence_polygon\) < 6:\n"
                     r"\s+self\._log_tournament_no_geofence\(\)", src)


def test_node_geofence_polygon_set_only_by_sanitize():
    init = _node_methods()['__init__']
    assigns = []
    for n in ast.walk(init):
        if isinstance(n, ast.Assign):
            for t in n.targets:
                for e in ast.walk(t):
                    if (isinstance(e, ast.Attribute) and e.attr == 'geofence_polygon'
                            and isinstance(e.value, ast.Name) and e.value.id == 'self'):
                        assigns.append(n)
    assert len(assigns) == 1, f"self.geofence_polygon 대입이 {len(assigns)}곳"
    v = assigns[0].value
    assert isinstance(v, ast.Call) and getattr(v.func, 'id', '') == 'sanitize_geofence_polygon'
    others = {name for name, fn in _node_methods().items() if name != '__init__'
              for n in ast.walk(fn) if isinstance(n, (ast.Assign, ast.AugAssign))
              for e in ast.walk(n) if isinstance(e, ast.Attribute)
              and e.attr == 'geofence_polygon' and isinstance(e.ctx, ast.Store)}
    assert not others, f"__init__ 밖에서 geofence_polygon 을 바꾼다: {others}"


def test_node_geofence_error_only_in_dedicated_method():
    """§1-7: ERROR 는 전용 메서드에서 self.get_logger().error(...) 로 직접(로거 변수 재사용 금지)."""
    methods = _node_methods()
    m = methods.get('_log_geofence_invalid')
    assert m is not None, "_log_geofence_invalid 메서드가 없다"
    calls = _logger_calls(m)
    assert [lv for lv, _ in calls] == ['error'], calls
    assert 'geofence_polygon 오류' in calls[0][1], calls
    for name, fn in methods.items():
        if name == '_log_geofence_invalid':
            continue
        for lv, msg in _logger_calls(fn):
            assert 'geofence_polygon 오류' not in msg, f"{name} 가 geofence 오류 로그를 직접 낸다"
    assert 'logger = self.get_logger()' not in _src()


def test_node_startup_log_distinguishes_off_reasons():
    src = _src()
    assert 'OFF(폴리곤 미설정' in src
    assert 'OFF(폴리곤 오류 — 위 ERROR)' in src
    assert re.search(r"elif self\.geofence_polygon_ok:\n\s+gf = \"OFF\(폴리곤 미설정", src)


def main():
    print("=== geofence_polygon 선언·검증 테스트 ===")
    for n, f in sorted(globals().items()):
        if n.startswith("test_") and callable(f):
            check(n[5:], f)
    print(f"\n{_p}/{_t} 통과")
    return 0 if _p == _t else 1


if __name__ == "__main__":
    sys.exit(main())
