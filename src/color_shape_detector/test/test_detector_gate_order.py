#!/usr/bin/env python3
"""
검출기 3종(gate·turn·dock)의 color_callback 에서 모드 게이트가 **어떤 발행보다 먼저** 오는지
정적 검사. ROS·OpenCV 없이 돈다 — 소스 파일을 ast 로만 읽는다(import 하지 않는다).
    python3 src/color_shape_detector/test/test_detector_gate_order.py

왜 (계약서_도킹게이트_20261028 위험 1):
  순수 함수(ModeGate, dock_frame_action)만 테스트하면 노드가 그 함수를 **실제로 쓰는지**,
  **어떤 순서로** 쓰는지는 못 잡는다. a007eb0 이 dock 의 게이트 앞에 무효 목표 발행을 끼워
  넣어 모드 2·3 에서 /image_angle=10000 이 나간 것이 정확히 이 유형이다.
"""

import ast
import os
import sys

PKG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'color_shape_detector')
DETECTORS = (
    'basic_image_subscribergate.py',
    'basic_image_subscriberturn.py',
    'basic_image_subscriberdock.py',
)

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


def _color_callback(fname):
    path = os.path.join(PKG_DIR, fname)
    with open(path, encoding='utf-8') as f:
        tree = ast.parse(f.read(), filename=path)
    found = [n for n in ast.walk(tree)
             if isinstance(n, ast.FunctionDef) and n.name == 'color_callback']
    assert len(found) == 1, f"{fname}: color_callback {len(found)}개"
    return found[0]


def _pos(node):
    return (node.lineno, node.col_offset)


def _attr_calls(func, attr):
    """func 안의 `<무엇>.attr(...)` 호출 노드를 소스 순서로."""
    calls = [n for n in ast.walk(func)
             if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
             and n.func.attr == attr]
    return sorted(calls, key=_pos)


def _gate_state_calls(func):
    """`self.mode_gate.state(...)` 호출."""
    return [c for c in _attr_calls(func, 'state')
            if isinstance(c.func.value, ast.Attribute) and c.func.value.attr == 'mode_gate']


def _name_calls(func, name):
    calls = [n for n in ast.walk(func)
             if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == name]
    return sorted(calls, key=_pos)


def _assert_gate_before_publish(fname):
    cb = _color_callback(fname)
    gates = _gate_state_calls(cb)
    assert gates, f"{fname}: color_callback 에 self.mode_gate.state( 호출이 없다"
    pubs = _attr_calls(cb, 'publish')
    assert pubs, f"{fname}: color_callback 에 .publish( 가 없다 (검사가 헛돈다)"
    g, p = _pos(gates[0]), _pos(pubs[0])
    assert g < p, (f"{fname}: 첫 .publish( (줄 {p[0]}) 가 mode_gate.state( (줄 {g[0]}) "
                   f"보다 앞이다 — 담당 모드가 아닐 때도 발행된다")


def test_gate_detector_gate_before_publish():
    _assert_gate_before_publish('basic_image_subscribergate.py')


def test_turn_detector_gate_before_publish():
    _assert_gate_before_publish('basic_image_subscriberturn.py')


def test_dock_detector_gate_before_publish():
    _assert_gate_before_publish('basic_image_subscriberdock.py')


def test_dock_branches_on_dock_frame_action():
    """dock 은 dock_frame_action 결과로 분기해야 한다 — 게이트 뒤, 첫 발행 앞에서 호출."""
    fname = 'basic_image_subscriberdock.py'
    cb = _color_callback(fname)
    acts = _name_calls(cb, 'dock_frame_action')
    assert len(acts) == 1, f"{fname}: dock_frame_action 호출 {len(acts)}개 (1개여야)"
    g = _pos(_gate_state_calls(cb)[0])
    a = _pos(acts[0])
    p = _pos(_attr_calls(cb, 'publish')[0])
    assert g < a < p, (f"{fname}: 순서가 state({g[0]}) < dock_frame_action({a[0]}) "
                       f"< publish({p[0]}) 가 아니다")
    # 무효 목표 분기가 함수 결과(FRAME_*)를 비교하는지 — _target_valid 를 직접 보는 옛 분기 금지
    names = {n.id for n in ast.walk(cb) if isinstance(n, ast.Name)}
    assert 'FRAME_SKIP' in names and 'FRAME_PUBLISH_INVALID' in names, \
        f"{fname}: FRAME_SKIP / FRAME_PUBLISH_INVALID 로 분기하지 않는다"
    direct = [n for n in ast.walk(cb)
              if isinstance(n, ast.Attribute) and n.attr == '_target_valid']
    assert len(direct) == 1, (f"{fname}: color_callback 이 _target_valid 를 {len(direct)}번 "
                              f"읽는다 (dock_frame_action 인자 1번이어야)")


def test_all_detectors_covered():
    """세 검출기 파일이 모두 있다 — 이름이 바뀌면 이 검사가 조용히 빠지지 않게."""
    for f in DETECTORS:
        assert os.path.isfile(os.path.join(PKG_DIR, f)), f"{f} 없음"


def main():
    print("=== 검출기 게이트 순서 정적 검사 ===")
    for n, f in sorted(globals().items()):
        if n.startswith("test_") and callable(f):
            check(n[5:], f)
    print(f"\n{_p}/{_t} 통과")
    return 0 if _p == _t else 1


if __name__ == "__main__":
    sys.exit(main())
