#!/usr/bin/env python3
"""
검출기 3종 — /wp_mode **값이 바뀌면** 유예(last_valid)·확정 창을 비운다(계약서_검출기모드리셋_20261106, 발견 7).
cv2·ROS 없이 돈다: 순수 함수 + 노드 파일 AST 정적 검사 + 시뮬 Det 클래스 소스 실행.
    python3 src/color_shape_detector/test/test_detector_mode_reset.py

왜 (발견 7):
  2(white)→3(red) 전환 뒤에도 turn 검출기의 fallback 이 유예 2 s 안이면 **흰 부표의 각도·색**을 그대로
  /image_angle 로 냈다 → ship_back 이 mode 3 에서 흰 부표 앞에 HOLD. 리셋 트리거는 반드시 **값 변화**다 —
  north 가 0.5 s 마다 같은 모드를 재발행하므로 "수신마다" 비우면 유예가 사실상 사라진다(위험 1).
  노드 파일은 `import cv2` 라 Mac 에서 import 가 안 된다 → 콜백이 helper 를 **실제로 부르는지**는 AST 로 본다(위험 5).
"""

import ast
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))

from color_shape_detector.mode_gate import ModeGate, mode_changed  # noqa: E402
from color_shape_detector.dock_logic import DetectionConfirmer, fallback_value  # noqa: E402

PKG_DIR = os.path.join(HERE, '..', 'color_shape_detector')
SIM_PATH = os.path.join(HERE, '..', '..', '..', 'tools', 'sim', 'full_course_sim.py')
DETECTORS = (
    'basic_image_subscriberturn.py',
    'basic_image_subscriberdock.py',
    'basic_image_subscribergate.py',
)
INVALID = 10000.0      # 노드·시뮬의 센티널과 같은 값(상수 공유 모듈화 금지 — 여기선 검증용 사본)
GRACE = 2.0

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


# ================================================================ mode_changed 경계
def test_mode_changed_boundaries():
    assert mode_changed(2, 3) is True, "2→3 은 변화"
    assert mode_changed(3, 3) is False, "3→3(north 0.5 s 반복)은 변화 아님 — 리셋하면 유예가 사라진다"
    assert mode_changed(None, 2) is True, "부팅 뒤 첫 수신(None→2)은 변화"
    assert mode_changed(0, 0) is False
    assert mode_changed(5, 7) is True


def test_update_failure_does_not_change_mode():
    """정수 변환 실패 → update False + mode 불변 → 호출자는 mode_changed 까지 가지 않는다."""
    g = ModeGate([2, 3])
    assert g.update(2, 0.0) is True
    prev = g.mode
    assert g.update("abc", 0.1) is False
    assert g.mode == prev == 2
    assert mode_changed(prev, g.mode) is False


def test_update_return_meaning_unchanged():
    g = ModeGate([2, 3])
    assert g.update(3, 0.0) is True and g.mode == 3
    assert g.update(None, 0.0) is False


# ================================================================ DetectionConfirmer.reset
def test_confirmer_reset_requires_n_again():
    c = DetectionConfirmer(confirm_frames=3)
    for _ in range(3):
        r = c.update("white")
    assert r == "white" and c.confirmed == "white"
    c.reset()
    assert c.confirmed is None, "reset 뒤 확정 없음"
    assert c.update("red") is None, "리셋 뒤 1 프레임째 — 미확정"
    assert c.update("red") is None, "리셋 뒤 2 프레임째 — 미확정"
    assert c.update("red") == "red", "리셋 뒤 3 프레임째 — 확정"


def test_confirmer_reset_same_key_restarts_count():
    """같은 키라도 리셋 뒤에는 처음부터 다시 센다(창이 비었으므로)."""
    c = DetectionConfirmer(confirm_frames=3, confirm_window=5)
    for _ in range(3):
        c.update("red")
    c.reset()
    assert c.update("red") is None
    assert c.update("red") is None
    assert c.update("red") == "red"


def test_confirmer_reset_n1_is_immediate():
    c = DetectionConfirmer(confirm_frames=1)
    assert c.update("white") == "white"
    c.reset()
    assert c.confirmed is None
    assert c.update("red") == "red", "N=1 은 리셋 뒤 첫 프레임에 바로 확정"


def test_confirmer_reset_is_idempotent_on_fresh():
    c = DetectionConfirmer(confirm_frames=3)
    c.reset()
    assert c.confirmed is None and c.update(None) is None


# ================================================================ fallback_value (turn 유예 판정)
def _fresh_last_valid():
    """노드 wp_mode_callback 의 리셋이 만드는 값과 같은 형태."""
    return {'angle': None, 'color': None, 'time': 0.0}


def test_fallback_value_basic():
    lv = {'angle': 12.5, 'color': 'white', 'time': 100.0}
    assert fallback_value(lv, 101.0, GRACE) == (12.5, 'white')
    assert fallback_value(lv, 102.0, GRACE) == (12.5, 'white'), "경계 2.0 s 는 포함(<=)"
    assert fallback_value(lv, 102.01, GRACE) is None, "유예 지나면 None → 10000"
    assert fallback_value(_fresh_last_valid(), 101.0, GRACE) is None, "유효값 없음(time 0) → None"
    assert fallback_value({'angle': None, 'color': 'white', 'time': 100.0}, 101.0, GRACE) is None


def _node_like_mode_rx(gate, last_valid, confirmer, value, t):
    """노드 wp_mode_callback 과 같은 순서로 흉내 낸다 — prev 는 update 전, 변화 때만 리셋."""
    prev = gate.mode
    if not gate.update(value, t):
        return last_valid, False
    if not mode_changed(prev, gate.mode):
        return last_valid, False
    confirmer.reset()
    return _fresh_last_valid(), True


def test_mode2_confirmed_then_mode3_rx_then_miss_is_invalid():
    """모드 2 에서 흰 부표 확정 → /wp_mode 3 수신(리셋) → 미검출 프레임 → 10000(유예 안 탐)."""
    gate = ModeGate([2, 3])
    conf = DetectionConfirmer(confirm_frames=1)
    lv, _ = _node_like_mode_rx(gate, _fresh_last_valid(), conf, 2, 100.0)
    assert conf.update("white") == "white"
    lv = {'angle': 8.0, 'color': 'white', 'time': 100.3}       # 확정 프레임이 적은 값
    assert fallback_value(lv, 100.4, GRACE) == (8.0, 'white'), "모드 2 안에서는 유예가 산다"
    lv, did = _node_like_mode_rx(gate, lv, conf, 3, 100.5)
    assert did, "2→3 은 리셋"
    assert conf.confirmed is None
    assert fallback_value(lv, 100.6, GRACE) is None, \
        "리셋 뒤 미검출 → None(10000). 이전엔 흰 부표 8.0° 가 2 s 더 나갔다(발견 7)"


def test_mode3_repeat_keeps_grace():
    """3→3 반복 수신(north 0.5 s) → 리셋 없음 → 미검출이어도 2 s 유예 유지."""
    gate = ModeGate([2, 3])
    conf = DetectionConfirmer(confirm_frames=1)
    lv, _ = _node_like_mode_rx(gate, _fresh_last_valid(), conf, 3, 200.0)
    conf.update("red")
    lv = {'angle': -5.0, 'color': 'red', 'time': 200.2}
    for t in (200.5, 201.0, 201.5, 202.0):
        lv, did = _node_like_mode_rx(gate, lv, conf, 3, t)
        assert not did, f"같은 값 반복({t}) 에 리셋하면 안 된다"
    assert conf.confirmed == "red"
    assert fallback_value(lv, 202.1, GRACE) == (-5.0, 'red'), "유예 2 s 안 → 마지막 유효값 유지"
    assert fallback_value(lv, 202.3, GRACE) is None, "2 s 지나면 10000"


def test_non_int_mode_rx_does_not_reset():
    gate = ModeGate([2, 3])
    conf = DetectionConfirmer(confirm_frames=1)
    lv, _ = _node_like_mode_rx(gate, _fresh_last_valid(), conf, 3, 0.0)
    conf.update("red")
    lv = {'angle': 1.0, 'color': 'red', 'time': 0.1}
    lv, did = _node_like_mode_rx(gate, lv, conf, "x", 0.2)
    assert not did and conf.confirmed == "red" and lv['angle'] == 1.0


# ================================================================ AST 정적 검사 — 노드 3개
def _parse(path):
    with open(path, encoding='utf-8') as f:
        return ast.parse(f.read(), filename=path)


def _func(tree, name, fname):
    found = [n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name]
    assert len(found) == 1, f"{fname}: {name} {len(found)}개"
    return found[0]


def _calls_named(func, name):
    return [n for n in ast.walk(func)
            if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == name]


def _attr_calls(func, attr):
    return [n for n in ast.walk(func)
            if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
            and n.func.attr == attr]


def _is_self_attr(node, attr):
    return (isinstance(node, ast.Attribute) and node.attr == attr
            and isinstance(node.value, ast.Name) and node.value.id == 'self')


def _assign_nodes(func, attr):
    """`self.<attr> = …` 또는 `self.<attr>[…] = …` 대입 노드들."""
    out = []
    for n in ast.walk(func):
        if not isinstance(n, ast.Assign):
            continue
        for tg in n.targets:
            if _is_self_attr(tg, attr) or \
                    (isinstance(tg, ast.Subscript) and _is_self_attr(tg.value, attr)):
                out.append(n)
                break
    return out


def _assigns_self_attr(func, attr):
    return bool(_assign_nodes(func, attr))


def _reads_self_attr_chain(func, chain):
    """`self.a.b` 읽기 노드들(소스 위치) — chain=('mode_gate','mode')."""
    out = []
    for n in ast.walk(func):
        if isinstance(n, ast.Attribute) and n.attr == chain[1] \
                and _is_self_attr(n.value, chain[0]) and isinstance(n.ctx, ast.Load):
            out.append((n.lineno, n.col_offset))
    return sorted(out)


def check_wp_mode_callback(path):
    """한 노드 파일의 wp_mode_callback 이 계약대로인지. 결함 주입 스크립트도 이 함수를 쓴다."""
    fname = os.path.basename(path)
    tree = _parse(path)
    # import
    imported = any(isinstance(n, ast.ImportFrom) and n.module == 'color_shape_detector.mode_gate'
                   and any(a.name == 'mode_changed' for a in n.names) for n in ast.walk(tree))
    assert imported, f"{fname}: mode_changed 를 color_shape_detector.mode_gate 에서 import 하지 않음"

    cb = _func(tree, 'wp_mode_callback', fname)
    mc = _calls_named(cb, 'mode_changed')
    assert mc, f"{fname}: wp_mode_callback 이 mode_changed() 를 부르지 않음 — 수신마다 리셋이면 유예가 사라진다"
    upd = [c for c in _attr_calls(cb, 'update') if _is_self_attr(c.func.value, 'mode_gate')]
    assert upd, f"{fname}: wp_mode_callback 에 self.mode_gate.update 호출 없음"
    reads = _reads_self_attr_chain(cb, ('mode_gate', 'mode'))
    u_pos = (upd[0].lineno, upd[0].col_offset)
    assert reads and reads[0] < u_pos, \
        f"{fname}: prev 는 update **전** 의 self.mode_gate.mode 여야 한다(읽기 {reads}, update {u_pos})"
    assert _assigns_self_attr(cb, 'last_valid'), f"{fname}: wp_mode_callback 이 last_valid 를 비우지 않음"
    assert _attr_calls(cb, 'reset'), f"{fname}: wp_mode_callback 이 confirmer.reset() 을 부르지 않음"
    assert _assigns_self_attr(cb, 'found_in_frame'), f"{fname}: found_in_frame 을 초기화하지 않음"
    pubs = _attr_calls(cb, 'publish')
    assert not pubs, \
        f"{fname}: wp_mode_callback 에 .publish( {len(pubs)}곳 — 발행은 color_callback 게이트 뒤에서만"
    infos = _attr_calls(cb, 'info')
    assert infos, f"{fname}: 리셋 INFO 1회가 없다"
    for c in infos:
        assert isinstance(c.func.value, ast.Call) \
            and isinstance(c.func.value.func, ast.Attribute) \
            and c.func.value.func.attr == 'get_logger', \
            f"{fname}: get_logger().info 를 직접 호출해야 한다(§1-7 — 로거를 변수에 담아 재사용 금지)"
    # 리셋 블록은 mode_changed 가드에 **지배**되어야 한다(가드가 로그만 감싸고 리셋은 무조건이면 결함).
    #   early-return 형(`if not mode_changed(...): return` 뒤에 리셋) 또는 if-본문 형(리셋이 body 안).
    ifs = [n for n in ast.walk(cb)
           if isinstance(n, ast.If) and any(m is c for c in ast.walk(n.test) for m in mc)]
    assert ifs, f"{fname}: mode_changed() 가 if 조건에 쓰이지 않는다"
    gi = ifs[0]
    resets = _assign_nodes(cb, 'last_valid') + _attr_calls(cb, 'reset')
    body_nodes = [x for b in gi.body for x in ast.walk(b)]
    if any(isinstance(x, ast.Return) for x in body_nodes):
        bad = [r.lineno for r in resets if r.lineno <= gi.lineno]
        assert not bad, \
            f"{fname}: early-return 가드(:{gi.lineno}) 앞에서 리셋(:{bad}) — mode_changed 와 무관하게 실행된다"
    else:
        bad = [r.lineno for r in resets if not any(r is x for x in body_nodes)]
        assert not bad, f"{fname}: 리셋(:{bad})이 mode_changed if(:{gi.lineno}) 본문 밖 — 무조건 실행된다"
    return True


def test_ast_turn_callback():
    check_wp_mode_callback(os.path.join(PKG_DIR, 'basic_image_subscriberturn.py'))


def test_ast_dock_callback():
    check_wp_mode_callback(os.path.join(PKG_DIR, 'basic_image_subscriberdock.py'))


def test_ast_gate_callback():
    check_wp_mode_callback(os.path.join(PKG_DIR, 'basic_image_subscribergate.py'))


def check_turn_uses_fallback_value(path):
    fname = os.path.basename(path)
    tree = _parse(path)
    cc = _func(tree, 'color_callback', fname)
    assert _calls_named(cc, 'fallback_value'), \
        f"{fname}: color_callback 이 fallback_value() 를 쓰지 않음(결정 ⑤)"
    return True


def test_ast_turn_color_callback_uses_fallback_value():
    check_turn_uses_fallback_value(os.path.join(PKG_DIR, 'basic_image_subscriberturn.py'))


def test_ast_gate_reset_covers_both_colors():
    """gate 는 red·green 둘 다 비운다 — last_valid 대입에 두 키가 있고 confirm 은 values() 순회 또는 둘 다."""
    path = os.path.join(PKG_DIR, 'basic_image_subscribergate.py')
    cb = _func(_parse(path), 'wp_mode_callback', 'gate')
    keys = set()
    for n in ast.walk(cb):
        if isinstance(n, ast.Assign) and any(_is_self_attr(t, 'last_valid') for t in n.targets) \
                and isinstance(n.value, ast.Dict):
            keys |= {k.value for k in n.value.keys if isinstance(k, ast.Constant)}
    assert keys >= {'red', 'green'}, f"gate last_valid 리셋에 red·green 둘 다 없다: {keys}"


# ============================================================ 시뮬 Det — 소스만 떼어 실행(무거운 import 없음)
def _load_sim_det():
    with open(SIM_PATH, encoding='utf-8') as f:
        src = f.read()
    tree = ast.parse(src, filename=SIM_PATH)
    cls = [n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Det']
    assert len(cls) == 1, "full_course_sim.py 에 class Det 가 1개여야 한다"
    seg = ast.get_source_segment(src, cls[0])
    ns = {'ModeGate': ModeGate, 'DetectionConfirmer': DetectionConfirmer, 'INVALID': INVALID}
    exec(compile(seg, SIM_PATH, 'exec'), ns)
    return ns['Det'], src


def test_sim_det_on_mode_change_then_miss_is_invalid():
    Det, _ = _load_sim_det()
    d = Det({"active_wp_modes": [2, 3], "confirm_frames": 1}, 1)
    assert d.frame("img", "white", 8.0, 100.0) == 8.0
    d.last_color = "white"
    assert d.frame("img", None, None, 100.5) == 8.0, "같은 모드 안 미검출은 유예"
    d.on_mode_change()
    assert d.frame("img", None, None, 100.6) == INVALID, "리셋 뒤 미검출 → INVALID(10000)"
    assert not hasattr(d, "last_color"), "turn 의 마지막 색도 지운다"


def test_sim_det_on_mode_change_requires_n_again():
    Det, _ = _load_sim_det()
    d = Det({"active_wp_modes": [7], "confirm_frames": 3}, 3)
    for t in (0.0, 0.1):
        d.frame("dock", ("red", "Square"), 1.0, t)
    assert d.frame("dock", ("red", "Square"), 1.0, 0.2) == 1.0
    d.on_mode_change()
    assert d.frame("dock", ("red", "Square"), 1.0, 0.3) == INVALID, "리셋 뒤 1 프레임 — 미확정·유예 없음"
    assert d.frame("dock", ("red", "Square"), 1.0, 0.4) == INVALID
    assert d.frame("dock", ("red", "Square"), 1.0, 0.5) == 1.0


def test_sim_calls_on_mode_change_only_on_value_change():
    """호출부: /wp_mode 반영 자리에서 `m.data != pub_mode` 일 때만 3 검출기 모두 on_mode_change()."""
    _, src = _load_sim_det()
    tree = ast.parse(src)
    calls = [n for n in ast.walk(tree)
             if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
             and n.func.attr == 'on_mode_change']
    assert len(calls) == 1, f"on_mode_change 호출 {len(calls)}곳(검출기 3개를 한 루프에서 1곳으로)"
    # 그 호출을 감싸는 If 의 조건이 pub_mode 비교(직접 또는 변수)를 쓰는지
    ifs = [n for n in ast.walk(tree)
           if isinstance(n, ast.If) and any(c is calls[0] for c in ast.walk(n))]
    assert ifs, "on_mode_change 호출이 if 안에 없다 — 수신마다 리셋"
    inner = min(ifs, key=lambda n: (calls[0].lineno - n.lineno))
    test_src = ast.get_source_segment(src, inner.test)
    ok = 'pub_mode' in test_src
    if not ok:                      # 조건 변수(mode_chg 등)가 pub_mode 비교로 정의됐는지
        names = {n.id for n in ast.walk(inner.test) if isinstance(n, ast.Name)}
        for n in ast.walk(tree):
            if isinstance(n, ast.Assign) \
                    and any(isinstance(t, ast.Name) and t.id in names for t in n.targets):
                ok = ok or ('pub_mode' in ast.get_source_segment(src, n.value))
    assert ok, f"on_mode_change 조건이 pub_mode 와의 값 비교가 아니다: {test_src}"
    loop = [n for n in ast.walk(tree)
            if isinstance(n, ast.For) and any(c is calls[0] for c in ast.walk(n))]
    assert loop, "on_mode_change 는 det_gate·det_turn·det_dock 루프 안에서 불러야 한다"
    it = ast.get_source_segment(src, loop[-1].iter)
    assert all(k in it for k in ('det_gate', 'det_turn', 'det_dock')), f"루프 대상: {it}"


# ================================================================ 실행
if __name__ == "__main__":
    print("=== 검출기 모드 전환 리셋 (계약서_검출기모드리셋_20261106) ===")
    for n, f in sorted(globals().items()):
        if n.startswith("test_") and callable(f):
            check(n[5:], f)
    print(f"\n{_p}/{_t} 통과")
    sys.exit(0 if _p == _t else 1)
