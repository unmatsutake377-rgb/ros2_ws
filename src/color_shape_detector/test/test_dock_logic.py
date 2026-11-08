#!/usr/bin/env python3
"""dock_logic 순수 로직 테스트. python3 src/color_shape_detector/test/test_dock_logic.py"""

import ast
import inspect
import math
import os
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from color_shape_detector.dock_logic import (  # noqa: E402
    TRIANGLE, SQUARE, CIRCLE, VALID_SHAPES, VALID_COLORS,
    circularity, classify_shape, DetectionConfirmer,
    dock_frame_action, FRAME_SKIP, FRAME_PUBLISH_INVALID, FRAME_DETECT,
)
from color_shape_detector.mode_gate import ModeGate  # noqa: E402

_PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
_VISION_YAML = os.path.join(_PKG, 'config', 'vision.yaml')
_DOCK_NODE = os.path.join(_PKG, 'color_shape_detector', 'basic_image_subscriberdock.py')

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


# ---- 이상적 도형의 기하 특징(면적·둘레) 생성기 (cv2 없이 모사) ----
def circle_feat(r):
    a = math.pi * r * r
    p = 2 * math.pi * r
    return dict(v_count=12, area=a, perimeter=p, w=2 * r, h=2 * r)


def square_feat(s):
    return dict(v_count=4, area=s * s, perimeter=4 * s, w=s, h=s)


def triangle_feat(s):
    # 정삼각형: 면적 = √3/4 s², 둘레 = 3s, 바운딩박스 w=s, h=√3/2 s
    a = math.sqrt(3) / 4 * s * s
    return dict(v_count=3, area=a, perimeter=3 * s, w=s, h=math.sqrt(3) / 2 * s)


# ================================================================ 원형도
def test_circularity_values():
    assert abs(circularity(math.pi * 100, 2 * math.pi * 10) - 1.0) < 1e-9, "원=1"
    assert abs(circularity(100, 40) - 0.785) < 0.01, "정사각형≈0.785"
    tri = triangle_feat(10)
    c = circularity(tri["area"], tri["perimeter"])
    assert 0.55 < c < 0.65, f"정삼각형≈0.605, got {c}"


def test_circularity_bad_perimeter():
    assert circularity(100, 0) is None
    assert circularity(100, None) is None


# ================================================================ 분류 (D3)
def test_classify_triangle():
    assert classify_shape(**triangle_feat(50)) == TRIANGLE


def test_classify_square():
    assert classify_shape(**square_feat(50)) == SQUARE


def test_classify_circle_by_vcount():
    assert classify_shape(**circle_feat(50)) == CIRCLE


def test_classify_scale_invariant():
    """스케일이 달라도 형상 판정은 같아야 한다(회전·거리 변화 모사)."""
    for s in (20, 30, 80, 200):  # s=12 는 면적<80 이라 의도적으로 탈락(작은 표식)
        assert classify_shape(**square_feat(s)) == SQUARE, f"square s={s}"
        assert classify_shape(**triangle_feat(s)) == TRIANGLE, f"tri s={s}"
    for r in (8, 25, 60, 150):
        assert classify_shape(**circle_feat(r)) == CIRCLE, f"circle r={r}"


def test_classify_rejects_small():
    """면적 하한 미달은 버린다(노이즈)."""
    assert classify_shape(**square_feat(5), min_area=80.0) is None


def test_classify_rejects_thin_rectangle():
    """가늘고 긴 사각형은 종횡비로 배제(표식이 아님)."""
    f = dict(v_count=4, area=100 * 10, perimeter=2 * (100 + 10), w=100, h=10)
    assert classify_shape(**f) is None, "aspect 10:1 은 사각 표식이 아니다"


def test_classify_low_extent_rejected():
    """꼭짓점은 4인데 채움비가 낮으면(속 빈 모양) 사각 아님."""
    # 바운딩박스는 큰데 실제 면적이 작다
    f = dict(v_count=4, area=100, perimeter=4 * 50, w=50, h=50)
    assert classify_shape(**f) is None


def test_classify_circularity_overrides_vcount():
    """
    🚨 D3 핵심: 꼭짓점 수가 애매해도 원형도로 바로잡는다.
    꼭짓점이 5(사각 범위)인데 원형도가 원이면 사각으로 확정하지 않는다.
    """
    r = 50
    a = math.pi * r * r
    p = 2 * math.pi * r
    # approxPolyDP 가 원을 5각형으로 근사한 경우 (실제로 잘 생김)
    f = dict(v_count=5, area=a, perimeter=p, w=2 * r, h=2 * r)
    assert classify_shape(**f) == CIRCLE, "원형도가 원이면 v=5 여도 원"


def test_classify_spiky_noise_rejected():
    """꼭짓점만 많고 원형도가 낮은 울퉁불퉁 노이즈는 버린다."""
    # v_count 큰데 둘레가 과도하게 길다(들쭉날쭉) → 원형도 낮음
    f = dict(v_count=12, area=1000, perimeter=500, w=40, h=40)
    c = circularity(1000, 500)
    assert c < 0.6
    assert classify_shape(**f) is None


def test_classify_params_are_tunable():
    """임계가 파라미터로 먹는지(하드코딩 아님)."""
    f = square_feat(50)
    assert classify_shape(**f, min_area=99999.0) is None, "면적 임계를 올리면 탈락"


# ================================================================ N프레임 확정 (D2)
def test_confirm_needs_n_frames():
    c = DetectionConfirmer(confirm_frames=3)
    assert c.update(("red", SQUARE)) is None, "1프레임"
    assert c.update(("red", SQUARE)) is None, "2프레임"
    assert c.update(("red", SQUARE)) == ("red", SQUARE), "3프레임 확정"


def test_confirm_single_frame_glitch_ignored():
    """
    🚨 D2 핵심: 안정된 판정 사이에 오탐 1프레임이 끼면 그 오탐은 발행되면 안 된다.
    """
    c = DetectionConfirmer(confirm_frames=3)
    c.update(("red", SQUARE))
    c.update(("red", SQUARE))
    c.update(("red", SQUARE))
    assert c.confirmed == ("red", SQUARE)
    # 오탐 1프레임(초록 삼각)이 끼어듦
    out = c.update(("green", TRIANGLE))
    assert out is None, "오탐 1프레임은 확정되면 안 된다"
    assert c.confirmed == ("red", SQUARE), "1프레임 glitch 로는 이전 확정이 안 내려간다(강건)"


def test_confirm_reset_on_missing():
    c = DetectionConfirmer(confirm_frames=3)
    c.update(("red", SQUARE))
    c.update(("red", SQUARE))
    assert c.update(None) is None, "미검출이면 리셋"
    assert c.update(("red", SQUARE)) is None, "다시 처음부터"


def test_confirm_switch_target():
    """목표가 진짜로 바뀌면(연속 N프레임) 새 목표로 확정."""
    c = DetectionConfirmer(confirm_frames=2)
    c.update(("red", SQUARE))
    c.update(("red", SQUARE))
    assert c.confirmed == ("red", SQUARE)
    c.update(("blue", CIRCLE))
    assert c.update(("blue", CIRCLE)) == ("blue", CIRCLE), "2프레임 연속이면 전환"


def test_confirm_frames_1_is_immediate():
    c = DetectionConfirmer(confirm_frames=1)
    assert c.update(("red", SQUARE)) == ("red", SQUARE)


# ================================================================ 창 M 중 N (A1, 2026-10-27)
class _LegacyConfirmer:
    """2026-08 '연속 N' 구현의 참조 사본 — 창=N 이면 새 구현과 같아야 한다(회귀 비교 전용)."""

    def __init__(self, n):
        self.n = max(1, int(n))
        self._cur = None
        self._count = 0
        self.confirmed = None

    def update(self, key):
        if key is None:
            self._cur = None
            self._count = 0
            return None
        if key == self._cur:
            self._count += 1
        else:
            self._cur = key
            self._count = 1
        if self._count >= self.n:
            self.confirmed = key
            return key
        return None


def test_window_flicker_one_miss():
    """깜빡임 r,r,None,r — 창 5 중 3 이면 4번째에 확정(옛 연속 N 은 영원히 미확정)."""
    c = DetectionConfirmer(confirm_frames=3, confirm_window=5)
    r = ("red", SQUARE)
    assert c.update(r) is None
    assert c.update(r) is None
    assert c.update(None) is None, "미검출 프레임은 None"
    assert c.update(r) == r, "창 [r,r,None,r] 에 r 3개 → 확정"


def test_window_flicker_alternating():
    """r,None,r,None,r — 창 5 중 3 이면 5번째에 확정."""
    c = DetectionConfirmer(confirm_frames=3, confirm_window=5)
    r = ("red", SQUARE)
    outs = [c.update(k) for k in (r, None, r, None, r)]
    assert outs == [None, None, None, None, r], outs


def test_window_equals_n_matches_legacy_random():
    """창=N(기본) 이면 옛 '연속 N' 과 반환값·confirmed 가 전부 같다(N∈{1,2,3} 무작위)."""
    import random
    rng = random.Random(20261027)
    keys = [("red", SQUARE), ("green", TRIANGLE), ("blue", CIRCLE), None, None]
    for n in (1, 2, 3):
        for trial in range(200):
            seq = [rng.choice(keys) for _ in range(rng.randint(1, 30))]
            new = DetectionConfirmer(confirm_frames=n)          # window 미지정 → = N
            old = _LegacyConfirmer(n)
            for i, k in enumerate(seq):
                a, b = new.update(k), old.update(k)
                assert a == b, f"N={n} trial={trial} step={i} seq={seq}: new={a} old={b}"
                assert new.confirmed == old.confirmed, f"N={n} trial={trial} step={i}"
            assert new.confirm_window == n


def test_window_n1_is_immediate_regardless_of_window():
    c = DetectionConfirmer(confirm_frames=1, confirm_window=5)
    r = ("red", SQUARE)
    assert c.update(r) == r
    assert c.update(("green", TRIANGLE)) == ("green", TRIANGLE)
    assert c.update(None) is None
    assert c.update(r) == r


def test_window_turn_color_flip_returns_none():
    """
    🚨 계약서 위험 2: 창 [red,red,white,red,white] — red 가 3개라도 **이번 프레임이 white** 면
    None 이어야 한다. red 를 돌려주면 노드는 white 각도를 red 로 발행 → ship_turn 반대 회전.
    """
    c = DetectionConfirmer(confirm_frames=3, confirm_window=5)
    outs = [c.update(k) for k in ("red", "red", "red", "white", "red", "white")]
    assert outs[2] == "red", "3번째(red): red 3 → red"
    assert outs[3] is None, f"4번째(white): 다른 키 → 창 리셋, white 1 → None, got {outs[3]}"
    assert c.confirmed == "red", "직전 확정은 유지"
    # 키가 바뀔 때마다 창이 비므로 교대 입력에선 어느 쪽도 확정되지 않는다(옛 연속-N 과 같음)
    assert outs[4] is None and outs[5] is None
    # white 가 연속 3 (사이에 None 은 허용) 이면 그때 white 로 전환
    assert c.update("white") is None
    assert c.update(None) is None
    assert c.update("white") == "white"
    assert c.confirmed == "white"


def test_window_alternating_colors_never_confirm():
    """리뷰 권고 1: r,w,r,w,… 교대 입력에서 창>N 이어도 확정이 매 프레임 뒤집히면 안 된다."""
    c = DetectionConfirmer(confirm_frames=3, confirm_window=5)
    outs = [c.update(k) for k in ("red", "white") * 6]
    assert all(o is None for o in outs), f"교대 입력은 영원히 미확정이어야 한다: {outs}"
    assert c.confirmed is None


def test_window_none_frame_returns_none_even_when_confirmed():
    c = DetectionConfirmer(confirm_frames=2, confirm_window=4)
    r = ("red", SQUARE)
    c.update(r)
    assert c.update(r) == r
    assert c.update(None) is None, "확정 뒤라도 미검출 프레임의 반환은 None"
    assert c.confirmed == r


def test_window_smaller_than_n_is_corrected():
    c = DetectionConfirmer(3, confirm_window=1)
    assert c.confirm_window == 3
    r = ("red", SQUARE)
    assert c.update(r) is None
    assert c.update(r) is None
    assert c.update(r) == r
    c2 = DetectionConfirmer(3)
    assert c2.confirm_window == 3, "미지정이면 = N"


def test_window_positional_first_arg_compat():
    """기존 호출부 DetectionConfirmer(cf) 위치 인자 호환."""
    c = DetectionConfirmer(2)
    assert c.confirm_frames == 2 and c.confirm_window == 2


def test_valid_sets():
    assert set(VALID_SHAPES) == {TRIANGLE, SQUARE, CIRCLE}
    assert "white" in VALID_COLORS and len(VALID_COLORS) == 6


# ---- dock_frame_action: 게이트가 무효 목표보다 먼저 (계약서_도킹게이트_20261028) ----
def _inactive_gates():
    """ModeGate 비활성 3사유를 실제로 만들어 (사유, active) 로 돌려준다."""
    out = []
    g = ModeGate([7], stale_sec=2.0)                 # 부팅 후 /wp_mode 미수신
    out.append(g.state(0.0))
    g = ModeGate([7], stale_sec=2.0)                 # 모드 7 이었는데 끊김
    g.update(7, 0.0)
    out.append(g.state(2.5))
    g = ModeGate([7], stale_sec=2.0)                 # 지금은 turn 차례 (2·3)
    g.update(3, 0.0)
    out.append(g.state(0.1))
    return out


def test_frame_action_inactive_reasons_all_skip():
    """게이트 비활성 3사유 모두 → 목표 유효/무효와 상관없이 SKIP (발행 없음)."""
    states = _inactive_gates()
    whys = sorted(why for _a, why in states)
    assert whys == sorted([ModeGate.R_NO_MODE, ModeGate.R_STALE, ModeGate.R_OTHER_MODE]), whys
    for active, why in states:
        assert active is False, why
        for tv in (True, False):
            got = dock_frame_action(tv, active)
            assert got == FRAME_SKIP, f"{why} target_valid={tv} → {got}"


def test_frame_action_invalid_target_mode_2_3_is_silent():
    """버그 재현 입력: 무효 목표 + 모드 2·3 → 10000 을 내면 안 된다(turn 방위를 지운다)."""
    for mode in (0, 1, 2, 3, 5, 8):
        g = ModeGate([7])
        g.update(mode, 0.0)
        active, _why = g.state(0.1)
        assert dock_frame_action(False, active) == FRAME_SKIP, mode


def test_frame_action_active_invalid_publishes_invalid():
    g = ModeGate([7])
    g.update(7, 0.0)
    active, _why = g.state(0.1)
    assert active is True
    assert dock_frame_action(False, active) == FRAME_PUBLISH_INVALID


def test_frame_action_active_valid_detects():
    g = ModeGate([7])
    g.update(7, 0.0)
    active, _why = g.state(0.1)
    assert dock_frame_action(True, active) == FRAME_DETECT


def test_frame_action_truth_table():
    """진리표 전수 + 세 값이 서로 다르다."""
    table = {
        (False, False): FRAME_SKIP,
        (True, False): FRAME_SKIP,
        (False, True): FRAME_PUBLISH_INVALID,
        (True, True): FRAME_DETECT,
    }
    for (tv, ga), want in table.items():
        assert dock_frame_action(tv, ga) == want, (tv, ga)
    assert len({FRAME_SKIP, FRAME_PUBLISH_INVALID, FRAME_DETECT}) == 3


# ================================================================ A: 원형도 임계 0.87 (계약서_도킹비전_사전수정 A)
# 특징값 출처: `python3 tools/sim/dock_shape_sweep.py --features` (cv2 5.0.0, eps 0.0315, 지터 없음).
#   형식 (v_count, area, perimeter, w, h). 키 (크기 px, 회전 °). 픽셀 기하 합성값 — 실영상 아님.
# 0.82 에선 Circle, 0.87 에선 Square 로 바뀌는 작은 사각(v=4, 원형도 0.82~0.86)
_SQUARE_FLIPPED = {
    (12, 0): (4, 142.0, 45.657, 13, 13),
    (16, 0): (4, 254.0, 61.657, 17, 17),
    (20, 0): (4, 398.0, 77.657, 21, 21),
    (20, 45): (4, 388.0, 75.882, 27, 27),
    (25, 0): (4, 623.0, 97.657, 26, 26),
    (25, 45): (4, 644.0, 98.510, 35, 35),
    (30, 45): (4, 878.0, 115.480, 41, 41),
}
# 두 임계 모두 Square
_SQUARE_BOTH = {(16, 15): (4, 250.0, 64.971, 19, 19)}
# 두 임계 모두 Circle 이어야 할 원(v=8 → 원 경로는 v>=7 이라 임계 무관. (30,15) 는 원형도 0.861)
_CIRCLE_BOTH = {
    (30, 15): (8, 670.0, 98.912, 31, 31),
    (12, 15): (8, 88.0, 35.314, 11, 11),
}
# 알려진 한계: 12 px 45° 마름모(v=4, 원형도 0.876)는 0.87 에서도 Circle
_SQUARE_KNOWN_LIMIT = {(12, 45): (4, 158.0, 47.598, 17, 17)}


def test_shape_sweep_small_squares_are_square_at_default():
    """🚨 기본 임계(0.87)에서 작은 사각의 픽셀 외곽선은 SQUARE. 0.82 면 같은 입력이 CIRCLE(옛 오인)."""
    for k, f in _SQUARE_FLIPPED.items():
        assert classify_shape(*f) == SQUARE, f"{k} {f}: 기본 임계에서 Square 여야 한다"
        assert classify_shape(*f, circle_circularity_min=0.82) == CIRCLE, \
            f"{k} {f}: 0.82 에선 Circle(옛 오인) — 특징값이 sweep 과 어긋났나?"
    for k, f in _SQUARE_BOTH.items():
        assert classify_shape(*f) == SQUARE, k
        assert classify_shape(*f, circle_circularity_min=0.82) == SQUARE, k


def test_shape_sweep_circles_stay_circle():
    for k, f in _CIRCLE_BOTH.items():
        assert classify_shape(*f) == CIRCLE, f"{k} {f}"
        assert classify_shape(*f, circle_circularity_min=0.82) == CIRCLE, f"{k} {f}"


def test_shape_sweep_known_limit_documented():
    """알려진 한계 고정 — 16 px 이하 45° 마름모는 0.87 에서도 원. 바뀌면 문서·주석도 같이 고쳐라."""
    for k, f in _SQUARE_KNOWN_LIMIT.items():
        assert classify_shape(*f) == CIRCLE, f"{k} {f}"


def _chamfered_square(r, s=100.0):
    """한 변 s 정사각의 네 모서리를 변의 r 만큼 45° 로 깎은 도형. 원형도 = 4π(1−2r²)/(4−2.343r)²."""
    area = s * s * (1.0 - 2.0 * r * r)
    perim = s * (4.0 - (8.0 - 4.0 * math.sqrt(2.0)) * r)
    # approxPolyDP(eps 0.0315·P)는 작은 깎임을 꼭짓점 4 로 접는다 → 사각 경로(4<=v<=6)
    return dict(v_count=4, area=area, perimeter=perim, w=s, h=s)


def test_chamfered_square_is_square():
    """블러로 모서리가 r 만큼 둥글어진 사각(해석적) → 기본 임계에서 SQUARE."""
    for r in (0.05, 0.08):
        f = _chamfered_square(r)
        c = circularity(f["area"], f["perimeter"])
        want = 4 * math.pi * (1 - 2 * r * r) / (4 - 2.343 * r) ** 2
        assert abs(c - want) < 1e-3, f"r={r}: 원형도 {c} vs 식 {want}"
        assert classify_shape(**f) == SQUARE, f"r={r} 원형도 {c:.3f}"
    # r=0.08(원형도 ≈0.853)은 옛 0.82 에선 원으로 갔다
    assert classify_shape(**_chamfered_square(0.08), circle_circularity_min=0.82) == CIRCLE


def test_triangle_branch_shares_threshold():
    """계약 위험 2·결정 3: v==3·원형도 0.85 는 0.87 에서 TRIANGLE(옛 0.82 에선 None, 노이즈로 버림)."""
    perim = 40.0
    area = 0.85 * perim * perim / (4 * math.pi)
    f = dict(v_count=3, area=area, perimeter=perim, w=14, h=12)
    assert abs(circularity(area, perim) - 0.85) < 1e-9
    assert classify_shape(**f) == TRIANGLE
    assert classify_shape(**f, circle_circularity_min=0.82) is None


def _yaml_section(name):
    """vision.yaml 에서 최상위 키 `name:` 절의 본문(다음 0열 비공백 줄 — 0열 주석 포함 — 전까지). pyyaml 없이."""
    txt = open(_VISION_YAML, encoding='utf-8').read()
    m = re.search(r'^' + re.escape(name) + r':\s*$(.*?)(?=^\S|\Z)', txt, re.M | re.S)
    assert m, f"vision.yaml 에 {name}: 절이 없다"
    return m.group(1)


def _yaml_scalar(section, key):
    """절 본문에서 `key: <값>` 의 값 문자열(주석 제외). 없으면 None."""
    m = re.search(r'^\s+' + re.escape(key) + r':\s*([^#\s]+)', section, re.M)
    return m.group(1) if m else None


def _dock_init():
    tree = ast.parse(open(_DOCK_NODE, encoding='utf-8').read(), filename=_DOCK_NODE)
    for n in ast.walk(tree):
        if isinstance(n, ast.FunctionDef) and n.name == '__init__':
            return n
    raise AssertionError("dock 노드에 __init__ 없음")


def _declare_calls(fn, name):
    return [n for n in ast.walk(fn)
            if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
            and n.func.attr == 'declare_parameter'
            and n.args and isinstance(n.args[0], ast.Constant) and n.args[0].value == name]


def test_circularity_default_same_in_node_yaml_and_logic():
    """🚨 §1-4: dock declare 기본값(AST) = vision.yaml dock 절 = classify_shape 기본 인자."""
    logic = inspect.signature(classify_shape).parameters['circle_circularity_min'].default
    calls = _declare_calls(_dock_init(), 'circle_circularity_min')
    assert len(calls) == 1, f"dock 노드 circle_circularity_min 선언 {len(calls)}개"
    node = calls[0].args[1]
    assert isinstance(node, ast.Constant) and isinstance(node.value, float), "기본값은 소수 리터럴"
    raw = _yaml_scalar(_yaml_section('image_subscriber_dock'), 'circle_circularity_min')
    assert raw is not None and '.' in raw, f"vision.yaml dock 절 circle_circularity_min 은 소수여야 한다: {raw}"
    assert logic == node.value == float(raw) == 0.87, (logic, node.value, raw)


# ================================================================ C: dock confirm_window (계약서_도킹비전_사전수정 C)
def test_dock_declares_confirm_window_defaulting_to_confirm_frames():
    """dock 이 confirm_window 를 confirm_frames 기본값으로 선언하고 confirmer 에 넘긴다.

    declare_parameter('confirm_window', self.confirm_frames)
    → self.confirmer = DetectionConfirmer(..., confirm_window=...). AST 로 본다(노드는 cv2 import).
    """
    init = _dock_init()
    calls = _declare_calls(init, 'confirm_window')
    assert len(calls) == 1, f"confirm_window 선언 {len(calls)}개"
    d = calls[0].args[1]
    assert isinstance(d, ast.Attribute) and d.attr == 'confirm_frames' \
        and isinstance(d.value, ast.Name) and d.value.id == 'self', \
        "기본값이 self.confirm_frames 가 아니다 — 기본 = 연속 N 이 깨진다"
    cf = _declare_calls(init, 'confirm_frames')
    assert cf and cf[0].lineno < calls[0].lineno, "confirm_frames 를 먼저 선언해야 한다"
    assigns = [n for n in ast.walk(init) if isinstance(n, ast.Assign)
               and isinstance(n.value, ast.Call) and isinstance(n.value.func, ast.Name)
               and n.value.func.id == 'DetectionConfirmer']
    assert len(assigns) == 1, f"DetectionConfirmer 생성 {len(assigns)}곳"
    tgt = assigns[0].targets[0]
    assert isinstance(tgt, ast.Attribute) and tgt.attr == 'confirmer', \
        "속성 이름은 self.confirmer 여야 한다(wp_mode_callback reset·test_detector_mode_reset 의존)"
    kws = {k.arg for k in assigns[0].value.keywords}
    assert 'confirm_window' in kws, "DetectionConfirmer 에 confirm_window 를 넘기지 않는다"
    assert calls[0].lineno < assigns[0].lineno, "confirm_window 선언이 DetectionConfirmer 생성보다 먼저여야 한다"


def test_yaml_confirm_params_are_int_and_dock_window_equals_frames():
    """🚨 세 검출기 절의 confirm_frames·confirm_window 는 정수(3.0 이면 부팅 예외). dock 은 창 = N."""
    common = _yaml_scalar(_yaml_section('/**'), 'confirm_frames')
    assert common is not None and re.fullmatch(r'\d+', common), f"/** confirm_frames: {common}"
    eff = {}
    for node in ('image_subscriber_gate', 'image_subscriber_turn', 'image_subscriber_dock'):
        sec = _yaml_section(node)
        cf = _yaml_scalar(sec, 'confirm_frames')
        cw = _yaml_scalar(sec, 'confirm_window')
        cf = common if cf is None else cf          # 절에 없으면 /** 값을 받는다
        assert re.fullmatch(r'\d+', cf), f"{node} confirm_frames 가 정수가 아니다: {cf}"
        assert cw is not None and re.fullmatch(r'\d+', cw), f"{node} confirm_window 가 정수가 아니다: {cw}"
        eff[node] = (int(cf), int(cw))
    cf, cw = eff['image_subscriber_dock']
    assert cw == cf, f"dock confirm_window {cw} != confirm_frames {cf} — 벤치 전엔 연속 N 그대로"
    assert DetectionConfirmer(cf, confirm_window=cw).confirm_window == cf


def main():
    print("=== dock_logic 테스트 ===")
    for n, f in sorted(globals().items()):
        if n.startswith("test_") and callable(f):
            check(n[5:], f)
    print(f"\n{_p}/{_t} 통과")
    return 0 if _p == _t else 1


if __name__ == "__main__":
    sys.exit(main())
