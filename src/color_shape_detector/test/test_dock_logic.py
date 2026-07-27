#!/usr/bin/env python3
"""dock_logic 순수 로직 테스트. python3 src/color_shape_detector/test/test_dock_logic.py"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from color_shape_detector.dock_logic import (  # noqa: E402
    TRIANGLE, SQUARE, CIRCLE, VALID_SHAPES, VALID_COLORS,
    circularity, classify_shape, DetectionConfirmer,
)

_p = _t = 0


def check(name, fn):
    global _p, _t
    _t += 1
    try:
        fn(); _p += 1; print(f"  ✅ {name}")
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
    c.update(("red", SQUARE)); c.update(("red", SQUARE)); c.update(("red", SQUARE))
    assert c.confirmed == ("red", SQUARE)
    # 오탐 1프레임(초록 삼각)이 끼어듦
    out = c.update(("green", TRIANGLE))
    assert out is None, "오탐 1프레임은 확정되면 안 된다"
    assert c.confirmed == ("red", SQUARE), "1프레임 glitch 로는 이전 확정이 안 내려간다(강건)"


def test_confirm_reset_on_missing():
    c = DetectionConfirmer(confirm_frames=3)
    c.update(("red", SQUARE)); c.update(("red", SQUARE))
    assert c.update(None) is None, "미검출이면 리셋"
    assert c.update(("red", SQUARE)) is None, "다시 처음부터"


def test_confirm_switch_target():
    """목표가 진짜로 바뀌면(연속 N프레임) 새 목표로 확정."""
    c = DetectionConfirmer(confirm_frames=2)
    c.update(("red", SQUARE)); c.update(("red", SQUARE))
    assert c.confirmed == ("red", SQUARE)
    c.update(("blue", CIRCLE))
    assert c.update(("blue", CIRCLE)) == ("blue", CIRCLE), "2프레임 연속이면 전환"


def test_confirm_frames_1_is_immediate():
    c = DetectionConfirmer(confirm_frames=1)
    assert c.update(("red", SQUARE)) == ("red", SQUARE)


def test_valid_sets():
    assert set(VALID_SHAPES) == {TRIANGLE, SQUARE, CIRCLE}
    assert "white" in VALID_COLORS and len(VALID_COLORS) == 6


def main():
    print("=== dock_logic 테스트 ===")
    for n, f in sorted(globals().items()):
        if n.startswith("test_") and callable(f):
            check(n[5:], f)
    print(f"\n{_p}/{_t} 통과")
    return 0 if _p == _t else 1


if __name__ == "__main__":
    sys.exit(main())
