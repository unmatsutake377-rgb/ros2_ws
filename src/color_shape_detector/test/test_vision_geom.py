#!/usr/bin/env python3
"""
vision_geom 순수 로직 테스트. ROS/OpenCV 없이 그냥 실행된다:
    python3 src/color_shape_detector/test/test_vision_geom.py
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from color_shape_detector.vision_geom import (  # noqa: E402
    DEFAULT_HFOV_DEG, EXACT_LEGACY_HFOV_DEG, LEGACY_HFOV_DEG,
    LEGACY_PIXEL_TO_ANGLE_K, MEASURED_D455_640x480,
    angle_from_pixel, fx_from_hfov, legacy_angle_from_pixel,
)

_passed = 0
_total = 0


def check(name, fn):
    global _passed, _total
    _total += 1
    try:
        fn()
    except AssertionError as e:
        print(f"  ❌ {name}\n     {e}")
        return
    _passed += 1
    print(f"  ✅ {name}")


def near(a, b, tol=1e-9):
    return abs(a - b) < tol


# ------------------------------------------------------------------ fx
def test_fx_matches_legacy():
    """작년 매직넘버의 등가 fx ≈ 444.44px 를 재현하나."""
    fx = fx_from_hfov(640, EXACT_LEGACY_HFOV_DEG)
    assert near(fx, 1.0 / LEGACY_PIXEL_TO_ANGLE_K, 1e-6), fx


def test_fx_scales_with_width():
    """같은 화각이면 fx 는 해상도에 비례한다 — 이래서 width 를 매 프레임 읽는다."""
    a = fx_from_hfov(640, 71.5)
    b = fx_from_hfov(1280, 71.5)
    assert near(b / a, 2.0, 1e-9), (a, b)


def test_fx_wider_hfov_smaller_fx():
    narrow = fx_from_hfov(640, 71.5)
    wide = fx_from_hfov(640, 120.0)
    assert wide < narrow, "광각일수록 fx 가 작다"


def test_fx_rejects_bad_input():
    for w, h in ((0, 71.5), (-640, 71.5), (None, 71.5)):
        try:
            fx_from_hfov(w, h)
        except ValueError:
            continue
        assert False, f"width={w!r} 를 조용히 받아들였다"
    for h in (0.0, 180.0, -10.0, 200.0):
        try:
            fx_from_hfov(640, h)
        except ValueError:
            continue
        assert False, f"hfov={h!r} 를 조용히 받아들였다"


# ------------------------------------------------------------------ 각도 규약
def test_angle_center_is_zero():
    assert near(angle_from_pixel(320, 640, 71.5), 0.0)


def test_angle_sign_left_positive():
    """왼쪽이 +. 뒤집히면 조향이 반대로 돈다 (/candidate_angle 규약)."""
    assert angle_from_pixel(100, 640, 71.5) > 0.0, "왼쪽 = 양수"
    assert angle_from_pixel(540, 640, 71.5) < 0.0, "오른쪽 = 음수"


def test_angle_symmetric():
    left = angle_from_pixel(320 - 200, 640, 71.5)
    right = angle_from_pixel(320 + 200, 640, 71.5)
    assert near(left, -right, 1e-12), (left, right)


def test_angle_edge_is_half_hfov():
    """가장자리 각도는 화각의 절반이어야 한다(정의 검증)."""
    for hfov in (71.5, 90.0, 120.0):
        a = angle_from_pixel(0, 640, hfov)
        assert near(a, hfov / 2.0, 1e-9), (hfov, a)


def test_angle_monotonic():
    prev = None
    for vx in range(0, 641, 10):
        a = angle_from_pixel(vx, 640, 71.5)
        if prev is not None:
            assert a < prev, f"단조감소여야 한다 (vx={vx})"
        prev = a


# ------------------------------------------------------------------ 🚨 회귀 비교
def test_regression_exact_hfov_is_bit_equal():
    """정밀 화각을 쓰면 작년 식과 사실상 완전히 같다."""
    worst = 0.0
    for vx in range(0, 641):
        new = angle_from_pixel(vx, 640, EXACT_LEGACY_HFOV_DEG)
        old = legacy_angle_from_pixel(vx, 640)
        worst = max(worst, abs(new - old))
    assert worst < 1e-9, f"최대오차 {worst}"


def test_legacy_constant_rounding_within_pixel_noise():
    """
    71.5(작년 등가 화각)를 **그 목적으로 쓸 때**의 반올림 오차만 본다.

    ⚠️ [2026-08-12] 예전엔 이 테스트가 `DEFAULT_HFOV_DEG` 를 검사했다.
       기본값이 71.5 였고, "작년 동작을 그대로 재현한다" 를 고정하는 게 목적이었다.
       **그런데 그 작년 동작 자체가 틀렸다는 게 밝혀졌다** — camera_info 실측 fx 는
       379.19(HFOV 80.32°)인데 작년 매직넘버는 444.44(71.5°)였다.
       그래서 검사 대상을 `DEFAULT_HFOV_DEG` → `LEGACY_HFOV_DEG` 로 바꿨다.
       이 테스트는 이제 '역사 기록' 이지 '올바름의 근거' 가 아니다.
    """
    one_px_deg = abs(angle_from_pixel(321, 640, LEGACY_HFOV_DEG)
                     - angle_from_pixel(320, 640, LEGACY_HFOV_DEG))
    worst = max(abs(angle_from_pixel(vx, 640, LEGACY_HFOV_DEG)
                    - legacy_angle_from_pixel(vx, 640))
                for vx in range(0, 641))
    assert worst < one_px_deg * 0.1, \
        f"최대오차 {worst:.6f}° 가 1픽셀({one_px_deg:.4f}°)의 10% 를 넘는다"
    assert worst < 0.005, f"{worst:.6f}°"


def test_default_hfov_matches_measured_camera_info():
    """
    🚨 기본 화각이 **카메라가 스스로 보고한 값**과 맞는지 못 박는다.

    근거는 벤치 캘리브레이션이 아니라 `camera_info`(공장 캘리브레이션)다.
    D455 S/N 117122250518, 640x480 에서 fx=379.189 → HFOV 80.32°.
    누가 이 값을 옛날 71.5 로 되돌리면 여기서 걸린다.
    """
    fx_measured = MEASURED_D455_640x480["fx"]
    fx_default = fx_from_hfov(640, DEFAULT_HFOV_DEG)
    # 80.32° 는 소수 둘째 자리 반올림값이라 fx 가 정확히 일치하진 않는다.
    # 0.1px 이내면 각도로는 1e-3° 수준이라 무해하다.
    assert abs(fx_default - fx_measured) < 0.1, \
        f"기본 화각 fx={fx_default:.3f} vs 실측 {fx_measured:.3f}"


def test_default_is_not_the_old_wrong_value():
    """
    🚨 71.5 로 되돌아가는 것을 막는다.

    그 값이면 화면 가장자리에서 각도가 4.4° 틀린다 — align_tol_deg(5°)의 88%다.
    에러가 안 나고 조용히 틀리는 종류라 테스트로 박아둔다.
    """
    assert DEFAULT_HFOV_DEG != LEGACY_HFOV_DEG
    edge_err = abs(angle_from_pixel(0, 640, DEFAULT_HFOV_DEG)
                   - angle_from_pixel(0, 640, LEGACY_HFOV_DEG))
    assert edge_err > 4.0, f"가장자리 차이가 {edge_err:.2f}° 밖에 안 된다 — 상수를 확인할 것"


def test_legacy_was_broken_at_other_resolutions():
    """
    🚨 이 테스트가 작년 코드의 숨은 결함을 고정한다.

    작년 식은 fx 가 **444.44px 로 고정**이다 — 해상도를 바꿔도 안 변한다.
    실제 카메라는 화각이 고정이고 fx 가 해상도에 비례한다. 즉 작년 코드는
    **640x480 에서만 맞았고, 해상도를 바꾸면 조용히 각도가 틀어졌다.**
    (현행 launch 는 rgb_camera.color_profile="640x480x30" 이라 실제로는 문제가 없었다.
     하지만 해상도 한 줄만 바꾸면 에러 없이 조향이 틀어지는 지뢰였다.)

    새 식은 width 를 매 프레임 읽어 fx 를 다시 계산하므로 이 지뢰가 없다.
    아래는 '얼마나 틀어졌을 것인가' 를 수치로 남기는 것이다.
    """
    # 640 에서는 완전 동일
    worst_640 = max(abs(angle_from_pixel(vx, 640, EXACT_LEGACY_HFOV_DEG)
                        - legacy_angle_from_pixel(vx, 640))
                    for vx in range(0, 641))
    assert worst_640 < 1e-9, f"현행 해상도에서는 완전 동일해야 한다: {worst_640}"

    # 다른 해상도에서는 작년 식이 틀린다
    for w in (320, 848, 1280):
        worst = max(abs(angle_from_pixel(vx, w, EXACT_LEGACY_HFOV_DEG)
                        - legacy_angle_from_pixel(vx, w))
                    for vx in range(0, w + 1, max(1, w // 200)))
        assert worst > 1.0, \
            f"width={w}: 작년 식이 여기서 틀렸어야 하는데 오차가 {worst:.3f}° 뿐"


def test_new_formula_is_resolution_invariant():
    """
    같은 화각·같은 '상대 위치' 면 해상도가 달라도 각도가 같아야 한다.
    (작년 식은 이게 안 됐다 — 위 테스트 참고)
    """
    for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
        angles = [angle_from_pixel(int(round(frac * w)), w, 71.5)
                  for w in (640, 1280)]
        assert abs(angles[0] - angles[1]) < 0.2, \
            f"frac={frac}: {angles} — 해상도에 따라 각도가 달라진다"


def test_wide_lens_changes_angle_a_lot():
    """
    광각으로 바꾸면 같은 픽셀이 다른 각도가 된다 — V3 를 지금 하는 이유.
    (하드코딩이었으면 카메라 교체일에 튜닝이 통째로 무효가 된다)
    """
    vx = 100
    narrow = angle_from_pixel(vx, 640, 71.5)
    wide = angle_from_pixel(vx, 640, 120.0)
    assert abs(wide - narrow) > 10.0, \
        f"71.5°→120° 인데 각도 차가 {abs(wide-narrow):.1f}° 뿐"


def main():
    print("=== vision_geom 테스트 ===")
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            check(name[5:], fn)
    print(f"\n{_passed}/{_total} 통과")
    return 0 if _passed == _total else 1


if __name__ == "__main__":
    sys.exit(main())
