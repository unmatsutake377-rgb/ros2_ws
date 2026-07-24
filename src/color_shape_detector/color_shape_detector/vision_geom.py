"""
비전 기하 — 픽셀 오프셋 → 방위각. ROS/OpenCV 비의존 순수 함수.

왜 다시 썼나 (V3, CLAUDE.md 3-3):
  작년 코드는 이랬다.
      rel_x  = vX - cx
      real_x = (rel_x / 80.0) * 0.09 * (distance / 0.5)
      angle  = -degrees(atan2(real_x, distance))
  `distance` 가 분자·분모에서 **약분**되어 실질은 `angle = -degrees(atan(rel_x * 0.00225))` 다.
  즉 뎁스를 쓰는 것처럼 보이지만 실제로는 **화각이 매직넘버 3개(80.0, 0.09, 0.5)에 흩어져
  하드코딩**돼 있었다. 역산하면:
      k  = (1/80) * 0.09 / 0.5 = 0.00225
      fx = 1/k ≈ 444.44 px
      HFOV(640px) = 2·atan(320/fx) ≈ 71.51°   ← RealSense 화각
  카메라를 광각(OAK-1 W 등)으로 바꾸면 **같은 픽셀이 다른 각도**가 되어 모든 각도 출력과
  상위 튜닝(align_tol_deg, pair_min/max_sep_deg …)이 통째로 무효가 된다.
  → 화각을 파라미터로 드러내고, 해상도는 매 프레임 실제 이미지에서 읽는다.

⚠️ 핀홀 모델이다. 광각 렌즈에서는 **가장자리 오차가 커진다**(왜곡).
   광각 카메라를 붙이면 rectified 토픽을 구독하거나 camera_info 의 왜곡계수 D 를
   cv2.undistortPoints 로 적용해야 한다. 이 함수만으로는 부족하다.
"""

import math

# 작년 매직넘버의 유효 상수. 회귀 비교의 기준으로만 남긴다 — 계산에는 쓰지 않는다.
LEGACY_PIXEL_TO_ANGLE_K = (1.0 / 80.0) * 0.09 / 0.5     # = 0.00225

# 위 상수를 640px 기준으로 역산한 화각. 기본값은 읽기 좋게 71.5 로 둔다.
# 완전 등가값은 71.5077745…이고, 71.5 로 반올림했을 때 640px 전 구간 최대 오차는
# **0.0039°** 다. 중심부 1픽셀이 0.129° 이므로 픽셀의 3% — 실질 동작 불변이다.
DEFAULT_HFOV_DEG = 71.5
EXACT_LEGACY_HFOV_DEG = 71.5077745088735


def fx_from_hfov(image_width, hfov_deg):
    """
    수평 화각 → 초점거리[px].

        fx = (width/2) / tan(hfov/2)

    image_width 는 **파라미터로 두지 않는다.** 해상도를 바꾸면 fx 가 따라 바뀌어야 하는데,
    파라미터로 두면 yaml 을 안 고친 채 해상도만 바꿨을 때 조용히 각도가 틀어진다.
    """
    if not (image_width and image_width > 0):
        raise ValueError(f"image_width 가 이상하다: {image_width!r}")
    if not (0.0 < hfov_deg < 180.0):
        raise ValueError(f"hfov_deg 는 0~180 사이여야 한다: {hfov_deg!r}")
    return (image_width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)


def angle_from_pixel(vX, image_width, hfov_deg):
    """
    이미지 x 좌표 → 상대 방위각[deg].

    부호 규약은 작년과 동일하다 — **왼쪽이 +**:
        화면 왼쪽(vX < 중심) → 양수
        화면 중앙            → 0
        화면 오른쪽          → 음수
    (`/candidate_angle` 계열과 같은 규약. 바꾸면 조향이 반대로 돈다.)
    """
    cx = image_width // 2
    fx = fx_from_hfov(image_width, hfov_deg)
    return -math.degrees(math.atan((vX - cx) / fx))


def legacy_angle_from_pixel(vX, image_width):
    """
    작년 식(약분 후 등가형). **회귀 비교 전용** — 실제 경로에서 쓰지 마라.
    새 식이 이것과 같은 값을 내는지 테스트가 확인한다.
    """
    cx = image_width // 2
    return -math.degrees(math.atan((vX - cx) * LEGACY_PIXEL_TO_ANGLE_K))
