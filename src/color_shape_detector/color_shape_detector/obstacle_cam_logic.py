"""카메라 장애물(빨간 누운 원통) → /camera_obstacle_state '가짜 LiDAR' 배열 — 순수 로직(ROS·OpenCV 없음).

근거: docs/문제와작업/계약서_카메라장애물_20261111.md U2 (Q1: 이 토픽에만 행 기반 거리 추정 예외).
토너먼트(mode 8) 원통은 수면 위 ~20 cm 라 LiDAR 빔이 못 볼 수 있다 → 카메라 bbox 로 방위·거리를 어림해
ship_direction (C) 회피에 min 병합한다(ship_direction/camera_merge.py). 노드(U4)와 시뮬(U3)이 **같은 함수**를 부른다.

입력은 ROS 없이 쓰기 쉬운 형태: bbox = (x_l, x_r, y_bottom) 픽셀(x 오른쪽 +, y 아래 +), 이미지 width·height,
파라미터 dict(DEFAULT_PARAMS 키). 출력 배열 형식(= /geofence_state):
    [angle_min_deg, angle_inc_deg, r0, r1, ...]  상대방위 0 = 정면, **좌 양수**(인덱스 0 = 우현),
    LiDAR 원점, 멀거나 없음 inf. 파라미터·이미지 크기 무효 → **빈 배열**(정보 없음).
    검출 없음 → 전부 inf(살아 있음).

거리 모델 — 평면 수면 핀홀(⚠️ 거칠다, 계약서 위험 4):
    fy = fx = (width/2)/tan(hfov/2)  (정사각 픽셀 가정 ⚠️), cy = height/2 (⚠️ camera_info 미확인)
    bbox 바닥 행 y 의 광선이 수평 아래로 δ = tilt + atan((y − cy)/fy) 만큼 내려간다.
    δ ≤ 0(수평선 위) → inf. 아니면 앞쪽 수평거리 Z = cam_height / tan(δ), 낸 값 = Z × range_scale.
    · 높이를 크게·틸트를 작게 잡으면 거리가 멀게 나온다 = 위험 쪽. range_scale(0.85)로 보수 편향.
    · Z 는 광축 방향 거리라 옆 방위에선 실제 반경거리(Z/cos b)보다 짧다(±32° 에서 15%) = 보수 쪽.
    · 방위는 틸트를 반영한 **정확식**(pixel_to_bearing_tilt, bbox 바닥 행 y 에서):
          xn = (x − cx)/fx,  yn = (y − cy)/fy,  방위 = −atan(xn / (cos t − yn·sin t))   (왼쪽 +)
      = 그 픽셀 광선이 수면에 닿는 점의 방위. 틸트 0 이면 vision_geom.angle_from_pixel 과 같다.
      (정정: 예전 문구 "틸트 11° 에서 차이 < 0.5%" 는 틀렸다 — 틀린 쪽 식(틸트 무시)은 화면 가장자리
       바닥 행(y 360)에서 2.3°(7%), 맨 위 행에서 −1.2° 어긋난다. vision_geom 은 다른 검출기 공용이라 안 바꾼다.)
      ⚠️ 바닥 가장자리 실제 방위(±34.3°)는 출력 격자(±hfov/2)를 넘는다 → 넘는 부분은 버려진다.
    · bbox 가 바닥(또는 ignore_bottom_frac 띠 경계)에 닿으면 실제 접점은 화면 밖(더 가까움) →
      그 경계 행의 거리를 낸다(볼 수 있는 가장 가까운 거리 = 실제 거리의 상한). 방위도 그 경계 행에서.

거짓 정지 대책(계약서 위험 2 — 석양·반사가 화면을 덮으면 후보 0 → 후진·정지):
    · 각폭(카메라 방위, 넓히기 전) > max_bbox_width_deg 인 bbox 는 버린다(그 방향 inf).
    · 한 프레임 bbox 수 > max_bboxes 면 **프레임 이상** → 빈 배열(병합 안 함). 확인-N 창은 '본 것 없음' 으로 흐른다.
    두 검사는 퓨전 경로(obstacle_fusion_logic.fused_detections_to_array)에도 같은 헬퍼로 걸린다.
"""

import math
import numbers
from collections import deque

from color_shape_detector.vision_geom import DEFAULT_HFOV_DEG, angle_from_pixel, fx_from_hfov

# 코드 기본값 — 계약서 '새 노드 image_subscriber_obstacle' 표의 안. ⚠️ 표시는 실측 필요.
DEFAULT_PARAMS = {
    "hfov_deg": DEFAULT_HFOV_DEG,     # vision.yaml /** 의 hfov_deg 를 상속(63.9, OAK 640×360)
    "cam_height_m": 0.35,             # ⚠️ 실측 필요 — 수면 위 카메라 광학 중심 높이
    "cam_tilt_deg": 11.0,             # ⚠️ 실측 필요 — 아래로 숙인 각(+ = 아래)
    "range_scale": 0.85,              # ⚠️ 실측 필요 — 보수 편향(1 보다 작으면 가깝게 낸다)
    "cam_x_from_lidar_m": 0.0,        # ⚠️ 실측 필요 — 카메라가 LiDAR 보다 앞(+)
    "cam_y_from_lidar_m": 0.0,        # ⚠️ 실측 필요 — 카메라가 LiDAR 보다 왼쪽(+)
    "max_range_m": 5.0,               # 넘으면 inf (DWA R_MAX_M 과 같은 값)
    "min_range_out_m": 0.5,           # 내는 거리의 하한
    "min_sector_deg": 3.0,            # 좁은 검출을 이 폭으로 넓혀 낸다(smooth_spikes 1.5° 에 안 지워지게)
    "out_angle_inc_deg": 0.5,         # 출력 배열 각 간격
    "ignore_bottom_frac": 0.0,        # ⚠️ 실측 필요 — 화면 아래 이 비율은 자기 선수(노드가 가림)
    "max_bbox_width_deg": 25.0,       # ⚠️ 실측 필요 — bbox 하나의 최대 각폭. 넘으면 그 bbox 버림(거짓 정지 대책)
    "max_bboxes": 4,                  # ⚠️ 실측 필요 — 🚨 정수. 프레임당 bbox 가 이보다 많으면 빈 배열(프레임 이상)
}
MAX_BBOXES_LIMIT = 50          # max_bboxes 허용 상한

DEFAULT_CONFIRM_FRAMES = 3     # 🚨 정수 — 최근 window 프레임 중 이만큼 보이면 확인(계약서 Q13)
DEFAULT_CONFIRM_WINDOW = 5     # 🚨 정수
DEFAULT_CONFIRM_MATCH_DEG = 10.0   # ⚠️ 같은 물체로 볼 방위 차(30°/s 선회 × 5 프레임 근처)

BOTTOM_TOUCH_PX = 1.0          # bbox 바닥이 경계에서 이 픽셀 안이면 '닿음'

# 검증 범위 — (최소, 최대, 최소 포함, 최대 포함). 이 밖이면 빈 배열(모르면 입 다문다).
_RANGES = {
    "hfov_deg": (0.0, 180.0, False, False),
    "cam_height_m": (0.0, 5.0, False, True),
    "cam_tilt_deg": (-45.0, 80.0, True, True),
    "range_scale": (0.0, 2.0, False, True),
    "cam_x_from_lidar_m": (-2.0, 2.0, True, True),
    "cam_y_from_lidar_m": (-2.0, 2.0, True, True),
    "max_range_m": (0.0, 50.0, False, True),
    "min_range_out_m": (0.0, 50.0, True, True),
    "min_sector_deg": (0.0, 90.0, True, True),
    "out_angle_inc_deg": (0.0, 10.0, False, True),
    "ignore_bottom_frac": (0.0, 0.9, True, True),
    "max_bbox_width_deg": (0.0, 180.0, False, True),
}


def _num(v):
    """bool·문자열이 아닌 유한 실수(numbers.Real — numpy 수 포함)면 float, 아니면 None."""
    if isinstance(v, bool) or not isinstance(v, numbers.Real):
        return None
    f = float(v)
    return f if math.isfinite(f) else None


def check_params(p):
    """파라미터 dict → 오류 문자열 리스트(빈 리스트 = 정상). 빠진 키·비유한·bool·범위 밖 = 오류."""
    errs = []
    for k, (lo, hi, lo_in, hi_in) in _RANGES.items():
        if k not in p:
            errs.append(f"{k} 없음")
            continue
        v = _num(p[k])
        if v is None:
            errs.append(f"{k}={p[k]!r} 유한 실수 아님")
            continue
        ok_lo = v >= lo if lo_in else v > lo
        ok_hi = v <= hi if hi_in else v < hi
        if not (ok_lo and ok_hi):
            errs.append(f"{k}={v} 범위 밖({lo}~{hi})")
    mb = p.get("max_bboxes")
    if isinstance(mb, bool) or not isinstance(mb, int):
        errs.append(f"max_bboxes={mb!r} 정수 아님")
    elif not (1 <= mb <= MAX_BBOXES_LIMIT):
        errs.append(f"max_bboxes={mb} 범위 밖(1~{MAX_BBOXES_LIMIT})")
    if not errs:
        if float(p["min_range_out_m"]) >= float(p["max_range_m"]):
            errs.append(f"min_range_out_m={p['min_range_out_m']} ≥ max_range_m={p['max_range_m']}")
        if float(p["min_sector_deg"]) >= float(p["hfov_deg"]):
            errs.append(f"min_sector_deg={p['min_sector_deg']} ≥ hfov_deg={p['hfov_deg']}")
    return errs


def _size_ok(width, height):
    return _num(width) is not None and _num(height) is not None and width > 0 and height > 0


# ---------------------------------------------------------------- (1) 픽셀 x → 상대방위
def pixel_to_bearing(x, width, hfov_deg):
    """이미지 x → 상대방위[deg], 왼쪽 +(vision_geom.angle_from_pixel 그대로). 무효 입력 → None."""
    x = _num(x)
    if x is None or not _size_ok(width, 1):
        return None
    try:
        return angle_from_pixel(x, width, hfov_deg)
    except (ValueError, TypeError, ZeroDivisionError):
        return None


def pixel_to_bearing_tilt(x, y, width, height, hfov_deg, tilt_deg):
    """이미지 (x, y) → 그 광선이 수면에 닿는 점의 상대방위[deg], 왼쪽 +. 틸트 반영 정확식.

        fx = fy = fx_from_hfov(width, hfov) (정사각 픽셀 ⚠️), cx = width // 2(angle_from_pixel 과 같게),
        cy = height / 2 (⚠️ camera_info 미확인 — row_to_range 와 같은 가정)
        xn = (x − cx)/fx,  yn = (y − cy)/fy,  t = tilt(아래 +)
        방위 = −atan(xn / (cos t − yn·sin t))
    유도: 카메라 광선 (xn, yn, 1) 을 수평 프레임으로 돌리면 앞 = cos t − yn·sin t, 오른쪽 = xn.
    틸트 0 → −atan(xn) = angle_from_pixel. 무효 입력·광선이 뒤/수직(분모 ≤ 0) → None.
    """
    x, y, t = _num(x), _num(y), _num(tilt_deg)
    if x is None or y is None or t is None or not _size_ok(width, height):
        return None
    try:
        f = fx_from_hfov(width, hfov_deg)
    except (ValueError, TypeError, ZeroDivisionError):
        return None
    tr = math.radians(t)
    xn = (x - width // 2) / f
    yn = (y - height / 2.0) / f
    den = math.cos(tr) - yn * math.sin(tr)
    if den <= 0.0:
        return None
    return -math.degrees(math.atan(xn / den))


# ---------------------------------------------------------------- (2) 바닥 행 y → 거리
def row_to_range(y, width, height, p):
    """이미지 행 y(아래 +) → 수면 위 앞쪽 수평거리[m] × range_scale. 수평선 위(δ ≤ 0) → inf.

    파라미터·이미지 크기·y 무효 → None. 단조: y 가 클수록(아래) 가깝다.
    """
    y = _num(y)
    if y is None or not _size_ok(width, height) or check_params(p):
        return None
    try:
        fy = fx_from_hfov(width, float(p["hfov_deg"]))
    except (ValueError, ZeroDivisionError):
        return None
    cy = height / 2.0
    delta = math.radians(float(p["cam_tilt_deg"])) + math.atan((y - cy) / fy)
    if delta <= 0.0:
        return math.inf
    if delta >= math.pi / 2.0:
        return 0.0
    return float(p["cam_height_m"]) / math.tan(delta) * float(p["range_scale"])


def horizon_row(width, height, p):
    """수평선 행(δ = 0). 이 행 위(작은 y)의 bbox 바닥은 거리 inf. 무효 → None."""
    if not _size_ok(width, height) or check_params(p):
        return None
    fy = fx_from_hfov(width, float(p["hfov_deg"]))
    return height / 2.0 - fy * math.tan(math.radians(float(p["cam_tilt_deg"])))


def bbox_range(y_bottom, width, height, p):
    """bbox 바닥 행 → 카메라 원점 거리. 바닥(또는 ignore_bottom_frac 경계)에 닿으면 그 경계 행 거리.

    경계 행 = height × (1 − ignore_bottom_frac). 닿음 = y_bottom ≥ 경계 − BOTTOM_TOUCH_PX.
    무효 → None, 수평선 위 → inf.
    """
    y = _num(y_bottom)
    if y is None or not _size_ok(width, height) or check_params(p):
        return None
    return row_to_range(_bottom_row(y, height, p), width, height, p)


def _bottom_row(y, height, p):
    """bbox 바닥 행 → 실제로 쓸 행: 경계(height × (1 − ignore_bottom_frac))에 닿으면 경계 행."""
    edge = height * (1.0 - float(p["ignore_bottom_frac"]))
    return edge if y >= edge - BOTTOM_TOUCH_PX else y


# ---------------------------------------------------------------- (3) 카메라 원점 → LiDAR 원점
def cam_to_lidar(bearing_deg, range_m, cam_x_m, cam_y_m):
    """카메라 원점 (방위 deg 좌 +, 거리 m) → LiDAR 원점 (방위, 거리). x 앞 +, y 왼쪽 +.

    카메라 광축은 LiDAR 정면과 평행하다고 본다(⚠️ 장착 요 오프셋 미확인). 거리 inf → (방위 그대로, inf).
    """
    if not math.isfinite(range_m):
        return float(bearing_deg), math.inf
    b = math.radians(bearing_deg)
    x = cam_x_m + range_m * math.cos(b)
    y = cam_y_m + range_m * math.sin(b)
    return math.degrees(math.atan2(y, x)), math.hypot(x, y)


# ---------------------------------------------------------------- (4) bbox 목록 → 가짜 LiDAR 배열
def out_grid(p):
    """출력 격자 → (angle_min_deg, inc_deg, 칸 수). 화각 ±hfov/2 안의 inc 배수만(0 을 지난다)."""
    inc = float(p["out_angle_inc_deg"])
    n_half = int(math.floor((float(p["hfov_deg"]) / 2.0) / inc + 1e-9))
    return -n_half * inc, inc, 2 * n_half + 1


def _bbox_cam(bbox, width, height, p):
    """bbox (x_l, x_r, y_bottom) → (b_left, b_center, b_right, y_row) 카메라 원점 방위[deg](좌 +), 또는 None.

    x 는 [0, width] 로 자르고, 방위는 바닥 행(경계에 닿으면 경계 행) y_row 에서 정확식(pixel_to_bearing_tilt).
    None: 무효 bbox(숫자 아님·x_r < x_l·모양 틀림) / 방위 계산 불가 /
          **각폭 b_left − b_right > max_bbox_width_deg**(거짓 정지 대책 — 화면을 덮는 반사·석양).
    파라미터는 호출자가 이미 검사했다고 본다.
    """
    try:
        x_l, x_r, y_b = (_num(v) for v in bbox)
    except (TypeError, ValueError):
        return None
    if x_l is None or x_r is None or y_b is None or x_r < x_l:
        return None
    x_l = min(max(x_l, 0.0), float(width))
    x_r = min(max(x_r, 0.0), float(width))
    y_row = _bottom_row(y_b, height, p)
    hfov, tilt = float(p["hfov_deg"]), float(p["cam_tilt_deg"])
    bs = [pixel_to_bearing_tilt(x, y_row, width, height, hfov, tilt) for x in (x_l, (x_l + x_r) / 2.0, x_r)]
    if any(b is None for b in bs):
        return None
    if bs[0] - bs[2] > float(p["max_bbox_width_deg"]):
        return None
    return bs[0], bs[1], bs[2], y_row


def bbox_center_bearing(bbox, width, height, p):
    """bbox 가운데 x 의 카메라 방위[deg](바닥 행·틸트 정확식). 무효·각폭 초과 → None. (확인-N 짝짓기용)"""
    cam = _bbox_cam(bbox, width, height, p)
    return None if cam is None else cam[1]


def _finish_sector(pts, p):
    """LiDAR 원점 [(방위, 거리)] → (lo, hi, r) 또는 None(r > max_range_m).

    r = 최소(하한 min_range_out_m), 폭이 min_sector_deg 미만이면 가운데 기준으로 넓힌다.
    """
    bs = [b for b, _ in pts]
    r = min(rr for _, rr in pts)
    if not (r <= float(p["max_range_m"])):
        return None
    r = max(r, float(p["min_range_out_m"]))
    lo, hi = min(bs), max(bs)
    ms = float(p["min_sector_deg"])
    if hi - lo < ms:
        c = (lo + hi) / 2.0
        lo, hi = c - ms / 2.0, c + ms / 2.0
    return lo, hi, r


def _bbox_sector(bbox, width, height, p):
    """bbox → (lo_deg, hi_deg, r_m) LiDAR 원점(행 기반 거리), 또는 None(무효·각폭 초과·수평선 위·max_range 초과)."""
    cam = _bbox_cam(bbox, width, height, p)
    if cam is None:
        return None
    r_cam = row_to_range(cam[3], width, height, p)
    if r_cam is None or not math.isfinite(r_cam):
        return None
    cx_m, cy_m = float(p["cam_x_from_lidar_m"]), float(p["cam_y_from_lidar_m"])
    return _finish_sector([cam_to_lidar(b, r_cam, cx_m, cy_m) for b in cam[:3]], p)


def _grid_array(sectors, p):
    """부채꼴 목록(None 은 건너뜀) → [angle_min_deg, inc_deg, r0, ...]. 칸 각이 [lo, hi] 안이면 그 칸에 min.

    부채꼴이 칸 사이에 끼어 한 칸도 안 덮으면 가운데에 가장 가까운 칸 하나. 격자(화각 범위) 밖 부분은 버린다.
    """
    a0, inc, n = out_grid(p)
    rs = [math.inf] * n
    for sec in sectors:
        if sec is None:
            continue
        lo, hi, r = sec
        k_lo = max(0, int(math.ceil((lo - a0) / inc - 1e-9)))
        k_hi = min(n - 1, int(math.floor((hi - a0) / inc + 1e-9)))
        if k_lo > k_hi:
            k = int(round(((lo + hi) / 2.0 - a0) / inc))
            if not (0 <= k < n):
                continue
            k_lo = k_hi = k
        for k in range(k_lo, k_hi + 1):
            if r < rs[k]:
                rs[k] = r
    return [float(a0), float(inc)] + rs


def _frame_boxes(bboxes, width, height, p, confirm=None):
    """프레임 공통 앞단(process_frame·퓨전 공용) → 확인-N 을 통과한 bbox 목록, 또는 None(→ 빈 배열).

    · 파라미터·이미지 크기 무효 → None, 창은 건드리지 않음
    · bbox 수 > max_bboxes → None(프레임 이상). 창은 update([]) — 프레임은 지나갔고 믿을 검출은 없다
    · confirm 있으면 가운데 방위로 창 갱신, 확인된 것만. 없으면 전부
    """
    if not _size_ok(width, height) or check_params(p):
        return None
    boxes = list(bboxes)
    if len(boxes) > p["max_bboxes"]:
        if confirm is not None:
            confirm.update([])
        return None
    if confirm is not None:
        ok = confirm.update([bbox_center_bearing(b, width, height, p) for b in boxes])
        boxes = [b for b, keep in zip(boxes, ok) if keep]
    return boxes


def detections_to_array(bboxes, width, height, p):
    """bbox 목록 → [angle_min_deg, inc_deg, r0, ...] (LiDAR 원점, 좌 양수, 인덱스 0 = 우현). 확인-N 없음.

    · 파라미터·이미지 크기 무효, bbox 수 > max_bboxes → [] (빈 배열 = 정보 없음)
    · 검출 없음 / 수평선 위 / max_range_m 초과 / 무효 bbox / 각폭 > max_bbox_width_deg → 그 bbox 는 안 냄(inf)
    · 각 bbox: 좌·가운데·우 끝 방위(바닥 행 정확식)를 LiDAR 원점으로 바꾼 부채꼴 [lo, hi](min_sector_deg 미만이면
      가운데 기준으로 넓힘), 거리 = 세 점 중 최소(하한 min_range_out_m). 칠하기는 _grid_array.
    """
    boxes = _frame_boxes(bboxes, width, height, p)
    if boxes is None:
        return []
    return _grid_array((_bbox_sector(b, width, height, p) for b in boxes), p)


# ---------------------------------------------------------------- (5) 확인-N 창
def check_confirm(frames, window, match_deg):
    """확인-N 파라미터 → 오류 문자열 리스트. frames·window 는 정수(bool·소수 금지), 1 ≤ frames ≤ window ≤ 30."""
    errs = []
    for name, v in (("confirm_frames", frames), ("confirm_window", window)):
        if isinstance(v, bool) or not isinstance(v, int):
            errs.append(f"{name}={v!r} 정수 아님")
    if not errs and not (1 <= frames <= window <= 30):
        errs.append(f"confirm_frames={frames}, confirm_window={window}: 1 ≤ frames ≤ window ≤ 30 아님")
    m = _num(match_deg)
    if m is None or not (0.0 < m <= 90.0):
        errs.append(f"confirm_match_deg={match_deg!r} 범위 밖(0~90)")
    return errs


class ConfirmWindow:
    """최근 window 프레임 중 confirm_frames 프레임 이상 '비슷한 방위'(±match_deg)에 보인 검출만 통과.

    단순하게: 프레임마다 검출 방위 목록을 저장하고, 이번 검출 b 에 대해 저장된 프레임(이번 포함) 중
    |b − x| ≤ match_deg 인 x 가 하나라도 있는 프레임 수를 센다. 물체 추적·ID 없음.
    🚨 처리한 프레임마다 **검출이 없어도** update([]) 를 불러야 창이 흐른다. 담당 모드 밖·영상 공백 뒤엔 reset().
    confirm_frames 1 = 끔(모두 통과).
    """

    def __init__(self, confirm_frames=DEFAULT_CONFIRM_FRAMES, confirm_window=DEFAULT_CONFIRM_WINDOW,
                 match_deg=DEFAULT_CONFIRM_MATCH_DEG):
        errs = check_confirm(confirm_frames, confirm_window, match_deg)
        if errs:
            raise ValueError("; ".join(errs))
        self.confirm_frames = confirm_frames
        self.match_deg = float(match_deg)
        self._hist = deque(maxlen=confirm_window)

    def reset(self):
        self._hist.clear()

    def update(self, bearings):
        """이번 프레임 검출 방위 목록 → 같은 길이의 bool 목록(True = 확인됨). None·비유한 방위는 False."""
        cur = [_num(b) for b in bearings]
        self._hist.append([b for b in cur if b is not None])
        out = []
        for b in cur:
            if b is None:
                out.append(False)
                continue
            seen = sum(1 for fr in self._hist if any(abs(b - x) <= self.match_deg for x in fr))
            out.append(seen >= self.confirm_frames)
        return out


def process_frame(bboxes, width, height, p, confirm=None):
    """한 프레임: (개수 검사 → 확인-N 창 →) 배열. 노드(U4)와 시뮬(U3)이 같은 경로를 쓰게 묶은 것.

    파라미터 무효 → [] (창은 건드리지 않음). bbox 수 > max_bboxes → [] (창은 '본 것 없음' 으로 흐름).
    confirm None 이면 확인 없이 전부 낸다. LiDAR 퓨전판은 obstacle_fusion_logic.fused_detections_to_array.
    """
    boxes = _frame_boxes(bboxes, width, height, p, confirm)
    if boxes is None:
        return []
    return _grid_array((_bbox_sector(b, width, height, p) for b in boxes), p)
