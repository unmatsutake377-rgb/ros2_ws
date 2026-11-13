"""도리도리 회피 검출기(image_subscriber_avoid) 순수 로직 — **ROS 비의존** (rclpy 임포트 없음).

계약서: docs/문제와작업/계약서_도리도리회피_v2_20261114.md U3 (=C6a, 비전 원자 그룹)
        + G1 (a) 형식 · U1 리뷰 메모 W3(age_s ≥ 0 clamp) · U2 리뷰 메모(mode 5 진입부터 하트비트)

    from color_shape_detector.avoid_detect_logic import (
        intrinsics_from_camera_info, detect_blobs, blobs_to_detections, frame_age_s, pack_detections)

하는 일
  OAK 컬러 프레임에서 빨강(낮은 장애물 후보)·노랑(부표) 윤곽을 **여러 개** 찾아,
  ship_avoid 가 읽는 /avoid_detections 한 줄로 만든다. 거리(range_m)는 U3 에선 늘 NaN(= 모름) —
  D455 창 거리는 U4. ship_avoid core 가 NaN 이면 단안(수면 기준) 거리로 대신한다.

/avoid_detections (Float32MultiArray) — G1 (a). ship_avoid avoid_mode_logic.parse_detections 와 **정확히** 같다.
  [VER=1, STRIDE=6, N, N × (cls, bearing_left_deg, v_norm, range_m, conf, age_s)]
    cls          0 = 빨강, 2 = 노랑 (/fused_objects color_id 와 **다른 번호** — 섞지 말 것)
    bearing      좌 +(deg) = −atan((u − cx)/fx). u = 윤곽 질량중심 x(px). fx·cx 는 camera_info K
    v_norm       (v_bottom − cy)/fy, 아래 +. v_bottom = 윤곽 맨 아래 행(px, 수면 접점 근사)
    range_m      NaN (U3). U4 에서 D455 창 거리
    conf         0~1 (아래 conf_score)
    age_s        촬영(header.stamp) → 발행까지 초. stamp 를 못 쓰면 이 노드 처리 시간. **0 이상으로 clamp**
                 (ship_avoid 는 age_s < 0 이 하나라도 있으면 메시지 전체를 형식 오류로 버린다 — W3)
  · 본 것이 없어도 [1, 6, 0] 을 낸다(하트비트) — ship_avoid 는 N=0 을 '살아 있음' 으로 센다.
  · 🚨 형식 상수(VER·STRIDE·머리 길이·cls 번호)는 ship_avoid 의 **사본**이다(§1-8 상수 공유 모듈화 금지).
    test_avoid_detect_logic 이 ship_avoid avoid_mode_logic 을 파일 경로로 읽어 값 일치·왕복을 검사한다.

conf (0~1) 근거 — 지금 ship_avoid core 는 conf 를 **읽지 않는다**(core.add_detections 가 받기만 함).
  그래서 값의 쓰임새는 U5/U6 의 LowMap R 가중에서 정한다. 지금은 단조·유계인 간단한 점수로만 둔다:
    conf = min(1, area / conf_full_area_px) × solidity
  · 면적 항: min_area_px 근처의 작은 덩어리가 노이즈일 가능성이 가장 크다(08-12 실측: 41 px² 노이즈가
    부표로 통과 — vision.yaml min_area_px 주석). 픽셀이 많을수록 색 판정이 안정적이다.
  · solidity(면적/볼록껍질): 윤슬·물보라 조각은 들쭉날쭉해 낮고, 부표(원통·구)는 1 에 가깝다
    (gate 검출기 require_shape=false 경로와 같은 지표, vision_geom.solidity).
  ⚠️ 실측 필요: conf_full_area_px(기본 400 px²)는 근거 있는 값이 아니다 — 실영상(rosbag) 으로 정한다.

왜곡: 핀홀만 쓴다(K). OAK-1 표준 화각이라 10-31 벤치 가장자리 오차 0.9° 안(vision.yaml hfov_deg 주석).
"""

import math

from color_shape_detector.vision_geom import solidity as _solidity

# ---- /avoid_detections 형식 — ship_avoid/avoid_mode_logic.py 의 사본(일치는 테스트가 검사) ----
DET_VER = 1
DET_STRIDE = 6
DET_HEADER = 3                      # VER, STRIDE, N

CLS_RED = 0
CLS_YELLOW = 2
# 이 검출기가 찾는 색 → cls. 색 이름은 hsv_ranges / vision.yaml hsv.<색> 키와 같다.
COLOR_CLS = {'red': CLS_RED, 'yellow': CLS_YELLOW}

RANGE_UNKNOWN = float('nan')        # U3: D455 거리 없음 → NaN (지어내지 않는다)

# 노드 declare_parameter 리터럴 = vision.yaml image_subscriber_avoid 절 = 아래 값(test_avoid_detect_logic 이 묶는다).
#   잘못된 값일 때 돌아갈 값이기도 하다. ⚠️ 실측 필요(전부).
DEFAULT_PARAMS = {
    'min_area_px': 40.0,
    'min_solidity': 0.7,
    'conf_full_area_px': 400.0,
    'max_detections': 16,
}


def _pos_finite(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) and v > 0.0


def check_params(raw):
    """부팅 검사. raw = {이름: yaml 값} → (쓸 값 dict, 오류 메시지 목록). 범위 밖이면 그 값만 기본값으로.

      min_area_px        > 0 유한(0 이하면 노이즈 한 점도 검출로 나간다)
      min_solidity       0 ≤ x ≤ 1 (solidity 정의역. 1 넘으면 모든 윤곽을 버려 '본 것 없음' 만 낸다)
      conf_full_area_px  > 0 유한(0 이면 conf 가 늘 0)
      max_detections     정수 ≥ 1 (0 이면 늘 N=0 — 살아 있는데 아무것도 안 보는 침묵 고장)
    빠진 키는 기본값(오류 아님).
    """
    out, errs = {}, []

    def bad(name, why):
        d = DEFAULT_PARAMS[name]
        errs.append(f"🚨 {name}={raw.get(name)!r} 는 잘못된 값({why}) → {d!r} 로 돈다. "
                    f"vision.yaml image_subscriber_avoid 절 확인.")
        out[name] = d

    for name in ('min_area_px', 'conf_full_area_px'):
        v = raw.get(name, DEFAULT_PARAMS[name])
        if _pos_finite(v):
            out[name] = float(v)
        else:
            bad(name, '0 보다 큰 유한수')
    v = raw.get('min_solidity', DEFAULT_PARAMS['min_solidity'])
    if isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) and 0.0 <= v <= 1.0:
        out['min_solidity'] = float(v)
    else:
        bad('min_solidity', '0~1')
    v = raw.get('max_detections', DEFAULT_PARAMS['max_detections'])
    if isinstance(v, int) and not isinstance(v, bool) and v >= 1:
        out['max_detections'] = v
    else:
        bad('max_detections', '1 이상 정수')
    return out, errs


def intrinsics_from_camera_info(k, width, height):
    """CameraInfo K(행 우선 3×3, 9개)·width·height → (fx, fy, cx, cy) 또는 None(못 씀).

    None 이면 노드는 **발행하지 않는다**(방위를 지어내지 않음 — §1-6). 조건:
      K 가 9개 숫자, fx·fy 양의 유한수, cx·cy 유한수, width·height 양의 정수.
    (OAK 는 캘리브 없으면 K 가 전부 0 일 수 있다 → None)
    """
    try:
        vals = [float(v) for v in k]
        w, h = int(width), int(height)
    except (TypeError, ValueError):
        return None
    if len(vals) != 9 or w <= 0 or h <= 0:
        return None
    fx, cx, fy, cy = vals[0], vals[2], vals[4], vals[5]
    if not all(math.isfinite(v) for v in (fx, fy, cx, cy)):
        return None
    if fx <= 0.0 or fy <= 0.0:
        return None
    return fx, fy, cx, cy


def bearing_left_deg(u, fx, cx):
    """픽셀 열 u → 방위(deg, **좌 +**). vision_geom.angle_from_pixel 과 같은 부호, fx·cx 는 camera_info."""
    return -math.degrees(math.atan((u - cx) / fx))


def v_norm_of(v_bottom, fy, cy):
    """픽셀 행 v_bottom → (v_bottom − cy)/fy (아래 +). ship_avoid core.ground_plane_range 입력."""
    return (v_bottom - cy) / fy


def conf_score(area, sol, conf_full_area_px):
    """min(1, area/conf_full_area_px) × solidity, 0~1 로 자름. 근거는 모듈 docstring."""
    if not (conf_full_area_px > 0.0) or sol is None:
        return 0.0
    a = min(1.0, max(0.0, area / conf_full_area_px))
    return min(1.0, max(0.0, a * sol))


def frame_age_s(now_s, stamp_s, proc_s):
    """촬영 → 발행 초. → (age_s ≥ 0, 원래 값, 출처 'stamp'|'proc').

    now_s   : 발행 순간의 ROS 시각(s) — header.stamp 와 같은 시계(노드 clock)
    stamp_s : 이미지 header.stamp(s). 0·None·비유한이면 못 쓴다 → proc_s(이 노드 처리 시간)
    proc_s  : 콜백 시작 → 발행 직전(time.monotonic 차)
    원래 값이 음수(카메라 시각이 노트북보다 앞섬)면 0 으로 clamp — ship_avoid 가 메시지를 통째로 버리지 않게(W3).
    호출자는 raw < 0 이면 WARN 으로 드러낸다(시계 어긋남을 숨기지 않음).
    """
    src = 'proc'
    raw = proc_s
    if stamp_s is not None and now_s is not None:
        try:
            st, nw = float(stamp_s), float(now_s)
        except (TypeError, ValueError):
            st = nw = float('nan')
        if math.isfinite(st) and math.isfinite(nw) and st > 0.0:
            raw, src = nw - st, 'stamp'
    try:
        raw = float(raw)
    except (TypeError, ValueError):
        raw = float('nan')
    if not math.isfinite(raw):
        return 0.0, raw, src
    return max(0.0, raw), raw, src


def detect_blobs(bgr, color_ranges, min_area_px, min_solidity, max_detections):
    """BGR 프레임 → 윤곽 목록 [{cls, color, u, v_bottom, area, solidity, contour}] (면적 큰 순, 최대 max_detections).

    color_ranges: {색: [(lo, hi), …]} — hsv_ranges.load 가 돌려주는 모양. COLOR_CLS 에 없는 색은 무시.
    색마다 마스크(범위 OR) → medianBlur 3 → 바깥 윤곽(gate 검출기와 같은 전처리) →
      면적 < min_area_px 버림 → solidity < min_solidity 버림(윤슬·물보라 조각) → 질량중심 u, 맨 아래 행 v_bottom.
    🚨 cv2·numpy 는 여기서만 쓴다(나머지 함수는 cv2 없이 테스트된다).
    """
    import cv2
    import numpy as np

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    out = []
    for color, ranges in color_ranges.items():
        cls = COLOR_CLS.get(color)
        if cls is None:
            continue
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lo, hi in ranges:
            mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
        mask = cv2.medianBlur(mask, 3)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = float(cv2.contourArea(cnt))
            if area < min_area_px:
                continue
            sol = _solidity(area, float(cv2.contourArea(cv2.convexHull(cnt))))
            if sol is None or sol < min_solidity:
                continue
            m = cv2.moments(cnt)
            if m['m00'] == 0:
                continue
            u = m['m10'] / m['m00']
            _x, y, _w, h = cv2.boundingRect(cnt)
            out.append({'cls': cls, 'color': color, 'u': float(u), 'v_bottom': float(y + h - 1),
                        'area': area, 'solidity': float(sol), 'contour': cnt})
    out.sort(key=lambda b: -b['area'])
    return out[:max(0, int(max_detections))]


def blobs_to_detections(blobs, intr, age_s, conf_full_area_px):
    """윤곽 목록 + (fx, fy, cx, cy) → [(cls, bearing_left_deg, v_norm, range_m=NaN, conf, age_s)]."""
    fx, fy, cx, cy = intr
    return [(b['cls'], bearing_left_deg(b['u'], fx, cx), v_norm_of(b['v_bottom'], fy, cy), RANGE_UNKNOWN,
             conf_score(b['area'], b['solidity'], conf_full_area_px), age_s) for b in blobs]


def _finite(v):
    return isinstance(v, (int, float)) and math.isfinite(v)


def pack_detections(dets):
    """[(cls, bearing, v_norm, range_m, conf, age_s)] → (data 리스트, 버린 수).

    빈 목록 → [1.0, 6.0, 0.0] (하트비트). cls 가 COLOR_CLS 값이 아니거나 bearing·v_norm·conf·age_s 가
    유한수가 아니거나 age_s < 0 인 검출은 **그 검출만** 빼고 N 을 맞춘다(ship_avoid 가 메시지 전체를 버리지 않게).
    range_m 은 유한하지 않으면 NaN 으로 낸다.
    """
    rows, dropped = [], 0
    valid_cls = set(COLOR_CLS.values())
    for d in dets:
        cls, brg, vn, rng, conf, age = d
        if (cls not in valid_cls or not all(_finite(v) for v in (brg, vn, conf, age)) or age < 0.0):
            dropped += 1
            continue
        rng = float(rng) if _finite(rng) else RANGE_UNKNOWN
        rows.append((float(cls), float(brg), float(vn), rng, float(conf), float(age)))
    data = [float(DET_VER), float(DET_STRIDE), float(len(rows))]
    for r in rows:
        data.extend(r)
    return data, dropped
