"""위치유지 남색 부표 검출기(image_subscriber_stop) 순수 로직 — **ROS 비의존** (rclpy 임포트 없음).

계약서 v2 (위치유지 사각형, ae9063a1 기준) 변경 단위 (a) — 비전 검출기–미션 원자 묶음.
  Davidson 답 v2: 검출기 출력 = 새 토픽 /buoy_bearings, 형식은 main 다중 배열 규약 [VER, STRIDE, N, …].

    from color_shape_detector.stop_detect_logic import (
        check_params, color_cls, blobs_to_rows, pack_bearings, unpack_bearings)

하는 일
  OAK 컬러 프레임에서 부표 색(buoy_color, 기본 navy) 윤곽을 **여러 개** 찾아 /buoy_bearings 한 줄로 만든다.
  윤곽 찾기·방위·conf·age 는 avoid_detect_logic 을 **그대로 재사용**한다(detect_blobs·bearing_left_deg·
  intrinsics_from_camera_info·conf_score·frame_age_s) — 다중 윤곽 로직을 두 벌 두지 않는다.
  거리는 내지 않는다(CLAUDE.md §3: 카메라는 방위만, 거리는 LiDAR).

/buoy_bearings (Float32MultiArray)
  [VER=1, STRIDE=4, N, N × (bearing_left_deg, area_px, conf, age_s)]
    bearing_left_deg  좌 +(deg) = −atan((u − cx)/fx). u = 윤곽 질량중심 x(px). fx·cx 는 camera_info K
                      (/avoid_detections 의 bearing 과 같은 식·부호. /image_angle 과도 같은 부호 — 0=정면, 왼쪽 +)
    area_px           윤곽 면적(px²). 가까울수록 크다(거리 아님 — 순위·참고용)
    conf              0~1 = min(1, area/conf_full_area_px) × solidity (avoid_detect_logic.conf_score 와 같음)
    age_s             촬영(header.stamp) → 발행까지 초. stamp 를 못 쓰면 이 노드 처리 시간. **0 이상으로 clamp**
  · 행 순서 = 면적 큰 순(detect_blobs 순서), 최대 max_detections 개.
  · 본 것이 없어도 [1, 4, 0] 을 낸다(하트비트) — "이 프레임을 봤는데 없다". 담당 모드 중 **프레임마다**.
  · 머리(VER·STRIDE·N)는 /avoid_detections·/fused_objects 와 같은 규약이다. 구독자(ship_back)가 형식을
    확인할 수 있게 VER·STRIDE 를 매 메시지에 싣는다. 🚨 형식을 바꾸면 VER 을 올린다.
  · 🚨 형식 상수는 구독자 쪽에 **사본**으로 둔다(§1-8 상수 공유 모듈화 금지). 지금(a) ship_back 은 원시 저장만
    하므로 사본이 없다 — (b) 에서 파싱을 넣을 때 사본 + 이 모듈과의 일치 테스트를 같이 넣는다.

안 내는 경우 (모르면 입을 다문다 — §1-6) — 노드(basic_image_subscriberstop) 쪽
  · 담당 모드 아님 / /wp_mode 미수신·stale → 처리·발행 없음
  · camera_info 미수신·K 무효·해상도 불일치 → 발행 없음 + ERROR (방위를 hfov_deg 로 지어내지 않는다)
"""

import math

from color_shape_detector import avoid_detect_logic as adl
from color_shape_detector.hsv_ranges import VALID_COLORS as _HSV_COLORS

# ---- /buoy_bearings 형식 ----
BB_VER = 1
BB_STRIDE = 4                       # bearing_left_deg, area_px, conf, age_s
BB_HEADER = 3                       # VER, STRIDE, N

CLS_BUOY = 0                        # detect_blobs 가 요구하는 cls 칸 — 이 토픽엔 싣지 않는다(색 하나)

# 부표 색 — hsv_ranges 가 HSV 범위를 아는 색이면 된다(모르는 색은 HSV 를 못 읽어 아무것도 못 찾는다).
BUOY_COLORS = tuple(_HSV_COLORS)
DEFAULT_BUOY_COLOR = 'navy'         # = 노드 declare_parameter 리터럴 = vision.yaml image_subscriber_stop 절

# 노드 declare_parameter 리터럴 = vision.yaml image_subscriber_stop 절 = 아래 값(test_stop_detect_logic 이 묶는다).
#   잘못된 값일 때 돌아갈 값이기도 하다. ⚠️ 실측 필요(buoy_color 제외 전부).
DEFAULT_PARAMS = {
    'buoy_color': DEFAULT_BUOY_COLOR,
    'min_area_px': 40.0,
    'min_solidity': 0.7,
    'conf_full_area_px': 400.0,
    'max_detections': 8,
}


def color_cls(buoy_color):
    """부표 색 → {색: cls} (detect_blobs 가 HSV 를 읽고 찾는 색 표)."""
    return {buoy_color: CLS_BUOY}


def _num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _pos_finite(v):
    return _num(v) and math.isfinite(v) and v > 0.0


def check_params(raw):
    """부팅 검사. raw = {이름: yaml 값} → (쓸 값 dict, 오류 메시지 목록). 범위 밖이면 그 값만 기본값으로.

      buoy_color         BUOY_COLORS 중 하나(대소문자·앞뒤 공백 무관). 모르는 색·문자열 아님 → 'navy'
      min_area_px        > 0 유한
      min_solidity       0 ≤ x ≤ 1
      conf_full_area_px  > 0 유한
      max_detections     정수 ≥ 1 (0 이면 늘 N=0 — 살아 있는데 아무것도 안 보는 침묵 고장)
    빠진 키는 기본값(오류 아님).
    """
    out, errs = {}, []

    def bad(name, why):
        d = DEFAULT_PARAMS[name]
        errs.append(f"🚨 {name}={raw.get(name)!r} 는 잘못된 값({why}) → {d!r} 로 돈다. "
                    f"vision.yaml image_subscriber_stop 절 확인.")
        out[name] = d

    v = raw.get('buoy_color', DEFAULT_PARAMS['buoy_color'])
    c = v.strip().lower() if isinstance(v, str) else None
    if c in BUOY_COLORS:
        out['buoy_color'] = c
    else:
        bad('buoy_color', f"허용 {', '.join(BUOY_COLORS)}")
    for name in ('min_area_px', 'conf_full_area_px'):
        v = raw.get(name, DEFAULT_PARAMS[name])
        if _pos_finite(v):
            out[name] = float(v)
        else:
            bad(name, '0 보다 큰 유한수')
    v = raw.get('min_solidity', DEFAULT_PARAMS['min_solidity'])
    if _num(v) and math.isfinite(v) and 0.0 <= v <= 1.0:
        out['min_solidity'] = float(v)
    else:
        bad('min_solidity', '0~1')
    v = raw.get('max_detections', DEFAULT_PARAMS['max_detections'])
    if isinstance(v, int) and not isinstance(v, bool) and v >= 1:
        out['max_detections'] = v
    else:
        bad('max_detections', '1 이상 정수')
    return out, errs


def blobs_to_rows(blobs, intr, age_s, conf_full_area_px):
    """detect_blobs 결과 + (fx, fy, cx, cy) → [(bearing_left_deg, area_px, conf, age_s)] (blobs 순서 그대로)."""
    fx, _fy, cx, _cy = intr
    return [(adl.bearing_left_deg(b['u'], fx, cx), float(b['area']),
             adl.conf_score(b['area'], b['solidity'], conf_full_area_px), age_s) for b in blobs]


def _finite(v):
    return _num(v) and math.isfinite(v)


def pack_bearings(rows):
    """[(bearing_left_deg, area_px, conf, age_s)] → (data 리스트, 버린 수).

    빈 목록 → [1.0, 4.0, 0.0] (하트비트). 값이 유한하지 않거나 area ≤ 0, conf 가 0~1 밖, age_s < 0 인 행은
    **그 행만** 빼고 N 을 맞춘다(지어낸 값·형식 오류를 내보내지 않는다 — 메시지 전체는 살린다).
    """
    out_rows, dropped = [], 0
    for r in rows:
        brg, area, conf, age = r
        if (not all(_finite(v) for v in (brg, area, conf, age))
                or area <= 0.0 or not (0.0 <= conf <= 1.0) or age < 0.0):
            dropped += 1
            continue
        out_rows.append((float(brg), float(area), float(conf), float(age)))
    data = [float(BB_VER), float(BB_STRIDE), float(len(out_rows))]
    for r in out_rows:
        data.extend(r)
    return data, dropped


def unpack_bearings(data):
    """/buoy_bearings data → (행 목록, 오류 문자열 또는 None). 형식이 하나라도 어긋나면 ([], 이유).

    형식 왕복 테스트용 기준 해석기(구독자 사본을 만들 때 이 동작과 맞춘다). 행 = (bearing, area, conf, age).
    """
    try:
        vals = [float(v) for v in data]
    except (TypeError, ValueError):
        return [], 'not numeric'
    if len(vals) < BB_HEADER:
        return [], f'short header ({len(vals)})'
    ver, stride, n = vals[0], vals[1], vals[2]
    if ver != float(BB_VER):
        return [], f'VER {ver:g} != {BB_VER}'
    if stride != float(BB_STRIDE):
        return [], f'STRIDE {stride:g} != {BB_STRIDE}'
    if not (math.isfinite(n) and n >= 0.0 and n == int(n)):
        return [], f'N {n!r}'
    n = int(n)
    if len(vals) != BB_HEADER + n * BB_STRIDE:
        return [], f'length {len(vals)} != {BB_HEADER + n * BB_STRIDE}'
    rows = [tuple(vals[BB_HEADER + i * BB_STRIDE: BB_HEADER + (i + 1) * BB_STRIDE]) for i in range(n)]
    return rows, None
