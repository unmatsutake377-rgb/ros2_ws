"""(C) 카메라 장애물 병합 — 가짜 LiDAR 배열을 실제 스캔에 min 병합 (ROS 비의존, 순수 함수).

근거: docs/문제와작업/계약서_카메라장애물_20261111.md U1.
토너먼트(mode 8)에 LiDAR 빔이 못 보는 빨간 누운 원통이 있다. 카메라 노드(U4)가 그것을
/camera_obstacle_state 로 낸다 — 형식은 /geofence_state 와 같은 '가짜 LiDAR':
    data = [angle_min_deg, angle_inc_deg, r0, r1, ...]
    상대방위 0 = 정면, **좌 양수**(인덱스 0 = 우현), 멀거나 없음 inf, 정보 없음 = 빈 배열.
    원점은 이미 LiDAR 원점으로 바꿔서 온다(발행자 책임).

여기서 하는 일은 _merge_geofence 와 **같은 인덱스 매핑**의 min 병합이 전부다:
    rel = a − forward_deg(80),  k = round((rel − a0) / inc),  out[i] = min(out[i], cam[k])
🚨 부호 변환을 하지 않는다 — 발행자가 좌 양수로 낸다(vision_geom.angle_from_pixel 왼쪽 +).
   한쪽에서라도 뒤집으면 거울상(계약서_geofence부호_20261104 전례).

병합하지 않는 경우('모르면 입을 다문다' — 옛 동작 그대로):
    · 한 번도 못 받음(cam_t None)              → 'none'
    · 수신 후 stale_sec **초과** 갱신 없음        → 'stale'
    · 받았지만 정보 없음(빈 배열·머리 이상)      → 'empty'
    · 값이 비유한·0 이하인 칸은 그 칸만 건너뜀
켜짐·모드 판정(camera_active)과 파라미터 검증(check_*)도 여기 둔다. 시각(now)은 호출자가 주입한다
(시뮬 rosstub 은 노드 모듈의 time 만 바꾼다 — 계약서 위험 5).
"""

import math

CAMERA_ALLOWED_MODES = (5, 8)     # 카메라 병합을 켤 수 있는 wp_mode (계약서 Q5). 0·1·2·3·7 금지
DEFAULT_CAMERA_MODES = [8]
DEFAULT_CAMERA_STALE_SEC = 0.5    # 계약서 Q7

ST_OK = 'ok'          # 병합함(칸 0 개여도 — 장애물 없음 = 전부 inf 도 '살아 있음')
ST_NONE = 'none'      # 한 번도 못 받음
ST_STALE = 'stale'    # 수신 후 stale_sec 초과
ST_EMPTY = 'empty'    # 신선하지만 정보 없음(빈 배열)


def parse_state(data):
    """msg.data → (a0_deg, inc_deg, [r...]) 또는 None(정보 없음).

    머리 두 칸 + 거리 한 칸 이상, a0 유한, inc 유한·양수일 때만 받는다(geofence_cb 와 같은 기준 +
    a0 유한). 거리 칸의 비유한 값은 그대로 둔다 — 병합에서 그 칸만 건너뛴다.
    """
    try:
        d = list(data)
    except TypeError:
        return None
    if len(d) < 3:
        return None
    try:
        a0 = float(d[0])
        inc = float(d[1])
        rs = [float(x) for x in d[2:]]
    except (TypeError, ValueError):
        return None
    if not (math.isfinite(a0) and math.isfinite(inc) and inc > 0.0):
        return None
    return (a0, inc, rs)


def camera_status(cam, cam_t, now, stale_sec):
    """→ ST_NONE / ST_STALE / ST_EMPTY / ST_OK. 경계: age == stale_sec 는 신선(초과일 때만 stale)."""
    if cam_t is None:
        return ST_NONE
    if (now - cam_t) > stale_sec:
        return ST_STALE
    if cam is None or not cam[2]:
        return ST_EMPTY
    return ST_OK


def camera_silent(cam_t, active_since, now, stale_sec):
    """켜짐 + 담당 모드에서 '침묵' 인가(로그 전용 — 병합 여부는 camera_status 가 정한다).

    기준 시각 = max(마지막 수신, 담당 모드 진입). 카메라 노드는 담당 모드 밖에선 발행하지 않으므로
    진입 순간엔 마지막 수신이 늘 묵었다 → 진입부터 stale_sec 은 기다린다(거짓 ERROR 방지).
    active_since None(진입 기록 없음)이면 False(모름 — 판정 안 함).
    """
    if active_since is None:
        return False
    ref = active_since if cam_t is None else max(cam_t, active_since)
    return (now - ref) > stale_sec


def camera_active(enabled, wp_mode, modes):
    """켜짐(True 정확히) + wp_mode 가 modes 안일 때만 True. wp_mode None(모름) → False."""
    return enabled is True and wp_mode is not None and wp_mode in modes


def merge_camera(distance_array, angle_array, cam, cam_t, now, stale_sec, forward_deg=80.0):
    """→ (병합 배열, 상태, 바뀐 칸 수, 최근접 병합 거리).

    상태가 ST_OK 가 아니면 distance_array **그 객체**를 그대로 돌려준다(병합 안 함).
    ST_OK 면 새 리스트: angle_array[i](LiDAR 프레임, forward_deg = 정면)마다
        rel = a − forward_deg,  k = int(round((rel − a0) / inc))   ← _merge_geofence 와 같은 줄
    k 가 배열 밖이면 건너뜀, cam[k] 가 비유한·0 이하면 건너뜀,
    cur 가 비유한(inf·nan)이거나 cam[k] < cur 이면 out[i] = cam[k] (min).
    """
    status = camera_status(cam, cam_t, now, stale_sec)
    if status != ST_OK:
        return distance_array, status, 0, math.inf
    a0, inc, cr = cam
    n = len(cr)
    out = list(distance_array)
    hits = 0
    nearest = math.inf
    for i, a in enumerate(angle_array):
        rel = a - forward_deg                 # LiDAR 프레임(80=정면) → 상대방위(0=정면, 좌 양수 그대로)
        k = int(round((rel - a0) / inc))
        if not (0 <= k < n):
            continue
        c = cr[k]
        if not math.isfinite(c) or c <= 0.0:
            continue
        cur = out[i]
        if (not math.isfinite(cur)) or c < cur:
            out[i] = c                         # ★ min(실제, 카메라)
            hits += 1
            nearest = min(nearest, c)
    return out, ST_OK, hits, nearest


# ---------------------------------------------------------------- 파라미터 검증
def check_enabled(raw):
    """→ (값, 오류 문자열 또는 None). bool 이 아니면 False(끔)."""
    if isinstance(raw, bool):
        return raw, None
    return False, f"camera_obstacle_enabled={raw!r} bool 아님 → false(끔) 사용"


def check_modes(raw, allowed=CAMERA_ALLOWED_MODES):
    """→ (허용 모드 리스트(중복 제거, 순서 유지), 버린 값 리스트). dwa_modes 와 같은 규칙.

    bool·허용 밖·리스트가 아닌 값은 버린다. 결과가 비면 호출자가 끈다.
    """
    try:
        items = list(raw)
    except TypeError:
        return [], [raw]
    modes, dropped = [], []
    for m in items:
        if isinstance(m, bool) or m not in allowed:
            dropped.append(m)
        elif int(m) not in modes:
            modes.append(int(m))
    return modes, dropped


def check_stale_sec(raw, default=DEFAULT_CAMERA_STALE_SEC):
    """→ (값, 오류 문자열 또는 None). 숫자 아님·bool·비유한·0 이하 → default."""
    if isinstance(raw, bool):
        v = None
    else:
        try:
            v = float(raw)
        except (TypeError, ValueError):
            v = None
    if v is None or not math.isfinite(v) or v <= 0.0:
        return float(default), f"camera_obstacle_stale_sec={raw!r} 이상 → 기본값 {default}초 사용"
    return v, None
