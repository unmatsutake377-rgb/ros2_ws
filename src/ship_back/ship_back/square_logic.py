"""ship_back 사각형 위치유지(hold_source=square) 로직 — **ROS 비의존 순수 파이썬** (rclpy 임포트 없음).

계약서 v2 위치유지 사각형(study_square_sim/CONTRACT_v2_ship_back_square.md) 변경 단위 (b) + Davidson 답 v1·v2.
알고리즘 원문(Davidson): "도리도리, 2개찾고, 그기준으로 돌면서 3개까지 찾고, 3개 찾은걸 바탕으로 안으로 들어가고,
  들어가서 4번째꺼 찾고, gps로 부표4개, 부표가이루는 도형의 중심을 구해서 gps값을 구하고, 구한 도형의 중앙 gps를
  기준으로 위치를 유지한다".

    from ship_back.square_logic import (
        SquareFSM, HeadingFuser, LandmarkMap, check_square_params, parse_buoy_bearings,
        fix_usable, ll_to_xy, project, fit_square, hull_inside, safe_region_exists, square_hold_applies)

상태(FSM) — 출력은 /candidate_angle 값 하나(틱마다)
  INIT   20000(손 뗌, north LOS 가 WP2 = 사각형 중심으로 끈다). /goal_distance(쓸 수 있는 값) < engage_m 이고
         진입 뒤 init_min_sec 지나면 DORI. 🚨 INIT·DONE 밖에서는 20000 을 **절대** 내지 않는다(계약서 위험 1:
         north 는 WP2 3.5 m 안에서 ship_back 의 마지막 값이 20000 이면 dwell 0.5 s 로 WP 를 넘긴다).
  DORI   제자리 도리도리: 진입 방위 H0 기준 0, +a, −a, +2a, −2a, +3a, −3a 로 돌아 보며(정지 펄스 선회) 부표를 모은다.
         확정 부표가 사각형 한 변(≈side) 또는 대각선(≈side·√2)을 이루면 SEEK3.
  SEEK3  두 부표로 나머지 꼭짓점 위치를 예측해 그 방위로 차례로 돌아 본다(변이면 배 반대쪽 가설 먼저).
         세 개가 직각 이등변(다리 ≈side, 빗변 ≈side·√2)이 되면 ENTER. 다 봐도 못 찾으면 두 부표 쪽(변 바깥
         entry_standoff_m, 대각선이면 중점)으로 다가가 한 번 더 보고, 그래도 못 찾으면 DORI 한 바퀴 뒤 다시.
  ENTER  3점 평행사변형 보완(D = A + C − B)으로 중심을 잡고 안으로 들어간다. 직선이 부표에 가까우면 그 변 바깥
         중간점(staging)을 먼저 거친다. 중심 hold_out_m 안이면 4점이면 HOLD, 아니면 FIND4(둘 다 hold_in_m 까지
         마저 다가간다 — 히스테리시스).
  FIND4  중심을 지키며(히스테리시스) 예측한 4번째 꼭짓점 방위 → +90° 씩 돌아 본다. 4점이 되면 HOLD.
  HOLD   네 부표 꼭짓점의 평균(도형 중심)을 기준으로 위치 유지. 선체 전체(직사각형 hull_length × hull_beam)가
         사각형 안쪽(변에서 buoy_radius + inside_margin 안쪽)에 **연속 hold_time_sec** 이면 DONE. 벗어나면 0 부터.
  DONE   20000(손 뗌) + /hold_done true. 성공·포기 모두(포기는 ERROR 로 구분) — 래치(모드 진입 때만 리셋).
제어 명령(INIT·DONE 밖): 조향 ±STEER_LIMIT_DEG 이내, 5000(우선회)·6000(좌선회) 펄스, 50000(정지),
  180(직진 후진 — 앞쪽 가까이에 물체가 있을 때만, reverse_pulse_sec 짧게, Davidson 답 v2 3번).
성공 금지: 4점이 아니거나, 모양 오차 > square_shape_tol_m(꼭짓점 확정은 부표 오차 ≤ buoy_pos_max_err_m),
  안전영역(안쪽으로 줄인 사각형에 선체가 들어갈 자리) 소멸, 센서(/buoy_bearings·fix·헤딩) 없음 → 타이머 0.
포기: 센서 없음이 unknown_giveup_sec 이어짐 / DORI 진입 뒤 square_max_sec → ERROR + DONE.

헤딩(Davidson 답 v2 5번 — 회전량은 /imu/yaw_raw 로 보정): HeadingFuser 참고.
좌표: 지역 평면 x = 동, y = 북(m). 나침반 방위 0 = 북, 시계 +. 상대 방위(/buoy_bearings·조향) 좌 +.
"""

import math

# ---- 특수 신호 사본 (CLAUDE.md §3 표 — ship_back.py 와 같은 값. 공유 모듈 금지 §1-8) ----
CANDIDATE_INVALID = 20000.0
STOP_HOLD = 50000.0
SPIN_RIGHT = 5000.0
SPIN_LEFT = 6000.0
REVERSE_REL = 180.0
STEER_LIMIT_DEG = 80.0

SQUARE_MODES = (2,)            # search_behavior:=hold 의 mode 3 은 vision 경로 그대로(gps 와 같은 방식)
# 🚨 사본: north_goal_angle.yaml arrive_radius_hold_modes_m (hold_logic.NORTH_ARRIVE_HOLD_M 과 같은 값).
NORTH_ARRIVE_HOLD_M = 3.5
ENGAGE_MIN_MARGIN_M = 2.0      # engage_m ≥ north 도착 반경 + 이 값 (INIT 20000 이 3.5 m 안까지 이어지지 않게)
ERROR_REPEAT_SEC = 5.0         # 같은 ERROR(센서 없음)를 다시 낼 간격 — 다른 노드의 5 s 주기와 같음
INIT_WARN_SEC = 20.0           # INIT 에서 /goal_distance 를 이만큼 못 받으면 WARN 1회(ship_back search_timeout_sec 와 같은 값)
SEEK_MOVE_MAX_FACTOR = 3.0     # SEEK3 다가가기 상한 = look_turn_max_sec × 이 값(못 닿으면 그 자리에서 다시 본다)

# ---- /buoy_bearings 형식 사본 (color_shape_detector.stop_detect_logic BB_* 와 같은 값 — test_square_logic 이 대조) ----
BB_VER = 1
BB_STRIDE = 4                  # bearing_left_deg, area_px, conf, age_s
BB_HEADER = 3

# 파라미터 기본값 = ship_back.py declare_parameter 리터럴 = config/ship_back.yaml (test_square_logic 이 셋을 묶는다).
DEFAULTS = {
    'square_side_m': 5.0,              # 규정 5 × 5 m (Davidson 답 1)
    'hull_length_m': 1.77,             # 공용 보수값(2번배 길이, Davidson 답 v1 7-(i))
    'hull_beam_m': 0.57,               # 공용 보수값(1번배 폭)
    'buoy_radius_m': 0.25,             # ⚠️ 실측 필요
    'square_inside_margin_m': 0.3,     # ⚠️ 실측 필요 — GPS·헤딩·안테나 위치 여유
    'square_shape_tol_m': 2.0,         # ⚠️ 실측 필요 — 변·대각선 길이 허용 오차(부표마다 표류 ~1 m → 변 ±2 m)
    'buoy_pos_max_err_m': 0.6,         # ⚠️ 실측 필요 — 부표 GPS 추정 흩어짐(RMS) 상한
    'buoy_confirm_obs': 5,             # 확정 관측 수
    'buoy_assoc_m': 1.2,               # 같은 부표로 묶는 거리
    'obs_min_conf': 0.1,               # /buoy_bearings conf 하한(conf 는 면적 비례 — 먼 부표는 낮다)
    'obs_max_range_m': 10.0,           # LiDAR 거리 상한(멀수록 헤딩 오차가 커진다)
    'obs_max_turn_dps': 15.0,          # ⚠️ 실측 필요 — 이보다 빨리 돌 때 관측 버림(영상·스캔 번짐)
    'bearings_stale_sec': 1.0,         # ⚠️ 실측 필요 — 프레임 간격의 여러 배(BEST_EFFORT d1)
    'fix_stale_sec': 2.0,              # GPS 1 Hz
    'yaw_stale_sec': 1.0,              # /imu/yaw (yaw_mux)
    'yaw_raw_stale_sec': 0.5,          # /imu/yaw_raw (iahrs 100 Hz)
    'heading_use_raw': True,           # false = /imu/yaw 그대로(옛 방식, 비교·롤백용)
    'yaw_offset_tau_sec': 3.0,         # ⚠️ 실측 필요 — offset 이 /imu/yaw 를 따라가는 시상수(COG 잡음 평활)
    'yaw_offset_max_turn_dps': 5.0,    # ⚠️ 실측 필요 — 이보다 빨리 돌 땐 offset 안 고침(선회 중 COG 지연)
    'yaw_offset_min_speed_mps': 0.6,   # ⚠️ 실측 필요 — GPS 로 잰 속도가 이보다 느리면 offset 안 고침(gps_cog HOLD·예측)
    'yaw_raw_invert': False,           # ⚠️ 실측 필요 — 🚨 ssf_heading_{A,B}.yaml invert_yaw 와 같게
    'square_engage_m': 8.0,            # INIT → DORI 거리(/goal_distance)
    'init_min_sec': 1.0,
    'look_tol_deg': 10.0,
    'look_dwell_sec': 1.5,
    'look_turn_max_sec': 8.0,
    'dori_amp_deg': 40.0,
    'spin_pulse_sec': 0.3,             # ⚠️ 실측 필요 — 선회 펄스/쉼(실물 선회율 미측정)
    'spin_pause_sec': 0.4,
    'hold_in_m': 0.4,                  # 중심 이만큼 안이면 정지
    'hold_out_m': 0.8,                 # 중심 이만큼 밖이면 다시 이동(히스테리시스)
    'steer_spin_deg': 60.0,            # 목표 상대방위가 이보다 크면 제자리 선회부터
    'entry_clear_m': 1.2,              # 진입 직선이 부표에 이보다 가까우면 변 바깥 중간점을 먼저
    'entry_standoff_m': 1.5,
    'close_m': 0.4,                    # ⚠️ 실측 필요 — 선체에서 물체까지 여유. near_hazard 참고
    'reverse_pulse_sec': 1.0,
    'square_max_sec': 90.0,            # DORI 진입 뒤 이 시간에 못 끝내면 포기 DONE
}

_BOOL_KEYS = ('heading_use_raw', 'yaw_raw_invert')
_INT_KEYS = ('buoy_confirm_obs',)
_NONNEG_KEYS = ('init_min_sec', 'spin_pause_sec', 'square_inside_margin_m', 'buoy_radius_m', 'yaw_offset_tau_sec')


def square_hold_applies(hold_source, wp_mode):
    """이 모드를 사각형 위치유지로 돌지. square 이고 mode 2 일 때만(mode 3 hold 옵션은 vision)."""
    return hold_source == 'square' and wp_mode in SQUARE_MODES


def _fnum(v):
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return None
    f = float(v)
    return f if math.isfinite(f) else None


def check_square_params(raw, north_arrive_m=NORTH_ARRIVE_HOLD_M):
    """사각형 파라미터 시작 가드. raw = {이름: 값} → (쓸 값 dict, [(심각도, 메시지), …]). 순수 함수.

    빠진 키 = 기본값(오류 아님). 잘못된 값(타입·비유한·범위) → ERROR + 그 값만 기본값.
      bool 키는 bool 만, buoy_confirm_obs 는 1 이상 정수, _NONNEG_KEYS 는 ≥ 0, 나머지 숫자는 > 0.
    묶음 검사(ERROR + 묶음 모두 기본값):
      hold_out_m ≤ hold_in_m / steer_spin_deg > 80 / square_engage_m < north 도착 반경 + 2.0 m.
    안전영역(ERROR, 값은 그대로 — 성공 금지로 드러난다): 규정 사각형을 buoy_radius + inside_margin 만큼 줄인 한 변이
      hull_length_m 보다 짧으면 선체가 들어갈 자리가 없다 → 그 설정으로는 DONE(성공)이 나지 않는다.
    """
    out, issues = {}, []
    for k, d in DEFAULTS.items():
        v = raw.get(k, d)
        if k in _BOOL_KEYS:
            ok = isinstance(v, bool)
            val = v
        elif k in _INT_KEYS:
            ok = isinstance(v, int) and not isinstance(v, bool) and v >= 1
            val = v
        else:
            f = _fnum(v)
            ok = f is not None and (f >= 0.0 if k in _NONNEG_KEYS else f > 0.0)
            val = f
        if ok:
            out[k] = val
        else:
            issues.append(('ERROR', f"🚨 {k}={v!r} 는 잘못된 값 → 기본 {d!r} 로 돈다. ship_back.yaml 확인."))
            out[k] = d
    if out['hold_out_m'] <= out['hold_in_m']:
        issues.append(('ERROR', f"🚨 hold_out_m={out['hold_out_m']:g} ≤ hold_in_m={out['hold_in_m']:g} → 둘 다 기본 "
                                f"{DEFAULTS['hold_in_m']:g}/{DEFAULTS['hold_out_m']:g} m."))
        out['hold_in_m'], out['hold_out_m'] = DEFAULTS['hold_in_m'], DEFAULTS['hold_out_m']
    if out['steer_spin_deg'] > STEER_LIMIT_DEG:
        issues.append(('ERROR', f"🚨 steer_spin_deg={out['steer_spin_deg']:g} > {STEER_LIMIT_DEG:g}(조향 한계) → 기본 "
                                f"{DEFAULTS['steer_spin_deg']:g}."))
        out['steer_spin_deg'] = DEFAULTS['steer_spin_deg']
    need = north_arrive_m + ENGAGE_MIN_MARGIN_M
    if out['square_engage_m'] < need:
        issues.append(('ERROR', f"🚨 square_engage_m={out['square_engage_m']:g} < north 도착 반경 {north_arrive_m:g} + "
                                f"{ENGAGE_MIN_MARGIN_M:g} m — INIT 20000 동안 north 가 WP2 를 넘길 수 있다 → 기본 "
                                f"{DEFAULTS['square_engage_m']:g} m."))
        out['square_engage_m'] = DEFAULTS['square_engage_m']
    inset = out['buoy_radius_m'] + out['square_inside_margin_m']
    room = out['square_side_m'] - 2.0 * inset
    if room < out['hull_length_m']:
        issues.append(('ERROR', f"🚨 안전영역 소멸: 사각형 {out['square_side_m']:g} m − 2×(부표 반지름 + 여유 {inset:g} m) "
                                f"= {room:.2f} m < 선체 길이 {out['hull_length_m']:g} m → 위치유지 성공(DONE) 판정이 나지 "
                                f"않는다(성공 금지). ship_back.yaml 확인."))
    return out, issues


# ==================== /buoy_bearings ====================
def parse_buoy_bearings(data):
    """/buoy_bearings data → ([(bearing_left_deg, area_px, conf, age_s)], None) 또는 ([], 오류 사유). 순수 함수.

    stop_detect_logic.unpack_bearings 와 같은 규칙(머리 VER·STRIDE·N·길이). 형식이 어긋나면 메시지 통째로 버린다.
    N=0 은 '살아 있고 부표 없음'(하트비트) → ([], None).
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
    return [tuple(vals[BB_HEADER + i * BB_STRIDE: BB_HEADER + (i + 1) * BB_STRIDE]) for i in range(n)], None


# ==================== GPS ====================
EARTH_R_M = 6378137.0


def fix_usable(status, lat, lon):
    """NavSatFix 를 위치로 쓸 수 있나. status ≥ 0(STATUS_FIX 이상), 위경도 유한, (0, 0) 아님(드라이버 무fix 값)."""
    try:
        s = int(status)
        la, lo = float(lat), float(lon)
    except (TypeError, ValueError):
        return False
    return s >= 0 and math.isfinite(la) and math.isfinite(lo) and not (la == 0.0 and lo == 0.0)


def ll_to_xy(lat, lon, lat0, lon0):
    """위경도 → 원점(lat0, lon0) 기준 지역 평면 (x 동, y 북) m. 등장방형 근사(수십 m 범위)."""
    x = math.radians(lon - lon0) * EARTH_R_M * math.cos(math.radians(lat0))
    y = math.radians(lat - lat0) * EARTH_R_M
    return x, y


def gps_velocity(prev, cur, max_dt):
    """연속 두 fix (lat, lon, status, t) → 대지 속도 벡터 (vx 동, vy 북) m/s. 못 쓰는 fix·간격 0 이하·max_dt 초과 → None."""
    if prev is None or cur is None:
        return None
    if not (fix_usable(prev[2], prev[0], prev[1]) and fix_usable(cur[2], cur[0], cur[1])):
        return None
    dt = cur[3] - prev[3]
    if dt <= 0.0 or dt > max_dt:
        return None
    x, y = ll_to_xy(cur[0], cur[1], prev[0], prev[1])
    return x / dt, y / dt


def gps_speed(prev, cur, max_dt):
    """gps_velocity 의 크기(m/s) 또는 None."""
    v = gps_velocity(prev, cur, max_dt)
    return None if v is None else math.hypot(v[0], v[1])


def dead_reckon(xy_fix, t_fix, vel, t):
    """fix 위치를 그 fix 의 대지 속도로 t 까지 민다(GPS 1 Hz 사이 — 이동 중 관측이 1 s 묵은 위치로 투영되지 않게).
    vel None 이면 fix 그대로."""
    if vel is None:
        return xy_fix
    dt = t - t_fix
    return xy_fix[0] + vel[0] * dt, xy_fix[1] + vel[1] * dt


def xy_to_ll(x, y, lat0, lon0):
    """ll_to_xy 의 역."""
    lat = lat0 + math.degrees(y / EARTH_R_M)
    lon = lon0 + math.degrees(x / (EARTH_R_M * math.cos(math.radians(lat0))))
    return lat, lon


# ==================== 각도·기하 ====================
def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def wrap360(a):
    return a % 360.0


def bearing_to(x, y, tx, ty):
    """(x, y) 에서 (tx, ty) 로의 나침반 방위(0 = 북, 시계 +)."""
    return math.degrees(math.atan2(tx - x, ty - y)) % 360.0


def project(bx, by, heading_deg, bearing_left_deg, range_m):
    """배 위치·헤딩·상대방위(좌 +)·거리 → 물체의 지역 평면 좌표. 나침반 방위 = 헤딩 − 상대방위."""
    th = math.radians(heading_deg - bearing_left_deg)
    return bx + range_m * math.sin(th), by + range_m * math.cos(th)


def _dist(p, q):
    return math.hypot(p[0] - q[0], p[1] - q[1])


def order_ccw(pts):
    """점들을 무게중심 둘레 반시계(x 동·y 북 평면) 순서로."""
    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)
    return sorted(pts, key=lambda p: math.atan2(p[1] - cy, p[0] - cx))


def _edge_sd(p, q, pt):
    """반시계 다각형 변 p→q 에서 pt 까지 부호 있는 거리(안쪽 +)."""
    ex, ey = q[0] - p[0], q[1] - p[1]
    n = math.hypot(ex, ey)
    if n <= 1e-9:
        return -math.inf
    return (ex * (pt[1] - p[1]) - ey * (pt[0] - p[0])) / n


def inside_inset(quad_ccw, pt, inset):
    """pt 가 볼록 다각형(반시계)을 inset 만큼 안쪽으로 줄인 영역 안인가(모든 변에서 ≥ inset)."""
    k = len(quad_ccw)
    return all(_edge_sd(quad_ccw[i], quad_ccw[(i + 1) % k], pt) >= inset for i in range(k))


def hull_corners(x, y, heading_deg, length, beam):
    """선체 직사각형 네 모서리(중심 = 배 위치 = GPS 안테나 가정 ⚠️). 헤딩 = 나침반."""
    h = math.radians(heading_deg)
    fx, fy = math.sin(h), math.cos(h)          # 앞
    rx, ry = math.cos(h), -math.sin(h)         # 오른쪽
    hl, hb = length / 2.0, beam / 2.0
    return [(x + sf * hl * fx + sr * hb * rx, y + sf * hl * fy + sr * hb * ry)
            for sf, sr in ((1, 1), (1, -1), (-1, -1), (-1, 1))]


def hull_inside(quad_ccw, x, y, heading_deg, length, beam, inset):
    """선체 전체(네 모서리)가 사각형을 inset 만큼 줄인 영역 안인가. 하나라도 모르면 False."""
    if quad_ccw is None or x is None or heading_deg is None:
        return False
    return all(inside_inset(quad_ccw, c, inset) for c in hull_corners(x, y, heading_deg, length, beam))


def safe_region_exists(quad_ccw, inset, length, beam, step_deg=15.0):
    """줄인 사각형 안에 선체가 들어갈 자리가 하나라도 있나(중심에서 헤딩 0~180 을 step 으로 시험).

    False = '안전영역 소멸' → 성공 금지. 무게중심 하나만 보는 근사(볼록 사각형이면 거의 최선 자리).
    """
    if quad_ccw is None:
        return False
    cx = sum(p[0] for p in quad_ccw) / len(quad_ccw)
    cy = sum(p[1] for p in quad_ccw) / len(quad_ccw)
    h = 0.0
    while h < 180.0:
        if hull_inside(quad_ccw, cx, cy, h, length, beam, inset):
            return True
        h += step_deg
    return False


def _seg_point_dist(a, b, p):
    ax, ay = b[0] - a[0], b[1] - a[1]
    L2 = ax * ax + ay * ay
    if L2 <= 1e-12:
        return _dist(a, p)
    t = max(0.0, min(1.0, ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / L2))
    return math.hypot(a[0] + t * ax - p[0], a[1] + t * ay - p[1])


def seek_approach_point(pair, boat, side, standoff_m):
    """두 부표(pair)를 알 때 나머지를 보러 다가갈 점. 순수 함수.

    변(거리 ≈ side 쪽)이면 그 변 중간점에서 배 쪽으로 standoff_m — 거기서 먼 꼭짓점은 √((side/2)² + (side+standoff)²)
    ≈ 7 m 안. 대각선이면 그 중간점(= 사각형 중심, 부표와 side/√2 떨어짐).
    """
    A, B = pair
    d = _dist(A, B)
    mx, my = (A[0] + B[0]) / 2.0, (A[1] + B[1]) / 2.0
    if abs(d - side) > abs(d - side * math.sqrt(2.0)):
        return mx, my
    nx, ny = -(B[1] - A[1]) / d, (B[0] - A[0]) / d
    if (nx * (boat[0] - mx) + ny * (boat[1] - my)) < 0.0:
        nx, ny = -nx, -ny
    return mx + nx * standoff_m, my + ny * standoff_m


def entry_target(boat, quad_ccw, center, clear_m, standoff_m):
    """진입 목표점. 배가 안이거나 배→중심 직선이 모든 부표에서 clear_m 넘게 떨어져 있으면 중심.

    아니면 배가 바깥쪽에 있는 변(부호 거리 가장 음수) 중간점에서 바깥으로 standoff_m 인 점(staging) —
    거기서 중심으로 가면 변 가운데를 지나 부표와 멀다.
    """
    if inside_inset(quad_ccw, boat, 0.0):
        return center, False
    if all(_seg_point_dist(boat, center, c) > clear_m for c in quad_ccw):
        return center, False
    k = len(quad_ccw)
    best, best_sd = None, math.inf
    for i in range(k):
        p, q = quad_ccw[i], quad_ccw[(i + 1) % k]
        sd = _edge_sd(p, q, boat)
        if sd < best_sd:
            best, best_sd = i, sd
    p, q = quad_ccw[best], quad_ccw[(best + 1) % k]
    mx, my = (p[0] + q[0]) / 2.0, (p[1] + q[1]) / 2.0
    ex, ey = q[0] - p[0], q[1] - p[1]
    n = math.hypot(ex, ey) or 1.0
    ox, oy = ey / n, -ex / n                   # 반시계 다각형의 바깥 법선(오른쪽)
    return (mx + ox * standoff_m, my + oy * standoff_m), True


# ==================== 헤딩 ====================
class HeadingFuser:
    """/imu/yaw(절대, gps_cog) + /imu/yaw_raw(IMU 상대) → 헤딩. Davidson 답 v2 5번.

    문제: gps_cog 는 0.5 m/s 미만이면 마지막 값을 그대로 낸다(HOLD), 선회·후진 IMU 보조도 10 s 상한 뒤 얼린다.
      사각형 미션은 정지·제자리 선회 위주라 그동안 /imu/yaw 는 배가 돌아도 그대로다 → 부표 GPS 투영이 통째로 틀린다.
    방식: H = s·raw + offset. offset = /imu/yaw − s·raw 를 **/imu/yaw 값이 바뀐 샘플에서만** 다시 잰다.
      · 값이 바뀜 = gps_cog 가 새 COG 를 채택(속도 있음)했거나 IMU Δ 보조 중 → 그때의 절대값이 믿을 만하다.
        (IMU 보조 중엔 /imu/yaw = 앵커 + Δraw 라 offset 이 그대로 = 이중 적용 없음)
        새 값을 그대로 쓰지 않고 시상수 tau 로 따라간다 — COG 잡음(저속 σ 수 °)을 평활한다. 첫 값은 그대로.
        다음이면 안 고친다: raw 회전율 > max_turn_dps(선회 중 COG 는 뱃머리보다 늦다), GPS 로 잰 속도 < min_speed
        또는 모름(gps_cog 는 저속에서 HOLD 하거나 직전 회전율로 최대 1.5 s 예측해 값이 바뀐다 — 그 값은 COG 가 아님).
        시뮬(tools/sim/square_hold_sim.py, gps_cog): 바뀐 값을 그대로 쓰면 정지 직후 예측값으로 헤딩 오차 p95 10~30°.
      · 값이 그대로(HOLD·얼림) = 배가 돌아도 /imu/yaw 는 모른다 → offset 유지, 회전량은 raw 로.
      · 절대 기준은 늘 /imu/yaw(COG, 진북). raw 는 상대 회전량만 — mount·편각은 offset 에 흡수된다.
      s = −1 if yaw_raw_invert (yaw_mux invert_yaw 와 같은 뜻: raw 가 시계 반대로 증가).
    모르면 None: /imu/yaw 가 yaw_stale 넘게 묵음(yaw_mux 죽음), raw 가 raw_stale 넘게 묵음, offset 없음,
      **한 번도 '움직이며 곧게'(속도 ≥ min_speed, 회전율 ≤ max_turn_dps) 배운 적 없음** — gps_cog 는 첫 COG 전엔
      WP 방위(INIT_GOAL)를 내므로 그 값은 헤딩이 아니다(시뮬: 정지 출발이면 오차 최대 180°, 접촉). 대회 흐름은 WP1→WP2
      이동 중에 배운다. 못 배웠으면 사각형 FSM 은 정지 + ERROR 뒤 포기(모르면 입을 다문다).
    use_raw=False 면 /imu/yaw 그대로(옛 방식 — 비교·롤백).
    """

    def __init__(self, use_raw=True, invert=False, yaw_stale=1.0, raw_stale=0.5, change_eps=1e-3,
                 hist_sec=2.0, tau_sec=3.0, max_turn_dps=5.0, min_speed=0.6):
        self.use_raw = bool(use_raw)
        self.tau_sec = float(tau_sec)
        self.max_turn_dps = float(max_turn_dps)
        self.min_speed = float(min_speed)
        self.sign = -1.0 if invert else 1.0
        self.yaw_stale = float(yaw_stale)
        self.raw_stale = float(raw_stale)
        self.change_eps = float(change_eps)
        self.hist_sec = float(hist_sec)
        self.reset()

    def reset(self):
        self.yaw = None            # (값, t)
        self.raw = None            # (s·raw, t)
        self.hist = []             # [(t, s·raw)] — 관측 시각 보간·회전율
        self.offset = None
        self.offset_t = None       # 마지막으로 offset 을 다시 잰 시각
        self.learned = False       # 움직이며 곧게 배운 적 있나(없으면 heading None)
        self._prev_yaw_v = None

    def update_raw(self, v, t):
        try:
            f = float(v)
        except (TypeError, ValueError):
            return
        if not math.isfinite(f):
            return
        r = wrap360(self.sign * f)
        self.raw = (r, t)
        self.hist.append((t, r))
        while len(self.hist) > 2 and self.hist[0][0] < t - self.hist_sec:
            self.hist.pop(0)

    def update_yaw(self, v, t, speed=None):
        """/imu/yaw 샘플. speed = GPS 로 잰 지금 속도(m/s, 모르면 None — 그러면 첫 offset 말고는 안 고친다)."""
        try:
            f = float(v)
        except (TypeError, ValueError):
            return
        if not math.isfinite(f):
            return
        y = wrap360(f)
        changed = self._prev_yaw_v is None or abs(wrap180(y - self._prev_yaw_v)) > self.change_eps
        self._prev_yaw_v = y
        self.yaw = (y, t)
        if self.raw is None:
            return
        d = wrap180(y - self.raw[0])
        if self.offset is None:
            self.offset, self.offset_t = d, t
        if (changed or not self.learned) and speed is not None and speed >= self.min_speed:
            rate = self.turn_rate(t)
            if rate is None or rate <= self.max_turn_dps:
                if not self.learned:
                    a = 1.0                    # 처음 배우는 값은 그대로(앞의 임시 offset 은 버린다)
                else:
                    dt = min(1.0, max(0.0, t - self.offset_t)) if self.offset_t is not None else 1.0
                    a = 1.0 if self.tau_sec <= 0.0 else 1.0 - math.exp(-dt / self.tau_sec)
                self.offset = wrap180(self.offset + a * wrap180(d - self.offset))
                self.offset_t = t
                self.learned = True

    def _fresh(self, now):
        if self.yaw is None or (now - self.yaw[1]) > self.yaw_stale:
            return False
        if not self.use_raw:
            return True
        return (self.raw is not None and (now - self.raw[1]) <= self.raw_stale and self.offset is not None
                and self.learned)

    def heading(self, now):
        if not self._fresh(now):
            return None
        if not self.use_raw:
            return self.yaw[0]
        return wrap360(self.raw[0] + self.offset)

    def heading_at(self, t_obs, now):
        """관측 시각 t_obs 의 헤딩(raw 이력 보간). 이력 밖이면 가장 가까운 끝값. 모르면 None."""
        if not self._fresh(now):
            return None
        if not self.use_raw:
            return self.yaw[0]
        h = self.hist
        if t_obs <= h[0][0]:
            r = h[0][1]
        elif t_obs >= h[-1][0]:
            r = h[-1][1]
        else:
            r = h[-1][1]
            for i in range(len(h) - 1, 0, -1):
                t0, r0 = h[i - 1]
                t1, r1 = h[i]
                if t0 <= t_obs <= t1:
                    a = 0.0 if t1 <= t0 else (t_obs - t0) / (t1 - t0)
                    r = wrap360(r0 + a * wrap180(r1 - r0))
                    break
        return wrap360(r + self.offset)

    def turn_rate(self, now, window=0.3):
        """raw 이력으로 잰 회전율(°/s, 절댓값). raw 를 안 쓰거나 이력이 모자라면 None(= 거를 수 없음)."""
        if not self.use_raw or self.raw is None or len(self.hist) < 2:
            return None
        t1, r1 = self.hist[-1]
        t0, r0 = self.hist[0]
        for tt, rr in reversed(self.hist[:-1]):
            t0, r0 = tt, rr
            if t1 - tt >= window:
                break
        if t1 - t0 <= 0.0:
            return None
        return abs(wrap180(r1 - r0)) / (t1 - t0)


# ==================== 부표 지도 ====================
class Landmark:
    __slots__ = ('sx', 'sy', 'sxx', 'syy', 'n', 't_last')

    def __init__(self, x, y, t):
        self.sx, self.sy, self.sxx, self.syy, self.n, self.t_last = x, y, x * x, y * y, 1, t

    def add(self, x, y, t):
        self.sx += x
        self.sy += y
        self.sxx += x * x
        self.syy += y * y
        self.n += 1
        self.t_last = t

    @property
    def mean(self):
        return self.sx / self.n, self.sy / self.n

    @property
    def spread(self):
        """관측 흩어짐 RMS(m) = √(var_x + var_y)."""
        mx, my = self.mean
        v = max(0.0, self.sxx / self.n - mx * mx) + max(0.0, self.syy / self.n - my * my)
        return math.sqrt(v)


class LandmarkMap:
    """남색 부표 GPS(지역 평면) 지도. 관측을 가장 가까운 부표(assoc_m 안)에 묶고, 없으면 새 부표.

    확정 = 관측 n ≥ confirm_n 이고 흩어짐 ≤ max_err(부표 GPS 오차 상한). 오탐 하나가 중심을 틀지 않게
    fit_square 는 확정 부표만 쓴다. 최대 max_n 개(넘으면 가장 오래 안 보인 미확정부터 버림).
    """

    def __init__(self, assoc_m, confirm_n, max_err, max_n=24):
        self.assoc_m = float(assoc_m)
        self.confirm_n = int(confirm_n)
        self.max_err = float(max_err)
        self.max_n = int(max_n)
        self.lms = []

    def reset(self):
        self.lms = []

    def add(self, x, y, t):
        best, bd = None, math.inf
        for lm in self.lms:
            d = _dist(lm.mean, (x, y))
            if d < bd:
                best, bd = lm, d
        if best is not None and bd <= self.assoc_m:
            best.add(x, y, t)
            return
        if len(self.lms) >= self.max_n:
            cand = [lm for lm in self.lms if not self.is_confirmed(lm)] or self.lms
            self.lms.remove(min(cand, key=lambda lm: lm.t_last))
        self.lms.append(Landmark(x, y, t))

    def is_confirmed(self, lm):
        return lm.n >= self.confirm_n and lm.spread <= self.max_err

    def confirmed(self):
        return [lm.mean for lm in self.lms if self.is_confirmed(lm)]


# ==================== 사각형 맞추기 ====================
class SquareFit:
    """fit_square 결과. kind 0 = 없음, 2 = 두 점(변/대각선), 3 = 세 점(직각) + 평행사변형 보완, 4 = 네 점.

    corners  : 반시계 네 꼭짓점(kind 3 은 보완점 포함, kind 2·0 은 None)
    center   : 도형 중심(kind 4 = 네 꼭짓점 평균, kind 3 = 빗변 중점, 그 밖 None)
    predicted: 아직 못 본 꼭짓점 예측(kind 2 = 가설 순서대로, kind 3 = 보완점 하나)
    resid    : 변·대각선 길이 오차 최대(m)
    """

    def __init__(self, kind=0, corners=None, center=None, predicted=None, resid=math.inf, pair=None):
        self.kind = kind
        self.corners = corners
        self.center = center
        self.predicted = predicted or []
        self.resid = resid
        self.pair = pair


SQRT2 = math.sqrt(2.0)


def _combos(seq, k):
    n = len(seq)
    if k == 2:
        for i in range(n):
            for j in range(i + 1, n):
                yield (seq[i], seq[j])
    elif k == 3:
        for i in range(n):
            for j in range(i + 1, n):
                for m in range(j + 1, n):
                    yield (seq[i], seq[j], seq[m])
    elif k == 4:
        for i in range(n):
            for j in range(i + 1, n):
                for m in range(j + 1, n):
                    for q in range(m + 1, n):
                        yield (seq[i], seq[j], seq[m], seq[q])


def fit_square(pts, side, tol, boat=None):
    """확정 부표 좌표 → SquareFit. 순수 함수. 네 점 > 세 점 > 두 점 순으로 가장 잘 맞는 것.

    네 점: 반시계로 놓고 네 변 |s − side| ≤ tol, 두 대각선 |d − side·√2| ≤ tol·√2(길이에 비례한 허용).
    세 점: 한 꼭짓점 B 에서 두 다리 ≈ side, 빗변 ≈ side·√2(±tol·√2) → 넷째 D = A + C − B(평행사변형 보완).
    resid = 변 오차와 대각선 오차/√2 중 최대.
    두 점: 거리 ≈ side(변) 또는 ≈ side·√2(대각선) 중 가까운 쪽. 변이면 나머지 두 점을 배 반대쪽 가설 먼저,
           대각선이면 중점에서 수직으로 대각선 반 길이 두 점.
    """
    diag = side * math.sqrt(2.0)
    pts = list(pts)
    best = None
    for four in _combos(pts, 4):
        q = order_ccw(list(four))
        sides = [_dist(q[i], q[(i + 1) % 4]) for i in range(4)]
        diags = [_dist(q[0], q[2]), _dist(q[1], q[3])]
        r = max([abs(s - side) for s in sides] + [abs(d - diag) / SQRT2 for d in diags])
        if r <= tol and (best is None or r < best.resid):
            c = (sum(p[0] for p in q) / 4.0, sum(p[1] for p in q) / 4.0)
            best = SquareFit(4, q, c, [], r)
    if best is not None:
        return best
    for three in _combos(pts, 3):
        for bi in range(3):
            B = three[bi]
            A, C = [three[i] for i in range(3) if i != bi]
            r = max(abs(_dist(A, B) - side), abs(_dist(C, B) - side), abs(_dist(A, C) - diag) / SQRT2)
            if r <= tol and (best is None or r < best.resid):
                D = (A[0] + C[0] - B[0], A[1] + C[1] - B[1])
                c = ((A[0] + C[0]) / 2.0, (A[1] + C[1]) / 2.0)
                best = SquareFit(3, order_ccw([A, B, C, D]), c, [D], r)
    if best is not None:
        return best
    for A, B in _combos(pts, 2):
        d = _dist(A, B)
        es, ed = abs(d - side), abs(d - diag) / SQRT2
        if min(es, ed) > tol:
            continue
        r = min(es, ed)
        if best is not None and r >= best.resid:
            continue
        ux, uy = (B[0] - A[0]) / d, (B[1] - A[1]) / d
        nx, ny = -uy, ux
        if es <= ed:                                   # 변
            far = 1.0
            if boat is not None and (nx * (boat[0] - A[0]) + ny * (boat[1] - A[1])) > 0.0:
                far = -1.0                             # 배가 +n 쪽 → 사각형은 −n 쪽일 가능성이 크다
            pred = []
            for s in (far, -far):
                pred += [(A[0] + s * side * nx, A[1] + s * side * ny), (B[0] + s * side * nx, B[1] + s * side * ny)]
        else:                                          # 대각선
            mx, my = (A[0] + B[0]) / 2.0, (A[1] + B[1]) / 2.0
            h = d / 2.0
            pred = [(mx + h * nx, my + h * ny), (mx - h * nx, my - h * ny)]
        best = SquareFit(2, None, None, pred, r, pair=(A, B))
    return best or SquareFit()


# ==================== 명령 ====================
def spin_cmd(err_cw_deg, phase_sec, pulse_sec, pause_sec):
    """목표 헤딩 쪽으로 제자리 선회 펄스. err > 0 = 시계 방향(오른쪽) → 5000, 아니면 6000. 쉬는 동안 50000."""
    cyc = pulse_sec + max(0.0, pause_sec)
    on = pause_sec <= 0.0 or (phase_sec % cyc) < pulse_sec
    if not on:
        return STOP_HOLD
    return SPIN_RIGHT if err_cw_deg > 0.0 else SPIN_LEFT


def steer_value(rel_left_deg):
    """상대 조향(좌 +) → /candidate_angle(0~360), ±STEER_LIMIT_DEG 로 자름."""
    r = max(-STEER_LIMIT_DEG, min(STEER_LIMIT_DEG, rel_left_deg))
    return (r + 360.0) % 360.0


def is_steer(v):
    """v 가 조향 값(±80 이내를 0~360 으로 낸 것)인가."""
    if v in (CANDIDATE_INVALID, STOP_HOLD, SPIN_RIGHT, SPIN_LEFT, REVERSE_REL):
        return False
    return 0.0 <= v <= STEER_LIMIT_DEG or 360.0 - STEER_LIMIT_DEG <= v < 360.0


def output_allowed(v, state):
    """출력 집합 계약: INIT·DONE = 20000 만, 그 밖 = {±80 조향, 5000, 6000, 50000, 180}. 🚨 DONE 전 20000 금지."""
    if state in ('INIT', 'DONE'):
        return v == CANDIDATE_INVALID
    return v in (SPIN_RIGHT, SPIN_LEFT, STOP_HOLD, REVERSE_REL) or is_steer(v)


def near_hazard(near, length, beam, clear_m):
    """LiDAR 최근접 (거리, 상대방위 좌 +) → (정면 막힘, 선회 금지). 순수 함수. near None → (False, False).

    정면 막힘 = 물체가 선수 앞(전방 성분 > 0)이고 옆으로 선체 반폭 + clear_m 안, 선수에서 clear_m 안
                → 전진 금지, 짧게 후진(180) — Davidson 답 v2 3번 '부표에 너무 가까울 때만'.
    선회 금지 = 거리 < 선체 반대각선(제자리 선회 휩쓸림 반지름) + clear_m.
    LiDAR 원점 = 선체 중심 가정(⚠️ 실측 필요 — 2번배는 선수 근처).
    """
    if near is None:
        return False, False
    r, rel = near
    if not (math.isfinite(r) and math.isfinite(rel)):
        return False, False
    fwd = r * math.cos(math.radians(rel))
    lat = r * math.sin(math.radians(rel))
    front = fwd > 0.0 and abs(lat) < beam / 2.0 + clear_m and (fwd - length / 2.0) < clear_m
    spin_blk = r < math.hypot(length / 2.0, beam / 2.0) + clear_m
    return front, spin_blk


class ContinuousTimer:
    """조건이 연속으로 참인 시간. 거짓이면 0 부터."""

    def __init__(self):
        self.t0 = None

    def reset(self):
        self.t0 = None

    def update(self, ok, now):
        if not ok:
            self.t0 = None
            return 0.0
        if self.t0 is None:
            self.t0 = now
        return now - self.t0


# ==================== FSM ====================
class SquareFSM:
    """사각형 위치유지 FSM. 시각·센서는 주입 — ROS 없이 테스트한다. step() 한 번 = 틱 하나 = 출력 하나.

    p = check_square_params 결과 + 'hold_time_sec', 'unknown_giveup_sec'.
    """

    STATES = ('INIT', 'DORI', 'SEEK3', 'ENTER', 'FIND4', 'HOLD', 'DONE')

    def __init__(self, p):
        self.p = dict(p)
        self.map = LandmarkMap(self.p['buoy_assoc_m'], self.p['buoy_confirm_obs'], self.p['buoy_pos_max_err_m'])
        self.timer = ContinuousTimer()
        self.reset(0.0)

    # ---- 상태 ----
    def reset(self, now):
        self.state = 'INIT'
        self.t_enter = now
        self.t_state = now
        self.t_engage = None
        self.map.reset()
        self.timer.reset()
        self.blind_t0 = None
        self.err_t = None              # 마지막 '센서 없음' ERROR 시각
        self.init_warned = False
        self.unsafe_warned = False
        self.h0 = None                 # DORI 기준 헤딩
        self.look_idx = 0
        self.look_t0 = None            # 이번 목표로 돌기 시작한 시각
        self.dwell_t0 = None           # 이번 목표에서 멈춰 보기 시작한 시각
        self.targets = []              # SEEK3: 예측 꼭짓점 좌표 목록
        self.seek_wait = False         # SEEK3 를 다 보고 돌아왔으면 DORI 한 바퀴를 다 돈 뒤에만 다시 SEEK3
        self.seek_goal = None          # SEEK3 에서 다가갈 점(못 봤을 때 한 번)
        self.seek_goal_t = None
        self.seek_moved = False
        self.driving = False           # 위치유지 히스테리시스
        self.rev_until = None          # 후진 펄스 끝
        self.rev_cool_until = None     # 다음 후진 가능 시각
        self.center = None             # 마지막 중심(지역 평면)
        self.quad = None
        self.fit = SquareFit()
        self.inside = False
        self.success = None            # True = 성공, False = 포기 (DONE 일 때)
        self.done_reason = ''

    def _go(self, s, now, logs, why=''):
        if s != self.state:
            logs.append(('INFO', f"⚓ 사각형 위치유지 {self.state} → {s}" + (f" ({why})" if why else "")))
            self.state = s
            self.t_state = now
            self.look_idx = 0
            self.look_t0 = None
            self.dwell_t0 = None
            self.seek_goal = None

    def _done(self, now, logs, success, text):
        if success:
            logs.append(('INFO', text))
        else:
            logs.append(('ERROR', text))
        self.success = success
        self.done_reason = text
        self._go('DONE', now, logs)

    # ---- 보조 ----
    def _look(self, now, heading, target_heading, dt_phase):
        """target_heading 으로 돌고 look_dwell_sec 동안 정지해 본다. → (명령, 이 목표 끝났나)."""
        p = self.p
        if self.look_t0 is None:
            self.look_t0 = now
        err = wrap180(target_heading - heading)
        if self.dwell_t0 is None and abs(err) > p['look_tol_deg'] \
                and (now - self.look_t0) < p['look_turn_max_sec']:
            return spin_cmd(err, dt_phase, p['spin_pulse_sec'], p['spin_pause_sec']), False
        if self.dwell_t0 is None:
            self.dwell_t0 = now
        if (now - self.dwell_t0) >= p['look_dwell_sec']:
            self.look_t0 = None
            self.dwell_t0 = None
            return STOP_HOLD, True
        return STOP_HOLD, False

    def _drive_to(self, xy, heading, target, phase):
        p = self.p
        brg = bearing_to(xy[0], xy[1], target[0], target[1])
        rel_left = wrap180(heading - brg)
        if abs(rel_left) > p['steer_spin_deg']:
            return spin_cmd(-rel_left, phase, p['spin_pulse_sec'], p['spin_pause_sec'])
        return steer_value(rel_left)

    def _keep(self, xy, heading, center, phase):
        """중심 히스테리시스 유지. → 명령 또는 None(정지 구간 — 호출자가 보기·정지를 고른다)."""
        d = _dist(xy, center)
        if d > self.p['hold_out_m']:
            self.driving = True
        elif d < self.p['hold_in_m']:
            self.driving = False
        if self.driving:
            return self._drive_to(xy, heading, center, phase)
        return None

    def _safety(self, now, cmd, near):
        """근접 물체 안전 덮개 — near_hazard 참고. 후진 펄스는 정면 막힘일 때만(쿨다운), 선회는 휩쓸림 원 안이면 금지,
        정면 막힘이면 전진 조향 금지. 선회가 막히면 그 쪽으로 최대 조향(전진하며 돈다, 정면 막힘이 아니면)."""
        p = self.p
        if self.rev_until is not None and now < self.rev_until:
            return REVERSE_REL
        front, spin_blk = near_hazard(near, p['hull_length_m'], p['hull_beam_m'], p['close_m'])
        if front and (self.rev_cool_until is None or now >= self.rev_cool_until):
            self.rev_until = now + p['reverse_pulse_sec']
            self.rev_cool_until = self.rev_until + p['reverse_pulse_sec']
            return REVERSE_REL
        if cmd in (SPIN_RIGHT, SPIN_LEFT) and spin_blk:
            if front:
                return STOP_HOLD
            return steer_value(-STEER_LIMIT_DEG if cmd == SPIN_RIGHT else STEER_LIMIT_DEG)
        if is_steer(cmd) and front:
            return STOP_HOLD
        return cmd

    # ---- 한 틱 ----
    def step(self, now, goal_dist=None, xy=None, heading=None, bearings_ok=False, obs=(), near=None):
        """한 틱. → (/candidate_angle 값, [(심각도, 메시지)]).

        goal_dist  : 쓸 수 있는 /goal_distance(m) 또는 None
        xy         : 배 위치(지역 평면, fix 신선) 또는 None
        heading    : 지금 헤딩(HeadingFuser) 또는 None
        bearings_ok: /buoy_bearings 가 bearings_stale_sec 안에 정상 형식으로 왔나
        obs        : 이번 틱 새 부표 관측 좌표 [(x, y)] (노드가 투영)
        near       : LiDAR 최근접 (거리 m, 상대방위 좌 +°) 또는 None
        """
        p = self.p
        logs = []
        for o in obs:
            self.map.add(o[0], o[1], now)
        if self.state == 'DONE':
            return CANDIDATE_INVALID, logs

        if self.state == 'INIT':
            if (now - self.t_enter) >= p['init_min_sec'] and goal_dist is not None \
                    and goal_dist < p['square_engage_m']:
                self.t_engage = now
                self._go('DORI', now, logs, f"WP {goal_dist:.1f} m < {p['square_engage_m']:g} m — 사각형 찾기")
            else:
                if not self.init_warned and (now - self.t_enter) > INIT_WARN_SEC and goal_dist is None:
                    logs.append(('WARN', f"/goal_distance 없음 {INIT_WARN_SEC:g} s — INIT(20000) 계속. north·WP2 확인"))
                    self.init_warned = True
                return CANDIDATE_INVALID, logs

        # ---- 공통: 시간 상한 ----
        if (now - self.t_engage) >= p['square_max_sec']:
            self._done(now, logs, False, f"사각형 위치유지 {p['square_max_sec']:g} s 안에 못 끝냄 → 포기, DONE")
            return CANDIDATE_INVALID, logs

        # ---- 공통: 센서 ----
        missing = []
        if not bearings_ok:
            missing.append('/buoy_bearings(stale·형식)')
        if xy is None:
            missing.append('GPS fix')
        if heading is None:
            missing.append('헤딩(/imu/yaw·/imu/yaw_raw)')
        if missing:
            self.timer.reset()
            self.inside = False
            if self.blind_t0 is None:
                self.blind_t0 = now
            if (now - self.blind_t0) >= p['unknown_giveup_sec']:
                self._done(now, logs, False, f"{', '.join(missing)} 없음 {now - self.blind_t0:.1f} s → 포기, DONE")
                return CANDIDATE_INVALID, logs
            if self.err_t is None or (now - self.err_t) >= ERROR_REPEAT_SEC:
                logs.append(('ERROR', f"사각형 위치유지: {', '.join(missing)} 없음 → 정지(50000), "
                                      f"{p['unknown_giveup_sec']:g} s 이어지면 포기"))
                self.err_t = now
            return STOP_HOLD, logs
        self.blind_t0 = None
        if self.h0 is None:
            self.h0 = heading

        self.fit = fit = fit_square(self.map.confirmed(), p['square_side_m'], p['square_shape_tol_m'], boat=xy)
        if fit.kind >= 3:
            self.center, self.quad = fit.center, fit.corners
        phase = now - self.t_engage
        cmd = STOP_HOLD

        if self.state == 'DORI':
            if fit.kind >= 3:
                self._go('ENTER', now, logs, '부표 3개 이상')
            elif fit.kind == 2 and not self.seek_wait:
                self.targets = list(fit.predicted)
                self._go('SEEK3', now, logs, '부표 2개')
            else:
                offs = [0.0]
                for k in (1, 2, 3):
                    offs += [k * p['dori_amp_deg'], -k * p['dori_amp_deg']]
                tgt = wrap360(self.h0 + offs[self.look_idx % len(offs)])
                cmd, fin = self._look(now, heading, tgt, phase)
                if fin:
                    self.look_idx += 1
                    if self.look_idx % len(offs) == 0:
                        self.h0 = heading          # 한 바퀴 다 봤으면 지금 방위 기준으로 다시
                        self.seek_wait = False
        if self.state == 'SEEK3':
            if fit.kind >= 3:
                self._go('ENTER', now, logs, '부표 3개')
            elif fit.kind < 2:
                self._go('DORI', now, logs, '부표 2개 아님')
            elif self.look_idx >= len(self.targets) and self.seek_goal is None and not self.seek_moved \
                    and fit.pair is not None:
                # '그 기준으로 돌면서' — 아는 두 부표 쪽으로 다가가(먼 꼭짓점이 LiDAR·카메라 거리 안으로) 다시 본다
                self.seek_goal = seek_approach_point(fit.pair, xy, p['square_side_m'], p['entry_standoff_m'])
                self.seek_moved = True
                self.seek_goal_t = now
                logs.append(('INFO', '예측 꼭짓점 못 봄 — 두 부표 쪽으로 다가가 다시 본다'))
            elif self.look_idx >= len(self.targets) and self.seek_goal is None:
                self.h0 = heading
                self.seek_wait = True
                self.seek_moved = False
                self._go('DORI', now, logs, '예측 꼭짓점 다 봄 — 도리도리 한 바퀴 더')
            if self.state == 'SEEK3' and self.seek_goal is not None:
                if _dist(xy, self.seek_goal) < p['hold_out_m'] \
                        or (now - self.seek_goal_t) > p['look_turn_max_sec'] * SEEK_MOVE_MAX_FACTOR:
                    self.seek_goal = None
                    self.look_idx = 0
                    self.targets = list(fit.predicted)
                else:
                    cmd = self._drive_to(xy, heading, self.seek_goal, phase)
            elif self.state == 'SEEK3':
                tx, ty = self.targets[self.look_idx]
                cmd, fin = self._look(now, heading, bearing_to(xy[0], xy[1], tx, ty), phase)
                if fin:
                    self.look_idx += 1
        if self.state == 'ENTER':
            if fit.kind < 3 and self.center is None:
                self._go('DORI', now, logs, '부표 3개 아님')
            else:
                tgt, staging = entry_target(xy, self.quad, self.center, p['entry_clear_m'], p['entry_standoff_m'])
                if not staging and _dist(xy, self.center) < p['hold_out_m']:
                    self.driving = True            # 히스테리시스 안쪽(hold_in)까지는 계속 다가간다
                    self._go('HOLD' if fit.kind == 4 else 'FIND4', now, logs, '중심 도착')
                else:
                    cmd = self._drive_to(xy, heading, tgt, phase)
        if self.state == 'FIND4':
            if fit.kind == 4:
                self._go('HOLD', now, logs, '부표 4개')
            else:
                k = self._keep(xy, heading, self.center, phase)
                if k is not None:
                    cmd = k
                    self.look_t0 = None
                    self.dwell_t0 = None
                else:
                    pred = self.fit.predicted[0] if (fit.kind == 3 and fit.predicted) else None
                    base = bearing_to(xy[0], xy[1], pred[0], pred[1]) if pred is not None else heading
                    tgt = wrap360(base + 90.0 * (self.look_idx % 4))
                    cmd, fin = self._look(now, heading, tgt, phase)
                    if fin:
                        self.look_idx += 1
        if self.state == 'HOLD':
            if fit.kind < 4:
                self.timer.reset()
                self.inside = False
                self._go('FIND4', now, logs, '4점 맞춤 깨짐')
                k = self._keep(xy, heading, self.center, phase)
                cmd = STOP_HOLD if k is None else k
            else:
                inset = p['buoy_radius_m'] + p['square_inside_margin_m']
                safe = safe_region_exists(self.quad, inset, p['hull_length_m'], p['hull_beam_m'])
                if not safe and not self.unsafe_warned:
                    logs.append(('ERROR', f"🚨 안전영역 소멸: 추정 사각형(모양 오차 {fit.resid:.2f} m)을 {inset:g} m 줄이면 "
                                          f"선체({p['hull_length_m']:g}×{p['hull_beam_m']:g} m)가 들어갈 자리 없음 → "
                                          f"성공 금지(정지 유지)"))
                    self.unsafe_warned = True
                self.inside = safe and hull_inside(self.quad, xy[0], xy[1], heading,
                                                   p['hull_length_m'], p['hull_beam_m'], inset)
                held = self.timer.update(self.inside, now)
                if held >= p['hold_time_sec']:
                    c_ll = self.center
                    self._done(now, logs, True,
                               f"✅ 사각형 위치유지: 선체 전체 내부 연속 {p['hold_time_sec']:g} s 달성 → 완료 "
                               f"(중심 x {c_ll[0]:.2f}, y {c_ll[1]:.2f} m, 모양 오차 {fit.resid:.2f} m)")
                    return CANDIDATE_INVALID, logs
                k = self._keep(xy, heading, self.center, phase)
                cmd = STOP_HOLD if k is None else k

        cmd = self._safety(now, cmd, near)
        return cmd, logs
