"""LOS(Line-of-Sight) 유도 — ROS 비의존 순수 로직.

현재 north_goal_angle 은 '현 위치 → 목표'의 직접 방위만 낸다. 그러면 조류·초기오차로
경로에서 밀리면 그 밀림을 되돌리지 못하고 비스듬히 접근한다. LOS 는 **이전 웨이포인트→목표를
잇는 '경로 선'** 을 기준으로, 배가 그 선에서 얼마나 벗어났나(cross-track error, XTE)를 재서
전방주시거리(lookahead) 안에서 선으로 되돌아오도록 방위를 보정한다.

ILOS(적분 LOS): 조류·바람처럼 **지속적으로 미는 힘**은 XTE 를 계속 남기므로, XTE 를 적분해
정상상태 오차를 없앤다(anti-windup 은 노드에서 클램프).

geopy 없이 haversine 으로 거리를 구해 배 없이(Mac)도 테스트한다.
프레임: 방위 0=북, 시계방향 증가(compass). north_goal_angle 계약과 동일.
"""

import math

_R_EARTH_M = 6371000.0


def calc_dist(lat1, lon1, lat2, lon2):
    """두 좌표 간 대권 거리(m) — haversine (수백 m 범위에서 geopy 와 오차 무시)."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * _R_EARTH_M * math.asin(min(1.0, math.sqrt(a)))


def calc_angle(lat1, lon1, lat2, lon2):
    """1→2 초기 방위(°). 0=북, 시계방향(compass)."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    x = math.sin(dl) * math.cos(p2)
    y = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0


def get_dynamic_lookahead(speed_mps, speed_gain, min_m, max_m):
    """속도 비례 전방주시거리. 빠를수록 멀리 봐 완만하게, 느릴수록 가까이 봐 민첩하게.
    min/max 로 클램프."""
    la = speed_mps * speed_gain
    return max(min_m, min(max_m, la))


def cross_track_error(p_lat, p_lon, wp_lat, wp_lon, boat_lat, boat_lon):
    """경로선(p→wp) 기준 배의 부호 있는 XTE(m) 와 경로 방위(°) 반환.

    XTE 부호: **양수 = 배가 경로 진행방향 기준 오른쪽으로 밀림**.
      경로 방위(path_bearing)와 'p→배' 방위(boat_bearing)의 최단 각차 sin 으로 낸다.
    """
    path_bearing = calc_angle(p_lat, p_lon, wp_lat, wp_lon)
    boat_bearing = calc_angle(p_lat, p_lon, boat_lat, boat_lon)
    boat_dist = calc_dist(p_lat, p_lon, boat_lat, boat_lon)
    ang = (boat_bearing - path_bearing + 540.0) % 360.0 - 180.0   # 배가 오른쪽이면 +
    xte = boat_dist * math.sin(math.radians(ang))
    return xte, path_bearing


def calc_los_bearing(p_lat, p_lon, wp_lat, wp_lon, boat_lat, boat_lon,
                     lookahead_m, integral_xte=0.0, ki=0.0):
    """경로선으로 되돌아오는 목표 방위(°) 와 이번 XTE(m) 반환.

    배가 오른쪽으로 밀렸으면(XTE>0) 방위를 왼쪽으로(경로방위보다 작게) 틀어 선으로 복귀한다.
    lookahead 가 클수록 완만하게 복귀(오버슈트↓), 작을수록 급하게.
    ILOS: effective_xte = xte + ki*∫xte 로 지속 외란을 상쇄.
    """
    xte, path_bearing = cross_track_error(p_lat, p_lon, wp_lat, wp_lon, boat_lat, boat_lon)
    effective = xte + ki * integral_xte
    # 오른쪽 밀림(+)이면 왼쪽으로(−) 보정 → path_bearing 에서 빼준다.
    correction = math.degrees(math.atan2(effective, max(0.1, lookahead_m)))
    desired = (path_bearing - correction + 360.0) % 360.0
    return desired, xte


# 경로 길이가 이보다 짧으면 '경로선'이 정의되지 않는다(예: WP8·WP9 같은 좌표) → 직접 방위.
PATH_MIN_LEN_M = 0.01


def along_track(p_lat, p_lon, wp_lat, wp_lon, boat_lat, boat_lon):
    """경로선(p→wp) 위로 투영한 배의 진행 거리 s(m) 와 경로 길이 L(m) 반환.

    s = |p→배| · cos(경로 방위와 p→배 방위의 각차). s ≥ L 이면 배가 WP 를 진행 방향으로 지나쳤다.
    """
    path_len = calc_dist(p_lat, p_lon, wp_lat, wp_lon)
    path_bearing = calc_angle(p_lat, p_lon, wp_lat, wp_lon)
    boat_bearing = calc_angle(p_lat, p_lon, boat_lat, boat_lon)
    boat_dist = calc_dist(p_lat, p_lon, boat_lat, boat_lon)
    ang = (boat_bearing - path_bearing + 540.0) % 360.0 - 180.0
    return boat_dist * math.cos(math.radians(ang)), path_len


def calc_guidance_bearing(p_lat, p_lon, wp_lat, wp_lon, boat_lat, boat_lon,
                          lookahead_m, arrive_radius_m, integral_xte=0.0, ki=0.0):
    """LOS + 지나침 복귀. (목표 방위°, 이번 XTE m, 직접 방위 여부) 반환.

    LOS 는 경로선을 따라가게 하므로 WP 를 반경 밖으로 지나치면(dwell 대기·오버슈트) 경로
    연장선을 따라 계속 멀어진다(전체 코스 시뮬 150 m 이탈, 계약서_전체코스_20261104 §2-1).
      · 경로 길이 < PATH_MIN_LEN_M  → 경로선이 없다 → 현위치→WP 직접 방위, XTE 0
      · s ≥ L 이고 WP 거리 ≥ 도착 반경 → 지나쳤다 → 현위치→WP 직접 방위, XTE 0
      · 그 밖(지나치기 전, 또는 반경 안) → 기존 calc_los_bearing 그대로
    XTE 0 을 돌려주므로 노드의 ILOS 적분은 직접 방위 동안 그대로 멈춘다(동결).
    도착 반경은 노드가 get_arrive_radius 로 구해 넘긴다(도착 판정과 같은 값).
    """
    s, path_len = along_track(p_lat, p_lon, wp_lat, wp_lon, boat_lat, boat_lon)
    if path_len < PATH_MIN_LEN_M:
        return calc_angle(boat_lat, boat_lon, wp_lat, wp_lon), 0.0, True
    if s >= path_len and calc_dist(boat_lat, boat_lon, wp_lat, wp_lon) >= arrive_radius_m:
        return calc_angle(boat_lat, boat_lon, wp_lat, wp_lon), 0.0, True
    bearing, xte = calc_los_bearing(p_lat, p_lon, wp_lat, wp_lon, boat_lat, boat_lon,
                                    lookahead_m, integral_xte, ki)
    return bearing, xte, False


# ---- 위치유지·탐색(mode 2·3) 도착 반경 (계약서_전체코스_20261104 §10 추가 결정 3) ----
# 속도 반경(2.5 m)은 ship_back 유지 거리 안이라, ship_back 이 손 뗀(20000) 뒤 north 가 dwell 을
# 채우려 부표 좌표로 들어가 접촉했다 → mode 2·3 은 유지 거리 밖까지 넓힌 고정 반경을 쓴다.
# 노드 파라미터 arrive_radius_hold_modes_m (north_goal_angle.yaml 과 같은 값 — 테스트가 잡는다).
# 3.5 > ship_back hold_dist 2.5 + tol 0.4 = 2.9 (유지 위치에서 이미 도착 반경 안이어야 한다).
DEFAULT_ARRIVE_RADIUS_HOLD_MODES_M = 3.5
HOLD_MODES = (2, 3)
# ship_back 유지 거리 바깥 끝 = hold_dist_m 2.5 + dist_tol_m 0.4 (ship_back.yaml). 반경이 이 이하면
# 유지 위치가 반경 밖이라 DONE(20000) 뒤 north 가 부표 쪽으로 파고든다 → 시작 WARN(값은 그대로 쓴다).
# ship_back.yaml 의 두 값을 바꾸면 여기도 같이 바꾼다(테스트가 묶어 둔다).
SHIP_BACK_HOLD_OUTER_M = 2.9


def sanitize_hold_radius(value, default=DEFAULT_ARRIVE_RADIUS_HOLD_MODES_M):
    """arrive_radius_hold_modes_m 검증 → (쓸 값, 정상 여부). 유한한 양수 실수만 정상.

    bool·문자열·NaN·inf·0 이하는 잘못된 값 → (default, False). 노드가 시작 ERROR 를 낸다.
    """
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return float(default), False
    v = float(value)
    if not math.isfinite(v) or v <= 0.0:
        return float(default), False
    return v, True


def hold_radius_inside_hold_dist(radius_m, hold_outer_m=SHIP_BACK_HOLD_OUTER_M):
    """정상(양수) 반경이 ship_back 유지 거리 바깥 끝 이하인가 → 노드가 시작 WARN 1회."""
    return radius_m <= hold_outer_m


# ---- 옛 녹화 파일 dwell 경고 (계약서_전체코스_20261104 §10 결정 6 보충 ③) ----
# mode 2·3 dwell 은 0.5 s 로 줄였다(결정 3). 재빌드 전 녹화 도구 템플릿·옛 녹화 파일은 3.0 이라
# DONE(20000) 뒤 dwell 동안 north 가 WP(부표) 쪽으로 간다 → 시작 때 WARN 1회. 값은 그대로 쓴다.
HOLD_DWELL_WARN_SEC = 1.0


def stale_hold_dwells(waypoints, labels=(), hold_modes=HOLD_MODES,
                      limit_sec=HOLD_DWELL_WARN_SEC):
    """mode 2·3 이면서 dwell > limit_sec 인 WP → [(순번, 구역, mode, dwell), ...].

    waypoints 는 parse_waypoints 결과 [[lat, lon, mode, dwell], ...]. labels 는 같은 순서의
    구역 이름(waypoint_labels) — 모자라면 ''. limit_sec 와 같으면 경고 대상이 아니다.
    """
    out = []
    for i, wp in enumerate(waypoints):
        mode, dwell = wp[2], wp[3]
        if mode in hold_modes and dwell > limit_sec:
            label = labels[i] if i < len(labels) else ""
            out.append((i, label, mode, dwell))
    return out


def get_arrive_radius(wp_mode, speed_mps, hold_radius_m=DEFAULT_ARRIVE_RADIUS_HOLD_MODES_M,
                      hold_modes=HOLD_MODES, dock_mode=7):
    """노드가 쓰는 도착 반경(m). mode 2·3 은 hold_radius_m 고정, 그 밖은 get_dynamic_arrive_radius."""
    if wp_mode in hold_modes:
        return hold_radius_m
    return get_dynamic_arrive_radius(wp_mode, speed_mps, dock_mode)


def get_dynamic_arrive_radius(wp_mode, speed_mps, dock_mode=7):
    """도착 판정 반경(m) — 도킹은 좁게, 그 외는 속도에 비례해(빠르면 넓게 오버슈트 방지)."""
    if wp_mode == dock_mode:
        return 2.0
    if speed_mps < 2.0:
        return 2.5
    if speed_mps < 5.0:
        return 3.5
    return 4.5
