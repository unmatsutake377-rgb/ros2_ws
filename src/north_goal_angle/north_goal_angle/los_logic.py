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


def get_dynamic_arrive_radius(wp_mode, speed_mps, dock_mode=7):
    """도착 판정 반경(m) — 도킹은 좁게, 그 외는 속도에 비례해(빠르면 넓게 오버슈트 방지)."""
    if wp_mode == dock_mode:
        return 2.0
    if speed_mps < 2.0:
        return 2.5
    if speed_mps < 5.0:
        return 3.5
    return 4.5
