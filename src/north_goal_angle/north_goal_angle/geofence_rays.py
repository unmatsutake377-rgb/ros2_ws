"""Geofence 광선 — ROS 비의존 순수 기하 (north_goal_angle 의 /geofence_state 계산부).

경기장 폴리곤을 **'가짜 LiDAR'** 로 쏜다: 각 상대방위마다 "이 방향으로 몇 m 가면 경계 밖인가".
노드(north_goal_angle.py)는 가드(폴리곤·GPS·IMU 신선도·이미 밖)만 하고 여기 `cast_rays` 를 부른다.
geopy·rclpy 없이 돌아가므로 Mac 에서도 테스트한다 (test/test_geofence_rays.py).

🚨 부호 규약 (계약서_geofence부호_20261104) — **상대방위 rel 은 좌 양수(좌현 = +)** 다.
  · 소비자 ship_direction 은 LiDAR 프레임 `a − 80`(80=정면, 커질수록 좌)과 /candidate_angle
    (vision_geom "왼쪽이 +")과 같은 부호로 배열 인덱스를 잡는다 → 발행도 그 규약을 따른다.
  · 배열 인덱스 0 (rel = ray_min_deg = −80) = **우현 끝**, 마지막 인덱스 (+80) = **좌현 끝**.
  · /imu/yaw 는 나침반(0=북, 시계방향 CW+). 좌 양수 rel 을 절대 방위로 바꾸려면
        absolute = yaw − rel          ← 🚨 `-` 가 맞다. `+` 로 "고치면" 좌우가 거울이 된다.
    (2026-11-04 전까지 `yaw + rel` 이었고, 그래서 소비자 쪽에서 좌우가 뒤집혀 있었다.
     avoid_sim "실코드 + 경계" 가 36/40 에 그친(4 런 정체) 원인. test_geofence_rays.py 비대칭 케이스가 이걸 고정한다.)
"""

import math


def _ray_polygon_dist(o, d, pts):
    """원점 o 에서 방향 d 로 쏜 광선이 폴리곤 변에 처음 닿는 거리. 안 닿으면 None.

    ★ 이게 '벽은 선이다'를 정직하게 푸는 방법이다. 최근접점 한 방향으로 원뿔을 막으면
      모서리에서 정면(탈출구)이 원뿔 사이 '틈'으로 열려 배가 대각선으로 빠져나간다.
    """
    ox, oy = o
    dx, dy = d
    best = None
    n = len(pts)
    for i in range(n):
        ax, ay = pts[i]
        bx, by = pts[(i + 1) % n]
        ex, ey = bx - ax, by - ay

        denom = dx * ey - dy * ex
        if abs(denom) < 1e-12:
            continue                       # 광선과 변이 평행

        rx, ry = ax - ox, ay - oy
        t = (rx * ey - ry * ex) / denom    # 광선 위 거리
        s = (rx * dy - ry * dx) / denom    # 변 위 위치 (0~1 이어야 선분 안)

        if t >= 0.0 and 0.0 <= s <= 1.0:
            if best is None or t < best:
                best = t
    return best


def _point_in_polygon(p, pts):
    """레이 캐스팅. 점 p 가 폴리곤 안인가."""
    px, py = p
    inside = False
    n = len(pts)
    for i in range(n):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % n]
        if (y1 > py) != (y2 > py):
            xint = x1 + (py - y1) * (x2 - x1) / (y2 - y1)
            if px < xint:
                inside = not inside
    return inside


def cast_rays(yaw_deg, pts_east_north, ray_min_deg, ray_max_deg, inc_deg, max_range_m):
    """보트(원점)에서 상대방위 격자 [ray_min, ray_max] (inc 간격) 로 경계까지 거리 목록.

    yaw_deg        : /imu/yaw — 나침반(0=북, CW+).
    pts_east_north : 폴리곤 꼭짓점, 보트 기준 로컬 미터 (east, north).
    반환           : [r0, r1, ...] — 인덱스 k 는 rel = ray_min + k·inc. 안 닿거나 max_range 보다 멀면 inf.
                     격자가 잘못됐으면(inc ≤ 0, max < min) 빈 리스트 — 노드는 이를 '없음'으로 다룬다.

    🚨 rel 은 **좌 양수**(인덱스 0 = 우현 끝). 절대 방위 = yaw − rel. 모듈 docstring 참조.
    """
    if inc_deg <= 0.0 or ray_max_deg < ray_min_deg:
        return []
    n = int(round((ray_max_deg - ray_min_deg) / inc_deg)) + 1
    ranges = []
    for k in range(n):
        rel = ray_min_deg + k * inc_deg
        # 🚨 `-` 가 맞다 (계약서_geofence부호_20261104). 좌 양수 rel → 나침반 CW+ 절대 방위.
        th = math.radians((yaw_deg - rel) % 360.0)
        d = (math.sin(th), math.cos(th))                 # (east, north) 단위벡터
        t = _ray_polygon_dist((0.0, 0.0), d, pts_east_north)
        ranges.append(float(t) if (t is not None and t <= max_range_m) else float('inf'))
    return ranges
