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

폴리곤 파라미터 검증 `sanitize_geofence_polygon` (계약서_지오펜스폴리곤타입_20261109 C1):
  노드는 geofence_polygon 을 dynamic_typing 으로 선언한다(Humble 은 기본값 [] 을 BYTE_ARRAY 로 정해
  소수 폴리곤을 넣으면 부팅 때 죽었다 — 11-09 S0 실측). 그래서 어떤 타입이든 노드까지 오고,
  여기서 판정한다. 틀리면 ([], False, 이유) → 노드가 ERROR 1회 + geofence 꺼짐(노드는 계속).
"""

import array
import math

from north_goal_angle.waypoint_loader import LAT_MAX, LAT_MIN, LON_MAX, LON_MIN

# 폴리곤 최소 원소 수 = 3점 × (위도, 경도)
GEOFENCE_MIN_VALUES = 6


def _in_lat(v):
    return LAT_MIN <= v <= LAT_MAX


def _in_lon(v):
    return LON_MIN <= v <= LON_MAX


def sanitize_geofence_polygon(raw):
    """geofence_polygon 파라미터 값 → (새 float 목록, ok, 이유).

    · None(yaml `[]` = NOT_SET) · 길이 0 → ([], True, '')   폴리곤 미설정, 꺼짐(ERROR 없음)
    · 정상 [위도1, 경도1, 위도2, 경도2, ...] (3점 이상, 원소마다 소수)
                                         → (새 float 목록, True, '')
    · 그 밖 → ([], False, 이유)  이유에 원소 위치(0부터)를 담는다. 거부 조건:
        값 전체가 문자열·bytes, 숫자 하나, 목록이 아님 /
        원소가 bool·str·bytes·None·정수(결정 2 — `35` 는 1° 격자 약 100 km 폴리곤이 된다)·NaN·inf /
        홀수 개 / 비어 있지 않은데 6개(3점) 미만(결정 3) /
        위도 LAT_MIN~LAT_MAX · 경도 LON_MIN~LON_MAX 밖(위경도 뒤바뀜 포함, waypoint_loader 와 같은 범위)
    list·tuple·array.array 를 받는다(Humble 실측은 list).
    """
    if raw is None:
        return [], True, ''
    if isinstance(raw, (str, bytes, bytearray)):
        return [], False, (f"값 전체가 문자열이다({raw!r}) — "
                           f"[위도, 경도, ...] 소수 목록으로 적어라")
    if isinstance(raw, (bool, int, float)):
        return [], False, (f"값이 숫자 하나다({raw!r}) — "
                           f"[위도1, 경도1, 위도2, 경도2, ...] 목록이어야 한다")
    if not isinstance(raw, (list, tuple, array.array)):
        return [], False, f"목록이 아니다({type(raw).__name__})"
    if len(raw) == 0:
        return [], True, ''

    out = []
    for i, v in enumerate(raw):
        where = f"원소 {i}({v!r})"
        if isinstance(v, bool):
            return [], False, f"{where} 가 참/거짓이다 — 소수 좌표여야 한다"
        if v is None or isinstance(v, (str, bytes, bytearray)):
            return [], False, f"{where} 가 숫자가 아니다 — 소수 좌표여야 한다"
        if isinstance(v, int):
            return [], False, (f"{where} 가 정수다 — 원소마다 소수로 적어라"
                               f"(예 35 → 35.0, 좌표는 소수 넷째 자리 이상)")
        if not isinstance(v, float):
            return [], False, f"{where} 가 숫자가 아니다({type(v).__name__})"
        if not math.isfinite(v):
            return [], False, f"{where} 가 NaN/inf 다"
        out.append(float(v))

    n = len(out)
    if n % 2 != 0:
        return [], False, (f"원소 {n}개 — 홀수다. [위도, 경도] 짝이 안 맞는다"
                           f"(마지막 원소 {n - 1} 확인)")
    if n < GEOFENCE_MIN_VALUES:
        return [], False, (f"원소 {n}개({n // 2}점) — 폴리곤은 최소 3점"
                           f"({GEOFENCE_MIN_VALUES}개)이어야 한다")

    for k in range(0, n, 2):
        lat, lon = out[k], out[k + 1]
        where = f"{k // 2}번째 점(원소 {k}·{k + 1}) [{lat!r}, {lon!r}]"
        if not (_in_lat(lat) and _in_lon(lon)):
            if _in_lat(lon) and _in_lon(lat):
                return [], False, f"{where}: 위도·경도 순서가 뒤바뀌었다 — [위도, 경도] 순서로 적어라"
            if not _in_lat(lat):
                return [], False, (f"{where}: 위도 {lat!r} 가 범위({LAT_MIN}~{LAT_MAX}) 밖이다"
                                   f"(오타·위경도 뒤바뀜 확인)")
            return [], False, f"{where}: 경도 {lon!r} 가 범위({LON_MIN}~{LON_MAX}) 밖이다"
    return out, True, ''


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


# ───────────── 장애물 구간 경계 (geofence_avoid_polygon) — 규정 종합임무 (4) ─────────────
# 「장애물 구간의 가상경계선을 확연하게 벗어나 경기를 진행하는 경우 … 실격」.
# 경기장 경계(geofence_polygon)와 따로, 회피 WP 에서만 켜는 두 번째 벽이다.
# 🚨 mode 5 만으로 켜지 않는다 — mode 5 는 도킹 접근점·연습 접근점 같은 '이동'에도 쓴다.
#    켜는 WP = mode 5 이고 **WP 좌표가 장애물 구간 폴리곤 안**. 구역 밖 mode 5 WP 에서는 꺼진다.
AVOID_FENCE_MODE = 5
M_PER_DEG_LAT = 110540.0          # north_goal_angle.py 와 같은 값
M_PER_DEG_LON_EQ = 111320.0


def polygon_pairs(poly):
    """[lat1, lon1, lat2, lon2, ...] → [(lat, lon), ...]."""
    return [(poly[i], poly[i + 1]) for i in range(0, len(poly) - 1, 2)]


def local_xy(lat0, lon0, lat, lon):
    """(lat0, lon0) 기준 로컬 미터 (east, north). 노드 _local_xy 와 같은 평면 근사."""
    kx = math.cos(math.radians(lat0)) * M_PER_DEG_LON_EQ
    return ((lon - lon0) * kx, (lat - lat0) * M_PER_DEG_LAT)


def _seg_dist(p, a, b):
    px, py = p
    ax, ay = a
    bx, by = b
    ex, ey = bx - ax, by - ay
    L2 = ex * ex + ey * ey
    s = 0.0 if L2 <= 0.0 else max(0.0, min(1.0, ((px - ax) * ex + (py - ay) * ey) / L2))
    return math.hypot(px - (ax + s * ex), py - (ay + s * ey))


def edge_dist_m(lat, lon, poly):
    """점(lat, lon)에서 폴리곤 변까지 최소 거리(m). 폴리곤이 3점 미만이면 None."""
    if len(poly) < GEOFENCE_MIN_VALUES:
        return None
    pts = [local_xy(lat, lon, la, lo) for la, lo in polygon_pairs(poly)]
    n = len(pts)
    return min(_seg_dist((0.0, 0.0), pts[i], pts[(i + 1) % n]) for i in range(n))


def latlon_in_polygon(lat, lon, poly):
    """점(lat, lon)이 폴리곤 안인가. 로컬 미터로 바꿔 판정(경기장 크기에선 평면 근사로 충분)."""
    if len(poly) < GEOFENCE_MIN_VALUES:
        return False
    pts = [local_xy(lat, lon, la, lo) for la, lo in polygon_pairs(poly)]
    return _point_in_polygon((0.0, 0.0), pts)


def avoid_fence_wps(waypoints, avoid_poly, mode=AVOID_FENCE_MODE):
    """WP 마다 장애물 구간 경계를 켜는가 — mode 5 이고 WP 좌표가 폴리곤 안. waypoints = [(lat, lon, mode, ...)]."""
    if len(avoid_poly) < GEOFENCE_MIN_VALUES:
        return [False] * len(waypoints)
    return [int(w[2]) == mode and latlon_in_polygon(w[0], w[1], avoid_poly) for w in waypoints]


def merge_min(a, b):
    """두 광선 목록을 방향마다 가까운 쪽으로. 길이가 다르면 짧은 쪽까지."""
    return [min(x, y) for x, y in zip(a, b)]
