"""north_goal_angle — 웨이포인트 FSM + 경계 이탈 방지(geofence). 6a.

고친 것:
  [3] 🚨 mode-7 폴백 제거 (ship_dock 수정과 **원자적 한 쌍**)
      작년: if wp_mode == 7: pub_candidate.publish(20000)  ← 0.5초마다
      이건 '도킹이 안 되는 걸 알고 덮어둔 흔적'이다. ship_dock 이 9 로 잡혀 침묵하니,
      배가 멈추지 않게 north 가 대신 폴백을 냈다.
      ship_dock 을 7 로 고치면 → 같은 /candidate_angle 에 발행자가 둘이 된다.
      도킹 중 조향이 GPS 방위로 튄다 (접안 직전 60초에 35° 스파이크 20번).
      → 폴백은 **담당 노드가 없는 모드에만** 남긴다: mode 5(회피), 8(토너먼트 회피).

  [6] Geofence 발행 — 경계를 **'가짜 LiDAR'** 로 낸다. 경기장 밖으로 나가면 실격인데 방어가 0 이었다.

      /geofence_state (Float32MultiArray)
        = [angle_min_deg, angle_inc_deg, r0, r1, r2, ...]   (상대방위 기준, 멀면 inf)
        정보 없으면 **빈 배열**.

      ★ 왜 '광선'인가 (원뿔 방식을 버린 이유):
        **벽은 '점'이 아니라 '선'이다.** 최근접점 한 방향으로 ±각도 원뿔을 막는 건 기하학적으로
        틀렸다. 40×40 경기장 (40,40) 모서리에서 배가 대각선(45°)을 향하면:
            북벽 최근접점 → 상대 -45°   /   동벽 최근접점 → 상대 +45°
            그런데 **탈출구(모서리)는 상대 0° — 배 정면이다.**
        half_block=40° 면 차단 구역이 [-85,-5] 와 [5,85] → **정면 0° 가 정확히 '틈'으로 열린다.**
        배가 대각선으로 탈출한다. 실격. (경기장이 사각형이 아니면 또 뚫린다.)
        광선으로 쏘면 정면 방향 경계 거리(2.12m)가 그대로 잡혀 벽이 된다. **튜닝 파라미터가 없다.**

      ★ 소비자(ship_direction)는 **스캔 병합 한 줄**이면 끝이다:
            ranges[i] = min(real_ranges[i], geofence_ranges[i])
        기존 dilate 가 배 폭 여유를 자동으로 더하고, 기존 갭-팔로잉이 알아서 피한다.
        특수 로직도 새 상태기계도 없다. "경계선을 그냥 벽으로 취급한다"를 끝까지 밀어붙인 것.

      계산량: 161방향 × 변 개수 @ 2Hz — 무시할 수준.

프레임 규약: 광선 각도는 **상대 방위**(0=정면)다 — /candidate_angle 과 같은 규약.
  ⚠️ 광선 방향이 /imu/yaw 에 통째로 의존한다 (CLAUDE.md 3-5).
     N1 에서 /imu/yaw 는 ssf_heading/yaw_mux 단독 발행으로 바뀌었다 —
     부팅 0점화와 GPS COG override 를 걷어낸 절대방위다.
     🚨 단, mount_offset_deg / invert_yaw 를 **벤치에서 아직 안 쟀다.**
        그 전까지 이 geofence 는 방향이 맞는다는 보장이 없다.
     IMU 가 묵으면 **빈 배열**을 낸다. 틀린 방향으로 '없는 벽'을 세우느니 안 내는 게 낫다.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from std_msgs.msg import Float32, Float32MultiArray, Int32, Float64
from sensor_msgs.msg import NavSatFix
from geopy import distance

# ⚠️ 실측 필요: 대회 코스 좌표. 작년 값이라 올해 다시 측정해야 한다.
waypoints = [
    [35.1862375, 128.5655118, 0, 3.0],      # WP0 게이트 시작
    [35.1863642, 128.5657123, 1, 3.0],      # WP1 게이트 끝
    [35.1868822, 128.5660465, 2, 3.0],      # WP2 위치유지
    [35.1868638, 128.56582129999, 3, 3.0],  # WP3 초록
    [35.1867763, 128.5658085, 3, 3.0],      # WP4 빨강
    [35.1868645, 128.5656684, 3, 3.0],      # WP5 하양
    [35.1866601, 128.5655597, 5, 50.0],     # WP6 회피 시작
    [35.186396099999, 128.5653297, 5, 50.0],  # WP7 회피끝
    [35.1859269, 128.5655428, 7, 60.0],     # WP8 도킹 시작
    [35.1859269, 128.5655428, 7, 60.0],     # WP9 도킹
    [35.1861956, 128.5660033, 8, 60.0],     # WP10 토너먼트 회피
]

CANDIDATE_INVALID = 20000.0
ARRIVE_RADIUS_M = 3.0
DEFAULT_TIMEOUT = 120.0
TURN_TIMEOUT = 90.0

# 담당 노드가 없는 모드 = 순수 회피 구간. 여기서만 폴백을 낸다. (CLAUDE.md 3-6)
FALLBACK_MODES = (5, 8)

# 위경도 → 로컬 미터 근사
M_PER_DEG_LAT = 110540.0
M_PER_DEG_LON_EQ = 111320.0


class NorthGoalAngle(Node):
    def __init__(self):
        super().__init__('north_goal_angle')
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)

        # ---- 파라미터 (config/north_goal_angle.yaml, CLAUDE.md 1-4) ----
        self.period_sec = float(self.declare_parameter('period_sec', 0.5).value)
        # 경기장 폴리곤: [lat1, lon1, lat2, lon2, ...] 평탄 배열.
        # ⚠️ 실측 필요 — 대회장 경계 좌표를 아직 모른다. 비어 있으면 geofence 는 꺼진다.
        self.geofence_polygon = list(
            self.declare_parameter('geofence_polygon', []).value or [])
        # 경계를 '가짜 LiDAR'로 쏜다 — 스캔과 같은 각도 격자(상대방위).
        # ★ 왜 광선인가: 벽은 '점'이 아니라 '선'이다. 최근접점 한 방향으로 ±각도 원뿔을 막는 건
        #   기하학적으로 틀렸다. 40x40 모서리에서 배가 대각선(45°)을 보면 북벽 최근접점은 -45°,
        #   동벽은 +45° 인데 **탈출구(모서리)는 정면 0°** 라 원뿔 사이 '틈'으로 그대로 빠져나간다.
        #   광선으로 쏘면 정면 방향의 경계 거리(2.12m)가 그대로 잡혀 벽이 된다. 튜닝도 필요 없다.
        self.ray_min_deg = float(self.declare_parameter('geofence_ray_min_deg', -80.0).value)
        self.ray_max_deg = float(self.declare_parameter('geofence_ray_max_deg', 80.0).value)
        self.ray_inc_deg = float(self.declare_parameter('geofence_ray_inc_deg', 1.0).value)
        self.geofence_max_range_m = float(
            self.declare_parameter('geofence_max_range_m', 30.0).value)   # 이보다 멀면 inf
        # 구독자 0 경고를 시작할 시각
        self.geofence_check_after_sec = float(
            self.declare_parameter('geofence_check_after_sec', 5.0).value)
        # 🚨 IMU 신선도. geofence 상대방위는 /imu/yaw 로 계산하므로 IMU 에 의존한다.
        #    IMU 가 죽으면 yaw 가 얼어붙고 → 경계 방위가 엉뚱해지고 → ship_direction 이
        #    '없는 벽'을 칠한다 → 배가 갇힌다.
        #    → 묵으면 geofence 도 [inf, nan] 을 낸다. (1단계 ship_goal_angle 과 같은 원칙:
        #      **모르면 입을 다문다.** 틀린 값을 내느니 안 내는 게 낫다.)
        self.imu_stale_sec = float(self.declare_parameter('imu_stale_sec', 0.5).value)

        # ---- 발행 (토픽 이름 불변 + /geofence_state 신규, CLAUDE.md 3-9) ----
        self.pub_dist = self.create_publisher(Float32, '/goal_distance', qos)
        self.pub_mode = self.create_publisher(Int32, '/wp_mode', qos)
        self.pub_candidate = self.create_publisher(Float32, '/candidate_angle', qos)
        self.pub_bearing = self.create_publisher(Float32, '/north_goal_angle_tp', qos)
        self.pub_remain = self.create_publisher(Float32, '/wp_remaining_time', qos)
        self.pub_geofence = self.create_publisher(Float32MultiArray, '/geofence_state', qos)

        # ---- 구독 ----
        self.create_subscription(NavSatFix, '/ublox_gps_node/fix', self.gps_cb, qos)
        self.create_subscription(Float64, '/imu/yaw', self.yaw_cb, qos)   # 경계 상대방위 계산용

        # ---- 상태 ----
        self.lat, self.lon = 0.0, 0.0
        self.have_fix = False
        self.yaw = None
        self.last_yaw_t = None          # /imu/yaw 마지막 수신 (monotonic)
        self.wp_idx = 0
        self.t_start = None
        self.wp_enter_time = None
        self.t0 = time.monotonic()

        self.create_timer(self.period_sec, self.timer_cb)

        gf = "OFF(폴리곤 미설정 — ⚠️ 실측 필요)" if len(self.geofence_polygon) < 6 else \
             f"ON({len(self.geofence_polygon) // 2}점)"
        self.get_logger().info(
            f"north_goal_angle 시작: 폴백은 mode {FALLBACK_MODES} 에만 발행"
            f"(mode 7 폴백 제거 — ship_dock 과 충돌했다). geofence {gf}."
        )

    # ───────────────────────── 콜백 ─────────────────────────
    def gps_cb(self, msg):
        self.lat, self.lon = msg.latitude, msg.longitude
        self.have_fix = True

    def yaw_cb(self, msg):
        self.yaw = float(msg.data)
        self.last_yaw_t = time.monotonic()

    # ───────────────────────── Geofence ─────────────────────────
    def _local_xy(self, lat, lon):
        """보트 기준 로컬 미터 (east, north)."""
        kx = math.cos(math.radians(self.lat)) * M_PER_DEG_LON_EQ
        return ((lon - self.lon) * kx, (lat - self.lat) * M_PER_DEG_LAT)

    def _geofence_ranges(self):
        """경계를 '가짜 LiDAR'로 쏜다 → (angle_min_deg, angle_inc_deg, [r0, r1, ...]) 또는 None.

        각 상대방위마다 '이 방향으로 몇 m 가면 경계 밖인가'를 구한다. 멀면 inf.
        소비자(ship_direction)는 이걸 실제 스캔과 min() 으로 병합하기만 하면 된다 —
        특수 로직이 아예 없어진다. 기존 dilate 가 배 폭 여유를 더하고, 갭-팔로잉이 알아서 피한다.

        None(=빈 배열) 을 내는 경우 — 전부 '모르면 입을 다문다':
          · 폴리곤 미설정 / GPS fix 없음
          · IMU stale     → yaw 가 얼면 광선 방향이 통째로 틀어져 '없는 벽'을 만든다
          · 이미 경기장 밖 → 경계를 벽으로 세우면 돌아갈 길을 막는다
        """
        poly = self.geofence_polygon
        if len(poly) < 6 or not self.have_fix:      # 최소 3점(=6수)
            return None

        # 🚨 IMU 신선도 — 광선 방향이 yaw 에 통째로 의존한다. 얼면 벽 전체가 엉뚱해진다.
        if self.yaw is None or self.last_yaw_t is None:
            self.get_logger().warn(
                "/imu/yaw 없음 → geofence 침묵(빈 배열)", throttle_duration_sec=5.0)
            return None
        yaw_age = time.monotonic() - self.last_yaw_t
        if yaw_age > self.imu_stale_sec:
            self.get_logger().warn(
                f"/imu/yaw stale ({yaw_age:.2f}s > {self.imu_stale_sec}s) → geofence 침묵. "
                f"틀린 방향으로 '없는 벽'을 세우느니 안 내는 게 낫다.",
                throttle_duration_sec=2.0)
            return None

        pts = [self._local_xy(poly[i], poly[i + 1]) for i in range(0, len(poly) - 1, 2)]

        if not _point_in_polygon((0.0, 0.0), pts):
            # 🚨 이미 밖이다. 경계를 벽으로 세우면 복귀로를 막는다 → 아무것도 내지 않는다.
            self.get_logger().error(
                "🚨 경기장 경계 밖이다 (실격 위험). GPS 웨이포인트로 복귀 중.",
                throttle_duration_sec=2.0)
            return None

        if self.ray_inc_deg <= 0.0 or self.ray_max_deg < self.ray_min_deg:
            return None

        n = int(round((self.ray_max_deg - self.ray_min_deg) / self.ray_inc_deg)) + 1
        ranges = []
        for k in range(n):
            rel = self.ray_min_deg + k * self.ray_inc_deg
            th = math.radians((self.yaw + rel) % 360.0)      # 절대 방위(compass)
            d = (math.sin(th), math.cos(th))                 # (east, north) 단위벡터
            t = _ray_polygon_dist((0.0, 0.0), d, pts)
            ranges.append(float(t) if (t is not None and t <= self.geofence_max_range_m)
                          else float('inf'))

        return self.ray_min_deg, self.ray_inc_deg, ranges

    def _publish_geofence(self):
        msg = Float32MultiArray()
        gs = self._geofence_ranges()
        if gs is None:
            msg.data = []                                    # 빈 배열 = 경계 정보 없음
        else:
            a0, inc, ranges = gs
            msg.data = [float(a0), float(inc)] + [float(r) for r in ranges]
        self.pub_geofence.publish(msg)

        # 🚨 '잊으면 최악' 방어: 구독자가 0 이면 경계 방어가 통째로 없는 것이다.
        #    (도킹이 1년간 침묵했던 것과 똑같은 사고 — 침묵하는 기능은 에러를 내지 않는다)
        if (time.monotonic() - self.t0) > self.geofence_check_after_sec:
            if self.pub_geofence.get_subscription_count() == 0:
                self.get_logger().error(
                    "🚨 /geofence_state 구독자가 0 이다 — 경계 이탈 방어가 작동하지 않는다(실격 위험). "
                    "6a-2 에서 ship_direction 이 구독해야 한다.",
                    throttle_duration_sec=10.0)

    # ───────────────────────── 메인 루프 ─────────────────────────
    def timer_cb(self):
        self._publish_geofence()             # 웨이포인트가 끝나도 경계 감시는 계속한다

        if self.wp_idx >= len(waypoints):
            return

        wp_lat, wp_lon, wp_mode, dwell = waypoints[self.wp_idx]

        dist = calc_dist(self.lat, self.lon, wp_lat, wp_lon)
        self.pub_dist.publish(Float32(data=dist))

        bearing = calc_angle(self.lat, self.lon, wp_lat, wp_lon)
        self.pub_bearing.publish(Float32(data=bearing))

        self.pub_mode.publish(Int32(data=wp_mode))

        # [3] 폴백은 '담당 노드가 없는 모드'(5, 8)에만.
        #     작년엔 mode 7(도킹)에도 냈다 → ship_dock 을 7 로 고치면 발행자가 둘이 되어
        #     도킹 중 조향이 GPS 방위로 튄다. 원자적 한 쌍이라 같은 커밋에서 고친다.
        if wp_mode in FALLBACK_MODES:
            self.pub_candidate.publish(Float32(data=CANDIDATE_INVALID))

        now = time.monotonic()               # 벽시계 금지 (CLAUDE.md 1-5)

        if self.wp_enter_time is None:
            self.wp_enter_time = now

        timeout = None
        if self.wp_idx in (3, 4, 5):
            timeout = TURN_TIMEOUT
        elif self.wp_idx <= 6:
            timeout = DEFAULT_TIMEOUT

        if timeout is not None:
            remain = max(timeout - (now - self.wp_enter_time), 0.0)
            self.pub_remain.publish(Float32(data=remain))

            if remain <= 0:
                self.get_logger().warn(f"🕒 WP{self.wp_idx} 시간 초과 → 다음 WP 이동")
                self.wp_idx += 1
                self.t_start = None
                self.wp_enter_time = None
                return

        if dist < ARRIVE_RADIUS_M:
            if self.t_start is None:
                self.t_start = now
            elif (now - self.t_start) >= dwell:
                self.get_logger().info(f"✔ WP{self.wp_idx} 완료 → 다음 WP 이동")
                self.wp_idx += 1
                self.t_start = None
                self.wp_enter_time = None
        else:
            self.t_start = None


# ───────────────────────── 순수 기하 (테스트 가능) ─────────────────────────
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


def calc_angle(lat1, lon1, lat2, lon2):
    from math import atan2, cos, radians, sin, degrees
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
    dlon = lon2 - lon1
    x = sin(dlon) * cos(lat2)
    y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)
    return (degrees(atan2(x, y)) + 360) % 360


def calc_dist(lat1, lon1, lat2, lon2):
    return distance.distance((lat1, lon1), (lat2, lon2)).m


def main(args=None):
    rclpy.init(args=args)
    node = NorthGoalAngle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
