"""north_goal_angle — 웨이포인트 FSM + 경계 이탈 방지(geofence). 6a.

고친 것:
  [3] 🚨 mode-7 폴백 제거 (ship_dock 수정과 **원자적 한 쌍**)
      작년: if wp_mode == 7: pub_candidate.publish(20000)  ← 0.5초마다
      이건 '도킹이 안 되는 걸 알고 덮어둔 흔적'이다. ship_dock 이 9 로 잡혀 침묵하니,
      배가 멈추지 않게 north 가 대신 폴백을 냈다.
      ship_dock 을 7 로 고치면 → 같은 /candidate_angle 에 발행자가 둘이 된다.
      도킹 중 조향이 GPS 방위로 튄다 (접안 직전 60초에 35° 스파이크 20번).
      → 폴백은 **담당 노드가 없는 모드에만** 남긴다: mode 5(회피), 8(토너먼트 회피).

  [6] Geofence 발행. 경기장 밖으로 나가면 실격인데 방어가 0 이었다.
      /geofence_state (Float32MultiArray) = [경계까지 거리(m), 경계 상대방위(deg)]
      멀거나 미설정이면 [inf, nan].

      ⚠️ 구독자는 6a-2 에서 ship_direction 이 붙인다. 그때까지는 아무도 안 듣는다.
         '발행했으니 됐다'고 잊으면 도킹이 1년간 침묵한 것과 똑같은 사고가 된다.
         → 부팅 5초 뒤 구독자가 0 이면 **ERROR 를 계속 찍는다.** (healthcheck 도 검사한다)

프레임 규약: 경계 방위는 **상대 방위**(0=정면)로 낸다 — /candidate_angle 과 같은 규약.
  소비자(ship_direction)가 LiDAR 이진 마스크(정면=80°)에 그대로 칠할 수 있게.
  ⚠️ 이 변환은 /imu/yaw 를 쓰므로, 4단계의 'IMU 절대방위' 수정에 의존한다
     (지금 yaw 는 부팅 0점화라 상대각 — CLAUDE.md 3-5. 4단계에서 고쳐진다).
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
        # 이보다 멀면 [inf, nan] 을 낸다 (가까울 때만 의미가 있다)
        self.geofence_report_dist_m = float(
            self.declare_parameter('geofence_report_dist_m', 10.0).value)
        # 구독자 0 경고를 시작할 시각
        self.geofence_check_after_sec = float(
            self.declare_parameter('geofence_check_after_sec', 5.0).value)

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

    # ───────────────────────── Geofence ─────────────────────────
    def _local_xy(self, lat, lon):
        """보트 기준 로컬 미터 (east, north)."""
        kx = math.cos(math.radians(self.lat)) * M_PER_DEG_LON_EQ
        return ((lon - self.lon) * kx, (lat - self.lat) * M_PER_DEG_LAT)

    def _geofence_state(self):
        """→ (거리 m, 상대방위 deg) 또는 None(미설정/멀다/이탈)."""
        poly = self.geofence_polygon
        if len(poly) < 6 or not self.have_fix:      # 최소 3점(=6수)
            return None

        pts = [self._local_xy(poly[i], poly[i + 1]) for i in range(0, len(poly) - 1, 2)]

        best_d, best_p = float('inf'), None
        n = len(pts)
        for i in range(n):
            d, p = _point_seg_dist((0.0, 0.0), pts[i], pts[(i + 1) % n])
            if d < best_d:
                best_d, best_p = d, p

        if best_p is None or not math.isfinite(best_d):
            return None

        inside = _point_in_polygon((0.0, 0.0), pts)
        if not inside:
            # 🚨 이미 경기장 밖이다. 이때 경계를 '장애물'로 칠하면 돌아갈 길을 막는다.
            #    → 아무것도 내지 않고(=[inf,nan]) GPS 웨이포인트가 안으로 끌어당기게 둔다.
            #    (6a-2 에서 ship_direction 의 처리를 확정할 것)
            self.get_logger().error(
                "🚨 경기장 경계 밖이다 (실격 위험). GPS 웨이포인트로 복귀 중.",
                throttle_duration_sec=2.0)
            return None

        if best_d > self.geofence_report_dist_m:
            return None                              # 멀다 → [inf, nan]

        if self.yaw is None:
            self.get_logger().warn(
                "/imu/yaw 없음 → 경계 상대방위를 계산할 수 없다", throttle_duration_sec=5.0)
            return None

        east, north = best_p
        compass = (math.degrees(math.atan2(east, north)) + 360.0) % 360.0
        rel = (compass - self.yaw) % 360.0           # 상대방위 (0 = 정면)
        return best_d, rel

    def _publish_geofence(self):
        msg = Float32MultiArray()
        gs = self._geofence_state()
        msg.data = [float('inf'), float('nan')] if gs is None else [float(gs[0]), float(gs[1])]
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
def _point_seg_dist(p, a, b):
    """점 p 에서 선분 ab 까지의 거리와 최근접점. → (거리, 최근접점)"""
    px, py = p
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    seg2 = dx * dx + dy * dy
    if seg2 <= 1e-12:
        return math.hypot(px - ax, py - ay), (ax, ay)
    t = ((px - ax) * dx + (py - ay) * dy) / seg2
    t = max(0.0, min(1.0, t))
    qx, qy = ax + t * dx, ay + t * dy
    return math.hypot(px - qx, py - qy), (qx, qy)


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
