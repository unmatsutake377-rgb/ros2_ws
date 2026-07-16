"""ship_turn — 부표 선회(orbit) 미션. 6c 에서 동작이 통째로 바뀐다.

작년 문제:
  [1] 🚨 '비껴 지나가기'였다. offset 을 줘서 부표 옆을 스쳐 지나갔다. **규정은 '선회(orbit)'** —
      규정 위반이었다. → 부표를 중심으로 원을 그리며 도는 orbit 기동으로 재작성.
      부표를 ±90°에 두고(제어법은 orbit_logic.orbit_steer), LiDAR 거리로 반경을 보정한다.
  [2] 🚨 흰색 부표 인식이 없었다. 규정: **빨강·초록 = 시계 / 흰색 = 반시계.** 작년은 색 매핑도
      엉뚱했다(red→left 등). → /buoy_color 로 색을 받아 회전 방향을 정한다.
  [3] 거리는 LiDAR 에서. /image_distance(카메라 뎁스, OAK-1 에서 죽음) 구독 폐기, /scan 사용.
      부표는 '점 물체'라 방위 매칭이 통한다(도크와 다르다).

파라미터(6c-[4]): orbit_radius_m(⚠️실측), radius_tol_m, approach_dist_m, k_radial_deg_per_m,
                  orbit_target_deg(360). SPIN 규약(6c-[5]): 5000=우선회, 6000=좌선회.

FSM: WAIT → SEARCH → APPROACH → ORBIT → COOLDOWN → (다음 부표 위해 SEARCH)
  WP3,4,5(부표 3개)는 모두 /wp_mode==3 으로 온다. 한 부표를 돌고 COOLDOWN 동안 손 떼면
  autonomous(GPS)가 다음 부표로 데려가고, 다시 SEARCH 로 잡는다.

프레임 규약: /image_angle=상대각(0=정면). orbit 조향각을 상대로 구해 rel_to_raw_0_360 으로
  /candidate_angle(0~360) 규약에 맞춰 발행 → ship_direction (B)에서 매핑. LiDAR 80°=정면.

⚠️ /buoy_color 는 5단계 비전이 발행해야 하는 신규 계약(String: "red"/"green"/"white").
   못 받으면 회전 방향을 모른다 → 그동안 SEARCH 에 머문다(틀린 방향으로 돌지 않는다).
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, Int32, String, Float64
from sensor_msgs.msg import LaserScan

from ship_turn.orbit_logic import (
    orbit_steer, OrbitProgress, orbit_direction_cw, clamp_pm180, wrap360)

IMAGE_ANGLE_INVALID = 10000.0
CANDIDATE_INVALID = 20000.0
SPIN_RIGHT = 5000.0          # 우선회 (CLAUDE.md 3-9)
SPIN_LEFT = 6000.0           # 좌선회
LIDAR_FORWARD_DEG = 80.0     # LiDAR 프레임 정면


class ShipTurn(Node):
    def __init__(self):
        super().__init__('ship_turn')
        self.lock = threading.Lock()
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)

        # ---- 파라미터 (config/ship_turn.yaml, CLAUDE.md 1-4) ----
        self.active_wp_mode = int(self.declare_parameter('active_wp_mode', 3).value)
        self.period_sec = float(self.declare_parameter('period_sec', 0.1).value)

        self.orbit_radius_m = float(self.declare_parameter('orbit_radius_m', 3.0).value)      # ⚠️ 실측
        self.radius_tol_m = float(self.declare_parameter('radius_tol_m', 0.5).value)
        self.approach_dist_m = float(self.declare_parameter('approach_dist_m', 4.0).value)    # ⚠️ 실측
        self.k_radial = float(self.declare_parameter('k_radial_deg_per_m', 30.0).value)
        self.radial_limit_deg = float(self.declare_parameter('radial_limit_deg', 45.0).value)
        self.orbit_target_deg = float(self.declare_parameter('orbit_target_deg', 360.0).value)

        self.hold_sec = float(self.declare_parameter('hold_sec', 2.0).value)
        self.search_timeout_sec = float(self.declare_parameter('search_timeout_sec', 20.0).value)
        self.orbit_max_sec = float(self.declare_parameter('orbit_max_sec', 60.0).value)       # 안전(진행각 못 잴 때)
        self.cooldown_sec = float(self.declare_parameter('cooldown_sec', 5.0).value)
        self.lost_sec = float(self.declare_parameter('lost_sec', 2.0).value)                  # 부표 놓침 허용

        self.angle_stale_sec = float(self.declare_parameter('angle_stale_sec', 1.0).value)
        self.color_stale_sec = float(self.declare_parameter('color_stale_sec', 2.0).value)
        self.scan_stale_sec = float(self.declare_parameter('scan_stale_sec', 1.0).value)
        self.lidar_match_half_deg = float(self.declare_parameter('lidar_match_half_deg', 3.0).value)

        # ---- 발행/구독 (토픽 이름 불변, /image_distance 는 버림) ----
        self.pub_candidate = self.create_publisher(Float32, '/candidate_angle', qos)
        self.create_subscription(Int32, '/wp_mode', self.wp_cb, qos)
        self.create_subscription(Float32, '/image_angle', self.angle_cb, qos)
        self.create_subscription(String, '/buoy_color', self.color_cb, qos)   # [2] 신규 계약
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Float64, '/imu/yaw', self.yaw_cb, qos)       # orbit 진행각용

        # ---- 상태 ----
        self.wp_mode = -1
        self.buoy_rel = None
        self.t_angle = None
        self.color = ""
        self.t_color = None
        self.latest_scan = None
        self.t_scan = None
        self.yaw = None
        self.t_yaw = None

        self.state = "INIT"
        self.state_t = None
        self.dir_cw = None
        self.progress = OrbitProgress()
        self.last_seen_t = None

        self.create_timer(self.period_sec, self.timer_cb)
        self.get_logger().info(
            f"ship_turn 시작: orbit r={self.orbit_radius_m}m(⚠️실측), 목표 {self.orbit_target_deg}°.\n"
            f"   빨강·초록=시계 / 흰색=반시계. 거리는 LiDAR. /buoy_color 없으면 SEARCH 유지."
        )

    # ---------------------- 콜백 ----------------------
    def wp_cb(self, msg):
        with self.lock:
            prev = self.wp_mode
            self.wp_mode = int(msg.data)
            if prev != self.active_wp_mode and self.wp_mode == self.active_wp_mode:
                self._reset("INIT")

    def angle_cb(self, msg):
        with self.lock:
            v = float(msg.data)
            if abs(v - IMAGE_ANGLE_INVALID) < 1e-3 or not math.isfinite(v):
                self.buoy_rel = None
            else:
                self.buoy_rel = clamp_pm180(v)
                self.t_angle = time.monotonic()

    def color_cb(self, msg):
        with self.lock:
            self.color = msg.data.strip().lower()
            self.t_color = time.monotonic()

    def scan_cb(self, msg):
        with self.lock:
            self.latest_scan = msg
            self.t_scan = time.monotonic()

    def yaw_cb(self, msg):
        with self.lock:
            self.yaw = float(msg.data)
            self.t_yaw = time.monotonic()

    # ---------------------- 보조 ----------------------
    def _reset(self, state):
        self.state = state
        self.state_t = time.monotonic()
        self.dir_cw = None
        self.progress.reset()
        self.last_seen_t = None

    def set_state(self, s):
        if s != self.state:
            self.get_logger().info(f"🔄 선회 {self.state} → {s}")
        self.state = s
        self.state_t = time.monotonic()

    def elapsed(self):
        return 0.0 if self.state_t is None else time.monotonic() - self.state_t

    def _buoy(self, now):
        """(상대각, 색) 신선하면 반환, 아니면 (None, 색or None)."""
        rel = None
        if self.buoy_rel is not None and self.t_angle is not None \
                and (now - self.t_angle) <= self.angle_stale_sec:
            rel = self.buoy_rel
        col = None
        if self.t_color is not None and (now - self.t_color) <= self.color_stale_sec:
            col = self.color
        return rel, col

    def _lidar_dist_at(self, rel_deg, now):
        """부표 상대방위의 LiDAR 최소거리(±match_half). 모르면 None."""
        scan = self.latest_scan
        if scan is None or self.t_scan is None or (now - self.t_scan) > self.scan_stale_sec:
            return None
        inc = math.degrees(scan.angle_increment)
        if not math.isfinite(inc) or inc <= 0.0:
            return None
        a_min = math.degrees(scan.angle_min)
        target = LIDAR_FORWARD_DEG + rel_deg
        lo = max(0, int((target - self.lidar_match_half_deg - a_min) / inc))
        hi = min(len(scan.ranges) - 1, int((target + self.lidar_match_half_deg - a_min) / inc))
        if lo > hi:
            return None
        best = float('inf')
        for i in range(lo, hi + 1):
            r = scan.ranges[i]
            if math.isfinite(r) and r > 0.0 and r < best:
                best = r
        return best if math.isfinite(best) else None

    def publish(self, value):
        self.pub_candidate.publish(Float32(data=float(value)))

    # ---------------------- FSM ----------------------
    def timer_cb(self):
        with self.lock:
            if self.wp_mode != self.active_wp_mode:
                return
            now = time.monotonic()
            if self.state_t is None:
                self.state_t = now
            rel, col = self._buoy(now)
            yaw = self.yaw if (self.t_yaw and (now - self.t_yaw) <= self.angle_stale_sec) else None

        dist = self._lidar_dist_at(rel, now) if rel is not None else None
        if rel is not None:
            self.last_seen_t = now
        lost = self.last_seen_t is None or (now - self.last_seen_t) > self.lost_sec

        # ---- INIT: 진입 직후 잠깐 정지(autonomous 가 부표로 접근) ----
        if self.state == "INIT":
            self.publish(CANDIDATE_INVALID)
            if self.elapsed() >= self.hold_sec:
                self.set_state("SEARCH")
            return

        # ---- SEARCH: 부표+색이 잡힐 때까지 제자리 선회 ----
        if self.state == "SEARCH":
            direction = orbit_direction_cw(col) if col else None
            if rel is not None and direction is not None:
                self.dir_cw = direction
                self.progress.reset()
                self.set_state("APPROACH")
                return
            if self.elapsed() > self.search_timeout_sec:
                self.get_logger().warn("부표/색 못 찾음(SEARCH 타임아웃) → 다음으로")
                self.set_state("COOLDOWN")
                return
            # 색을 알면 그 방향으로, 모르면 우선회로 탐색
            spin = SPIN_LEFT if (col and orbit_direction_cw(col) is False) else SPIN_RIGHT
            self.publish(spin)
            return

        # ---- APPROACH: 부표를 향해 접근, orbit 반경 부근에서 ORBIT ----
        if self.state == "APPROACH":
            if lost:
                self.set_state("SEARCH")
                return
            if dist is not None and dist <= self.approach_dist_m:
                self.progress.reset()
                self.set_state("ORBIT")
                return
            self.publish(rel_to_raw(rel))              # 부표 방향으로 직진(거리 미상이어도)
            return

        # ---- ORBIT: 반경 유지하며 한 바퀴 ----
        if self.state == "ORBIT":
            if lost:
                self.set_state("SEARCH")
                return
            if self.elapsed() > self.orbit_max_sec:
                self.get_logger().warn("orbit 안전 타임아웃 → 완료 처리")
                self.set_state("COOLDOWN")
                return

            d = dist if dist is not None else self.orbit_radius_m   # 거리 미상이면 접선만
            steer = orbit_steer(rel, d, self.orbit_radius_m, self.dir_cw,
                                self.k_radial, self.radial_limit_deg)

            # 진행각 누적 (yaw 있을 때만). 목표 도달 → 완료.
            if yaw is not None:
                prog = self.progress.update(wrap360(yaw + rel))
                if prog >= self.orbit_target_deg:
                    self.get_logger().info(f"✅ 선회 {prog:.0f}° 완료")
                    self.set_state("COOLDOWN")
                    return

            self.publish(rel_to_raw(steer))
            return

        # ---- COOLDOWN: 손 떼고(autonomous 로) 다음 부표로 이동 ----
        if self.state == "COOLDOWN":
            self.publish(CANDIDATE_INVALID)
            if self.elapsed() >= self.cooldown_sec:
                self.set_state("SEARCH")               # 다음 부표 탐색
            return

        # 알 수 없는 상태 → 안전
        self.publish(CANDIDATE_INVALID)


def rel_to_raw(a):
    return (a + 360.0) % 360.0


def main(args=None):
    rclpy.init(args=args)
    node = ShipTurn()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
