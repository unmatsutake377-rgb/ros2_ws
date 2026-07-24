"""ship_back — 부표 위치유지(station-keeping) 미션. 6d 에서 재작성.

작년 문제:
  [1] 🚨 하는 일이 '그냥 5초간 PWM 중립'이었다. 조류 0.5m/s 면 5초에 2.5m 밀린다.
      규정은 '부표 5m 이내 5초 유지' → 밀려나서 실패한다.
      → 부표까지 거리를 LiDAR 로 재면서 전진/후진으로 능동 유지. hold_dist_m(2.5) 를 목표로,
        데드밴드(dist_tol_m)를 벗어나면 되돌린다. (제어는 hold_logic.station_keep_action)
  [2] 🚨 자기를 'ship_turn' 으로 등록했다(복붙 실수): super().__init__('ship_turn').
      클래스명도 ShipTurn 이었다. 노드 두 개가 같은 이름으로 떠서, 올해 yaml 파라미터를
      노드 이름으로 찾을 때 ship_back 이 ship_turn 의 값을 받아버린다. → super().__init__('ship_back').
  [3] 거리는 LiDAR 에서. /image_distance(카메라 뎁스, OAK-1 에서 죽음) 폐기, /scan 사용.
      부표는 점 물체라 방위 매칭이 통한다. 부표 방위는 /image_angle, 없으면 LiDAR 전방 최근접.

파라미터(6d-[4]): active_wp_mode(2), keep_radius_m(5.0 규정), hold_time_sec(5.0 규정),
                  hold_dist_m(2.5 ⚠️실측), dist_tol_m.

FSM: INIT → SEARCH → HOLD → DONE.
  keep_radius(5m) 이내에 hold_time(5s) '연속' 머물면 완료(규정 만족) → autonomous 로 다음 WP.

프레임 규약: /image_angle=상대각(0=정면). 전진은 부표 방위로 조향, 후진은 상대 180°.
  /candidate_angle 은 0~360(0=정면) 규약. LiDAR 80°=정면.

⚠️ ship_last 는 6b 에서 이미 제거됨(6d-[3]). 여기선 ship_back 만 다룬다.
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import LaserScan

from ship_back.hold_logic import station_keep_action, HoldTimer

IMAGE_ANGLE_INVALID = 10000.0
CANDIDATE_INVALID = 20000.0
STOP_HOLD = 50000.0
REVERSE_REL = 180.0          # 상대 180° = 후진 (→ ship_direction 260 → motor_control 분기 5)
LIDAR_FORWARD_DEG = 80.0     # LiDAR 프레임 정면



# QoS-B: /scan 표준 sensor-data QoS. 작년은 depth=10 기본 RELIABLE 이었다.
#   [1] 묵은 큐: LiDAR 10Hz × depth 10 = **1초치**가 쌓인다. 콜백이 한 번 밀리면
#       그 뒤로 묵은 스캔이 burst 로 몰려와 '1초 전 장면' 으로 조향한다.
#   [2] 워치독 왜곡: 스테일 판정이 '콜백 도착 시각' 기준이라, burst 가 워치독을 먹여
#       실제로는 늦은 데이터인데 신선하다고 착각시킨다. depth=1 은 도착=신선을 일치시킨다.
#   [3] 호환성: 구독자 BEST_EFFORT 는 발행자가 RELIABLE 이든 BEST_EFFORT 든 **전부 호환**된다
#       (그 반대가 비호환). 현행 rplidar_ros 2.1.4 는 rplidar_node.cpp:440 에서
#       rclcpp::QoS(KeepLast(10)) = RELIABLE 로 발행한다 — 소스 확인함. 드라이버를 갈아도 안 깨진다.
#   드롭이 생겨도 '콜백 부재 → 스테일 → 페일세이프 발동' 으로 안전한 방향으로 실패한다.
SCAN_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

class ShipBack(Node):     # 🚨 작년엔 ShipTurn 이었다(복붙). 이름을 파일·패키지와 맞춘다.
    def __init__(self):
        super().__init__('ship_back')     # 🚨 작년엔 'ship_turn' 이었다
        self.lock = threading.Lock()
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)

        # ---- 파라미터 (config/ship_back.yaml, CLAUDE.md 1-4) ----
        self.active_wp_mode = int(self.declare_parameter('active_wp_mode', 2).value)
        self.period_sec = float(self.declare_parameter('period_sec', 0.1).value)

        self.keep_radius_m = float(self.declare_parameter('keep_radius_m', 5.0).value)   # 규정
        self.hold_time_sec = float(self.declare_parameter('hold_time_sec', 5.0).value)   # 규정
        self.hold_dist_m = float(self.declare_parameter('hold_dist_m', 2.5).value)       # ⚠️ 실측
        self.dist_tol_m = float(self.declare_parameter('dist_tol_m', 0.4).value)

        self.search_timeout_sec = float(self.declare_parameter('search_timeout_sec', 20.0).value)
        self.lost_sec = float(self.declare_parameter('lost_sec', 2.0).value)

        self.angle_stale_sec = float(self.declare_parameter('angle_stale_sec', 1.0).value)
        self.scan_stale_sec = float(self.declare_parameter('scan_stale_sec', 1.0).value)
        self.lidar_match_half_deg = float(self.declare_parameter('lidar_match_half_deg', 3.0).value)
        # 부표 방위를 못 받을 때 LiDAR 전방 섹터에서 최근접을 부표로 본다
        self.front_sector_half_deg = float(self.declare_parameter('front_sector_half_deg', 30.0).value)

        # ---- 발행/구독 (토픽 이름 불변, /image_distance 는 버림) ----
        self.pub_candidate = self.create_publisher(Float32, '/candidate_angle', qos)
        self.create_subscription(Int32, '/wp_mode', self.wp_cb, qos)
        self.create_subscription(Float32, '/image_angle', self.angle_cb, qos)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, SCAN_QOS)

        # ---- 상태 ----
        self.wp_mode = -1
        self.buoy_rel = None
        self.t_angle = None
        self.latest_scan = None
        self.t_scan = None

        self.state = "INIT"
        self.state_t = None
        self.last_seen_t = None
        self.hold_timer = HoldTimer(self.keep_radius_m, self.hold_time_sec)

        self.create_timer(self.period_sec, self.timer_cb)
        self.get_logger().info(
            f"ship_back 시작(이름 수정: 작년 'ship_turn' → 'ship_back'). "
            f"위치유지: hold_dist={self.hold_dist_m}m(⚠️실측)/tol {self.dist_tol_m}, "
            f"규정 {self.keep_radius_m}m 이내 {self.hold_time_sec}s."
        )

    # ---------------------- 콜백 ----------------------
    def wp_cb(self, msg):
        with self.lock:
            prev = self.wp_mode
            self.wp_mode = int(msg.data)
            if prev != self.active_wp_mode and self.wp_mode == self.active_wp_mode:
                self.state = "INIT"
                self.state_t = time.monotonic()
                self.last_seen_t = None
                self.hold_timer.reset()

    def angle_cb(self, msg):
        with self.lock:
            v = float(msg.data)
            if abs(v - IMAGE_ANGLE_INVALID) < 1e-3 or not math.isfinite(v):
                self.buoy_rel = None
            else:
                self.buoy_rel = clamp_pm180(v)
                self.t_angle = time.monotonic()

    def scan_cb(self, msg):
        with self.lock:
            self.latest_scan = msg
            self.t_scan = time.monotonic()

    # ---------------------- 보조 ----------------------
    def set_state(self, s):
        if s != self.state:
            self.get_logger().info(f"⚓ 위치유지 {self.state} → {s}")
        self.state = s
        self.state_t = time.monotonic()

    def elapsed(self):
        return 0.0 if self.state_t is None else time.monotonic() - self.state_t

    def _scan_fresh(self, now):
        s = self.latest_scan
        if s is None or self.t_scan is None or (now - self.t_scan) > self.scan_stale_sec:
            return None
        inc = math.degrees(s.angle_increment)
        if not math.isfinite(inc) or inc <= 0.0:
            return None
        return s, inc, math.degrees(s.angle_min)

    def _dist_at(self, rel_deg, now):
        """부표 상대방위 ±match_half 의 LiDAR 최소거리. 모르면 None."""
        info = self._scan_fresh(now)
        if info is None:
            return None
        s, inc, a_min = info
        target = LIDAR_FORWARD_DEG + rel_deg
        lo = max(0, int((target - self.lidar_match_half_deg - a_min) / inc))
        hi = min(len(s.ranges) - 1, int((target + self.lidar_match_half_deg - a_min) / inc))
        return _min_range(s.ranges, lo, hi)

    def _front_nearest(self, now):
        """전방 섹터 최근접 (거리, 상대방위). 부표 방위를 못 받을 때의 대체. 모르면 (None,None)."""
        info = self._scan_fresh(now)
        if info is None:
            return None, None
        s, inc, a_min = info
        lo = max(0, int((LIDAR_FORWARD_DEG - self.front_sector_half_deg - a_min) / inc))
        hi = min(len(s.ranges) - 1, int((LIDAR_FORWARD_DEG + self.front_sector_half_deg - a_min) / inc))
        best_d, best_i = float('inf'), -1
        for i in range(lo, hi + 1):
            r = s.ranges[i]
            if math.isfinite(r) and r > 0.0 and r < best_d:
                best_d, best_i = r, i
        if best_i < 0:
            return None, None
        rel = (a_min + best_i * inc) - LIDAR_FORWARD_DEG
        return best_d, rel

    def _target(self, now):
        """부표 (거리, 상대방위). 비전 방위 우선, 없으면 LiDAR 전방 최근접."""
        with self.lock:
            rel = self.buoy_rel if (self.buoy_rel is not None and self.t_angle is not None
                                    and (now - self.t_angle) <= self.angle_stale_sec) else None
        if rel is not None:
            return self._dist_at(rel, now), rel
        return self._front_nearest(now)

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

        dist, rel = self._target(now)
        if rel is not None:
            self.last_seen_t = now
        lost = self.last_seen_t is None or (now - self.last_seen_t) > self.lost_sec

        # ---- INIT: 진입 직후 손 떼(autonomous 가 부표 WP 로) ----
        if self.state == "INIT":
            self.publish(CANDIDATE_INVALID)
            if self.elapsed() >= 1.0:
                self.set_state("SEARCH")
            return

        # ---- SEARCH: 부표 잡힐 때까지 (autonomous 가 접근 중) ----
        if self.state == "SEARCH":
            if rel is not None:
                self.hold_timer.reset()
                self.set_state("HOLD")
                return
            if self.elapsed() > self.search_timeout_sec:
                self.get_logger().warn("부표 못 찾음(SEARCH 타임아웃) → 종료")
                self.set_state("DONE")
                return
            self.publish(CANDIDATE_INVALID)      # 못 찾는 동안 autonomous 유지
            return

        # ---- HOLD: 능동 위치유지 ----
        if self.state == "HOLD":
            if lost:
                self.set_state("SEARCH")
                return

            # 규정 판정: keep_radius 이내 연속 hold_time
            if self.hold_timer.satisfied(dist, now):
                self.get_logger().info(f"✅ 위치유지 {self.hold_time_sec}s 달성(규정 만족) → 완료")
                self.set_state("DONE")
                return

            # bang-bang + 데드밴드. 부표를 향해 조향하며 전/후진.
            action = station_keep_action(dist, self.hold_dist_m, self.dist_tol_m)
            if action == 'forward':
                self.publish(rel_to_raw(rel))    # 부표 방향으로 전진
            elif action == 'reverse':
                self.publish(REVERSE_REL)        # 후진 (조류로 붙었을 때 되돌림)
            elif action == 'hold':
                self.publish(STOP_HOLD)          # 데드밴드 안 → 중립 (벗어나면 위에서 되돌림)
            else:  # unknown: 거리를 모른다 → 부표를 향하되 함부로 전진하지 않음
                self.publish(STOP_HOLD)
            return

        # ---- DONE: 손 떼(autonomous 가 다음 WP) ----
        self.publish(CANDIDATE_INVALID)


def clamp_pm180(a):
    return (a + 180.0) % 360.0 - 180.0


def rel_to_raw(a):
    return (a + 360.0) % 360.0


def _min_range(ranges, lo, hi):
    if lo > hi:
        return None
    best = float('inf')
    for i in range(lo, hi + 1):
        r = ranges[i]
        if math.isfinite(r) and r > 0.0 and r < best:
            best = r
    return best if math.isfinite(best) else None


def main(args=None):
    rclpy.init(args=args)
    node = ShipBack()
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
