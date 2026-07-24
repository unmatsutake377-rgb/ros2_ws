"""ship_dock — 도킹 미션 (점수 15%). 6a 에서 전면 재작성.

작년 상태: **1년 내내 침묵했다.** active_wp_mode 가 9 인데 웨이포인트의 도킹 모드는 7 이었다.
          배는 멀쩡히 돌아다녔고 도킹만 안 했다. 침묵하는 노드는 에러를 내지 않는다.

고친 것:
  [1] active_wp_mode 9 → 7 (도킹 부활)
  [2] 카메라 뎁스 폐기. OAK-1 W POE 는 RGB 전용이라 /image_distance 가 죽는다.
      → 거리는 LiDAR 전방 섹터(정면 ±front_sector_half_deg)의 **최소거리**만 쓴다.
      ★ '표식 방위의 LiDAR 거리'를 찾지 않는다. 도크는 넓은 구조물이라 비스듬히 접근하면
        표식(중앙) 방위와 LiDAR 최근접점 방위가 최대 15° 어긋난다 → 매칭이 틀린다.
        "그 표식까지 얼마"가 아니라 "전방에 뭔가 얼마나 가까운가"만 알면 된다.
        도크에 접근 중이면 전방 최소거리 = 도크까지 거리다. 각도 정합 문제가 통째로 사라진다.
  [4] 정면 접근(head-on) FSM. 비스듬한 접근이 모든 문제의 근원이다(도형 면적 감소, 정합 오차).
      INIT → SEARCH → ALIGN → APPROACH → CONTACT → BACKOFF → DONE
      ALIGN 은 **제자리 선회**로 표식을 정면에 놓는다 (2단계에서 spin_forward_pwm=1500 로
      진짜 제자리 선회가 됐기 때문에 가능해졌다. 작년 SPIN 은 순항속도 원이라 도크에 부딪혔다).
  [5] SPIN 규약: 5000=우선회, 6000=좌선회 (CLAUDE.md 3-9 상수표)

⚠️ 이 노드는 mode 7 에서 /candidate_angle 의 **유일한 발행자**다.
   (6a 에서 north_goal_angle 의 mode-7 폴백을 제거했다 — 안 그러면 발행자가 둘이라 조향이 튄다.)
   따라서 mode 7 동안 **매 틱 반드시 발행해야 한다.** 침묵하면 ship_direction 이 묵은 값을 붙든다.

프레임 규약 (코드에서 확인함, 추측 아님):
   /candidate_angle = 상대 방위(도). 0 = 정면. ship_direction 이 desired = 80 + rel 로 매핑한다.
   LiDAR 프레임도 80° = 정면 (ship_direction 이 두 프레임을 같게 다룬다).
   후진 = 상대 180 → desired 260 → motor_control 분기 (5).
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

# 특수 신호 상수 (CLAUDE.md 3-9 상수표)
SPIN_RIGHT = 5000.0          # 우선회
SPIN_LEFT = 6000.0           # 좌선회
CANDIDATE_INVALID = 20000.0  # 미션 없음
STOP_HOLD = 50000.0          # 정지/대기

REVERSE_REL = 180.0          # 상대 180° = 후진 (→ desired 260 → motor_control 분기 5)
IMAGE_INVALID = 10000.0      # /image_angle 의 '표식 없음' 센티널 (작년 규약 유지)

LIDAR_FORWARD_DEG = 80.0     # LiDAR 프레임에서 정면에 해당하는 각도



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

class ShipDock(Node):
    def __init__(self):
        super().__init__("ship_dock")

        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        self.lock = threading.Lock()

        # ---- 파라미터 (config/ship_dock.yaml, CLAUDE.md 1-4) ----
        self.active_wp_mode = int(self.declare_parameter("active_wp_mode", 7).value)  # 🚨 작년 9 (침묵)
        self.period_sec = float(self.declare_parameter("period_sec", 0.1).value)

        # 거리 (전부 LiDAR 전방 섹터 최소거리 기준)
        self.front_sector_half_deg = float(
            self.declare_parameter("front_sector_half_deg", 15.0).value)
        self.standoff_dist_m = float(self.declare_parameter("standoff_dist_m", 3.0).value)  # ⚠️ 실측 필요
        self.contact_dist_m = float(self.declare_parameter("contact_dist_m", 1.0).value)    # ⚠️ 실측 필요(도크 규격 공지 후)
        self.backoff_dist_m = float(self.declare_parameter("backoff_dist_m", 2.5).value)    # ⚠️ 실측 필요

        # 정렬
        self.align_tol_deg = float(self.declare_parameter("align_tol_deg", 8.0).value)
        self.realign_tol_deg = float(self.declare_parameter("realign_tol_deg", 25.0).value)
        # /image_angle 의 부호 규약: 양수가 좌현인가? ⚠️ 벤치 검증 필요 (카메라 좌표계)
        self.image_positive_is_left = bool(
            self.declare_parameter("image_positive_is_left", True).value)
        # SEARCH sweep 방향
        self.sweep_right = bool(self.declare_parameter("sweep_right", True).value)

        # 시간
        self.init_wait_sec = float(self.declare_parameter("init_wait_sec", 3.0).value)
        self.search_timeout_sec = float(self.declare_parameter("search_timeout_sec", 25.0).value)
        self.align_timeout_sec = float(self.declare_parameter("align_timeout_sec", 15.0).value)
        self.approach_timeout_sec = float(self.declare_parameter("approach_timeout_sec", 30.0).value)
        self.contact_hold_sec = float(self.declare_parameter("contact_hold_sec", 3.0).value)  # ⚠️ 규정 확인
        self.backoff_timeout_sec = float(self.declare_parameter("backoff_timeout_sec", 8.0).value)

        # 신선도 (묵은 데이터로 도킹하지 않는다)
        self.image_stale_sec = float(self.declare_parameter("image_stale_sec", 1.0).value)
        self.scan_stale_sec = float(self.declare_parameter("scan_stale_sec", 1.0).value)

        # ---- 구독/발행 (토픽 이름 불변, CLAUDE.md 1-3) ----
        self.pub_candidate = self.create_publisher(Float32, "/candidate_angle", qos)

        self.create_subscription(Int32, "/wp_mode", self.wp_mode_cb, qos)
        self.create_subscription(Float32, "/image_angle", self.image_angle_cb, qos)
        self.create_subscription(LaserScan, "/scan", self.scan_cb, SCAN_QOS)
        # ※ /image_distance 는 **구독하지 않는다**. 카메라 뎁스라 OAK-1 에서 죽는다. [2]

        # ---- 상태 ----
        self.wp_mode = -1
        self.image_angle = IMAGE_INVALID
        self.last_image_t = None
        self.latest_scan = None
        self.last_scan_t = None

        self.state = "INIT"
        self.state_t = None
        self.standoff_aligned = False   # standoff 에서 정밀 재정렬을 이미 했는가 (1회만)

        self.create_timer(self.period_sec, self.timer_cb)

        self.get_logger().info(
            f"ship_dock 시작: active_wp_mode={self.active_wp_mode} (작년엔 9 라 1년간 침묵했다).\n"
            f"   거리는 LiDAR 전방 ±{self.front_sector_half_deg}° 최소거리만 사용 (카메라 뎁스 안 씀).\n"
            f"   standoff={self.standoff_dist_m}m contact={self.contact_dist_m}m"
        )

    # ───────────────────────── 콜백 ─────────────────────────
    def wp_mode_cb(self, msg):
        with self.lock:
            mode = int(msg.data)
            if mode != self.wp_mode and mode != self.active_wp_mode:
                # 내 차례가 아니다 → FSM 초기화
                self.state = "INIT"
                self.state_t = None
                self.standoff_aligned = False
            self.wp_mode = mode

    def image_angle_cb(self, msg):
        with self.lock:
            self.image_angle = float(msg.data)
            self.last_image_t = time.monotonic()

    def scan_cb(self, msg):
        with self.lock:
            self.latest_scan = msg
            self.last_scan_t = time.monotonic()

    # ───────────────────────── 보조 ─────────────────────────
    def set_state(self, new_state):
        if new_state != self.state:
            self.get_logger().info(f"🛳 도킹 {self.state} → {new_state}")
        self.state = new_state
        self.state_t = time.monotonic()

    def elapsed(self):
        return 0.0 if self.state_t is None else (time.monotonic() - self.state_t)

    def marker_ok(self, now):
        """표식이 유효한가 (센티널 아님 + 신선함)."""
        if self.last_image_t is None:
            return False
        if (now - self.last_image_t) > self.image_stale_sec:
            return False
        return self.image_angle != IMAGE_INVALID and math.isfinite(self.image_angle)

    def front_min_dist(self, now):
        """전방 섹터(정면 ±half)의 LiDAR 최소거리. 모르면 None.

        ★ 표식 방위로 매칭하지 않는다. 도크는 넓어서 표식 방위와 LiDAR 최근접점이 최대 15° 어긋난다.
          "전방에 뭔가 얼마나 가까운가"만 알면 접안에는 충분하다."""
        scan = self.latest_scan
        if scan is None or self.last_scan_t is None:
            return None
        if (now - self.last_scan_t) > self.scan_stale_sec:
            return None

        inc = math.degrees(scan.angle_increment)
        if not math.isfinite(inc) or inc <= 0.0:
            return None                      # 0 나눗셈 방어

        a_min = math.degrees(scan.angle_min)
        lo_deg = LIDAR_FORWARD_DEG - self.front_sector_half_deg
        hi_deg = LIDAR_FORWARD_DEG + self.front_sector_half_deg

        lo = int((lo_deg - a_min) / inc)
        hi = int((hi_deg - a_min) / inc)
        lo = max(0, lo)
        hi = min(len(scan.ranges) - 1, hi)
        if lo > hi:
            return None

        best = float('inf')
        for i in range(lo, hi + 1):
            r = scan.ranges[i]
            if math.isfinite(r) and r > 0.0 and r < best:
                best = r
        return best if math.isfinite(best) else None

    def spin_toward(self, angle):
        """표식 쪽으로 제자리 선회하는 SPIN 코드를 고른다."""
        marker_is_left = (angle > 0.0) if self.image_positive_is_left else (angle < 0.0)
        return SPIN_LEFT if marker_is_left else SPIN_RIGHT

    def publish(self, value):
        self.pub_candidate.publish(Float32(data=float(value)))

    # ───────────────────────── FSM ─────────────────────────
    def timer_cb(self):
        with self.lock:
            if self.wp_mode != self.active_wp_mode:
                return                       # 내 차례가 아니다 → 발행하지 않는다

            now = time.monotonic()
            if self.state_t is None:
                self.state_t = now

            dist = self.front_min_dist(now)
            has_marker = self.marker_ok(now)
            angle = self.image_angle if has_marker else 0.0

            # 거리를 모르면 접근하지 않는다. 눈 감고 도킹하지 않는다.
            if dist is None and self.state in ("APPROACH", "CONTACT", "BACKOFF"):
                self.get_logger().warn(
                    "LiDAR 거리 없음 → 도킹 접근 중단(정지)", throttle_duration_sec=2.0)
                self.publish(STOP_HOLD)
                return

            # ---- INIT: 잠깐 멈춰 안정화 ----
            if self.state == "INIT":
                self.publish(STOP_HOLD)
                if self.elapsed() >= self.init_wait_sec:
                    self.set_state("SEARCH")
                return

            # ---- SEARCH: 제자리 선회하며 표식을 찾는다 ----
            if self.state == "SEARCH":
                if has_marker:
                    self.set_state("ALIGN")
                    return
                if self.elapsed() > self.search_timeout_sec:
                    self.get_logger().error("도킹 표식을 못 찾음(SEARCH 타임아웃) → 정지")
                    self.set_state("DONE")
                    return
                self.publish(SPIN_RIGHT if self.sweep_right else SPIN_LEFT)
                return

            # ---- ALIGN: 제자리 선회로 표식을 정면에 놓는다 (정면 접근의 핵심) ----
            if self.state == "ALIGN":
                if not has_marker:
                    self.set_state("SEARCH")
                    return
                if abs(angle) <= self.align_tol_deg:
                    self.set_state("APPROACH")
                    return
                if self.elapsed() > self.align_timeout_sec:
                    self.get_logger().error("정렬 실패(ALIGN 타임아웃) → 재탐색")
                    self.set_state("SEARCH")
                    return
                self.publish(self.spin_toward(angle))
                return

            # ---- APPROACH: 표식을 향해 직진. 거리는 LiDAR 전방 최소거리. ----
            if self.state == "APPROACH":
                if not has_marker:
                    self.set_state("SEARCH")
                    return
                if self.elapsed() > self.approach_timeout_sec:
                    self.get_logger().error("접근 타임아웃 → 후진")
                    self.set_state("BACKOFF")
                    return

                if dist <= self.contact_dist_m:
                    self.set_state("CONTACT")
                    return

                # standoff 지점에서 한 번 정밀 재정렬 (비스듬한 접안 방지)
                if (not self.standoff_aligned) and dist <= self.standoff_dist_m \
                        and abs(angle) > self.align_tol_deg:
                    self.standoff_aligned = True
                    self.set_state("ALIGN")
                    return

                # 크게 틀어졌으면 다시 제자리 정렬 (비스듬히 밀고 들어가지 않는다)
                if abs(angle) > self.realign_tol_deg:
                    self.set_state("ALIGN")
                    return

                self.publish(angle)          # 표식 방위로 조향하며 전진
                return

            # ---- CONTACT: 접안 완료. 유지. ----
            if self.state == "CONTACT":
                self.publish(STOP_HOLD)
                if self.elapsed() >= self.contact_hold_sec:
                    self.set_state("BACKOFF")
                return

            # ---- BACKOFF: 후진으로 빠져나온다 ----
            if self.state == "BACKOFF":
                if dist is not None and dist >= self.backoff_dist_m:
                    self.set_state("DONE")
                    return
                if self.elapsed() > self.backoff_timeout_sec:
                    self.set_state("DONE")
                    return
                self.publish(REVERSE_REL)    # 상대 180° = 후진
                return

            # ---- DONE: 정지 유지 (mode 7 동안 계속 발행해야 한다) ----
            self.publish(STOP_HOLD)


def main(args=None):
    rclpy.init(args=args)
    node = ShipDock()
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


if __name__ == "__main__":
    main()
