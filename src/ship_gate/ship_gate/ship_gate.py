"""ship_gate — 빨강-초록 게이트 통과. 6b 에서 재작성.

고친 것:
  [1] 🚨 yellow 폴백 제거. 규정은 '빨강-초록' 게이트인데 작년 코드에 yellow 가 섞여 있었다.
  [2] active_wp_modes = [0, 1]. 작년엔 ship_gate=1, ship_last=0 (WP0 "게이트 시작"인데 ship_last 가
      잡고 GPS 폴백만 냈다). 게이트 접근 구간(mode 0)부터 비전을 쓰면 정렬 시간을 번다.
      ship_last 는 6b 에서 제거했다 → mode 0 은 ship_gate 단독(발행자 둘 충돌 없음).
      ★ north 의 FALLBACK_MODES 에 0 을 넣으면 안 된다: ship_gate 가 mode 0 을 발행하는데 north 도
        폴백을 내면 다시 발행자 둘이 된다. FALLBACK_MODES 는 (5,8) 로 유지.
  [3] 🚨 게이트 쌍 제약. '다음 게이트 빨강' + '이번 게이트 초록'이 같이 보이면 둘의 중간으로
      가버린다(엉뚱). 각도차가 [pair_min_sep, pair_max_sep] 일 때만 쌍으로 인정. 벗어나면
      가까운 쪽 하나만. (판정은 gate_logic.gate_candidate — 순수 함수, 테스트됨)
  [4] 순차 통과 카운트 → /gates_passed (Int32) 발행. 부표가 ±pass_behind_deg 를 넘어 뒤로 가면
      1 증가. ★ 이름 주의: /gate_pass_count 아니라 /gates_passed (0단계 blackbox 가 구독 중).
  [5] 거리는 LiDAR 에서. /red_distance·/green_distance 는 카메라 뎁스라 OAK-1 에서 죽는다.
      → 그 두 토픽 구독을 버리고 /scan 을 구독. 부표는 '점 물체'라 표식-LiDAR 매칭이 통한다
        (도크와 다르다). 거리는 '먼 게이트를 미리 카운트하지 않게' arming 을 거르는 데만 쓴다
        (LiDAR 가 부표를 못 봐 거리 미상이면 막지 않는다 — 카운트를 죽이면 안 되니까).
  [6] red_on_port 파라미터 (빨강이 좌현인가). ⚠️ 벤치 검증 필요.

  기타: time.monotonic() (CLAUDE.md 1-5). /candidate_angle, /red_angle, /green_angle 이름 불변.
        작년 구독 배선이 뒤엉켜 있었다(/green_angle→red 변수 등) → 바로잡았다.

프레임 규약: /red_angle·/green_angle = 상대각(0=정면). 게이트 중점을 상대각으로 구해
  rel_to_raw_0_360 으로 /candidate_angle(0~360) 규약에 맞춰 발행 → ship_direction 이 (B)에서 매핑.
  LiDAR 프레임은 80°=정면.
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import LaserScan

from ship_gate.gate_logic import GateCounter, gate_candidate, rel_to_raw_0_360, clamp_pm180

INVALID_ANGLE = 10000.0       # /red_angle·/green_angle 의 '안 보임' 센티널 (작년 규약 유지)
CANDIDATE_INVALID = 20000.0   # 게이트 없음 → ship_direction 자율(GPS)
LIDAR_FORWARD_DEG = 80.0      # LiDAR 프레임에서 정면 (상대각 0 에 대응)


class ShipGate(Node):
    def __init__(self):
        super().__init__("ship_gate")

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.lock = threading.Lock()

        # ---- 파라미터 (config/ship_gate.yaml, CLAUDE.md 1-4) ----
        self.active_wp_modes = list(self.declare_parameter("active_wp_modes", [0, 1]).value)
        self.period_sec = float(self.declare_parameter("period_sec", 0.1).value)

        # 게이트 쌍 제약 [3]
        self.pair_min_sep_deg = float(self.declare_parameter("pair_min_sep_deg", 5.0).value)
        self.pair_max_sep_deg = float(self.declare_parameter("pair_max_sep_deg", 80.0).value)
        # 한쪽만 보일 때 게이트 중심 쪽으로 트는 각 (작년 DELTA_FIX 대체)
        self.single_offset_deg = float(self.declare_parameter("single_offset_deg", 40.0).value)
        # 🚨 빨강이 좌현인가 — 한쪽만 보일 때 트는 방향을 정한다. ⚠️ 벤치 검증 필요 [6]
        self.red_on_port = bool(self.declare_parameter("red_on_port", True).value)

        # 통과 카운트 [4]
        self.pass_behind_deg = float(self.declare_parameter("pass_behind_deg", 75.0).value)
        # 먼 게이트를 미리 카운트하지 않게 arming 을 이 거리 안으로 제한(거리 미상이면 막지 않음)
        self.gate_arm_dist_m = float(self.declare_parameter("gate_arm_dist_m", 6.0).value)
        self.lidar_match_half_deg = float(self.declare_parameter("lidar_match_half_deg", 3.0).value)

        # 신선도
        self.angle_stale_sec = float(self.declare_parameter("angle_stale_sec", 0.7).value)
        self.scan_stale_sec = float(self.declare_parameter("scan_stale_sec", 1.0).value)

        # ---- 발행 ----
        self.pub_candidate = self.create_publisher(Float32, "/candidate_angle", qos)
        self.pub_gates = self.create_publisher(Int32, "/gates_passed", qos)   # 🚨 /gate_pass_count 아님

        # ---- 구독 (토픽 이름 확정. /red_distance·/green_distance 는 버린다 [5]) ----
        self.create_subscription(Int32, "/wp_mode", self.wp_mode_cb, qos)
        self.create_subscription(Float32, "/red_angle", self.red_angle_cb, qos)
        self.create_subscription(Float32, "/green_angle", self.green_angle_cb, qos)
        self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)

        # ---- 상태 ----
        self.wp_mode = -1
        self.red_rel_raw = INVALID_ANGLE
        self.green_rel_raw = INVALID_ANGLE
        self.t_red = None
        self.t_green = None
        self.latest_scan = None
        self.t_scan = None

        self.counter = GateCounter(self.pass_behind_deg)

        self.create_timer(self.period_sec, self.timer_cb)

        self.get_logger().info(
            f"ship_gate 시작: active_wp_modes={self.active_wp_modes}, red_on_port={self.red_on_port}"
            f"(⚠️벤치검증). 쌍 제약 {self.pair_min_sep_deg}~{self.pair_max_sep_deg}°, "
            f"통과 ±{self.pass_behind_deg}° → /gates_passed."
        )

    # ---------------------- 콜백 ----------------------
    def wp_mode_cb(self, msg):
        with self.lock:
            self.wp_mode = int(msg.data)

    def red_angle_cb(self, msg):
        with self.lock:
            self.red_rel_raw = float(msg.data)
            self.t_red = time.monotonic()

    def green_angle_cb(self, msg):
        with self.lock:
            self.green_rel_raw = float(msg.data)
            self.t_green = time.monotonic()

    def scan_cb(self, msg):
        with self.lock:
            self.latest_scan = msg
            self.t_scan = time.monotonic()

    # ---------------------- 보조 ----------------------
    def _fresh_angle(self, raw, t, now):
        """센티널 아니고 신선하면 clamp 된 상대각, 아니면 None."""
        if raw == INVALID_ANGLE or not math.isfinite(raw):
            return None
        if t is None or (now - t) > self.angle_stale_sec:
            return None
        return clamp_pm180(raw)

    def _lidar_dist_at(self, rel_deg, now):
        """부표 상대방위의 LiDAR 최소거리(±match_half). 모르면 None.
        부표는 점 물체라 방위 매칭이 통한다(도크와 다르다)."""
        scan = self.latest_scan
        if scan is None or self.t_scan is None or (now - self.t_scan) > self.scan_stale_sec:
            return None
        inc = math.degrees(scan.angle_increment)
        if not math.isfinite(inc) or inc <= 0.0:
            return None
        a_min = math.degrees(scan.angle_min)
        target = LIDAR_FORWARD_DEG + rel_deg
        lo = int((target - self.lidar_match_half_deg - a_min) / inc)
        hi = int((target + self.lidar_match_half_deg - a_min) / inc)
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

    # ---------------------- 메인 루프 ----------------------
    def timer_cb(self):
        with self.lock:
            if self.wp_mode not in self.active_wp_modes:
                return
            now = time.monotonic()
            red = self._fresh_angle(self.red_rel_raw, self.t_red, now)
            green = self._fresh_angle(self.green_rel_raw, self.t_green, now)

        # 조향각 결정 (순수 함수 — 쌍 제약/단일 폴백 포함)
        steer_rel, valid_pair = gate_candidate(
            red, green, self.red_on_port,
            self.pair_min_sep_deg, self.pair_max_sep_deg, self.single_offset_deg)

        # ---- 통과 카운트 ----
        # 보이는 부표들의 |상대각|
        abs_angles = [abs(a) for a in (red, green) if a is not None]
        both_ahead = valid_pair and all(x <= self.pass_behind_deg for x in abs_angles)

        # 먼 게이트를 미리 카운트하지 않게 — LiDAR 거리로 arming 을 거른다(미상이면 막지 않음).
        close_enough = True
        if both_ahead:
            dists = [d for d in (self._lidar_dist_at(a, now) for a in (red, green) if a is not None)
                     if d is not None]
            if dists:
                close_enough = min(dists) <= self.gate_arm_dist_m

        gate_ahead = both_ahead and close_enough
        passed = self.counter.update(gate_ahead, abs_angles)
        self.pub_gates.publish(Int32(data=int(self.counter.count)))
        if passed:
            self.get_logger().info(f"🚩 게이트 통과 #{self.counter.count}")

        # ---- 조향 발행 ----
        if steer_rel is None:
            self.pub_candidate.publish(Float32(data=CANDIDATE_INVALID))   # 게이트 없음 → 자율(GPS)
        else:
            self.pub_candidate.publish(Float32(data=rel_to_raw_0_360(steer_rel)))


def main(args=None):
    rclpy.init(args=args)
    node = ShipGate()
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
