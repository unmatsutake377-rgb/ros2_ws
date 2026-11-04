"""ship_turn — 부표 선회(orbit) 미션. 6c 에서 동작이 통째로 바뀐다.

작년 문제:
  [1] 🚨 '비껴 지나가기'였다. offset 을 줘서 부표 옆을 스쳐 지나갔다. **규정은 '선회(orbit)'** —
      규정 위반이었다. → 부표를 중심으로 원을 그리며 도는 orbit 기동으로 재작성.
      부표를 ±90°에 두고(제어법은 orbit_logic.orbit_steer), LiDAR 거리로 반경을 보정한다.
  [2] 🚨 흰색 부표 인식이 없었다. 규정: **빨강·초록 = 시계 / 흰색 = 반시계.** 작년은 색 매핑도
      엉뚱했다(red→left 등). → /image_color 로 색을 받아 회전 방향을 정한다.
  [3] 거리는 LiDAR 에서. /image_distance(카메라 뎁스, OAK-1 에서 죽음) 구독 폐기, /scan 사용.
      부표는 '점 물체'라 방위 매칭이 통한다(도크와 다르다).
  [4] 🚨 [2026-11-04] ORBIT 진입 직후 노드가 죽었다(계약서_ship_turn선회_20261104). 세 가지가 겹쳤다:
      (C) orbit_logic 이 "+ = 우현" 으로 쓰여 좌 양수 입·출력과 반대 → CW 요청에 CCW. 좌 양수로 정정.
      (A) orbit 자세(부표 ±90°)는 카메라 HFOV(±40°) 밖이라 /image_angle 이 10000 → rel None →
          orbit_steer(None) TypeError → 프로세스 종료(respawn 없음) → ship_direction 이 마지막 조향 영구 고정.
          → ORBIT 에서 방위가 없으면 직전 유효 조향을 유지(없으면 접선 0), lost_sec 뒤 SEARCH(기존 규약).
      (B) 카메라 밖에서도 선회를 이어가려면 LiDAR 로 방위를 추적해야 한다(lidar_track.py).
          ORBIT 과 APPROACH 에서도 켜지며 실질 게이트는 lidar_track_max_m(5 m) 뿐 — 카메라가 한 틱 끊기면
          5 m 안 다른 반사(수조 벽·게이트 부표)를 잡을 수 있다. 카메라 방위가 돌아오면 그것이 덮어쓴다.

파라미터(6c-[4]): orbit_radius_m(⚠️실측), radius_tol_m, approach_dist_m, k_radial_deg_per_m,
                  orbit_target_deg(360). SPIN 규약(6c-[5]): 5000=우선회, 6000=좌선회.
  enabled(bool, 기본 true): false 면 이 노드는 **어떤 모드에서도** /candidate_angle 을 안 낸다.
    launch 가 search_behavior:=hold 면 false 로 띄운다(탐색 = 부표 앞 위치유지, ship_back 담당).
    🚨 끄려고 active_wp_mode 를 -1 로 두지 마라 — 부팅 초기값(self.wp_mode=-1)과 일치해
    /wp_mode 첫 수신 전에 게이트를 통과, SPIN 5000 을 낸다.

FSM: WAIT → SEARCH → APPROACH → ORBIT → COOLDOWN → (다음 부표 위해 SEARCH)
  WP3,4,5(부표 3개)는 모두 /wp_mode==3 으로 온다. 한 부표를 돌고 COOLDOWN 동안 손 떼면
  autonomous(GPS)가 다음 부표로 데려가고, 다시 SEARCH 로 잡는다.

프레임 규약: /image_angle=상대각(0=정면, **좌 양수**). orbit 조향각을 상대(좌 양수)로 구해 rel_to_raw 로
  /candidate_angle(0~360) 규약에 맞춰 발행 → ship_direction (B)에서 80+raw 로 매핑. LiDAR 80°=정면, a−80 좌 양수.
  CW(빨강·초록) = 부표를 우현(rel −90)에 둔다. 부표 월드 방위 = yaw − rel (yaw 는 CW 양수).
  발행 전 조향을 ±STEER_LIMIT_DEG(80) 로 클램프 — ship_direction 매핑 범위(0~160) 밖은 STOP_HOLD 가 된다.

색 토픽 = /image_color (String: "red"/"green"/"white"). 발행자는 basic_image_subscriberturn.
   🚨 [2026-09-23 정정] 종전엔 /buoy_color 를 구독했는데 그 이름을 발행하는 노드가 없었다.
   비전은 처음부터 /image_color 로 내보내고 있었다 — 토픽명 불일치로 탐색이 SEARCH 에서 영원히 멈췄을 것이다.
   못 받으면 회전 방향을 모른다 → 그동안 SEARCH 에 머문다(틀린 방향으로 돌지 않는다).
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32, Int32, String, Float64
from sensor_msgs.msg import LaserScan

from ship_turn.orbit_logic import (
    orbit_steer, orbit_expected_rel, buoy_world_bearing, OrbitProgress, orbit_direction_cw,
    clamp, clamp_pm180, is_active)
from ship_turn.lidar_track import lidar_track

IMAGE_ANGLE_INVALID = 10000.0
CANDIDATE_INVALID = 20000.0
SPIN_RIGHT = 5000.0          # 우선회 (CLAUDE.md 3-9)
SPIN_LEFT = 6000.0           # 좌선회
LIDAR_FORWARD_DEG = 80.0     # LiDAR 프레임 정면
# ship_direction (B) 매핑: mapped = 80 + raw, 정상 범위 0~160 → 상대 조향 |steer| ≤ 80 만 통과하고
# 그 밖은 STOP_HOLD(1 s 스로틀 warn 뿐). orbit 진입(rel≈0, 90−gain)은 80 을 쉽게 넘으므로 발행 전 클램프.
# 튜닝값이 아니라 하류 프로토콜 상수(LIDAR_FORWARD_DEG 와 같은 성격) — 파라미터로 두지 않는다.
STEER_LIMIT_DEG = 80.0


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


class ShipTurn(Node):
    def __init__(self):
        super().__init__('ship_turn')
        self.lock = threading.Lock()
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)

        # ---- 파라미터 (config/ship_turn.yaml, CLAUDE.md 1-4) ----
        # enabled=False: 어떤 모드에서도 발행 안 함 (launch search_behavior:=hold).
        #   active_wp_mode 는 그대로 3 — -1 센티널은 부팅 초기값과 겹쳐 금지(상단 docstring).
        self.enabled = bool(self.declare_parameter('enabled', True).value)
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
        # (B) LiDAR 방위 추적 — 카메라 밖(ORBIT·APPROACH 말기)에서 직전 유효 방위 ±창의 최근접 빔을 부표로.
        #   🚨 대회 launch 는 yaml 을 안 넘긴다(launch_files.launch.py 는 enabled 만) → 이 기본값이 대회값.
        #   ⚠️ 실측 필요: LiDAR 높이 ≠ 부표 높이면 못 본다 / 수조(6×6)에선 벽이 창 안 최근접이 될 수 있다.
        self.lidar_track_window_deg = float(self.declare_parameter('lidar_track_window_deg', 25.0).value)
        self.lidar_track_max_m = float(self.declare_parameter('lidar_track_max_m', 5.0).value)  # = orbit_radius 3 + 2

        # ---- 발행/구독 (토픽 이름 불변, /image_distance 는 버림) ----
        self.pub_candidate = self.create_publisher(Float32, '/candidate_angle', qos)
        self.create_subscription(Int32, '/wp_mode', self.wp_cb, qos)
        self.create_subscription(Float32, '/image_angle', self.angle_cb, qos)
        # [2] 비전 turn 노드(basic_image_subscriberturn)가 발행한다.
        #     🚨 09-23: /buoy_color 를 구독하고 있었는데 그 이름을 내는 노드가 없어
        #     탐색이 SEARCH 에서 멈췄다. 이름을 바꾸지 마라 — CLAUDE.md §3-9.
        self.create_subscription(String, '/image_color', self.color_cb, qos)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, SCAN_QOS)
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
        self.last_rel = None        # (B) 마지막 유효 부표 방위(카메라든 LiDAR 든) — LiDAR 추적의 예상 방위
        self.last_steer = None      # (A) ORBIT 마지막 유효 조향 — 방위 없을 때 유지
        self.rel_src = None         # "cam" / "lidar" — 소스 전환 로그용

        self.create_timer(self.period_sec, self.timer_cb)
        self.get_logger().info(
            f"ship_turn 시작: enabled={self.enabled}, active_wp_mode={self.active_wp_mode}, "
            f"orbit r={self.orbit_radius_m}m(⚠️실측), 목표 {self.orbit_target_deg}°.\n"
            f"   빨강·초록=시계 / 흰색=반시계. 거리는 LiDAR. /image_color 없으면 SEARCH 유지."
        )
        if not self.enabled:
            self.get_logger().warn(
                "search_behavior=hold: ship_turn 침묵, 탐색은 ship_back 담당")

    # ---------------------- 콜백 ----------------------
    def wp_cb(self, msg):
        with self.lock:
            prev = self.wp_mode
            self.wp_mode = int(msg.data)
            if prev != self.active_wp_mode and \
                    is_active(self.enabled, self.wp_mode, self.active_wp_mode):
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
        self._forget_buoy()

    def _forget_buoy(self):
        """부표를 놓았을 때(SEARCH/COOLDOWN 진입) 추적 상태를 지운다 — 묵은 방위로 LiDAR 를 매칭하지 않게."""
        self.last_seen_t = None
        self.last_rel = None
        self.last_steer = None
        self.rel_src = None

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

    def _lidar_track(self, expected_rel, now):
        """(B) 카메라 방위가 없을 때: 예상 방위 ±lidar_track_window_deg, 거리 ≤ lidar_track_max_m 의
        최근접 빔을 부표로 → (rel, dist). 스캔이 묵었거나 창 안에 없으면 None."""
        scan = self.latest_scan
        if scan is None or self.t_scan is None or (now - self.t_scan) > self.scan_stale_sec:
            return None
        return lidar_track(scan.ranges, math.degrees(scan.angle_min), math.degrees(scan.angle_increment),
                           expected_rel, self.lidar_track_window_deg, self.lidar_track_max_m,
                           LIDAR_FORWARD_DEG)

    def publish(self, value):
        self.pub_candidate.publish(Float32(data=float(value)))

    def publish_steer(self, rel_deg):
        """상대 조향(좌 양수)을 ±STEER_LIMIT_DEG 로 잘라 /candidate_angle 0~360 으로 발행."""
        self.publish(rel_to_raw(clamp(rel_deg, -STEER_LIMIT_DEG, STEER_LIMIT_DEG)))

    # ---------------------- FSM ----------------------
    def timer_cb(self):
        with self.lock:
            if not is_active(self.enabled, self.wp_mode, self.active_wp_mode):
                return                      # enabled=False 면 어떤 모드에서도 침묵
            now = time.monotonic()
            if self.state_t is None:
                self.state_t = now
            rel, col = self._buoy(now)
            yaw = self.yaw if (self.t_yaw and (now - self.t_yaw) <= self.angle_stale_sec) else None

        # ---- 부표 (rel, dist): 카메라 방위 우선, 없으면 (B) LiDAR 추적 ----
        dist = None
        src = None
        if rel is not None:
            src = "cam"
            dist = self._lidar_dist_at(rel, now)
        elif self.state in ("APPROACH", "ORBIT"):
            # 예상 방위 = 직전 유효 방위. 없으면(이론상 ORBIT 진입엔 항상 있다) 설계 자세(CW 우현 −90).
            expected = self.last_rel
            if expected is None and self.dir_cw is not None:
                expected = orbit_expected_rel(self.dir_cw)
            tracked = self._lidar_track(expected, now) if expected is not None else None
            if tracked is not None:
                rel, dist = tracked
                src = "lidar"
        if rel is not None:
            self.last_seen_t = now
            self.last_rel = rel
            if src != self.rel_src:
                self.get_logger().info(f"부표 방위 소스 {self.rel_src} → {src} (rel={rel:.0f}°)",
                                       throttle_duration_sec=1.0)
                self.rel_src = src
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
                self._forget_buoy()
                self.set_state("SEARCH")
                return
            if dist is not None and dist <= self.approach_dist_m:
                self.progress.reset()
                self.last_steer = None
                self.set_state("ORBIT")
                return
            if rel is None:
                # (A) lost_sec 안의 순간 끊김: 직전 유효 방위로 계속 접근(카메라·LiDAR 둘 다 없을 때)
                rel = self.last_rel
            if rel is None:
                # 정상 흐름엔 없는 상태(SEARCH→APPROACH 는 rel 이 있어야 넘어온다)지만 크래시 경로는 0 이어야 한다.
                self.publish(CANDIDATE_INVALID)
                return
            self.publish_steer(rel)                    # 부표 방향으로 직진(거리 미상이어도)
            return

        # ---- ORBIT: 반경 유지하며 한 바퀴 ----
        if self.state == "ORBIT":
            if lost:
                self._forget_buoy()
                self.set_state("SEARCH")
                return
            if self.elapsed() > self.orbit_max_sec:
                self.get_logger().warn("orbit 안전 타임아웃 → 완료 처리")
                self.set_state("COOLDOWN")
                return

            if rel is None:
                # (A) 카메라·LiDAR 둘 다 방위 없음(lost_sec 안): orbit_steer(None) 으로 죽지 않는다.
                #     직전 유효 조향을 유지, 그것도 없으면 접선 가정(원 위·접선 자세 = 조향 0).
                #     진행각은 방위 없이는 못 재니 갱신하지 않는다.
                steer = self.last_steer if self.last_steer is not None else 0.0
            else:
                d = dist if dist is not None else self.orbit_radius_m   # 거리 미상이면 접선만
                steer = orbit_steer(rel, d, self.orbit_radius_m, self.dir_cw,
                                    self.k_radial, self.radial_limit_deg)
                self.last_steer = steer

                # 진행각 누적 (yaw 있을 때만). 월드 방위 = yaw − rel (좌 양수 rel). 목표 도달 → 완료.
                if yaw is not None:
                    prog = self.progress.update(buoy_world_bearing(yaw, rel))
                    if prog >= self.orbit_target_deg:
                        self.get_logger().info(f"✅ 선회 {prog:.0f}° 완료")
                        self.set_state("COOLDOWN")
                        return

            self.publish_steer(steer)
            return

        # ---- COOLDOWN: 손 떼고(autonomous 로) 다음 부표로 이동 ----
        if self.state == "COOLDOWN":
            self.publish(CANDIDATE_INVALID)
            if self.elapsed() >= self.cooldown_sec:
                self._forget_buoy()
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
