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
      부표는 점 물체라 방위 매칭이 통한다. 부표 방위는 /image_angle(검출기 turn 의 모드별 색).
      예외(계약서_인식융합_20261112 §10 Q1 승인): distance_source 가 fill/fused 면 /fused_objects
      (perception_fusion: LiDAR·D455 뎁스 융합 물체 목록)의 거리를 쓴다 — 비전 방위 분기(_target) 한 곳만.
      fill = LiDAR 먼저·못 보면 fused, fused = fused 먼저·없으면 LiDAR, lidar = 옛 동작(구독도 없음).
  [4] (계약 전체코스 §2-4·§2-5) 획득(SEARCH→HOLD)은 비전 색 확인으로만. LiDAR 전방 최근접은
      HOLD 유지 중 직전 비전 방위가 lost_sec 안일 때 보조로만(lidar_acquire=true 면 옛 동작).
      SEARCH 시간 초과로 DONE 래치하지 않는다(search_timeout_ends=true 면 옛 동작).
      DONE 은 HOLD 에서만(성공, 또는 포기: 거리 unknown 이 unknown_giveup_sec 이어짐 §10-추가 4,
      한 WP 의 HOLD 합이 hold_max_sec §10-추가 5) — 예외: 재획득 정지 창 중 합 상한(아래 [5])
      → 3→3 미리셋(래치 유지) 그대로.
  [5] (§10-추가 결정 6) HOLD 에서 놓쳐(lost_sec) SEARCH 로 갈 때만 reacquire_stop_sec 동안
      SEARCH 가 20000 대신 STOP_HOLD — 그 자리에 멈춰 다시 찾는다. 그 안에 비전 재획득 → HOLD
      (HOLD 합 이어 셈), 못 잡으면 20000(SEARCH 그대로) → north 진행. 첫 SEARCH(INIT 뒤)는 20000.
      결정 6 보충 ①: 창(STOP_HOLD) 시간도 HOLD 총시간 합(hold_max_sec)에 든다 — 창 중에도 상한을
      판정해 넘으면 포기 DONE(20000). 창이 끝난 뒤 20000(SEARCH) 시간은 넣지 않는다.

파라미터(6d-[4]): active_wp_modes([2] = search_behavior 기본 orbit; hold 옵션이면 launch 가 [2, 3]),
                  keep_radius_m(5.0 규정), hold_time_sec(5.0 규정), hold_dist_m(2.5 ⚠️실측), dist_tol_m.

FSM: INIT → SEARCH → HOLD → DONE.
  keep_radius(5m) 이내에 hold_time(5s) '연속' 머물면 완료(규정 만족) → autonomous 로 다음 WP.
  INIT 20000(1 s) → SEARCH 20000(비전 색 확인까지) → HOLD 전/후진·STOP_HOLD.
  HOLD 에서 lost → SEARCH: 재획득 정지 창(reacquire_stop_sec) 동안 STOP_HOLD, 지나면 20000.
  HOLD 총시간 합(hold_max_sec) = HOLD 구간 + 재획득 정지 창(창 중에도 판정 → 넘으면 DONE).
  SEARCH 경과(search_timeout_sec WARN 기준)는 SEARCH 진입부터 — 창 시간 포함.

프레임 규약: /image_angle=상대각(0=정면). 전진은 부표 방위로 조향, 후진은 상대 180°.
  /candidate_angle 은 0~360(0=정면) 규약. LiDAR 80°=정면.
  전진 조향만 발행 직전 ±STEER_LIMIT_DEG(80) 클램프 — 180.0(후진 신호)이 우연히 나가지 않게.

⚠️ ship_last 는 6b 에서 이미 제거됨(6d-[3]). 여기선 ship_back 만 다룬다.
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32, Int32, Float32MultiArray
from sensor_msgs.msg import LaserScan

from ship_back.hold_logic import station_keep_action, HoldTimer, should_reset_on_mode
from ship_back.hold_logic import target_source, counts_as_seen, search_timed_out, unknown_giveup
from ship_back.hold_logic import search_timeout_warn, hold_total_exceeded
from ship_back.hold_logic import reacquire_stop_active, reacquire_window_elapsed
from ship_back.hold_logic import check_hold_params
from ship_back.hold_logic import check_distance_source, fused_allowed_src_ids, parse_fused_objects
from ship_back.hold_logic import fused_fresh, match_fused, choose_distance

IMAGE_ANGLE_INVALID = 10000.0
CANDIDATE_INVALID = 20000.0
STOP_HOLD = 50000.0
REVERSE_REL = 180.0          # 상대 180° = 후진 (→ ship_direction 260 → motor_control 분기 5)
#   근거: ship_direction (B) 정확 일치 예외(test_candidate_reverse)
LIDAR_FORWARD_DEG = 80.0     # LiDAR 프레임 정면
# ship_direction (B) 매핑: mapped = 80 + raw, 정상 범위 0~160 → 상대 조향 |steer| ≤ 80 만 조향이 된다.
# 180.0 정확 일치는 후진(260) 예외라, 부표 방위가 ±180 이면 전진 대신 후진한다 → 발행 직전 클램프.
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
# /fused_objects 구독 QoS(계약서_인식융합 §4): 발행 RELIABLE 10 ↔ 구독 BEST_EFFORT d1 — /scan 과 같은 이유.
FUSED_QOS = QoSProfile(
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
        # 담당 모드 목록. 기본 [2] = launch search_behavior 기본 orbit(2026-11 경기규정: 탐색 = 원 선회,
        # ship_turn 담당). hold 옵션이면 launch 가 [2, 3] 을 넘긴다(그때 ship_turn 은 enabled=false).
        # 코드 기본값 = yaml(config/ship_back.yaml). 🚨 빈 리스트 금지.
        self.active_wp_modes = [
            int(m) for m in self.declare_parameter('active_wp_modes', [2]).value]
        self.period_sec = float(self.declare_parameter('period_sec', 0.1).value)

        self.keep_radius_m = float(self.declare_parameter('keep_radius_m', 5.0).value)   # 규정
        self.hold_time_sec = float(self.declare_parameter('hold_time_sec', 5.0).value)   # 규정
        # ⚠️ 실측(hold_dist_m). 코드 기본값 2.5 / 0.4 = hold_logic.DEFAULT_* = yaml(테스트가 묶음).
        #   기본값은 리터럴로 둔다 — launch_files T3/T4 가 ast 로 읽는다.
        raw_hold = self.declare_parameter('hold_dist_m', 2.5).value
        raw_tol = self.declare_parameter('dist_tol_m', 0.4).value
        # 시작 값 가드(계약서_옆비킴_현장조정_20261106 결정 ⑤): 잘못된 값 → ERROR + 두 값 기본값,
        #   유지 띠 바깥 ≥ keep_radius → WARN(값 유지), 기본 2.9 m 초과 → north 반경 확인 INFO.
        #   시작 때 한 번만(파라미터 콜백 없음). 심각도마다 호출 줄을 따로 둔다(§1-7).
        self.hold_dist_m, self.dist_tol_m, hold_issues = check_hold_params(
            raw_hold, raw_tol, self.keep_radius_m)
        for level, text in hold_issues:
            if level == 'ERROR':
                self.get_logger().error(text)
            elif level == 'WARN':
                self.get_logger().warn(text)
            else:
                self.get_logger().info(text)

        # SEARCH 시간 초과로 DONE 래치할지(§2-4). 기본 false = 계속 SEARCH(20000), 진행은 north 의
        #   WP 시간 초과. true 면 옛 동작 — search_timeout_sec 는 그때만 쓴다.
        self.search_timeout_ends = bool(
            self.declare_parameter('search_timeout_ends', False).value)
        self.search_timeout_sec = float(self.declare_parameter('search_timeout_sec', 20.0).value)
        self.lost_sec = float(self.declare_parameter('lost_sec', 2.0).value)
        # LiDAR 전방 최근접으로 획득(SEARCH→HOLD)할지(§2-5). 기본 false = 비전 색 확인으로만 획득,
        #   LiDAR 는 HOLD 중 직전 비전 방위가 lost_sec 안일 때 보조. true 면 옛 동작.
        self.lidar_acquire = bool(self.declare_parameter('lidar_acquire', False).value)
        # HOLD 에서 거리 'unknown'(LiDAR 가 부표를 못 봄)이 이 시간 이어지면 포기 → DONE(20000),
        #   ERROR 1회. 거리가 다시 잡히면 리셋. (계약 전체코스 §10-추가 결정 4)
        self.unknown_giveup_sec = float(
            self.declare_parameter('unknown_giveup_sec', 10.0).value)
        # 한 WP(담당 모드 진입)에서 HOLD 에 머문 시간 합이 이 값에 닿으면 포기 → DONE(20000),
        #   ERROR 1회. HOLD↔SEARCH 를 오가도 합산, 모드 진입 때만 0. (§10-추가 결정 5)
        #   재획득 정지 창(아래) 시간도 합에 든다, 창 끝 뒤 20000 구간은 안 듦. (결정 6 보충 ①)
        self.hold_max_sec = float(self.declare_parameter('hold_max_sec', 60.0).value)
        # HOLD 에서 놓쳐(lost_sec) SEARCH 로 갈 때 이 시간 동안 20000 대신 STOP_HOLD(제자리 재탐색).
        #   0 이면 끔(옛 동작). 첫 SEARCH(INIT·모드 진입 뒤)는 해당 없음. (§10-추가 결정 6)
        self.reacquire_stop_sec = float(
            self.declare_parameter('reacquire_stop_sec', 5.0).value)

        self.angle_stale_sec = float(self.declare_parameter('angle_stale_sec', 1.0).value)
        self.scan_stale_sec = float(self.declare_parameter('scan_stale_sec', 1.0).value)
        self.lidar_match_half_deg = float(self.declare_parameter('lidar_match_half_deg', 3.0).value)
        # HOLD 중 비전 방위가 잠깐 끊길 때 LiDAR 전방 섹터 최근접을 부표로 본다(lidar_acquire 참조)
        self.front_sector_half_deg = float(self.declare_parameter('front_sector_half_deg', 30.0).value)

        # 거리 출처(계약서_인식융합_20261112 §10). 코드 기본값 = yaml = 'fill'(대회값). 잘못된 값 → ERROR 후 lidar.
        #   lidar 면 /fused_objects 구독을 만들지 않는다(DDS 그래프까지 옛 동작). 구독보다 먼저 선언.
        self.distance_source, src_err = check_distance_source(
            self.declare_parameter('distance_source', 'fill').value)
        self.fused_topic = str(self.declare_parameter('fused_topic', '/fused_objects').value)
        self.fused_stale_sec = float(self.declare_parameter('fused_stale_sec', 0.5).value)   # ⚠️ 실측
        self.fused_match_deg = float(self.declare_parameter('fused_match_deg', 5.0).value)   # ⚠️ 실측
        self.fused_min_conf = float(self.declare_parameter('fused_min_conf', 0.5).value)     # ⚠️ 실측
        self.fused_allowed_ids, bad_src = fused_allowed_src_ids(
            self.declare_parameter('fused_allowed_sources', ['lidar', 'fused', 'depth']).value)
        if src_err is not None:
            self.get_logger().error(src_err)
        if bad_src:
            self.get_logger().error(
                f"fused_allowed_sources 에 모르는 출처 {bad_src} → 무시(lidar|depth|fused|row 만)")

        # ---- 발행/구독 (토픽 이름 불변, /image_distance 는 버림) ----
        self.pub_candidate = self.create_publisher(Float32, '/candidate_angle', qos)
        self.create_subscription(Int32, '/wp_mode', self.wp_cb, qos)
        self.create_subscription(Float32, '/image_angle', self.angle_cb, qos)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, SCAN_QOS)
        if self.distance_source != 'lidar':
            self.create_subscription(Float32MultiArray, self.fused_topic, self.fused_cb, FUSED_QOS)

        # ---- 상태 ----
        self.wp_mode = -1
        self.buoy_rel = None
        self.t_angle = None
        self.latest_scan = None
        self.t_scan = None
        self.fused_objs = None         # 마지막 정상 /fused_objects 물체 목록
        self.t_fused = None            # 그 수신 시각(monotonic) — stale 판정
        self.dist_src = None           # 마지막 거리 출처 이름('lidar'·'fused:depth' …) — 바뀔 때 INFO

        self.state = "INIT"
        self.state_t = None
        self.last_seen_t = None
        self.unknown_t0 = None         # HOLD 'unknown' 연속 시작 시각(None = 아님)
        self.search_warned = False     # 이번 SEARCH 에서 '못 찾음' WARN 을 냈나(진입마다 1회)
        self.hold_total = 0.0          # 이번 모드 진입 뒤 끝난 HOLD 구간·재획득 정지 창 합(s)
        self.hold_enter_t = None       # 지금 HOLD 구간 시작 시각(None = HOLD 아님)
        self.reacquire_t0 = None       # 재획득 정지 창 시작(HOLD→SEARCH lost) 시각, None = 창 없음
        self.hold_timer = HoldTimer(self.keep_radius_m, self.hold_time_sec)

        self.create_timer(self.period_sec, self.timer_cb)
        self.get_logger().info(
            f"ship_back 시작(이름 수정: 작년 'ship_turn' → 'ship_back'). 담당 모드 {self.active_wp_modes}. "
            f"위치유지: hold_dist={self.hold_dist_m}m(⚠️실측)/tol {self.dist_tol_m}, "
            f"규정 {self.keep_radius_m}m 이내 {self.hold_time_sec}s. "
            f"search_timeout_ends={self.search_timeout_ends}, lidar_acquire={self.lidar_acquire}, "
            f"unknown_giveup={self.unknown_giveup_sec}s, hold_max={self.hold_max_sec}s, "
            f"reacquire_stop={self.reacquire_stop_sec}s. "
            f"distance_source={self.distance_source}"
            + ("" if self.distance_source == 'lidar' else f"({self.fused_topic})") + "."
        )

    # ---------------------- 콜백 ----------------------
    def wp_cb(self, msg):
        with self.lock:
            prev = self.wp_mode
            self.wp_mode = int(msg.data)
            # 🚨 `prev not in modes` 로 쓰면 [2,3] 에서 2→3 전이에 리셋이 안 된다 (hold_logic 참조)
            if should_reset_on_mode(prev, self.wp_mode, self.active_wp_modes):
                self.state = "INIT"
                self.state_t = time.monotonic()
                self.last_seen_t = None
                self.unknown_t0 = None
                self.search_warned = False
                self.hold_total = 0.0          # HOLD 총시간은 담당 모드 진입 때만 0
                self.hold_enter_t = None
                self.reacquire_t0 = None       # 재획득 정지 창 닫음 — 첫 SEARCH 는 20000
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

    def fused_cb(self, msg):
        objs, err = parse_fused_objects(msg.data)
        if err is not None:
            # 형식 불일치 = 엉뚱한 칸을 거리로 읽을 위험(계약 위험 1) → 버리고 LiDAR. 5 s 주기.
            self.get_logger().error(f"{self.fused_topic} 형식 불일치({err}) → 무시",
                                    throttle_duration_sec=5.0)
            return
        with self.lock:
            self.fused_objs = objs
            self.t_fused = time.monotonic()

    # ---------------------- 보조 ----------------------
    def set_state(self, s):
        now = time.monotonic()
        if s != self.state:
            self.get_logger().info(f"⚓ 위치유지 {self.state} → {s}")
            if self.state == "HOLD" and self.hold_enter_t is not None:
                self.hold_total += max(0.0, now - self.hold_enter_t)   # HOLD 구간 합산
            # 재획득 정지 창 중 SEARCH 를 떠나면(재획득 HOLD·DONE) 창 시간도 합산(결정 6 보충 ①)
            self.hold_total += reacquire_window_elapsed(
                self.reacquire_t0, now, self.reacquire_stop_sec)
            self.hold_enter_t = now if s == "HOLD" else None
        self.state = s
        self.state_t = now
        self.unknown_t0 = None        # unknown 연속은 HOLD 한 번 안에서만 센다
        self.search_warned = False    # '못 찾음' WARN 은 SEARCH 진입마다 1회
        self.reacquire_t0 = None      # 재획득 정지 창은 HOLD lost 분기에서만 연다(set_state 뒤)

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

    def _fused_hit(self, rel_deg, now):
        """/fused_objects 에서 부표 방위(색 모름 → 방위만)에 맞는 물체 (거리, src_id). 없거나 stale 이면 None."""
        with self.lock:
            objs, t_rx = self.fused_objs, self.t_fused
        if not fused_fresh(t_rx, now, self.fused_stale_sec):
            self.get_logger().warn(
                f"distance_source={self.distance_source} 인데 {self.fused_topic} "
                + ("미수신" if t_rx is None else f"{now - t_rx:.1f}s 묵음") + " → LiDAR 거리만",
                throttle_duration_sec=5.0)
            return None
        return match_fused(objs, rel_deg, 0, self.fused_match_deg, self.fused_min_conf,
                           self.fused_allowed_ids)

    def _buoy_dist(self, rel_deg, now):
        """부표 방위의 거리 — distance_source 분기(hold_logic.choose_distance). lidar 면 _dist_at 그대로."""
        lidar_d = self._dist_at(rel_deg, now)
        if self.distance_source == 'lidar':
            return lidar_d
        need = self.distance_source == 'fused' or lidar_d is None
        hit = self._fused_hit(rel_deg, now) if need else None
        dist, label = choose_distance(self.distance_source, lidar_d, hit)
        if label is not None and label != self.dist_src:
            if self.dist_src is not None:
                self.get_logger().info(f"dist src {self.dist_src}→{label}", throttle_duration_sec=1.0)
            self.dist_src = label
        return dist

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
        """부표 (거리, 상대방위, 출처). 출처는 hold_logic.target_source(§2-5) — 모르면 전부 None.

        비전 = 그 방위의 LiDAR 거리. LiDAR 보조 = 전방 섹터 최근접(거리·방위).
        """
        with self.lock:
            fresh = (self.buoy_rel is not None and self.t_angle is not None
                     and (now - self.t_angle) <= self.angle_stale_sec)
            rel = self.buoy_rel if fresh else None
            since_seen = None if self.last_seen_t is None else now - self.last_seen_t
            state = self.state
        src = target_source(state, fresh, since_seen, self.lost_sec, self.lidar_acquire)
        if src == 'vision':
            return self._buoy_dist(rel, now), rel, src     # distance_source 분기(lidar 면 _dist_at 그대로)
        if src == 'lidar':
            d, r = self._front_nearest(now)
            return d, r, src
        return None, None, None

    def publish(self, value):
        self.pub_candidate.publish(Float32(data=float(value)))

    def publish_steer(self, rel_deg):
        """상대 조향(좌 양수)을 ±STEER_LIMIT_DEG 로 잘라 /candidate_angle 0~360 으로 발행.

        LiDAR 매칭(_dist_at)은 클램프 전 방위를 쓴다 — 여기서만 자른다.
        NaN 은 +80 이 되므로 상류(angle_cb)에서 걸러야 한다 — 가드 지우지 말 것.
        """
        if not (-STEER_LIMIT_DEG <= rel_deg <= STEER_LIMIT_DEG):
            self.get_logger().warn(
                f"위치유지 조향 {rel_deg:.1f}° 가 ±{STEER_LIMIT_DEG:.0f}° 밖 → 잘라서 발행",
                throttle_duration_sec=1.0)
        self.publish(rel_to_raw(clamp(rel_deg, -STEER_LIMIT_DEG, STEER_LIMIT_DEG)))

    # ---------------------- FSM ----------------------
    def timer_cb(self):
        with self.lock:
            if self.wp_mode not in self.active_wp_modes:
                return
            now = time.monotonic()
            if self.state_t is None:
                self.state_t = now

        dist, rel, src = self._target(now)
        # '봤다' = 비전 색 확인(lidar_acquire=false). LiDAR 보조는 last_seen 을 늘리지 않는다.
        seen = rel is not None and counts_as_seen(src, self.lidar_acquire)
        if seen:
            self.last_seen_t = now
        lost = self.last_seen_t is None or (now - self.last_seen_t) > self.lost_sec

        # ---- INIT: 진입 직후 손 떼(autonomous 가 부표 WP 로) ----
        if self.state == "INIT":
            self.publish(CANDIDATE_INVALID)
            if self.elapsed() >= 1.0:
                self.set_state("SEARCH")
            return

        # ---- SEARCH: 부표(모드별 색) 비전으로 잡힐 때까지 (autonomous 가 접근 중) ----
        if self.state == "SEARCH":
            # 재획득 정지 창 시간도 HOLD 총시간 합(결정 6 보충 ①) — 창 끝 틱까지 판정(창 길이로 자름)
            if self.reacquire_t0 is not None:
                win = reacquire_window_elapsed(self.reacquire_t0, now, self.reacquire_stop_sec)
                if hold_total_exceeded(self.hold_total, None, now, self.hold_max_sec, win):
                    self.get_logger().error(
                        f"HOLD 총 {self.hold_max_sec:g} s 넘음(성공 못 함, 재획득 정지 창 포함) "
                        f"→ 포기, DONE")
                    self.set_state("DONE")       # DONE 은 래치 — ERROR 는 이 전이에서 1회
                    return
            if seen:
                self.hold_timer.reset()
                self.set_state("HOLD")
                return
            # 기본(search_timeout_ends=false)은 래치 안 함 — 계속 20000, 진행은 north WP 시간 초과
            if search_timed_out(self.elapsed(), self.search_timeout_sec, self.search_timeout_ends):
                self.get_logger().warn("부표 못 찾음(SEARCH 타임아웃, search_timeout_ends) → 종료")
                self.set_state("DONE")
                return
            if search_timeout_warn(self.elapsed(), self.search_timeout_sec,
                                   self.search_timeout_ends, self.search_warned):
                self.get_logger().warn(
                    f"부표 못 찾음 {self.search_timeout_sec:g} s — 계속 탐색(20000), "
                    f"WP 시간 초과로 진행")
                self.search_warned = True
            # 재획득 정지 창(§10-추가 6): HOLD 에서 놓친 직후만 제자리 정지로 다시 찾는다
            if self.reacquire_t0 is not None:
                if reacquire_stop_active(self.reacquire_t0, now, self.reacquire_stop_sec):
                    self.publish(STOP_HOLD)
                    return
                self.get_logger().warn(
                    f"부표 재획득 실패 {self.reacquire_stop_sec:g} s → 탐색 계속(20000), WP 진행")
                # 창 시간(stop_sec 로 자름)만 합에 넣고 닫는다 — 이 뒤 20000 구간은 안 셈
                self.hold_total += reacquire_window_elapsed(
                    self.reacquire_t0, now, self.reacquire_stop_sec)
                self.reacquire_t0 = None         # 창 끝 WARN 은 1회
            self.publish(CANDIDATE_INVALID)      # 못 찾는 동안 autonomous 유지
            return

        # ---- HOLD: 능동 위치유지 ----
        if self.state == "HOLD":
            if lost:
                self.set_state("SEARCH")         # HOLD 합 마감·창 정리
                if self.reacquire_stop_sec > 0.0:
                    self.reacquire_t0 = now      # 재획득 정지 창 시작(§10-추가 6)
                    self.get_logger().warn(
                        f"부표 놓침 → {self.reacquire_stop_sec:g}s 정지 후 재탐색")
                return

            # 규정 판정: keep_radius 이내 연속 hold_time
            if self.hold_timer.satisfied(dist, now):
                self.get_logger().info(f"✅ 위치유지 {self.hold_time_sec}s 달성(규정 만족) → 완료")
                self.set_state("DONE")
                return

            # HOLD 총시간 상한(§10-추가 5): 성공 판정 뒤에 본다 — 같은 틱이면 성공이 우선
            if hold_total_exceeded(self.hold_total, self.hold_enter_t, now, self.hold_max_sec):
                self.get_logger().error(
                    f"HOLD 총 {self.hold_max_sec:g} s 넘음(성공 못 함) → 포기, DONE")
                self.set_state("DONE")           # DONE 은 래치 — ERROR 는 이 전이에서 1회
                return

            # bang-bang + 데드밴드. 부표를 향해 조향하며 전/후진.
            action = station_keep_action(dist, self.hold_dist_m, self.dist_tol_m)
            if action != 'unknown':
                self.unknown_t0 = None           # 거리가 다시 잡힘 → 리셋
            elif self.unknown_t0 is None:
                self.unknown_t0 = now
            if unknown_giveup(self.unknown_t0, now, self.unknown_giveup_sec):
                self.get_logger().error(
                    f"LiDAR 거리 없음 {now - self.unknown_t0:.1f}s → 포기, DONE")
                self.set_state("DONE")           # DONE 은 래치 — ERROR 는 이 전이에서 1회
                return
            if action == 'forward':
                self.publish_steer(rel)          # 부표 방향으로 전진 (±80 클램프)
            elif action == 'reverse':
                self.publish(REVERSE_REL)        # 후진 (조류로 붙었을 때 되돌림)
            elif action == 'hold':
                self.publish(STOP_HOLD)          # 데드밴드 안 → 중립 (벗어나면 위에서 되돌림)
            else:  # unknown: 거리를 모른다 → 부표를 향하되 함부로 전진하지 않음
                # (계약 위험 4) LiDAR 가 부표를 못 보면 여기서 정지 — unknown_giveup_sec 뒤 위에서 포기
                self.publish(STOP_HOLD)
            return

        # ---- DONE: 손 떼(autonomous 가 다음 WP) ----
        self.publish(CANDIDATE_INVALID)


def clamp_pm180(a):
    return (a + 180.0) % 360.0 - 180.0


def rel_to_raw(a):
    return (a + 360.0) % 360.0


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


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
