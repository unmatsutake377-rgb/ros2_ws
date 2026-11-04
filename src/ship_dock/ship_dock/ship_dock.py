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
      (TRANSIT →) INIT → SEARCH → ALIGN → APPROACH → CONTACT → BACKOFF → DONE
      ALIGN 은 **제자리 선회**로 표식을 정면에 놓는다 (2단계에서 spin_forward_pwm=1500 로
      진짜 제자리 선회가 됐기 때문에 가능해졌다. 작년 SPIN 은 순항속도 원이라 도크에 부딪혔다).
  [5] SPIN 규약: 5000=우선회, 6000=좌선회 (CLAUDE.md 3-9 상수표)
  [6] APPROACH 재정렬(→ALIGN) 전이에 거리 하한 realign_min_dist_m. 그 안에서는 제자리 선회 없이
      표식 방위로 조향하며 접근한다 (dock_sim 실패 3/500 이 전부 접안 직전 선회→관성→표식 유실).
      판단은 dock_fsm_logic.approach_decision (순수 함수, ROS 없이 테스트).
  [7] APPROACH 조향 발행만 ±STEER_LIMIT_DEG(80) 클램프 — 180.0(후진 신호)이 우연히 나가지 않게.
  [8] TRANSIT (계약서_전체코스_20261104 C4): mode 7 진입 첫 상태. 20000 을 내 GPS 주행에 맡긴다.
      /goal_distance 가 신선(goal_dist_stale_sec)하고 dock_engage_dist_m 미만이면 INIT.
      진입 후 GOAL_DIST_ENTRY_GUARD_SEC 지나 받은 값만 쓴다(직전 WP 거리 배제). 모르면 TRANSIT 유지.
      DONE 은 원인별(계약서 §10-추가·추가 결정 2):
        · 후진 이탈 완료(backoff_dist 도달) → 20000 → GPS 가 다음 WP(이탈점)로.
        · SEARCH 타임아웃 → WP8(칸 입구) 동안 STOP_HOLD, north 가 WP9(이탈점)로 넘어간 뒤 20000.
          (WP8 동안 20000 이면 칸 안으로 몰고 들어간다 — 카메라 끈 시뮬 20/20 칸막이 접촉)
        · 후진 타임아웃·원인 모름 → STOP_HOLD (이탈 거리 미확인 = 아직 도크 앞일 수 있다).
      WP8→WP9 는 7→7 이라 /wp_mode 로 안 보인다 → /wp_remaining_time 리셋으로 감지한다(wp_reset_detected).
      7→7(WP8→WP9)은 리셋하지 않는다 — 이탈점으로 가는 동안 도킹 상태 유지.

⚠️ 이 노드는 mode 7 에서 /candidate_angle 의 **유일한 발행자**다.
   (6a 에서 north_goal_angle 의 mode-7 폴백을 제거했다 — 안 그러면 발행자가 둘이라 조향이 튄다.)
   따라서 mode 7 동안 **매 틱 반드시 발행해야 한다.** 침묵하면 ship_direction 이 묵은 값을 붙든다.

프레임 규약 (코드에서 확인함, 추측 아님):
   /candidate_angle = 상대 방위(도). 0 = 정면. ship_direction 이 desired = 80 + rel 로 매핑한다.
   LiDAR 프레임도 80° = 정면 (ship_direction 이 두 프레임을 같게 다룬다).
   후진 = 상대 180 → desired 260 → motor_control 분기 (5).
     (근거: ship_direction (B) 정확 일치 예외 — test_candidate_reverse. 수신 0.5 s 넘게 끊기면 ship_direction 이 정지)
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

from ship_dock.dock_fsm_logic import (
    approach_decision, check_realign_min, CONTACT, ALIGN_STANDOFF, ALIGN)

# 특수 신호 상수 (CLAUDE.md 3-9 상수표)
SPIN_RIGHT = 5000.0          # 우선회
SPIN_LEFT = 6000.0           # 좌선회
CANDIDATE_INVALID = 20000.0  # 미션 없음
STOP_HOLD = 50000.0          # 정지/대기

REVERSE_REL = 180.0          # 상대 180° = 후진 (→ desired 260 → motor_control 분기 5)
                             #   근거: ship_direction (B) 정확 일치 예외(test_candidate_reverse)
IMAGE_INVALID = 10000.0      # /image_angle 의 '표식 없음' 센티널 (작년 규약 유지)

LIDAR_FORWARD_DEG = 80.0     # LiDAR 프레임에서 정면에 해당하는 각도
# ship_direction (B) 매핑: mapped = 80 + raw, 정상 범위 0~160 → 상대 조향 |steer| ≤ 80 만 조향이 된다.
# 180.0 정확 일치는 후진(260) 예외라, APPROACH 조향이 우연히 180 이면 후진한다 → 발행 직전 클램프.
# 튜닝값이 아니라 하류 프로토콜 상수(LIDAR_FORWARD_DEG 와 같은 성격) — 파라미터로 두지 않는다.
STEER_LIMIT_DEG = 80.0
# [8] mode 7 진입 후 이 시간 이상 지나서 받은 /goal_distance 만 쓴다. north 는 한 틱 안에서
# 거리(현재 WP) → /wp_mode 순으로 내므로 첫 mode 7 직전 거리는 이미 WP8 거리다. 이 guard 는
# 토픽 간 순서 뒤바뀜·지연(0.5~1.0 s)으로 직전 WP(도착해 ~0 m) 거리가 늦게 오는 경우를 막는 보수적 장치.
# 한계: Float32 에 시각이 없어 1.0 s 넘게 밀려 도착한 옛 메시지는 못 거른다(수신 시각 기준).
# 계약서 §2-3 고정값(north 주기 0.5 s × 2) — 파라미터 목록(§4)에 없어 상수로 둔다.
GOAL_DIST_ENTRY_GUARD_SEC = 1.0
# [8] north /wp_remaining_time(= 시간 초과 − WP 안 경과)은 한 WP 안에서 줄기만 한다(시계 기준, GPS 무관).
# 이 값보다 크게 늘면 north 가 다음 WP 로 넘어간 것(wp_enter 리셋). mode 7 안 최소 실제 증가 ≈ dwell 1.0 s
# (도착 틱 ≤ 150 − 1.0 → 다음 WP 150), 시간 초과면 0 → 150. Float32 반올림(150 근처 ~1e-5)보다 훨씬 크다.
WP_RESET_EPS_SEC = 0.25
# 수신 간격이 이보다 길면(north 주기 0.5 s 한 틱이라도 빠짐 — MANUAL→AUTO 복귀·north 정지 등) 기준을 버린다.
# north 는 AUTO 가 아니면 /wp_remaining_time 을 안 내고 WP 진입 시각을 지운다 → 복귀 때 150 이 다시 온다.
# 그걸 리셋으로 보면 WP8(칸 입구)인데 20000 이 나간다. AUTO 중 실제 전환은 0.5 s 간격이라 그대로 잡힌다.
WP_REMAIN_GAP_SEC = 0.8


def wp_reset_detected(prev_remain, new_remain, eps=WP_RESET_EPS_SEC):
    """[8] north 가 다음 WP 로 넘어갔나 — /wp_remaining_time 이 eps 보다 크게 늘었나. 순수 함수.

    prev_remain 이 None(기준 없음)이거나 값이 비유한이면 False(모르면 넘어간 걸로 안 본다).
    """
    if prev_remain is None or new_remain is None:
        return False
    if not (math.isfinite(prev_remain) and math.isfinite(new_remain)):
        return False
    return new_remain > prev_remain + eps


def transit_engage(goal_dist, goal_dist_t, mode_enter_t, now,
                   engage_dist, stale_sec, entry_guard_sec):
    """[8] TRANSIT 한 틱: 도킹 FSM(INIT)으로 넘어가도 되는가. 순수 함수(ROS 안 씀). → bool.

    goal_dist·goal_dist_t : 마지막 /goal_distance 값(m)·받은 시각. 없으면 None.
    mode_enter_t          : mode 7 에 들어온 시각. 없으면 None.
    모르면 False(TRANSIT 유지): 값·시각 없음, 비유한·음수, 진입 후 guard 전 수신, stale.
    경계: 수신 − 진입 == guard 는 사용, 지금 − 수신 == stale 은 신선, 거리 == engage 는 진입 안 함.
    """
    if goal_dist is None or goal_dist_t is None or mode_enter_t is None:
        return False
    if not math.isfinite(goal_dist) or goal_dist < 0.0:
        return False
    if (goal_dist_t - mode_enter_t) < entry_guard_sec:
        return False
    if (now - goal_dist_t) > stale_sec:
        return False
    return goal_dist < engage_dist


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
        # 재정렬 하한 [6]: 이 거리 안(dist < 하한)에서는 위 두 재정렬(standoff 1회·realign_tol)을 안 한다.
        #   🚨 대회 launch_files.launch.py 는 ship_dock 에 yaml 을 안 넘긴다 → **이 코드 기본값이 대회값**.
        #      config/ship_dock.yaml 과 같은 값으로 유지할 것. contact_dist_m 바꾸면 같이(contact + 0.5 권장).
        self.realign_min_dist_m = float(
            self.declare_parameter("realign_min_dist_m", 1.5).value)  # ⚠️ 실측 필요 — contact + 관성 여유(v·τ)
        floor_check = check_realign_min(
            self.realign_min_dist_m, self.contact_dist_m, self.standoff_dist_m)
        if floor_check == "below_contact":
            # 조용히 종전 동작(접안 직전 선회)으로 돌아간다 — 침묵 금지(CLAUDE.md 1-6). 부팅은 계속.
            self.get_logger().error(
                f"realign_min_dist_m={self.realign_min_dist_m} <= contact_dist_m={self.contact_dist_m}: "
                "재정렬 하한이 영구 무효다 (contact 판정이 먼저라 하한 안 구간이 없다) → "
                "접안 직전 제자리 선회→표식 유실 경로가 그대로 남는다. contact 보다 크게 잡아라.")
        elif floor_check == "above_standoff":
            self.get_logger().warn(
                f"realign_min_dist_m={self.realign_min_dist_m} >= standoff_dist_m={self.standoff_dist_m}: "
                "standoff 안이 전부 하한 안이라 standoff 1회 재정렬이 영영 안 걸린다.")
        # /image_angle 의 부호 규약: 양수가 좌현인가? ⚠️ 벤치 검증 필요 (카메라 좌표계)
        self.image_positive_is_left = bool(
            self.declare_parameter("image_positive_is_left", True).value)
        # SEARCH sweep 방향
        self.sweep_right = bool(self.declare_parameter("sweep_right", True).value)

        # 시간
        self.init_wait_sec = float(self.declare_parameter("init_wait_sec", 3.0).value)
        # 60 s: 일시적 놓침도 재탐색할 여유, mode 7 WP 시간 초과(150 s) 안 (계약서 §10-추가, 이전 25)
        self.search_timeout_sec = float(self.declare_parameter("search_timeout_sec", 60.0).value)
        self.align_timeout_sec = float(self.declare_parameter("align_timeout_sec", 15.0).value)
        self.approach_timeout_sec = float(self.declare_parameter("approach_timeout_sec", 30.0).value)
        self.contact_hold_sec = float(self.declare_parameter("contact_hold_sec", 3.0).value)  # ⚠️ 규정 확인
        self.backoff_timeout_sec = float(self.declare_parameter("backoff_timeout_sec", 8.0).value)

        # 신선도 (묵은 데이터로 도킹하지 않는다)
        self.image_stale_sec = float(self.declare_parameter("image_stale_sec", 1.0).value)
        self.scan_stale_sec = float(self.declare_parameter("scan_stale_sec", 1.0).value)

        # [8] TRANSIT → INIT 진입 (GPS 거리 /goal_distance 기준). 대회 launch 는 yaml 을 안 넘긴다
        #     → 이 코드 기본값이 대회값. config/ship_dock.yaml 과 같게 유지.
        self.dock_engage_dist_m = float(
            self.declare_parameter("dock_engage_dist_m", 12.0).value)   # ⚠️ 실측 필요
        self.goal_dist_stale_sec = float(
            self.declare_parameter("goal_dist_stale_sec", 2.0).value)
        if not math.isfinite(self.dock_engage_dist_m) or self.dock_engage_dist_m <= 0.0:
            self.get_logger().warn(
                f"dock_engage_dist_m={self.dock_engage_dist_m} 이상 → 기본값 12.0 m 사용")
            self.dock_engage_dist_m = 12.0
        if not math.isfinite(self.goal_dist_stale_sec) or self.goal_dist_stale_sec <= 0.0:
            self.get_logger().warn(
                f"goal_dist_stale_sec={self.goal_dist_stale_sec} 이상 → 기본값 2.0초 사용")
            self.goal_dist_stale_sec = 2.0

        # ---- 구독/발행 (토픽 이름 불변, CLAUDE.md 1-3) ----
        self.pub_candidate = self.create_publisher(Float32, "/candidate_angle", qos)

        self.create_subscription(Int32, "/wp_mode", self.wp_mode_cb, qos)
        self.create_subscription(Float32, "/image_angle", self.image_angle_cb, qos)
        self.create_subscription(LaserScan, "/scan", self.scan_cb, SCAN_QOS)
        # [8] north_goal_angle 의 현재 WP 까지 거리(m). TRANSIT → INIT 판단에만 쓴다.
        self.create_subscription(Float32, "/goal_distance", self.goal_distance_cb, qos)
        # [8] north 의 현재 WP 남은 시간. 리셋(증가) = WP8→WP9 전환 감지에만 쓴다(7→7 이라 /wp_mode 로 안 보임).
        self.create_subscription(Float32, "/wp_remaining_time", self.wp_remaining_cb, qos)
        # ※ /image_distance 는 **구독하지 않는다**. 카메라 뎁스라 OAK-1 에서 죽는다. [2]

        # ---- 상태 ----
        self.wp_mode = -1
        self.image_angle = IMAGE_INVALID
        self.last_image_t = None
        self.latest_scan = None
        self.last_scan_t = None

        self.goal_dist = None
        self.goal_dist_t = None
        self.mode_enter_t = None        # mode 7 에 들어온 시각 [8]

        self.state = "TRANSIT"
        self.state_t = None
        self.standoff_aligned = False   # standoff 에서 정밀 재정렬을 이미 했는가 (1회만)
        self.done_reason = None         # DONE 원인: 'backoff' | 'search_timeout' | 'backoff_timeout'
        self.remain_prev = None         # 진입 guard 뒤 마지막 /wp_remaining_time (WP 전환 감지 기준)
        self.remain_t = None            # 그 값을 받은 시각 (간격 > WP_REMAIN_GAP_SEC 면 기준 버림)
        self.wp_resets = 0              # mode 7 진입 뒤 감지한 WP 전환 수 (1 이상 = WP8 지남)
        self.wp_reset_logged = False

        self.create_timer(self.period_sec, self.timer_cb)

        self.get_logger().info(
            f"ship_dock 시작: active_wp_mode={self.active_wp_mode} (작년엔 9 라 1년간 침묵했다).\n"
            f"   거리는 LiDAR 전방 ±{self.front_sector_half_deg}° 최소거리만 사용 (카메라 뎁스 안 씀).\n"
            f"   standoff={self.standoff_dist_m}m contact={self.contact_dist_m}m "
            f"realign_min={self.realign_min_dist_m}m (이 안에서는 재정렬 선회 없음)\n"
            f"   mode 7 진입 = TRANSIT(20000, GPS 주행) → /goal_distance < "
            f"{self.dock_engage_dist_m}m(신선 {self.goal_dist_stale_sec}s) 이면 INIT"
        )

    # ───────────────────────── 콜백 ─────────────────────────
    def wp_mode_cb(self, msg):
        with self.lock:
            mode = int(msg.data)
            if mode != self.wp_mode and mode != self.active_wp_mode:
                # 내 차례가 아니다 → FSM 초기화 (다음 진입은 TRANSIT 부터)
                self.state = "TRANSIT"
                self.state_t = None
                self.standoff_aligned = False
                self.mode_enter_t = None
                self.done_reason = None
                self._reset_wp_track()
            elif mode == self.active_wp_mode and self.wp_mode != self.active_wp_mode:
                # 다른 모드 → 7 진입. 7→7(WP8→WP9)은 여기 안 온다 = 리셋 안 함 [8]
                self.mode_enter_t = time.monotonic()
                self._reset_wp_track()
            self.wp_mode = mode

    def image_angle_cb(self, msg):
        with self.lock:
            self.image_angle = float(msg.data)
            self.last_image_t = time.monotonic()

    def scan_cb(self, msg):
        with self.lock:
            self.latest_scan = msg
            self.last_scan_t = time.monotonic()

    def _reset_wp_track(self):
        self.remain_prev = None
        self.remain_t = None
        self.wp_resets = 0
        self.wp_reset_logged = False

    def wp_remaining_cb(self, msg):
        """[8] mode 7 진입 guard 뒤 값만 본다 — 직전 모드 WP 의 늦게 온 값이 기준이 되지 않게."""
        with self.lock:
            now = time.monotonic()
            if self.wp_mode != self.active_wp_mode or self.mode_enter_t is None:
                return
            if (now - self.mode_enter_t) < GOAL_DIST_ENTRY_GUARD_SEC:
                return
            remain = float(msg.data)
            if not math.isfinite(remain):
                return
            if self.remain_t is not None and (now - self.remain_t) > WP_REMAIN_GAP_SEC:
                self.get_logger().warn(
                    f"/wp_remaining_time {now - self.remain_t:.1f}s 끊김 → WP 전환 기준 버림 "
                    f"(복귀 첫 값 {remain:.1f} 은 전환으로 안 봄)")
                self.remain_prev = None
            if wp_reset_detected(self.remain_prev, remain):
                self.get_logger().info(
                    f"north WP 전환 감지: /wp_remaining_time {self.remain_prev:.1f} → {remain:.1f}")
                self.wp_resets += 1
            self.remain_prev = remain
            self.remain_t = now

    def goal_distance_cb(self, msg):
        with self.lock:
            self.goal_dist = float(msg.data)
            self.goal_dist_t = time.monotonic()

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

    def publish_steer(self, rel_deg):
        """[7] 상대 조향(부호 그대로, rel_to_raw 없음)을 ±STEER_LIMIT_DEG 로 잘라 발행.

        NaN 은 +80 이 되므로 상류(marker_ok)에서 걸러야 한다 — 가드 지우지 말 것.
        """
        if not (-STEER_LIMIT_DEG <= rel_deg <= STEER_LIMIT_DEG):
            self.get_logger().warn(
                f"도킹 조향 {rel_deg:.1f}° 가 ±{STEER_LIMIT_DEG:.0f}° 밖 → 잘라서 발행",
                throttle_duration_sec=1.0)
        self.publish(max(-STEER_LIMIT_DEG, min(STEER_LIMIT_DEG, rel_deg)))

    # ───────────────────────── FSM ─────────────────────────
    def timer_cb(self):
        with self.lock:
            if self.wp_mode != self.active_wp_mode:
                return                       # 내 차례가 아니다 → 발행하지 않는다

            now = time.monotonic()
            if self.state_t is None:
                self.state_t = now
            if self.mode_enter_t is None:
                self.mode_enter_t = now      # 진입 콜백을 못 거친 경우(첫 틱 = 진입)

            dist = self.front_min_dist(now)
            has_marker = self.marker_ok(now)
            angle = self.image_angle if has_marker else 0.0

            # 거리를 모르면 접근하지 않는다. 눈 감고 도킹하지 않는다.
            if dist is None and self.state in ("APPROACH", "CONTACT", "BACKOFF"):
                self.get_logger().warn(
                    "LiDAR 거리 없음 → 도킹 접근 중단(정지)", throttle_duration_sec=2.0)
                self.publish(STOP_HOLD)
                return

            # ---- TRANSIT [8]: GPS 주행(20000)으로 WP8 쪽으로. 가까워지면 INIT. 모르면 유지 ----
            if self.state == "TRANSIT":
                if not transit_engage(
                        self.goal_dist, self.goal_dist_t, self.mode_enter_t, now,
                        self.dock_engage_dist_m, self.goal_dist_stale_sec,
                        GOAL_DIST_ENTRY_GUARD_SEC):
                    if (now - self.mode_enter_t) > (
                            GOAL_DIST_ENTRY_GUARD_SEC + self.goal_dist_stale_sec) and (
                            self.goal_dist_t is None
                            or (now - self.goal_dist_t) > self.goal_dist_stale_sec
                            or not math.isfinite(self.goal_dist) or self.goal_dist < 0.0):
                        self.get_logger().warn(
                            f"/goal_distance 없음·stale·비정상({self.goal_dist}) → "
                            "TRANSIT 유지(20000, GPS 주행)",
                            throttle_duration_sec=5.0)
                    self.publish(CANDIDATE_INVALID)
                    return
                self.get_logger().info(
                    f"🛳 /goal_distance {self.goal_dist:.1f} m < "
                    f"{self.dock_engage_dist_m} m → 도킹 시작")
                self.set_state("INIT")           # 아래 INIT 이 이 틱에 STOP_HOLD 를 낸다

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
                    self.get_logger().error(
                        "도킹 표식을 못 찾음(SEARCH 타임아웃) → DONE — WP8(칸 입구) 동안 정지, "
                        "WP9 전환 감지 시 20000")
                    self.done_reason = "search_timeout"
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

                # 접안 / standoff 1회 정밀 재정렬 / 크게 틀어짐 재정렬 / 조향 접근 — 판단은 순수 함수 [6]
                #   contact 가 맨 앞. realign_min 안에서는 두 재정렬 다 건너뛰고 조향 접근(STEER).
                decision, self.standoff_aligned = approach_decision(
                    dist, angle, self.standoff_aligned,
                    self.contact_dist_m, self.standoff_dist_m, self.realign_min_dist_m,
                    self.align_tol_deg, self.realign_tol_deg)
                if decision == CONTACT:
                    self.set_state("CONTACT")
                    return
                if decision in (ALIGN_STANDOFF, ALIGN):
                    self.set_state("ALIGN")     # 제자리 선회로 다시 정면에 놓는다 (비스듬히 밀고 들어가지 않는다)
                    return

                self.publish_steer(angle)    # 표식 방위로 조향하며 전진 (하한 안 경로도 여기 — 매 틱 발행 유지)
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
                    self.get_logger().info(
                        f"후진 이탈 완료(전방 {dist:.1f} m) → DONE(20000, GPS 가 다음 WP 로)")
                    self.done_reason = "backoff"
                    self.set_state("DONE")
                    return
                if self.elapsed() > self.backoff_timeout_sec:
                    # 이탈 거리를 확인 못 함 → 아직 도크 앞일 수 있다. 모르면 GPS 에 넘기지 않는다.
                    self.get_logger().error(
                        f"후진 타임아웃({self.backoff_timeout_sec}s, 전방 "
                        f"{'없음' if dist is None else f'{dist:.1f} m'}) → DONE(정지 50000 유지)")
                    self.done_reason = "backoff_timeout"
                    self.set_state("DONE")
                    return
                self.publish(REVERSE_REL)    # 상대 180° = 후진
                return

            # ---- DONE [8]: 원인별. mode 7 동안 매 틱 발행. 7→7 은 리셋 안 함 ----
            if self.done_reason == "backoff":
                self.publish(CANDIDATE_INVALID)          # 후진 이탈 완료 → GPS 로 이탈점
                return
            if self.done_reason == "search_timeout" and self.wp_resets > 0:
                if not self.wp_reset_logged:
                    self.get_logger().info(
                        "north WP 전환 감지(/wp_remaining_time 리셋) → 칸 입구 지남, "
                        "DONE 20000(GPS 로 이탈점)")
                    self.wp_reset_logged = True
                self.publish(CANDIDATE_INVALID)
                return
            if self.done_reason == "search_timeout" and self.remain_prev is None and (
                    now - self.mode_enter_t) > (
                    GOAL_DIST_ENTRY_GUARD_SEC + self.goal_dist_stale_sec):
                self.get_logger().warn(
                    "/wp_remaining_time 없음 → WP 전환 감지 불가, DONE 정지 유지",
                    throttle_duration_sec=5.0)
            self.publish(STOP_HOLD)                      # WP8 동안·후진 타임아웃·원인 모름


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
