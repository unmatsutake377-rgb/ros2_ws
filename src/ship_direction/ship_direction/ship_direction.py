import math
import time
from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int32, Float32, Float32MultiArray
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# 판정 로직은 ROS 비의존 모듈에 있다 (ROS 없이 단위 테스트 가능).
# 재수출: from ship_direction.ship_direction import SensorWatch 로도 접근 가능.
from ship_direction.failsafe import SensorWatch, TemporalVote, median_min

# 특수 신호 상수 (CLAUDE.md 3-9 상수표)
SPIN_RIGHT = 5000.0          # 우선회
SPIN_LEFT = 6000.0           # 좌선회
CANDIDATE_INVALID = 20000.0  # 미션 없음 → yaw_error 기반 자율 회피
STOP_HOLD = 50000.0          # 정지/대기

REVERSE_ANGLE = 260.0        # 회피 경로 없음 → 후진
LIDAR_FORWARD_DEG = 80.0     # LiDAR 프레임에서 정면 (상대방위 0 에 대응)



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

class ShipDirection(Node):
    """LiDAR 회피 + 페일세이프. /desired_angle · /obstacle_distance_array · /failsafe_level 발행.

    3단계에서 고친 것:

    [A] 제어 루프를 고정 주기 타이머로 분리했다.
        작년: 모든 계산·발행이 scan_callback 안에 있었다 → **LiDAR 가 끊기면 콜백이 아예 안 와서
              아무도 /desired_angle 을 발행하지 않았다.** (측정: LiDAR 1~3초 끊김 → 발행 0회, 완전 침묵.
              페일세이프 레벨은 올라가는데 출력이 없으니 L1/L2 가 '존재'하지 않았다.)
        지금: scan_cb 는 **스캔 저장만**. control_cb 가 고정 주기로 **항상** 계산·발행한다.
              → ship_direction 이 살아있는 한 /desired_angle 은 흐른다.
                침묵 = ship_direction 사망 → motor_control 이 cmd_timeout 으로 잡는다(0.5s 로 조일 수 있게 됨).

    [B] 페일세이프(독립 watchdog 타이머). scan 이 묵은 정도로 레벨을 매긴다. **3상태·2임계.**
        0 정상        : 계산한 각도        / 속도 100%
        1 경고(0.7s)  : 계산한 각도 그대로 / 속도 70%    ← 각도 안 건드림
        2 정지(3.0s)  : STOP_HOLD override / 정지        ← 각도를 덮어쓰는 건 여기뿐

        ★ 임계가 적을수록 틀릴 곳이 적다. 중간에 'freeze' 레벨을 뒀다가 뺐다 —
          새 스캔이 없는데 재계산하면 같은 스캔으로 같은 답이 나온다.
          즉 '재계산'과 'freeze'는 애초에 같은 동작이라 레벨을 나눌 이유가 없었다.
        ★ 경고에서 '직진 강제' 같은 걸 하면 회피 기동 중간에 핸들을 놓는 셈이다.
          부표를 피하던 중 직진하면 그대로 박는다. 0.7초 묵었다고 조향을 포기하는 건 과잉반응이다.
          묵은 데이터라도 없는 것보다 낫다 — 대신 속도를 줄여 오차를 줄인다.

        속도는 /desired_angle(각도)에 담을 수 없다 → motor_control 이 /failsafe_level 을 구독해
        속도 상한을 건다. (레벨 2 는 STOP_HOLD 가 각도로 오므로 motor_control 분기 (1) 이 처리)

        오탐 방지(CLAUDE.md §5): ARMED(스캔 한 번은 받아야 평가 시작) + 올릴 땐 연속 N회 확인 +
        내릴 땐 즉시(자동 복구) + time.monotonic().

    [C] rear_obstacle_ignore_margin 제거 — 튜닝값이 아니라 증명된 결함이다.
        물보라 반사 0.3m 하나가 front_min_distance 를 끌어내려, 실제 1.6m 부표를 통째로
        마스크에서 지웠다 (17셀 → 0셀).

    [D] time.time() → time.monotonic() 전부 교체.

    [E] 워치독이 /scan 과 /yaw_error 를 **둘 다** 감시한다 (센서별 dict, 가장 나쁜 센서가 레벨을 정함).
        /scan 만 보면 IMU 사망을 놓친다. 1단계에서 만든 사슬은
          IMU 사망 → ship_goal_angle 이 /yaw_error 를 끊는다 → ship_direction 이 감지해 정지
        인데, 세 번째 화살표(침묵을 들을 귀)가 없으면 마지막 yaw_error 를 영원히 붙들고 조향한다.
        (측정: IMU 사망 시 정지율 0%, 접촉 3회, 14.8m 계속 주행)
        센서별로 첫 데이터를 받은 뒤에만 감시(ARMED) — 부팅 오발동 방지.

    [F] angle_increment == 0 가드. min_index/max_index 계산에서 이 값으로 나눈다 →
        LiDAR 가 이상 스캔을 보내면 ZeroDivisionError 로 노드가 통째로 죽는다
        (작년 코드에도 방어가 없었다). 이상 스캔은 scan_callback 에서 버리고 즉시 정지.
        하드 폴트라 확인 N회 없이 바로 레벨 2. 정상 스캔이 오면 스스로 풀린다.

    [G] 시간 투표 필터(TemporalVote) — **기본 OFF(temporal_frames=1)**. 코드는 남겨둔다.
        최근 N프레임 중 votes 회 이상 나타난 셀만 장애물로 인정하는 필터다.
        처음엔 효과가 있어 보였으나(단일 시드), **시드 20개로 재보니 효과가 없었다:**
          필터 OFF → 접촉 19/20, 전진 25.5m  /  필터 ON → 접촉 20/20, 전진 23.4m
        해당 시나리오(정면 부표 + 물보라 60%)는 어느 코드든 95% 접촉하는 '통과 불가' 판이었다.
        → **무해하지만 무익.** (TTC 는 '해로워서' 삭제했지만, 이건 '무익해서' 기본 OFF다.)
        실제 물보라의 시간 특성이 시뮬과 다를 수 있으니 켤 수 있게 남긴다: frames=3, votes=2.
        ★ 켤 때는 반드시 dilate 전, 원본 마스크에 건다
          (팽창 후에 걸면 진짜 부표의 팽창 영역까지 표가 갈려 부표가 얇아진다).

    [H] Geofence (6a-2). 경기장 밖으로 나가면 실격인데 방어가 0 이었다.
        north 가 경계를 **'가짜 LiDAR'** 로 쏴서 보낸다:
          /geofence_state = [angle_min_deg, angle_inc_deg, r0, r1, ...] (상대방위, 멀면 inf)
        여기서 하는 일은 **스캔 병합 한 줄**이 전부다:
          ranges[i] = min(real_ranges[i], geofence_ranges[i])
        그 뒤는 기존 파이프라인이 알아서 한다 — detection_distance 로 마스크가 되고,
        dilate 가 배 폭 여유를 더하고, 갭-팔로잉이 피한다.
        **특수 로직도, 새 상태기계도, 튜닝 파라미터(half_block)도 없다.**

        ★ 왜 '원뿔 칠하기'를 버렸나: **벽은 '점'이 아니라 '선'이다.**
          최근접점 한 방향으로 ±각도를 막는 건 기하학적으로 틀렸다. 40×40 모서리에서
          배가 대각선(45°)을 보면 북벽 최근접점 -45°, 동벽 +45° 인데 **탈출구(모서리)는 정면 0°**.
          half_block=40° 면 [-85,-5]·[5,85] 만 막혀 **정면 0° 가 '틈'으로 열린다** → 대각선 탈출, 실격.
          광선으로 쏘면 정면 경계 거리(2.12m)가 그대로 벽이 된다. 경기장이 사각형이 아니어도 맞다.

        ★ 빈 배열/stale 이면 병합하지 않는다 — 묵은 '없는 벽'은 배를 가둔다.

    [I] Geofence 하드 가드 — (A)SPIN·(B)candidate 구간 전용 (6a-2 보강).
        [H]의 스캔 병합은 (C) 자율회피에서만 일어난다. 그런데 **도킹은 SPIN+candidate 로 조향**하고
        이 구간은 distance_array 를 안 만들어 병합할 대상이 없다. 도크가 경기장 가장자리면
        접안하다 경계로 파고들어 이탈 → 실격. (측정으로 확인된 실제 위험.)
        → _compute 맨 앞에서 검사: 경계 최소거리가 geofence_stop_margin_m(2.0) 안이고 그 상대방위가
          '전진하려는 방향 ±geofence_stop_cone_deg(60°)' 와 겹치면 → candidate/SPIN 무시하고 STOP_HOLD.
        **미션보다 실격 방지가 우선.** 경계를 등지고 멈춘다.
        ⚠️ north 광선은 전방 ±80° 만 → 후진 중 뒤쪽 경계는 못 본다(접안=전방 방어용).

    [J] 감속 신호용 공간 median 필터 (obst_median_kernel, **기본 ON**).
        _closest_obstacle 이 필터 없는 raw min 이라 **물보라 반사 한 점(0.3m)이 그대로
        motor_control 감속을 물렸다.** 이웃과 어긋나는 고립 스파이크를 range 값 수준에서 지운다.
        측정(시드 20, 물보라 30%): raw-min 가짜감속 **36.8% → median 8.9%**(기준선). 접촉은 전 조건 0.
        ★ **감속 신호에만** 건다 — _compute 의 회피 마스크는 안 건드린다(그래서 접촉이 안 변한다).
        ★ [G] TemporalVote(시간 투표)와 다르다: 저건 효과 0 이라 OFF, 이건 명확한 이득이라 ON.
          시간 투표는 프레임 간, 이건 한 프레임 안의 공간.
        ⚠️ 시뮬 물보라는 '단일점' 모델이다. 실제 물보라가 작은 군집이면 커널5로 부족할 수 있다
           → 배 뜬 뒤 블랙박스의 obstacle_min_dist 로 실측 확인하고 커널을 7/9 로 올릴 것.

    ※ 판정 로직(SensorWatch, TemporalVote, median_min)은 **ROS 비의존 모듈** ship_direction/failsafe.py 에 있다.
      워치독을 노드 안에 인라인하면 단위 테스트가 불가능해진다. rclpy 없이 임포트 가능:
          from ship_direction.failsafe import SensorWatch, TemporalVote   # ← ROS 불필요(권장)
          from ship_direction.ship_direction import SensorWatch           # ← 재수출(ROS 필요)
    """

    def __init__(self):
        super().__init__('ship_direction')

        self.lock = Lock()

        # ----------------------
        #   파라미터 (config yaml, CLAUDE.md 1-4)
        # ----------------------
        self.control_period = float(self.declare_parameter('control_period_sec', 0.1).value)    # 10Hz
        self.watchdog_period = float(self.declare_parameter('watchdog_period_sec', 0.1).value)

        # 페일세이프 임계(센서가 묵은 시간). 3상태·2임계 (CLAUDE.md §5)
        self.fs_warn_sec = float(self.declare_parameter('failsafe_warn_sec', 0.7).value)   # → 레벨 1
        self.fs_stop_sec = float(self.declare_parameter('failsafe_stop_sec', 3.0).value)   # → 레벨 2
        self.fs_confirm_n = int(self.declare_parameter('failsafe_confirm_n', 3).value)

        # ---- Geofence (경계 이탈 = 실격). 6a-2 ----
        # north 가 경계를 '가짜 LiDAR'로 쏴서 보낸다 → 실제 스캔과 min() 병합만 하면 끝이다.
        # 특수 로직도, 튜닝 파라미터(half_block)도 없다. 아래 [H] 참고.
        # 🚨 묵으면 병합하지 않는다. 묵은 '없는 벽'은 배를 가둔다. (north 주기 0.5s 의 4배)
        self.geofence_stale_sec = float(self.declare_parameter('geofence_stale_sec', 2.0).value)
        # 하드 가드([I]): 경계가 이 거리 안이고 전진 방향과 겹치면 미션 무시하고 정지.
        # (A)SPIN/(B)candidate 구간은 마스크를 안 만들어 병합이 안 되므로, 도킹이 경계로 파고들면
        # 이탈한다. 미션보다 실격 방지가 우선 → 경계를 등지고 멈춘다.
        self.geofence_stop_margin_m = float(
            self.declare_parameter('geofence_stop_margin_m', 2.0).value)
        self.geofence_stop_cone_deg = float(
            self.declare_parameter('geofence_stop_cone_deg', 60.0).value)

        # ---- [J] 감속 신호용 공간 median 필터. **기본 ON.** ----
        # _closest_obstacle(감속 신호)에만 건다. 회피 마스크(_compute)는 안 건드린다.
        # 측정(시드 20, 물보라 30%): raw-min 가짜감속 36.8% → median 8.9%. 접촉 전 조건 0.
        # 0 이면 raw-min 폴백.
        self.obst_median_kernel = int(self.declare_parameter('obst_median_kernel', 5).value)

        # 시간 투표 필터 — **기본 OFF(frames=1)**. 시드 20개로 재보니 효과가 없었다(아래 [G]).
        # 무해하지만 무익하다. 실제 물보라의 시간 특성이 시뮬과 다를 수 있으니 코드는 남겨두고
        # 켤 수 있게만 해둔다: temporal_frames=3, temporal_votes=2 로 켠다.
        self.temporal_frames = int(self.declare_parameter('temporal_frames', 1).value)
        self.temporal_votes = int(self.declare_parameter('temporal_votes', 2).value)

        # 회피 튜닝 — §6 시뮬 스윕으로 검증된 값. 임의로 바꾸지 말 것.
        self.detection_distance_default = float(
            self.declare_parameter('detection_distance_default', 3.0).value)   # 1.8 이면 8/8 충돌
        self.detection_distance_gate = float(
            self.declare_parameter('detection_distance_gate', 2.0).value)      # 3.0 이면 게이트가 좁아 접촉 증가
        # 게이트 구간의 wp_mode. 확정된 웨이포인트 표(3-6): mode 0=게이트 시작, 1=게이트 끝.
        # (작년 코드는 wp_mode==2 (위치유지)를 키로 썼는데, 그건 게이트가 아니다.)
        self.gate_wp_modes = list(self.declare_parameter('gate_wp_modes', [0, 1]).value)

        self.half_width = float(self.declare_parameter('half_width', 0.45).value)
        # 0.20→접촉 1.7회 / 0.25→0.2회 / 0.30→0.8회 / 0.45→807초 폭주(과보수)
        self.clearance = float(self.declare_parameter('clearance', 0.25).value)
        self.border_margin = int(self.declare_parameter('border_margin', 2).value)
        self.max_spike_ratio = float(self.declare_parameter('max_spike_ratio', 0.01).value)
        # ※ rear_obstacle_ignore_margin 은 제거했다 (파라미터 자체를 없앰). 위 [C] 참고.
        self.min_obstacle_cells = int(
            self.declare_parameter('min_obstacle_cells', 1).value)   # 3 이면 작은 부표가 무시된다
        self.reverse_cooldown = float(self.declare_parameter('reverse_cooldown_sec', 3.0).value)

        self.detection_distance = self.detection_distance_default

        # ----------------------
        #      Subscriptions
        # ----------------------
        self.create_subscription(LaserScan, '/scan', self.scan_callback, SCAN_QOS)
        self.create_subscription(Float32, '/yaw_error', self.yaw_error_callback, 10)
        self.create_subscription(Float32, '/candidate_angle', self.candidate_callback, 10)
        self.create_subscription(Int32, '/wp_mode', self.wp_mode_cb, 10)
        self.create_subscription(Float32MultiArray, '/geofence_state', self.geofence_cb, 10)

        # ----------------------
        #       Publishers  (토픽/타입 불변 + /failsafe_level 신규, CLAUDE.md 3-9)
        # ----------------------
        self.pub_desired_angle = self.create_publisher(Float32, '/desired_angle', 10)
        self.pub_obstacle_distance = self.create_publisher(Float32MultiArray, '/obstacle_distance_array', 10)
        self.pub_failsafe = self.create_publisher(Int32, '/failsafe_level', 10)

        # ----------------------
        #       State
        # ----------------------
        self.wp_mode = -1
        self.wp4_enter_time = None
        self.yaw_error = 0.0
        self.candidate_angle = CANDIDATE_INVALID

        self.latest_scan = None       # scan_cb 는 저장만 한다
        self.last_reverse_time = time.monotonic()

        # geofence: (거리 m, 상대방위 deg) 또는 None. north 가 [inf,nan] 을 내면 None.
        self.geofence = None
        self.last_geofence_t = None

        # 페일세이프 감시 — /scan 과 /yaw_error 를 둘 다 본다.
        # ★ /scan 만 보면 IMU 사망을 놓친다. IMU 가 죽으면 ship_goal_angle 이 /yaw_error 를
        #   끊어주는데(1단계 설계), 그 침묵을 들을 귀가 없으면 마지막 yaw_error 로 영원히 조향한다.
        self.watch = SensorWatch(
            ['scan', 'yaw_error'],
            warn_sec=self.fs_warn_sec,
            stop_sec=self.fs_stop_sec,
            confirm_n=self.fs_confirm_n,
        )
        self.failsafe_level = 0

        # 시간 투표 필터 (dilate 전, 원본 마스크에 적용)
        self.temporal = TemporalVote(self.temporal_frames, self.temporal_votes)

        # ----------------------
        #  타이머: 제어 / 워치독 (서로 독립)
        # ----------------------
        self.control_timer = self.create_timer(self.control_period, self.control_cb)
        self.watchdog_timer = self.create_timer(self.watchdog_period, self.watchdog_cb)

        self.get_logger().info(
            f"ship_direction 시작: 제어 {1.0 / self.control_period:.0f}Hz (스캔 유무 무관 항상 발행), "
            f"워치독 {1.0 / self.watchdog_period:.0f}Hz.\n"
            f"   페일세이프 경고={self.fs_warn_sec}s(레벨1) 정지={self.fs_stop_sec}s(레벨2), "
            f"확인 {self.fs_confirm_n}회. 각도 override 는 레벨2 뿐.\n"
            f"   시간투표 {self.temporal_frames}프레임 중 {self.temporal_votes}표 "
            f"({'ON' if self.temporal.enabled else 'OFF'}), 감시 센서: scan + yaw_error."
        )

    # =====================================================
    # CALLBACKS — 저장만 한다
    # =====================================================
    def scan_callback(self, msg: LaserScan):
        """저장만. 계산·발행은 control_cb 가 고정 주기로 한다.
        (작년엔 여기서 다 했다 → LiDAR 가 죽으면 콜백이 안 와서 아무도 발행하지 않았다)

        [F] angle_increment 가드: 0/음수/NaN 이면 min_index 계산에서 ZeroDivisionError 로
        노드가 통째로 죽는다(작년 코드에도 방어가 없었다). 이상 스캔은 문 앞에서 버리고
        bad_scan 을 세워 즉시 정지시킨다. 정상 스캔이 오면 스스로 풀린다."""
        inc = math.degrees(msg.angle_increment)
        if not math.isfinite(inc) or inc <= 0.0:
            self.get_logger().error(
                f"스캔 angle_increment 이상({msg.angle_increment}) → 이 스캔 폐기, 정지",
                throttle_duration_sec=1.0,
            )
            with self.lock:
                self.watch.set_fault(True)     # 하드 폴트 → 즉시 레벨 2
            return

        with self.lock:
            self.watch.set_fault(False)        # 정상 스캔이 오면 스스로 풀린다
            self.latest_scan = msg
            self.watch.feed('scan', time.monotonic())

    def yaw_error_callback(self, msg):
        with self.lock:
            self.yaw_error = (360.0 - msg.data) % 360.0
            self.watch.feed('yaw_error', time.monotonic())   # [E] IMU 체인 생존 신호

    def geofence_cb(self, msg):
        """/geofence_state = [angle_min_deg, angle_inc_deg, r0, r1, ...] (상대방위, 멀면 inf).
        경계를 '가짜 LiDAR'로 쏜 것이다. **빈 배열 = 경계 정보 없음**
        (north 가 '모르면 입을 다무는' 계약: 폴리곤 미설정 / IMU stale / 이미 이탈)."""
        d = msg.data
        with self.lock:
            if len(d) >= 3 and math.isfinite(d[1]) and d[1] > 0.0:
                self.geofence = (float(d[0]), float(d[1]), [float(x) for x in d[2:]])
            else:
                self.geofence = None
            self.last_geofence_t = time.monotonic()

    def candidate_callback(self, msg):
        val = msg.data
        with self.lock:
            if val in (CANDIDATE_INVALID, STOP_HOLD, SPIN_RIGHT, SPIN_LEFT):
                self.candidate_angle = val
            else:
                self.candidate_angle = (val % 360.0)

    def wp_mode_cb(self, msg):
        with self.lock:
            prev = self.wp_mode
            self.wp_mode = msg.data

            # 게이트 구간(mode 0,1)은 좁아서 detection 을 짧게 (3.0 이면 오히려 접촉 증가, §6)
            new_dd = (self.detection_distance_gate
                      if self.wp_mode in self.gate_wp_modes
                      else self.detection_distance_default)
            if abs(new_dd - self.detection_distance) > 1e-6:
                self.detection_distance = new_dd

            if prev != 4 and self.wp_mode == 4:
                self.wp4_enter_time = time.monotonic()
            elif prev == 4 and self.wp_mode != 4:
                self.wp4_enter_time = None

    # =====================================================
    # WATCHDOG — 독립 타이머 (CLAUDE.md §5)
    # =====================================================
    def watchdog_cb(self):
        """페일세이프를 독립 타이머로 평가한다.
        scan_cb 안에서 평가하면 LiDAR 가 완전히 죽었을 때 콜백이 아예 안 와서
        페일세이프가 영원히 평가되지 않는다 (§5 에 기록된 실제 버그).

        [E] 감시 대상은 /scan 과 /yaw_error **둘 다**. 가장 나쁜 센서로 레벨을 정한다.
            /scan 만 보면 IMU 사망을 놓친다: IMU 가 죽으면 ship_goal_angle 이 /yaw_error 를
            끊어주는데(1단계 설계), 그 침묵을 들을 귀가 없으면 ship_direction 은 마지막
            yaw_error 를 영원히 붙들고 조향한다. (측정: 정지율 0%, 접촉 3회, 14.8m 계속 주행)
            센서별로 첫 데이터를 받은 뒤에만 감시(ARMED) — 부팅 오발동 방지.

        판정 로직은 SensorWatch(ROS 비의존)에 있다 — 단위 테스트 가능."""
        now = time.monotonic()
        prev = self.failsafe_level

        with self.lock:
            level = self.watch.update(now)
            worst = self.watch.worst

        if level != prev:
            log = self.get_logger().warn if level > prev else self.get_logger().info
            log(f"페일세이프 L{prev} → L{level}"
                + (f"  (원인: {worst})" if worst else "  (복구)"))

        self.failsafe_level = level
        self.pub_failsafe.publish(Int32(data=int(level)))

    # =====================================================
    # CONTROL — 고정 주기, 스캔 유무와 무관하게 항상 발행
    # =====================================================
    def control_cb(self):
        level = self.failsafe_level

        with self.lock:
            scan = self.latest_scan
            bad = self.watch.fault
            wp4_enter_time = self.wp4_enter_time
            candidate_angle = self.candidate_angle
            yaw_error = self.yaw_error
            detection_distance = self.detection_distance

        # ---- 레벨 2(정지) 또는 이상 스캔: STOP_HOLD 로 override. 각도를 덮어쓰는 건 여기뿐. ----
        # bad 를 여기서도 보는 이유: 워치독 틱(0.1s)을 기다리지 않고 즉시 멈추기 위해서다.
        if level >= 2 or bad:
            self.pub_obstacle_distance.publish(self._closest_obstacle(scan))
            self.pub_desired_angle.publish(Float32(data=STOP_HOLD))
            return

        # 스캔을 한 번도 못 받음(부팅 중) → 발행할 각도가 없다.
        # motor_control 은 '첫 명령 전 중립'이라 안전하다.
        if scan is None:
            return

        # ---- 레벨 0·1: 계산한 각도를 그대로 낸다. ----
        # 레벨 1(경고)에서도 각도는 안 건드린다. 속도만 줄인다
        # → motor_control 이 /failsafe_level 을 보고 상한(70%)을 건다.
        angle, obst = self._compute(scan, wp4_enter_time, candidate_angle,
                                    yaw_error, detection_distance)

        self.pub_obstacle_distance.publish(obst)
        self.pub_desired_angle.publish(Float32(data=angle))

    # =====================================================
    # 계산 (회피 알고리즘 — 작년 로직 보존)
    # =====================================================
    def _closest_obstacle(self, msg):
        """전방 0~160° 최근접 → Float32MultiArray([거리(m), 각도(deg)]). 없으면 [inf, nan]."""
        obst = Float32MultiArray()
        if msg is None:
            obst.data = [float('inf'), float('nan')]
            return obst

        angle_min = math.degrees(msg.angle_min)
        angle_increment_deg = math.degrees(msg.angle_increment)
        ranges = msg.ranges

        # [F] 0 나눗셈 방어 (scan_callback 이 이미 걸러내지만, 여기도 막아둔다)
        if not math.isfinite(angle_increment_deg) or angle_increment_deg <= 0.0:
            obst.data = [float('inf'), float('nan')]
            return obst

        min_index = int((0 - angle_min) / angle_increment_deg)
        max_index = int((160 - angle_min) / angle_increment_deg)

        # ---- [J] 공간 median 후 최소거리 (감속 신호 전용) ----
        # 작년엔 필터 없는 raw min 이라 물보라 반사 한 점(0.3m)이 그대로 motor_control 감속을
        # 물렸다. median 은 이웃과 어긋나는 고립 스파이크를 지운다.
        # ★ 여기(감속 신호)에만 건다 — _compute 의 회피 마스크는 손대지 않는다(접촉 성능 불변).
        d, i = median_min(ranges, min_index, max_index, self.obst_median_kernel)
        if d is None:
            obst.data = [float('inf'), float('nan')]
        else:
            obst.data = [d, angle_min + i * angle_increment_deg]
        return obst

    def _merge_geofence(self, distance_array, angle_array):
        """경계 '가짜 LiDAR'를 실제 스캔과 병합한다 — **한 줄이 전부다**:
              ranges[i] = min(real_ranges[i], geofence_ranges[i])

        이후는 기존 파이프라인이 알아서 한다: detection_distance 로 이진 마스크가 되고,
        dilate 가 배 폭(half_width+clearance) 여유를 더하고, 갭-팔로잉이 피한다.
        특수 로직도, 새 상태기계도, 튜닝 파라미터(half_block)도 없다.

        병합하지 않는 경우 (전부 '모르면 입을 다문다'):
          · geofence 가 None  → north 가 빈 배열 (미설정 / IMU stale / 이미 이탈)
          · geofence 가 stale → 묵은 '없는 벽'은 배를 가둔다
        """
        with self.lock:
            gf = self.geofence
            gf_t = self.last_geofence_t

        if gf is None or gf_t is None:
            return distance_array
        if (time.monotonic() - gf_t) > self.geofence_stale_sec:
            self.get_logger().warn(
                "geofence stale → 병합 안 함 (묵은 '없는 벽'은 배를 가둔다)",
                throttle_duration_sec=5.0)
            return distance_array

        a0, inc, gr = gf
        n = len(gr)
        if n == 0 or inc <= 0.0:
            return distance_array

        out = list(distance_array)
        hits = 0
        nearest = float('inf')
        for i, a in enumerate(angle_array):
            rel = a - LIDAR_FORWARD_DEG          # LiDAR 프레임(80=정면) → 상대방위(0=정면)
            k = int(round((rel - a0) / inc))
            if not (0 <= k < n):
                continue
            g = gr[k]
            if not math.isfinite(g):
                continue
            cur = out[i]
            if (not math.isfinite(cur)) or g < cur:
                out[i] = g                        # ★ min(실제, 경계)
                hits += 1
                nearest = min(nearest, g)

        if hits:
            self.get_logger().warn(
                f"🚧 경계가 스캔에 병합됨 ({hits}셀, 최근접 {nearest:.1f}m)",
                throttle_duration_sec=2.0)
        return out

    def _geofence_blocks(self, intended_rel):
        """[I] 경계가 stop_margin 안이고, 그 방위가 '전진하려는 방향 ±cone' 과 겹치면 True.

        (A)SPIN·(B)candidate 구간은 distance_array 를 안 만들어 스캔 병합이 안 된다.
        도킹은 SPIN+candidate 로 조향하므로, 도크가 경기장 가장자리면 접안하다 경계로 파고든다.
        → 여기서 하드하게 막는다. **미션보다 실격 방지가 우선.** 경계를 등지고 멈춘다.
        (C) 자율회피는 이미 _merge_geofence 로 처리되므로 이 가드를 호출하지 않는다.
        ⚠️ north 의 광선은 전방 ±80° 만 쏜다 → 뒤쪽(후진 중) 경계는 못 본다. 접안 방향(전방) 방어용.
        """
        with self.lock:
            gf = self.geofence
            gf_t = self.last_geofence_t

        if gf is None or gf_t is None:
            return False
        if (time.monotonic() - gf_t) > self.geofence_stale_sec:
            return False

        a0, inc, gr = gf
        if not gr or inc <= 0.0:
            return False

        # 최근접 경계와 그 상대방위
        best_d, best_rel = float('inf'), None
        for k, g in enumerate(gr):
            if math.isfinite(g) and g < best_d:
                best_d, best_rel = g, a0 + k * inc
        if best_rel is None or best_d > self.geofence_stop_margin_m:
            return False

        # 전진하려는 방향과 겹치나 (각도차를 [-180,180]로 래핑)
        diff = ((best_rel - intended_rel) + 180.0) % 360.0 - 180.0
        if abs(diff) <= self.geofence_stop_cone_deg:
            self.get_logger().warn(
                f"🚧🛑 경계 {best_d:.1f}m (상대 {best_rel:.0f}°) 가 전진 방향({intended_rel:.0f}°)과 "
                f"겹침 → 미션 무시하고 정지 (실격 방지 우선)",
                throttle_duration_sec=1.0)
            return True
        return False

    def _compute(self, msg, wp4_enter_time, candidate_angle, yaw_error, detection_distance):
        """(desired_angle, obstacle_array) 를 만든다. 작년 scan_callback 의 로직 그대로."""
        now = time.monotonic()

        obst = self._closest_obstacle(msg)

        # ---- (0) WP4 초기 5초 & STOP_HOLD 우선 처리 ----
        if (wp4_enter_time is not None and (now - wp4_enter_time) < 5.0) or \
           (candidate_angle == STOP_HOLD):
            return STOP_HOLD, obst

        # ---- [I] Geofence 하드 가드 ((A)SPIN/(B)candidate 전용) ----
        # 이 두 구간은 마스크 병합이 없어 경계로 파고들 수 있다(도킹이 가장자리일 때).
        # (C)(CANDIDATE_INVALID)는 아래에서 _merge_geofence 로 처리되므로 제외한다.
        if candidate_angle != CANDIDATE_INVALID:
            # 전진하려는 방향: candidate 는 그 상대각, SPIN(제자리)은 전방(0°)으로 본다.
            intended_rel = 0.0 if candidate_angle in (SPIN_RIGHT, SPIN_LEFT) else candidate_angle
            if self._geofence_blocks(intended_rel):
                return STOP_HOLD, obst

        # ---- (A) PASS-THROUGH: 미션 노드가 선회를 직접 지시 ----
        if candidate_angle in (SPIN_RIGHT, SPIN_LEFT):
            return candidate_angle, obst

        # ---- (B) 일반 candidate_angle 매핑 (360° → 전방중심 80°) ----
        if candidate_angle != CANDIDATE_INVALID:
            c_raw = candidate_angle % 360.0
            c_mapped = 80 + c_raw if c_raw <= 180 else 80 - (360 - c_raw)
            return c_mapped, obst

        # ---- (C) candidate == 20000 → yaw_error 기반 자율 회피 ----
        angle_min = math.degrees(msg.angle_min)
        angle_increment_deg = math.degrees(msg.angle_increment)
        angle_increment_rad = msg.angle_increment
        ranges = list(msg.ranges)

        # [F] 0 나눗셈 방어 (scan_callback 이 이미 걸러내지만, 여기도 막아둔다)
        if not math.isfinite(angle_increment_deg) or angle_increment_deg <= 0.0:
            self.get_logger().error("angle_increment 이상 → 정지", throttle_duration_sec=1.0)
            return STOP_HOLD, obst

        min_index = int((0 - angle_min) / angle_increment_deg)
        max_index = int((160 - angle_min) / angle_increment_deg)
        sub_ranges = ranges[min_index:max_index + 1]

        angle_array = []
        distance_array = []

        for i, r in enumerate(sub_ranges):
            angle_array.append(angle_min + (min_index + i) * angle_increment_deg)
            distance_array.append(r)

        # ---- [H] Geofence 병합 (6a-2). 이게 전부다: ranges[i] = min(실제, 경계) ----
        # 경계는 north 가 '가짜 LiDAR'로 쏴서 보낸다. 여기서 스캔에 섞어버리면
        # 아래 파이프라인(마스크 → dilate → 갭-팔로잉)이 경계를 그냥 '벽'으로 취급한다.
        distance_array = self._merge_geofence(distance_array, angle_array)

        # ---- Binary obstacle mask ----
        # ※ rear_obstacle_ignore_margin 제거됨. 작년엔 여기서
        #      if abs(r - front_min_distance) > rear_obstacle_ignore_margin: binary.append(0)
        #    로 '최근접점에서 먼 셀'을 지웠는데, 물보라 반사 0.3m 하나가 front_min_distance 를
        #    끌어내리면 실제 1.6m 부표가 통째로 지워졌다 (17셀 → 0셀). 튜닝값이 아니라 결함이었다.
        binary = []
        for r in distance_array:
            if math.isinf(r) or math.isnan(r):
                binary.append(0)
            elif r < detection_distance:
                binary.append(1)
            else:
                binary.append(0)

        # ---- [G] 시간 투표 — 물보라 한 프레임짜리 오탐 제거 ----
        # ★ 반드시 dilate 전, 원본 마스크에 건다. 팽창 후에 걸면 진짜 부표의 팽창 영역까지
        #   표가 갈려 부표가 얇아진다.
        binary = self.temporal.apply(binary)

        binary = self.smooth_spikes(binary)
        binary = self.suppress_spike_edges(binary)
        binary = self.dilate_obstacles(binary, distance_array, angle_increment_deg)

        # ---- safe zone 탐색 ----
        safe_zones = []
        start = None
        for i, v in enumerate(binary):
            if v == 0 and start is None:
                start = i
            elif v == 1 and start is not None:
                safe_zones.append((start, i - 1))
                start = None
        if start is not None:
            safe_zones.append((start, len(binary) - 1))

        # ---- 통과 가능한 zone 선별 ----
        valid_safe_zones = []
        min_required_width = self.half_width * 2 + self.clearance

        for s, e in safe_zones:
            r_s = distance_array[s] if not math.isinf(distance_array[s]) and not math.isnan(distance_array[s]) else detection_distance
            r_e = distance_array[e] if not math.isinf(distance_array[e]) and not math.isnan(distance_array[e]) else detection_distance
            r_edge = min(r_s, r_e)
            arc_len = angle_increment_rad * r_edge * (e - s)
            if arc_len >= min_required_width:
                valid_safe_zones.append((s, e, r_edge))

        # ---- yaw_error 를 safe zone 안으로 clamp → 최적 후보 ----
        yaw_raw = yaw_error % 360.0
        yaw_mapped = 80 + yaw_raw if yaw_raw <= 180 else 80 - (360 - yaw_raw)

        candidates = []
        for s, e, r_edge in valid_safe_zones:
            zone_min = angle_array[s]
            zone_max = angle_array[e]

            if yaw_mapped < zone_min:
                best_angle = zone_min
            elif yaw_mapped > zone_max:
                best_angle = zone_max
            else:
                best_angle = yaw_mapped

            angle_diff = abs(best_angle - yaw_mapped)
            arc_length = angle_increment_rad * r_edge * (e - s)
            candidates.append((angle_diff, -arc_length, best_angle, s, e))

        # ---- 회피 경로가 없으면 후진 ----
        if not candidates:
            final_angle = REVERSE_ANGLE
            if now - self.last_reverse_time >= self.reverse_cooldown:
                self.last_reverse_time = now
        else:
            candidates.sort(key=lambda x: (x[0], x[1]))
            final_angle = candidates[0][2]

        return final_angle, obst

    # =====================================================
    # 스파이크 억제 / 팽창 (작년 그대로)
    # =====================================================
    def smooth_spikes(self, binary):
        total_len = len(binary)
        max_spike_length = int(total_len * self.max_spike_ratio)
        smoothed = binary[:]
        count = 0
        start = None
        for i, v in enumerate(binary):
            if v == 1:
                if start is None:
                    start = i
                count += 1
            else:
                if start is not None and count <= max_spike_length:
                    for j in range(start, i):
                        smoothed[j] = 0
                start = None
                count = 0
        if start is not None and count <= max_spike_length:
            for j in range(start, len(binary)):
                smoothed[j] = 0
        return smoothed

    def suppress_spike_edges(self, binary):
        suppressed = binary[:]
        for i, v in enumerate(binary):
            if v == 1:
                for off in range(-self.border_margin, self.border_margin + 1):
                    j = i + off
                    if 0 <= j < len(binary) and binary[j] == 0:
                        suppressed[i] = 0
                        break
        return suppressed

    def dilate_obstacles(self, binary, distance_array, angle_increment_deg):
        n = len(binary)
        out = binary[:]
        i = 0
        while i < n:
            if binary[i] == 1:
                start = i
                while i + 1 < n and binary[i + 1] == 1:
                    i += 1
                end = i

                length = end - start + 1
                if length >= self.min_obstacle_cells:
                    r_edge = self.detection_distance
                    for k in range(start, end + 1):
                        r = distance_array[k]
                        if not math.isinf(r) and not math.isnan(r):
                            r_edge = min(r_edge, r)

                    lateral = self.half_width + self.clearance
                    r_use = max(r_edge, 0.01)
                    ang_margin = math.degrees(math.atan(lateral / r_use))
                    cells_margin = max(1, int(round(ang_margin / max(angle_increment_deg, 1e-6))))

                    new_start = max(0, start - cells_margin)
                    new_end = min(n - 1, end + cells_margin)

                    for j in range(new_start, new_end + 1):
                        out[j] = 1
                i += 1
            else:
                i += 1
        return out


def main(args=None):
    rclpy.init(args=args)
    node = ShipDirection()

    executor = MultiThreadedExecutor(num_threads=4)
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
