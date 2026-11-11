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
from ship_direction.dwa import DwaPlanner, HULLS as DWA_HULLS
from ship_direction.failsafe import SensorWatch, TemporalVote, median_min
from ship_direction.failsafe import effective_candidate, candidate_is_stale, CandidateStaleLog
from ship_direction.failsafe import heading_missing
from ship_direction.gap_select import obstacle_blobs, rank_candidates, next_side_memory
from ship_direction.gap_select import side_escape
from ship_direction.side_lock import SideLockWait, SideLockFilter, side_states, allowed_range
from ship_direction.side_lock import angle_allowed
from ship_direction.side_lock import clip_candidates, check_params, UNKNOWN
from ship_direction.camera_merge import CAMERA_ALLOWED_MODES, ST_EMPTY, camera_active, camera_silent
from ship_direction.camera_merge import merge_camera, parse_state
from ship_direction.camera_merge import check_enabled as check_camera_enabled
from ship_direction.camera_merge import check_modes as check_camera_modes
from ship_direction.camera_merge import check_stale_sec as check_camera_stale_sec

# 특수 신호 상수 (CLAUDE.md 3-9 상수표)
SPIN_RIGHT = 5000.0          # 우선회
SPIN_LEFT = 6000.0           # 좌선회
CANDIDATE_INVALID = 20000.0  # 미션 없음 → yaw_error 기반 자율 회피
STOP_HOLD = 50000.0          # 정지/대기

REVERSE_ANGLE = 260.0        # 후진 (motor_control 분기 5). 두 곳에서 나온다:
#   (C) 회피 경로 없음 → [L] 상태기계, (B) /candidate_angle 180.0 정확 일치(아래)
REVERSE_REL = 180.0          # /candidate_angle 180.0 = 후진 요청 (ship_dock BACKOFF·ship_back).
#   (B) 에서 범위 검사 앞 정확 일치로만 260 으로 통과 (계약서_후진통과_20261104).
LIDAR_FORWARD_DEG = 80.0     # LiDAR 프레임에서 정면 (상대방위 0 에 대응)
CANDIDATE_STALE_LOG_PERIOD_SEC = 5.0  # candidate stale 지속 중 ERROR 반복 주기(로그 전용, 제어 무관)
YAW_MISSING_LOG_PERIOD_SEC = 5.0  # [N] yaw_error 미수신 정지 지속 중 ERROR 반복 주기(로그 전용, 제어 무관)
CANDIDATE_SLOWEST_PUB_SEC = 0.5  # 가장 느린 정상 /candidate_angle 발행 주기(north period_sec).
# candidate_stale_sec 가 이보다 작으면 거짓 stale 경고만(값은 그대로)
DWA_ALLOWED_MODES = (5, 8)   # DWA 를 켤 수 있는 wp_mode (계약서_DWA_20261106 팀장 결정 ①). 0·1·2·3·7 금지
DWA_AGILITY_MIN, DWA_AGILITY_MAX = 0.5, 2.0   # dwa_model_agility 허용 범위(결정 5)
DWA_LIDAR_X_MAX_M = 1.0      # |dwa_lidar_x_m| 상한 (선체 길이 안)
DWA_SLOW_WARN_SEC = 0.05     # DWA 1회가 이보다 오래 걸리면 WARN(결정 12, 로그 전용)
DWA_LOG_PERIOD_SEC = 5.0     # DWA 느림·예외 로그 throttle(로그 전용)
DWA_IDLE_RESET_SEC = 1.0     # 같은 DWA 모드 안에서 (C) 틱이 이보다 오래 끊기면 추정 초기화
#   ((0)·레벨2·[N]·(A)(B) 동안 배가 멈췄을 수 있다 — 추정은 (C) 틱에서만 굴린다)
CAMERA_SILENT_LOG_PERIOD_SEC = 5.0  # 카메라 장애물 침묵 지속 중 ERROR 반복 주기(로그 전용, 제어 무관)
CAMERA_LOG_PERIOD_SEC = 5.0  # 카메라 빈 배열 WARN throttle(로그 전용)


def edge_range(distance_array, i, detection_distance):
    """zone 가장자리 셀의 거리. inf·nan 은 '측정 없음' 이라 detection_distance 로 본다.

    모듈 함수로 둔 이유: 노드 메서드 안 클로저였을 땐 테스트가 못 닿았다 (과제 S4).
    """
    v = distance_array[i]
    return detection_distance if (math.isinf(v) or math.isnan(v)) else v


def select_valid_zones(safe_zones, distance_array, angle_increment_rad,
                       detection_distance, min_required_width):
    """safe zone 중 배가 지나갈 수 있는 것만 고른다 → [(s, e, r_edge), ...].

    폭 = 양 끝 중 가까운 거리(r_edge) × 호 길이. r_edge 를 쓰는 이유는 가장자리가
    먼 쪽 거리로 재면 실제보다 넓게 보기 때문이다. min_required_width 는
    half_width*2 + clearance (CLAUDE.md §6, yaml 과 같은 값).
    """
    valid = []
    for s, e in safe_zones:
        r_edge = min(edge_range(distance_array, s, detection_distance),
                     edge_range(distance_array, e, detection_distance))
        arc_len = angle_increment_rad * r_edge * (e - s)
        if arc_len >= min_required_width:
            valid.append((s, e, r_edge))
    return valid


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
# /camera_obstacle_state 구독 — 이미지 구독과 같은 규칙(묵은 프레임으로 조향 금지, 계약서_카메라장애물 토픽 계약).
#   발행자(RELIABLE)와 호환. depth=1 이라 도착 = 최신.
CAMERA_OBSTACLE_QOS = QoSProfile(
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
        2 정지(3.0s)  : STOP_HOLD override / 정지        ← 페일세이프 레벨 중 각도를 덮어쓰는 건 여기뿐
          (레벨 0 이어도 [N] yaw 미수신+20000·[I] 경계는 50000)

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
        ※ 그래서 '부팅부터 /yaw_error 가 한 번도 안 옴'은 워치독이 못 본다(레벨 0 그대로) → [N].

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
          🚨 상대방위는 **좌 양수** — LiDAR 프레임 `a−80`·/candidate_angle 과 같은 부호.
             배열 인덱스 0 (rel −80) = 우현, 끝 인덱스 (+80) = 좌현. north 가 이 규약으로 낸다
             (계약서_geofence부호_20261104). 여기(소비자)는 변환하지 않는다 — 양쪽을 다 고치면 다시 거울.
        여기서 하는 일은 **스캔 병합 한 줄**이 전부다:
          ranges[i] = min(real_ranges[i], geofence_ranges[i])
        그 뒤는 기존 파이프라인이 알아서 한다 — detection_distance 로 마스크가 되고,
        dilate 가 배 폭 여유를 더하고, 갭-팔로잉이 피한다.
        **특수 로직도, 새 상태기계도, 튜닝 파라미터(half_block)도 없다.**

        ★ 왜 '원뿔 칠하기'를 버렸나: **벽은 '점'이 아니라 '선'이다.**
          최근접점 한 방향으로 ±각도를 막는 건 기하학적으로 틀렸다. 40×40 모서리에서
          배가 대각선(45°)을 보면 북벽 최근접점 +45°(좌), 동벽 -45°(우) 인데 **탈출구(모서리)는 정면 0°**.
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
        ⚠️ 후진(/candidate_angle 180)에는 구조상 가드가 걸리지 않음: 광선 rel −80~+80 과 180 의 차가
           ≥ 100° 라 cone 60° 안에 들 수 없다. 뒤쪽 경계는 아무도 안 본다 (계약서_후진통과_20261104 §4).

    [J] 감속 신호용 공간 median 필터 (obst_median_kernel, **기본 ON**).
        _closest_obstacle 이 필터 없는 raw min 이라 **물보라 반사 한 점(0.3m)이 그대로
        motor_control 감속을 물렸다.** 이웃과 어긋나는 고립 스파이크를 range 값 수준에서 지운다.
        측정(시드 20, 물보라 30%): raw-min 가짜감속 **36.8% → median 8.9%**(기준선). 접촉은 전 조건 0.
        ★ **감속 신호에만** 건다 — _compute 의 회피 마스크는 안 건드린다(그래서 접촉이 안 변한다).
        ★ [G] TemporalVote(시간 투표)와 다르다: 저건 효과 0 이라 OFF, 이건 명확한 이득이라 ON.
          시간 투표는 프레임 간, 이건 한 프레임 안의 공간.
        ⚠️ 시뮬 물보라는 '단일점' 모델이다. 실제 물보라가 작은 군집이면 커널5로 부족할 수 있다
           → 배 뜬 뒤 블랙박스의 obstacle_min_dist 로 실측 확인하고 커널을 7/9 로 올릴 것.

    [M] candidate 신선도 (계약서_전체코스_20261104 §2-6). /candidate_angle 이 수신 후
        candidate_stale_sec(1.0) 넘게 갱신이 없으면 control_cb 가 _compute 전에 20000 으로 바꾼다
        (발행 노드가 죽으면 묵은 조향각·SPIN·STOP_HOLD 를 모드가 바뀔 때까지 붙들던 결함).
        wp_mode 가 바뀌면 candidate 를 20000·수신 기록 없음으로 초기화. 진입 ERROR 1회 + 5 s 주기 ERROR,
        복구 INFO 1회. 180(후진)은 0.5 s 규칙 유지(0.5~1.0 s STOP_HOLD, 그 뒤 20000).

    [N] yaw_error 미수신 정지 (계약서_yaw미수신정지_20261105, 안 A). /yaw_error 를 부팅 뒤 **한 번도**
        못 받았는데(yaw_error None) 실효 candidate 가 20000(갭 선택)이면 control_cb 가 _compute 전에
        candidate 를 STOP_HOLD 로 바꾼다. 예전엔 초기값 0.0 을 '목표가 정면'으로 읽어 뱃머리 쪽 갭으로
        전진했다(GPS fix 없음 → north 가 tp 미발행 → ship_goal_angle 이 /yaw_error 미발행인데,
        [E] 워치독은 ARMED 전이라 레벨 0). 판정은 `is None` — 0.0 은 정상값.
        · 각도·5000·6000·180 은 yaw_error 를 안 읽으므로 그대로 따른다(거짓 정지 없음).
        · [M] 의 stale·wp_mode 변경으로 생긴 20000 도 같은 입구라 함께 막힌다.
        · /failsafe_level 은 0 그대로(mission_monitor 엔 정상으로 보임) → ERROR 로그가 유일한 단서.
          진입 ERROR 1회 + YAW_MISSING_LOG_PERIOD_SEC(5 s) 주기 ERROR(막힌 틱에서만),
          복구 INFO 1회(yaw_error 첫 수신에서만). 후보각이 20000 ↔ 각도로 깜빡여도 폭주 안 함.
        · 받은 뒤 끊김은 [E] 가 맡는다(ARMED 뒤 stale → 레벨 2, 모든 모드 정지).
        · _compute (C) 첫머리에도 같은 가드(None → STOP_HOLD) — None % 360 TypeError 로 노드가
          죽는 걸 막는 방어선.

    [P] 카메라 장애물 병합 (계약서_카메라장애물_20261111 U1). LiDAR 빔이 못 보는 낮은 물체(누운 원통)를
        카메라 노드가 /camera_obstacle_state 로 낸다 — /geofence_state 와 같은 '가짜 LiDAR' 형식·좌 양수.
        camera_obstacle_enabled(기본 false) + wp_mode ∈ camera_obstacle_modes([8]) 일 때만 (C) 에서
        [H] geofence 병합 **바로 뒤** 같은 매핑으로 min 병합(계산은 ship_direction/camera_merge.py).
        끔 = 아무것도 안 함(기존과 비트 동일). 감속 신호(_closest_obstacle)·옆 잠금·[I] 하드 가드에는
        안 넣는다(원 스캔 그대로). stale(camera_obstacle_stale_sec 0.5 s)·빈 배열·못 받음 → 병합 안 함.
        SensorWatch·/failsafe_level 에 넣지 않는다 — 카메라가 멈춰도 제어 체인은 돈다(OAK 재연결 공백).
        침묵(켜짐 + 담당 모드에서 수신 없음)은 ERROR 진입 1회 + 5 s 주기 + 복구 INFO 로만 드러낸다.

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

        self.half_width = float(self.declare_parameter('half_width', 0.29).value)
        # 0.20→접촉 1.7회 / 0.25→0.2회 / 0.30→0.8회 / 0.45→807초 폭주(과보수)
        self.clearance = float(self.declare_parameter('clearance', 0.25).value)
        self.border_margin = int(self.declare_parameter('border_margin', 2).value)
        self.max_spike_ratio = float(self.declare_parameter('max_spike_ratio', 0.01).value)
        # ※ rear_obstacle_ignore_margin 은 제거했다 (파라미터 자체를 없앰). 위 [C] 참고.
        self.min_obstacle_cells = int(
            self.declare_parameter('min_obstacle_cells', 1).value)   # 3 이면 작은 부표가 무시된다

        # ---- [L] 후진 후 재탐색 상태기계 ----
        # 회피 경로가 없으면 reverse_duration 동안 후진하고,
        # 이후 reverse_cooldown 동안 STOP_HOLD 상태로 재탐색한다.
        self.reverse_duration = float(
            self.declare_parameter('reverse_duration_sec', 0.5).value)
        self.reverse_cooldown = float(
            self.declare_parameter('reverse_cooldown_sec', 3.0).value)

        # ---- (B) candidate 후진(180.0) 신선도 (계약서_후진통과_20261104 §10-4) ----
        # 180 을 낸 미션 노드(ship_dock·ship_back)가 후진 도중 죽으면 마지막 180 을 붙들고
        # 모드가 바뀔 때까지 후진한다. 그래서 180 은 수신 후 이 시간 안일 때만 260 으로 통과시키고,
        # 넘으면 STOP_HOLD. 발행 노드는 10 Hz 매 틱 발행 → 0.5 s = 5 틱 누락. 다른 candidate 값엔 안 건다.
        self.candidate_reverse_stale_sec = float(
            self.declare_parameter('candidate_reverse_stale_sec', 0.5).value)
        # ---- candidate 전체 신선도 (계약서_전체코스_20261104 §2-6·§10-3) ----
        # 수신 후 이 시간 넘게 갱신이 없으면 발행 노드 사망으로 보고 candidate 를 20000 으로 바꾼다
        # (GPS 주행 + LiDAR 회피). 발행 주기 0.1 s(미션 노드)·0.5 s(north) → 1.0 = north 주기 × 2.
        # 180(후진)은 위 0.5 s 규칙 유지: 0.5~1.0 s 는 _compute 가 STOP_HOLD, 1.0 s 넘으면 20000.
        # 🚨 대회 launch 가 config/ship_direction.yaml 을 넘긴다(11-05) → yaml 값이 대회값.
        #    yaml 과 같이 바꿀 것.
        self.candidate_stale_sec = float(
            self.declare_parameter('candidate_stale_sec', 1.0).value)

        # 잘못된 파라미터 방어
        if not math.isfinite(self.candidate_reverse_stale_sec) or \
                self.candidate_reverse_stale_sec <= 0.0:
            self.get_logger().warn(
                f"candidate_reverse_stale_sec={self.candidate_reverse_stale_sec} 이상 "
                f"→ 기본값 0.5초 사용")
            self.candidate_reverse_stale_sec = 0.5
        if not math.isfinite(self.candidate_stale_sec) or self.candidate_stale_sec <= 0.0:
            self.get_logger().warn(
                f"candidate_stale_sec={self.candidate_stale_sec} 이상 → 기본값 1.0초 사용")
            self.candidate_stale_sec = 1.0
        elif self.candidate_stale_sec < CANDIDATE_SLOWEST_PUB_SEC:
            self.get_logger().warn(
                f"candidate_stale_sec={self.candidate_stale_sec} < north 발행 주기 "
                f"{CANDIDATE_SLOWEST_PUB_SEC}초 → 거짓 stale 위험(정상 발행 사이에 20000). 값은 그대로 사용")
        if not math.isfinite(self.reverse_duration) or self.reverse_duration <= 0.0:
            self.get_logger().warn(
                f"reverse_duration_sec={self.reverse_duration} 이상 → 기본값 0.5초 사용")
            self.reverse_duration = 0.5
        if not math.isfinite(self.reverse_cooldown) or self.reverse_cooldown < 0.0:
            self.get_logger().warn(
                f"reverse_cooldown_sec={self.reverse_cooldown} 이상 → 기본값 3.0초 사용")
            self.reverse_cooldown = 3.0

        # ---- [K] 조향각 변화율 제한 (rate limiter). 팀원 피드백으로 "0~160° 한정" 적용. ----
        # 원안(전체 구간 적용)은 /desired_angle 이 각도+특수신호(260/5000/6000/20000/50000) 혼용
        # 프로토콜이라, 정상값→특수신호 사이를 15°/tick 씩 기어가면 motor_control 이 그 경유값을
        # 후진/선회/무효로 오해할 위험이 있었다(팀 지적). 0~160° 구간(정상 조향값)일 때만 제한을
        # 걸고, 그 밖(특수신호 및 candidate 매핑의 범위 밖 부산물)은 그대로 통과시킨다.
        # 덤: 180° 급반전(정후방 목표 → 좌우 zone 끝 선택)도 결과가 항상 0~160° 안이라 이 필터로
        # 완화된다. 실제 각속도 = 이 값 / control_period(예: 15°, 0.1s 면 150°/s) — 실측 후 튜닝.
        self.rate_limit_max_deg_per_tick = float(
            self.declare_parameter('rate_limit_max_deg_per_tick', 15.0).value)

        # ---- (C) 후보 정렬 벌점(도) — 발견 8 (계약서_회피편선택_20261106) ----
        # 편 유지: 직전 선택(리미터 전)과 정면 80 반대편 후보에 +side_hysteresis_deg.
        # 가로지름: 정면과 후보 사이에 장애물 덩어리 중심이 끼면 +cross_penalty_deg.
        # 0.0 = 끔(기존 정렬과 같음). 비유한·음수 → 기본값 + WARN 1회.
        # 🚨 대회 launch 가 yaml 을 넘긴다 → yaml 과 같이 바꿀 것.
        self.side_hysteresis_deg = float(
            self.declare_parameter('side_hysteresis_deg', 40.0).value)
        self.cross_penalty_deg = float(
            self.declare_parameter('cross_penalty_deg', 90.0).value)
        if not math.isfinite(self.side_hysteresis_deg) or self.side_hysteresis_deg < 0.0:
            self.get_logger().warn(
                f"side_hysteresis_deg={self.side_hysteresis_deg} 이상 → 기본값 40.0도 사용")
            self.side_hysteresis_deg = 40.0
        if not math.isfinite(self.cross_penalty_deg) or self.cross_penalty_deg < 0.0:
            self.get_logger().warn(
                f"cross_penalty_deg={self.cross_penalty_deg} 이상 → 기본값 90.0도 사용")
            self.cross_penalty_deg = 90.0

        # ---- (C) 후보 0 개 + 옆 장애물 → 후진 대신 비킴 — 발견 9 (계약서_옆비킴_현장조정_20261106) ----
        # 가장 가까운 덩어리 중심이 정면에서 15~60° 옆이고 정면·호·목표 창이 비었으면 반대편으로
        # 이 각도(도)만큼 꺾은 각을 낸다(260 대신). 게이트 구간(gate_wp_modes)은 비킴 안 함.
        # 0.0 = 끔(기존 후진과 같음, 기본). 0 < 값 ≤ 80(목표각 0~160 안). 비유한·음수·> 80 → 0.0 + WARN.
        # 🚨 대회 launch 가 yaml 을 넘긴다 → yaml 과 같이 바꿀 것. 실배 확인 뒤 배별 yaml 에서 60.0.
        self.side_escape_deg = float(
            self.declare_parameter('side_escape_deg', 0.0).value)
        if not math.isfinite(self.side_escape_deg) or self.side_escape_deg < 0.0 or \
                self.side_escape_deg > LIDAR_FORWARD_DEG:
            self.get_logger().warn(
                f"side_escape_deg={self.side_escape_deg} 이상(0~{LIDAR_FORWARD_DEG:.0f} 밖) "
                f"→ 0.0(비킴 끔) 사용")
            self.side_escape_deg = 0.0

        # ---- (C) DWA — 발견 10·11 (계약서_DWA_20261106) ----
        # wp_mode 가 dwa_modes 이고 (C)(20000) 이면 조향각 후보 0~160 을 3자유도 모델로 2.5 s 굴려
        # 선체 직사각형이 LiDAR 점에 0.25 m 안으로 안 드는 각 중 목표에 가까운 것을 고른다(ship_direction/
        # dwa.py). 고르면 비킴·후진 상태기계를 건너뛴다. 전부 불가면 기존 결과(갭 정렬·비킴·후진).
        # 끔(기본) = 기존과 비트 동일. 🚨 대회 launch 가 yaml 을 넘긴다 → yaml 과 같이 바꿀 것.
        self.dwa_enabled = self.declare_parameter('dwa_enabled', False).value
        dwa_modes_raw = self.declare_parameter('dwa_modes', [8]).value
        self.dwa_hull = self.declare_parameter('dwa_hull', 'A').value
        self.dwa_model_agility = float(
            self.declare_parameter('dwa_model_agility', 1.0).value)
        self.dwa_lidar_x_m = float(
            self.declare_parameter('dwa_lidar_x_m', 0.0).value)   # ⚠️ 실측 필요(LiDAR 앞뒤 위치)
        if not isinstance(self.dwa_enabled, bool):
            self.get_logger().warn(
                f"dwa_enabled={self.dwa_enabled!r} bool 아님 → false(DWA 끔) 사용")
            self.dwa_enabled = False
        try:
            dwa_modes_raw = list(dwa_modes_raw)
        except TypeError:
            dwa_modes_raw = []
        self.dwa_modes = []
        dropped = []
        for m in dwa_modes_raw:
            if isinstance(m, bool) or m not in DWA_ALLOWED_MODES:
                dropped.append(m)
            elif int(m) not in self.dwa_modes:
                self.dwa_modes.append(int(m))
        if dropped:
            self.get_logger().warn(
                f"dwa_modes 의 {dropped} 는 허용({list(DWA_ALLOWED_MODES)}) 밖 → 제거, "
                f"남은 {self.dwa_modes}")
        if self.dwa_enabled and not self.dwa_modes:
            self.get_logger().warn("dwa_modes 에 허용 모드가 하나도 없음 → DWA 끔")
            self.dwa_enabled = False
        if not isinstance(self.dwa_hull, str) or self.dwa_hull not in DWA_HULLS:
            self.get_logger().error(
                f"dwa_hull={self.dwa_hull!r} 는 'A'/'B' 가 아님 → DWA 끔")
            self.dwa_enabled = False
        if not math.isfinite(self.dwa_model_agility) or \
                not (DWA_AGILITY_MIN <= self.dwa_model_agility <= DWA_AGILITY_MAX):
            self.get_logger().warn(
                f"dwa_model_agility={self.dwa_model_agility} 이상({DWA_AGILITY_MIN}~"
                f"{DWA_AGILITY_MAX} 밖) → 1.0 사용")
            self.dwa_model_agility = 1.0
        if not math.isfinite(self.dwa_lidar_x_m) or abs(self.dwa_lidar_x_m) > DWA_LIDAR_X_MAX_M:
            self.get_logger().warn(
                f"dwa_lidar_x_m={self.dwa_lidar_x_m} 이상(|x| > {DWA_LIDAR_X_MAX_M} 또는 비유한) "
                f"→ 0.0 사용")
            self.dwa_lidar_x_m = 0.0
        # 끄면 planner 를 만들지 않는다. 켜면 시작 INFO 한 줄(선체 글자 — 배별 yaml 확인용, 결정 9).
        self._dwa = None
        self._dwa_active = False      # 지금 DWA 모드 구간인가(진입 리셋·INFO 1회용)
        self._dwa_picked = False      # 직전 DWA 틱에 DWA 가 골랐나(폴백 INFO 용)
        self._dwa_last_t = None       # 마지막 DWA (C) 틱 시각(monotonic). 공백 판정용
        self._dwa_needs_reset = False  # observe 예외 → 다음 DWA 틱에 조용히 초기화
        if self.dwa_enabled:
            self._dwa = DwaPlanner(self.dwa_hull, self.dwa_model_agility, self.dwa_lidar_x_m)
            self.get_logger().info(
                f"DWA 켬: 선체 {self.dwa_hull}, 모드 {self.dwa_modes}, "
                f"배율 {self.dwa_model_agility}, lidar_x {self.dwa_lidar_x_m} m")

        # ---- (C) 옆 잠금 — 토너먼트 T2-1 (계약서_토너먼트분리_20261109 §9-6·§9-7) ----
        # 배 옆 구역(상대방위 ±side_lock_min_deg~side_lock_max_deg, 좌 양수)의 최솟값이 side_lock_dist_m
        # 안인 새 스캔이 연속 side_lock_confirm_frames 번이면 그쪽을 잠그고(dist + side_lock_release_margin_m
        # 이상이면 풀림), 그쪽으로 정면(80) 대비 side_lock_turn_deg 넘게 꺾는 후보(갭·DWA·비킴)를 뺀다
        # (규정: 나란히 가다 침로를 바꾼 쪽이 접촉 책임). **잠금 때문에** 후보가 0 개가 되면 후진하지 않고
        # STOP_HOLD 로 매 틱 재평가, side_lock_wait_max_sec 넘으면 기존 동작으로 폴백 + WARN. 잠금과
        # 무관하게 막힌 곳(잠금 없이도 후보 0)은 기존 동작(비킴·후진) 그대로. 스캔이 옆 구역을
        # 못 덮으면 그쪽은 '모름' → 안 잠금(기존 동작). 상대 배·부표 구분 안 함. 계산은 side_lock.py.
        # 켜짐 여부는 이 스위치뿐 — wp_mode 로 가르지 않는다(main 에도 mode 8). 끔(기본) = 기존과 같다.
        # 이상값(bool 아님·비유한·범위 밖) → ERROR 1회 + 끔. 🚨 대회 launch 가 yaml 을 넘긴다 → yaml 과 같이.
        # ⚠️ RPLIDAR 옆·뒤 가림·자기 선체 반사 미확인(side_lock.py 머리말) — 물 위 확인 뒤 켬.
        self.side_lock_enabled = self.declare_parameter('side_lock_enabled', False).value
        self.side_lock_dist_m = float(self.declare_parameter('side_lock_dist_m', 1.5).value)
        self.side_lock_min_deg = float(self.declare_parameter('side_lock_min_deg', 45.0).value)
        self.side_lock_max_deg = float(self.declare_parameter('side_lock_max_deg', 135.0).value)
        self.side_lock_turn_deg = float(self.declare_parameter('side_lock_turn_deg', 15.0).value)
        self.side_lock_wait_max_sec = float(
            self.declare_parameter('side_lock_wait_max_sec', 10.0).value)
        # 정수 — int() 로 감싸지 않는다(소수·bool 이면 가드가 잡게). 쓸 때는 SideLockFilter 가 int.
        self.side_lock_confirm_frames = self.declare_parameter('side_lock_confirm_frames', 3).value
        self.side_lock_release_margin_m = float(
            self.declare_parameter('side_lock_release_margin_m', 0.3).value)
        bad = check_params(self.side_lock_enabled, self.side_lock_dist_m, self.side_lock_min_deg,
                           self.side_lock_max_deg, self.side_lock_turn_deg,
                           self.side_lock_wait_max_sec, self.side_lock_confirm_frames,
                           self.side_lock_release_margin_m)
        if bad:
            self.get_logger().error(f"옆 잠금 파라미터 이상({'; '.join(bad)}) → 옆 잠금 끔")
            self.side_lock_enabled = False
        self._side_lock_wait = None      # 켰을 때만 만든다(후보 0 개 대기 상태, §9-7)
        self._side_lock_filter = None    # 켰을 때만 만든다(연속 프레임 확인 + 풀림 히스테리시스)
        self._side_lock_last_scan = None  # 마지막으로 센 스캔 메시지(같은 스캔 재처리는 안 셈)
        self._side_lock_seen = False     # 직전 _compute 틱이 옆 잠금을 평가한 (C) 틱이었나
        self._side_lock_prev = (False, False)   # 직전 (좌, 우) 잠김 — 잠김 INFO 용(제어엔 안 씀)
        self._side_lock_unknown_warned = False  # 옆 구역 '모름' WARN 1회
        if self.side_lock_enabled:
            self._side_lock_wait = SideLockWait(self.side_lock_wait_max_sec)
            self._side_lock_filter = SideLockFilter(self.side_lock_confirm_frames,
                                                    self.side_lock_release_margin_m)
            self.get_logger().info(
                f"옆 잠금 켬: dist {self.side_lock_dist_m} m, 옆 구역 ±{self.side_lock_min_deg}~"
                f"{self.side_lock_max_deg}°, turn {self.side_lock_turn_deg}°, "
                f"wait {self.side_lock_wait_max_sec} s, 확인 {self.side_lock_confirm_frames}프레임, "
                f"풀림 여유 {self.side_lock_release_margin_m} m")

        # ---- [P] 카메라 장애물 병합 (계약서_카메라장애물_20261111 U1, Q4·Q5·Q7) ----
        # /camera_obstacle_state('가짜 LiDAR', 좌 양수)를 (C) 에서 geofence 병합 바로 뒤에 min 병합.
        # 끔(기본) = 기존과 비트 동일. 모드는 5·8 만 허용(밖이면 빼고 WARN, 다 빠지면 끔).
        # stale: 수신 후 이 시간 넘게 갱신 없으면 병합 안 함(비유한·≤0 → 0.5 + WARN).
        # 🚨 대회 launch 가 yaml 을 넘긴다 → yaml 과 같이 바꿀 것. 켜는 건 물 위 확인 뒤 토너먼트 yaml 만(U7).
        cam_enabled_raw = self.declare_parameter('camera_obstacle_enabled', False).value
        cam_modes_raw = self.declare_parameter('camera_obstacle_modes', [8]).value
        cam_stale_raw = self.declare_parameter('camera_obstacle_stale_sec', 0.5).value
        self.camera_obstacle_enabled, err = check_camera_enabled(cam_enabled_raw)
        if err:
            self.get_logger().warn(err)
        self.camera_obstacle_modes, dropped = check_camera_modes(cam_modes_raw)
        if dropped:
            self.get_logger().warn(
                f"camera_obstacle_modes 의 {dropped} 는 허용({list(CAMERA_ALLOWED_MODES)}) 밖 → 제거, "
                f"남은 {self.camera_obstacle_modes}")
        if self.camera_obstacle_enabled and not self.camera_obstacle_modes:
            self.get_logger().warn("camera_obstacle_modes 에 허용 모드가 하나도 없음 → 카메라 병합 끔")
            self.camera_obstacle_enabled = False
        self.camera_obstacle_stale_sec, err = check_camera_stale_sec(cam_stale_raw)
        if err:
            self.get_logger().warn(err)
        if self.camera_obstacle_enabled:
            self.get_logger().info(
                f"카메라 장애물 병합 켬: 모드 {self.camera_obstacle_modes}, "
                f"stale {self.camera_obstacle_stale_sec} s (감속 신호·페일세이프엔 안 넣음)")

        self.detection_distance = self.detection_distance_default

        # ----------------------
        #      Subscriptions
        # ----------------------
        self.create_subscription(LaserScan, '/scan', self.scan_callback, SCAN_QOS)
        self.create_subscription(Float32, '/yaw_error', self.yaw_error_callback, 10)
        self.create_subscription(Float32, '/candidate_angle', self.candidate_callback, 10)
        self.create_subscription(Int32, '/wp_mode', self.wp_mode_cb, 10)
        self.create_subscription(Float32MultiArray, '/geofence_state', self.geofence_cb, 10)
        # [P] 끔이어도 구독은 만든다 — 발행자(카메라 노드)가 '구독자 0' 을 겪지 않게(계약서 U1).
        self.create_subscription(Float32MultiArray, '/camera_obstacle_state',
                                 self.camera_obstacle_cb, CAMERA_OBSTACLE_QOS)

        # ----------------------
        #       Publishers  (토픽/타입 불변 + /failsafe_level 신규, CLAUDE.md 3-9)
        # ----------------------
        self.pub_desired_angle = self.create_publisher(Float32, '/desired_angle', 10)
        self.pub_obstacle_distance = self.create_publisher(Float32MultiArray, '/obstacle_distance_array', 10)
        # D6: median 필터가 버린 고립 스파이크 수 (관측 전용, 신규 토픽 — 계약 불변).
        #   제어에 안 쓴다. blackbox 가 CSV 에 기록해 '노이즈 수 ↔ 기상 열화' 상관 검증용.
        self.pub_reject = self.create_publisher(Int32, '/obstacle_reject_count', 10)
        self.pub_failsafe = self.create_publisher(Int32, '/failsafe_level', 10)

        # ----------------------
        #       State
        # ----------------------
        self.wp_mode = -1
        self.wp4_enter_time = None
        # [N] None = 부팅 뒤 /yaw_error 를 아직 한 번도 못 받음(0.0 아님 — 0.0 은 '정확히 정렬' 정상값).
        #   이 상태에서 candidate 20000 이면 control_cb 가 STOP_HOLD 로 바꾼다.
        self.yaw_error = None
        # [N] 로그 시점(진입 ERROR 1회·5 s 주기 ERROR·복구 INFO). control_cb 에서만 쓴다.
        self.yaw_missing_log = CandidateStaleLog(YAW_MISSING_LOG_PERIOD_SEC)
        self.candidate_angle = CANDIDATE_INVALID
        self.candidate_t = None       # candidate 마지막 수신 시각(monotonic). None = 아직 못 받음
        # candidate stale 로그 시점(진입 ERROR 1회·5 s 주기 ERROR·복구 INFO). control_cb 에서만 쓴다.
        self.candidate_stale_log = CandidateStaleLog(CANDIDATE_STALE_LOG_PERIOD_SEC)

        self.latest_scan = None       # scan_cb 는 저장만 한다

        # [L] 후진 상태기계
        # READY: 새 후진 시작 가능 / REVERSING: 일정 시간 후진 / COOLDOWN: STOP 재탐색
        self.reverse_state = 'READY'
        self.reverse_start_time = None
        self.reverse_cooldown_start_time = None

        # (C) 편 유지 기억: 직전 틱 (C) 선택각(리미터 전). None = 기억 없음.
        #   (C) 를 타는 틱에서만 유지 — (0)(A)(B)·STOP·레벨2·wp_mode 변경 틱엔 None (발견 8).
        self._side_prev = None
        self._side_wp_mode = None     # wp_mode 변경 감지용(control_cb 스냅샷 비교)
        # (C) 비킴 중인가(직전 _compute 틱). 진입 INFO 1회용 — 제어엔 안 쓴다 (발견 9).
        self._side_escape_active = False

        self.prev_final_angle = None  # [K] rate limiter 상태. None = "직전 값 없음/특수신호였음" → 다음 정상값에 즉시 스냅

        # geofence: (거리 m, 상대방위 deg) 또는 None. north 가 [inf,nan] 을 내면 None.
        self.geofence = None
        self.last_geofence_t = None

        # [P] 카메라 장애물: (a0, inc, [r...]) 또는 None(빈 배열). 수신 시각 None = 아직 못 받음.
        self.camera_obstacle = None
        self.camera_obstacle_t = None
        self._camera_active_since = None   # 켜짐 + 담당 모드에 들어온 시각(침묵 판정 기준). control_cb 가 갱신
        # 침묵 로그 시점(진입 ERROR 1회·5 s 주기 ERROR·복구 INFO). (C) 틱에서만 쓴다.
        self.camera_obstacle_log = CandidateStaleLog(CAMERA_SILENT_LOG_PERIOD_SEC)

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
        상대방위는 **좌 양수**(인덱스 0 = −80 = 우현) — LiDAR a−80·/candidate_angle 과 동일.
        경계를 '가짜 LiDAR'로 쏜 것이다. **빈 배열 = 경계 정보 없음**
        (north 가 '모르면 입을 다무는' 계약: 폴리곤 미설정 / IMU stale / 이미 이탈)."""
        d = msg.data
        with self.lock:
            if len(d) >= 3 and math.isfinite(d[1]) and d[1] > 0.0:
                self.geofence = (float(d[0]), float(d[1]), [float(x) for x in d[2:]])
            else:
                self.geofence = None
            self.last_geofence_t = time.monotonic()

    def camera_obstacle_cb(self, msg):
        """[P] /camera_obstacle_state = [angle_min_deg, angle_inc_deg, r0, r1, ...] — 저장만.

        형식·부호는 /geofence_state 와 같다(좌 양수, 인덱스 0 = 우현, 멀면 inf, LiDAR 원점).
        **빈 배열 = 정보 없음**(카메라 노드: 영상 없음·파라미터 무효·프레임 비정상) → None 으로 저장.
        병합·로그는 _compute (C) 가 한다. 끔이어도 저장은 한다(쓰지 않을 뿐).
        """
        cam = parse_state(msg.data)
        with self.lock:
            self.camera_obstacle = cam
            self.camera_obstacle_t = time.monotonic()

    def candidate_callback(self, msg):
        val = msg.data
        with self.lock:
            if val in (CANDIDATE_INVALID, STOP_HOLD, SPIN_RIGHT, SPIN_LEFT):
                self.candidate_angle = val
            else:
                self.candidate_angle = (val % 360.0)
            # 수신 시각 — 신선도 판정용([D] monotonic): 전체 candidate_stale_sec(control_cb)
            #   + (B) 후진(180) candidate_reverse_stale_sec(_compute)
            self.candidate_t = time.monotonic()

    def wp_mode_cb(self, msg):
        with self.lock:
            prev = self.wp_mode
            self.wp_mode = msg.data

            # 모드가 바뀌면 이전 모드 발행자의 candidate 를 버린다 (계약서_전체코스_20261104 §2-6).
            # 새 모드 발행자가 낼 때까지 20000(GPS 주행+회피) = 부팅 초기값과 같은 상태.
            if prev != self.wp_mode:
                self.candidate_angle = CANDIDATE_INVALID
                self.candidate_t = None

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
            # 🚨 로거 호출을 변수에 담아 한 줄에서 warn/info 를 번갈아 쓰면 안 된다.
            #    rclpy 는 로그 호출을 **소스 위치(파일:줄)** 로 캐싱해서, 같은 줄이 다른 심각도로
            #    불리면 ValueError('Logger severity cannot be changed between calls.') 를 던진다.
            #    레벨 상승은 warn, 복구는 info 라서 **복구되는 순간 노드가 죽었다** (실물에서 발견).
            #    → 센서가 고장나서가 아니라 '고장에서 회복해서' 배가 멈추는 최악의 형태였다.
            #    호출 지점을 둘로 나눠서 각 줄이 한 가지 심각도만 쓰게 한다.
            msg = (f"페일세이프 L{prev} → L{level}"
                   + (f"  (원인: {worst})" if worst else "  (복구)"))
            if level > prev:
                self.get_logger().warn(msg)
            else:
                self.get_logger().info(msg)

        self.failsafe_level = level
        self.pub_failsafe.publish(Int32(data=int(level)))

    # =====================================================
    # [K] RATE LIMITER — 0~160°(정상 조향값)에서만 변화율 제한
    # =====================================================
    def _apply_rate_limit(self, target_angle):
        """0~160° 구간일 때만 한 tick당 변화폭을 제한한다.

        특수신호는 즉시 통과시키고 상태를 초기화한다. NaN/inf는 STOP_HOLD로 바꾼다.
        """
        if not math.isfinite(target_angle):
            self.prev_final_angle = None
            return STOP_HOLD

        in_range = 0.0 <= target_angle <= 160.0

        if not in_range:
            self.prev_final_angle = None
            return target_angle

        if self.prev_final_angle is None or not (0.0 <= self.prev_final_angle <= 160.0):
            self.prev_final_angle = target_angle
            return target_angle

        limit = self.rate_limit_max_deg_per_tick
        if not math.isfinite(limit) or limit <= 0.0:
            self.prev_final_angle = target_angle
            return target_angle

        diff = target_angle - self.prev_final_angle
        diff = max(-limit, min(limit, diff))
        limited = self.prev_final_angle + diff
        self.prev_final_angle = limited
        return limited

    # =====================================================
    # [L] REVERSE STATE MACHINE
    # =====================================================
    def _apply_reverse_sequence(self, now):
        """경로가 없을 때 일정 시간 후진한 뒤 STOP 재탐색을 수행한다."""
        if self.reverse_state == 'REVERSING':
            if now - self.reverse_start_time < self.reverse_duration:
                return REVERSE_ANGLE

            self.reverse_state = 'COOLDOWN'
            self.reverse_start_time = None
            self.reverse_cooldown_start_time = now
            self.get_logger().info(
                f"후진 {self.reverse_duration:.2f}초 완료 → "
                f"STOP 재탐색 {self.reverse_cooldown:.2f}초 시작",
                throttle_duration_sec=1.0)
            return STOP_HOLD

        if self.reverse_state == 'COOLDOWN':
            if now - self.reverse_cooldown_start_time < self.reverse_cooldown:
                return STOP_HOLD

            self.reverse_state = 'REVERSING'
            self.reverse_start_time = now
            self.reverse_cooldown_start_time = None
            self.get_logger().warn(
                f"STOP 재탐색 {self.reverse_cooldown:.2f}초 후에도 경로 없음 → 후진 재시작",
                throttle_duration_sec=1.0)
            return REVERSE_ANGLE

        self.reverse_state = 'REVERSING'
        self.reverse_start_time = now
        self.reverse_cooldown_start_time = None
        self.get_logger().warn(
            f"회피 경로 없음 → {self.reverse_duration:.2f}초 후진 시작",
            throttle_duration_sec=1.0)
        return REVERSE_ANGLE

    def _reset_reverse_sequence(self):
        """정상 경로나 외부 미션 명령이 생기면 후진/쿨다운 상태를 해제한다."""
        self.reverse_state = 'READY'
        self.reverse_start_time = None
        self.reverse_cooldown_start_time = None

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
            candidate_t = self.candidate_t
            yaw_error = self.yaw_error
            detection_distance = self.detection_distance
            wp_mode = self.wp_mode

        now = time.monotonic()

        # (C) 편 유지 기억은 wp_mode 가 바뀐 틱에 버린다 (구간이 바뀌면 직전 편은 의미 없음).
        if wp_mode != getattr(self, '_side_wp_mode', wp_mode):
            self._side_prev = None
        self._side_wp_mode = wp_mode
        # DWA 구간을 벗어난 틱 → 다음 진입 때 추정 초기화(결정 6). 끔이면 _dwa_active 가 늘 False.
        if getattr(self, '_dwa_active', False) and wp_mode not in self.dwa_modes:
            self._dwa_leave()
        # [P] 카메라 담당 구간 진입 시각(침묵 판정 기준) — 끔이면 아무것도 안 함.
        if getattr(self, 'camera_obstacle_enabled', False):
            self._camera_track_active(wp_mode, now)

        # ---- candidate 신선도: candidate_stale_sec 넘게 갱신 없으면 20000 (_compute 전) ----
        candidate_angle = self._effective_candidate(
            candidate_angle, candidate_t, now, wp_mode)

        # ---- 레벨 2(정지) 또는 이상 스캔: STOP_HOLD 로 override(_compute 건너뜀). ----
        # 페일세이프 레벨로 덮어쓰는 건 여기뿐 — 레벨과 별개인 정지는 아래 [N](yaw 미수신+20000)·
        # _compute (0)/[I].
        # bad 를 여기서도 보는 이유: 워치독 틱(0.1s)을 기다리지 않고 즉시 멈추기 위해서다.
        if level >= 2 or bad:
            self._side_prev = None
            self._side_escape_active = False
            self._side_lock_seen = False     # 옆 잠금 대기 시간은 연속 (C) 틱으로만 센다
            self.pub_obstacle_distance.publish(self._closest_obstacle(scan))
            angle = self._apply_rate_limit(STOP_HOLD)   # [K] 범위 밖 값 → 통과 + 상태 리셋
            self.pub_desired_angle.publish(Float32(data=angle))
            return

        # 스캔을 한 번도 못 받음(부팅 중) → 발행할 각도가 없다.
        # motor_control 은 '첫 명령 전 중립'이라 안전하다.
        if scan is None:
            self._side_prev = None
            self._side_escape_active = False
            self._side_lock_seen = False
            return

        # ---- [N] 20000(갭 선택)인데 yaw_error 를 한 번도 못 받음 → STOP_HOLD (_compute 전) ----
        # 50000 은 _compute (0) 경로 → 후진 상태기계 리셋, 레이트 리미터도 범위 밖 값이라 리셋.
        # 로그: 막힌 틱은 update(True)(진입·주기 ERROR), yaw_error 가 생긴 뒤 틱은 update(False)(첫 번만
        # 복구 INFO), 그 밖의 틱(yaw 없음 + 각도 등)은 안 부른다 → 20000 ↔ 각도 깜빡임에도 ERROR 폭주
        # 없음. 심각도별 호출 줄 분리(§1-7).
        blocked = heading_missing(candidate_angle, yaw_error, CANDIDATE_INVALID)
        if blocked:
            ev = self.yaw_missing_log.update(True, now)
        elif yaw_error is not None:
            ev = self.yaw_missing_log.update(False, now)
        else:
            ev = None
        if ev == 'enter':
            self.get_logger().error(
                f"/yaw_error 를 부팅 뒤 한 번도 받지 못함 + candidate 20000(갭 선택, wp_mode {wp_mode}) "
                f"→ 목표 방위 없음, 50000 정지. GPS fix·RTK·IMU·ship_goal_angle 확인 "
                f"(/failsafe_level 은 0 그대로)")
        elif ev == 'repeat':
            self.get_logger().error(
                f"/yaw_error 미수신 지속 (wp_mode {wp_mode}) → 20000 대신 50000 유지")
        elif ev == 'recover':
            self.get_logger().info("/yaw_error 첫 수신 → 20000(갭 선택)도 목표 방위로 계산")
        if blocked:
            candidate_angle = STOP_HOLD

        # ---- 레벨 0·1: 계산한 각도를 그대로 낸다. ----
        # 레벨 1(경고)에서도 각도는 안 건드린다. 속도만 줄인다
        # → motor_control 이 /failsafe_level 을 보고 상한(70%)을 건다.
        angle, obst = self._compute(scan, wp4_enter_time, candidate_angle,
                                    yaw_error, detection_distance, candidate_t)
        angle = self._apply_rate_limit(angle)   # [K] 0~160° 안일 때만 실제로 제한됨

        self.pub_obstacle_distance.publish(obst)
        self.pub_desired_angle.publish(Float32(data=angle))

    def _effective_candidate(self, candidate_angle, candidate_t, now, wp_mode):
        """effective_candidate()(failsafe.py, 순수) + stale 로그. → _compute 에 넘길 candidate.

        수신 후 candidate_stale_sec 넘게 갱신 없음 → 20000. 수신 기록 없음(None) → 20000(로그 없음).
        로그(CLAUDE.md §1-7): 심각도마다 호출 줄을 따로 둔다 — 진입 ERROR 1회, 지속 중
        CANDIDATE_STALE_LOG_PERIOD_SEC 주기 ERROR, 복구(새 수신·wp_mode 변경 초기화) INFO 1회.
        """
        stale = candidate_is_stale(candidate_t, now, self.candidate_stale_sec)
        ev = self.candidate_stale_log.update(stale, now)
        age = None if candidate_t is None else now - candidate_t
        if ev == 'enter':
            self.get_logger().error(
                f"candidate {age:.1f}s 갱신 없음(> {self.candidate_stale_sec:.1f}s, "
                f"마지막 값 {candidate_angle:.1f}, wp_mode {wp_mode}) → 20000(GPS 주행+회피). "
                f"발행 노드 사망 의심")
        elif ev == 'repeat':
            self.get_logger().error(
                f"candidate stale 지속 {age:.1f}s (wp_mode {wp_mode}) → 20000 유지")
        elif ev == 'recover':
            how = "새 수신" if candidate_t is not None else "wp_mode 변경 초기화"
            self.get_logger().info(f"candidate stale 해제({how}, wp_mode {wp_mode})")
        return effective_candidate(candidate_angle, candidate_t, now,
                                   self.candidate_stale_sec, CANDIDATE_INVALID)

    # =====================================================
    # 계산 (회피 알고리즘 — 작년 로직 보존)
    # =====================================================
    def _closest_obstacle(self, msg):
        """전방 0~160° 최근접 → Float32MultiArray([거리(m), 각도(deg)]). 없으면 [inf, nan]."""
        obst = Float32MultiArray()
        if msg is None:
            obst.data = [float('inf'), float('nan')]
            self.pub_reject.publish(Int32(data=0))     # D6: 스캔 없음 = 기각 0
            return obst

        angle_min = math.degrees(msg.angle_min)
        angle_increment_deg = math.degrees(msg.angle_increment)
        ranges = msg.ranges

        # [F] 0 나눗셈 방어 (scan_callback 이 이미 걸러내지만, 여기도 막아둔다)
        if not math.isfinite(angle_increment_deg) or angle_increment_deg <= 0.0:
            obst.data = [float('inf'), float('nan')]
            self.pub_reject.publish(Int32(data=0))
            return obst

        min_index = int((0 - angle_min) / angle_increment_deg)
        max_index = int((160 - angle_min) / angle_increment_deg)

        # ---- [J] 공간 median 후 최소거리 (감속 신호 전용) ----
        # 작년엔 필터 없는 raw min 이라 물보라 반사 한 점(0.3m)이 그대로 motor_control 감속을
        # 물렸다. median 은 이웃과 어긋나는 고립 스파이크를 지운다.
        # ★ 여기(감속 신호)에만 건다 — _compute 의 회피 마스크는 손대지 않는다(접촉 성능 불변).
        # D6: 기각수(rejected)를 함께 받아 /obstacle_reject_count 로 낸다. 관측 전용 — 제어 불변.
        d, i, rejected = median_min(
            ranges, min_index, max_index, self.obst_median_kernel, return_rejected=True)
        self.pub_reject.publish(Int32(data=int(rejected)))
        if d is None:
            obst.data = [float('inf'), float('nan')]
        else:
            obst.data = [d, angle_min + i * angle_increment_deg]
        return obst

    def _merge_geofence(self, distance_array, angle_array):
        """경계 '가짜 LiDAR'를 실제 스캔과 병합한다 — **한 줄이 전부다**:
              ranges[i] = min(real_ranges[i], geofence_ranges[i])

        🚨 부호 규약 (계약서_geofence부호_20261104): geofence 상대방위 = LiDAR `a − 80` 과 **같은 좌 양수**.
           배열 인덱스 0 (rel −80) = 우현 = LiDAR 0°, 끝 인덱스 (+80) = 좌현 = LiDAR 160°.
           north 가 이 규약으로 발행하므로 여기서는 부호 변환을 하지 않는다 (test_geofence_merge.py 가 고정).

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
            rel = a - LIDAR_FORWARD_DEG          # LiDAR 프레임(80=정면) → 상대방위(0=정면, 좌 양수 그대로)
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
        🚨 부호 규약: best_rel(geofence, 인덱스 0 = −80 = 우현)과 intended_rel(/candidate_angle, 좌 양수)은
           **같은 좌 양수** 라 그대로 뺀다. north 가 그 규약으로 낸다 (계약서_geofence부호_20261104).
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

    # =====================================================
    # [P] 카메라 장애물 병합 (계약서_카메라장애물_20261111 U1) — 계산은 ship_direction/camera_merge.py
    # =====================================================
    def _camera_track_active(self, wp_mode, now):
        """켜짐일 때 control_cb 가 매 틱 부른다: 담당 모드 진입 시각을 기록, 벗어나면 지운다.

        진입 시각을 침묵 판정 기준에 넣는 이유: 카메라 노드는 담당 모드 밖에선 발행하지 않는다 →
        mode 8 진입 순간엔 마지막 수신이 늘 묵었다. 진입부터 stale_sec 은 기다려야 거짓 ERROR 가 없다.
        벗어날 때 침묵 로그 상태도 조용히 처음으로(다음 진입 첫 틱의 거짓 '복구' INFO 방지).
        """
        if camera_active(self.camera_obstacle_enabled, wp_mode, self.camera_obstacle_modes):
            if self._camera_active_since is None:
                self._camera_active_since = now
        elif self._camera_active_since is not None:
            self._camera_active_since = None
            self.camera_obstacle_log = CandidateStaleLog(CAMERA_SILENT_LOG_PERIOD_SEC)

    def _merge_camera(self, distance_array, angle_array, now):
        """[P] (C) 에서 geofence 병합 바로 뒤에 부른다(켜짐일 때만). → 병합한(또는 그대로인) 배열.

        담당 모드(control_cb 스냅샷 _side_wp_mode)가 아니면 그대로. 담당이면 merge_camera(순수):
        stale·빈 배열·못 받음 → 그대로(옛 동작). 로그(CLAUDE.md §1-7, 심각도마다 호출 줄 따로):
          · 침묵(진입 뒤 stale_sec 넘게 수신 없음): 진입 ERROR 1회 + 5 s 주기 ERROR, 복구 INFO 1회
          · 빈 배열(카메라 노드는 살아 있으나 정보 없음): WARN throttle 5 s
          · 병합됨: INFO throttle 2 s
        """
        mode = getattr(self, '_side_wp_mode', None)
        if not camera_active(self.camera_obstacle_enabled, mode, self.camera_obstacle_modes):
            return distance_array
        if self._camera_active_since is None:      # control_cb 를 안 거친 직접 호출
            self._camera_active_since = now
        with self.lock:
            cam = self.camera_obstacle
            cam_t = self.camera_obstacle_t
        out, status, hits, nearest = merge_camera(
            distance_array, angle_array, cam, cam_t, now, self.camera_obstacle_stale_sec,
            LIDAR_FORWARD_DEG)

        silent = camera_silent(cam_t, self._camera_active_since, now, self.camera_obstacle_stale_sec)
        ev = self.camera_obstacle_log.update(silent, now)
        if ev == 'enter':
            how = "한 번도 못 받음" if cam_t is None else f"{now - cam_t:.1f}s 갱신 없음"
            self.get_logger().error(
                f"📷 /camera_obstacle_state {how}(> {self.camera_obstacle_stale_sec:.1f}s, wp_mode {mode}) "
                f"→ 카메라 장애물 병합 안 함(LiDAR 만으로 회피). 카메라 노드·OAK 확인")
        elif ev == 'repeat':
            self.get_logger().error(
                f"📷 /camera_obstacle_state 침묵 지속 (wp_mode {mode}) → 카메라 병합 안 함")
        elif ev == 'recover' and status == ST_EMPTY:
            # 재개 첫 수신이 빈 배열 = 노드는 살아 있으나 정보 없음 → 병합 안 함을 사실대로(아래 WARN 도 뜬다)
            self.get_logger().info(f"📷 /camera_obstacle_state 수신 재개(빈 배열 — 병합 안 함) (wp_mode {mode})")
        elif ev == 'recover':
            self.get_logger().info(f"📷 /camera_obstacle_state 수신 재개 (wp_mode {mode}) → 카메라 병합")
        if status == ST_EMPTY:
            self.get_logger().warn(
                "📷 /camera_obstacle_state 빈 배열(카메라 정보 없음) → 병합 안 함",
                throttle_duration_sec=CAMERA_LOG_PERIOD_SEC)
        if hits:
            self.get_logger().info(
                f"📷 카메라 장애물이 스캔에 병합됨 ({hits}셀, 최근접 {nearest:.1f}m)",
                throttle_duration_sec=2.0)
        return out

    def _compute(self, msg, wp4_enter_time, candidate_angle, yaw_error, detection_distance,
                 candidate_t=None):
        """(desired_angle, obstacle_array) 를 만든다. 작년 scan_callback 의 로직 그대로.

        candidate_t: candidate 수신 시각(monotonic) 스냅샷. (B) 후진(180) 신선도에만 쓴다.
        None(못 받음)이면 180 은 묵은 것으로 보고 STOP_HOLD — 모르면 입을 다문다.
        """
        now = time.monotonic()

        # (C) 편 유지 기억: 꺼내 두고 None 으로 비운다 → (C) 에서 후보를 골랐을 때만 다시 채운다.
        #   (0)(A)(B)·[I]·후보 0 개 틱은 None 으로 남는다. 가짜 self(테스트) 호환 위해 getattr.
        side_prev = getattr(self, '_side_prev', None)
        self._side_prev = None
        # (C) 비킴 진입 로그용. 비킴 틱에서만 다시 True (발견 9).
        was_escaping = getattr(self, '_side_escape_active', False)
        self._side_escape_active = False
        # 옆 잠금: 직전 틱이 잠금 평가 (C) 틱이었나 — 꺼내 두고 비운다((C) 에서 평가하면 다시 True).
        lock_seen = getattr(self, '_side_lock_seen', False)
        self._side_lock_seen = False
        # DWA 구간 판정(control_cb 와 같음) — control_cb 를 안 거치는 직접 호출 대비. 가짜 self 호환 getattr.
        if getattr(self, '_dwa_active', False) and \
                getattr(self, '_side_wp_mode', None) not in self.dwa_modes:
            self._dwa_leave()

        obst = self._closest_obstacle(msg)

        # ---- (0) WP4 초기 5초 & STOP_HOLD 우선 처리 ----
        if (wp4_enter_time is not None and (now - wp4_enter_time) < 5.0) or \
           (candidate_angle == STOP_HOLD):
            self._reset_reverse_sequence()
            return STOP_HOLD, obst

        # ---- [I] Geofence 하드 가드 ((A)SPIN/(B)candidate 전용) ----
        # 이 두 구간은 마스크 병합이 없어 경계로 파고들 수 있다(도킹이 가장자리일 때).
        # (C)(CANDIDATE_INVALID)는 아래에서 _merge_geofence 로 처리되므로 제외한다.
        if candidate_angle != CANDIDATE_INVALID:
            # 전진하려는 방향: candidate 는 그 상대각, SPIN(제자리)은 전방(0°)으로 본다.
            intended_rel = 0.0 if candidate_angle in (SPIN_RIGHT, SPIN_LEFT) else candidate_angle
            if self._geofence_blocks(intended_rel):
                self._reset_reverse_sequence()
                return STOP_HOLD, obst

        # ---- (A) PASS-THROUGH: 미션 노드가 선회를 직접 지시 ----
        if candidate_angle in (SPIN_RIGHT, SPIN_LEFT):
            self._reset_reverse_sequence()
            return candidate_angle, obst

        # ---- (B) 일반 candidate_angle 매핑 (360° → 전방중심 80°) ----
        if candidate_angle != CANDIDATE_INVALID:
            self._reset_reverse_sequence()
            c_raw = candidate_angle % 360.0

            # 후진 요청: 180.0 **정확 일치**만 260(motor_control 분기 5)으로 통과 (계약서_후진통과_20261104).
            # ed31dda 의 범위 검사가 260 을 STOP_HOLD 로 바꿔 ship_dock BACKOFF·ship_back 후진이
            # 조용히 정지가 됐었다. 수신 후 candidate_reverse_stale_sec 넘게 갱신이 없으면 발행 노드가
            # 죽은 것으로 보고 정지(묵은 180 으로 무한 후진 방지). 다른 값엔 신선도를 안 건다.
            if c_raw == REVERSE_REL:
                if candidate_t is not None and \
                        (now - candidate_t) <= self.candidate_reverse_stale_sec:
                    return REVERSE_ANGLE, obst
                self.get_logger().warn(
                    f"candidate 후진(180) {self.candidate_reverse_stale_sec:.2f}s 넘게 갱신 없음 "
                    f"→ STOP_HOLD (발행 노드 사망 의심)",
                    throttle_duration_sec=1.0)
                return STOP_HOLD, obst

            c_mapped = 80.0 + c_raw if c_raw <= 180.0 else 80.0 - (360.0 - c_raw)

            # 정상 조향값은 0~160°여야 한다. 범위를 벗어나면 motor_control이 후진으로
            # 해석할 수 있으므로 안전하게 정지한다. (예외: 바로 위 180.0 정확 일치 = 의도된 후진)
            if not math.isfinite(c_mapped) or not (0.0 <= c_mapped <= 160.0):
                self.get_logger().warn(
                    f"candidate 매핑 범위 초과: raw={c_raw:.1f}°, "
                    f"mapped={c_mapped:.1f}° → STOP_HOLD",
                    throttle_duration_sec=1.0)
                return STOP_HOLD, obst
            return c_mapped, obst

        # ---- (C) candidate == 20000 → yaw_error 기반 자율 회피 ----
        # [N] 방어 가드: 목표 방위를 한 번도 못 받음(None) → 정지. control_cb 가 이미 50000 으로 바꾸지만,
        #   직접 호출(시뮬·테스트)에서 None % 360 TypeError 로 노드가 죽는 걸 막는다. 판정은 `is None`.
        if yaw_error is None:
            self._reset_reverse_sequence()
            return STOP_HOLD, obst

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
        # ---- [P] 카메라 장애물 병합 — geofence 바로 뒤, 같은 매핑 min. 끔이면 부르지도 않는다. ----
        if getattr(self, 'camera_obstacle_enabled', False):
            distance_array = self._merge_camera(distance_array, angle_array, now)

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
        # 가로지름 벌점용 장애물 덩어리 — 팽창 **전** 마스크(필터 뒤)에서 (발견 8)
        blobs = obstacle_blobs(binary, angle_array, distance_array)
        binary = self.dilate_obstacles(binary, distance_array, angle_increment_deg,
                                       detection_distance)

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
        min_required_width = self.half_width * 2 + self.clearance
        valid_safe_zones = select_valid_zones(
            safe_zones, distance_array, angle_increment_rad, detection_distance, min_required_width)

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

        # ---- (C) 옆 잠금 (계약서_토너먼트분리_20261109 T2-1) — 갭·DWA·비킴 후보를 잠금 구간으로 ----
        #   끔(기본)·잠긴 쪽 없음이면 lock_on False → cand_use 는 candidates 그대로, 아래는 기존과 같다.
        lock_on, a_lo, a_hi = False, 0.0, 160.0
        if getattr(self, 'side_lock_enabled', False):
            lock_on, a_lo, a_hi = self._side_lock_eval(msg, lock_seen)
        cand_use = candidates
        if lock_on:
            cand_use = clip_candidates(candidates, angle_array, yaw_mapped, a_lo, a_hi)

        # ---- (C) DWA (계약서_DWA_20261106 결정 7) — 비킴·후진·갭 정렬 분기보다 **먼저** ----
        #   DWA 모드가 아니거나 끔이면 (False, None) → 아래는 기존과 같다.
        #   고르면 후진 상태기계 리셋, 비킴·후진 분기는 건너뛴다(거짓 '후진 시작' WARN 없음).
        #   편 기억 _side_prev 는 갭 정렬 기준 그대로(후보 있으면 정렬 첫째, 0 개면 None).
        #   옆 잠금 중이면 잠금 구간 안 후보만(dwa.choose allowed).
        dwa_on, dwa_angle = False, None
        if getattr(self, 'dwa_enabled', False):
            dwa_on, dwa_angle = self._dwa_select(angle_array, distance_array, yaw_mapped,
                                                 (a_lo, a_hi) if lock_on else None)

        # ---- [L] 회피 경로가 없으면 후진. 팀원 피드백 C안 반영 ----
        # 작년/2단계 코드는 여기서 매 tick 무조건 REVERSE_ANGLE 을 냈다 — reverse_cooldown 은
        # 타임스탬프만 갱신할 뿐 final_angle 에 영향이 없었다(dead code, 팀 지적).
        # ⚠️ north 광선은 전방 ±80° 만 봐서 후진 중 뒤쪽 경계는 못 본다([I] 참고).
        #    즉 지금 구조로 '계속 후진'은 뒤를 못 보며 맹목 후진하는 것과 같다.
        # → C안: 후진은 쿨다운마다 '한 번만' 짧게 내고, 그 사이(쿨다운 중)는 STOP_HOLD 로
        #   멈춰서 재탐색한다. 쿨다운이 끝났는데도 여전히 후보가 없으면 다시 한 번 후진한다.
        # 발견 9 (계약서_옆비킴_현장조정_20261106): 가장 가까운 장애물이 옆(15~60°)이고 정면·호·
        #   목표 창이 비었으면 후진 대신 반대편으로 side_escape_deg 비킴. 후진·쿨다운 도중에도 끊는다
        #   (결정 ④). 비킴 틱은 편 기억 None(결정 ⑥). 게이트 구간 제외(결정 ③). 0.0 이면 기존과 같다.
        #   wp_mode 는 control_cb 가 lock 으로 뜬 스냅샷(_side_wp_mode) — detection_distance 와 같은 틱 값.
        #   직접 호출(control_cb 안 거침)은 None → 모르면 비킴 안 함.
        def escape_for(cands, dwa_a):
            if dwa_a is None and not cands and getattr(self, 'side_escape_deg', 0.0) > 0.0:
                wp_mode = getattr(self, '_side_wp_mode', None)
                if wp_mode is not None and wp_mode not in self.gate_wp_modes:
                    return side_escape(blobs, angle_array, distance_array, self.side_escape_deg,
                                       LIDAR_FORWARD_DEG)
            return None

        esc = escape_for(cand_use, dwa_angle)

        # ---- 옆 잠금 후보 0 개 (§9-7): 비킴도 잠금 구간 안이어야 한다. 남은 게 없으면 ----
        #   잠금 없이 고르면 후보(갭·DWA·비킴)가 있었는지 본다 — 있었으면 **잠금이 지운 것**:
        #   ② 후진하지 않고 STOP_HOLD 로 매 틱 재평가 ③ side_lock_wait_max_sec 넘으면 기존 동작(폴백).
        #   잠금 없이도 0 개(막다른 곳)면 대기하지 않고 기존 동작(후진 시퀀스) 그대로.
        #   잠금 없는 DWA 재계산은 이 판단이 필요한 틱(잠금 지키는 후보 0 개)에서만. 끔이면 이 블록 없음.
        lock_act = 'go'
        if lock_on:
            if esc is not None and not angle_allowed(esc, a_lo, a_hi):
                esc = None
            blocked, dwa_plain, esc_plain = False, None, None
            if dwa_angle is None and not cand_use and esc is None:
                if dwa_on:
                    dwa_plain = self._dwa_choose_plain(angle_array, distance_array, yaw_mapped)
                esc_plain = escape_for(candidates, dwa_plain)
                blocked = bool(candidates) or dwa_plain is not None or esc_plain is not None
            lock_act = self._side_lock_step(blocked, now)
            if lock_act == 'fallback':
                # 기존 동작 그대로 — 잠금 없이 고른 갭·DWA·비킴.
                cand_use, dwa_angle, esc = candidates, dwa_plain, esc_plain
            if dwa_on:
                # DWA 선택 기록·폴백 INFO 는 실제로 쓴 값 기준(대기 틱 = 안 고름, 로그 없음)
                self._dwa_settle(None if lock_act == 'wait' else dwa_angle,
                                 log=lock_act != 'wait')
        elif getattr(self, 'side_lock_enabled', False):
            self._side_lock_step(False, now)

        if lock_act == 'wait':
            self._reset_reverse_sequence()
            final_angle = STOP_HOLD
        elif dwa_angle is not None:
            self._reset_reverse_sequence()
            final_angle = dwa_angle
            if cand_use:
                ranked = rank_candidates(cand_use, side_prev, blobs,
                                         self.side_hysteresis_deg, self.cross_penalty_deg,
                                         LIDAR_FORWARD_DEG)
                self._side_prev = next_side_memory(ranked[0][2], len(cand_use))
        elif not cand_use and esc is not None:
            self._reset_reverse_sequence()
            final_angle = esc
            self._side_escape_active = True
            if not was_escaping:
                c, d = min(blobs, key=lambda b: b[1])
                self.get_logger().info(
                    f"회피 경로 없음 + 장애물 옆(중심 {c:.0f}°, {d:.2f} m) "
                    f"→ 후진 대신 {esc:.0f}° 로 비킴",
                    throttle_duration_sec=1.0)
        elif not cand_use:
            # 일정 시간 후진 → 일정 시간 STOP 재탐색 → 필요하면 반복
            final_angle = self._apply_reverse_sequence(now)
        else:
            self._reset_reverse_sequence()
            # 편 유지·가로지름 벌점을 더한 정렬 (발견 8, 계약서_회피편선택_20261106).
            #   벌점 0.0/0.0 이면 기존 (angle_diff, -arc) 정렬과 같다.
            ranked = rank_candidates(cand_use, side_prev, blobs,
                                     self.side_hysteresis_deg, self.cross_penalty_deg,
                                     LIDAR_FORWARD_DEG)
            final_angle = ranked[0][2]
            self._side_prev = next_side_memory(final_angle, len(cand_use))

        if dwa_on:
            self._dwa_observe(final_angle)     # 리미터 전 값(폴백 포함) — 시제품과 같다
        return final_angle, obst

    # =====================================================
    # (C) 옆 잠금 보조 (계약서_토너먼트분리_20261109 T2-1) — 계산은 ship_direction/side_lock.py
    # =====================================================
    def _side_lock_eval(self, msg, lock_seen):
        """이번 스캔의 옆 구역 판정 → (잠긴 쪽 있나, a_lo, a_hi). 켰을 때 (C) 틱에서만 부른다.

        새 스캔(메시지 객체가 바뀜)만 연속 프레임으로 센다 — 제어 10 Hz 가 같은 스캔을 두 번 볼 수 있다.

        한 바퀴 스캔 전체를 본다(0~160 창 밖). 스캔이 그 구역을 못 덮으면 '모름' → 안 잠금 + WARN 1회.
        직전 틱이 잠금 평가 (C) 틱이 아니었으면((0)(A)(B)·레벨2·스캔 없음) 대기 상태만 조용히 처음으로
        (잠금 필터·잠김 상태는 유지).
        """
        if not lock_seen:
            # 대기 시간만 처음부터. 잠금 필터는 유지 — 한 틱 끊겼다고 잠금을 풀면 재출발 직후
            # 확인 프레임 동안(0.3 s) 잠금 공백이 생긴다. '모름' 초기화는 필터가 따로 한다.
            self._side_lock_wait.reset()
        self._side_lock_seen = True
        new_frame = msg is not self._side_lock_last_scan
        self._side_lock_last_scan = msg
        (ls, ld), (rs, rd) = side_states(
            msg.ranges, math.degrees(msg.angle_min), math.degrees(msg.angle_increment),
            LIDAR_FORWARD_DEG, self.side_lock_dist_m, self.side_lock_min_deg,
            self.side_lock_max_deg, getattr(msg, 'range_min', 0.0))
        if UNKNOWN in (ls, rs) and not self._side_lock_unknown_warned:
            self._side_lock_unknown_warned = True
            self.get_logger().warn(
                f"옆 잠금: 스캔이 옆 구역을 다 덮지 않음(좌 {ls}, 우 {rs}, angle_min "
                f"{math.degrees(msg.angle_min):.1f}°, {len(msg.ranges)}셀) → 그쪽은 안 잠금")
        # 순간 판정(LOCKED = 최솟값 < dist) → 연속 프레임 확인·풀림 히스테리시스
        left, right = self._side_lock_filter.update(((ls, ld), (rs, rd)), self.side_lock_dist_m,
                                                    new_frame)
        a_lo, a_hi = allowed_range(left, right, LIDAR_FORWARD_DEG, self.side_lock_turn_deg)
        if (left or right) and (left, right) != self._side_lock_prev:
            sides = ' '.join(f"{n} {d:.2f} m" for n, on, d in (('좌', left, ld), ('우', right, rd))
                             if on)
            self.get_logger().info(
                f"옆 잠금 걸림: {sides} → 조향 {a_lo:.0f}~{a_hi:.0f}° 안에서만",
                throttle_duration_sec=2.0)
        self._side_lock_prev = (left, right)
        return (left or right), a_lo, a_hi

    def _side_lock_step(self, blocked, now):
        """대기 상태기계(SideLockWait) 한 걸음 + 전이 로그(§1-7: 심각도마다 호출 줄을 따로) → 동작."""
        w = self._side_lock_wait
        t0 = w.wait_t0
        act, ev = w.update(blocked, now)
        if ev == 'wait_start':
            self.get_logger().info(
                f"옆 잠금: 잠금이 후보를 지움 → 후진 대신 50000 대기(매 틱 재평가, "
                f"상한 {self.side_lock_wait_max_sec:.1f} s)",
                throttle_duration_sec=1.0)
        elif ev == 'resume':
            self.get_logger().info(
                f"옆 잠금 대기 끝({now - t0:.1f} s) → 재출발",
                throttle_duration_sec=1.0)
        elif ev == 'timeout':
            self.get_logger().warn(
                f"옆 잠금 대기 {self.side_lock_wait_max_sec:.1f} s 넘음 → 기존 동작(비킴·후진)으로 폴백",
                throttle_duration_sec=1.0)
        elif ev == 'rearm':
            self.get_logger().info(
                "옆 잠금 폴백 끝(잠금이 후보를 지우지 않음) → 다시 잠금 지킴",
                throttle_duration_sec=1.0)
        return act

    # =====================================================
    # (C) DWA 보조 (계약서_DWA_20261106) — 계산은 ship_direction/dwa.py
    # =====================================================
    def _dwa_leave(self):
        """DWA 구간 상태를 내린다 → 다음 DWA 모드 진입 때 reset + INFO(결정 6)."""
        self._dwa_active = False
        self._dwa_picked = False
        self._dwa_last_t = None
        self._dwa_needs_reset = False

    def _dwa_select(self, angle_array, distance_array, yaw_mapped, allowed=None):
        """→ (DWA 모드 (C) 틱인가, 선택각 또는 None). 끔·모드 밖이면 (False, None), 아무것도 안 함.

        allowed = (a_lo, a_hi) 면 옆 잠금 구간 안 후보만 고른다(None = 기존 호출 그대로).

        예외는 잡아서 None(기존 결과) + ERROR. 1회 > DWA_SLOW_WARN_SEC 면 WARN(결정 12).
        로그(CLAUDE.md §1-7): 심각도마다 호출 줄을 따로 둔다.
        """
        mode = getattr(self, '_side_wp_mode', None)
        if not self.dwa_enabled or mode not in self.dwa_modes:
            return False, None
        t0 = time.monotonic()
        idle = None if self._dwa_last_t is None else t0 - self._dwa_last_t
        if not self._dwa_active:
            self._dwa.reset()
            self._dwa_active = True
            self._dwa_picked = False
            self.get_logger().info(
                f"DWA 사용(mode {mode}) — 선체 {self.dwa_hull}, 배율 {self.dwa_model_agility}")
        elif idle is not None and idle > DWA_IDLE_RESET_SEC:
            self._dwa.reset()
            self._dwa_picked = False
            self.get_logger().info(
                f"DWA (C) 공백 {idle:.1f} s > {DWA_IDLE_RESET_SEC:.1f} s → 추정 초기화(mode {mode})",
                throttle_duration_sec=DWA_LOG_PERIOD_SEC)
        elif self._dwa_needs_reset:
            self._dwa.reset()                 # observe 예외 뒤 — ERROR 는 이미 냈다(INFO 없음)
        self._dwa_needs_reset = False
        self._dwa_last_t = t0
        try:
            if allowed is None:
                angle = self._dwa.choose(angle_array, distance_array, yaw_mapped)
            else:
                angle = self._dwa.choose(angle_array, distance_array, yaw_mapped, allowed=allowed)
        except Exception as e:   # 어떤 계산 실패든 노드를 죽이지 않는다(기존 결과로)
            self.get_logger().error(
                f"DWA 계산 예외 → 기존 결과 사용: {e!r}",
                throttle_duration_sec=DWA_LOG_PERIOD_SEC)
            angle = None
        dt = time.monotonic() - t0
        if dt > DWA_SLOW_WARN_SEC:
            self.get_logger().warn(
                f"DWA 1회 {dt * 1000.0:.0f} ms > {DWA_SLOW_WARN_SEC * 1000.0:.0f} ms "
                f"(제어 주기 {self.control_period:.2f} s)",
                throttle_duration_sec=DWA_LOG_PERIOD_SEC)
        if allowed is None:      # 옆 잠금 중이면 _compute 가 실제로 쓴 값으로 _dwa_settle 를 부른다
            self._dwa_settle(angle)
        return True, angle

    def _dwa_settle(self, angle, log=True):
        """이번 틱 DWA 가 실제로 고른 각(None = 안 고름)으로 선택 기록·폴백 INFO(직전에 골랐을 때 1회)."""
        if log and angle is None and self._dwa_picked:
            self.get_logger().info(
                f"DWA 가능한 후보 없음(mode {getattr(self, '_side_wp_mode', None)}) "
                f"→ 기존 결과(갭·비킴·후진) 폴백",
                throttle_duration_sec=1.0)
        self._dwa_picked = angle is not None

    def _dwa_choose_plain(self, angle_array, distance_array, yaw_mapped):
        """옆 잠금 폴백 틱 전용 — 잠금 없이 한 번 더 고른다(추정·로그 상태는 안 건드림). 예외 → None."""
        try:
            return self._dwa.choose(angle_array, distance_array, yaw_mapped)
        except Exception as e:   # _dwa_select 와 같음
            self.get_logger().error(
                f"DWA 계산 예외(옆 잠금 폴백) → 기존 결과 사용: {e!r}",
                throttle_duration_sec=DWA_LOG_PERIOD_SEC)
            return None

    def _dwa_observe(self, final_angle):
        """이번 틱 (C) 결과(리미터 전)로 DWA 추정을 굴린다. 예외면 ERROR, 다음 DWA 틱에 초기화."""
        try:
            self._dwa.observe(final_angle)
        except Exception as e:   # 위와 같음
            self.get_logger().error(
                f"DWA 추정 갱신 예외 → 다음 DWA 틱에 추정 초기화: {e!r}",
                throttle_duration_sec=DWA_LOG_PERIOD_SEC)
            self._dwa_needs_reset = True

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

    def dilate_obstacles(self, binary, distance_array, angle_increment_deg,
                         detection_distance):
        # 팀원 제안 버그2: 작년엔 여기서 self.detection_distance 를 직접 읽어, control_cb 가
        #   lock 으로 뜬 스냅샷과 어긋날 수 있었다(wp_mode_cb 가 다른 스레드에서 바꾸므로).
        #   실제 영향은 거의 없다 — r_edge 는 아래에서 유효 거리로 덮이는 폴백이라, binary=1 셀은
        #   정의상 유효 거리가 있어 폴백이 안 쓰인다. 그래도 한 _compute 안에서 값이 하나로
        #   유지되도록 스냅샷을 파라미터로 받는다(lock 일관성 원칙).
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
                    r_edge = detection_distance
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
