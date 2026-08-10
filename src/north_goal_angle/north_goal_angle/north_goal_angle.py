"""north_goal_angle — 웨이포인트 FSM + 경계 이탈 방지(geofence). 6a.

고친 것:
  [3] 🚨 mode-7 폴백 제거 (ship_dock 수정과 **원자적 한 쌍**)
      작년: if wp_mode == 7: pub_candidate.publish(20000)  ← 0.5초마다
      이건 '도킹이 안 되는 걸 알고 덮어둔 흔적'이다. ship_dock 이 9 로 잡혀 침묵하니,
      배가 멈추지 않게 north 가 대신 폴백을 냈다.
      ship_dock 을 7 로 고치면 → 같은 /candidate_angle 에 발행자가 둘이 된다.
      도킹 중 조향이 GPS 방위로 튄다 (접안 직전 60초에 35° 스파이크 20번).
      → 폴백은 **담당 노드가 없는 모드에만** 남긴다: mode 5(회피), 8(토너먼트 회피).

  [6] Geofence 발행 — 경계를 **'가짜 LiDAR'** 로 낸다. 경기장 밖으로 나가면 실격인데 방어가 0 이었다.

      /geofence_state (Float32MultiArray)
        = [angle_min_deg, angle_inc_deg, r0, r1, r2, ...]   (상대방위 기준, 멀면 inf)
        정보 없으면 **빈 배열**.

      ★ 왜 '광선'인가 (원뿔 방식을 버린 이유):
        **벽은 '점'이 아니라 '선'이다.** 최근접점 한 방향으로 ±각도 원뿔을 막는 건 기하학적으로
        틀렸다. 40×40 경기장 (40,40) 모서리에서 배가 대각선(45°)을 향하면:
            북벽 최근접점 → 상대 -45°   /   동벽 최근접점 → 상대 +45°
            그런데 **탈출구(모서리)는 상대 0° — 배 정면이다.**
        half_block=40° 면 차단 구역이 [-85,-5] 와 [5,85] → **정면 0° 가 정확히 '틈'으로 열린다.**
        배가 대각선으로 탈출한다. 실격. (경기장이 사각형이 아니면 또 뚫린다.)
        광선으로 쏘면 정면 방향 경계 거리(2.12m)가 그대로 잡혀 벽이 된다. **튜닝 파라미터가 없다.**

      ★ 소비자(ship_direction)는 **스캔 병합 한 줄**이면 끝이다:
            ranges[i] = min(real_ranges[i], geofence_ranges[i])
        기존 dilate 가 배 폭 여유를 자동으로 더하고, 기존 갭-팔로잉이 알아서 피한다.
        특수 로직도 새 상태기계도 없다. "경계선을 그냥 벽으로 취급한다"를 끝까지 밀어붙인 것.

      계산량: 161방향 × 변 개수 @ 2Hz — 무시할 수준.

프레임 규약: 광선 각도는 **상대 방위**(0=정면)다 — /candidate_angle 과 같은 규약.
  ⚠️ 광선 방향이 /imu/yaw 에 통째로 의존한다 (CLAUDE.md 3-5).
     N1 에서 /imu/yaw 는 ssf_heading/yaw_mux 단독 발행으로 바뀌었다 —
     부팅 0점화와 GPS COG override 를 걷어낸 절대방위다.
     🚨 단, mount_offset_deg / invert_yaw 를 **벤치에서 아직 안 쟀다.**
        그 전까지 이 geofence 는 방향이 맞는다는 보장이 없다.
     IMU 가 묵으면 **빈 배열**을 낸다. 틀린 방향으로 '없는 벽'을 세우느니 안 내는 게 낫다.
"""

import math
import os
import time

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from std_msgs.msg import Float32, Float32MultiArray, Int32, Float64
from sensor_msgs.msg import NavSatFix
from geopy import distance
from ament_index_python.packages import get_package_share_directory

from north_goal_angle.waypoint_loader import parse_waypoints, WaypointError
from north_goal_angle.gps_guard import fix_is_usable
from north_goal_angle import los_logic          # LOS 유도 (순수, 테스트됨)
from north_goal_angle.gps_filter import GPSFilter  # 공분산·점프·스무딩 (순수, 테스트됨)

# 🚨 대회 좌표는 코드가 아니라 config/waypoints.yaml 에서 읽는다 (비전공자도 편집 가능).
#    대회장에서 GPS 좌표를 바꿀 사람은 그 파일의 lat/lon 만 고치면 된다 — 이 파일은 안 건드린다.
#    로드/검증은 waypoint_loader.parse_waypoints (순수 로직, 테스트됨).
#    파일이 없거나 형식이 틀리면 노드를 띄우지 않는다 — 틀린 좌표로 달리느니 멈춘다.

CANDIDATE_INVALID = 20000.0
ARRIVE_RADIUS_M = 3.0
DEFAULT_TIMEOUT = 120.0
TURN_TIMEOUT = 90.0

# 담당 노드가 없는 모드 = 순수 회피 구간. 여기서만 폴백을 낸다. (CLAUDE.md 3-6)
FALLBACK_MODES = (5, 8)

# 🚀 자율 시작 게이트: 아두이노 모드(ssf_bridge /boat_mode, Int32). 0=WAIT 1=MANUAL 2=AUTO.
#    상수 공유 모듈화는 대회 전 금지(CLAUDE.md 3-9) → 여기 인라인(값은 ssf_bridge status_parser 와 일치).
MODE_AUTO = 2


def mission_should_run(boat_mode, require_auto, mode_auto=MODE_AUTO):
    """미션 FSM(웨이포인트 전진)을 돌려도 되나 — ROS 비의존 순수 판정.

    require_auto=False → 항상 True (아두이노 없이 벤치 테스트용).
    require_auto=True  → boat_mode 가 AUTO(2)일 때만. WAIT/MANUAL/None(모름) 이면 False.
      → 기본이 '안 돈다'라 안전하다: RC 를 AUTO 로 넘기기 전엔 미션이 전진하지 않는다.
    """
    if not require_auto:
        return True
    return boat_mode == mode_auto

# 위경도 → 로컬 미터 근사
M_PER_DEG_LAT = 110540.0
M_PER_DEG_LON_EQ = 111320.0


class NorthGoalAngle(Node):
    def __init__(self):
        super().__init__('north_goal_angle')
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)

        # ---- 웨이포인트(GPS 좌표) 로드 — config/waypoints.yaml 에서 읽는다 ----
        # 파일 경로는 파라미터로도 바꿀 수 있다(기본: 패키지 share 의 config/waypoints.yaml).
        default_wp = os.path.join(
            get_package_share_directory('north_goal_angle'), 'config', 'waypoints.yaml')
        self.waypoints_file = str(
            self.declare_parameter('waypoints_file', default_wp).value)
        self.waypoints = self._load_waypoints(self.waypoints_file)

        # ---- 파라미터 (config/north_goal_angle.yaml, CLAUDE.md 1-4) ----
        self.period_sec = float(self.declare_parameter('period_sec', 0.5).value)

        # ---- LOS 유도 (경로선 추종 + 조류 보상 ILOS) ----
        # 직접 방위(현위치→목표)는 조류·초기오차로 밀리면 비스듬히 접근한다. LOS 는 이전 WP→목표
        # '경로 선'으로 되돌아오게 하고, ILOS 적분항이 지속 외란(조류·바람)을 상쇄한다.
        self.los_min_lookahead_m = float(self.declare_parameter('los_min_lookahead_m', 2.0).value)
        self.los_max_lookahead_m = float(self.declare_parameter('los_max_lookahead_m', 6.0).value)
        self.los_speed_gain = float(self.declare_parameter('los_speed_gain', 1.2).value)
        self.ilos_ki = float(self.declare_parameter('ilos_ki', 0.02).value)
        self.ilos_max = float(self.declare_parameter('ilos_integral_max', 50.0).value)  # anti-windup
        # GPS 필터 (fix_is_usable 통과분에 품질·이상치·지터 처리)
        self.gps_cov_threshold = float(self.declare_parameter('gps_cov_threshold', 2.0).value)
        self.gps_max_speed_mps = float(self.declare_parameter('gps_max_speed_mps', 15.0).value)

        # 🚀 자율 시작 게이트: True 면 RC 모드가 AUTO(/boat_mode==2) 여야 미션이 전진한다.
        #    기본 True = 안전(수동 이동 중 미션 스킵 방지). 벤치(아두이노 없음)는 false 로.
        self.require_auto = bool(self.declare_parameter('require_boat_mode_auto', True).value)
        # 경기장 폴리곤: [lat1, lon1, lat2, lon2, ...] 평탄 배열.
        # ⚠️ 실측 필요 — 대회장 경계 좌표를 아직 모른다. 비어 있으면 geofence 는 꺼진다.
        self.geofence_polygon = list(
            self.declare_parameter('geofence_polygon', []).value or [])
        # 경계를 '가짜 LiDAR'로 쏜다 — 스캔과 같은 각도 격자(상대방위).
        # ★ 왜 광선인가: 벽은 '점'이 아니라 '선'이다. 최근접점 한 방향으로 ±각도 원뿔을 막는 건
        #   기하학적으로 틀렸다. 40x40 모서리에서 배가 대각선(45°)을 보면 북벽 최근접점은 -45°,
        #   동벽은 +45° 인데 **탈출구(모서리)는 정면 0°** 라 원뿔 사이 '틈'으로 그대로 빠져나간다.
        #   광선으로 쏘면 정면 방향의 경계 거리(2.12m)가 그대로 잡혀 벽이 된다. 튜닝도 필요 없다.
        self.ray_min_deg = float(self.declare_parameter('geofence_ray_min_deg', -80.0).value)
        self.ray_max_deg = float(self.declare_parameter('geofence_ray_max_deg', 80.0).value)
        self.ray_inc_deg = float(self.declare_parameter('geofence_ray_inc_deg', 1.0).value)
        self.geofence_max_range_m = float(
            self.declare_parameter('geofence_max_range_m', 30.0).value)   # 이보다 멀면 inf
        # 구독자 0 경고를 시작할 시각
        self.geofence_check_after_sec = float(
            self.declare_parameter('geofence_check_after_sec', 5.0).value)
        # 🚨 IMU 신선도. geofence 상대방위는 /imu/yaw 로 계산하므로 IMU 에 의존한다.
        #    IMU 가 죽으면 yaw 가 얼어붙고 → 경계 방위가 엉뚱해지고 → ship_direction 이
        #    '없는 벽'을 칠한다 → 배가 갇힌다.
        #    → 묵으면 geofence 도 [inf, nan] 을 낸다. (1단계 ship_goal_angle 과 같은 원칙:
        #      **모르면 입을 다문다.** 틀린 값을 내느니 안 내는 게 낫다.)
        self.imu_stale_sec = float(self.declare_parameter('imu_stale_sec', 0.5).value)

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
        # 🚀 아두이노 모드(ssf_bridge 발행). AUTO 일 때만 미션 전진. RELIABLE 발행이라 qos 호환.
        self.create_subscription(Int32, '/boat_mode', self.boat_mode_cb, qos)

        # ---- 상태 ----
        self.lat, self.lon = 0.0, 0.0
        self.have_fix = False
        self.yaw = None
        self.last_yaw_t = None          # /imu/yaw 마지막 수신 (monotonic)
        self.wp_idx = 0
        self.t_start = None
        self.wp_enter_time = None
        self.t0 = time.monotonic()

        # LOS 상태
        self.gps_filter = GPSFilter(
            cov_threshold=self.gps_cov_threshold, max_speed_mps=self.gps_max_speed_mps)
        self.integral_xte = 0.0          # ILOS 적분 (WP 전환 시 리셋)
        self.prev_lat = None             # 경로선 시작점(이전 WP). None=첫 구간
        self.prev_lon = None
        self.boat_mode = None            # ssf_bridge /boat_mode (0 WAIT/1 MANUAL/2 AUTO). None=아직 모름

        self.create_timer(self.period_sec, self.timer_cb)

        gf = "OFF(폴리곤 미설정 — ⚠️ 실측 필요)" if len(self.geofence_polygon) < 6 else \
             f"ON({len(self.geofence_polygon) // 2}점)"
        self.get_logger().info(
            f"north_goal_angle 시작: 폴백은 mode {FALLBACK_MODES} 에만 발행"
            f"(mode 7 폴백 제거 — ship_dock 과 충돌했다). geofence {gf}."
        )

    # ───────────────────────── 웨이포인트 로드 ─────────────────────────
    def _load_waypoints(self, path):
        """config/waypoints.yaml 을 읽어 검증한다. 실패하면 노드를 세운다(예외).

        🚨 '모르면 침묵 말고 알린다' — 파일이 없거나 좌표가 이상하면 조용히 기본값으로
        넘어가지 않고, 어디가 왜 틀렸는지 ERROR 로그를 내고 예외를 던져 노드를 멈춘다.
        틀린 좌표로 배가 달리는 것보다 출발 전에 멈추는 게 안전하다.
        """
        try:
            with open(path, 'r', encoding='utf-8') as f:
                raw = yaml.safe_load(f)
        except FileNotFoundError:
            self.get_logger().error(
                f"🚨 웨이포인트 파일이 없다: {path}\n"
                f"   config/waypoints.yaml 이 설치됐는지 확인하라(colcon build 필요).")
            raise
        except yaml.YAMLError as e:
            self.get_logger().error(
                f"🚨 waypoints.yaml 문법 오류(콤마·중괄호·들여쓰기 확인): {e}")
            raise

        try:
            wps = parse_waypoints(raw)
        except WaypointError as e:
            self.get_logger().error(f"🚨 웨이포인트 내용 오류 → {e}")
            raise

        modes = [w[2] for w in wps]
        self.get_logger().info(
            f"웨이포인트 {len(wps)}개 로드: {path}\n   미션 순서(mode): {modes}")
        return wps

    # ───────────────────────── 콜백 ─────────────────────────
    def gps_cb(self, msg):
        # 🚨 status 를 반드시 본다. 드라이버는 fix 가 없어도 /fix 를 1Hz 로 계속 내며,
        #    그때 lat/lon 은 0.0 이다. 예전 코드는 이걸 그대로 믿고 (0,0) 기준 방위를
        #    계산해 발행했다 — 실측 47.96° = calc_angle(0, 0, WP0). 판정은 gps_guard 참고.
        if not fix_is_usable(msg.status.status, msg.latitude, msg.longitude):
            if self.have_fix:
                self._log_fix_lost(msg.status.status)
            self.have_fix = False
            return

        # fix 는 유효하다 → 품질·이상치·지터를 GPSFilter 로 거른다. 여기서 버리는 건 '한 프레임'만
        # (공분산 큼 / 순간이동)이지 fix 상실이 아니다 → have_fix·이전 위치는 유지한다.
        cov = msg.position_covariance[0] if len(msg.position_covariance) > 0 else None
        accepted, reason = self.gps_filter.update(
            msg.latitude, msg.longitude, cov, time.monotonic())
        if not accepted:
            self.get_logger().warn(f"🛰️ GPS fix 한 프레임 버림: {reason}",
                                   throttle_duration_sec=2.0)
            return
        if not self.have_fix:
            self._log_fix_acquired(msg.status.status)
        self.lat = self.gps_filter.filtered_lat
        self.lon = self.gps_filter.filtered_lon
        self.have_fix = True

    # ⚠️ 로거 호출 지점을 함수로 분리한 이유: rclpy 는 로그 호출을 **소스 위치**로 캐싱해서,
    #    같은 줄이 warn/info 로 번갈아 불리면 ValueError 로 노드가 죽는다 (CLAUDE.md §5 사례 ②).
    def _log_fix_lost(self, status):
        self.get_logger().warn(
            f"🛰️ GPS fix 상실(status={status}) → 목표 방위·거리 발행 중단. "
            "모르면 입을 다문다 (소비자는 침묵을 처리하도록 설계돼 있다)")

    def _log_fix_acquired(self, status):
        self.get_logger().info(f"🛰️ GPS fix 확보(status={status}) → 항법 재개")

    def yaw_cb(self, msg):
        self.yaw = float(msg.data)
        self.last_yaw_t = time.monotonic()

    def boat_mode_cb(self, msg):
        # 0=WAIT 1=MANUAL 2=AUTO. AUTO 로 바뀌면 미션이 WP0 부터 시작한다(timer_cb 게이트).
        prev = self.boat_mode
        self.boat_mode = int(msg.data)
        if prev != self.boat_mode and self.boat_mode == MODE_AUTO:
            self.get_logger().info("🚀 AUTO 전환 감지 → 미션 시작 (WP0 부터)")

    # ───────────────────────── Geofence ─────────────────────────
    def _local_xy(self, lat, lon):
        """보트 기준 로컬 미터 (east, north)."""
        kx = math.cos(math.radians(self.lat)) * M_PER_DEG_LON_EQ
        return ((lon - self.lon) * kx, (lat - self.lat) * M_PER_DEG_LAT)

    def _geofence_ranges(self):
        """경계를 '가짜 LiDAR'로 쏜다 → (angle_min_deg, angle_inc_deg, [r0, r1, ...]) 또는 None.

        각 상대방위마다 '이 방향으로 몇 m 가면 경계 밖인가'를 구한다. 멀면 inf.
        소비자(ship_direction)는 이걸 실제 스캔과 min() 으로 병합하기만 하면 된다 —
        특수 로직이 아예 없어진다. 기존 dilate 가 배 폭 여유를 더하고, 갭-팔로잉이 알아서 피한다.

        None(=빈 배열) 을 내는 경우 — 전부 '모르면 입을 다문다':
          · 폴리곤 미설정 / GPS fix 없음
          · IMU stale     → yaw 가 얼면 광선 방향이 통째로 틀어져 '없는 벽'을 만든다
          · 이미 경기장 밖 → 경계를 벽으로 세우면 돌아갈 길을 막는다
        """
        poly = self.geofence_polygon
        if len(poly) < 6 or not self.have_fix:      # 최소 3점(=6수)
            return None

        # 🚨 IMU 신선도 — 광선 방향이 yaw 에 통째로 의존한다. 얼면 벽 전체가 엉뚱해진다.
        if self.yaw is None or self.last_yaw_t is None:
            self.get_logger().warn(
                "/imu/yaw 없음 → geofence 침묵(빈 배열)", throttle_duration_sec=5.0)
            return None
        yaw_age = time.monotonic() - self.last_yaw_t
        if yaw_age > self.imu_stale_sec:
            self.get_logger().warn(
                f"/imu/yaw stale ({yaw_age:.2f}s > {self.imu_stale_sec}s) → geofence 침묵. "
                f"틀린 방향으로 '없는 벽'을 세우느니 안 내는 게 낫다.",
                throttle_duration_sec=2.0)
            return None

        pts = [self._local_xy(poly[i], poly[i + 1]) for i in range(0, len(poly) - 1, 2)]

        if not _point_in_polygon((0.0, 0.0), pts):
            # 🚨 이미 밖이다. 경계를 벽으로 세우면 복귀로를 막는다 → 아무것도 내지 않는다.
            self.get_logger().error(
                "🚨 경기장 경계 밖이다 (실격 위험). GPS 웨이포인트로 복귀 중.",
                throttle_duration_sec=2.0)
            return None

        if self.ray_inc_deg <= 0.0 or self.ray_max_deg < self.ray_min_deg:
            return None

        n = int(round((self.ray_max_deg - self.ray_min_deg) / self.ray_inc_deg)) + 1
        ranges = []
        for k in range(n):
            rel = self.ray_min_deg + k * self.ray_inc_deg
            th = math.radians((self.yaw + rel) % 360.0)      # 절대 방위(compass)
            d = (math.sin(th), math.cos(th))                 # (east, north) 단위벡터
            t = _ray_polygon_dist((0.0, 0.0), d, pts)
            ranges.append(float(t) if (t is not None and t <= self.geofence_max_range_m)
                          else float('inf'))

        return self.ray_min_deg, self.ray_inc_deg, ranges

    def _publish_geofence(self):
        msg = Float32MultiArray()
        gs = self._geofence_ranges()
        if gs is None:
            msg.data = []                                    # 빈 배열 = 경계 정보 없음
        else:
            a0, inc, ranges = gs
            msg.data = [float(a0), float(inc)] + [float(r) for r in ranges]
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

        if self.wp_idx >= len(self.waypoints):
            return

        wp_lat, wp_lon, wp_mode, dwell = self.waypoints[self.wp_idx]

        # 🚨 위치에서 나오는 값(거리·방위)은 fix 가 유효할 때만 낸다.
        #    fix 가 없으면 lat/lon 이 0 이거나 옛값이라 방위가 통째로 틀린다.
        #    소비자(ship_goal_angle)는 침묵을 이미 처리한다 — /yaw_error 를 멈춘다.
        #    ⚠️ /wp_mode 는 계속 낸다. 미션 단계는 '지금 위치' 에서 나오는 값이 아니고,
        #       멈추면 비전 검출기 3종이 전부 비활성이 된다(CLAUDE.md 3-4) — 부작용이 더 크다.
        #       대신 아래 도착 판정(wp_idx 전진)은 fix 없이는 하지 않는다.
        if self.have_fix:
            # 첫 구간은 이전점이 없다 → 현 위치를 경로 시작으로(=직접 접근). 이후엔 완료한 WP.
            if self.prev_lat is None:
                self.prev_lat, self.prev_lon = self.lat, self.lon

            dist = los_logic.calc_dist(self.lat, self.lon, wp_lat, wp_lon)
            self.pub_dist.publish(Float32(data=dist))

            # LOS: 경로선(prev→wp)에서 벗어난 만큼 전방주시거리 안에서 되돌아오는 방위.
            lookahead = los_logic.get_dynamic_lookahead(
                self.gps_filter.estimated_speed_mps, self.los_speed_gain,
                self.los_min_lookahead_m, self.los_max_lookahead_m)
            bearing, xte = los_logic.calc_los_bearing(
                self.prev_lat, self.prev_lon, wp_lat, wp_lon, self.lat, self.lon,
                lookahead, self.integral_xte, self.ilos_ki)
            # ILOS 적분 + anti-windup 클램프
            self.integral_xte += xte * self.period_sec
            self.integral_xte = max(-self.ilos_max, min(self.ilos_max, self.integral_xte))
            self.pub_bearing.publish(Float32(data=bearing))
        else:
            dist = None
            self.get_logger().warn(
                "🛰️ GPS fix 없음 → 목표 방위 미발행 (하늘이 안 보이거나 안테나 확인)",
                throttle_duration_sec=5.0)

        self.pub_mode.publish(Int32(data=wp_mode))

        # [3] 폴백은 '담당 노드가 없는 모드'(5, 8)에만.
        #     작년엔 mode 7(도킹)에도 냈다 → ship_dock 을 7 로 고치면 발행자가 둘이 되어
        #     도킹 중 조향이 GPS 방위로 튄다. 원자적 한 쌍이라 같은 커밋에서 고친다.
        if wp_mode in FALLBACK_MODES:
            self.pub_candidate.publish(Float32(data=CANDIDATE_INVALID))

        # 🚀 자율 시작 게이트 — AUTO 가 아니면 미션을 '전진'시키지 않는다(WP0 에서 대기).
        #    수동(RC)으로 시작점까지 몰고 가는 동안 GPS 도착/타임아웃으로 wp_idx 가 새는 걸 막는다.
        #    (위에서 방위·거리·wp_mode 는 계속 낸다 — 정보용. 모터는 아두이노 MANUAL 이 무시한다.)
        #    RC 모드를 AUTO 로 넘기는 순간부터 아래 도착/타임아웃 로직이 살아나 WP0 부터 진짜 시작한다.
        if not mission_should_run(self.boat_mode, self.require_auto):
            self.wp_enter_time = None         # 타임아웃 시계 리셋 → AUTO 시작 시점부터 카운트
            self.t_start = None               # dwell 리셋
            self.integral_xte = 0.0           # LOS 적분 리셋
            self.prev_lat = self.prev_lon = None   # 경로선 시작점 = AUTO 전환 시점 위치
            self.get_logger().info(
                "🕹 AUTO 대기 — RC 모드를 AUTO 로 넘기면 WP0 부터 자율 시작 "
                "(벤치는 require_boat_mode_auto:=false 또는 /boat_mode 2 발행)",
                throttle_duration_sec=5.0)
            return

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
                self.prev_lat, self.prev_lon = wp_lat, wp_lon   # 새 경로선 시작점
                self.integral_xte = 0.0                          # ILOS 적분 리셋
                self.wp_idx += 1
                self.t_start = None
                self.wp_enter_time = None
                return

        # fix 가 없으면 '도착했는지' 를 판단할 근거가 없다 → wp_idx 를 전진시키지 않는다.
        # (시간 초과 전진은 위에서 이미 처리한다 — 그건 위치와 무관한 안전장치라 유지)
        # 도착 반경은 속도·모드에 따라 동적(빠르면 넓게 오버슈트 방지, 도킹은 좁게).
        arrive_r = los_logic.get_dynamic_arrive_radius(
            wp_mode, self.gps_filter.estimated_speed_mps)
        if dist is None:
            self.t_start = None
        elif dist < arrive_r:
            if self.t_start is None:
                self.t_start = now
            elif (now - self.t_start) >= dwell:
                self.get_logger().info(
                    f"✔ WP{self.wp_idx} 완료 (반경 {arrive_r:.1f}m) → 다음 WP 이동")
                self.prev_lat, self.prev_lon = wp_lat, wp_lon   # 새 경로선 시작점
                self.integral_xte = 0.0                          # ILOS 적분 리셋
                self.wp_idx += 1
                self.t_start = None
                self.wp_enter_time = None
        else:
            self.t_start = None


# ───────────────────────── 순수 기하 (테스트 가능) ─────────────────────────
def _ray_polygon_dist(o, d, pts):
    """원점 o 에서 방향 d 로 쏜 광선이 폴리곤 변에 처음 닿는 거리. 안 닿으면 None.

    ★ 이게 '벽은 선이다'를 정직하게 푸는 방법이다. 최근접점 한 방향으로 원뿔을 막으면
      모서리에서 정면(탈출구)이 원뿔 사이 '틈'으로 열려 배가 대각선으로 빠져나간다.
    """
    ox, oy = o
    dx, dy = d
    best = None
    n = len(pts)
    for i in range(n):
        ax, ay = pts[i]
        bx, by = pts[(i + 1) % n]
        ex, ey = bx - ax, by - ay

        denom = dx * ey - dy * ex
        if abs(denom) < 1e-12:
            continue                       # 광선과 변이 평행

        rx, ry = ax - ox, ay - oy
        t = (rx * ey - ry * ex) / denom    # 광선 위 거리
        s = (rx * dy - ry * dx) / denom    # 변 위 위치 (0~1 이어야 선분 안)

        if t >= 0.0 and 0.0 <= s <= 1.0:
            if best is None or t < best:
                best = t
    return best


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
