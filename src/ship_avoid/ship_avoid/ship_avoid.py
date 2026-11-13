"""도리도리회피 — ship_avoid: mode 5 장애물 회피 미션 노드 (장애물(설정 색, 기본 흰) + 도리도리).

계약서: docs/문제와작업/계약서_도리도리회피_20261112.md (초안, 미승인)
        docs/문제와작업/계약서_도리도리회피_launch_20261113.md C1 (avoid_mode·검출 침묵 폴백·/wp_mode stale)
        docs/문제와작업/계약서_도리도리회피_v2_20261114.md U1 (/avoid_zone · ZONE_OFF · /avoid_detections VER 1)
        docs/문제와작업/계약서_흰부표_거리융합_v3_20261114.md W2 (흰 장애물 LiDAR 연관 · 단안 피치·롤 규격)
avoid_mode (yaml, 기본 'fused' — 사용자 결정 11-14): lidar = 담당 모드에서 매 틱 20000 만(GPS·yaw·/avoid_zone 무관,
  지금 north 폴백과 같음). 이상값 → ERROR + lidar. 옛 이름 'doridori' → 'fused' 로 받고 WARN.
  fused = 도리도리 코어(명세 1~8). 단 /avoid_zone 이 신선하고 true 일 때만(아니면 20000, ZONE_OFF),
  그리고 /avoid_detections 침묵이면 20000 으로 넘기고 도리도리를 멈춘다.
  지도 수명 = zone 구간: zone true 동안 검출 침묵으로 잠깐 멈춰도 지도·코스 축 유지,
  zone false·stale 이나 담당 모드 이탈이면 지도를 버린다.
  판정·검출 형식 파싱은 avoid_mode_logic (순수 함수 plan_tick·parse_detections).
  🚨 기본 fused. north 는 /avoid_zone 을 늘 낸다(U2) — 폴리곤이 비면 늘 false → mode 5 에서 20000(ZONE_OFF).
     폴리곤을 채우면 WP6→WP7 다리에서 true → 검출기(U3 image_subscriber_avoid) 하트비트가 있으면 도리도리(DORI),
     검출기가 안 떴거나 camera_info 가 없어 침묵하면 그 다리에서 DET_DEAD(20000 + ERROR).
🚨 대회 launch 에 들어 있다(launch_files _ship_avoid, 계약서 C2 원자 묶음 — 한 커밋):
   north_goal_angle FALLBACK_MODES (8,)(mode 5 폴백 뺌) + ssf_tools wp_nodes 5→ship_avoid + launch Node.
   하나만 되돌리면 mode 5 /candidate_angle 발행자가 2개이거나 healthcheck 거짓 경보(test_avoid_mission 이 검사).
   avoid_mode 는 시작 때만 읽는다 → 바꾸면 launch 재시작(ros2 param set 은 효과 없음).

입력
  /wp_mode (Int32)                 담당 모드(active_wp_mode, 기본 5)일 때만 발행.
                                   wp_mode_stale_sec 넘게 묵으면 발행 중지(north 주기 0.5 s)
  /imu/yaw (Float64)               나침반 yaw(deg)
  /imu/data (sensor_msgs/Imu)      피치·롤(수면 교점 단안 거리). pitch_sign·roll_sign ⚠️ 실측.
                                   한 번도 안 오면 장착값(cam_pitch_deg·cam_roll_deg — 설계값 11°·0°)만 쓰고 활성 때
                                   WARN 1회(지금 iahrs 는 발행 안 함 = 정상, 11-14 결정). 받다가 imu_stale_sec 넘게
                                   끊기면 피치·롤 0(장착값만) + 5 s 주기 WARN
  /ublox_gps_node/fix (NavSatFix)  위치 → 첫 fix 기준 로컬 m
  /scan (LaserScan)                부표 추출(lidar_buoys — 노랑 + 키 큰 흰 장애물, 폭 검사 반경 max(buoy_r, obstacle_r),
                                   노랑 폭으론 안 되는 넓은 군집 표시 classify)
                                   — BEST_EFFORT depth 1
  /avoid_zone (Bool)               장애물 구간 다리(명세 8, north 발행 — U2). avoid_zone_stale_sec 넘게 묵으면 false 로 본다.
       north 가 0.5 s 마다 낸다 — /wp_mode 는 오는데 이게 한 번도 안 오면 north 버전·QoS 문제(WARN 1회).
  /avoid_detections (Float32MultiArray)  [VER=1, STRIDE=6, N, N × (cls, bearing_left_deg, v_norm, range_m, conf, age_s)]
       cls 0 = 장애물(검출기 설정 색, 기본 흰), 2 = 노랑. bearing 좌 +. v_norm = (v_bottom − cy)/fy.
       range_m 모르면 NaN(지금 검출기는 늘 NaN → 단안). cls 0 은 기본(obstacle_requires_lidar true) LiDAR 점에
       방위·거리로 일대일 연관될 때만 지도에.
       age_s = 촬영부터 발행까지 초 — det_max_age_s 넘는 검출은 버린다(지금은 그 밖에 안 씀).
       N = 0 = 본 것 없음(살아 있음 하트비트). VER·STRIDE·길이 불일치 → 메시지 버림 + ERROR(살아 있음으로 안 침).
       예전 stride 5 형식(머리 없음)은 받지 않는다.
       발행자 = color_shape_detector image_subscriber_avoid(U3, mode 5 프레임마다 — 본 것 없어도 N=0).
       fused 여도 침묵이면 20000 폴백(진입 후 quiet 초 넘으면 ERROR).
출력
  /candidate_angle (Float32)       lidar·검출 침묵 = 20000, 주행 = (B) 각도, 도리도리 = 5000/6000/50000
  /virtual_obstacle_state (Float32MultiArray)  /geofence_state 형식 (ship_direction 병합은 계약 단위 ①)
                                   도리도리 코어가 돌 때만 낸다(lidar·20000 폴백 땐 지도가 없으니 침묵)
  /avoid_state (String)            관측용 — 'LIDAR' / 'FUSED_ZONE_OFF' / 'FUSED_NO_DET …' / 코어 상태
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, Float32, Float32MultiArray, Float64, Int32, String
from sensor_msgs.msg import Imu, LaserScan, NavSatFix

from ship_avoid import avoid_mode_logic as ml
from ship_avoid.core import AvoidCore, AvoidParams, virtual_ranges
from ship_avoid.lidar_buoys import cluster_buoys

EARTH_R = 6371000.0
SCAN_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=1)


class ShipAvoid(Node):
    def __init__(self):
        super().__init__('ship_avoid')
        # 🚨 선언은 한 줄에 하나, 키·기본값 리터럴로 — test_launch_params._declared·test_node_params 가 ast 로 읽는다.
        #    기본값 = core.AvoidParams 기본값 = config/ship_avoid.yaml (소수점 타입까지).
        self.active_wp_mode = int(self.declare_parameter('active_wp_mode', 5).value)
        self.period = float(self.declare_parameter('period_sec', 0.1).value)
        self.gps_stale = float(self.declare_parameter('gps_stale_sec', 2.5).value)
        self.yaw_stale = float(self.declare_parameter('yaw_stale_sec', 0.5).value)
        det_quiet = self.declare_parameter('detections_quiet_warn_sec', 5.0).value
        self.lidar_forward = float(self.declare_parameter('lidar_forward_deg', 80.0).value)
        self.pitch_sign = float(self.declare_parameter('pitch_sign', 1.0).value)    # ⚠️ 실측: 선수 숙임 = 양수가 되게
        self.roll_sign = float(self.declare_parameter('roll_sign', 1.0).value)      # ⚠️ 실측: 우현 아래 = 양수가 되게
        imu_stale = self.declare_parameter('imu_stale_sec', 1.0).value                # 넘으면 피치·롤 0(장착값만)
        self.virtual_extra = float(self.declare_parameter('virtual_extra_m', 0.2).value)
        avoid_mode = self.declare_parameter('avoid_mode', 'fused').value              # 'lidar' | 'fused'
        wp_stale = self.declare_parameter('wp_mode_stale_sec', 1.0).value             # north /wp_mode 주기 0.5 s 의 2배
        zone_stale = self.declare_parameter('avoid_zone_stale_sec', 1.0).value        # north /avoid_zone 주기 0.5 s 의 2배
        det_max_age = self.declare_parameter('det_max_age_s', 0.5).value              # ⚠️ 실측 필요(검출기 지연)
        P = AvoidParams()
        P.half_l = float(self.declare_parameter('half_l', 0.83).value)
        P.half_w = float(self.declare_parameter('half_w', 0.285).value)
        P.low_r = float(self.declare_parameter('low_r', 0.2).value)
        P.buoy_r = float(self.declare_parameter('buoy_r', 0.12).value)
        P.obstacle_r = float(self.declare_parameter('obstacle_r', 0.25).value)          # ⚠️ 흰 부표 반경 가정
        P.default_half_width_m = float(self.declare_parameter('default_half_width_m', 1.5).value)
        P.clear_low = float(self.declare_parameter('clear_low', 0.45).value)
        P.clear_wall = float(self.declare_parameter('clear_wall', 0.35).value)
        P.guard_m = float(self.declare_parameter('guard_m', 0.15).value)
        P.stop_lead_s = float(self.declare_parameter('stop_lead_s', 0.6).value)
        P.dori_angle_deg = float(self.declare_parameter('dori_angle_deg', 60.0).value)
        P.dori_tol_deg = float(self.declare_parameter('dori_tol_deg', 4.0).value)
        P.dori_lead_s = float(self.declare_parameter('dori_lead_s', 0.3).value)
        P.dori_settle_s = float(self.declare_parameter('dori_settle_s', 0.3).value)
        P.dori_hold_s = float(self.declare_parameter('dori_hold_s', 1.2).value)
        P.dori_sweep_margin_m = float(self.declare_parameter('dori_sweep_margin_m', 0.25).value)
        P.dori_defer_max_m = float(self.declare_parameter('dori_defer_max_m', 1.5).value)
        P.dori_explore_m = float(self.declare_parameter('dori_explore_m', 3.0).value)
        P.dori_timeout_s = float(self.declare_parameter('dori_timeout_s', 25.0).value)
        P.stop_speed = float(self.declare_parameter('stop_speed', 0.05).value)
        P.look_m = float(self.declare_parameter('look_m', 1.5).value)
        P.turn_in_m = float(self.declare_parameter('turn_in_m', 2.5).value)
        P.max_steer_deg = float(self.declare_parameter('max_steer_deg', 60.0).value)
        P.step_deg = float(self.declare_parameter('step_deg', 10.0).value)
        P.turn_gain_per_s = float(self.declare_parameter('turn_gain_per_s', 0.5).value)  # ⚠️ 실측(선회 응답)
        P.cam_h = float(self.declare_parameter('cam_h', 0.415).value)                  # 설계값(mast v2), 실측 아님 ⚠️
        P.cam_pitch_deg = float(self.declare_parameter('cam_pitch_deg', 11.0).value)   # 설계값(mast v2), 실측 아님 ⚠️
        P.cam_roll_deg = float(self.declare_parameter('cam_roll_deg', 0.0).value)      # ⚠️ 실측 필요
        P.cam_h_sigma_m = float(self.declare_parameter('cam_h_sigma_m', 0.05).value)   # ⚠️ 가정
        P.pixel_sigma = float(self.declare_parameter('pixel_sigma', 0.004).value)      # 정규 좌표(px/fy) ⚠️
        P.bearing_sigma_deg = float(self.declare_parameter('bearing_sigma_deg', 0.7).value)
        P.pitch_sigma_hold = float(self.declare_parameter('pitch_sigma_hold', 0.5).value)
        P.pitch_sigma_move = float(self.declare_parameter('pitch_sigma_move', 2.0).value)
        P.pos_sigma = float(self.declare_parameter('pos_sigma', 0.03).value)
        P.range_sigma_rel = float(self.declare_parameter('range_sigma_rel', 0.03).value)
        P.max_range = float(self.declare_parameter('max_range', 8.0).value)
        P.confirm_n = int(self.declare_parameter('confirm_n', 3).value)
        P.red_ratio = float(self.declare_parameter('red_ratio', 0.6).value)
        P.wall_fill_step = float(self.declare_parameter('wall_fill_step', 0.15).value)
        P.buoy_match_m = float(self.declare_parameter('buoy_match_m', 0.35).value)
        P.obstacle_match_deg = float(self.declare_parameter('obstacle_match_deg', 6.5).value)   # ⚠️ 근거 core 주석
        P.cam_yaw_offset_deg = float(self.declare_parameter('cam_yaw_offset_deg', 0.0).value)   # ⚠️ 실측(배별)
        P.obstacle_requires_lidar = bool(self.declare_parameter('obstacle_requires_lidar', True).value)

        self.avoid_mode, err, note = ml.check_avoid_mode(avoid_mode)
        if err:
            self._err_param(err)
        if note:
            self._warn_param(note)
        self.wp_stale, err = ml.check_stale_sec('wp_mode_stale_sec', wp_stale, ml.DEFAULT_WP_MODE_STALE_SEC)
        if err:
            self._err_param(err)
        self.det_quiet, err = ml.check_stale_sec('detections_quiet_warn_sec', det_quiet, ml.DEFAULT_DET_QUIET_SEC)
        if err:
            self._err_param(err)
        self.zone_stale, err = ml.check_stale_sec('avoid_zone_stale_sec', zone_stale, ml.DEFAULT_ZONE_STALE_SEC)
        if err:
            self._err_param(err)
        self.det_max_age, err = ml.check_stale_sec('det_max_age_s', det_max_age, ml.DEFAULT_DET_MAX_AGE_S)
        if err:
            self._err_param(err)
        self.imu_stale, err = ml.check_stale_sec('imu_stale_sec', imu_stale, ml.DEFAULT_IMU_STALE_SEC)
        if err:
            self._err_param(err)
        self.core = AvoidCore(P)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_cand = self.create_publisher(Float32, '/candidate_angle', qos)
        self.pub_virtual = self.create_publisher(Float32MultiArray, '/virtual_obstacle_state', qos)
        self.pub_state = self.create_publisher(String, '/avoid_state', qos)
        self.create_subscription(Int32, '/wp_mode', self.wp_mode_cb, qos)
        self.create_subscription(Float64, '/imu/yaw', self.yaw_cb, qos)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, qos)
        self.create_subscription(NavSatFix, '/ublox_gps_node/fix', self.gps_cb, qos)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, SCAN_QOS)
        self.create_subscription(Float32MultiArray, '/avoid_detections', self.det_cb, qos)
        self.create_subscription(Bool, '/avoid_zone', self.zone_cb, qos)

        self.wp_mode = None
        self.wp_mode_t = None
        self.active_since = None          # 이번 담당 구간 진입 시각(검출 침묵 유예 기준)
        self.yaw = None
        self.yaw_t = None
        self.imu_t = None                 # /imu/data 마지막 수신(없으면 장착 피치·롤만)
        self.lat0 = self.lon0 = None
        self.x = self.y = None
        self.gps_t = None
        self.last_det_t = None
        self.zone = None
        self.zone_t = None
        self.zone_never_warned = False
        self.was_active = False           # 코어가 지금 돌고 있나(step 을 부르나)
        self.map_live = False             # 이번 zone 구간에 코어를 켠 적이 있나(= 지도·코스 축이 살아 있나)
        self.create_timer(self.period, self.timer_cb)
        self.get_logger().info(
            f"도리도리회피(ship_avoid) 시작: avoid_mode={self.avoid_mode}, mode {self.active_wp_mode} 담당 "
            f"(lidar = 매 틱 20000), /wp_mode {self.wp_stale:g} s 넘게 묵으면 발행 중지, "
            f"/avoid_zone {self.zone_stale:g} s 넘게 묵으면 zone 밖, 검출 {self.det_max_age:g} s 넘게 묵으면 버림, "
            f"도리도리 ±{P.dori_angle_deg:.0f}°")

    # ── 입력 ──
    def wp_mode_cb(self, msg):
        self.wp_mode = int(msg.data)
        self.wp_mode_t = time.monotonic()

    def yaw_cb(self, msg):
        if math.isfinite(msg.data):
            self.yaw = float(msg.data) % 360.0
            self.yaw_t = time.monotonic()

    def imu_cb(self, msg):
        q = msg.orientation
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, sinp))))
        roll = math.degrees(math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y)))
        if not (math.isfinite(pitch) and math.isfinite(roll)):
            return
        self.core.set_pitch(self.pitch_sign * pitch)
        self.core.set_roll(self.roll_sign * roll)
        self.imu_t = time.monotonic()

    def gps_cb(self, msg):
        if msg.status.status < 0 or (msg.latitude == 0.0 and msg.longitude == 0.0):
            return                         # fix 없음 — 위치 갱신 안 함(stale 로 드러난다)
        if self.lat0 is None:
            self.lat0, self.lon0 = msg.latitude, msg.longitude
        k = math.pi / 180.0 * EARTH_R
        self.x = (msg.longitude - self.lon0) * k * math.cos(math.radians(self.lat0))
        self.y = (msg.latitude - self.lat0) * k
        self.gps_t = time.monotonic()

    def scan_cb(self, msg):
        if self.x is None or self.yaw is None:
            return
        p = self.core.p
        bl = cluster_buoys(list(msg.ranges), math.degrees(msg.angle_min), math.degrees(msg.angle_increment),
                           lidar_forward_deg=self.lidar_forward, buoy_r=p.buoy_r,
                           width_r=max(p.buoy_r, p.obstacle_r), classify=True)
        pts = []
        for rel, d, wide in bl:
            wb = math.radians(self.yaw - rel)
            pts.append((self.x + d * math.sin(wb), self.y + d * math.cos(wb), wide))
        self.core.set_buoys(pts)

    def zone_cb(self, msg):
        self.zone = bool(msg.data)
        self.zone_t = time.monotonic()

    def det_cb(self, msg):
        dets, n_old, err = ml.parse_detections(msg.data, self.det_max_age)
        if dets is None:                   # 형식 오류 — 버림, 살아 있음으로도 안 친다(침묵 → DET_DEAD 로 드러남)
            self._err_det_format(err)
            return
        self.last_det_t = time.monotonic()
        if n_old:
            self._warn_det_old(n_old)
        if self.x is None or self.yaw is None or not dets:
            return
        self.core.add_detections(time.monotonic(), self.x, self.y, self.yaw, dets)

    # ── 로그 (호출 지점 분리 — CLAUDE.md §1-7) ──
    def _err_param(self, msg):
        self.get_logger().error(f"ship_avoid: {msg}")

    def _warn_param(self, msg):
        self.get_logger().warn(f"ship_avoid: {msg}")

    def _err_stale(self, what):
        self.get_logger().error(f"ship_avoid: {what} → 발행 중지(모르면 입을 다문다)", throttle_duration_sec=5.0)

    def _err_wp_stale(self, age):
        self.get_logger().error(
            f"ship_avoid: /wp_mode {age:.1f} s 묵음(> {self.wp_stale:g} s, 마지막 mode {self.wp_mode}) "
            "→ /candidate_angle 발행 중지(north 확인)", throttle_duration_sec=5.0)

    def _err_det_dead(self):
        self.get_logger().error(
            f"ship_avoid: fused 인데 /avoid_detections 가 {self.det_quiet:g} s 넘게 침묵 → 20000 폴백, "
            "도리도리 시작 안 함(검출기 확인)", throttle_duration_sec=5.0)

    def _err_det_format(self, why):
        self.get_logger().error(
            f"ship_avoid: /avoid_detections 형식 오류({why}) → 그 메시지 버림 "
            f"(형식 [VER={ml.DET_VER}, STRIDE={ml.DET_STRIDE}, N, N×6] — 검출기 확인)", throttle_duration_sec=5.0)

    def _warn_det_old(self, n):
        self.get_logger().warn(
            f"ship_avoid: /avoid_detections 검출 {n}개 age_s > {self.det_max_age:g} s → 버림(검출기 지연 확인)",
            throttle_duration_sec=5.0)

    def _warn_zone_never(self):
        self.get_logger().warn(
            "ship_avoid: fused 인데 /avoid_zone 을 한 번도 못 받음 → 20000(도리도리 안 함). "
            "north 가 떠 있는데 이 경고가 나오면 north 버전(pull·빌드)·QoS 확인 — 이 경고는 1회만")

    def _err_zone_stale(self, age):
        self.get_logger().error(
            f"ship_avoid: fused 인데 /avoid_zone {age:.1f} s 묵음(> {self.zone_stale:g} s) → 20000, "
            "지도 초기화(north 확인)", throttle_duration_sec=5.0)

    def _warn_no_imu(self):
        self.get_logger().warn(
            "ship_avoid: /imu/data 를 한 번도 못 받음 → 단안 거리에 장착 피치·롤(cam_pitch_deg·cam_roll_deg)만 씀 "
            "(지금 iahrs 는 /imu/data 를 발행하지 않는다 — 정상, 파도 피치 미보정. 이 경고는 zone 구간마다 1회)")

    def _warn_imu_stale(self, age):
        self.get_logger().warn(
            f"ship_avoid: /imu/data {age:.1f} s 묵음(> {self.imu_stale:g} s) → IMU 피치·롤 0, 장착값만 씀(iahrs 확인)",
            throttle_duration_sec=5.0)

    def _info_state(self, s):
        self.get_logger().info(f"ship_avoid: {s}")

    # ── 주기 ──
    def _leave(self, why):
        """zone 구간 끝 — 코어를 멈추고 지도·코스 축을 버린다(다음 구간은 새 지도)."""
        if self.was_active or self.map_live:
            self.core.reset_keep_pose()
            self._info_state(f"{why} → 도리도리 비활성, 지도·코스 축 초기화")
        self.was_active = False
        self.map_live = False

    def _pause(self):
        """zone 구간 안 검출 침묵 — 코어만 멈추고 지도·코스 축은 둔다(검출이 돌아오면 resume)."""
        if self.was_active:
            self.core.deactivate()
            self._info_state("검출 침묵 → 도리도리 일시 중지(zone 구간 안 — 지도·코스 축 유지)")
        self.was_active = False

    def _log_zone_off(self, now):
        zs = ml.zone_status(self.zone, self.zone_t, now, self.zone_stale)
        if zs == ml.ZONE_NEVER and not self.zone_never_warned:
            self.zone_never_warned = True
            self._warn_zone_never()
        elif zs == ml.ZONE_STALE:
            self._err_zone_stale(now - self.zone_t)

    def _check_imu(self, now):
        """받다가 끊긴 /imu/data 는 묵은 자세로 쓰지 않는다(W2b ⑦): 피치·롤 0 + 코어가 돌 때 주기 WARN.
        한 번도 못 받은 경우(지금 배 정상)는 활성 때 WARN 1회(_warn_no_imu)뿐."""
        if ml.imu_status(self.imu_t, now, self.imu_stale) != ml.IMU_STALE:
            return
        self.core.set_pitch(0.0)
        self.core.set_roll(0.0)
        if self.was_active:
            self._warn_imu_stale(now - self.imu_t)

    def timer_cb(self):
        now = time.monotonic()
        self._check_imu(now)
        state = ml.plan_tick(self.avoid_mode, self.wp_mode, self.wp_mode_t, self.active_wp_mode, now,
                             self.wp_stale, self.last_det_t, self.active_since, self.det_quiet,
                             self.zone, self.zone_t, self.zone_stale)
        if not ml.is_active(state):
            self.active_since = None
            self._leave("mode 이탈" if state == ml.IDLE else "/wp_mode stale")
            if state == ml.WP_STALE:
                self._err_wp_stale(now - self.wp_mode_t)
            return
        if self.active_since is None:
            self.active_since = now
        fixed = ml.fixed_candidate(state)
        if fixed is not None:                                  # LIDAR · ZONE_OFF · DET_WAIT · DET_DEAD → 20000
            if ml.keeps_map(state):                            # DET_* — zone 안 검출 침묵
                self._pause()
            else:                                              # LIDAR · ZONE_OFF
                self._leave("zone 밖")
            if state == ml.ZONE_OFF:
                self._log_zone_off(now)
            elif state == ml.DET_DEAD:
                self._err_det_dead()
            self.pub_cand.publish(Float32(data=fixed))
            if state == ml.LIDAR:
                label = 'LIDAR'
            elif state == ml.ZONE_OFF:
                label = 'FUSED_ZONE_OFF'
            else:
                label = f'FUSED_NO_DET {state}'
            self.pub_state.publish(String(data=label))
            return
        # DORI — 도리도리 코어 경로
        if self.gps_t is None or now - self.gps_t > self.gps_stale:
            self._err_stale("GPS 위치 없음/stale")
            return
        if self.yaw_t is None or now - self.yaw_t > self.yaw_stale:
            self._err_stale("/imu/yaw 없음/stale")
            return
        self.core.set_pose(now, self.x, self.y, self.yaw)
        if not self.was_active:
            if self.map_live:                                  # 같은 zone 구간 — 지도·코스 축 이어 감
                if not self.core.resume(now):
                    return
                self._info_state(f"재개 — 지도 유지(코스 축 {self.core.psi0:.1f}°), 도리도리 1회")
            else:
                if not self.core.activate(now):
                    return
                self._info_state(f"활성 — 코스 축 {self.core.psi0:.1f}° 잠금, 시작 도리도리")
                if self.imu_t is None:
                    self._warn_no_imu()
            self.was_active = True
            self.map_live = True
        cand = self.core.step(now)
        if cand is None:
            return
        self.pub_cand.publish(Float32(data=float(cand)))
        vo = self.core.virtual_obstacles(self.virtual_extra)
        arr = Float32MultiArray()
        arr.data = [float(v) for v in virtual_ranges(self.x, self.y, self.yaw, vo)] if vo else []
        self.pub_virtual.publish(arr)
        self.pub_state.publish(String(data=self.core.status()))


def main(args=None):
    rclpy.init(args=args)
    node = ShipAvoid()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
