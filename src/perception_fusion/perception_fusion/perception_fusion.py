"""perception_fusion — LiDAR 군집 + D455 뎁스 덩어리 + OAK 검출 방위(색) → 물체 목록 /fused_objects.

계약서_인식융합_20261112 (§10 결정이 본문보다 우선), 근거 계약서_거리융합_20261112 §4·§5.
  · 미션 노드(ship_back·ship_turn APPROACH·ship_gate)는 distance_source 가 fill/fused 일 때만 이 목록을 본다.
    lidar 면 구독도 안 한다. 기본 yaml = fused(11-14 v3 G-9), 어긋나면 LiDAR(disagree_policy lidar).
  · CLAUDE.md §3 "카메라는 방위각만, 거리는 LiDAR" 의 예외(§10 Q1 승인) — distance_source fill/fused 일 때만.
  · 물 위 실측 0회(§10). D455 장착 오프셋·행 띠·거리 범위는 ⚠️ 실측 필요 값(yaml).

입력
  /scan (LaserScan, BEST_EFFORT d1) — a = 80 + rel(좌 +), 정면 ±90° 만
  depth_topic (Image 16UC1 mm, 0 = 모름, BEST_EFFORT d1) + depth_info_topic (CameraInfo: fx = k[0], cx = k[2])
  /red_angle·/green_angle·/image_angle (Float32 상대각 좌 +, 10000 = 안 보임) + /image_color (String) — 색 붙이기만
  /wp_mode (Int32) — 관측만(/fusion_status 에 찍음)
  P1(계약서_흰부표_거리융합_v3 B-2, use_mono·use_triangulation 기본 false — 꺼지면 아래 넷은 받기만 하고 안 쓴다):
  /oak_detections (Float32MultiArray, BEST_EFFORT d1) — D1 다중 물체 검출, 형식은 fusion_logic 머리말.
    VER·STRIDE·길이 불일치 = 메시지 버림 + 주기 ERROR. 발행자 D1 은 아직 없다(구독자가 먼저 — §4).
  /ublox_gps_node/fix (NavSatFix, RELIABLE 10) — 삼각측량 기선. RTK FIXED 판정 = fusion_logic.fix_quality
    (P1b-2: 검출은 RayQueue 에 넣고, 촬영 뒤 GPS 표본(1 Hz)이 오면 publish_cb 에서 보간해 광선으로 만든다.
     결과는 '지금' 자세(GPS 외삽 + 현재 yaw)로 다시 잰다. 버림 사유는 warn_period_sec 주기 WARN.)
  /imu/yaw (Float64, RELIABLE 10) — 세계 방위(나침반 도, 시계 +)
  /imu/data (Imu, RELIABLE 10) — 피치·롤(단안). ⚠️ iahrs_driver 는 발행자만 만들고 publish 하지 않는다
    (iahrs_driver.cpp:81 생성, 루프에 publish 없음) → 지금은 장착값만 + WARN 1회.
출력
  /fused_objects (Float32MultiArray, RELIABLE 10, publish_hz 고정주기 타이머) — 형식은 fusion_logic 머리말
    LiDAR stale → 뎁스만(flags bit1), 둘 다 stale → 발행 중단(침묵), 빈 목록 = 살아 있고 물체 없음
  /fusion_status (String, RELIABLE 10, status_hz) — 예 'L0.82 D0.61 agree0.93 lidar_ok depth_ok n=3 m=2'
    (use_mono/use_triangulation 켜짐이면 뒤에 'oak_ok mono=1/2 tri=0/0 imu_mount rtk_fixed' — fusion_logic.format_status)
침묵·오류: 입력 stale = warn_period_sec 주기 WARN, 파라미터 무효 = ERROR + 기본값, 뎁스 형식 오류 = 주기 ERROR.
  /health_ok 는 건드리지 않는다(§10 Q8: 침묵은 로그만). 로거는 심각도마다 호출 줄을 나눈다(§1-7).
"""

import math
import time

from perception_fusion import fusion_logic as fl
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan, NavSatFix
from std_msgs.msg import Float32, Float32MultiArray, Float64, Int32, String

# 센서(/scan·뎁스·camera_info) = BEST_EFFORT + KEEP_LAST + depth 1 (CLAUDE.md §3 QoS — 도착 = 신선).
SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
# 검출기·/wp_mode 구독과 관찰 토픽 발행 = RELIABLE 10 (§3: 발행은 RELIABLE — ros2 topic echo 가 받게).
RELIABLE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)

DEPTH_ENCODINGS = ('16UC1', 'mono16')


class PerceptionFusion(Node):
    def __init__(self):
        super().__init__('perception_fusion')

        # ---- 파라미터 (config/perception_fusion.yaml, 기본값 == yaml == fusion_logic.DEFAULTS) ----
        #   🚨 구독·타이머보다 먼저 선언한다. 기본값은 리터럴 — launch_files T3/T4 와 테스트가 ast 로 읽는다.
        raw = {
            'depth_topic': self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw').value,
            'depth_info_topic': self.declare_parameter('depth_info_topic', '/camera/camera/depth/camera_info').value,
            'publish_hz': self.declare_parameter('publish_hz', 10.0).value,
            'status_hz': self.declare_parameter('status_hz', 1.0).value,
            'lidar_stale_sec': self.declare_parameter('lidar_stale_sec', 0.5).value,
            'depth_stale_sec': self.declare_parameter('depth_stale_sec', 0.5).value,
            'det_stale_sec': self.declare_parameter('det_stale_sec', 0.7).value,
            'warn_period_sec': self.declare_parameter('warn_period_sec', 5.0).value,
            'lidar_half_fov_deg': self.declare_parameter('lidar_half_fov_deg', 90.0).value,
            'cluster_gap_deg': self.declare_parameter('cluster_gap_deg', 2.0).value,
            'cluster_gap_m': self.declare_parameter('cluster_gap_m', 0.3).value,
            'cluster_min_points': self.declare_parameter('cluster_min_points', 2).value,
            'fusion_margin_deg': self.declare_parameter('fusion_margin_deg', 3.0).value,
            'color_match_deg': self.declare_parameter('color_match_deg', 3.0).value,
            'weight_min': self.declare_parameter('weight_min', 0.3).value,
            'weight_max': self.declare_parameter('weight_max', 0.7).value,
            'agree_abs_m': self.declare_parameter('agree_abs_m', 0.3).value,            # ⚠️ 실측 필요
            'agree_rel': self.declare_parameter('agree_rel', 0.1).value,                # ⚠️ 실측 필요
            'disagree_penalty': self.declare_parameter('disagree_penalty', 0.1).value,
            'disagree_policy': self.declare_parameter('disagree_policy', 'lidar').value,  # lidar | min
            'conf_window_frames': self.declare_parameter('conf_window_frames', 10).value,
            'conf_mad_ref_m': self.declare_parameter('conf_mad_ref_m', 0.3).value,
            'conf_jump_m': self.declare_parameter('conf_jump_m', 0.5).value,
            'depth_min_m': self.declare_parameter('depth_min_m', 0.5).value,            # ⚠️ 실측 필요
            'depth_max_m': self.declare_parameter('depth_max_m', 6.0).value,            # ⚠️ 실측 필요
            'depth_row_lo_frac': self.declare_parameter('depth_row_lo_frac', 0.4).value,  # ⚠️ 실측 필요
            'depth_row_hi_frac': self.declare_parameter('depth_row_hi_frac', 0.6).value,  # ⚠️ 실측 필요
            'depth_min_valid_frac': self.declare_parameter('depth_min_valid_frac', 0.3).value,  # ⚠️ 실측 필요
            'cam_x_from_lidar_m': self.declare_parameter('cam_x_from_lidar_m', 0.0).value,  # ⚠️ 실측 필요
            'cam_y_from_lidar_m': self.declare_parameter('cam_y_from_lidar_m', 0.0).value,  # ⚠️ 실측 필요
            'cam_yaw_offset_deg': self.declare_parameter('cam_yaw_offset_deg', 0.0).value,  # ⚠️ 실측 필요
            # ---- P1 단안·삼각측량 (기본 꺼짐, 장착값 전부 ⚠️ 가정값 — yaml 주석) ----
            'use_mono': self.declare_parameter('use_mono', False).value,
            'use_triangulation': self.declare_parameter('use_triangulation', False).value,
            'oak_cam_h_m': self.declare_parameter('oak_cam_h_m', 0.6).value,                # ⚠️ 실측 필요
            'oak_cam_h_sigma_m': self.declare_parameter('oak_cam_h_sigma_m', 0.05).value,   # ⚠️ 실측 필요
            'oak_pitch_deg': self.declare_parameter('oak_pitch_deg', 0.0).value,            # ⚠️ 실측 필요
            'oak_roll_deg': self.declare_parameter('oak_roll_deg', 0.0).value,              # ⚠️ 실측 필요
            'oak_x_from_lidar_m': self.declare_parameter('oak_x_from_lidar_m', 0.0).value,  # ⚠️ 실측 필요
            'oak_y_from_lidar_m': self.declare_parameter('oak_y_from_lidar_m', 0.0).value,  # ⚠️ 실측 필요
            'oak_yaw_offset_deg': self.declare_parameter('oak_yaw_offset_deg', 0.0).value,  # ⚠️ 실측 필요(배별)
            'oak_fy_px': self.declare_parameter('oak_fy_px', 513.0).value,                  # ⚠️ camera_info 확인
            'mono_pitch_sigma_deg': self.declare_parameter('mono_pitch_sigma_deg', 2.0).value,  # ⚠️ 실측 필요
            'mono_pixel_sigma': self.declare_parameter('mono_pixel_sigma', 2.0).value,      # ⚠️ 실측 필요
            'mono_max_range_m': self.declare_parameter('mono_max_range_m', 8.0).value,      # ⚠️ 실측 필요
            'imu_stale_sec': self.declare_parameter('imu_stale_sec', 0.5).value,
            'imu_pitch_sign': self.declare_parameter('imu_pitch_sign', 1.0).value,          # ⚠️ 실측 필요
            'imu_roll_sign': self.declare_parameter('imu_roll_sign', 1.0).value,            # ⚠️ 실측 필요
            'gps_to_cam_x_m': self.declare_parameter('gps_to_cam_x_m', 0.0).value,          # ⚠️ 실측 필요
            'gps_to_cam_y_m': self.declare_parameter('gps_to_cam_y_m', 0.0).value,          # ⚠️ 실측 필요
            'tri_min_baseline_m': self.declare_parameter('tri_min_baseline_m', 0.8).value,
            'tri_min_parallax_deg': self.declare_parameter('tri_min_parallax_deg', 5.0).value,
            'tri_max_dt_s': self.declare_parameter('tri_max_dt_s', 3.0).value,              # ⚠️ 실측 필요
            'tri_assoc_deg': self.declare_parameter('tri_assoc_deg', 5.0).value,            # ⚠️ 실측 필요
            'tri_pose_max_gap_s': self.declare_parameter('tri_pose_max_gap_s', 0.3).value,
            'gps_period_s': self.declare_parameter('gps_period_s', 1.0).value,          # zed_f9p_rover rate 1.0
            'tri_interp_sigma_frac': self.declare_parameter('tri_interp_sigma_frac', 0.1).value,  # ⚠️ 실측 필요
            'tri_pending_max': self.declare_parameter('tri_pending_max', 45).value,
            'tri_parallax_sigma_k': self.declare_parameter('tri_parallax_sigma_k', 2.0).value,
            'tri_max_rays': self.declare_parameter('tri_max_rays', 8).value,
            'cam_max_rel_sigma': self.declare_parameter('cam_max_rel_sigma', 0.3).value,  # ⚠️ 실측 필요
            'tri_yaw_sigma_deg': self.declare_parameter('tri_yaw_sigma_deg', 0.5).value,    # ⚠️ 실측 필요
            'tri_rtk_max_sigma_m': self.declare_parameter('tri_rtk_max_sigma_m', 0.05).value,
            'tri_max_range_m': self.declare_parameter('tri_max_range_m', 15.0).value,       # ⚠️ 실측 필요
            'lidar_sigma_m': self.declare_parameter('lidar_sigma_m', 0.05).value,           # ⚠️ 실측 필요
            'depth_sigma_k': self.declare_parameter('depth_sigma_k', 0.005).value,          # ⚠️ 실측 필요
            'fusion_gate_sigma': self.declare_parameter('fusion_gate_sigma', 3.0).value,
        }
        self.p, issues = fl.check_params(raw)
        for text in issues:
            self.get_logger().error(f'perception_fusion 파라미터 {text}')
        p = self.p

        # ---- 상태 ----
        self.lidar = fl.SourceTrack(p['conf_window_frames'])
        self.depth = fl.SourceTrack(p['conf_window_frames'])
        self.fx = None                 # camera_info 를 받기 전엔 뎁스를 안 쓴다(모르면 입을 다문다)
        self.cx = None
        self.info_w = None
        self.red = None                # (수신 시각, 상대각) 또는 None
        self.green = None
        self.image = None
        self.image_color = None        # (수신 시각, 문자열)
        self.wp_mode = -1
        self.stats = {'lidar_ok': False, 'depth_ok': False, 'n': None,
                      'conf_l': None, 'conf_d': None, 'agree': None}
        self.inputs_ok = None          # (lidar_ok, depth_ok) — 바뀔 때 INFO
        # P1 카메라 경로
        self.oak = None                # (수신 시각, 검출 목록) — 형식 오류 메시지는 넣지 않는다
        self.imu_warned = False        # /imu/data 없음 WARN 1회(다시 들어오면 풀림)
        # 촬영 시각(t_cap = 수신 − age_s) 값으로 찾을 버퍼(P1b-2·5). GPS 를 기다리는 만큼 길게 둔다.
        horizon = p['tri_max_dt_s'] + (fl.GPS_WAIT_FACTOR + fl.GPS_GAP_FACTOR) * p['gps_period_s'] + 1.0
        self.att = fl.TimeSeries(horizon)   # (IMU 피치, IMU 롤) — 부호 적용 뒤
        self.gps = fl.TimeSeries(horizon)   # (x 동, y 북, σ, fixed) 로컬 m
        self.yaw = fl.TimeSeries(horizon)
        self.geo0 = None               # 첫 유효 fix (lat, lon) = 로컬 원점
        self.gps_nofix = True          # 마지막 /fix 가 NO_FIX 였나(rtk_fixed·삼각측량 출력을 바로 끈다)
        self.rayq = fl.RayQueue()      # 촬영 뒤 GPS 표본을 기다리는 검출(publish_cb 에서 광선으로)
        self.tri = fl.TriTracker()
        self.log_t = {}                # 주기 로그 키 → 마지막 시각

        # ---- 발행/구독/타이머 (파라미터 선언 뒤) ----
        self.pub_objects = self.create_publisher(Float32MultiArray, '/fused_objects', RELIABLE_QOS)
        self.pub_status = self.create_publisher(String, '/fusion_status', RELIABLE_QOS)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, SENSOR_QOS)
        self.create_subscription(Image, p['depth_topic'], self.depth_cb, SENSOR_QOS)
        self.create_subscription(CameraInfo, p['depth_info_topic'], self.info_cb, SENSOR_QOS)
        self.create_subscription(Float32, '/red_angle', self.red_cb, RELIABLE_QOS)
        self.create_subscription(Float32, '/green_angle', self.green_cb, RELIABLE_QOS)
        self.create_subscription(Float32, '/image_angle', self.image_cb, RELIABLE_QOS)
        self.create_subscription(String, '/image_color', self.color_cb, RELIABLE_QOS)
        self.create_subscription(Int32, '/wp_mode', self.wp_cb, RELIABLE_QOS)
        # P1: 다중 물체 검출 = 센서처럼 BEST_EFFORT d1(도착 = 신선), GPS·yaw·IMU = RELIABLE 10 (계약 토픽 표)
        self.create_subscription(Float32MultiArray, '/oak_detections', self.oak_cb, SENSOR_QOS)
        self.create_subscription(NavSatFix, '/ublox_gps_node/fix', self.gps_cb, RELIABLE_QOS)
        self.create_subscription(Float64, '/imu/yaw', self.yaw_cb, RELIABLE_QOS)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, RELIABLE_QOS)
        # 제어 소비자가 있는 출력이라 고정주기 타이머에서만 낸다(데이터 콜백에서 내지 않는다).
        self.create_timer(1.0 / p['publish_hz'], self.publish_cb)
        self.create_timer(1.0 / p['status_hz'], self.status_cb)

        self.get_logger().info(
            f"perception_fusion 시작: /scan + {p['depth_topic']} (+ {p['depth_info_topic']}) + 검출기 방위 → "
            f"/fused_objects {p['publish_hz']} Hz (VER {fl.VER}, STRIDE {fl.STRIDE}). stale LiDAR "
            f"{p['lidar_stale_sec']} s / 뎁스 {p['depth_stale_sec']} s, 뎁스 {p['depth_min_m']}~{p['depth_max_m']} m, "
            f"행 띠 {p['depth_row_lo_frac']}~{p['depth_row_hi_frac']}, 장착 x {p['cam_x_from_lidar_m']} "
            f"y {p['cam_y_from_lidar_m']} yaw {p['cam_yaw_offset_deg']}°, 어긋남 = {p['disagree_policy']} "
            f"(⚠️ 실측 0회)")
        self.get_logger().info(
            f"perception_fusion 카메라 거리: use_mono={p['use_mono']} use_triangulation={p['use_triangulation']} "
            f"(/oak_detections VER {fl.OAK_VER} STRIDE {fl.OAK_STRIDE}). OAK h {p['oak_cam_h_m']} m "
            f"피치 {p['oak_pitch_deg']}° 롤 {p['oak_roll_deg']}° yaw {p['oak_yaw_offset_deg']}°, "
            f"삼각측량 기선 ≥ {p['tri_min_baseline_m']} m · 시차 ≥ {p['tri_min_parallax_deg']}° · "
            f"Δt ≤ {p['tri_max_dt_s']} s · RTK FIXED 만 (⚠️ 장착값 전부 가정값, 배별 통로 없음)")
        if p['use_mono'] and p['oak_pitch_deg'] == fl.DEFAULTS['oak_pitch_deg']:
            self.get_logger().warn(
                f"perception_fusion: use_mono=true 인데 oak_pitch_deg 가 기본값 {p['oak_pitch_deg']}°(미실측) — "
                '장착 피치 1° 오차가 3 m 에서 약 0.4 m. 실측값을 yaml 에 넣기 전엔 단안 거리를 믿지 말 것')

    # ---------------------- 주기 로그 ----------------------
    def _due(self, key, now):
        if fl.due(self.log_t.get(key), now, self.p['warn_period_sec']):
            self.log_t[key] = now
            return True
        return False

    # ---------------------- 센서 콜백 (처리만, 발행 안 함) ----------------------
    def scan_cb(self, msg):
        now = time.monotonic()
        pts = fl.scan_points(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min,
                             self.p['lidar_half_fov_deg'])
        if pts is None:
            if self._due('scan_bad', now):
                self.get_logger().error(
                    f'perception_fusion: /scan angle_increment={msg.angle_increment} 무효 → 이 스캔 버림')
            return
        self.lidar.push(now, pts[0], pts[1])

    def info_cb(self, msg):
        now = time.monotonic()
        fx, cx, w = float(msg.k[0]), float(msg.k[2]), int(msg.width)
        if not (fx > 0.0 and w > 0 and 0.0 <= cx <= w):
            if self._due('info_bad', now):
                self.get_logger().error(
                    f'perception_fusion: camera_info fx={fx} cx={cx} width={w} 무효 → 뎁스 안 씀')
            return
        self.fx, self.cx, self.info_w = fx, cx, w

    def depth_cb(self, msg):
        now = time.monotonic()
        if self.fx is None:
            if self._due('info_missing', now):
                self.get_logger().warn(
                    f"perception_fusion: {self.p['depth_info_topic']} 미수신 — fx·cx 를 몰라 뎁스 안 씀")
            return
        if msg.encoding not in DEPTH_ENCODINGS:
            if self._due('depth_enc', now):
                self.get_logger().error(
                    f'perception_fusion: 뎁스 encoding={msg.encoding!r} (16UC1 아님) → 뎁스 안 씀')
            return
        if int(msg.width) != self.info_w:
            if self._due('depth_w', now):
                self.get_logger().error(
                    f'perception_fusion: 뎁스 폭 {msg.width} ≠ camera_info 폭 {self.info_w} → 뎁스 안 씀')
            return
        img = fl.depth_image(msg.data, msg.height, msg.width, msg.step, bool(msg.is_bigendian))
        if img is None:
            if self._due('depth_size', now):
                self.get_logger().error(
                    f'perception_fusion: 뎁스 크기 {msg.width}x{msg.height} step {msg.step} 와 데이터 길이 불일치 → 버림')
            return
        rel, r = fl.depth_points(img, self.fx, self.cx, self.p)
        self.depth.push(now, rel, r)

    # ---------------------- 검출기·모드 ----------------------
    def _det(self, v):
        """Float32 상대각 → (시각, 각) 또는 None(10000 센티넬·nan = 안 보임)."""
        v = float(v)
        if not fl.detector_angle_valid(v):
            return None
        return (time.monotonic(), float(fl.wrap180(v)))

    def red_cb(self, msg):
        self.red = self._det(msg.data)

    def green_cb(self, msg):
        self.green = self._det(msg.data)

    def image_cb(self, msg):
        self.image = self._det(msg.data)

    def color_cb(self, msg):
        self.image_color = (time.monotonic(), str(msg.data))

    def wp_cb(self, msg):
        self.wp_mode = int(msg.data)

    # ---------------------- P1 카메라 경로 입력 (처리만, 발행 안 함) ----------------------
    def oak_cb(self, msg):
        now = time.monotonic()
        dets, bad, err = fl.parse_oak_detections(msg.data)
        if dets is None:
            if self._due('oak_format', now):
                self.get_logger().error(f'perception_fusion: /oak_detections 형식 오류({err}) → 메시지 버림')
            return
        if bad:
            if self._due('oak_rows', now):
                self.get_logger().warn(
                    f'perception_fusion: /oak_detections 무효 줄 {bad}개 버림(비유한·색 밖·|방위|≥90·age<0 등)')
        self.oak = (now, dets)
        if self.p['use_triangulation']:
            self.rayq.push(now, dets, self.p)     # 광선은 촬영 뒤 GPS 표본이 오면 publish_cb 에서 만든다

    def gps_cb(self, msg):
        now = time.monotonic()
        lat, lon = float(msg.latitude), float(msg.longitude)
        if int(msg.status.status) <= fl.NAV_STATUS_NO_FIX or not (math.isfinite(lat) and math.isfinite(lon)) \
                or (abs(lat) < 1e-7 and abs(lon) < 1e-7):
            self.gps_nofix = True      # fix 아님 — 버퍼에 안 넣고 rtk_fixed·삼각측량 출력을 바로 끈다
            return
        self.gps_nofix = False
        fixed, sigma = fl.fix_quality(msg.status.status, msg.position_covariance, msg.position_covariance_type,
                                      self.p['tri_rtk_max_sigma_m'])
        if self.geo0 is None:
            self.geo0 = (lat, lon)
        x, y = fl.ll_to_local(lat, lon, self.geo0[0], self.geo0[1])
        self.gps.push(now, (x, y, sigma if sigma is not None else float('inf'), fixed))

    def yaw_cb(self, msg):
        v = float(msg.data)
        if math.isfinite(v):
            self.yaw.push(time.monotonic(), v % 360.0)

    def imu_cb(self, msg):
        q = msg.orientation
        vals = (float(q.x), float(q.y), float(q.z), float(q.w))
        if not all(math.isfinite(v) for v in vals) or sum(v * v for v in vals) < 1e-6:
            return
        pitch, roll = fl.quat_pitch_roll(*vals)
        self.att.push(time.monotonic(), (self.p['imu_pitch_sign'] * pitch, self.p['imu_roll_sign'] * roll))

    # ---------------------- 타이머 ----------------------
    def publish_cb(self):
        now = time.monotonic()
        dets = fl.detections_from(now, self.red, self.green, self.image, self.image_color, self.p['det_stale_sec'])
        cams, cam_live, cam = self._camera_measurements(now)
        objs, stats = fl.build_objects(now, self.lidar, self.depth, dets, self.p, cams=cams, cam_live=cam_live)
        if cam is not None:
            stats['cam'] = cam
        self.stats = stats
        self._report_inputs(now, stats)
        if objs is None:
            return                     # 둘 다 stale → 침묵(미션 노드는 LiDAR 로 되돌아간다)
        msg = Float32MultiArray()
        msg.data = fl.encode(objs)
        self.pub_objects.publish(msg)

    def _camera_measurements(self, now):
        """→ (카메라 측정 목록 또는 None, cam_live, /fusion_status 용 dict 또는 None).

        둘 다 꺼짐 → (None, False, None): P1 이전 경로 그대로. /oak_detections 가 det_stale_sec 넘게 없으면
        (None, False, 상태) — 기여 비트도 안 채워 출력이 비트 동일하다.
        """
        p = self.p
        if not (p['use_mono'] or p['use_triangulation']):
            return None, False, None
        oak_ok = self.oak is not None and now - self.oak[0] <= p['det_stale_sec']
        _, _, imu_ok = fl.camera_attitude(self.att, now, p)
        pose = None
        if p['use_triangulation']:
            for rays in self.rayq.pop_ready(now, self.gps, self.yaw, self.att, p):
                self.tri.add(rays, p)
            self._report_ray_drops(now)
            if not self.gps_nofix:
                pose = fl.current_pose(self.gps, self.yaw, now, p)
        g = None if self.gps_nofix else fl.gps_now(self.gps, now, p)
        rtk_fixed = g is not None and g[3]            # 신선도 포함(끊김 1.5 주기·NO_FIX → False)
        if p['use_mono'] and not imu_ok:
            if not self.imu_warned:
                self.imu_warned = True
                self.get_logger().warn(
                    f"perception_fusion: /imu/data 없음·묵음({p['imu_stale_sec']} s) → 단안은 장착 피치 "
                    f"{p['oak_pitch_deg']}° · 롤 {p['oak_roll_deg']}° 만 (파도 피치 미보정 — σ 는 그대로)")
        elif imu_ok:
            self.imu_warned = False
        cam = {'use_mono': p['use_mono'], 'use_tri': p['use_triangulation'], 'oak_ok': oak_ok,
               'imu_ok': imu_ok, 'rtk_fixed': rtk_fixed}
        if not oak_ok:
            return None, False, cam
        cams = []
        if p['use_mono']:
            cams += fl.mono_measurements(self.oak[1], self.oak[0], now, self.att, p)
        if p['use_triangulation']:
            cams += self.tri.estimates(now, pose, p)
        return cams, True, cam

    def _report_ray_drops(self, now):
        """삼각측량 광선 버림 사유(yaw·gps·rtk·queue) — warn_period_sec 주기로 그동안 센 수를 WARN 하고 비운다."""
        drops = self.rayq.drops
        if sum(drops.values()) and self._due('ray_drop', now):
            d = self.rayq.take_drops()
            self.get_logger().warn(
                f"perception_fusion: 삼각측량 광선 버림 yaw {d['yaw']} · gps {d['gps']}(촬영 앞뒤 표본 없음·"
                f"{fl.GPS_GAP_FACTOR * self.p['gps_period_s']:g} s 넘는 간격) · rtk {d['rtk']}(FIXED 아님) · "
                f"queue {d['queue']}(대기 {self.p['tri_pending_max']} 넘침)")

    def _report_inputs(self, now, stats):
        lok, dok = stats['lidar_ok'], stats['depth_ok']
        if not lok and not dok:
            if self._due('lidar_stale', now):
                self.get_logger().warn(
                    f"perception_fusion: LiDAR /scan·뎁스 둘 다 stale(LiDAR {self.p['lidar_stale_sec']} s, "
                    f"뎁스 {self.p['depth_stale_sec']} s) → /fused_objects 발행 중단")
        elif not lok:
            if self._due('lidar_stale', now):
                self.get_logger().warn(
                    f"perception_fusion: LiDAR /scan stale({self.p['lidar_stale_sec']} s) → 뎁스만으로 계속(flags bit1)")
        if lok and not dok:
            if self._due('depth_stale', now):
                self.get_logger().warn(
                    f"perception_fusion: 뎁스 {self.p['depth_topic']} stale({self.p['depth_stale_sec']} s) → "
                    'LiDAR 만 (d455_depth:=false 면 정상)')
        if self.inputs_ok != (lok, dok):
            self.inputs_ok = (lok, dok)
            self.get_logger().info(
                f"perception_fusion 입력: {'lidar_ok' if lok else 'lidar_stale'} "
                f"{'depth_ok' if dok else 'depth_stale'}")

    def status_cb(self):
        msg = String()
        msg.data = fl.format_status(self.stats, self.wp_mode)
        self.pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionFusion()
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
