"""perception_fusion — LiDAR 군집 + D455 뎁스 덩어리 + OAK 검출 방위(색) → 물체 목록 /fused_objects.

계약서_인식융합_20261112 (§10 결정이 본문보다 우선), 근거 계약서_거리융합_20261112 §4·§5.
  · 미션 노드(ship_back·ship_turn APPROACH·ship_gate)는 distance_source 가 fill/fused 일 때만 이 목록을 본다.
    기본 lidar 면 구독도 안 한다. 대회 yaml = fill(LiDAR 먼저, 못 볼 때만 이 목록).
  · CLAUDE.md §3 "카메라는 방위각만, 거리는 LiDAR" 의 예외(§10 Q1 승인) — distance_source fill/fused 일 때만.
  · 물 위 실측 0회(§10). D455 장착 오프셋·행 띠·거리 범위는 ⚠️ 실측 필요 값(yaml).

입력
  /scan (LaserScan, BEST_EFFORT d1) — a = 80 + rel(좌 +), 정면 ±90° 만
  depth_topic (Image 16UC1 mm, 0 = 모름, BEST_EFFORT d1) + depth_info_topic (CameraInfo: fx = k[0], cx = k[2])
  /red_angle·/green_angle·/image_angle (Float32 상대각 좌 +, 10000 = 안 보임) + /image_color (String) — 색 붙이기만
  /wp_mode (Int32) — 관측만(/fusion_status 에 찍음)
출력
  /fused_objects (Float32MultiArray, RELIABLE 10, publish_hz 고정주기 타이머) — 형식은 fusion_logic 머리말
    LiDAR stale → 뎁스만(flags bit1), 둘 다 stale → 발행 중단(침묵), 빈 목록 = 살아 있고 물체 없음
  /fusion_status (String, RELIABLE 10, status_hz) — 예 'L0.82 D0.61 agree0.93 lidar_ok depth_ok n=3 m=2'
침묵·오류: 입력 stale = warn_period_sec 주기 WARN, 파라미터 무효 = ERROR + 기본값, 뎁스 형식 오류 = 주기 ERROR.
  /health_ok 는 건드리지 않는다(§10 Q8: 침묵은 로그만). 로거는 심각도마다 호출 줄을 나눈다(§1-7).
"""

import time

from perception_fusion import fusion_logic as fl
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Float32, Float32MultiArray, Int32, String

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
        # 제어 소비자가 있는 출력이라 고정주기 타이머에서만 낸다(데이터 콜백에서 내지 않는다).
        self.create_timer(1.0 / p['publish_hz'], self.publish_cb)
        self.create_timer(1.0 / p['status_hz'], self.status_cb)

        self.get_logger().info(
            f"perception_fusion 시작: /scan + {p['depth_topic']} (+ {p['depth_info_topic']}) + 검출기 방위 → "
            f"/fused_objects {p['publish_hz']} Hz (VER {fl.VER}, STRIDE {fl.STRIDE}). stale LiDAR "
            f"{p['lidar_stale_sec']} s / 뎁스 {p['depth_stale_sec']} s, 뎁스 {p['depth_min_m']}~{p['depth_max_m']} m, "
            f"행 띠 {p['depth_row_lo_frac']}~{p['depth_row_hi_frac']}, 장착 x {p['cam_x_from_lidar_m']} "
            f"y {p['cam_y_from_lidar_m']} yaw {p['cam_yaw_offset_deg']}° (⚠️ 실측 0회)")

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

    # ---------------------- 타이머 ----------------------
    def publish_cb(self):
        now = time.monotonic()
        dets = fl.detections_from(now, self.red, self.green, self.image, self.image_color, self.p['det_stale_sec'])
        objs, stats = fl.build_objects(now, self.lidar, self.depth, dets, self.p)
        self.stats = stats
        self._report_inputs(now, stats)
        if objs is None:
            return                     # 둘 다 stale → 침묵(미션 노드는 LiDAR 로 되돌아간다)
        msg = Float32MultiArray()
        msg.data = fl.encode(objs)
        self.pub_objects.publish(msg)

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
