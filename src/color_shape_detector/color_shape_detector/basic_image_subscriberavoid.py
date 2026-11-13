"""도리도리 회피 검출기 — 노드 이름 image_subscriber_avoid (mode 5 담당 ship_avoid 의 눈).

계약서: docs/문제와작업/계약서_도리도리회피_v2_20261114.md U3 (=C6a, 비전 원자 그룹 — vision.yaml·setup.py·
        test_mode_gate.py·launch 와 같은 커밋)

입력  OAK  /oak/rgb/image_raw (image_topic, vision.yaml `/**`) + /oak/rgb/camera_info (camera_info_topic)
      — SENSOR_QOS(BEST_EFFORT, depth 1, gate 검출기와 같음)
      /wp_mode (RELIABLE 10) — ModeGate, 담당 active_wp_modes [5]
출력  /avoid_detections  Float32MultiArray  RELIABLE 10
      [VER=1, STRIDE=6, N, N × (cls 0빨강/2노랑, bearing_left_deg, v_norm, range_m=NaN, conf, age_s)]
      형식·각 칸의 뜻은 avoid_detect_logic docstring. 구독자 = ship_avoid(avoid_mode_logic.parse_detections).

발행 시점 = **프레임 콜백**(고정 주기 타이머가 아니다) — 기존 검출기 3종과 같은 관례.
  · 하트비트(N=0)는 "이 프레임을 봤는데 없다" 는 뜻이다. 타이머로 내면 카메라가 죽어도 N=0 이 계속 나가
    ship_avoid 가 '살아 있고 아무것도 없음' 으로 믿는다 = 모르는데 0 을 내는 것(§1-6 위반).
    프레임마다 내면 카메라·camera_info 가 죽을 때 토픽이 침묵하고, ship_avoid 의 고정 주기 timer_cb 가
    detections_quiet_warn_sec(5 s) 뒤 DET_DEAD(20000 + ERROR)로 드러낸다 — 워치독은 소비자 타이머에 있다(§5).
  · 제어 출력(/candidate_angle)은 ship_avoid 가 자기 타이머에서 낸다 — 이 노드는 관측만 낸다.
  · mode 5 진입부터 하트비트: /wp_mode 5 를 받은 뒤 첫 프레임부터 낸다(ModeGate 는 수신 즉시 활성).
    ship_avoid 의 검출 침묵 시계도 mode 5 진입부터 센다(U2 리뷰 메모).

안 내는 경우 (모르면 입을 다문다 — §1-6)
  · 담당 모드 아님 / /wp_mode 미수신·stale(2 s) → 처리·발행 없음(미수신·stale 은 5 s 주기 WARN, gate 와 같음)
  · camera_info 미수신·K 무효·해상도 불일치 → 발행 없음 + ERROR(5 s throttle). hfov_deg 로 대신하지 않는다
    (계약: 방위는 camera_info fx·cx). → ship_avoid 가 5 s 뒤 DET_DEAD 로 같이 드러낸다.

🚨 §7-2 안전 스위치: active_wp_modes [5] · debug_view false. 바꾸면 test_mode_gate.py LAYOUT 도 같이.
"""

import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32MultiArray, Int32

from color_shape_detector import avoid_detect_logic as adl
from color_shape_detector import hsv_ranges
from color_shape_detector.mode_gate import ModeGate, mode_changed

# 이미지 구독: 묵은 프레임으로 판단하지 않는다(CLAUDE.md §3 QoS) — basic_image_subscribergate.SENSOR_QOS 와 같다.
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
# /avoid_detections 발행: ship_avoid 구독(RELIABLE depth 10)과 같다. 관찰 토픽도 발행은 RELIABLE(§3 QoS).
DET_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


def _stamp_s(stamp):
    """builtin_interfaces/Time → 초. 못 읽으면 None."""
    try:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    except (AttributeError, TypeError, ValueError):
        return None


class ImageSubscriberAvoid(Node):
    def __init__(self):
        super().__init__('image_subscriber_avoid')

        # ---- 파라미터 — 🚨 구독·발행자 생성보다 먼저(뒤면 부팅 즉시 AttributeError) ----
        # 이미지 토픽은 vision.yaml `/**` 공통값(OAK). camera_info 는 이 노드만 쓴다(절에 있음).
        self.image_topic = str(self.declare_parameter('image_topic', '/oak/rgb/image_raw').value)
        self.camera_info_topic = str(self.declare_parameter('camera_info_topic', '/oak/rgb/camera_info').value)
        # 🚨 §7-2: 담당 모드 [5]. 권위 출처 = ship_avoid active_wp_mode(5). test_mode_gate LAYOUT 이 겹침을 본다.
        self.mode_gate = ModeGate(
            self.declare_parameter('active_wp_modes', [5]).value,
            stale_sec=float(self.declare_parameter('wp_mode_stale_sec', 2.0).value))
        self._last_gate_log = 0.0
        # 🚨 §7-2: 헤드리스에서 cv2.imshow 는 노드를 죽인다 → 기본 false.
        self.debug_view = bool(self.declare_parameter('debug_view', False).value)
        # HSV — vision.yaml `/**` 의 hsv.red(두 구간)·hsv.yellow 가 단일 출처(gate·turn·dock 과 같은 값).
        #   잘못된 값이면 기본값 + ERROR(hsv_ranges.load).
        self.color_ranges = hsv_ranges.load(
            lambda n, d: self.declare_parameter(n, d).value,
            tuple(adl.COLOR_CLS),
            on_error=self.get_logger().error)
        # ⚠️ 실측 필요 — 아래 넷은 vision.yaml image_subscriber_avoid 절 = adl.DEFAULT_PARAMS(test_avoid_detect_logic).
        #   범위 밖이면 그 값만 기본값 + 부팅 ERROR 1회(adl.check_params).
        raw = {
            'min_area_px': self.declare_parameter('min_area_px', 40.0).value,
            'min_solidity': self.declare_parameter('min_solidity', 0.7).value,
            'conf_full_area_px': self.declare_parameter('conf_full_area_px', 400.0).value,
            'max_detections': self.declare_parameter('max_detections', 16).value,
        }
        vals, errs = adl.check_params(raw)
        for e in errs:
            self.get_logger().error(e)
        self.min_area_px = vals['min_area_px']
        self.min_solidity = vals['min_solidity']
        self.conf_full_area_px = vals['conf_full_area_px']
        self.max_detections = vals['max_detections']

        self.br = CvBridge()
        self.cam_info = None

        # ---- 구독·발행 ----
        self.create_subscription(Int32, '/wp_mode', self.wp_mode_callback, 10)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, SENSOR_QOS)
        self.create_subscription(Image, self.image_topic, self.color_callback, SENSOR_QOS)
        self.det_pub = self.create_publisher(Float32MultiArray, '/avoid_detections', DET_QOS)

        self.get_logger().info(
            f"도리도리 검출기 시작: mode {sorted(self.mode_gate.active_modes)} 에서 {self.image_topic} + "
            f"{self.camera_info_topic} → /avoid_detections [VER {adl.DET_VER}, STRIDE {adl.DET_STRIDE}, N, …] "
            f"(색 {list(self.color_ranges)}, min_area_px {self.min_area_px:g}, min_solidity {self.min_solidity:g}, "
            f"최대 {self.max_detections}개, range_m NaN, debug_view {self.debug_view})")

    # ------------------------------------------------------------------ 콜백
    def wp_mode_callback(self, msg):
        prev = self.mode_gate.mode
        if not self.mode_gate.update(msg.data, time.monotonic()):
            return
        if mode_changed(prev, self.mode_gate.mode) and self.mode_gate.mode in self.mode_gate.active_modes:
            # 이 검출기는 유예·확정 창이 없어 비울 것이 없다. 진입만 알린다(하트비트 시작 시점 확인용).
            self.get_logger().info(f"wp_mode {prev}→{self.mode_gate.mode}: /avoid_detections 프레임마다 발행 시작")

    def camera_info_callback(self, msg):
        self.cam_info = msg

    def color_callback(self, msg):
        # 🚨 게이트가 맨 앞 — 담당 모드가 아니면 변환·HSV·윤곽·발행 전부 건너뛴다(§3, test_detector_gate_order 패턴).
        t0 = time.monotonic()
        active, why = self.mode_gate.state(t0)
        if not active:
            if why != ModeGate.R_OTHER_MODE and (t0 - self._last_gate_log) > 5.0:
                self._last_gate_log = t0
                self.get_logger().warn(f"비활성({why}) → 검출 정지. /wp_mode 가 오고 있나?")
            return

        intr = self._intrinsics(msg)
        if intr is None:                      # ERROR 는 _intrinsics 가 냈다 — 발행 안 함
            return

        frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        blobs = adl.detect_blobs(frame, self.color_ranges, self.min_area_px, self.min_solidity,
                                 self.max_detections)
        age, raw, src = adl.frame_age_s(self.get_clock().now().nanoseconds * 1e-9,
                                        _stamp_s(msg.header.stamp), time.monotonic() - t0)
        if raw < 0.0:
            self.get_logger().warn(
                f"이미지 header.stamp 가 노트북 시각보다 {-raw:.3f} s 앞섬 → age_s 0 으로 냄(카메라 시각 동기 확인)",
                throttle_duration_sec=10.0)
        dets = adl.blobs_to_detections(blobs, intr, age, self.conf_full_area_px)
        data, dropped = adl.pack_detections(dets)
        if dropped:
            self.get_logger().error(
                f"/avoid_detections 검출 {dropped}개 값 이상(비유한) → 그 검출만 뺌 (age 출처 {src})",
                throttle_duration_sec=5.0)
        self.det_pub.publish(Float32MultiArray(data=data))

        if self.debug_view:
            view = frame.copy()
            for b in blobs:
                cv2.drawContours(view, [b['contour']], -1, (0, 255, 0), 2)
                cv2.putText(view, f"{b['color']} {b['area']:.0f}px", (int(b['u']), int(b['v_bottom'])),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.imshow("Avoid detections", view)
            cv2.waitKey(1)

    # ------------------------------------------------------------------ camera_info
    def _intrinsics(self, img):
        """→ (fx, fy, cx, cy) 또는 None(+ ERROR). 방위·v_norm 을 지어내지 않는다."""
        ci = self.cam_info
        if ci is None:
            self.get_logger().error(
                f"{self.camera_info_topic} 미수신 → /avoid_detections 안 냄(방위를 hfov 로 지어내지 않음). "
                "OAK 드라이버·토픽 이름 확인", throttle_duration_sec=5.0)
            return None
        intr = adl.intrinsics_from_camera_info(ci.k, ci.width, ci.height)
        if intr is None:
            self.get_logger().error(
                f"{self.camera_info_topic} K 무효(fx·fy ≤ 0 또는 비유한, 크기 {ci.width}x{ci.height}) "
                "→ /avoid_detections 안 냄. OAK 캘리브 확인", throttle_duration_sec=5.0)
            return None
        if (int(ci.width), int(ci.height)) != (int(img.width), int(img.height)):
            self.get_logger().error(
                f"camera_info {ci.width}x{ci.height} ≠ 이미지 {img.width}x{img.height} → /avoid_detections 안 냄 "
                "(다른 해상도의 K 로 방위를 내지 않음). OAK i_width/i_height·ISP 확인", throttle_duration_sec=5.0)
            return None
        return intr


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriberAvoid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
