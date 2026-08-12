import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from color_shape_detector.mode_gate import ModeGate
from color_shape_detector.vision_geom import (
    DEFAULT_HFOV_DEG, angle_from_pixel,
)
from color_shape_detector import hsv_ranges
from color_shape_detector.dock_logic import (
    VALID_COLORS, VALID_SHAPES, classify_shape, DetectionConfirmer,
)
import cv2
import numpy as np
import time
from rclpy.executors import MultiThreadedExecutor

IMAGE_ANGLE_INVALID = 10000.0

# V3(T2-4): 픽셀→각도 환산을 명시형으로 재작성했다.
#   작년: (rel_x/80.0)*0.09*(distance/0.5) + atan2  ← distance 가 약분되어 실질 fx≈444px **고정**
#   지금: fx = (width/2)/tan(hfov/2),  angle = -degrees(atan((vX-cx)/fx))
#   width 는 파라미터가 아니라 **매 프레임 실제 이미지에서** 읽는다(해상도 변경 자동 흡수).
#   ※ 작년 식은 fx 가 고정이라 640x480 에서만 맞았다 — 해상도를 바꾸면 조용히 각도가 틀어졌다.
#   순수 함수 + 회귀 테스트: vision_geom.py / test_vision_geom.py

# V4(T2-6): 표준 sensor-data QoS (작년 depth=10 RELIABLE).
#   콜백이 밀리면 묵은 프레임이 쌓여 '몇 백 ms 전 장면' 으로 조향한다. depth=1 = 항상 최신.
#   구독자 BEST_EFFORT 는 발행자가 RELIABLE 이어도 호환된다(그 반대가 비호환).
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber_dock')

        # V5(T2-2): 이미지 토픽 파라미터화. 기본값은 현 RealSense 토픽 그대로다.
        #   OAK 로 바꾸는 날 코드를 고치지 않고 **yaml 한 줄**로 끝내려는 것이다.
        #   ⚠️ 구독보다 반드시 먼저 선언해야 한다 — 뒤에 두면 부팅 즉시 AttributeError 다.
        #   3-4 이후 launch 가 이 노드를 직접 띄우므로 yaml 파라미터가 그대로 닿는다.
        self.image_topic = str(self.declare_parameter(
            'image_topic', '/camera/camera/color/image_raw').value)

        # 3-4: 상주 + 모드 게이팅. subprocess 로 죽였다 살리는 방식을 폐기했다.
        #   담당 모드는 **각 노드가 소유**한다 (미션 노드 ship_gate/dock/turn/back 과 같은 패턴).
        #   🚨 dock 과 turn 은 둘 다 /image_angle 을 발행한다 — 모드가 겹치면 한 토픽에
        #      발행자 2개가 되어 에러 없이 값이 섞인다. 겹침 없음은 test_mode_gate 가 검사한다.
        #   ⚠️ 권위 출처는 미션 노드의 active_wp_mode 다. 바꿀 땐 양쪽을 함께 바꿔라.
        self.mode_gate = ModeGate(
            self.declare_parameter('active_wp_modes', [7]).value,
            stale_sec=float(self.declare_parameter('wp_mode_stale_sec', 2.0).value))
        self._last_gate_log = 0.0

        # Color 이미지 구독
        # V1(T2-3): depth 구독 제거 — OAK 는 뎁스가 없어 depth 를 기다리면 침묵 사망한다.
        #   거리는 소비자(ship_dock)가 LiDAR 전방 섹터 최소거리로 구한다.
        self.subscription_color = self.create_subscription(
            Image,
            self.image_topic,
            self.color_callback,
            SENSOR_QOS)

        # 3-4: 모드 게이팅용. 상주하면서 자기 차례일 때만 일한다.
        self.create_subscription(
            Int32, '/wp_mode', self.wp_mode_callback, 10)

        self.br = CvBridge()

        # 퍼블리셔: angle 만 (이름·타입 불변)
        # V1: /image_distance 발행 제거 — 소비자 0개(6단계에서 ship_dock/turn/back 이 LiDAR 로 전환).
        self.angle_pub    = self.create_publisher(Float32, '/image_angle', 10)

        # fallback
        self.last_valid = {
            'angle': None,
            'time': 0.0
        }
        self.grace_period_s = 2.0

        # V4(T2-5): 헤드리스에서 cv2.imshow 는 예외로 노드를 죽인다. 배는 SSH 로 띄운다.
        #   try/except 로 덮지 않고 파라미터로 원천 차단. false 면 frame.copy() 와 그리기도 생략.
        self.debug_view = bool(self.declare_parameter('debug_view', False).value)

        # V3(T2-4): 카메라 수평 화각. 기본값은 현 RealSense 를 역산한 값이다.
        #   OAK-1 W(광각)로 바꾸면 **이 값만 yaml 에서 고치면 된다.**
        #   하드코딩이었을 땐 카메라 교체일에 모든 각도 출력과 상위 튜닝
        #   (align_tol_deg, pair_min/max_sep_deg …)이 통째로 무효가 됐다.
        #   ⚠️ 광각은 핀홀 모델이 가장자리에서 깨진다 — rectified 토픽 구독 또는
        #      camera_info 의 왜곡계수 D 적용이 추가로 필요하다.
        self.hfov_deg = float(
            self.declare_parameter('hfov_deg', DEFAULT_HFOV_DEG).value)

        self.found_in_frame = False
        self.last_log_time = time.time()

        # ---- D1: 목표 표식 파라미터화 (당일 아침 공지 대응) ----
        # 작년: target_color="red", target_shape="Square" 하드코딩 → 공지된 조합이 다르면
        #   코드를 고쳐야 했다. 파라미터로 빼서 yaml 한 줄(또는 --ros-args)로 바꾼다.
        #   유효값 검증: 오타(예: "Squre")면 조용히 아무것도 못 찾는다 → ERROR 로그 + 발행 중단.
        self.target_color = str(self.declare_parameter('target_color', 'red').value)
        self.target_shape = str(self.declare_parameter('target_shape', 'Square').value)
        self._target_valid = True
        if self.target_color not in VALID_COLORS:
            self._target_valid = False
            self.get_logger().error(
                f"🚨 target_color '{self.target_color}' 는 유효하지 않다. "
                f"가능: {list(VALID_COLORS)}. 발행 중단 — 오타로 조용히 못 찾는 것 방지.")
        if self.target_shape not in VALID_SHAPES:
            self._target_valid = False
            self.get_logger().error(
                f"🚨 target_shape '{self.target_shape}' 는 유효하지 않다. "
                f"가능: {list(VALID_SHAPES)}. 발행 중단.")

        # ---- D2: N프레임 시간 안정화 ----
        self.confirm_frames = int(self.declare_parameter('confirm_frames', 3).value)
        # grace_period 도 파라미터로 (열화 환경에서 N·grace 를 올려 대응)
        self.grace_period_s = float(
            self.declare_parameter('grace_period_s', 2.0).value)
        self.confirmer = DetectionConfirmer(confirm_frames=self.confirm_frames)

        # ---- D3: 도형 분류·검출 임계 파라미터화 ----
        self.approx_epsilon = float(self.declare_parameter('approx_epsilon', 0.0315).value)
        self.min_area_px = float(self.declare_parameter('min_area_px', 80.0).value)
        self.square_extent_min = float(
            self.declare_parameter('square_extent_min', 0.4).value)
        self.square_aspect_lo = float(self.declare_parameter('square_aspect_lo', 0.6).value)
        self.square_aspect_hi = float(self.declare_parameter('square_aspect_hi', 1.4).value)
        self.circle_circularity_min = float(
            self.declare_parameter('circle_circularity_min', 0.82).value)
        self.square_circularity_max = float(
            self.declare_parameter('square_circularity_max', 0.60).value)
        self.y_zone_lo = float(self.declare_parameter('y_zone_lo', 0.15).value)
        self.y_zone_hi = float(self.declare_parameter('y_zone_hi', 0.55).value)

        # ---- D5: HSV 6색 슬롯 — config/vision.yaml 의 hsv.* 가 단일 출처 ----
        # 미션 표식 색이 당일 아침 공지되므로 슬롯을 전부 열어둔다.
        # 🚨 [2026-08-12] 여기 박혀 있던 값 중 red·green 이 gate/turn 과 **달랐다**:
        #    red 는 V≥200 이라 어두운 빨강을 놓쳤고, green 은 H 28~40(연두)에 S≤100 이라
        #    **선명한 초록을 오히려 거부**했다 — 같은 부표를 게이트는 보고 도킹은 못 봤다.
        #    버린 값은 hsv_ranges.SUPERSEDED 에 남겨 뒀다.
        self.color_ranges = hsv_ranges.load(
            lambda n, d: self.declare_parameter(n, d).value,
            hsv_ranges.VALID_COLORS,
            on_error=self.get_logger().error)


    # ============================================
    # Color topic 콜백
    # ============================================
    def wp_mode_callback(self, msg):
        self.mode_gate.update(msg.data, time.monotonic())

    def color_callback(self, msg):
        # D1: 목표 표식이 유효하지 않으면(오타 등) 검출을 하지 않는다. 센티널만 발행.
        #   '조용히 아무것도 못 찾는' 상태를 만들지 않는다 — 소비자는 INVALID 를 명확히 본다.
        if not self._target_valid:
            self.angle_pub.publish(Float32(data=IMAGE_ANGLE_INVALID))
            return

        # 3-4: 내 차례가 아니면 **여기서 끝낸다.**
        #   cv_bridge 변환·HSV·컨투어를 전부 건너뛴다 → 비활성 노드의 CPU 는 거의 0.
        #   발행도 안 한다 → /image_angle 발행자 충돌이 원천 차단된다.
        now_m = time.monotonic()
        active, why = self.mode_gate.state(now_m)
        if not active:
            if why != ModeGate.R_OTHER_MODE and (now_m - self._last_gate_log) > 5.0:
                self._last_gate_log = now_m
                self.get_logger().warn(
                    f"비활성({why}) → 검출 정지. /wp_mode 가 오고 있나?")
            return

        frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        view_frame = frame.copy() if self.debug_view else None

        # 이번 프레임에서 목표 표식을 찾는다 → (각도, (색,형상)) 또는 (None, None)
        angle_deg, key = self.process_image(frame, view_frame)

        # D2: N프레임 시간 안정화. 같은 (색,형상)이 confirm_frames 연속일 때만 확정.
        #   한 프레임 오탐이 그대로 조향에 튀는 것을 막는다.
        confirmed = self.confirmer.update(key)

        if self.debug_view:
            cv2.imshow("Dock Detection", view_frame)
            cv2.waitKey(1)

        now = time.time()

        def publish(angle_val):
            self.angle_pub.publish(Float32(data=float(angle_val)))

        if confirmed is not None and angle_deg is not None:
            # 확정 + 이번 프레임 각도 있음 → 발행
            self.last_valid['angle'] = float(angle_deg)
            self.last_valid['time'] = now
            publish(angle_deg)
            if now - self.last_log_time > 0.5:
                self.get_logger().info(
                    f"[Dock] {confirmed[0]} {confirmed[1]}: angle={angle_deg:.1f}")
                self.last_log_time = now
        else:
            # 미확정/미검출 → grace 기간이면 마지막 유효값, 아니면 센티널
            if (self.last_valid['time'] > 0 and
                    (now - self.last_valid['time']) <= self.grace_period_s and
                    self.last_valid['angle'] is not None):
                publish(self.last_valid['angle'])
            else:
                publish(IMAGE_ANGLE_INVALID)


    # ============================================
    # 이미지 처리 (도형 인식) — 이번 프레임의 (각도, (색,형상)) 반환
    # ============================================
    def process_image(self, cv_image, view_frame):
        # V1(T2-3): depth 가드 제거 (뎁스 없는 카메라에서 콜백이 영원히 막히는 것 방지)
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        img_h, img_w = cv_image.shape[:2]

        ranges = self.color_ranges.get(self.target_color)
        if ranges is None:
            return None, None

        mask = np.zeros(hsv_image.shape[:2], dtype=np.uint8)
        for lower, upper in ranges:
            mask |= cv2.inRange(hsv_image, np.array(lower), np.array(upper))
        mask = cv2.medianBlur(mask, 3)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 목표 형상과 일치하는 후보 중 **가장 큰 것**을 고른다 (가까울수록 크다).
        best = None   # (area, angle, vX, vY, approx)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area_px:
                continue

            perimeter = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, self.approx_epsilon * perimeter, True)
            x, y, w, h = cv2.boundingRect(approx)

            # D3: 형상 분류는 순수 로직(원형도 보조). 임계는 전부 파라미터.
            shape = classify_shape(
                len(approx), area, perimeter, w, h,
                min_area=self.min_area_px,
                square_extent_min=self.square_extent_min,
                square_aspect_lo=self.square_aspect_lo,
                square_aspect_hi=self.square_aspect_hi,
                circle_circularity_min=self.circle_circularity_min,
                square_circularity_max=self.square_circularity_max)
            if shape != self.target_shape:
                continue

            M = cv2.moments(contour)
            if M["m00"] == 0:
                continue
            vertices = approx.reshape(-1, 2)
            vX = int(np.mean(vertices[:, 0]))
            vY = int(np.mean(vertices[:, 1]))

            # y범위 필터 (파라미터화). ⚠️ 실측 필요 — 거리별 표식 화면 높이로 확정.
            if not (img_h * self.y_zone_lo <= vY <= img_h * self.y_zone_hi):
                continue

            if best is None or area > best[0]:
                best = (area, angle_from_pixel(vX, img_w, self.hfov_deg), vX, vY, approx)

        if best is None:
            return None, None

        _, angle_deg, vX, vY, approx = best
        if self.debug_view:
            cv2.drawContours(view_frame, [approx], -1, (0, 255, 0), 2)
            cv2.circle(view_frame, (vX, vY), 4, (255, 255, 0), -1)
            cv2.putText(view_frame,
                        f"{self.target_color} {self.target_shape} {angle_deg:.1f}deg",
                        (vX + 10, vY - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        return angle_deg, (self.target_color, self.target_shape)



def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # D5: node.pipeline.stop() 제거 — 존재하지 않는 속성이라 종료 때마다 예외였다.
        #     (RealSense pyrealsense 파이프라인 잔재. 이 노드는 ROS 이미지 구독이라 파이프라인이 없다.)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
