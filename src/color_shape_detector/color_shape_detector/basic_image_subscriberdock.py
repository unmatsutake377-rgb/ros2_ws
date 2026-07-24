import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from color_shape_detector.vision_geom import (
    DEFAULT_HFOV_DEG, angle_from_pixel,
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
        #   ⚠️ 이 노드는 subscribermode 가 `ros2 run` 으로 띄운다 → launch 파라미터가 안 닿는다.
        #      subscribermode 가 --ros-args 로 넘겨준다(vision_image_topic).
        self.image_topic = str(self.declare_parameter(
            'image_topic', '/camera/camera/color/image_raw').value)

        # Color 이미지 구독
        # V1(T2-3): depth 구독 제거 — OAK 는 뎁스가 없어 depth 를 기다리면 침묵 사망한다.
        #   거리는 소비자(ship_dock)가 LiDAR 전방 섹터 최소거리로 구한다.
        self.subscription_color = self.create_subscription(
            Image,
            self.image_topic,
            self.color_callback,
            SENSOR_QOS)

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

        # 목표 도형
        self.target_color = "red"
        self.target_shape = "Square"


    # ============================================
    # Color topic 콜백
    # ============================================
    def color_callback(self, msg):

        self.found_in_frame = False

        frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        view_frame = frame.copy() if self.debug_view else None

        self.process_image(frame, view_frame)

        if self.debug_view:
            cv2.imshow("Dock Detection", view_frame)
            cv2.waitKey(1)

        # fallback 처리
        now = time.time()

        def publish(angle_val):
            self.angle_pub.publish(Float32(data=float(angle_val)))

        if not self.found_in_frame:
            if (
                self.last_valid['time'] > 0 and
                (now - self.last_valid['time']) <= self.grace_period_s and
                self.last_valid['angle'] is not None
            ):
                publish(self.last_valid['angle'])
            else:
                publish(IMAGE_ANGLE_INVALID)


    # ============================================
    # 이미지 처리 (도형 인식)
    # ============================================
    def process_image(self, cv_image, view_frame):
        # V1(T2-3): depth 가드 제거 (뎁스 없는 카메라에서 콜백이 영원히 막히는 것 방지)
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        img_h, img_w = cv_image.shape[:2]
        cy = img_h // 2

        # HSV 범위 (너가 준 그대로)
       
        color_ranges = {
            "red":    [([0, 80, 200], [5, 255, 255]), ([165, 80, 200], [180, 255, 255])],
            #"orange": [([3, 130, 100], [20, 255, 255])],
            #"yellow": [([21, 120, 60], [37, 255, 255])],
            "green":  [([28, 30, 235], [40, 100, 255])],
            "blue":   [([130, 18, 160], [175, 60, 200])],
            #"white":  [([0, 240, 240], [180, 255, 255])]
                     }#도킹용)


        for color, ranges in color_ranges.items():

            if color != self.target_color:
                continue

            mask = np.zeros(hsv_image.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:
                mask |= cv2.inRange(hsv_image, np.array(lower), np.array(upper))

            mask = cv2.medianBlur(mask, 3)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:

                area = cv2.contourArea(contour)
                if area < 80:
                    continue

                approx = cv2.approxPolyDP(contour, 0.0315 * cv2.arcLength(contour, True), True)
                v_count = len(approx)

                shape = None

                # 기존 조건을 그대로 유지
                if v_count == 3:
                    shape = "Triangle"

                elif 4 <= v_count <= 6:
                    x, y, w, h = cv2.boundingRect(approx)
                    aspect_ratio = w / float(h)
                    extent = area / float(w * h)

                    if extent > 0.4:
                        shape = "Square"

                elif v_count >= 7:
                    shape = "Circle"

                # 목표 도형이 아니면 continue
                if shape != self.target_shape:
                    continue

                # 중심 계산
                M = cv2.moments(contour)
                if M["m00"] == 0:
                    continue

                vertices = approx.reshape(-1, 2)
                vX = int(np.mean(vertices[:, 0]))
                vY = int(np.mean(vertices[:, 1]))

                # y-coordinate filtering: valid detection zone only
                if not (img_h * 0.15 <= vY <= img_h * 0.55):  #위쪽범위 <= vY <=아래쪽범위
                    continue

                # angle 계산 (depth 불필요 — 기존 식에서 distance 가 약분되어 값이 완전히 같다)
                angle_deg = angle_from_pixel(vX, img_w, self.hfov_deg)

                # 퍼블리시 갱신 (각도만)
                self.angle_pub.publish(Float32(data=float(angle_deg)))

                self.found_in_frame = True
                self.last_valid['angle'] = float(angle_deg)
                self.last_valid['time'] = time.time()

                # 시각화 (디버그 창이 켜져 있을 때만)
                if self.debug_view:
                    cv2.drawContours(view_frame, [approx], -1, (0, 255, 0), 2)
                    cv2.circle(view_frame, (vX, vY), 4, (255, 255, 0), -1)
                    cv2.putText(view_frame,
                                f"{color} {shape} {angle_deg:.1f}deg",
                                (vX + 10, vY - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)

                if time.time() - self.last_log_time > 0.5:
                    print(f"[Dock] {color} {shape}: angle={angle_deg:.1f}")
                    self.last_log_time = time.time()

        return



def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    node.pipeline.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
