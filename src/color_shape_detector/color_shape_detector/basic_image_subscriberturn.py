import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String, Int32
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
import numpy as np
import math
import time

IMAGE_ANGLE_INVALID = 10000.0

# ⚠️ 임시 — V3(T2-4)에서 hfov_deg 파라미터 + msg.width 기반으로 재작성.
# 기존 `(rel_x/80.0)*0.09*(distance/0.5)` + atan2 는 distance 가 약분되어 아래와 완전히 같다.
#   K = 0.00225 → 등가 fx ≈ 444.4px → 640px 기준 HFOV ≈ 71.5° (RealSense 화각이 박혀 있었다)
PIXEL_TO_ANGLE_K = (1.0 / 80.0) * 0.09 / 0.5

# V4(T2-6): 표준 sensor-data QoS (작년 depth=10 RELIABLE).
#   콜백이 밀리면 묵은 프레임이 쌓여 '몇 백 ms 전 장면' 으로 조향한다. depth=1 = 항상 최신.
#   구독자 BEST_EFFORT 는 발행자가 RELIABLE 이어도 호환된다(그 반대가 비호환).
#   ⚠️ 이미지에만 적용한다. /wp_mode 는 센서가 아니라 '모드 명령' 이라 한 장도 놓치면 안 된다
#      — RELIABLE 유지.
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber_turn')

        self.br = CvBridge()

        # =============================
        # SUBSCRIBERS
        # =============================
        self.color_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.color_callback,
            SENSOR_QOS)

        # ★ 추가: WP 모드 구독
        self.wp_mode_sub = self.create_subscription(
            Int32,
            '/wp_mode',
            self.wp_mode_callback,
            10)

        # =============================
        # PUBLISHERS
        # =============================
        # V1(T2-3): /image_distance 발행 제거 — 소비자 0개(ship_turn 이 LiDAR 로 전환).
        self.angle_pub = self.create_publisher(Float32, '/image_angle', 10)
        self.color_pub = self.create_publisher(String, '/image_color', 10)

        # =============================
        # STATE / FALLBACK
        # =============================
        self.last_valid = {
            'angle': None,
            'color': None,
            'time': 0.0
        }
        self.grace_period_s = 2.0
        # V4(T2-5): 헤드리스에서 cv2.imshow 는 예외로 노드를 죽인다. 배는 SSH 로 띄운다.
        #   try/except 로 덮지 않고 파라미터로 원천 차단. false 면 frame.copy() 와 그리기도 생략.
        self.debug_view = bool(self.declare_parameter('debug_view', False).value)

        self.found_in_frame = False

        # ★ 추가: WP 모드 초기값
        self.wp_mode = -1

        # ★ 추가: 각 WP에서 인식할 색상 지정
        self.wp_color_map = {
            2: ["white"],
            3: ["red"],
            4: ["red"],
            5: ["red"]
        }


    def wp_mode_callback(self, msg: Int32):
        self.wp_mode = msg.data


    def color_callback(self, msg):
        # V1(T2-3): depth 가드 제거 — 뎁스 없는 카메라에서 콜백이 영원히 막히는 것 방지
        self.found_in_frame = False

        frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        view = frame.copy() if self.debug_view else None

        self.process_image(frame, view)

        if self.debug_view:
            cv2.imshow("TURN Detection", view)
            cv2.waitKey(1)

        # ========== Fallback ==========
        now = time.time()

        def publish(angle, color):
            self.angle_pub.publish(Float32(data=float(angle)))
            self.color_pub.publish(String(data=str(color)))

        if not self.found_in_frame:
            lv = self.last_valid
            if (
                lv['time'] > 0 and
                (now - lv['time']) <= self.grace_period_s and
                lv['angle'] is not None
            ):
                publish(lv['angle'], lv['color'])
            else:
                publish(IMAGE_ANGLE_INVALID, "none")


    def process_image(self, cv_image, view_frame):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        img_h, img_w = cv_image.shape[:2]
        cx = img_w // 2

        # =============================
        # 지원 색상
        # =============================
        color_ranges = {
            "red":     [([0, 140, 80], [5, 255, 255]),
                       ([165, 140, 80], [180, 255, 255])],
            "green":   [([60, 120,120], [85, 255, 255])],
            "white":   [([5, 2, 230], [33, 30, 255]),
                        ([75, 7, 60], [105, 45, 140])]
        		}#부표용)


        # ★ 현재 WP에서 사용할 색만 선택
        if self.wp_mode in self.wp_color_map:
            target_colors = self.wp_color_map[self.wp_mode]
        else:
            target_colors = color_ranges.keys()  # Default: 모든 색

        # V1: 후보 선택 기준 '가장 가까운 것(depth)' → '가장 큰 것(면적)'
        best = {
            'angle': None,
            'area': -1.0,
            'vX': None,
            'vY': None,
            'vertices': None,
            'color': None
        }

        for color, ranges in color_ranges.items():
            if color not in target_colors:
                continue  # 📌 WP 외 색상 무시!

            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lo, hi in ranges:
                mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
            mask = cv2.medianBlur(mask, 3)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 40:
                    continue

                approx = cv2.approxPolyDP(cnt, 0.0315 * cv2.arcLength(cnt, True), True)
                v = len(approx)

                if not (4 <= v <= 8):
                    continue

                x, y, w, h = cv2.boundingRect(approx)
                aspect = w / float(h)
                extent = area / float(w * h)
                if not (extent > 0.4 and 0.6 <= aspect <= 1.4):
                    continue

                M = cv2.moments(cnt)
                if M["m00"] == 0:
                    continue
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                vertices = approx.reshape(-1, 2)
                vX = int(np.mean(vertices[:, 0]))
                vY = int(np.mean(vertices[:, 1]))

                if not (img_h * 0.10 <= vY <= img_h * 0.55):
                    continue

                # angle (depth 불필요 — 기존 식에서 distance 가 약분되어 값이 완전히 같다)
                rel_x = vX - cx
                angle_deg = -math.degrees(math.atan(rel_x * PIXEL_TO_ANGLE_K))

                if area > best['area']:
                    best.update({
                        'angle': angle_deg,
                        'area': area,
                        'vX': vX,
                        'vY': vY,
                        'vertices': vertices,
                        'color': color
                    })

                if self.debug_view:
                    cv2.drawContours(view_frame, [approx], -1, (0, 255, 0), 2)

        if best['angle'] is not None:
            self.found_in_frame = True
            now = time.time()

            self.last_valid.update({
                'angle': best['angle'],
                'color': best['color'],
                'time': now
            })

            self.angle_pub.publish(Float32(data=best['angle']))
            self.color_pub.publish(String(data=best['color']))

            if self.debug_view:
                cv2.circle(view_frame, (best['vX'], best['vY']), 7, (0, 255, 255), -1)
                cv2.putText(view_frame,
                            f"{best['color']}  {best['angle']:.1f}deg",
                            (best['vX']+10, best['vY']-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (255,255,255), 2)


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
