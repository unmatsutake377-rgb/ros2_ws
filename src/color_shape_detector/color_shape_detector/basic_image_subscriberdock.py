import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import time
from rclpy.executors import MultiThreadedExecutor

IMAGE_ANGLE_INVALID = 10000.0

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber_dock')

        # Color 이미지 구독 (turn과 동일)
        self.subscription_color = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.color_callback,
            10)

        # Depth 이미지 구독 (turn과 동일)
        self.subscription_depth = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10)

        self.br = CvBridge()
        self.latest_depth = None

        # 퍼블리셔: angle, distance
        self.angle_pub    = self.create_publisher(Float32, '/image_angle', 10)
        self.distance_pub = self.create_publisher(Float32, '/image_distance', 10)

        # fallback
        self.last_valid = {
            'angle': None,
            'distance': None,
            'time': 0.0
        }
        self.grace_period_s = 2.0

        self.found_in_frame = False
        self.last_log_time = time.time()

        # 목표 도형
        self.target_color = "red"
        self.target_shape = "Square"


    # ============================================
    # Depth topic 콜백
    # ============================================
    def depth_callback(self, msg):
        self.latest_depth = self.br.imgmsg_to_cv2(msg, desired_encoding='passthrough')


    # ============================================
    # Color topic 콜백
    # ============================================
    def color_callback(self, msg):

        self.found_in_frame = False

        frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        view_frame = frame.copy()

        self.process_image(frame, view_frame)

        # 화면 표시
        cv2.imshow("Dock Detection", view_frame)
        cv2.waitKey(1)

        # fallback 처리
        now = time.time()

        def publish(angle_val, dist_val):
            self.angle_pub.publish(Float32(data=float(angle_val)))
            self.distance_pub.publish(Float32(data=float(dist_val)))

        if not self.found_in_frame:
            if (
                self.last_valid['time'] > 0 and
                (now - self.last_valid['time']) <= self.grace_period_s and
                self.last_valid['angle'] is not None
            ):
                publish(self.last_valid['angle'], self.last_valid['distance'])
            else:
                publish(IMAGE_ANGLE_INVALID, IMAGE_ANGLE_INVALID)


    # ============================================
    # 이미지 처리 (도형 인식)
    # ============================================
    def process_image(self, cv_image, view_frame):

        if self.latest_depth is None:
            return

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        img_h, img_w = cv_image.shape[:2]
        cx = img_w // 2
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

                # Depth distance
                distance = self.latest_depth[vY, vX] * 0.001
                if distance <= 0:
                    continue

                # angle 계산 (turn과 완전 동일)
                rel_x = vX - cx
                real_x_offset = (rel_x / 80.0) * 0.09 * (distance / 0.5)
                angle_rad = math.atan2(real_x_offset, distance)
                angle_deg = -math.degrees(angle_rad)

                # 퍼블리시 갱신
                self.angle_pub.publish(Float32(data=float(angle_deg)))
                self.distance_pub.publish(Float32(data=float(distance)))

                self.found_in_frame = True
                self.last_valid['angle'] = float(angle_deg)
                self.last_valid['distance'] = float(distance)
                self.last_valid['time'] = time.time()

                # 시각화
                cv2.drawContours(view_frame, [approx], -1, (0, 255, 0), 2)
                cv2.circle(view_frame, (vX, vY), 4, (255, 255, 0), -1)
                cv2.putText(view_frame,
                            f"{color} {shape} {distance:.2f}m {angle_deg:.1f}deg",
                            (vX + 10, vY - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)

                if time.time() - self.last_log_time > 0.5:
                    print(f"[Dock] {color} {shape}: d={distance:.2f}m  angle={angle_deg:.1f}")
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
