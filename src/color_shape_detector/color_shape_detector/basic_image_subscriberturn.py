import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String, Int32
from cv_bridge import CvBridge

import cv2
import numpy as np
import math
import time

IMAGE_ANGLE_INVALID = 10000.0


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
            10)

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10)

        # ★ 추가: WP 모드 구독
        self.wp_mode_sub = self.create_subscription(
            Int32,
            '/wp_mode',
            self.wp_mode_callback,
            10)

        # =============================
        # PUBLISHERS
        # =============================
        self.angle_pub = self.create_publisher(Float32, '/image_angle', 10)
        self.distance_pub = self.create_publisher(Float32, '/image_distance', 10)
        self.color_pub = self.create_publisher(String, '/image_color', 10)

        # =============================
        # STATE / FALLBACK
        # =============================
        self.latest_depth = None
        self.last_valid = {
            'angle': None,
            'distance': None,
            'color': None,
            'time': 0.0
        }
        self.grace_period_s = 2.0
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


    def depth_callback(self, msg):
        self.latest_depth = self.br.imgmsg_to_cv2(msg, desired_encoding='passthrough')


    def color_callback(self, msg):
        if self.latest_depth is None:
            return

        self.found_in_frame = False

        frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        view = frame.copy()

        self.process_image(frame, view)

        cv2.imshow("TURN Detection", view)
        cv2.waitKey(1)

        # ========== Fallback ==========
        now = time.time()

        def publish(angle, dist, color):
            self.angle_pub.publish(Float32(data=float(angle)))
            self.distance_pub.publish(Float32(data=float(dist)))
            self.color_pub.publish(String(data=str(color)))

        if not self.found_in_frame:
            lv = self.last_valid
            if (
                lv['time'] > 0 and
                (now - lv['time']) <= self.grace_period_s and
                lv['angle'] is not None
            ):
                publish(lv['angle'], lv['distance'], lv['color'])
            else:
                publish(IMAGE_ANGLE_INVALID, IMAGE_ANGLE_INVALID, "none")


    def process_image(self, cv_image, view_frame):
        depth_img = self.latest_depth
        if depth_img is None:
            return

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

        best = {
            'angle': None,
            'distance': None,
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

                distance = depth_img[vY, vX] * 0.001
                if distance <= 0:
                    continue

                rel_x = vX - cx
                real_x = (rel_x / 80.0) * 0.09 * (distance / 0.5)
                angle_deg = -math.degrees(math.atan2(real_x, distance))

                if best['distance'] is None or distance < best['distance']:
                    best.update({
                        'angle': angle_deg,
                        'distance': distance,
                        'vX': vX,
                        'vY': vY,
                        'vertices': vertices,
                        'color': color
                    })

                cv2.drawContours(view_frame, [approx], -1, (0, 255, 0), 2)

        if best['distance'] is not None:
            self.found_in_frame = True
            now = time.time()

            self.last_valid.update({
                'angle': best['angle'],
                'distance': best['distance'],
                'color': best['color'],
                'time': now
            })

            self.angle_pub.publish(Float32(data=best['angle']))
            self.distance_pub.publish(Float32(data=best['distance']))
            self.color_pub.publish(String(data=best['color']))

            cv2.circle(view_frame, (best['vX'], best['vY']), 7, (0, 255, 255), -1)
            cv2.putText(view_frame,
                        f"{best['color']}  {best['distance']:.2f}m",
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
