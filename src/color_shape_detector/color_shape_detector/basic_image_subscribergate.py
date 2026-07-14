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
        super().__init__('image_subscriber_gate')

        self.br = CvBridge()

        # -----------------------------
        # COLOR / DEPTH SUBSCRIBERS
        # -----------------------------
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

        # -----------------------------
        # PUBLISHERS FOR RED / YELLOW
        # -----------------------------
        self.red_angle_pub = self.create_publisher(Float32, '/red_angle', 10)
        self.red_distance_pub = self.create_publisher(Float32, '/red_distance', 10)

        self.green_angle_pub = self.create_publisher(Float32, '/green_angle', 10)
        self.green_distance_pub = self.create_publisher(Float32, '/green_distance', 10)

        # Depth buffer
        self.latest_depth = None

        # fallback 저장
        self.last_valid = {
            'red':    {'angle': None, 'distance': None, 'time': 0.0},
            'green': {'angle': None, 'distance': None, 'time': 0.0},
        }
        self.grace_period_s = 2.0

        # 내부 상태
        self.last_log_time = time.time()
        self.found_in_frame = {'red': False, 'green': False}


    # -----------------------------------
    # Depth Callback
    # -----------------------------------
    def depth_callback(self, msg):
        self.latest_depth = self.br.imgmsg_to_cv2(msg, desired_encoding='passthrough')


    # -----------------------------------
    # Color Callback
    # -----------------------------------
    def color_callback(self, msg):

        if self.latest_depth is None:
            return

        self.found_in_frame = {'red': False, 'green': False}

        frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        view = frame.copy()

        self.process_image(frame, view)

        # ---- Show window ----
        cv2.imshow("Detected Shapes", view)
        cv2.waitKey(1)

        # ---- Fallback ----
        now = time.time()

        def publish_fallback(color, angle_pub, dist_pub):
            lv = self.last_valid[color]

            if (
                lv['time'] > 0 and
                (now - lv['time']) <= self.grace_period_s and
                lv['angle'] is not None
            ):
                angle_pub.publish(Float32(data=lv['angle']))
                dist_pub.publish(Float32(data=lv['distance']))
            else:
                angle_pub.publish(Float32(data=IMAGE_ANGLE_INVALID))
                dist_pub.publish(Float32(data=IMAGE_ANGLE_INVALID))

        # red fallback
        if not self.found_in_frame['red']:
            publish_fallback('red', self.red_angle_pub, self.red_distance_pub)

        # yellow fallback
        if not self.found_in_frame['green']:
            publish_fallback('green', self.green_angle_pub, self.green_distance_pub)



    # -----------------------------------
    # PROCESS IMAGE
    # -----------------------------------
    def process_image(self, cv_image, view_frame):

        depth_img = self.latest_depth
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        img_h, img_w = cv_image.shape[:2]
        cx = img_w // 2

        # ----------------------------
        # HSV ranges (NEVER remove lines)
        # ----------------------------
        color_ranges = {
            "red":     [([0, 140, 80], [5, 255, 255]),
                       ([165, 140, 80], [180, 255, 255])],
            "green":   [([60, 120,120], [85, 255, 255])]
            #"white":   [([5, 2, 230], [33, 30, 255]),
                        #([75, 7, 60], [105, 45, 140])]
        		}#부표용)
        # 색상별 best 후보
        best = {
            'red':    {'angle': None, 'distance': None, 'vX': None, 'vY': None, 'vertices': None},
            'green': {'angle': None, 'distance': None, 'vX': None, 'vY': None, 'vertices': None}
        }

        for color, ranges in color_ranges.items():

            # Mask 생성
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lo, hi in ranges:
                mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))

            mask = cv2.medianBlur(mask, 3)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:

                area = cv2.contourArea(cnt)
                if area < 40:
                    continue

                # polygon approx
                approx = cv2.approxPolyDP(cnt, 0.0315 * cv2.arcLength(cnt, True), True)
                v = len(approx)

                # ---------------------------
                # ★ ONLY SQUARES (TURN과 동일)
                # ---------------------------
                if 4 <= v <= 8:
                    x, y, w, h = cv2.boundingRect(approx)
                    aspect = w / float(h)
                    extent = area / float(w * h)

                    if not (extent > 0.4 and 0.6 <= aspect <= 1.4):
                        continue
                    shape = "Square"
                else:
                    continue

                # 중심점 계산
                M = cv2.moments(cnt)
                if M["m00"] == 0:
                    continue
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                vertices = approx.reshape(-1, 2)
                vX = int(np.mean(vertices[:, 0]))
                vY = int(np.mean(vertices[:, 1]))

                # y-coordinate filtering: valid detection zone only
                if not (img_h * 0.15 <= vY <= img_h * 0.55):  #위쪽범위 <= vY <=아래쪽범위
                    continue



                # ---- depth ----
                distance = depth_img[vY, vX] * 0.001

                #depth 연산 유효거리 제한
                if not (1.0 <= distance <= 6.0):
                    continue

                # ---- angle ----
                rel_x = vX - cx
                real_x = (rel_x / 80.0) * 0.09 * (distance / 0.5)
                angle_deg = -math.degrees(math.atan2(real_x, distance))

                # 가장 가까운 것만 저장
                prev = best[color]
                if prev['distance'] is None or distance < prev['distance']:
                    best[color] = {
                        'angle': angle_deg,
                        'distance': distance,
                        'vX': vX,
                        'vY': vY,
                        'vertices': vertices
                    }

                # contour 그리기
                cv2.drawContours(view_frame, [approx], -1, (0, 255, 0), 2)
                cv2.circle(view_frame, (cX, cY), 3, (255, 255, 0), -1)



        # ----------------------------
        # 색상별 퍼블리시
        # ----------------------------
        now = time.time()

        for c in ['red', 'green']:
            cand = best[c]
            if cand['distance'] is None:
                continue

            self.found_in_frame[c] = True

            self.last_valid[c]['angle'] = cand['angle']
            self.last_valid[c]['distance'] = cand['distance']
            self.last_valid[c]['time'] = now

            # publish
            msg_a = Float32(); msg_a.data = cand['angle']
            msg_d = Float32(); msg_d.data = cand['distance']

            if c == 'red':
                self.red_angle_pub.publish(msg_a)
                self.red_distance_pub.publish(msg_d)
            else:
                self.green_angle_pub.publish(msg_a)
                self.green_distance_pub.publish(msg_d)

            # draw best
            cv2.circle(view_frame, (cand['vX'], cand['vY']), 5, (0, 255, 255), -1)
            cv2.putText(view_frame, f"{c} Square  {cand['distance']:.2f}m",
                        (cand['vX'] + 10, cand['vY'] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # debug vertices
            for pt in cand['vertices']:
                cv2.circle(view_frame, tuple(pt), 4, (0, 0, 255), -1)


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()
