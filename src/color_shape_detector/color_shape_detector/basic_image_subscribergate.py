import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
import numpy as np
import math
import time
from rclpy.executors import MultiThreadedExecutor

IMAGE_ANGLE_INVALID = 10000.0

# ⚠️ 임시 상수 — V3(T2-4)에서 hfov_deg 파라미터 + msg.width 기반으로 재작성한다.
# 기존 코드는 `real_x=(rel_x/80.0)*0.09*(distance/0.5); angle=atan2(real_x, distance)` 였는데,
# distance 가 분자·분모에서 **약분**되어 실질 `angle = atan(rel_x * K)` 와 완전히 같다.
#   K = (1/80)*0.09/0.5 = 0.00225  →  등가 fx = 1/K ≈ 444.4px  →  640px 기준 HFOV ≈ 71.5°
# 즉 RealSense 화각이 매직넘버에 박혀 있었다. 여기서는 **동작을 바꾸지 않고** depth 의존만 끊는다.
PIXEL_TO_ANGLE_K = (1.0 / 80.0) * 0.09 / 0.5     # = 0.00225

# V4(T2-6): 표준 sensor-data QoS. 작년은 depth=10 (기본 RELIABLE) 이었다.
#   콜백이 밀리면 묵은 프레임 10 장이 큐에 쌓이고, 배는 '몇 백 ms 전 장면' 을 보고 조향한다.
#   depth=1 이면 항상 최신 프레임만 본다 — 늦은 프레임은 버리는 게 맞다.
#   ⚠️ 구독자 BEST_EFFORT 는 발행자가 RELIABLE 이어도 호환된다(그 반대가 비호환).
#      RealSense·OAK 어느 쪽이든 안전하다.
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber_gate')

        self.br = CvBridge()

        # -----------------------------
        # COLOR SUBSCRIBER
        # -----------------------------
        # V1(T2-3): depth 구독 제거. OAK-1 은 뎁스가 없어 depth 를 기다리면 color 콜백이
        # 영원히 침묵하고 /red_angle·/green_angle 이 안 나간다(에러도 안 남는 침묵 사망).
        # 거리는 소비자(ship_gate)가 LiDAR 로 구한다 — 카메라는 방위각만.
        self.color_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.color_callback,
            SENSOR_QOS)

        # -----------------------------
        # PUBLISHERS (각도만 — 이름·타입 불변)
        # -----------------------------
        # V1(T2-3): /red_distance·/green_distance 발행 제거.
        #   근거: 소비자 0개 — 6단계에서 ship_gate 가 카메라 거리 구독을 버리고 /scan(LiDAR)
        #   방위 매칭으로 전환했다(부표=점 물체). 검증일 커밋 b91c45a 기준 전수 확인.
        self.red_angle_pub = self.create_publisher(Float32, '/red_angle', 10)
        self.green_angle_pub = self.create_publisher(Float32, '/green_angle', 10)

        # fallback 저장
        self.last_valid = {
            'red':    {'angle': None, 'time': 0.0},
            'green': {'angle': None, 'time': 0.0},
        }
        self.grace_period_s = 2.0

        # V4(T2-5): 헤드리스(디스플레이 없음)에서 cv2.imshow 는 예외를 던져 노드를 죽인다.
        #   배는 SSH 로 띄운다 — 기본 false 가 맞다. try/except 로 덮지 않고 파라미터로 원천 차단한다.
        #   false 면 그리기 연산과 frame.copy() 자체를 건너뛴다(매 프레임 전체 memcpy).
        self.debug_view = bool(self.declare_parameter('debug_view', False).value)

        # 내부 상태
        self.last_log_time = time.time()
        self.found_in_frame = {'red': False, 'green': False}


    # -----------------------------------
    # Color Callback
    # -----------------------------------
    def color_callback(self, msg):
        # V1(T2-3): `if self.latest_depth is None: return` 가드 제거.
        #   OAK 처럼 뎁스가 없는 카메라에선 이 한 줄이 콜백을 영원히 막아
        #   /red_angle·/green_angle 이 조용히 죽는다(에러 없음 = 발견이 늦다).
        self.found_in_frame = {'red': False, 'green': False}

        frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # V4: 디버그 창이 꺼져 있으면 복사본을 만들지 않는다. 그리기도 전부 건너뛴다.
        view = frame.copy() if self.debug_view else None

        self.process_image(frame, view)

        if self.debug_view:
            cv2.imshow("Detected Shapes", view)
            cv2.waitKey(1)

        # ---- Fallback ----
        now = time.time()

        def publish_fallback(color, angle_pub):
            lv = self.last_valid[color]

            if (
                lv['time'] > 0 and
                (now - lv['time']) <= self.grace_period_s and
                lv['angle'] is not None
            ):
                angle_pub.publish(Float32(data=lv['angle']))
            else:
                angle_pub.publish(Float32(data=IMAGE_ANGLE_INVALID))

        # red fallback
        if not self.found_in_frame['red']:
            publish_fallback('red', self.red_angle_pub)

        # green fallback
        if not self.found_in_frame['green']:
            publish_fallback('green', self.green_angle_pub)



    # -----------------------------------
    # PROCESS IMAGE
    # -----------------------------------
    def process_image(self, cv_image, view_frame):

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
        # V1: 'distance' → 'area'. 후보 선택 기준을 '가장 가까운 것(depth)' 에서
        #     '가장 큰 것(면적)' 으로 바꾼다 — 같은 크기 표식이면 큰 쪽이 가까운 쪽이다.
        best = {
            'red':    {'angle': None, 'area': -1.0, 'vX': None, 'vY': None, 'vertices': None},
            'green': {'angle': None, 'area': -1.0, 'vX': None, 'vY': None, 'vertices': None}
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



                # ---- angle (depth 불필요) ----
                # V1: 기존 식에서 distance 가 약분되므로 아래가 **완전히 같은 값**이다.
                #     (파일 상단 PIXEL_TO_ANGLE_K 주석 참고. V3 에서 hfov 파라미터화)
                # ※ depth 유효거리 필터(1.0~6.0m)도 함께 제거했다 — depth 가 없으면
                #   distance 가 미정의라 그대로 두면 즉사하고, 값을 대체하면 필터 의미가 사라진다.
                #   후보 검증은 색·형상·면적·y범위 조건으로 일원화한다.
                rel_x = vX - cx
                angle_deg = -math.degrees(math.atan(rel_x * PIXEL_TO_ANGLE_K))

                # 가장 '큰' 것만 저장 (면적 = 거리의 대용)
                prev = best[color]
                if area > prev['area']:
                    best[color] = {
                        'angle': angle_deg,
                        'area': area,
                        'vX': vX,
                        'vY': vY,
                        'vertices': vertices
                    }

                # contour 그리기 (디버그 창이 켜져 있을 때만)
                if self.debug_view:
                    cv2.drawContours(view_frame, [approx], -1, (0, 255, 0), 2)
                    cv2.circle(view_frame, (cX, cY), 3, (255, 255, 0), -1)



        # ----------------------------
        # 색상별 퍼블리시
        # ----------------------------
        now = time.time()

        for c in ['red', 'green']:
            cand = best[c]
            if cand['angle'] is None:      # V1: distance 센티널 → angle 센티널
                continue

            self.found_in_frame[c] = True

            self.last_valid[c]['angle'] = cand['angle']
            self.last_valid[c]['time'] = now

            # publish (각도만 — 거리는 소비자가 LiDAR 로 구한다)
            msg_a = Float32(); msg_a.data = cand['angle']

            if c == 'red':
                self.red_angle_pub.publish(msg_a)
            else:
                self.green_angle_pub.publish(msg_a)

            # draw best (디버그 창이 켜져 있을 때만)
            if self.debug_view:
                cv2.circle(view_frame, (cand['vX'], cand['vY']), 5, (0, 255, 255), -1)
                cv2.putText(view_frame, f"{c} Square  {cand['angle']:.1f}deg",
                            (cand['vX'] + 10, cand['vY'] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

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
