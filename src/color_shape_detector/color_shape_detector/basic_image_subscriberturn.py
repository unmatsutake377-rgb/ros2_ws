import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String, Int32
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from color_shape_detector.mode_gate import ModeGate
from color_shape_detector.vision_geom import (
    DEFAULT_HFOV_DEG, angle_from_pixel,
)
from color_shape_detector import hsv_ranges
from color_shape_detector.dock_logic import DetectionConfirmer

import cv2
import numpy as np
import time

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
            self.declare_parameter('active_wp_modes', [2, 3]).value,
            stale_sec=float(self.declare_parameter('wp_mode_stale_sec', 2.0).value))
        self._last_gate_log = 0.0

        self.br = CvBridge()

        # =============================
        # SUBSCRIBERS
        # =============================
        self.color_sub = self.create_subscription(
            Image,
            self.image_topic,
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

        # V3(T2-4): 카메라 수평 화각. 기본값은 현 RealSense 를 역산한 값이다.
        #   OAK-1 W(광각)로 바꾸면 **이 값만 yaml 에서 고치면 된다.**
        #   하드코딩이었을 땐 카메라 교체일에 모든 각도 출력과 상위 튜닝
        #   (align_tol_deg, pair_min/max_sep_deg …)이 통째로 무효가 됐다.
        #   ⚠️ 광각은 핀홀 모델이 가장자리에서 깨진다 — rectified 토픽 구독 또는
        #      camera_info 의 왜곡계수 D 적용이 추가로 필요하다.
        self.hfov_deg = float(
            self.declare_parameter('hfov_deg', DEFAULT_HFOV_DEG).value)

        # HSV 색 범위 — config/vision.yaml 의 hsv.* 가 단일 출처(hsv_ranges.py).
        # 잘못된 값이면 기본값으로 되돌리고 ERROR 로 알린다(조용히 못 찾는 것 방지).
        self.color_ranges = hsv_ranges.load(
            lambda n, d: self.declare_parameter(n, d).value,
            ("red", "green", "white"),
            on_error=self.get_logger().error)

        # 최소 면적[px²] — 예전엔 `if area < 40:` 로 코드에 박혀 있었다(gate 와 각각).
        # ⚠️ 40 은 근거 없는 값이다. 실측에서 41px² 노이즈가 통과했다. 야외에서 확정할 것.
        self.min_area_px = float(
            self.declare_parameter('min_area_px', 40.0).value)

        # 확인-N프레임. 기본 1 = 꺼짐 — 측정 없이 동작을 바꾸지 않는다.
        # 🚨 여기 key 는 **색**이다. 색이 바뀌면(빨강→흰색) 카운트가 리셋된다 —
        #    ship_turn 이 색으로 회전 방향을 정하므로(빨강·초록=시계, 흰색=반시계)
        #    색이 흔들리는 채로 발행하면 **배가 반대로 돈다.**
        cf = int(self.declare_parameter('confirm_frames', 1).value)
        self.confirm = DetectionConfirmer(cf)

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
        self.mode_gate.update(msg.data, time.monotonic())
        self.wp_mode = msg.data


    def color_callback(self, msg):
        # V1(T2-3): depth 가드 제거 — 뎁스 없는 카메라에서 콜백이 영원히 막히는 것 방지
        # 3-4: 내 차례가 아니면 **여기서 끝낸다.**
        #   cv_bridge 변환·HSV·컨투어를 전부 건너뛴다 → 비활성 노드의 CPU 는 거의 0.
        #   발행도 안 한다 → /image_angle 발행자 충돌이 원천 차단된다.
        now_m = time.monotonic()
        active, why = self.mode_gate.state(now_m)
        if not active:
            # 조용히 멈추지 않는다. '내 차례가 아님' 은 정상이라 로그를 안 내지만,
            # /wp_mode 자체가 없거나 끊긴 건 FSM 문제라 알린다.
            if why != ModeGate.R_OTHER_MODE and (now_m - self._last_gate_log) > 5.0:
                self._last_gate_log = now_m
                self.get_logger().warn(
                    f"비활성({why}) → 검출 정지. /wp_mode 가 오고 있나?")
            return

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

        # =============================
        # 지원 색상
        # =============================
        # HSV 범위는 **여기 없다** — config/vision.yaml 의 hsv.* 가 단일 출처다.
        # 🚨 white 는 예전에 색상대 2개(5~33, 75~105)였는데, 그건 특정 조명 색조에
        #    맞춘 흔적으로 보여 '색상 무관 + 저채도 + 고명도' 정의로 통일했다.
        #    옛 값은 hsv_ranges.SUPERSEDED["white@turn"] 에 남겨 뒀다 — 실외에서 비교할 것.
        color_ranges = self.color_ranges


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
                if area < self.min_area_px:
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
                angle_deg = angle_from_pixel(vX, img_w, self.hfov_deg)

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

        if best['angle'] is None:
            self.confirm.update(None)
        elif self.confirm.update(best['color']) is None:
            # 확정 전 — found_in_frame 을 세우지 않아 fallback 이 이어받는다
            pass
        else:
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
