import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String, Int32
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from color_shape_detector.mode_gate import ModeGate
from color_shape_detector.vision_geom import (
    DEFAULT_HFOV_DEG, angle_from_pixel, solidity,
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
        # A3(2026-10-27): 예전엔 2.0 이 코드에 박혀 있었다(dock 만 파라미터). 이름은 dock 과 같다.
        self.grace_period_s = float(
            self.declare_parameter('grace_period_s', 2.0).value)
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
        # A1(2026-10-27): 창 M 중 N. 기본 = confirm_frames → 옛 "연속 N" 과 동일(동작 불변).
        #   깜빡임(검출-놓침-검출)에서 연속 N 이 영원히 안 되던 기아를 없애려 켤 땐 3/5 로.
        #   window < N 이면 DetectionConfirmer 가 N 으로 보정한다.
        #   🚨 창 안 다수결이 아니라 **이번 프레임 색**이 창 안에 N 번일 때만 그 색을 돌려준다
        #      — 그래야 각도와 색이 같은 프레임 것이라 ship_turn 이 반대로 돌지 않는다.
        cw = int(self.declare_parameter('confirm_window', cf).value)
        self.confirm = DetectionConfirmer(cf, confirm_window=cw)

        # ---- A3: 검출 기하 임계 파라미터화 (예전엔 아래 값이 process_image 에 리터럴로 박혀 있었다) ----
        #   이름은 dock(basic_image_subscriberdock) 과 같다. 기본값 = 옛 리터럴 그대로.
        #   ⚠️ y_zone_lo 는 gate(0.15)와 다르게 **0.10** 이다 — vision.yaml 의 `/**` 공통 절에
        #      넣으면 조용히 0.15 로 바뀐다. 노드 절(image_subscriber_turn)에만 둔다.
        #   y_zone: 카메라 틸트 0°(마스트 v4.4)라 수평선이 화면 중앙 → 낮은 부표는 가까워질수록
        #   0.55h 아래로 내려가 잘릴 수 있다. ⚠️ 실측 필요 — 거리별 부표 화면 높이로 확정.
        self.approx_epsilon = float(self.declare_parameter('approx_epsilon', 0.0315).value)
        self.y_zone_lo = float(self.declare_parameter('y_zone_lo', 0.10).value)
        self.y_zone_hi = float(self.declare_parameter('y_zone_hi', 0.55).value)
        self.shape_vertices_min = int(self.declare_parameter('shape_vertices_min', 4).value)
        self.shape_vertices_max = int(self.declare_parameter('shape_vertices_max', 8).value)
        self.square_extent_min = float(
            self.declare_parameter('square_extent_min', 0.4).value)
        self.square_aspect_lo = float(self.declare_parameter('square_aspect_lo', 0.6).value)
        self.square_aspect_hi = float(self.declare_parameter('square_aspect_hi', 1.4).value)

        # ---- A4: 부표에 사각형 검사를 강제할지 ----
        #   규정 부표는 형상 규정이 없다(원통·구). 파도에 반쯤 가리면 꼭짓점 3 또는 9+ 가 돼
        #   색은 맞는데 버린다. false 면 색+면적+solidity(면적/볼록껍질 ≥ min_solidity)로만 거른다.
        #   🚨 기본 true = 현재 동작 그대로. 야외에서 true/false 유실 횟수를 재본 뒤 바꾼다.
        self.require_shape = bool(self.declare_parameter('require_shape', True).value)
        self.min_solidity = float(self.declare_parameter('min_solidity', 0.7).value)

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
        # A2: monotonic (CLAUDE.md 1-5). last_valid['time'] 을 적는 process_image 와 같은 시계.
        now = time.monotonic()

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

                # polygon approx (epsilon 은 파라미터 — A3)
                approx = cv2.approxPolyDP(
                    cnt, self.approx_epsilon * cv2.arcLength(cnt, True), True)
                v = len(approx)

                if self.require_shape:
                    # 사각형 검사 (gate 와 동일) — 기본 경로, 동작 불변
                    if not (self.shape_vertices_min <= v <= self.shape_vertices_max):
                        continue
                    x, y, w, h = cv2.boundingRect(approx)
                    aspect = w / float(h)
                    extent = area / float(w * h)
                    if not (extent > self.square_extent_min and
                            self.square_aspect_lo <= aspect <= self.square_aspect_hi):
                        continue
                else:
                    # A4: 형상 무관 — 색+면적+solidity. 윤슬·물보라 조각(들쭉날쭉)을 거른다.
                    hull_area = cv2.contourArea(cv2.convexHull(cnt))
                    sol = solidity(area, hull_area)
                    if sol is None or sol < self.min_solidity:
                        continue

                # 🚨 moments 는 m00==0 인 퇴화 컨투어 **가드**다 — 건드리지 마라.
                #    require_shape=true(기본) 경로의 각도는 예전처럼 꼭짓점 평균(vX)에서 나온다.
                #    A4(2026-10-27): require_shape=false 경로만 질량중심(cX,cY)을 쓴다 —
                #    울퉁불퉁한 컨투어는 꼭짓점 평균이 튄다.
                M = cv2.moments(cnt)
                if M["m00"] == 0:
                    continue

                vertices = approx.reshape(-1, 2)
                if self.require_shape:
                    vX = int(np.mean(vertices[:, 0]))
                    vY = int(np.mean(vertices[:, 1]))
                else:
                    vX = int(M["m10"] / M["m00"])
                    vY = int(M["m01"] / M["m00"])

                if not (img_h * self.y_zone_lo <= vY <= img_h * self.y_zone_hi):
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
            now = time.monotonic()   # A2: fallback 의 now 와 같은 시계

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
                            (255, 255, 255), 2)


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
