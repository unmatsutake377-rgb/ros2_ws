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
from color_shape_detector.dock_logic import DetectionConfirmer

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
            self.declare_parameter('active_wp_modes', [0, 1]).value,
            stale_sec=float(self.declare_parameter('wp_mode_stale_sec', 2.0).value))
        self._last_gate_log = 0.0

        # 3-4: 모드 게이팅용. 상주하면서 자기 차례일 때만 일한다.
        self.create_subscription(
            Int32, '/wp_mode', self.wp_mode_callback, 10)

        self.br = CvBridge()

        # -----------------------------
        # COLOR SUBSCRIBER
        # -----------------------------
        # V1(T2-3): depth 구독 제거. OAK-1 은 뎁스가 없어 depth 를 기다리면 color 콜백이
        # 영원히 침묵하고 /red_angle·/green_angle 이 안 나간다(에러도 안 남는 침묵 사망).
        # 거리는 소비자(ship_gate)가 LiDAR 로 구한다 — 카메라는 방위각만.
        self.color_sub = self.create_subscription(
            Image,
            self.image_topic,
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
            ("red", "green"),
            on_error=self.get_logger().error)

        # 최소 면적[px²] — 이보다 작은 덩어리는 무시한다.
        # 🚨 예전엔 `if area < 40:` 로 **코드에 박혀** 있었다(gate·turn 각각).
        #    야외에서 값을 바꾸려면 파이썬을 고쳐야 했다 — CLAUDE.md 1-4 위반.
        #    ⚠️ 40 은 근거가 없는 값이다. 실측에서 **41px² 짜리 노이즈가 부표로 통과**했다.
        #       그렇다고 올리면 먼 부표를 놓친다(20cm 부표는 8m 에서 70px²).
        #       부표 실제 지름을 모르면 못 정한다 → 야외에서 확정.
        self.min_area_px = float(
            self.declare_parameter('min_area_px', 40.0).value)

        # 확인-N프레임 — 연속 N프레임 잡혀야 발행한다(CLAUDE.md 6-2 [F]).
        # 🚨 기본값 1 = **꺼짐**. 오늘 실측(빨강 1220/1220, 초록 520/520)은 전부
        #    확인 없이 나온 결과다. 측정 없이 동작을 바꾸지 않는다
        #    (시간투표 필터를 근거 없이 켰다가 무익으로 판명난 전례가 있다 — CLAUDE.md 6장).
        #    ⚠️ dock 은 이미 3 을 쓴다. 야외에서 물보라·윤슬 노이즈를 재본 뒤
        #       여기도 3 으로 올릴지 정한다. yaml 한 줄이면 된다.
        cf = int(self.declare_parameter('confirm_frames', 1).value)
        self.confirm = {c: DetectionConfirmer(cf) for c in ('red', 'green')}

        # 내부 상태
        self.last_log_time = time.time()
        self.found_in_frame = {'red': False, 'green': False}


    # -----------------------------------
    # Color Callback
    # -----------------------------------
    def wp_mode_callback(self, msg):
        self.mode_gate.update(msg.data, time.monotonic())

    def color_callback(self, msg):
        # V1(T2-3): `if self.latest_depth is None: return` 가드 제거.
        #   OAK 처럼 뎁스가 없는 카메라에선 이 한 줄이 콜백을 영원히 막아
        #   /red_angle·/green_angle 이 조용히 죽는다(에러 없음 = 발견이 늦다).
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

        # ----------------------------
        # HSV 범위는 **여기 없다** — config/vision.yaml 의 hsv.* 파라미터가 단일 출처다.
        # (2026-08-12: 같은 색이 검출기 3개에 서로 다른 값으로 박혀 있었다. hsv_ranges.py 주석 참고)
        color_ranges = self.color_ranges
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
                if area < self.min_area_px:
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
                # ※ depth 유효거리 필터(1.0~6.0m)도 함께 제거했다 — depth 가 없으면
                #   distance 가 미정의라 그대로 두면 즉사하고, 값을 대체하면 필터 의미가 사라진다.
                #   후보 검증은 색·형상·면적·y범위 조건으로 일원화한다.
                angle_deg = angle_from_pixel(vX, img_w, self.hfov_deg)

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
                self.confirm[c].update(None)
                continue

            # 확정 전이면 발행하지 않는다. found_in_frame 을 세우지 **않으므로**
            # 아래 fallback 이 이어받아 grace 기간 동안 마지막 유효값을 낸다
            # → 토픽이 침묵하지 않는다(침묵은 소비자에게 '사라졌다' 로 읽힌다).
            if self.confirm[c].update(c) is None:
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
