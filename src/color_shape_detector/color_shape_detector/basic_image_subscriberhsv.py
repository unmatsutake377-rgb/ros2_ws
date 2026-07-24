#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import numpy as np

# V4(T2-6): 표준 sensor-data QoS (작년 depth=10 RELIABLE).
#   구독자 BEST_EFFORT 는 발행자가 RELIABLE 이어도 호환된다(그 반대가 비호환).
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class HSVSubscriber(Node):
    def __init__(self):
        super().__init__('hsv_debug_subscriber')

        # V5(T2-2): 이미지 토픽 파라미터화. 기본값은 현 RealSense 토픽 그대로다.
        #   OAK 로 바꾸는 날 코드를 고치지 않고 **yaml 한 줄**로 끝내려는 것이다.
        #   ⚠️ 구독보다 반드시 먼저 선언해야 한다 — 뒤에 두면 부팅 즉시 AttributeError 다.
        #   ⚠️ 이 노드는 subscribermode 가 `ros2 run` 으로 띄운다 → launch 파라미터가 안 닿는다.
        #      subscribermode 가 --ros-args 로 넘겨준다(vision_image_topic).
        self.image_topic = str(self.declare_parameter(
            'image_topic', '/camera/camera/color/image_raw').value)

        self.br = CvBridge()

        # 🎯 RealSense 컬러 영상 구독
        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            SENSOR_QOS)

        # V4(T2-5): 이 노드만 debug_view 기본 true 다 — HSV 값을 눈과 마우스로 읽는 게 존재 이유라
        #   창이 없으면 할 일이 없다. 그래도 게이트는 필요하다: namedWindow 가 __init__ 에 있어
        #   헤드리스에서 실행하면 '노드 생성 시점' 에 터진다(콜백 전에 죽어 원인 파악이 더 어렵다).
        #   ⚠️ 이 노드는 튜닝 전용이다. 대회 launch 에 넣지 마라.
        self.debug_view = bool(self.declare_parameter('debug_view', True).value)

        if self.debug_view:
            cv2.namedWindow("HSV Debug", cv2.WINDOW_NORMAL)
            cv2.setMouseCallback("HSV Debug", self.mouse_callback)
        else:
            self.get_logger().warn(
                "debug_view=false 로 HSV 튜닝 노드를 띄웠다. 창이 없으면 이 노드는 할 일이 없다.")

        self.mouse_x, self.mouse_y = -1, -1

        print("\n🚀 HSV Debug Subscriber STARTED")
        print("👉 Move mouse over the frame to print HSV\n")

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_MOUSEMOVE:
            self.mouse_x, self.mouse_y = x, y

    def image_callback(self, msg):
        if not self.debug_view:
            return

        frame = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 창 크기 강제 설정 (HD급)
        cv2.resizeWindow("HSV Debug", 800, 600)

        if (0 <= self.mouse_x < frame.shape[1]) and \
           (0 <= self.mouse_y < frame.shape[0]):

            h, s, v = hsv[self.mouse_y, self.mouse_x]
            text = f"HSV({h},{s},{v})"
            print(text)

            cv2.circle(frame, (self.mouse_x, self.mouse_y), 6, (0,255,255), -1)
            cv2.putText(frame, text,
                        (self.mouse_x+10, self.mouse_y+10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0,255,255), 2)

        cv2.imshow("HSV Debug", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = HSVSubscriber()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if node.debug_view:
            cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
