#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class HSVSubscriber(Node):
    def __init__(self):
        super().__init__('hsv_debug_subscriber')

        self.br = CvBridge()

        # 🎯 RealSense 컬러 영상 구독
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',  # ← 실제 토픽으로 수정!
            self.image_callback,
            10)

        # ✨ 디버그 창
        cv2.namedWindow("HSV Debug", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("HSV Debug", self.mouse_callback)

        self.mouse_x, self.mouse_y = -1, -1

        print("\n🚀 HSV Debug Subscriber STARTED")
        print("👉 Move mouse over the frame to print HSV\n")

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_MOUSEMOVE:
            self.mouse_x, self.mouse_y = x, y

    def image_callback(self, msg):
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
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
