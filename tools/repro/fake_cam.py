#!/usr/bin/env python3
"""카메라 없이 비전 검출기를 시험하는 가짜 컬러 영상 발행기.

    python3 tools/repro/fake_cam.py            # 회색 화면 (부표 없음)
    python3 tools/repro/fake_cam.py --buoy     # 빨간 원(부표) 하나, 오른쪽 위 1/3

🚨 160×120 으로 낸다. 640×480 은 이 노트북 DDS 기본 설정에서 전달이 안 됐다(2026-10-28).
   검출기는 해상도를 매 프레임 msg.width 로 읽으므로 각도 계산은 그대로 맞다.
"""
import sys
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

W, H = 160, 120
img = np.full((H, W, 3), 90, np.uint8)
if '--buoy' in sys.argv:
    cv2.circle(img, (110, 40), 14, (0, 0, 230), -1)   # BGR 빨강, y_zone 0.10~0.55 안


class FakeCam(Node):
    def __init__(self):
        super().__init__('fake_cam')
        self.pub = self.create_publisher(Image, '/camera/camera/color/image_raw', 10)
        self.create_timer(1 / 15, self.tick)

    def tick(self):
        m = Image()
        m.height, m.width, m.encoding, m.step = H, W, 'bgr8', W * 3
        m.data = img.tobytes()
        m.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(m)


rclpy.init()
rclpy.spin(FakeCam())
