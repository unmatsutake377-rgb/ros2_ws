#!/usr/bin/env python3
"""/image_angle 을 N 초 받아 값별 건수를 찍는다.   python3 tools/repro/count_angle.py 6 "라벨" """
import sys
import time
import collections
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

rclpy.init()
n = Node('count_angle')
c = collections.Counter()
n.create_subscription(Float32, '/image_angle', lambda m: c.update([round(m.data, 1)]), 10)
t0 = time.monotonic()
while time.monotonic() - t0 < float(sys.argv[1]):
    rclpy.spin_once(n, timeout_sec=0.1)
print(f"{sys.argv[2] if len(sys.argv) > 2 else ''} | /image_angle {sum(c.values())}건 · 값 분포 {dict(c)}")
