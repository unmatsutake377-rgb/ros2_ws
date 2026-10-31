#!/usr/bin/env python3
"""/red_angle 3초 평균 + 영상에서 빨간 원 픽셀 위치 → 광축 기준 벽 위 위치. 사용: angle_read.py D[m]
⚠️ fx 513.5·cx 326.8 은 OAK 640x360 ISP 1/3 camera_info 값(10-31) — 설정이 바뀌면 고칠 것."""
import math, time, numpy as np, cv2, rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
rclpy.init(); n = Node('angle_read'); br = CvBridge(); A = []; X = []
def ia(m):
    if m.data < 1000: A.append(m.data)
def im(m):
    f = br.imgmsg_to_cv2(m, 'bgr8'); hsv = cv2.cvtColor(f, cv2.COLOR_BGR2HSV)
    mk = cv2.inRange(hsv, (0,140,80), (5,255,255)) | cv2.inRange(hsv, (165,140,80), (180,255,255))
    cs, _ = cv2.findContours(mk, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    cs = [c for c in cs if cv2.contourArea(c) > 40]
    if cs:
        M = cv2.moments(max(cs, key=cv2.contourArea)); X.append((M['m10']/M['m00'], M['m01']/M['m00'], len(cs)))
n.create_subscription(Float32, '/red_angle', ia, 10); n.create_subscription(Image, '/oak/rgb/image_raw', im, 5)
t0 = time.time()
while time.time() - t0 < 3: rclpy.spin_once(n, timeout_sec=0.05)
if not X: print('원 안 보임'); raise SystemExit
x, y, k = np.mean([p[0] for p in X]), np.mean([p[1] for p in X]), max(p[2] for p in X)
fx = 513.5; cx = 326.8; import sys; D = float(sys.argv[1])
print(f'원 픽셀 x={x:.1f} y={y:.1f} (화면높이의 {y/360:.2f}) 빨간덩어리 {k}개')
if A: print(f'/red_angle 평균 {np.mean(A):+.2f}° (표본 {len(A)}, 흔들림 {np.std(A):.2f}°)')
else: print('/red_angle 유효값 없음 (게이트가 원을 부표로 인정 안 함)')
off = (x - cx) / fx * D
print(f'광축이 벽에 닿는 점 기준 원 위치: {"오른쪽" if off>0 else "왼쪽"} {abs(off)*100:.0f} cm (D={D} m 가정)')
