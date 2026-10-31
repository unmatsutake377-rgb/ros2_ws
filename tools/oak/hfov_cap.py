#!/usr/bin/env python3
"""OAK 영상 N장에서 빨간 블롭 중심을 평균 → fx·hfov 계산. 사용: hfov_cap.py W[m] D[m] out.png"""
import sys, math, time, numpy as np, cv2, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
W, D, OUT = float(sys.argv[1]), float(sys.argv[2]), sys.argv[3]
N = 60
class Cap(Node):
    def __init__(s):
        super().__init__('hfov_cap'); s.br = CvBridge(); s.frames = []; s.ci = None
        s.create_subscription(Image, '/oak/rgb/image_raw', s.cb, 5)
        s.create_subscription(CameraInfo, '/oak/rgb/camera_info', lambda m: setattr(s, 'ci', m), 5)
    def cb(s, m):
        if len(s.frames) < N: s.frames.append(s.br.imgmsg_to_cv2(m, 'bgr8'))
rclpy.init(); n = Cap(); t0 = time.time()
while (len(n.frames) < N or n.ci is None) and time.time() - t0 < 30: rclpy.spin_once(n, timeout_sec=0.1)
if len(n.frames) < N: print('영상 못 받음', len(n.frames)); sys.exit(1)
img = n.frames[-1].copy(); h, w = img.shape[:2]
def blobs(f):
    hsv = cv2.cvtColor(f, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, (0, 140, 80), (5, 255, 255)) | cv2.inRange(hsv, (165, 140, 80), (180, 255, 255))
    cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    out = []
    for c in cs:
        a = cv2.contourArea(c)
        if a < 40: continue
        M = cv2.moments(c); out.append((M['m10']/M['m00'], M['m01']/M['m00'], a))
    return sorted(out, key=lambda b: -b[2])
per = [blobs(f) for f in n.frames]
cnt = [len(p) for p in per]
print(f'해상도 {w}x{h}, 프레임 {N}장, 프레임별 빨간 블롭 수 min {min(cnt)} max {max(cnt)}')
b0 = per[-1]
for x, y, a in b0[:6]: print(f'  블롭 중심 ({x:.1f}, {y:.1f}) 면적 {a:.0f}px²  (화면 높이의 {y/h:.2f})')
cv2.line(img, (w//2, 0), (w//2, h), (255, 255, 0), 1); cv2.line(img, (0, h//2), (w, h//2), (255, 255, 0), 1)
for x, y, a in b0[:6]: cv2.drawMarker(img, (int(x), int(y)), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
cv2.imwrite(OUT, img)
if min(cnt) >= 2:
    xs = np.array([sorted([b[0] for b in p[:2]]) for p in per]); ys = np.array([[b[1] for b in sorted(p[:2])] for p in per])
    xl, xr = xs.mean(0); dx = xr - xl
    fx = dx * D / W; hf = 2 * math.degrees(math.atan((w/2) / fx))
    K = n.ci.k; fxc = K[0]; hfc = 2 * math.degrees(math.atan((w/2) / fxc))
    dx_all = xs[:,1]-xs[:,0]
    print(f"원 간격 프레임별: 평균 {dx_all.mean():.2f} 표준편차 {dx_all.std():.2f} 최소 {dx_all.min():.1f} 최대 {dx_all.max():.1f}")
    print(f'두 원 중심 x = {xl:.2f}, {xr:.2f} (프레임 간 흔들림 {xs.std(0).max():.2f}px) → 간격 {dx:.2f}px')
    print(f'실측(원본 픽셀) fx = {fx:.1f} → hfov = {hf:.2f}°')
    pts = np.array([[[xl, ys.mean(0)[0]]], [[xr, ys.mean(0)[1]]]], dtype=np.float64)
    Km = np.array(K).reshape(3, 3); und = cv2.undistortPoints(pts, Km, np.array(n.ci.d), P=Km).reshape(-1, 2)
    fxu = (und[1, 0] - und[0, 0]) * D / W
    print(f'왜곡 보정 후 fx = {fxu:.1f} → hfov = {2*math.degrees(math.atan((w/2)/fxu)):.2f}°')
    print(f'카메라 보정값 fx = {fxc:.1f} → hfov = {hfc:.2f}° | 실측과 차이 {hf-hfc:+.2f}°')
    print(f'좌우 대칭: 화면 중심에서 왼쪽 원 {w/2-xl:.1f}px, 오른쪽 원 {xr-w/2:.1f}px')
