#!/usr/bin/env python3
"""OAK 영상 N장 → 도킹 노드와 같은 classify_shape·파라미터로 화면 속 빨강/초록 도형마다 판정. 사용: dock_label.py out.png [N]"""
import sys, time, math, collections, numpy as np, cv2, rclpy, yaml
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from color_shape_detector.dock_logic import classify_shape, circularity
from color_shape_detector.vision_geom import angle_from_pixel
OUT = sys.argv[1]; N = int(sys.argv[2]) if len(sys.argv) > 2 else 30
import os
Y = yaml.safe_load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'src', 'color_shape_detector', 'config', 'vision.yaml')))
P = dict(Y['/**']['ros__parameters']); P.update(Y['image_subscriber_dock']['ros__parameters']); P['hfov_deg'] = 63.9
def rng(v): return [(v[i:i+3], v[i+3:i+6]) for i in range(0, len(v), 6)]
COL = {'red': rng(P['hsv.red']), 'green': rng(P['hsv.green'])}
rclpy.init(); n = Node('dock_label'); br = CvBridge(); F = []
n.create_subscription(Image, '/oak/rgb/image_raw', lambda m: F.append(br.imgmsg_to_cv2(m, 'bgr8')) if len(F) < N else None, 5)
t = time.time()
while len(F) < N and time.time() - t < 20: rclpy.spin_once(n, timeout_sec=0.1)
if not F: print('영상 없음'); sys.exit(1)
votes = collections.defaultdict(list)
def analyse(f, draw=None):
    h, w = f.shape[:2]; hsv = cv2.cvtColor(f, cv2.COLOR_BGR2HSV); res = []
    for cname, rs in COL.items():
        m = np.zeros((h, w), np.uint8)
        for lo, hi in rs: m |= cv2.inRange(hsv, np.array(lo), np.array(hi))
        m = cv2.medianBlur(m, 3)
        cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cs:
            a = cv2.contourArea(c)
            if a < P['min_area_px']: continue
            per = cv2.arcLength(c, True); ap = cv2.approxPolyDP(c, P['approx_epsilon'] * per, True)
            x, y, bw, bh = cv2.boundingRect(ap)
            s = classify_shape(len(ap), a, per, bw, bh, min_area=P['min_area_px'], square_extent_min=P['square_extent_min'],
                               square_aspect_lo=P['square_aspect_lo'], square_aspect_hi=P['square_aspect_hi'],
                               circle_circularity_min=P['circle_circularity_min'], square_circularity_max=P['square_circularity_max'])
            v = ap.reshape(-1, 2); vX, vY = int(v[:, 0].mean()), int(v[:, 1].mean())
            inz = P['y_zone_lo'] * h <= vY <= P['y_zone_hi'] * h
            res.append(dict(color=cname, shape=s or '판정없음', v=len(ap), circ=circularity(a, per), area=a, x=vX, y=vY, inz=inz,
                            ang=angle_from_pixel(vX, w, P['hfov_deg'])))
            if draw is not None:
                col = (0, 255, 0) if s and inz else (0, 165, 255) if s else (0, 0, 255)
                cv2.drawContours(draw, [ap], -1, col, 2)
                lab = f"{cname} {s or 'none'} v{len(ap)} c{circularity(a, per):.2f}"
                cv2.putText(draw, lab, (x, max(12, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 3)
                cv2.putText(draw, lab, (x, max(12, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
    return res
for f in F:
    for r in analyse(f): votes[(r['color'], round(r['x'] / 20))].append(r)
img = F[-1].copy(); h, w = img.shape[:2]
for yy in (P['y_zone_lo'], P['y_zone_hi']): cv2.line(img, (0, int(h * yy)), (w, int(h * yy)), (255, 200, 0), 1)
last = analyse(F[-1], img); cv2.imwrite(OUT, img)
print(f'{N}장 판정 (도킹 파라미터: eps {P["approx_epsilon"]}, 원형도 원≥{P["circle_circularity_min"]}, 사각<{P["square_circularity_max"]}, y띠 {P["y_zone_lo"]}~{P["y_zone_hi"]})')
print('색 | 위치x | 판정(장수) | 꼭짓점 | 원형도 | 면적 | y띠 | 각도')
for k, rs in sorted(votes.items(), key=lambda kv: kv[1][0]['x']):
    if len(rs) < N * 0.3: continue
    cnt = collections.Counter(r['shape'] for r in rs)
    vs = collections.Counter(r['v'] for r in rs)
    print(f"{k[0]} | {np.mean([r['x'] for r in rs]):.0f} | {dict(cnt)} | {dict(vs)} | {np.mean([r['circ'] for r in rs]):.2f} | {np.mean([r['area'] for r in rs]):.0f} | {'안' if rs[-1]['inz'] else '밖'} | {np.mean([r['ang'] for r in rs]):+.1f}°")
