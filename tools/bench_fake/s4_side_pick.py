import subprocess
import sys
import rclpy
from benchlib import Bench, scan_ranges

rclpy.init()
b = Bench(sys.argv[1])
for key in ('side_hysteresis_deg', 'cross_penalty_deg'):
    r = subprocess.run(['ros2', 'param', 'get', '/ship_direction', key], capture_output=True, text=True, timeout=20)
    b.log(f'param get {key}: {r.stdout.strip() or r.stderr.strip()}')
LEFT7 = scan_ranges([(82.5, 91.5, 1.9)])
b.set('/candidate_angle', 20000.0, 10)
b.set('/wp_mode', 5, 1)
b.log('# 정면 7° 왼쪽(중심 87) 부표 1.9 m + 목표 20° 왼쪽 (토픽 −20)')
b.set('/scan', LEFT7, 10)
b.set('/yaw_error', -20.0, 20)  # 토픽 −20 = 내부 20 = 목표 왼쪽
t = b.now()
b.spin_for(5.0)
b.summary('LEFT7 yaw왼쪽20', '/desired_angle', t + 2.0)
b.set('/scan', scan_ranges([], default=10.0), 10)
b.set('/yaw_error', 0.0, 20)
b.spin_for(3.0)
b.log('# 정면 중심 80 부표 1.9 m + /yaw_error ±2° 0.5 s 마다 교대 (20 s)')
CENTER = scan_ranges([(75.5, 84.5, 1.9)])
b.set('/scan', CENTER, 10)
t = b.now()
sign = 1
for _ in range(40):
    b.set('/yaw_error', 2.0 * sign, 20)
    sign = -sign
    b.spin_for(0.5)
vals = b.window('/desired_angle', t + 1.0)
left = sum(1 for v in vals if 80.0 < v <= 160.0)
right = sum(1 for v in vals if 0.0 <= v < 80.0)
flips, prev = 0, None
for v in vals:
    s = 'L' if v > 80.0 else ('R' if v < 80.0 else 'C')
    if prev and s != prev:
        flips += 1
    prev = s
b.summary('CENTER ±2', '/desired_angle', t + 1.0)
b.log(f'== CENTER ±2: 왼쪽 {left} / 오른쪽 {right} / 편 바뀜 {flips}회')
rclpy.shutdown()
