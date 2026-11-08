import subprocess
import sys
import rclpy
from benchlib import Bench, scan_ranges

rclpy.init()
b = Bench(sys.argv[1])
for node, key in (('/ship_direction', 'side_escape_deg'), ('/ship_back', 'hold_dist_m')):
    r = subprocess.run(['ros2', 'param', 'get', node, key], capture_output=True, text=True, timeout=20)
    b.log(f'param get {node} {key}: {r.stdout.strip() or r.stderr.strip()}')

LEFT = scan_ranges([(0.0, 89.5, 1.5), (100.0, 120.0, 0.45)])
RIGHT = scan_ranges([(40.0, 60.0, 0.45), (70.5, 160.0, 1.5)])
WALL = scan_ranges([(0.0, 160.0, 0.5)])
b.set('/candidate_angle', 20000.0, 10)
b.set('/wp_mode', 5, 1)
b.set('/yaw_error', -20.0, 20)  # 테스트와 같은 내부 20
for name, sc in (('좌 110° 0.45 m', LEFT), ('우 50° 0.45 m (거울)', RIGHT), ('정면 벽 0.5 m', WALL)):
    b.log(f'# /scan {name} + yaw 왼쪽20(토픽 −20) + 20000 + wp_mode 5')
    b.set('/scan', sc, 10)
    t = b.now()
    b.spin_for(5.0)
    b.summary(name, '/desired_angle', t, t + 5.0)
    seq = b.window('/desired_angle', t, t + 5.0)
    b.log(f'   첫 15개: {[round(v) for v in seq[:15]]}')
    b.set('/scan', scan_ranges([], default=10.0), 10)
    b.spin_for(4.0)
b.log('# wp_mode 1(게이트) + 좌 110° → 비킴 안 함(후진) 확인')
b.set('/wp_mode', 1, 1)
b.spin_for(1.5)
b.set('/scan', LEFT, 10)
t = b.now()
b.spin_for(4.0)
b.summary('wp_mode 1 좌 110°', '/desired_angle', t, t + 4.0)
rclpy.shutdown()
