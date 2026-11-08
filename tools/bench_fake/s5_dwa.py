import statistics
import subprocess
import sys
import rclpy
from benchlib import Bench, scan_ranges

rclpy.init()
b = Bench(sys.argv[1])
for key in ('dwa_enabled', 'dwa_modes', 'dwa_hull', 'dwa_model_agility', 'dwa_lidar_x_m'):
    r = subprocess.run(['ros2', 'param', 'get', '/ship_direction', key], capture_output=True, text=True, timeout=20)
    b.log(f'param get {key}: {r.stdout.strip() or r.stderr.strip()}')
SCENES = (('정면 1.5 m 부표(78~82°)', scan_ranges([(78.0, 82.0, 1.5)])),
          ('정면 약간 왼쪽 2.8 m(82~86°)', scan_ranges([(82.0, 86.0, 2.8)])),
          ('부표 6개 2.8 m(갭 0)', scan_ranges([(c - 1.0, c + 1.0, 2.8) for c in (10.0, 35.0, 65.0, 95.0, 125.0, 150.0)])),
          ('사방 0.3 m', scan_ranges([(0.0, 160.0, 0.3)])))
b.set('/candidate_angle', 20000.0, 10)
b.set('/yaw_error', 0.0, 20)
for mode in (8, 7, 0):
    b.set('/wp_mode', mode, 1)
    b.set('/scan', scan_ranges([], default=10.0), 10)
    b.spin_for(2.0)
    for name, sc in SCENES:
        b.set('/scan', sc, 10)
        t = b.now()
        b.spin_for(3.0)
        b.summary(f'wp_mode {mode} {name}', '/desired_angle', t + 1.0)
        b.set('/scan', scan_ranges([], default=10.0), 10)
        b.spin_for(3.0)
# 지터: wp_mode 8, 정면 1.5 m 부표 20 s
b.set('/wp_mode', 8, 1)
b.set('/scan', SCENES[0][1], 10)
b.spin_for(2.0)
t = b.now()
b.spin_for(20.0)
ts = [x for x, _ in b.rx.get('/desired_angle', []) if x >= t]
d = sorted(bb - a for a, bb in zip(ts, ts[1:]))
if d:
    b.log(f'== 지터(wp_mode 8 정면 1.5 m, 20 s): n={len(d)} 간격 p50 {statistics.median(d)*1000:.1f} ms '
          f'p99 {d[int(len(d)*0.99)-1]*1000:.1f} ms 최대 {d[-1]*1000:.1f} ms')
rclpy.shutdown()
