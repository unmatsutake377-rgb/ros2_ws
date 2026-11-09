"""도킹 물보라 필터(front_median_kernel) 벤치 — 계약서_도킹물보라필터_20261109 ②~④."""
import sys
import rclpy
from benchlib import Bench, scan_ranges, motor_fmt, INC_DEG

rclpy.init()
b = Bench(sys.argv[1])
BASE = scan_ranges([], default=5.0)                       # 전방 5 m
ONE = scan_ranges([(80.0, 80.0, 0.5)], default=5.0)       # 정면 한 광선만 0.5 m
THREE = scan_ranges([(79.5, 80.5, 0.8)], default=5.0)     # 정면 연속 3광선 0.8 m


def sparse(r=0.6, every=10):
    """고립 반사만: 10 광선(5°)마다 1점, 나머지 inf (신선)."""
    rs = scan_ranges([])
    for i in range(0, len(rs), every):
        rs[i] = r
    return rs


def enter(label, scan):
    b.log(f'### {label}: wp_mode 6 → 7, goal 5.0, image 0.0')
    b.set('/scan', BASE, 10)
    b.set('/wp_mode', 6, 2)
    b.spin_for(2.5)
    b.set('/wp_mode', 7, 2)
    b.spin_for(4.0)                                       # init_wait 3 s
    b.set('/scan', scan, 10)
    return b.now()


def show(label, t_from, t_to=None):
    b.summary(label, '/candidate_angle', t_from, t_to)
    b.summary(label, 'Motor_run', t_from, t_to, fmt=motor_fmt)


b.log(f'(scan 0.5° 간격 {INC_DEG}, 정면 ±15° = 61 광선)')
b.set('/goal_distance', 5.0, 2)
b.set('/image_angle', 0.0, 10)
b.set('/yaw_error', 0.0, 20)

t = enter('② 정면 한 광선 0.5 m', ONE)
b.spin_for(4.0)
show('② 한 광선 0.5 m', t + 0.5)

b.log('# ③ 정면 연속 3광선 0.8 m')
b.set('/scan', THREE, 10)
t = b.now()
b.spin_for(1.5)
show('③ 3광선 0.8 m', t + 0.3)

b.log('# ④ CONTACT 3 s 뒤 BACKOFF 중 고립 반사만(10 광선마다 0.6 m)')
b.set('/scan', sparse(), 10)
t = b.now()
b.spin_for(7.0)
show('④ 고립 반사 3.5~7 s (BACKOFF)', t + 3.5, t + 7.0)

t = enter('④-2 APPROACH 중 고립 반사만', sparse())
b.spin_for(4.0)
show('④-2 APPROACH 고립 반사', t + 0.5)
b.set('/scan', BASE, 10)
b.spin_for(1.0)
rclpy.shutdown()
