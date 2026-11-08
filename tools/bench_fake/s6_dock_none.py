import sys
import rclpy
from benchlib import Bench, scan_ranges, motor_fmt

rclpy.init()
b = Bench(sys.argv[1])
INF = scan_ranges([])                                  # 신선한데 전방(65~95°) 유한 반환 없음


def FRONT(r):
    return scan_ranges([(70.0, 90.0, r)])            # 정면 ±10° 에 r


def enter_approach(label, scan):
    b.log(f'### {label}: wp_mode 6 → 7, goal 5.0, image 0.0, /scan={"없음" if scan is None else "있음"}')
    b.set('/wp_mode', 6, 2)
    b.spin_for(2.5)
    b.set('/scan', scan, 10)
    b.set('/wp_mode', 7, 2)
    t = b.now()
    b.spin_for(10.0)
    return t


def show(label, t_from, t_to=None):
    b.summary(label, '/candidate_angle', t_from, t_to)
    b.summary(label, '/desired_angle', t_from, t_to)
    b.summary(label, 'Motor_run', t_from, t_to, fmt=motor_fmt)
    b.log(f'   /candidate_angle 최대 간격 {b.gaps("/candidate_angle", t_from, t_to):.3f}s')


b.set('/goal_distance', 5.0, 2)
b.set('/image_angle', 0.0, 10)
b.set('/yaw_error', 0.0, 20)

# (가) /scan 없음 → APPROACH 정지
t = enter_approach('(가)', None)
show('(가) 진입 7~10 s', t + 7.0)

# (나) 전방 0.8 m → CONTACT, 이후 전방 inf(신선) → 3 s 뒤 BACKOFF 180, 8 s 뒤 타임아웃
b.log('# (나) /scan 전방 0.8 m')
b.set('/scan', FRONT(0.8), 10)
t = b.now()
b.spin_for(1.5)
show('(나) 0.8 m', t + 0.3)
b.log('# (나) /scan 전방 inf (신선)')
b.set('/scan', INF, 10)
t = b.now()
b.spin_for(14.0)
show('(나) inf 0~2.8 s (CONTACT)', t, t + 2.8)
show('(나) inf 3.3~10.5 s (BACKOFF)', t + 3.3, t + 10.5)
show('(나) inf 11.6~14 s (타임아웃 뒤)', t + 11.6, t + 14.0)

# (나-2) BACKOFF 중 /scan 중단(stale) → 후진 안 함(결정 2b)
t = enter_approach('(나-2)', FRONT(0.8))
b.log('# (나-2) 0.8 m 접안 뒤 /scan 중단')
b.set('/scan', None)
t = b.now()
b.spin_for(8.0)
show('(나-2) 중단 3.5~8 s', t + 3.5, t + 8.0)
b.set('/scan', INF, 10)

# (다) BACKOFF 중 전방 3.0 m 복귀 → DONE 20000
t = enter_approach('(다)', FRONT(0.8))
b.set('/scan', INF, 10)
t = b.now()
b.spin_for(5.0)
show('(다) BACKOFF 3.5~5 s', t + 3.5, t + 5.0)
b.log('# (다) /scan 전방 3.0 m')
b.set('/scan', FRONT(3.0), 10)
t = b.now()
b.spin_for(3.0)
show('(다) 3.0 m 뒤', t + 0.5)
ts = [x for x, _ in b.rx.get('/candidate_angle', [])]
for a, c in zip(ts, ts[1:]):
    if c - a > 0.15:
        b.log(f'== /candidate_angle 빈 구간 {a:.2f} → {c:.2f} ({c - a:.3f}s)')
rclpy.shutdown()
