import sys
import rclpy
from benchlib import Bench, scan_ranges

rclpy.init()
b = Bench(sys.argv[1])
clear = scan_ranges([], default=10.0)
b.log('# 1단계: /scan 정상 + /candidate_angle 20000 + /wp_mode 1, /yaw_error 없음 (GPS 없는 부팅)')
b.set('/scan', clear, 10)
b.set('/candidate_angle', 20000.0, 10)
b.set('/wp_mode', 1, 1)
b.spin_for(12.0)
b.summary('1단계', '/desired_angle', 1.0, 12.0)
b.summary('1단계', '/failsafe_level', 1.0, 12.0)
b.log('# 2단계: /yaw_error 0.0 20 Hz 시작')
t2 = b.now()
b.set('/yaw_error', 0.0, 20)
b.spin_for(8.0)
b.summary('2단계', '/desired_angle', t2 + 1.0)
b.summary('2단계', '/failsafe_level', t2 + 1.0)
b.log(f'== 2단계 /desired_angle 최대 간격 {b.gaps("/desired_angle", t2 + 1.0):.3f}s')
rclpy.shutdown()
