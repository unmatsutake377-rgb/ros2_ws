import sys
import rclpy
from benchlib import Bench, motor_fmt

rclpy.init()
b = Bench(sys.argv[1])
b.set('/desired_angle', 80.0, 20)
for d in (10.0, 6.0, 3.0, 0.0):
    b.log(f'# /desired_angle 80 + /goal_distance {d}')
    b.set('/goal_distance', d, 2)
    t = b.now()
    b.spin_for(4.0)
    b.summary(f'goal {d}', 'Motor_run', t + 1.5, fmt=motor_fmt)
b.log('# /goal_distance 발행 중단 (마지막 0.0)')
b.set('/goal_distance', None)
t = b.now()
b.spin_for(4.0)
for a, z in ((0.0, 1.5), (2.6, 4.0)):
    b.summary(f'중단 뒤 {a}~{z}s', 'Motor_run', t + a, t + z, fmt=motor_fmt)
b.set('/goal_distance', 0.0, 2)
for ang in (260.0, 5000.0, 6000.0):
    b.log(f'# /goal_distance 0.0 + /desired_angle {ang}')
    b.set('/desired_angle', ang, 20)
    t = b.now()
    b.spin_for(3.0)
    b.summary(f'angle {ang}', 'Motor_run', t + 1.0, fmt=motor_fmt)
b.log('# /goal_distance 0.0 + /desired_angle 120 (왼쪽 조향) 차동 확인')
b.set('/desired_angle', 120.0, 20)
t = b.now()
b.spin_for(3.0)
b.summary('angle 120 goal 0', 'Motor_run', t + 1.0, fmt=motor_fmt)
b.set('/goal_distance', None)
t = b.now()
b.spin_for(4.0)
b.summary('angle 120 goal 없음', 'Motor_run', t + 2.6, fmt=motor_fmt)
rclpy.shutdown()
