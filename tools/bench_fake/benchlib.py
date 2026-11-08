"""센서 없는 벤치용 가짜 토픽 발행 + 기록. 노드는 run_bench.sh 가 띄운다 (2026-11-08 1번배 벤치)."""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32, Int32, Bool
from sensor_msgs.msg import LaserScan
from rcl_interfaces.msg import Log

INC_DEG = 0.5
WATCH_NODES = ('ship_direction', 'motor_control', 'ship_dock')


def scan_ranges(obstacles, default=math.inf):
    """0~179.5° 0.5° 간격. obstacles = [(lo_deg, hi_deg, r)]. 80° = 정면, 클수록 왼쪽."""
    n = 360
    rs = [default] * n
    for lo, hi, r in obstacles:
        for i in range(n):
            if lo <= i * INC_DEG <= hi:
                rs[i] = r
    return rs


class Bench(Node):
    def __init__(self, out_path):
        super().__init__('bench_harness')
        self.out = open(out_path, 'w')
        self.t0 = time.monotonic()
        self.pubs = {}
        self.state = {}          # 토픽 -> 현재 발행할 값 (None = 발행 안 함)
        self.rate = {}           # 토픽 -> Hz
        self.last_pub = {}
        self.rx = {}             # 토픽 -> [(t, val)]
        best = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pubs['/scan'] = self.create_publisher(LaserScan, '/scan', best)
        for name in ('/yaw_error', '/candidate_angle', '/goal_distance',
                     '/image_angle', '/desired_angle', '/wp_remaining_time'):
            self.pubs[name] = self.create_publisher(Float32, name, 10)
        self.pubs['/wp_mode'] = self.create_publisher(Int32, '/wp_mode', 10)
        for name, typ in (('/desired_angle', Float32), ('/candidate_angle', Float32),
                          ('/failsafe_level', Int32), ('Motor_run', Int32),
                          ('/motor_reverse', Bool)):
            self.create_subscription(typ, name, self._mk_rx(name), 10)
        self.create_subscription(Log, '/rosout', self._rosout, 100)
        self.create_timer(0.01, self._tick)

    def now(self):
        return time.monotonic() - self.t0

    def log(self, s):
        line = f'{self.now():7.2f} {s}'
        print(line, flush=True)
        self.out.write(line + '\n')
        self.out.flush()

    def _mk_rx(self, name):
        def cb(msg):
            self.rx.setdefault(name, []).append((self.now(), msg.data))
        return cb

    def _rosout(self, msg):
        if msg.name in WATCH_NODES:
            lv = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}.get(msg.level, msg.level)
            self.log(f'[ROSOUT {msg.name} {lv}] {msg.msg}')

    def set(self, topic, value, hz=None):
        self.state[topic] = value
        if hz is not None:
            self.rate[topic] = hz
        self.last_pub.setdefault(topic, -1e9)

    def _tick(self):
        t = self.now()
        for topic, val in self.state.items():
            if val is None:
                continue
            hz = self.rate.get(topic, 10.0)
            if t - self.last_pub[topic] < 1.0 / hz:
                continue
            self.last_pub[topic] = t
            if topic == '/scan':
                m = LaserScan()
                m.header.frame_id = 'laser'
                m.header.stamp = self.get_clock().now().to_msg()
                m.angle_min = 0.0
                m.angle_increment = math.radians(INC_DEG)
                m.angle_max = math.radians(INC_DEG * (len(val) - 1))
                m.range_min = 0.15
                m.range_max = 25.0
                m.ranges = [float(x) for x in val]
            elif topic == '/wp_mode':
                m = Int32(data=int(val))
            else:
                m = Float32(data=float(val))
            self.pubs[topic].publish(m)

    def spin_for(self, sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def window(self, topic, t_from, t_to=None):
        t_to = self.now() if t_to is None else t_to
        return [v for t, v in self.rx.get(topic, []) if t_from <= t <= t_to]

    def summary(self, label, topic, t_from, t_to=None, fmt=None):
        vals = self.window(topic, t_from, t_to)
        if fmt:
            vals = [fmt(v) for v in vals]
        counts = {}
        for v in vals:
            k = round(v, 2) if isinstance(v, float) else v
            counts[k] = counts.get(k, 0) + 1
        self.log(f'== {label}: {topic} n={len(vals)} 값별 {dict(sorted(counts.items(), key=lambda kv: -kv[1])[:8])}')
        return vals

    def gaps(self, topic, t_from, t_to=None):
        t_to = self.now() if t_to is None else t_to
        ts = [t for t, _ in self.rx.get(topic, []) if t_from <= t <= t_to]
        return max((b - a for a, b in zip(ts, ts[1:])), default=float('nan'))


def motor_fmt(v):
    return f'R{v // 10000}/L{v % 10000}'
