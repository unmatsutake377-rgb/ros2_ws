import time
import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, Int32

INVALID_ANGLE = 10000.0
CANDIDATE_INVALID = 20000.0

MSG_TIMEOUT_SEC = 0.7
DELTA_FIX = 40.0  # 🔥 한쪽만 보일 때 적용할 고정 보정각 (조절하면 됨)


def clamp_pm180(a):
    return (a + 180) % 360 - 180


def rel_to_raw_0_360(a):
    return (a + 360) % 360


def circular_mid(a1, a2):
    """두 부표 보일 때 중간 각"""
    if abs(a1 - a2) > 180:
        if a1 < a2:
            a1 += 360
        else:
            a2 += 360
    return clamp_pm180((a1 + a2) / 2)


class ShipGate(Node):
    def __init__(self):
        super().__init__("ship_gate")

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.lock = threading.Lock()

        # Publisher
        self.pub_candidate = self.create_publisher(Float32, '/candidate_angle', qos)

        # Subscribers
        self.create_subscription(Int32, '/wp_mode', self.wp_mode_cb, qos)
        self.create_subscription(Float32, '/green_angle', self.red_angle_cb, qos)
        self.create_subscription(Float32, '/red_angle', self.yellow_angle_cb, qos)
        self.create_subscription(Float32, '/green_distance', self.red_dist_cb, qos)
        self.create_subscription(Float32, '/red_distance', self.yellow_dist_cb, qos)

        # 내부 상태
        self.declare_parameter("active_wp_mode", 1)
        self.active_wp_mode = self.get_parameter("active_wp_mode").value
        self.wp_mode = -1

        self.a1 = INVALID_ANGLE  # red
        self.a2 = INVALID_ANGLE  # yellow
        self.d1 = float('inf')
        self.d2 = float('inf')
        self.t_d1 = 0.0
        self.t_d2 = 0.0

        self.create_timer(0.2, self.timer_cb)

    # ---------------------- Callbacks ----------------------
    def wp_mode_cb(self, msg):
        with self.lock:
            self.wp_mode = msg.data

    def red_angle_cb(self, msg):
        with self.lock:
            self.a1 = msg.data  # red angle

    def yellow_angle_cb(self, msg):
        with self.lock:
            self.a2 = msg.data  # yellow angle

    def red_dist_cb(self, msg):
        with self.lock:
            self.d1 = msg.data
            self.t_d1 = time.time()

    def yellow_dist_cb(self, msg):
        with self.lock:
            self.d2 = msg.data
            self.t_d2 = time.time()

    # ======================================================
    def timer_cb(self):
        with self.lock:
            if self.wp_mode != self.active_wp_mode:
                return

            a1 = clamp_pm180(self.a1) if self.a1 != INVALID_ANGLE else INVALID_ANGLE
            a2 = clamp_pm180(self.a2) if self.a2 != INVALID_ANGLE else INVALID_ANGLE

            now = time.time()
            d1 = self.d1 if now - self.t_d1 <= MSG_TIMEOUT_SEC else float('inf')
            d2 = self.d2 if now - self.t_d2 <= MSG_TIMEOUT_SEC else float('inf')

        # ---------- 1) 둘 다 안 보임 ----------
        if a1 == INVALID_ANGLE and a2 == INVALID_ANGLE:
            self.pub_candidate.publish(Float32(data=CANDIDATE_INVALID))
            return

        # ---------- 2) 둘 다 보임 ----------
        if a1 != INVALID_ANGLE and a2 != INVALID_ANGLE:
            mid = circular_mid(a1, a2)
            self.pub_candidate.publish(Float32(data=rel_to_raw_0_360(mid)))
            return

        # ---------- 3) red만 보임 ----------
        if a1 != INVALID_ANGLE and a2 == INVALID_ANGLE:
            cand_rel = clamp_pm180(a1 - DELTA_FIX)
            self.pub_candidate.publish(Float32(data=rel_to_raw_0_360(cand_rel)))
            return

        # ---------- 4) yellow만 보임 ----------
        if a1 == INVALID_ANGLE and a2 != INVALID_ANGLE:
            cand_rel = clamp_pm180(a2 + DELTA_FIX)
            self.pub_candidate.publish(Float32(data=rel_to_raw_0_360(cand_rel)))
            return


def main(args=None):
    rclpy.init(args=args)
    node = ShipGate()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
