import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, Int32


# CONSTANTS
STOP_VALUE = 50000.0
RIGHT_SPIN = 5000.0  # 오른쪽 회전
DIST_THRESHOLD = 1.5
INVALID = 10000.0

FORWARD_DURATION = 5.0  # 시작 STOP 유지

def apply_angle_offset(raw_angle):
    if -15.0 <= raw_angle <= 15.0:
        return raw_angle
    if 15.0 < raw_angle <= 90.0:
        return raw_angle + 5.0
    if -90.0 <= raw_angle < -15.0:
        return raw_angle - 5.0
    return raw_angle


class ShipDock(Node):
    def __init__(self):
        super().__init__("ship_dock")

        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)

        self.declare_parameter("active_wp_mode", 9)
        self.active_wp_mode = self.get_parameter("active_wp_mode").value

        self.lock = threading.Lock()

        self.pub_candidate = self.create_publisher(Float32, "/candidate_angle", qos)

        self.create_subscription(Int32, "/wp_mode", self.wp_mode_cb, qos)
        self.create_subscription(Float32, "/image_angle", self.image_angle_cb, qos)
        self.create_subscription(Float32, "/image_distance", self.image_distance_cb, qos)

        self.wp_mode = -1
        self.image_angle = INVALID
        self.image_distance = INVALID

        # FSM states
        self.state = "INIT_WAIT"
        self.state_start_time = None

        self.create_timer(0.5, self.timer_cb)


    def wp_mode_cb(self, msg):
        with self.lock:
            mode = msg.data
            if mode != self.active_wp_mode:
                self.state = "IDLE"
                self.state_start_time = None
            self.wp_mode = mode

    def image_angle_cb(self, msg):
        with self.lock:
            self.image_angle = msg.data

    def image_distance_cb(self, msg):
        with self.lock:
            self.image_distance = msg.data

    def in_state(self, name):
        return self.state == name

    def set_state(self, new_state):
        self.state = new_state
        self.state_start_time = time.time()

    def elapsed(self):
        if self.state_start_time is None:
            return 0
        return time.time() - self.state_start_time


    def timer_cb(self):

        with self.lock:
            if self.wp_mode != self.active_wp_mode:
                return

            # INIT_WAIT
            if self.in_state("INIT_WAIT"):
                if self.state_start_time is None:
                    self.state_start_time = time.time()

                if self.elapsed() < FORWARD_DURATION:
                    self.pub_candidate.publish(Float32(data=STOP_VALUE))
                    return

                self.set_state("SEARCH_RIGHT_ONLY")
                return


            # SEARCH_RIGHT_ONLY
            if self.in_state("SEARCH_RIGHT_ONLY"):

                # 도형 인식되면 DOCKING으로
                if self.image_angle != INVALID:
                    self.set_state("DOCKING")
                    return

                # 계속 오른쪽 회전
                self.pub_candidate.publish(Float32(data=RIGHT_SPIN))
                return


            # DOCKING
            if self.in_state("DOCKING"):

                # 도킹거리 이내 → STOP 유지 상태
                if self.image_distance < DIST_THRESHOLD:
                    self.set_state("DOCK_STOP")
                    return

                adj = apply_angle_offset(self.image_angle)
                self.pub_candidate.publish(Float32(data=float(adj)))
                return


            # DOCK_STOP → 정지 유지
            if self.in_state("DOCK_STOP"):
                self.pub_candidate.publish(Float32(data=STOP_VALUE))
                return



def main(args=None):
    rclpy.init(args=args)
    node = ShipDock()

    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
