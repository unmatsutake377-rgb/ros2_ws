import time
import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, Int32, String, Float32MultiArray


IMAGE_ANGLE_INVALID = 10000.0
CANDIDATE_INVALID   = 20000.0
STOP_VALUE          = 50000.0   # LiDAR Stop Signal

DIST_VISIBLE_MAX    = 20.0      # 최대 인식 거리(20m)

def wrap360(x): return (x % 360 + 360) % 360
def clamp_pm180(a): return (a + 180) % 360 - 180
def rel_to_raw_0_360(a): return (a + 360) % 360


class ShipTurn(Node):
    def __init__(self):
        super().__init__('ship_turn')
        self.lock = threading.Lock()

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)

        # 파라미터: 기본값 3
        self.declare_parameter("active_wp_mode", 3)
        self.active_wp_mode = self.get_parameter("active_wp_mode").value

        # ⭐ WP3,4,5에서 활성
        self.allowed_wp_modes = (3, 4, 5)

        self.HOLD_SECONDS = self.declare_parameter("hold_seconds", 2.0).value
        self.TURN_SECONDS = self.declare_parameter("turn_seconds", 10.0).value

        self.LATERAL_OFF_M = 2.0    
        self.OFFSET_FALL_D = 2.5      

        self.LEFT_PASS_COLS  = set(["red", "orange"])
        self.RIGHT_PASS_COLS = set(["yellow", "green", "blue"])

        self.LEFT_TURN_DEG  = -50.0
        self.RIGHT_TURN_DEG = 50.0

        self.MSG_TIMEOUT_SEC = 2.5

        self.pub_candidate = self.create_publisher(Float32, '/candidate_angle', qos)
        self.create_subscription(Int32,   '/wp_mode', self.wp_cb, qos)
        self.create_subscription(Float32, '/image_angle', self.angle_cb, qos)
        self.create_subscription(Float32, '/image_distance', self.dist_cb, qos)
        self.create_subscription(String,  '/image_color', self.color_cb, qos)
        self.create_subscription(Float32MultiArray,
                                 '/obstacle_distance_array', self.obst_cb, qos)

        with self.lock:
            self.wp_mode = -1
            self.image_angle = float('nan')
            self.image_distance = float('nan')
            self.image_color = ""
            self.t_angle = self.t_dist = self.t_color = 0.0
            self.obst_distance = float('inf')
            self.obst_angle = float('nan')
            self.t_obst = 0.0
            self.turning = False
            self.turn_done = False
            self.turn_start = None
            self.mode_enter = None

        self.create_timer(0.15, self.timer_cb)

    def wp_cb(self, msg):
        with self.lock:
            prev = self.wp_mode
            self.wp_mode = msg.data
            if prev not in self.allowed_wp_modes and self.wp_mode in self.allowed_wp_modes:
                self.mode_enter = time.time()
                self.turning = False
                self.turn_done = False

    def angle_cb(self, msg):
        v = msg.data
        with self.lock:
            if abs(v - IMAGE_ANGLE_INVALID) < 1e-3:
                self.image_angle = float('nan')
                return
            self.image_angle = rel_to_raw_0_360(clamp_pm180(v))
            self.t_angle = time.time()

    def dist_cb(self, msg):
        with self.lock:
            self.image_distance = msg.data
            self.t_dist = time.time()

    def color_cb(self, msg):
        with self.lock:
            self.image_color = msg.data.strip().lower()
            self.t_color = time.time()

    def obst_cb(self, msg):
        with self.lock:
            if len(msg.data) >= 2:
                self.obst_distance = msg.data[0]
                self.obst_angle = msg.data[1]
                self.t_obst = time.time()

    def timer_cb(self):
        now = time.time()

        with self.lock:
            wp = self.wp_mode
            img_a = self.image_angle
            img_d = self.image_distance
            img_c = self.image_color
            t_a = self.t_angle
            t_d = self.t_dist
            t_c = self.t_color
            obst_d = self.obst_distance
            obst_a = self.obst_angle
            t_o = self.t_obst
            turning = self.turning
            turn_done = self.turn_done
            turn_start = self.turn_start
            mode_enter = self.mode_enter

        # ⭐ WP3,4,5 외에서는 동작 안 함
        if wp not in self.allowed_wp_modes:
            return

        if mode_enter is None or (now - mode_enter) < self.HOLD_SECONDS:
            self.pub_candidate.publish(Float32(data=CANDIDATE_INVALID))
            return

        if turn_done:
            self.pub_candidate.publish(Float32(data=CANDIDATE_INVALID))
            return

        if turning:
            if (now - turn_start < self.TURN_SECONDS):
                self.pub_candidate.publish(Float32(data=self.turn_fixed_angle))
                return
            with self.lock:
                self.turning = False
                self.turn_done = True
            self.pub_candidate.publish(Float32(data=CANDIDATE_INVALID))
            return

        invalid_img = (
            (now - t_a) > self.MSG_TIMEOUT_SEC or
            (now - t_d) > self.MSG_TIMEOUT_SEC or
            not math.isfinite(img_d) or
            img_d > DIST_VISIBLE_MAX
        )
        if invalid_img:
            self.pub_candidate.publish(Float32(data=CANDIDATE_INVALID))
            return

        base = img_a
        side = None
        if (now - t_c) <= self.MSG_TIMEOUT_SEC:
            if img_c in self.LEFT_PASS_COLS:
                side = 'left'
            elif img_c in self.RIGHT_PASS_COLS:
                side = 'right'

        offset_deg = math.degrees(math.atan(self.LATERAL_OFF_M / max(img_d, 0.01)))
        out = wrap360(base - offset_deg if side == 'left' else base + offset_deg if side == 'right' else base)

        if (now - t_o) <= self.MSG_TIMEOUT_SEC and math.isfinite(obst_d):

            if 140.0 <= obst_a <= 150.0 and obst_d <= 2.5:
                self._start_turn('left', now)
                return

            if 10.0 <= obst_a <= 20.0 and obst_d <= 2.5:
                self._start_turn('right', now)
                return

        self.pub_candidate.publish(Float32(data=out))

    def _start_turn(self, side, now):
        ang = self.LEFT_TURN_DEG if side == 'left' else self.RIGHT_TURN_DEG
        
        with self.lock:
            self.turning = True
            self.turn_done = False
            self.turn_start = now
            self.turn_fixed_angle = ang

        self.pub_candidate.publish(Float32(data=ang))


def main(args=None):
    rclpy.init(args=args)
    node = ShipTurn()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
