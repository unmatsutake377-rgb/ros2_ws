import time
import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, Int32, String, Float32MultiArray

IMAGE_ANGLE_INVALID = 10000.0
CANDIDATE_INVALID   = 20000.0
STOP_VALUE          = 50000.0

# ===== 회피 / 정지 파라미터 =====
BUOY_NEAR_DISTANCE = 1.2
AVOID_SECONDS      = 2.0
BUOY_HOLD_SECONDS  = 5.0

LATERAL_OFF_M      = 1.0
OFFSET_FALL_D      = 1.5

LIDAR_SEARCH_SEC   = 20.0
MSG_TIMEOUT_SEC    = 4.0
LIDAR_STOP_DIST    = 1.0
LIDAR_STOP_SEC     = 5.0

DIST_VISIBLE_MAX   = 20.0

# LiDAR 분석 각도 범위
LIDAR_MIN_DEG = 1.0
LIDAR_MAX_DEG = 159.0

def wrap360(x): return (x % 360 + 360) % 360
def clamp_pm180(a): return (a + 180) % 360 - 180


class ShipTurn(Node):
    def __init__(self):
        super().__init__('ship_turn')
        self.lock = threading.Lock()

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.active_wp_mode = self.declare_parameter("active_wp_mode", 2).value

        self.pub_candidate = self.create_publisher(Float32, '/candidate_angle', qos)
        self.create_subscription(Int32,   '/wp_mode',        self.wp_cb,   qos)
        self.create_subscription(Float32, '/image_angle',    self.angle_cb, qos)
        self.create_subscription(Float32, '/image_distance', self.dist_cb,  qos)
        self.create_subscription(String,  '/image_color',    self.color_cb, qos)
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

            self.state = -1
            self.state_enter_time = time.time()

            self.lidar_stop = False
            self.lidar_stop_time = 0.0

            self.avoid_angle = 0.0

        self.create_timer(0.15, self.timer_cb)

    # ======================================
    # CALLBACKS
    # ======================================
    def wp_cb(self, msg):
        with self.lock:
            prev = self.wp_mode
            self.wp_mode = msg.data
            if prev != self.active_wp_mode and self.wp_mode == self.active_wp_mode:
                self.state = -1
                self.state_enter_time = time.time()
                self.lidar_stop = False

    def angle_cb(self, msg):
        v = msg.data
        with self.lock:
            if abs(v - IMAGE_ANGLE_INVALID) < 1e-3:
                self.image_angle = float('nan')
            else:
                self.image_angle = clamp_pm180(v)
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
                obstacle_angle = msg.data[1]
                obstacle_dist  = msg.data[0]

                if (LIDAR_MIN_DEG <= obstacle_angle <= LIDAR_MAX_DEG):
                    self.obst_angle = obstacle_angle
                    self.obst_distance = obstacle_dist
                    self.t_obst = time.time()
                else:
                    self.obst_distance = float('inf')


    # ======================================
    # MAIN LOGIC
    # ======================================
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

            state = self.state
            state_time = self.state_enter_time
            lidar_stop = self.lidar_stop
            lidar_stop_time = self.lidar_stop_time
            avoid_angle = self.avoid_angle

        if wp != self.active_wp_mode:
            return

        # -----------------------------------------------------------------
        # WHITE VISIBILITY 판단 (거리 + 새 timeout 기준 포함)
        # -----------------------------------------------------------------
        white_visible = (
            (now - t_a) <= MSG_TIMEOUT_SEC and
            (now - t_c) <= MSG_TIMEOUT_SEC and
            math.isfinite(img_a) and math.isfinite(img_d) and
            img_d <= DIST_VISIBLE_MAX and
            'white' in img_c
        )

        # -----------------------------------------------------------------
        # LiDAR STOP 최우선 로직
        # -----------------------------------------------------------------
        if (now - t_o) <= MSG_TIMEOUT_SEC and math.isfinite(obst_d):
            if obst_d <= LIDAR_STOP_DIST:
                with self.lock:
                    if not self.lidar_stop:
                        self.lidar_stop = True
                        self.lidar_stop_time = now

        if lidar_stop:
            elapsed = now - lidar_stop_time

            if elapsed < LIDAR_STOP_SEC:
                self.pub_candidate.publish(Float32(data=STOP_VALUE))
                return

            # 5초 지나면 장애물 유지 상관없이 무조건 WP단계 신호
            self.pub_candidate.publish(Float32(data=CANDIDATE_INVALID))
            return


        # -----------------------------------------------------------------
        # FSM
        # -----------------------------------------------------------------

        # === STATE -1: LiDAR 탐색 ===
        if state == -1:

            # 🔥 흰색 보이면 처음부터 State1 전환!
            if white_visible:
                with self.lock:
                    self.state = 1
                    self.state_enter_time = now
                self.pub_candidate.publish(Float32(data=float(img_a)))
                return

            # 탐색 20초 넘어가면 fallback
            if now - state_time >= LIDAR_SEARCH_SEC:
                with self.lock:
                    self.state = 0
                    self.state_enter_time = now
                self.pub_candidate.publish(Float32(data=CANDIDATE_INVALID))
                return

            # LiDAR로 탐색
            if math.isfinite(obst_d):
                self.pub_candidate.publish(Float32(data=float(obst_a)))
            else:
                self.pub_candidate.publish(Float32(data=CANDIDATE_INVALID))
            return


        # === STATE 0: fallback ===
        if state == 0:
            self.pub_candidate.publish(Float32(data=CANDIDATE_INVALID))
            if white_visible:
                with self.lock:
                    self.state = 1
                    self.state_enter_time = now
            return

        # === STATE 1: 흰색 접근 ===
        if state == 1:
            if not white_visible:
                with self.lock:
                    self.state = 0
                    self.state_enter_time = now
                self.pub_candidate.publish(Float32(data=CANDIDATE_INVALID))
                return

            self.pub_candidate.publish(Float32(data=float(img_a)))

            dist_valid = (now - t_d) <= MSG_TIMEOUT_SEC and math.isfinite(img_d)
            dist = img_d if dist_valid else None

            if dist is not None and dist <= BUOY_NEAR_DISTANCE + 0.4:
                use_d = dist if dist_valid else OFFSET_FALL_D
                offset_deg = math.degrees(math.atan(LATERAL_OFF_M / max(use_d, 0.01)))

                base = img_a
                avoid = wrap360(base + offset_deg) if base < 0 else wrap360(base - offset_deg)

                with self.lock:
                    self.avoid_angle = avoid
                    self.state = 2
                    self.state_enter_time = now
                return
            return

        # === STATE 2: 회피 ===
        if state == 2:
            self.pub_candidate.publish(Float32(data=float(avoid_angle)))
            if now - state_time >= AVOID_SECONDS:
                with self.lock:
                    self.state = 3
                    self.state_enter_time = now
            return

        # === STATE 3: 정지 ===
        if state == 3:
            self.pub_candidate.publish(Float32(data=CANDIDATE_INVALID))
            if now - state_time >= BUOY_HOLD_SECONDS:
                with self.lock:
                    self.state = 4
                    self.state_enter_time = now
            return

        # === STATE 4: 종료 ===
        if state == 4:
            self.pub_candidate.publish(Float32(data=CANDIDATE_INVALID))
            return


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()
    node = ShipTurn()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
