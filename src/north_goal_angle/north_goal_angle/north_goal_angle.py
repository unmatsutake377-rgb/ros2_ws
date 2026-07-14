# north_goal_angle.py (FSM 전담)
import rclpy
import time
from math import atan2, cos, radians, sin, degrees
from geopy import distance
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import NavSatFix

waypoints = [
    [35.1862375, 128.5655118, 0, 3.0], #WP0 게이트 시작
    [35.1863642, 128.5657123, 1, 3.0], #WP1 게이트 끝
    [35.1868822, 128.5660465, 2, 3.0], #WP2 위치유지
    [35.1868638, 128.56582129999, 3, 3.0], #WP3 초록
    [35.1867763, 128.5658085, 3, 3.0], #WP4 빨강
    [35.1868645, 128.5656684, 3, 3.0], #WP5 하양
    [35.1866601, 128.5655597, 5, 50.0], #WP6 회피 시작
    [35.186396099999, 128.5653297, 5, 50.0], #WP7 회피끝
    [35.1859269, 128.5655428, 7, 60.0], #WP8 도킹 시작
    [35.1859269, 128.5655428, 7, 60.0], #WP9 도킹
    [35.1861956, 128.5660033, 8, 60.0] #WP 토너먼트 회피
]

CANDIDATE_INVALID = 20000.0
ARRIVE_RADIUS_M = 3.0
DEFAULT_TIMEOUT = 120.0
TURN_TIMEOUT = 90.0


class NorthGoalAngle(Node):
    def __init__(self):
        super().__init__('north_goal_angle')
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)

        self.pub_dist      = self.create_publisher(Float32, '/goal_distance', qos)
        self.pub_mode      = self.create_publisher(Int32,   '/wp_mode', qos)
        self.pub_candidate = self.create_publisher(Float32, '/candidate_angle', qos)
        self.pub_bearing   = self.create_publisher(Float32, '/north_goal_angle_tp', qos)
        
        # ⭐ 추가 : 남은 시간 퍼블리셔
        self.pub_remain    = self.create_publisher(Float32, '/wp_remaining_time', qos)

        self.create_subscription(NavSatFix, '/ublox_gps_node/fix', self.gps_cb, qos)

        self.lat, self.lon = 0.0, 0.0
        self.wp_idx = 0
        self.t_start = None
        self.wp_enter_time = None

        self.create_timer(0.5, self.timer_cb)


    def gps_cb(self, msg):
        self.lat, self.lon = msg.latitude, msg.longitude


    def timer_cb(self):
        if self.wp_idx >= len(waypoints):
            return

        wp_lat, wp_lon, wp_mode, dwell = waypoints[self.wp_idx]

        dist = calc_dist(self.lat, self.lon, wp_lat, wp_lon)
        self.pub_dist.publish(Float32(data=dist))

        bearing = calc_angle(self.lat, self.lon, wp_lat, wp_lon)
        self.pub_bearing.publish(Float32(data=bearing))

        self.pub_mode.publish(Int32(data=wp_mode))

        if wp_mode == 7:
            self.pub_candidate.publish(Float32(data=CANDIDATE_INVALID))

        now = time.time()

        if self.wp_enter_time is None:
            self.wp_enter_time = now

        # WP별 타임아웃 정책
        timeout = None
        if self.wp_idx in (3, 4, 5):
            timeout = TURN_TIMEOUT
        elif self.wp_idx <= 6:
            timeout = DEFAULT_TIMEOUT

        # ⭐ 타임아웃 남은 시간 퍼블리시
        if timeout is not None:
            remain = max(timeout - (now - self.wp_enter_time), 0.0)
            self.pub_remain.publish(Float32(data=remain))  # ★ 퍼블리시

            if remain <= 0:
                self.get_logger().warn(f"🕒 WP{self.wp_idx} 시간 초과 → 다음 WP 이동")
                self.wp_idx += 1
                self.t_start = None
                self.wp_enter_time = None
                return

        # 정상 도착 확인
        if dist < ARRIVE_RADIUS_M:
            if self.t_start is None:
                self.t_start = now
            elif (now - self.t_start) >= dwell:
                self.get_logger().info(f"✔ WP{self.wp_idx} 완료 → 다음 WP 이동")
                self.wp_idx += 1
                self.t_start = None
                self.wp_enter_time = None
        else:
            self.t_start = None


def calc_angle(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
    dlon = lon2 - lon1
    x = sin(dlon) * cos(lat2)
    y = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(dlon)
    return (degrees(atan2(x, y)) + 360) % 360

def calc_dist(lat1, lon1, lat2, lon2):
    return distance.distance((lat1, lon1), (lat2, lon2)).m


def main(args=None):
    rclpy.init(args=args)
    node = NorthGoalAngle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
