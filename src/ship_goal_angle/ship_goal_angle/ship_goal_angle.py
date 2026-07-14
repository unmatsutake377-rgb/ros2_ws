import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float32
from rclpy.qos import ReliabilityPolicy, QoSProfile


class ShipGoalAngle(Node):
    """ShipGoalAngle 노드 — IMU yaw와 목표 yaw의 오차를 계산해 `/yaw_error`로 퍼블리시.

    *오류 수정 버전: 들여쓰기·함수명 오타 등 문법 문제만 고쳤습니다.*
    """

    def __init__(self):
        super().__init__('ship_goal_angle')

        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)

        # Yaw 값 구독
        self.create_subscription(Float64, '/imu/yaw', self.yaw_callback, qos)

        # 목표 각도 구독
        self.create_subscription(Float32, '/north_goal_angle_tp', self.goal_angle_callback, qos)

        # Yaw 오차 퍼블리셔
        self.yaw_error_publisher = self.create_publisher(Float32, '/yaw_error', qos)

        self.current_yaw = 0.0
        self.goal_yaw = 0.0

    # ───────────────────────── 콜백 ─────────────────────────
    def yaw_callback(self, msg):
        self.current_yaw = msg.data

    def goal_angle_callback(self, msg):
        self.goal_yaw = msg.data

    # ───────────────────────── 보조 함수 ─────────────────────
    def normalize_angle_0_to_360(self, angle):
        """0‥360° 범위로 라핑"""
        return angle % 360

    # ───────────────────────── 메인 루프 ─────────────────────
    def publish_yaw_error(self):
        yaw_error = self.goal_yaw - self.current_yaw
        yaw_error = self.normalize_angle_0_to_360(yaw_error)

        yaw_error_msg = Float32()
        yaw_error_msg.data = yaw_error
        self.yaw_error_publisher.publish(yaw_error_msg)

        #self.get_logger().info(
        #    f"Current yaw: {self.current_yaw:.2f}°, Goal yaw: {self.goal_yaw:.2f}°, Yaw error: {yaw_error:.2f}°"
        #)


# ───────────────────────── 엔트리포인트 ─────────────────────

def main(args=None):
    rclpy.init(args=args)
    ship_goal_angle = ShipGoalAngle()

    # 주기적으로 Yaw 오차를 퍼블리시하는 타이머 생성 (0.5 s)
    ship_goal_angle.create_timer(0.5, ship_goal_angle.publish_yaw_error)

    rclpy.spin(ship_goal_angle)
    ship_goal_angle.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
