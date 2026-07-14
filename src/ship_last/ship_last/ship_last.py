import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, Int32


LAST_VALUE = 20000.0  # 기본 퍼블리시 값


class ShipLast(Node):
    def __init__(self):
        super().__init__('ship_last')

        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)

        # ───────────────────────────────
        # 파라미터: 이 노드가 활성화될 기본 WP 모드 하나만 설정
        self.declare_parameter("active_wp_mode", 0)
        self.active_wp_mode = self.get_parameter("active_wp_mode").value

        # 여러 WP 모드를 자동 활성화로 허용
        self.allowed_wp_modes = (0, 6, 8)

        # 퍼블리셔
        self.pub_candidate = self.create_publisher(Float32, '/candidate_angle', qos)

        # 구독
        self.create_subscription(Int32, '/wp_mode', self.wp_mode_cb, qos)

        # 내부 상태
        self.wp_mode = -1

        # 타이머
        self.create_timer(0.5, self.timer_cb)


    # ── wp_mode 업데이트 ─────────────────────────────
    def wp_mode_cb(self, msg):
        self.wp_mode = msg.data


    # ── 주기 콜백 ─────────────────────────────────────
    def timer_cb(self):
        # 허용된 WP 모드 외에서는 동작 안 함
        if self.wp_mode not in self.allowed_wp_modes:
            return

        # 활성 WP에서는 20000 계속 퍼블리시
        self.pub_candidate.publish(Float32(data=LAST_VALUE))
        # self.get_logger().info(f"🟡 WP{self.wp_mode} → candidate_angle={LAST_VALUE}")


def main(args=None):
    rclpy.init(args=args)
    node = ShipLast()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
