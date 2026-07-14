import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32


class MotorController(Node):
    def __init__(self):
        super().__init__("motor_control")

        self.sub_angle = self.create_subscription(Float32, "/desired_angle", self.angle_callback, 2)
        self.pub_motor = self.create_publisher(Int32, "Motor_run", 2)

        self.desired_angle = 80.0
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.base_pwm = 1360
        self.max_angle = 81   # -1~161도 전체를 조향에 반영
        self.max_diff = 100

        # 부드러운 왼쪽 선회
        self.turn_offset = -60

    def angle_callback(self, msg):
        self.desired_angle = msg.data

    def linear_diff(self, offset):
        off = max(-self.max_angle, min(self.max_angle, offset))
        diff = (abs(off) / self.max_angle) * self.max_diff
        return int(diff)

    def timer_callback(self):
        angle = self.desired_angle

        # (1) STOP
        if angle >= 50000:
            pwm_r = 1500
            pwm_l = 1500

        # (2) fallback 전진
        elif 20000 <= angle < 50000:
            pwm_r = self.base_pwm
            pwm_l = self.base_pwm

        # (3) 느린 왼쪽 선회 (좌측 방향으로 고정 오프셋)
        elif 5000 <= angle < 20000:
            diff = self.linear_diff(self.turn_offset)
            pwm_r = self.base_pwm - diff
            pwm_l = self.base_pwm + diff

        # (4) 정상 조향 구간
        elif -1.0 <= angle <= 161.0:
            offset = angle - 80.0   # 중앙(정면) 80도 기준
            diff = self.linear_diff(offset)

            if offset > 0:
                # 왼쪽 조향
                pwm_r = self.base_pwm - diff
                pwm_l = self.base_pwm + diff
            else:
                # 오른쪽 조향
                pwm_r = self.base_pwm + diff
                pwm_l = self.base_pwm - diff

        # (5) 후진 구간
        elif 161.0 < angle < 5000.0:
            pwm_r = 1590
            pwm_l = 1590

        # (6) 기본값: 직진 유지
        else:
            pwm_r = self.base_pwm
            pwm_l = self.base_pwm

        msg = Int32()
        msg.data = pwm_r * 10000 + pwm_l
        self.pub_motor.publish(msg)
        #self.get_logger().info(f"[angle={angle}] R={pwm_r}, L={pwm_l}")


def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

