import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float32
from rclpy.qos import ReliabilityPolicy, QoSProfile


class ShipGoalAngle(Node):
    """IMU yaw 와 목표 yaw 의 오차를 계산해 `/yaw_error` 로 발행.

    ⚠️ 이 노드는 ship_direction 의 'IMU 살아있음' 신호원이기도 하다.
       ship_direction 은 `/yaw_error` 가 오는 것을 보고 IMU 체인 생존을 판단한다
       (watch.feed('imu') 가 yaw_error 콜백에 있음).
       그래서 입력이 묵으면 '마지막 값으로 계속 발행' 하지 않고 '발행을 멈춘다'.
       멈추면 ship_direction 이 감지해 감속/정지한다 — 그게 설계다.

    작년(71줄) 대비 고친 것:
      [1] 2Hz → period_sec(기본 0.05=20Hz). 작년엔 yaw_error 만 느려 제어 체인 병목.
      [2] current_yaw/goal_yaw 를 0.0 이 아니라 None 으로 시작. 둘 다 도착 전엔 발행 안 함
          (부팅 직후 yaw_error=0.0 을 발행해 ship_direction 이 '완벽 정렬, 직진' 오인하던 버그).
      [3] IMU 가 stale 하면 발행 중단 (아래 참고).
    토픽/타입은 그대로: /imu/yaw(Float64), /north_goal_angle_tp(Float32) → /yaw_error(Float32).
    """

    def __init__(self):
        super().__init__('ship_goal_angle')

        # ---- 파라미터 (config/ship_goal_angle.yaml) ----
        self.period_sec = float(self.declare_parameter('period_sec', 0.05).value)  # [1] 20Hz
        self.stale_sec = float(self.declare_parameter('stale_sec', 0.5).value)     # [3] IMU 신선도 한계

        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)

        # 토픽/타입 불변 (CLAUDE.md 1-3)
        self.create_subscription(Float64, '/imu/yaw', self.yaw_callback, qos)
        self.create_subscription(Float32, '/north_goal_angle_tp', self.goal_angle_callback, qos)
        self.yaw_error_publisher = self.create_publisher(Float32, '/yaw_error', qos)

        # [2] 0.0 이 아니라 None 으로 시작 — 값이 오기 전엔 발행하지 않는다
        self.current_yaw = None
        self.goal_yaw = None
        # [3] 마지막 수신 시각(단조시계, CLAUDE.md 1-5). None = 아직 못 받음
        self.last_yaw_t = None
        self.last_goal_t = None

        self.timer = self.create_timer(self.period_sec, self.publish_yaw_error)

        self.get_logger().info(
            f"ship_goal_angle 시작: {1.0 / self.period_sec:.0f}Hz "
            f"(period={self.period_sec}s), stale={self.stale_sec}s.\n"
            f"   IMU 가 stale 하면 /yaw_error 발행을 멈춘다(설계) → ship_direction 이 감속/정지."
        )

    # ───────────────────────── 콜백 ─────────────────────────
    def yaw_callback(self, msg):
        self.current_yaw = msg.data
        self.last_yaw_t = time.monotonic()

    def goal_angle_callback(self, msg):
        self.goal_yaw = msg.data
        self.last_goal_t = time.monotonic()

    # ───────────────────────── 보조 함수 ─────────────────────
    def normalize_angle_0_to_360(self, angle):
        """0‥360° 범위로 라핑 (yaw_error 계약 유지 — 값 의미 안 바꿈)"""
        return angle % 360

    # ───────────────────────── 메인 루프 ─────────────────────
    def publish_yaw_error(self):
        now = time.monotonic()

        # [2] 둘 다 도착하기 전엔 발행 안 함 (부팅 직후 yaw_error=0.0 발행 금지)
        if self.current_yaw is None or self.goal_yaw is None:
            return

        # [3] IMU 가 stale 하면 발행을 멈춘다.
        #     마지막 값으로 계속 발행하면 IMU 가 죽어도 ship_direction 이 살아있다고 착각한다
        #     (페일세이프가 눈이 먼다). 멈추면 ship_direction 이 /yaw_error 두절을 감지해 감속/정지.
        #     stale 은 IMU 에만 건다: 목표각(/north_goal_angle_tp)은 2Hz 라 0.5s 한계에 걸면
        #     매 주기 경계에서 거짓 정지가 난다(팀 최우선 우려). IMU 가 [3]의 생존 신호원이다.
        imu_age = now - self.last_yaw_t
        if imu_age > self.stale_sec:
            self.get_logger().warn(
                f"IMU stale ({imu_age:.2f}s > {self.stale_sec}s) → /yaw_error 발행 중단",
                throttle_duration_sec=1.0,
            )
            return

        yaw_error = self.normalize_angle_0_to_360(self.goal_yaw - self.current_yaw)
        self.yaw_error_publisher.publish(Float32(data=yaw_error))


# ───────────────────────── 엔트리포인트 ─────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ShipGoalAngle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
