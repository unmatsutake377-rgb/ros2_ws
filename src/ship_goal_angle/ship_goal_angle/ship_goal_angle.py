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
      [1] 발행 주기를 period_sec 파라미터로 뺌(기본 0.05=20Hz).
          ※ 검토 하네스 측정: 주기 2~50Hz 결과가 완전히 동일했다. "2Hz 병목" 은 근거 없음(철회).
             20Hz 를 유지하는 이유는 비용이 사실상 0이고 무해하기 때문이지 성능 개선이 아니다.
             1단계의 실제 값어치는 아래 [2],[3] 이다.
      [2] current_yaw/goal_yaw 를 0.0 이 아니라 None 으로 시작. 둘 다 도착 전엔 발행 안 함
          (부팅 직후 yaw_error=0.0 을 발행해 ship_direction 이 '완벽 정렬, 직진' 오인하던 버그).
      [3] 입력이 stale 하면 발행 중단 (아래 참고). IMU 와 목표각을 각각 다른 한계로 감시:
          IMU 는 stale_sec(0.5s), 목표각은 goal_stale_sec(2.0s=발행주기 4배).
    토픽/타입은 그대로: /imu/yaw(Float64), /north_goal_angle_tp(Float32) → /yaw_error(Float32).
    """

    def __init__(self):
        super().__init__('ship_goal_angle')

        # ---- 파라미터 (config/ship_goal_angle.yaml) ----
        self.period_sec = float(self.declare_parameter('period_sec', 0.05).value)      # [1] 20Hz
        self.stale_sec = float(self.declare_parameter('stale_sec', 0.5).value)         # [3] IMU 신선도 한계
        # [3b] 목표각(/north_goal_angle_tp) 신선도 한계. 발행 주기 0.5s 의 4배.
        #      IMU 처럼 0.5s 로 걸면 2Hz 발행자라 거짓 정지가 나므로 넉넉히.
        #      north_goal_angle 이 통째로 죽으면 목표각이 얼어붙어 배가 고정 방위로 영원히
        #      직진(경기장 이탈)한다 → 이 값으로 '진짜 죽음' 을 잡는다.
        self.goal_stale_sec = float(self.declare_parameter('goal_stale_sec', 2.0).value)

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
            f"(period={self.period_sec}s), IMU stale={self.stale_sec}s, "
            f"목표각 stale={self.goal_stale_sec}s.\n"
            f"   입력이 stale 하면 /yaw_error 발행을 멈춘다(설계) → ship_direction 이 감속/정지."
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

        # [3] 입력이 stale 하면 발행을 멈춘다.
        #     마지막 값으로 계속 발행하면 소스가 죽어도 ship_direction 이 살아있다고 착각한다
        #     (페일세이프가 눈이 먼다). 멈추면 ship_direction 이 /yaw_error 두절을 감지해 감속/정지.
        #     IMU 와 목표각은 발행 주기가 달라 stale 한계를 따로 둔다:
        #       - IMU(/imu/yaw)          : 빠른 센서 → stale_sec(기본 0.5s)
        #       - 목표각(/north_goal_angle_tp): 2Hz(0.5s) 발행 → goal_stale_sec(기본 2.0s=4배).
        #         짧게 걸면 매 주기 경계에서 거짓 정지가 난다(팀 최우선 우려). 넉넉히 걸어
        #         north_goal_angle 이 '통째로 죽은' 경우만 잡는다(목표각 얼어붙어 고정 방위 직진 → 이탈).
        imu_age = now - self.last_yaw_t
        if imu_age > self.stale_sec:
            self.get_logger().warn(
                f"IMU stale ({imu_age:.2f}s > {self.stale_sec}s) → /yaw_error 발행 중단",
                throttle_duration_sec=1.0,
            )
            return

        goal_age = now - self.last_goal_t
        if goal_age > self.goal_stale_sec:
            self.get_logger().warn(
                f"목표각 stale ({goal_age:.2f}s > {self.goal_stale_sec}s) → /yaw_error 발행 중단 "
                f"(north_goal_angle 두절 의심)",
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
