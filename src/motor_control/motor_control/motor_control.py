import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Int32


class MotorController(Node):
    """조향각(/desired_angle)을 모터 PWM(Motor_run)으로 변환하는 체인의 끝단.

    PWM 규약: Motor_run = pwm_r*10000 + pwm_l.  1500=정지, <1500=전진, >1500=후진.

    2단계에서 이은 것 (CLAUDE.md 3-2 / 4장):
      · 감속 복구: /obstacle_distance_array(전방 최근접 거리)를 구독해, 장애물이 가까우면
        전진 성분을 중립(1500) 쪽으로 당겨 감속한다. 작년엔 이 노드가 /desired_angle 하나만
        구독해 '장애물이 코앞이어도 감속하지 않았다'.
      · 명령 워치독: /desired_angle 이 cmd_timeout_sec 넘게 끊기면 중립(1500/1500).
        + 부팅 직후 첫 명령이 오기 전에도 중립 (명령 없이 전진 금지).

    ※ TTC 비상제동은 넣지 않는다. /obstacle_distance_array 최소거리는 시간필터 없는 생값이라
      미분(접근속도)이 노이즈를 증폭 → 물보라 반사 하나로 급정지. 시뮬상 이득 0/위험 막대라 폐기.
      감속(점진적)은 노이즈에 강하므로 그것만 둔다.
    """

    def __init__(self):
        super().__init__("motor_control")

        # ---- 파라미터 (config/motor_control.yaml, CLAUDE.md 1-4) ----
        self.slow_start_dist = float(self.declare_parameter("slow_start_dist", 1.2).value)  # 감속 시작 거리(m)
        self.min_speed_ratio = float(self.declare_parameter("min_speed_ratio", 0.7).value)  # 최저 속도비
        # 명령 워치독: 3.5s. 지금 ship_direction 은 scan_cb 에서만 /desired_angle 을 발행해서
        # LiDAR 가 잠깐 끊기면 /desired_angle 도 멈춘다. 짧게(0.5s) 잡으면 ship_direction 자체
        # 페일세이프(0.7s 감속/3.0s 정지)를 덮어써 배를 죽인다. 3.5>3.0 이라 안 건드림.
        # ⚠️ 3단계에서 ship_direction 제어루프를 고정주기 타이머로 분리하면 0.5 로 조인다.
        self.cmd_timeout_sec = float(self.declare_parameter("cmd_timeout_sec", 3.5).value)

        # ---- 기존 조향 상수 (작년 값 유지 — 보트별 파라미터화는 별도 범위) ----
        self.base_pwm = 1360
        self.max_angle = 81   # -1~161도 전체를 조향에 반영
        self.max_diff = 100
        self.turn_offset = -60  # 부드러운 왼쪽 선회

        # ---- 구독/발행 (토픽·타입 불변, CLAUDE.md 1-3) ----
        self.sub_angle = self.create_subscription(Float32, "/desired_angle", self.angle_callback, 2)
        self.sub_obstacle = self.create_subscription(
            Float32MultiArray, "/obstacle_distance_array", self.obstacle_callback, 10)
        self.pub_motor = self.create_publisher(Int32, "Motor_run", 2)

        # ---- 상태 ----
        self.desired_angle = None          # 부팅 중립: 첫 명령 전엔 None → 전진 안 함
        self.last_cmd_t = None             # /desired_angle 마지막 수신 시각(monotonic, CLAUDE.md 1-5)
        self.nearest_dist = float("inf")   # 전방 최근접 장애물 거리(없으면 inf → 감속 안 함)

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info(
            f"motor_control 시작: 20Hz. 감속 slow_start={self.slow_start_dist}m/"
            f"min_ratio={self.min_speed_ratio}, 명령 워치독={self.cmd_timeout_sec}s.\n"
            f"   첫 명령 전/명령 끊김 시 중립(1500/1500). PWM: <1500 전진, >1500 후진."
        )

    # ───────────────────────── 콜백 ─────────────────────────
    def angle_callback(self, msg):
        self.desired_angle = msg.data
        self.last_cmd_t = time.monotonic()

    def obstacle_callback(self, msg):
        # ship_direction 계약: data = [closest_distance(m), closest_angle(deg)] 또는 [inf, nan].
        # 거리는 data[0] 하나뿐(data[1]은 각도).
        if len(msg.data) >= 1 and math.isfinite(msg.data[0]):
            self.nearest_dist = msg.data[0]
        else:
            self.nearest_dist = float("inf")

    # ───────────────────────── 보조 ─────────────────────────
    def linear_diff(self, offset):
        off = max(-self.max_angle, min(self.max_angle, offset))
        diff = (abs(off) / self.max_angle) * self.max_diff
        return int(diff)

    def speed_ratio(self):
        """전방 최근접 거리 → 속도비 [min_speed_ratio, 1.0]. dist>=slow_start_dist 면 1.0."""
        d = self.nearest_dist
        if not math.isfinite(d) or d >= self.slow_start_dist:
            return 1.0
        frac = max(0.0, d) / self.slow_start_dist
        return self.min_speed_ratio + (1.0 - self.min_speed_ratio) * frac

    def apply_slow(self, pwm, ratio):
        """전진 성분(pwm<1500)만 중립(1500) 쪽으로 당긴다.
        후진(>1500)/정지(1500)는 안 건드림 — 전방 장애물이 후진을 늦추면 안 되니까."""
        if pwm < 1500:
            return int(round(1500 + (pwm - 1500) * ratio))
        return pwm

    def publish_pwm(self, pwm_r, pwm_l):
        msg = Int32()
        msg.data = pwm_r * 10000 + pwm_l
        self.pub_motor.publish(msg)

    # ───────────────────────── 메인 루프 ─────────────────────
    def timer_callback(self):
        now = time.monotonic()

        # (워치독) 부팅 직후 첫 명령 전, 또는 명령이 끊기면 중립.
        if self.desired_angle is None:
            self.publish_pwm(1500, 1500)   # 명령 없이 전진 금지
            return
        cmd_age = now - self.last_cmd_t
        if cmd_age > self.cmd_timeout_sec:
            self.get_logger().warn(
                f"/desired_angle stale ({cmd_age:.2f}s > {self.cmd_timeout_sec}s) → 중립(1500/1500)",
                throttle_duration_sec=1.0,
            )
            self.publish_pwm(1500, 1500)
            return

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

        # (감속 복구) 장애물 근접 시 전진 성분을 중립 쪽으로 당겨 감속
        ratio = self.speed_ratio()
        if ratio < 1.0:
            pwm_r = self.apply_slow(pwm_r, ratio)
            pwm_l = self.apply_slow(pwm_l, ratio)

        self.publish_pwm(pwm_r, pwm_l)


def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
