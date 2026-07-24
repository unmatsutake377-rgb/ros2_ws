import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32

# /desired_angle · /candidate_angle 특수 신호 상수 (CLAUDE.md 3-9 상수표)
# ★ 작년엔 이 값들이 6개 파일에 흩어져 각자 정의돼 어긋났다. 여기 값은 그 표를 따른다.
SPIN_RIGHT = 5000.0         # 우선회 (ship_dock RIGHT_SPIN 계승)
SPIN_LEFT = 6000.0          # 좌선회
CANDIDATE_INVALID = 20000.0 # 미션 없음 → fallback 전진
STOP_HOLD = 50000.0         # 정지/대기


class MotorController(Node):
    """조향각(/desired_angle)을 모터 PWM(Motor_run)으로 변환하는 체인의 끝단.

    PWM 규약: Motor_run = pwm_r*10000 + pwm_l.  1500=정지, <1500=전진, >1500=후진.

    2단계에서 이은 것 (CLAUDE.md 3-2 / 4장):
      · 감속 복구: /obstacle_distance_array(전방 최근접 거리)를 구독해, 장애물이 가까우면
        '전진 명령'(분기 2,4,6)을 중립(1500) 쪽으로 당겨 감속한다. SPIN(3)·후진(5)·STOP(1)은
        제외 — 다가가지 않으므로 감속 대상이 아니다. 작년엔 이 노드가 /desired_angle 하나만
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
        # 명령 워치독: 0.5s (3단계에서 3.5 → 0.5 로 조임).
        # 예전엔 ship_direction 이 scan_cb 에서만 발행해서 LiDAR 가 잠깐 끊기면 /desired_angle 도
        # 멈췄다 → 짧게 잡으면 ship_direction 자체 페일세이프를 덮어써 배를 죽였다(그래서 3.5).
        # 지금은 제어 루프가 고정주기 타이머라 ship_direction 이 살아있는 한 항상 발행한다
        # (LiDAR 4초 끊겨도 STOP_HOLD 를 40회 계속 발행하는 것으로 검증).
        # → /desired_angle 침묵 = 'ship_direction 사망' 이다(LiDAR 끊김이 아니라). 0.5 로 조인다.
        self.cmd_timeout_sec = float(self.declare_parameter("cmd_timeout_sec", 0.5).value)

        # ---- 페일세이프 속도 상한 (/failsafe_level 구독, ship_direction 이 발행) ----
        # /desired_angle 은 '각도'라 속도를 담을 수 없다 → 속도는 여기서 건다.
        #   레벨0 100% / 레벨1(경고) 70% / 레벨2(정지)는 STOP_HOLD 가 각도로 오므로 분기 (1) 이 처리.
        # ⚠️ 경고 감속의 효과는 아직 미측정. 1.0 으로 두면 꺼진다(효과 없으면 뺀다, TTC 때처럼).
        self.fs_l1_speed = float(self.declare_parameter("failsafe_l1_speed", 0.7).value)

        # ---- 조향/추력 파라미터 (배 2척 A/B 크기·추력 다름 → config, CLAUDE.md 1-4) ----
        self.base_pwm = int(self.declare_parameter("base_pwm", 1360).value)         # ⚠️ 실측 필요(순항 추력)
        self.max_diff = int(self.declare_parameter("max_diff", 100).value)          # ⚠️ 실측 필요(최대 차동)
        self.reverse_pwm = int(self.declare_parameter("reverse_pwm", 1590).value)   # ⚠️ 실측 필요(후진 추력)
        # SPIN(제자리 선회): 중심 PWM 과 차동을 따로 둔다.
        #   spin_forward_pwm=1500 → 1400/1600 = 순 추력 0 = 진짜 제자리 선회.
        #   =1360 이면 1260/1460 = 작년 방식(순항 속도로 원, 반경 2.1m → 도크에 부딪힘). 되돌릴 수 있는 노브.
        self.spin_forward_pwm = int(self.declare_parameter("spin_forward_pwm", 1500).value)
        self.spin_diff = int(self.declare_parameter("spin_diff", 100).value)
        # 조향 좌/우 반전 노브: 모터 배선이 반대면 조향(4)·SPIN(3)이 함께 반대로 나온다(원인 하나).
        # ★ 물 위 첫 시험에서 확정. 지금 추측 금지(배가 아직 없음). CLAUDE.md §8 "가장 흔한 사고".
        self.steer_invert = bool(self.declare_parameter("steer_invert", False).value)
        self.max_angle = 81   # -1~161도 전체를 조향에 반영 (매핑 상수, 보트 무관)

        # ---- 구독/발행 (토픽·타입 불변, CLAUDE.md 1-3) ----
        self.sub_angle = self.create_subscription(Float32, "/desired_angle", self.angle_callback, 2)
        self.sub_obstacle = self.create_subscription(
            Float32MultiArray, "/obstacle_distance_array", self.obstacle_callback, 10)
        self.sub_failsafe = self.create_subscription(
            Int32, "/failsafe_level", self.failsafe_callback, 10)
        self.pub_motor = self.create_publisher(Int32, "Motor_run", 2)

        # N2: 후진 상태를 밖에 알린다 (신규 토픽 — 기존 계약 불변, CLAUDE.md 1-3).
        #   ssf_heading/yaw_mux 가 COG 오프셋 표본을 모을지 말지 정하는 데 쓴다.
        #   후진 중엔 COG 가 뱃머리와 180° 뒤집혀서, 후진만 지속되면 추정이
        #   R=1.0 으로 **정확히 180° 틀린 값에 수렴**한다(검산됨). 신뢰도로는 못 막는다.
        #   ⚠️ 판정을 여기서 하는 이유: PWM 규약(1500 기준, <전진 >후진)의 소유자가 이 노드다.
        #      구독자가 Motor_run 을 디코드하게 두면 규약이 두 곳에 복제된다.
        self.pub_reverse = self.create_publisher(Bool, "/motor_reverse", 2)

        # ---- 상태 ----
        self.desired_angle = None          # 부팅 중립: 첫 명령 전엔 None → 전진 안 함
        self.last_cmd_t = None             # /desired_angle 마지막 수신 시각(monotonic, CLAUDE.md 1-5)
        self.nearest_dist = float("inf")   # 전방 최근접 장애물 거리(없으면 inf → 감속 안 함)
        self.failsafe_level = 0            # ship_direction 이 발행. 못 받으면 0(상한 없음)

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info(
            f"motor_control 시작: 20Hz. 감속 slow_start={self.slow_start_dist}m/"
            f"min_ratio={self.min_speed_ratio}, 명령 워치독={self.cmd_timeout_sec}s, "
            f"steer_invert={self.steer_invert}.\n"
            f"   SPIN(5000=우/6000=좌) 중심={self.spin_forward_pwm}(1500=제자리). "
            f"첫 명령 전/끊김 시 중립. PWM: <1500 전진, >1500 후진."
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

    def failsafe_callback(self, msg):
        self.failsafe_level = int(msg.data)

    # ───────────────────────── 보조 ─────────────────────────
    def linear_diff(self, offset):
        off = max(-self.max_angle, min(self.max_angle, offset))
        diff = (abs(off) / self.max_angle) * self.max_diff
        return int(diff)

    def apply_steer(self, signed_diff, center=None):
        """center 중심 좌우 차동. signed_diff>0 = '왼쪽 패턴'(오른쪽 모터가 더 전진).
        center 기본은 base_pwm(전진 조향), SPIN 은 spin_forward_pwm 을 넘긴다.
        steer_invert=True 면 좌/우를 통째로 뒤집는다 — 조향(4)·SPIN(3) 공통(배선 반대 대비)."""
        c = self.base_pwm if center is None else center
        s = -signed_diff if self.steer_invert else signed_diff
        return c - s, c + s

    def speed_ratio(self):
        """전방 최근접 거리 → 속도비 [min_speed_ratio, 1.0]. dist>=slow_start_dist 면 1.0."""
        d = self.nearest_dist
        if not math.isfinite(d) or d >= self.slow_start_dist:
            return 1.0
        frac = max(0.0, d) / self.slow_start_dist
        return self.min_speed_ratio + (1.0 - self.min_speed_ratio) * frac

    def failsafe_speed_cap(self):
        """페일세이프 레벨 → 속도 상한.
        레벨2(정지)는 ship_direction 이 STOP_HOLD 를 각도로 보내므로 분기 (1) 이 처리한다."""
        if self.failsafe_level == 1:
            return self.fs_l1_speed
        return 1.0

    def apply_slow(self, pwm, ratio):
        """PWM 을 중립(1500) 쪽으로 ratio 만큼 당긴다.
        '전진 명령' 분기(2,4,6)에서만 호출한다 — 감속은 '앞으로 가며 장애물에 다가가니 천천히'.
        제자리 선회(3)·후진(5)·정지(1)는 다가가지 않으므로 감속 대상이 아니다.
        (과거 '전진 성분(pwm<1500)' 규칙은 SPIN 의 1400 쪽만 깎아 배가 뒤로 밀리며 돌게 했다.)"""
        return int(round(1500 + (pwm - 1500) * ratio))

    def publish_pwm(self, pwm_r, pwm_l):
        msg = Int32()
        msg.data = pwm_r * 10000 + pwm_l
        self.pub_motor.publish(msg)

        # 후진 = 좌우 평균이 중립보다 뒤 (순 추력 기준).
        #   분기가 아니라 '실제로 나가는 PWM' 으로 판정한다 → 워치독 중립·감속·SPIN 까지
        #   자동으로 맞는다. 분기로 판정하면 나중에 분기를 늘릴 때 여기가 조용히 낡는다.
        #   예: 1500/1500 중립=False, 1400/1400 전진=False, 1590/1590 후진=True,
        #       1400/1600 제자리선회=False(순 추력 0 — 병진이 없으니 COG 도 없다)
        rev = Bool()
        rev.data = ((pwm_r + pwm_l) / 2.0) > 1500.0
        self.pub_reverse.publish(rev)

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
        slow_ok = False   # 감속은 '전진 명령'(2,4,6)에만. SPIN(3)·후진(5)·STOP(1)은 제외.

        # (1) STOP
        if angle >= STOP_HOLD:
            pwm_r = 1500
            pwm_l = 1500

        # (2) fallback 전진 (미션 없음)
        elif CANDIDATE_INVALID <= angle < STOP_HOLD:
            pwm_r = self.base_pwm
            pwm_l = self.base_pwm
            slow_ok = True

        # (3a) SPIN_RIGHT(5000) → 우선회 (제자리, 중심=spin_forward_pwm).
        #      작년 버그 2개: ① 왼쪽으로 돌았고 ② base_pwm 중심이라 순항 속도로 원을 그려 도크에 부딪혔다.
        elif SPIN_RIGHT <= angle < SPIN_LEFT:
            pwm_r, pwm_l = self.apply_steer(-self.spin_diff, center=self.spin_forward_pwm)  # 오른쪽 패턴

        # (3b) SPIN_LEFT(6000) → 좌선회 (제자리)
        elif SPIN_LEFT <= angle < CANDIDATE_INVALID:
            pwm_r, pwm_l = self.apply_steer(+self.spin_diff, center=self.spin_forward_pwm)  # 왼쪽 패턴

        # (4) 정상 조향 구간
        elif -1.0 <= angle <= 161.0:
            offset = angle - 80.0   # 중앙(정면) 80도 기준
            diff = self.linear_diff(offset)
            # offset>0 = 왼쪽 조향(+), offset<=0 = 오른쪽 조향(-)
            pwm_r, pwm_l = self.apply_steer(diff if offset > 0 else -diff)
            slow_ok = True

        # (5) 후진 구간
        elif 161.0 < angle < SPIN_RIGHT:
            pwm_r = self.reverse_pwm
            pwm_l = self.reverse_pwm

        # (6) 기본값: 직진 유지
        else:
            pwm_r = self.base_pwm
            pwm_l = self.base_pwm
            slow_ok = True

        # (감속) '전진 명령'일 때만. 두 가지 이유로 느려진다 — 더 보수적인 쪽(min)을 쓴다:
        #   ① 장애물 근접 (speed_ratio)
        #   ② 페일세이프 레벨 (failsafe_speed_cap) — 센서가 묵었으니 속도를 줄여 오차를 줄인다
        # ★ 반드시 slow_ok 뒤에 둔다. SPIN(제자리 1400/1600)에 감속이 걸리면 전진 쪽만 깎여
        #   배가 뒤로 밀리며 돈다(이미 겪은 버그). 후진·STOP 도 같은 이유로 제외.
        ratio = min(self.speed_ratio(), self.failsafe_speed_cap())
        if slow_ok and ratio < 1.0:
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
