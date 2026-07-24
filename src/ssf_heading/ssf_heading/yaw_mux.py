"""
yaw_mux — /imu/yaw 의 **단독 발행자**.

왜 있나 (CLAUDE.md 3-5):
  작년엔 iahrs_driver 가 /imu/yaw 를 직접 냈는데, 그 값이 절대방위가 아니었다.
    ② 부팅 시점 뱃머리를 0 으로 만들어 상대각이 됐고 (north_goal_angle 은 절대방위를 계산한다)
    ③ GPS COG 가 유효하면 IMU 를 통째로 덮어썼다 (COG 는 뱃머리가 아니다. 정지 시 노이즈다)
  → 드라이버는 보정 없는 상대 yaw 를 /imu/yaw_raw 로만 내고, 절대방위 합성은 여기서 한다.

🚨 발행자는 하나뿐이어야 한다.
  드라이버가 /imu/yaw 를 계속 내는 채로 이 노드를 띄우면 **한 토픽에 발행자 2개**가 되어
  두 값이 번갈아 나온다. 에러는 안 난다 — 이 프로젝트가 반복해 당한 침묵 실패다.
  (도킹 mode 9 vs 7, /gate_pass_count vs /gates_passed 와 같은 유형)
  드라이버 쪽 yaw_topic 파라미터를 반드시 /imu/yaw_raw 로 두고 띄울 것. launch 에 배선돼 있다.

동작:
  - 고정주기 타이머로 발행한다 (3단계에서 확립한 패턴 — 입력 콜백에 매달지 않는다)
  - 입력이 stale 하거나 소스가 미구현이면 **발행하지 않는다.**
    틀린 방위 하나가 배를 정반대로 보낸다. 소비자는 이미 침묵을 처리한다:
      ship_goal_angle  → /yaw_error 발행 중단 → ship_direction 이 감속/정지
      north_goal_angle → geofence 침묵(빈 배열)
  - /heading_status(String) 로 상태를 밖에 알린다 (healthcheck·blackbox 관찰용, 제어엔 안 쓴다)
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64, String
from geometry_msgs.msg import TwistWithCovarianceStamped

from ssf_heading.heading_logic import (
    ALL_SOURCES, SRC_IMU_RELATIVE, ST_OK,
    COGOffsetEstimator, HeadingMux, cog_from_velocity,
)


class YawMux(Node):
    def __init__(self):
        super().__init__('yaw_mux')

        # ---- 파라미터 (config/ssf_heading.yaml) ----
        source = str(self.declare_parameter('heading_source', SRC_IMU_RELATIVE).value)
        if source not in ALL_SOURCES:
            # 조용히 기본값으로 떨어지지 않는다. 오타 하나가 엉뚱한 소스를 쓰게 두면 안 된다.
            raise RuntimeError(
                f"heading_source '{source}' 를 모른다. 가능한 값: {list(ALL_SOURCES)}")

        self.period_sec = float(self.declare_parameter('period_sec', 0.02).value)   # 50Hz
        self.stale_sec = float(self.declare_parameter('stale_sec', 0.5).value)

        # ⚠️ 실측 필요 — 전부 벤치에서 확정한다. 지금 추측한 값이 아니다.
        #    배를 정북으로 놓고 /imu/yaw 가 0 을 읽을 때까지 mount_offset_deg 를 맞춘다.
        #    부호가 틀리면 배가 정반대로 간다 (CLAUDE.md 3-5).
        invert_yaw = bool(self.declare_parameter('invert_yaw', False).value)
        mount_offset_deg = float(self.declare_parameter('mount_offset_deg', 0.0).value)
        declination_deg = float(
            self.declare_parameter('magnetic_declination_deg', 0.0).value)

        raw_topic = str(self.declare_parameter('raw_yaw_topic', '/imu/yaw_raw').value)
        out_topic = str(self.declare_parameter('yaw_topic', '/imu/yaw').value)

        # ---- N2: 옵션 B (COG 오프셋 추정) ----
        gps_vel_topic = str(self.declare_parameter(
            'gps_vel_topic', '/ublox_gps_node/fix_velocity').value)
        # 🚨 프레임을 틀리면 헤딩이 90° 돌거나 좌우가 뒤집힌다. 벤치 확인 항목.
        #    배를 정북으로 천천히 전진시키고 /heading_status 의 cog 가 0 근처인지 본다.
        self.gps_vel_frame = str(self.declare_parameter('gps_vel_frame', 'enu').value)
        gps_rate_hz = float(self.declare_parameter('gps_rate_hz', 5.0).value)

        self.mux = HeadingMux(
            source,
            invert_yaw=invert_yaw,
            mount_offset_deg=mount_offset_deg,
            declination_deg=declination_deg,
            stale_sec=self.stale_sec,
            estimator=COGOffsetEstimator(
                min_speed_mps=float(
                    self.declare_parameter('cog_min_speed_mps', 0.8).value),
                max_turn_rate_dps=float(
                    self.declare_parameter('cog_max_turn_rate_dps', 8.0).value),
                min_samples=float(
                    self.declare_parameter('cog_min_samples', 30.0).value),
                min_resultant=float(
                    self.declare_parameter('cog_min_resultant', 0.9).value),
                half_life_sec=float(
                    self.declare_parameter('cog_half_life_sec', 60.0).value),
                # 후진 상태를 모르면 표본을 안 모은다 (모르면 안 모은다).
                # ⚠️ motor_control 이 안 떠 있으면 이 게이트가 영영 안 와서
                #    cog_offset 이 **영구히 수렴하지 않는다.** 그 상태를 조용히 두지 않으려고
                #    /heading_status 에 rej=no_reverse_gate 로 노출하고 주기 경고를 낸다.
                #    헤딩만 벤치할 때는 이 파라미터를 false 로.
                require_reverse_gate=bool(
                    self.declare_parameter('cog_require_reverse_gate', True).value),
            ),
        )

        # 🚨 감쇠가 있으면 표본수가 포화한다: n_max = 1/(1-0.5^(dt/half_life)).
        #    min_samples 를 그 위로 잡으면 **영원히 수렴하지 않는다 — 에러도 없이.**
        #    부팅 때 잡아서 시끄럽게 알린다(조용히 안 죽는 것을 막는다).
        if gps_rate_hz > 0.0 and not self.mux.estimator.min_samples_reachable(
                1.0 / gps_rate_hz):
            self.get_logger().error(
                f"🚨 cog_min_samples({self.mux.estimator.min_samples:.0f}) 가 "
                f"GPS {gps_rate_hz:.1f}Hz + half_life {self.mux.estimator.half_life_sec:.0f}s "
                f"의 포화상한을 넘는다 → cog_offset 은 영원히 수렴하지 않는다. "
                f"cog_min_samples 를 줄이거나 cog_half_life_sec 를 늘려라.")

        # 🚨 벤치 편의 스위치가 대회 설정에 남는 사고 방지 (CLAUDE.md 7-2).
        #    끄면 후진 표본이 섞여 헤딩이 정확히 180° 틀린 값에 수렴할 수 있다.
        #    부팅 때 시끄럽게 알리고, /heading_status 에 gate=off 를 실어
        #    healthcheck 가 출발 전에 /health_ok=false 로 잡게 한다.
        if not self.mux.estimator.require_reverse_gate:
            self.get_logger().error(
                "🚨 cog_require_reverse_gate=false — 후진 게이트가 꺼져 있다. "
                "벤치 전용이다. 대회 설정이면 지금 true 로 되돌려라. "
                "(후진 중 표본이 섞이면 헤딩이 180° 틀린 값에 수렴할 수 있다)")

        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Float64, raw_topic, self.raw_yaw_cb, qos)
        # COG 표본. 소스가 cog_offset 이 아니어도 구독한다 — 추정을 돌려두면
        # /heading_status 로 '지금 전환하면 쓸 만한가' 를 물 위에서 미리 볼 수 있다.
        self.create_subscription(
            TwistWithCovarianceStamped, gps_vel_topic, self.gps_vel_cb, qos)
        reverse_topic = str(self.declare_parameter(
            'motor_reverse_topic', '/motor_reverse').value)
        self.create_subscription(Bool, reverse_topic, self.reverse_cb, qos)
        self.yaw_pub = self.create_publisher(Float64, out_topic, qos)
        self.status_pub = self.create_publisher(String, '/heading_status', qos)

        self._last_status = None
        self._last_status_pub_t = 0.0

        self.timer = self.create_timer(self.period_sec, self.tick)

        self.get_logger().info(
            f"yaw_mux 시작: source={source}, {1.0 / self.period_sec:.0f}Hz, "
            f"stale={self.stale_sec}s\n"
            f"   {raw_topic} → {out_topic} "
            f"(invert={invert_yaw}, mount={mount_offset_deg}°, decl={declination_deg}°)\n"
            f"   ⚠️ iahrs_driver 는 {raw_topic} 로 내야 한다. {out_topic} 로 두면 발행자 2개다."
        )

    # ───────────────────────── 콜백 ─────────────────────────
    def raw_yaw_cb(self, msg):
        self.mux.update_imu(msg.data, time.monotonic())   # 단조시계 (CLAUDE.md 1-5)

    def reverse_cb(self, msg):
        self.mux.update_reverse(bool(msg.data), time.monotonic())

    def gps_vel_cb(self, msg):
        v = msg.twist.twist.linear
        cog, speed = cog_from_velocity(v.x, v.y, self.gps_vel_frame)
        if cog is None:
            return          # 정지/무효 — COG 가 정의되지 않는다. 0.0 을 대신 넣지 않는다.
        self.mux.update_cog(cog, speed, time.monotonic())

    # ───────────────────────── 주기 ─────────────────────────
    def tick(self):
        now = time.monotonic()
        yaw, status = self.mux.heading(now)

        if yaw is not None and status == ST_OK:
            self.yaw_pub.publish(Float64(data=float(yaw)))
        else:
            # 발행하지 않는다 — 소비자가 stale 로 감지해 스스로 안전 동작한다.
            self.get_logger().warn(
                f"헤딩 없음({status}) → /imu/yaw 발행 중단. "
                f"ship_goal_angle 정지, geofence 침묵.",
                throttle_duration_sec=2.0)

        # 게이트가 안 와서 추정이 멈춰 있으면 조용히 두지 않는다.
        # (포화 함정과 같은 유형 — '왜 수렴을 안 하지' 로 시간을 버리는 것을 막는다)
        if (self.mux.estimator.require_reverse_gate
                and self.mux.reverse_state(now) is None):
            self.get_logger().warn(
                "/motor_reverse 없음(또는 stale) → COG 오프셋 표본 수집 정지. "
                "motor_control 이 떠 있나? 헤딩만 벤치 중이면 "
                "cog_require_reverse_gate:=false.",
                throttle_duration_sec=10.0)

        self._publish_status(status, now)

    def _publish_status(self, status, now):
        """상태가 바뀌었거나 1초 지났을 때만 (50Hz 로 문자열을 뿌리지 않는다)."""
        if status == self._last_status and (now - self._last_status_pub_t) < 1.0:
            return
        self._last_status = status
        self._last_status_pub_t = now
        est = self.mux.estimator
        # 추정 진행도를 항상 같이 싣는다 — cog_offset 이 아닐 때도 '전환하면 쓸 만한가' 가 보인다.
        off = est.offset_deg
        # gate=off 는 안전 스위치가 꺼졌을 때만 붙인다 — healthcheck 가 이 문자열을 본다.
        gate = "" if est.require_reverse_gate else " gate=off"
        self.status_pub.publish(String(data=(
            f"{self.mux.source}:{status}"
            f" n={est.samples:.0f} R={est.resultant:.2f}"
            f" off={'--' if off is None else f'{off:.1f}'}"
            f" rej={est.last_reject or '-'}{gate}")))


def main(args=None):
    rclpy.init(args=args)
    node = YawMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
