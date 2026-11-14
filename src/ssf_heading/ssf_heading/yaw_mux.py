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
    (예외 — gps_cog: HOLD 는 마지막 방위를, INIT_GOAL 은 /north_goal_angle_tp 를 낸다.
     계약서_GPS방위_20261114 Q1 (c) 승인. tp 도 없으면 INIT 로 침묵.
     /boat_mode 가 AUTO(2)로 바뀌는 순간 첫 COG 전 상태로 되돌린다 — 보완 A.
     선회·후진 기동 구간엔 IMU 상대 회전량(Δ)으로 방위를 따라간다(ASSIST) — 보완 B)
    틀린 방위 하나가 배를 정반대로 보낸다. 소비자는 이미 침묵을 처리한다:
      ship_goal_angle  → /yaw_error 발행 중단 → ship_direction 이 감속/정지
      north_goal_angle → geofence 침묵(빈 배열)
  - /heading_status(String) 로 상태를 밖에 알린다 (healthcheck·blackbox 관찰용, 제어엔 안 쓴다)
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, Float64, Int32, String
from geometry_msgs.msg import TwistWithCovarianceStamped

from ssf_heading.heading_logic import (
    ALL_SOURCES, BOAT_MODE_AUTO, GPS_COG_INIT_GOAL, PUBLISH_STATES,
    SRC_COG_OFFSET, SRC_GPS_COG,
    ST_HOLD, ST_IMU_ASSIST, ST_INIT_GOAL, ST_OK,
    COGOffsetEstimator, GPSCOGTracker, HeadingMux, cog_from_velocity, gps_cog_status_text,
)


class YawMux(Node):
    # gps_cog 에서 부팅 뒤 이 시간 넘게 /boat_mode 를 한 번도 못 받으면 WARN (보완 A, §1-6 드러내기).
    # 계약서 고정값 — 로그 시점일 뿐 제어에 안 쓴다(새 yaml 키는 boat_mode_topic 하나만, 보완 A).
    BOAT_MODE_WARN_AFTER_SEC = 5.0
    # gps_cog IMU 보조(보완 B) 'IMU 없음' WARN 도 부팅 뒤 이 시간부터 (첫 틱엔 IMU 가 아직 안 왔다).
    IMU_ASSIST_WARN_AFTER_SEC = BOAT_MODE_WARN_AFTER_SEC

    def __init__(self):
        super().__init__('yaw_mux')

        # ---- 파라미터 (config/ssf_heading.yaml) ----
        # 기본값 gps_cog — 두 배 공통(보완 C, 사용자 11-15). 공통 yaml 과 같이 맞출 것(§1-4).
        source = str(self.declare_parameter('heading_source', SRC_GPS_COG).value)
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
        gps_rate_hz = float(self.declare_parameter('gps_rate_hz', 1.0).value)

        # 🚨 CLAUDE.md 7-2 안전 스위치. cog_offset 후진 게이트이자 gps_cog 의 후진·선회 게이트.
        #    (gps_cog 벤치는 이 하나로 둘 다 끈다 — 계약서_GPS방위_20261114 Q5)
        require_gate = bool(self.declare_parameter('cog_require_reverse_gate', True).value)

        # ---- gps_cog: GPS 도플러 진행방향 그대로 (계약서_GPS방위_20261114) ----
        #    구독 생성보다 먼저 선언한다. 코드 기본값 = 공통 ssf_heading.yaml 값.
        desired_angle_topic = str(self.declare_parameter(
            'desired_angle_topic', '/desired_angle').value)
        goal_bearing_topic = str(self.declare_parameter(
            'goal_bearing_topic', '/north_goal_angle_tp').value)
        # 보완 A: AUTO 진입(→2) 순간 gps_cog 를 첫 COG 전 상태로 (MANUAL 로 들고 온 COG 를 버린다)
        boat_mode_topic = str(self.declare_parameter(
            'boat_mode_topic', '/boat_mode').value)
        self.gps_cog_hold_warn_sec = float(
            self.declare_parameter('gps_cog_hold_warn_sec', 5.0).value)
        gps_cog = GPSCOGTracker(
            min_speed_mps=float(self.declare_parameter('gps_cog_min_speed_mps', 0.5).value),
            fresh_sec=float(self.declare_parameter('gps_cog_fresh_sec', 1.5).value),
            predict_max_sec=float(
                self.declare_parameter('gps_cog_predict_max_sec', 1.5).value),
            max_rate_dps=float(self.declare_parameter('gps_cog_max_rate_dps', 30.0).value),
            settle_sec=float(self.declare_parameter('gps_cog_settle_sec', 2.0).value),
            jump_deg=float(self.declare_parameter('gps_cog_jump_deg', 90.0).value),
            # 모르는 값이면 ValueError 로 부팅 거부 — 조용히 다른 초기화로 떨어지지 않는다.
            init=str(self.declare_parameter('gps_cog_init', GPS_COG_INIT_GOAL).value),
            init_stale_sec=float(
                self.declare_parameter('gps_cog_init_stale_sec', 2.0).value),
            require_gate=require_gate,
            # 보완 B: 기동(선회·후진) 구간 IMU 상대 회전량 보조. 런타임 되돌리기 = false + 재시작.
            imu_assist=bool(self.declare_parameter('gps_cog_imu_assist', True).value),
            imu_assist_max_sec=float(
                self.declare_parameter('gps_cog_imu_assist_max_sec', 10.0).value),
            imu_max_step_deg=float(
                self.declare_parameter('gps_cog_imu_max_step_deg', 10.0).value),
            imu_stale_sec=self.stale_sec,      # 새 키 없음 — stale_sec 재사용 (B-Q3)
        )

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
                require_reverse_gate=require_gate,
            ),
            gps_cog=gps_cog,
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
            if source == SRC_GPS_COG:
                self.get_logger().error(
                    "🚨 gps_cog: 게이트가 꺼져 후진·제자리 선회 중 COG 도 방위로 채택한다 "
                    "(후진이면 180° 반대). 벤치 전용 — 대회 설정이면 true 로.")

        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Float64, raw_topic, self.raw_yaw_cb, qos)
        # COG 표본. 소스가 cog_offset 이 아니어도 구독한다 — 추정을 돌려두면
        # /heading_status 로 '지금 전환하면 쓸 만한가' 를 물 위에서 미리 볼 수 있다.
        self.create_subscription(
            TwistWithCovarianceStamped, gps_vel_topic, self.gps_vel_cb, qos)
        reverse_topic = str(self.declare_parameter(
            'motor_reverse_topic', '/motor_reverse').value)
        self.create_subscription(Bool, reverse_topic, self.reverse_cb, qos)
        # gps_cog 선회 판정(5000/6000)·첫 COG 전 초기 방위. 둘 다 발행자가 이미 있는 토픽.
        # 소스가 gps_cog 가 아니어도 구독한다(추정기와 같은 이유 — 출력엔 안 쓴다).
        self.create_subscription(Float32, desired_angle_topic, self.desired_angle_cb, qos)
        self.create_subscription(Float32, goal_bearing_topic, self.goal_bearing_cb, qos)
        # ssf_bridge 발행 RELIABLE(§3 /boat_*) ↔ 이 구독 RELIABLE d2 — 호환
        self.create_subscription(Int32, boat_mode_topic, self.boat_mode_cb, qos)
        self.yaw_pub = self.create_publisher(Float64, out_topic, qos)
        self.status_pub = self.create_publisher(String, '/heading_status', qos)

        self._last_status = None
        self._last_status_pub_t = 0.0
        # gps_cog 로그 상태 (HOLD WARN 뒤 복구 INFO, 1회성 INFO)
        self._gps_hold_warned = False
        self._gps_init_goal_logged = False
        self._gps_first_cog_logged = False
        self._boot_t = time.monotonic()      # /boat_mode 미수신 WARN 기준 (단조시계, §1-5)
        # 보완 B 로그: 추적기 누적 카운터의 증가분으로 낸다 (보조 시작마다는 로그 없음 — 상태 문자열로만)
        self._seen_assist_limit = 0
        self._seen_assist_imu_stale = 0
        self._seen_imu_step_rejects = 0

        self.timer = self.create_timer(self.period_sec, self.tick)

        self.get_logger().info(
            f"yaw_mux 시작: source={source}, {1.0 / self.period_sec:.0f}Hz, "
            f"stale={self.stale_sec}s\n"
            f"   {raw_topic} → {out_topic} "
            f"(invert={invert_yaw}, mount={mount_offset_deg}°, decl={declination_deg}°)\n"
            f"   ⚠️ iahrs_driver 는 {raw_topic} 로 내야 한다. {out_topic} 로 두면 발행자 2개다."
        )
        # 위 줄의 mount=… 가 cog_offset 에도 걸리는 것처럼 읽히는 오해를 막는다.
        if source == SRC_COG_OFFSET:
            self.get_logger().info(
                "cog_offset: mount_offset_deg 미적용 — 추정 오프셋(off)이 mount + 게걸음을 "
                "흡수한다. invert_yaw 는 s·yaw 로 한 번만 적용(오프셋 위에 다시 안 건다).")
        if source == SRC_GPS_COG:
            trk = self.mux.gps_cog
            self.get_logger().info(
                f"gps_cog: /imu/yaw = GPS 도플러 진행방향(게걸음 포함). IMU 절대값 배제, "
                f"mount_offset_deg·편각 미적용. "
                f"채택 속도≥{trk.min_speed_mps}m/s, settle={trk.settle_sec}s, "
                f"fresh={trk.fresh_sec}s, 첫 COG 전 초기값={goal_bearing_topic}(INIT_GOAL), "
                f"{boat_mode_topic}={BOAT_MODE_AUTO}(AUTO) 진입마다 초기화\n"
                f"   IMU 보조={'on' if trk.imu_assist else 'off'} "
                f"(선회·후진 기동 구간에만 {raw_topic} 상대 회전량 Δ, invert_yaw 는 Δ 부호에 반영), "
                f"상한=마지막 기동 신호 뒤 {trk.imu_assist_max_sec}s, "
                f"튐 기준 |Δ|>{trk.imu_max_step_deg}°/샘플, IMU stale {trk.imu_stale_sec}s")

    # ───────────────────────── 콜백 ─────────────────────────
    def raw_yaw_cb(self, msg):
        self.mux.update_imu(msg.data, time.monotonic())   # 단조시계 (CLAUDE.md 1-5)

    def reverse_cb(self, msg):
        self.mux.update_reverse(bool(msg.data), time.monotonic())

    def gps_vel_cb(self, msg):
        v = msg.twist.twist.linear
        cog, speed = cog_from_velocity(v.x, v.y, self.gps_vel_frame)
        # 정지/무효면 cog=None — 0.0 을 대신 넣지 않는다. 그래도 넘긴다: gps_cog 는 이 순간
        # 얼려야 한다(외삽 계속 금지). 추정기 경로는 update_cog 안에서 예전처럼 무시한다.
        self.mux.update_cog(cog, speed, time.monotonic())

    def desired_angle_cb(self, msg):
        self.mux.update_desired_angle(float(msg.data), time.monotonic())

    def goal_bearing_cb(self, msg):
        self.mux.update_goal_bearing(float(msg.data), time.monotonic())

    def boat_mode_cb(self, msg):
        entered = self.mux.update_boat_mode(int(msg.data))
        if not (entered and self.mux.source == SRC_GPS_COG):
            return
        now = time.monotonic()
        yaw, status = self.mux.heading(now)
        # 진입 INFO 가 INIT_GOAL 을 알렸으면 tick 의 INIT_GOAL INFO 는 다시 안 낸다
        # (INIT 으로 들어갔으면 나중에 tp 가 와서 INIT_GOAL 이 될 때 알린다).
        # 진입 뒤 첫 COG 채택은 다시 알린다. HOLD WARN 이력은 새 출발이라 지운다.
        self._gps_init_goal_logged = (status == ST_INIT_GOAL)
        self._gps_first_cog_logged = False
        self._gps_hold_warned = False
        what = (f"INIT_GOAL(현재 WP 방위 {yaw:.1f}°)" if status == ST_INIT_GOAL
                else f"{status}(현재 WP 방위 없음 → /imu/yaw 침묵)")
        self.get_logger().info(f"AUTO 진입 → gps_cog 초기화, {what}")

    # ───────────────────────── 주기 ─────────────────────────
    def tick(self):
        now = time.monotonic()
        yaw, status = self.mux.heading(now)

        # gps_cog 의 HOLD·INIT_GOAL 도 발행한다(PUBLISH_STATES). imu_relative 등은 예전처럼 OK 만.
        if yaw is not None and status in PUBLISH_STATES:
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

        if self.mux.source == SRC_GPS_COG:
            self._gps_cog_logs(yaw, status, now)

        self._publish_status(status, now)

    def _gps_cog_logs(self, yaw, status, now):
        """gps_cog 상태 로그. 🚨 심각도마다 호출 지점을 나눈다 (CLAUDE.md 1-7)."""
        trk = self.mux.gps_cog
        if status == ST_INIT_GOAL and not self._gps_init_goal_logged:
            self._gps_init_goal_logged = True
            self.get_logger().info(
                f"gps_cog INIT_GOAL: 첫 유효 COG 전 → 현재 WP 방위 {yaw:.1f}° 를 /imu/yaw 로 발행 "
                f"(전제: 출발 때 뱃머리를 현재 WP 로 겨눴다)")
        if status in (ST_OK, ST_HOLD, ST_IMU_ASSIST) and not self._gps_first_cog_logged:
            self._gps_first_cog_logged = True
            self.get_logger().info(f"gps_cog: 첫 COG 채택 → {yaw:.1f}°")

        if status == ST_HOLD:
            age = trk.age(now)
            if age is not None and age > self.gps_cog_hold_warn_sec:
                self._gps_hold_warned = True
                self.get_logger().warn(
                    f"gps_cog HOLD {age:.0f}s — 새 COG 채택 없음(rej={trk.last_reject or '-'}) "
                    f"→ 마지막 방위 {yaw:.1f}° 유지 발행",
                    throttle_duration_sec=5.0)
        elif status == ST_OK and self._gps_hold_warned:
            self._gps_hold_warned = False
            self.get_logger().info(f"gps_cog 복구 → OK {yaw:.1f}°")

        # /boat_mode 를 한 번도 못 받으면 AUTO 진입 초기화가 안 일어난다(= C1 동작) — 드러낸다 (§1-6)
        if (self.mux.boat_mode is None
                and (now - self._boot_t) > self.BOAT_MODE_WARN_AFTER_SEC):
            self.get_logger().warn(
                "/boat_mode 미수신 → AUTO 진입 때 gps_cog 초기화 안 됨"
                "(MANUAL 중 채택한 COG 가 그대로 남는다). ssf_bridge 가 떠 있나?",
                throttle_duration_sec=10.0)

        self._gps_cog_assist_logs(yaw, now)

        # 선회 게이트가 안 와서 갱신이 멈춰 있으면 조용히 두지 않는다 (후진 게이트 경고와 같은 취지)
        if trk.require_gate and self.mux.spin_state(now) is None:
            self.get_logger().warn(
                "/desired_angle 없음(또는 stale) → gps_cog 갱신 정지(마지막 방위 유지). "
                "ship_direction 이 떠 있나? 헤딩만 벤치 중이면 cog_require_reverse_gate:=false.",
                throttle_duration_sec=10.0)

    def _gps_cog_assist_logs(self, yaw, now):
        """보완 B IMU 보조 로그. 🚨 심각도마다 호출 지점을 나눈다 (CLAUDE.md 1-7)."""
        trk = self.mux.gps_cog
        yaw_s = '--' if yaw is None else f'{yaw:.1f}'

        n = trk.assist_ends["limit"]
        if n != self._seen_assist_limit:
            self._seen_assist_limit = n
            self.get_logger().info(
                f"gps_cog IMU 보조 상한 종료: 마지막 기동 신호 뒤 {trk.imu_assist_max_sec}s 경과 "
                f"→ {yaw_s}° 에서 얼림(새 COG 채택까지 HOLD) (누적 {n}회)",
                throttle_duration_sec=10.0)

        n = trk.assist_ends["imu_stale"]
        if n != self._seen_assist_imu_stale:
            self._seen_assist_imu_stale = n
            self.get_logger().warn(
                f"gps_cog IMU 보조 중 IMU 끊김({trk.imu_stale_sec}s 넘게 미수신) → 보조 종료, "
                f"마지막 IMU 샘플 시점 값 {yaw_s}° 에서 얼림 (누적 {n}회)",
                throttle_duration_sec=5.0)

        n = trk.imu_step_rejects
        if n != self._seen_imu_step_rejects:
            self._seen_imu_step_rejects = n
            self.get_logger().warn(
                f"gps_cog IMU 보조: 튐 샘플 Δ 버림 누적 {n}회 "
                f"(샘플당 |Δ|>{trk.imu_max_step_deg}°)",
                throttle_duration_sec=5.0)

        # healthcheck 는 /imu/yaw OK 면 원시 yaw 를 안 본다 → 이게 유일한 표시 (§1-6)
        if (trk.imu_assist and not trk.imu_fresh(now)
                and (now - self._boot_t) > self.IMU_ASSIST_WARN_AFTER_SEC):
            self.get_logger().warn(
                "gps_cog: IMU 없음(/imu/yaw_raw 미수신·stale) → 기동 중 회전 보조 불가(C1 얼림). "
                "iahrs_driver 가 떠 있나?",
                throttle_duration_sec=10.0)

    def _publish_status(self, status, now):
        """상태가 바뀌었거나 1초 지났을 때만 (50Hz 로 문자열을 뿌리지 않는다)."""
        if status == self._last_status and (now - self._last_status_pub_t) < 1.0:
            return
        self._last_status = status
        self._last_status_pub_t = now
        if self.mux.source == SRC_GPS_COG:
            self.status_pub.publish(String(data=self._gps_cog_status(status, now)))
            return
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

    def _gps_cog_status(self, status, now):
        """gps_cog 상태 문자열 — 순수 로직 gps_cog_status_text 로 옮겼다(테스트 가능하게, 보완 B)."""
        return gps_cog_status_text(self.mux.gps_cog, status, now)


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
