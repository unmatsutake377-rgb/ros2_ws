#!/usr/bin/env python3
"""
healthcheck — 2초마다 '배가 살아있나'를 점검하고 /health_ok(Bool) 를 발행한다.

원칙 (CLAUDE.md 0단계 / 3-6):
  * 구독만 한다. 새로 발행하는 건 /health_ok 하나뿐(신규 토픽 → 기존 계약 불변).
  * 두 가지를 본다:
      1) 센서 생존   : /scan, /imu/yaw, /ublox_gps_node/fix 가 제때 들어오나
      2) 매핑표 검사 : wp_mode -> 담당 노드 표에서 '빠진 모드 / 침묵하는 노드' 적발
                       (작년 도킹 노드가 1년 내내 침묵한 사고의 재발 방지)

오탐 방지 (CLAUDE.md 5장):
  * ARMED: 센서를 '한 번이라도' 받은 뒤에만 죽음 판정. 부팅 중엔 '대기'로 본다.
  * time.monotonic() 사용 (벽시계 점프에 안 속음, CLAUDE.md 1-5).
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import Int32, Float32, Float64, Bool, String
from sensor_msgs.msg import Image, LaserScan, NavSatFix


OBSERVER_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class HealthCheck(Node):
    def __init__(self):
        super().__init__("healthcheck")

        # ---- 파라미터 ----
        self.period = float(self.declare_parameter("health_period_sec", 2.0).value)
        self.timeout = float(self.declare_parameter("sensor_timeout_sec", 2.0).value)

        scan_topic = self.declare_parameter("scan_topic", "/scan").value
        imu_yaw_topic = self.declare_parameter("imu_yaw_topic", "/imu/yaw").value
        # N1: /imu/yaw 경로에 yaw_mux 가 끼면서 'IMU 죽음' 과 'yaw_mux 죽음' 이 구분이 안 된다.
        #     원시 yaw 를 함께 보면 어느 쪽인지 즉시 안다. 진단 전용 — /health_ok 판정엔 안 쓴다
        #     (거짓 정지 금지 원칙).
        imu_raw_topic = self.declare_parameter("imu_raw_yaw_topic", "/imu/yaw_raw").value
        heading_status_topic = self.declare_parameter(
            "heading_status_topic", "/heading_status").value
        gps_topic = self.declare_parameter("gps_topic", "/ublox_gps_node/fix").value
        # T2-7: 카메라 이미지 수신 감시. 카메라는 '조용히' 죽는다 — USB 가 빠져도, 드라이버가
        #   멈춰도 에러 토픽은 안 나온다. V1 에서 depth 가드를 없앴으니 비전 노드는 이제
        #   이미지가 안 와도 예외 없이 그냥 침묵한다 → 여기서 잡지 않으면 아무도 모른다.
        #   ⚠️ image_topic 은 V5 에서 파라미터가 됐다. 카메라를 바꾸면 여기도 같이 바꿀 것.
        image_topic = self.declare_parameter(
            "image_topic", "/camera/camera/color/image_raw").value
        # 이미지는 센서 중 가장 느릴 수 있다(30fps 여도 처리 지연). 별도 타임아웃.
        self.image_timeout = float(
            self.declare_parameter("image_timeout_sec", 3.0).value)
        wp_mode_topic = self.declare_parameter("wp_mode_topic", "/wp_mode").value
        candidate_topic = self.declare_parameter("candidate_topic", "/candidate_angle").value

        # mode 0,1 → ship_gate (6b 에서 ship_last 제거, mode 0 인수). 5,8 = 회피(담당 없음).
        wp_modes = list(self.declare_parameter("wp_modes", [0, 1, 2, 3, 5, 7, 8]).value)
        wp_nodes = list(self.declare_parameter(
            "wp_nodes",
            ["ship_gate", "ship_gate", "ship_back", "ship_turn", "none", "ship_dock", "none"]).value)

        # 🚨 geofence 배선 검사 — '잊으면 최악'.
        # north_goal_angle 이 /geofence_state 를 발행하는데 구독자가 0 이면 경계 이탈 방어가
        # 통째로 없는 것이다(실격). 발행자만 있고 아무도 안 듣는 상태는 에러를 내지 않는다
        # — 도킹이 1년간 침묵한 것과 똑같은 사고다. **출발 전에** 여기서 잡는다.
        self.geofence_topic = str(
            self.declare_parameter("geofence_topic", "/geofence_state").value)
        self.require_geofence_sub = bool(
            self.declare_parameter("require_geofence_subscriber", True).value)

        # wp_mode -> 담당 노드 매핑표 (yaml 한 곳에 모음 = CLAUDE.md 3-6 의 단일 출처)
        self.wp_map = {}
        for m, n in zip(wp_modes, wp_nodes):
            self.wp_map[int(m)] = str(n)

        # ---- 마지막 수신 시각(단조시계). None = 아직 한 번도 못 받음(비ARMED) ----
        self._last = {"scan": None, "imu": None, "gps": None, "candidate": None,
                      "imu_raw": None, "image": None}
        self._cur_wp_mode = None
        self._heading_status = None

        # ---- 구독 (센서 생존 감시 + 현재 모드) ----
        self.create_subscription(LaserScan, scan_topic, self._cb_scan, OBSERVER_QOS)
        self.create_subscription(Float64, imu_yaw_topic, self._cb_imu, OBSERVER_QOS)
        self.create_subscription(Float64, imu_raw_topic, self._cb_imu_raw, OBSERVER_QOS)
        self.create_subscription(
            String, heading_status_topic, self._cb_heading_status, OBSERVER_QOS)
        self.create_subscription(NavSatFix, gps_topic, self._cb_gps, OBSERVER_QOS)
        # 이미지 발행자는 BEST_EFFORT 인 경우가 흔하다 → 관찰자도 BEST_EFFORT 여야 받는다
        # (구독자 RELIABLE + 발행자 BEST_EFFORT = 0건. OBSERVER_QOS 가 BEST_EFFORT 라 안전)
        self.create_subscription(Image, image_topic, self._cb_image, OBSERVER_QOS)
        self.create_subscription(Int32, wp_mode_topic, self._cb_wp_mode, OBSERVER_QOS)
        # candidate 는 '담당 노드가 실제로 말하고 있나' 확인용
        self.create_subscription(Float32, candidate_topic, self._cb_candidate, OBSERVER_QOS)

        # ---- 발행: /health_ok (신규 토픽) ----
        self.pub_health = self.create_publisher(Bool, "/health_ok", 10)

        # ---- 매핑표 정합성 정적 검사 (부팅 1회) ----
        self._static_map_check(wp_modes, wp_nodes)

        # ---- 독립 타이머로 평가 (센서가 죽어 콜백이 안 와도 평가는 계속) ----
        self.timer = self.create_timer(self.period, self._on_timer)

        self.get_logger().info(
            f"🩺 healthcheck 시작: {self.period:.0f}s 주기, 센서 타임아웃 {self.timeout:.0f}s\n"
            f"   매핑표: {self.wp_map}\n"
            f"   /health_ok 발행 (그 외 기존 토픽엔 발행 안 함)"
        )

    # ---- 매핑표 정적 검사 ----
    def _static_map_check(self, wp_modes, wp_nodes):
        problems = []
        if len(wp_modes) != len(wp_nodes):
            problems.append(f"wp_modes({len(wp_modes)})/wp_nodes({len(wp_nodes)}) 길이 불일치")
        dup_modes = {m for m in wp_modes if wp_modes.count(m) > 1}
        if dup_modes:
            problems.append(f"중복 모드: {sorted(dup_modes)}")
        # ※ 한 노드가 여러 모드를 담당하는 건 정상이다 (ship_gate 는 mode 0,1 둘 다 맡는다).
        #   그래서 '노드 중복' 은 검사하지 않는다. '모드 중복'(같은 mode 를 두 노드가) 만 오류다.
        if problems:
            for p in problems:
                self.get_logger().error(f"🩺 매핑표 정적 검사 ❌ {p}")
        else:
            self.get_logger().info("🩺 매핑표 정적 검사 ✅ 중복/누락 없음")

    # ---- 콜백: 수신 시각 기록 ----
    def _now(self):
        return time.monotonic()

    def _cb_scan(self, _msg): self._last["scan"] = self._now()
    def _cb_imu(self, _msg): self._last["imu"] = self._now()
    def _cb_gps(self, _msg): self._last["gps"] = self._now()
    def _cb_image(self, _msg): self._last["image"] = self._now()
    def _cb_candidate(self, _msg): self._last["candidate"] = self._now()
    def _cb_imu_raw(self, _msg): self._last["imu_raw"] = self._now()
    def _cb_wp_mode(self, msg): self._cur_wp_mode = int(msg.data)
    def _cb_heading_status(self, msg): self._heading_status = str(msg.data)

    # ---- 상태 판정 ----
    def _sensor_state(self, key, timeout=None):
        """반환: 'OK' | 'DEAD' | 'WAIT'(아직 한 번도 못 받음)"""
        t = self._last[key]
        if t is None:
            return "WAIT"
        lim = self.timeout if timeout is None else timeout
        return "OK" if (self._now() - t) <= lim else "DEAD"

    # ---- 2초마다 평가 ----
    def _on_timer(self):
        scan = self._sensor_state("scan")
        imu = self._sensor_state("imu")
        gps = self._sensor_state("gps")
        image = self._sensor_state("image", self.image_timeout)
        sensors = {"LiDAR/scan": scan, "IMU/yaw": imu, "GPS/fix": gps,
                   "CAM/image": image}

        # 살아있는 노드 목록 (침묵하는 노드 적발용)
        alive_nodes = set(self.get_node_names())

        lines = ["🩺 healthcheck"]
        icon = {"OK": "✅", "DEAD": "❌", "WAIT": "…"}
        lines.append("   센서: " + "  ".join(
            f"{name} {icon[st]}" for name, st in sensors.items()))

        # --- 헤딩 경로 원인 분리 (진단만, /health_ok 엔 반영 안 함) ---
        # /imu/yaw 는 이제 yaw_mux 가 낸다. 끊겼을 때 원인이 IMU 냐 mux 냐를 여기서 가른다.
        imu_raw = self._sensor_state("imu_raw")
        if imu != "OK":
            hs = self._heading_status or "(상태 없음)"
            if imu_raw == "OK":
                lines.append(
                    f"   ⚠️ /imu/yaw 끊김인데 원시 yaw 는 살아있다 → yaw_mux 문제. status={hs}")
            elif imu_raw == "DEAD":
                lines.append("   ⚠️ 원시 yaw 도 끊김 → IMU/드라이버 문제 (yaw_mux 아님)")
            else:
                lines.append(
                    f"   ⚠️ 원시 yaw 를 한 번도 못 받음 → iahrs_driver 미기동 또는 "
                    f"yaw_topic 설정 확인. status={hs}")
        elif self._heading_status and not self._heading_status.endswith(":OK"):
            lines.append(f"   ⚠️ heading_status={self._heading_status}")

        # --- 매핑표 런타임 검사 ---
        map_ok = True
        for mode, node in sorted(self.wp_map.items()):
            if str(node).lower() == "none":
                continue  # mode 5,8 = 순수 회피 구간(담당 노드 없음이 정상). 경고 금지.
            present = node in alive_nodes
            if not present:
                lines.append(f"   ❌ mode {mode} → {node} : 노드가 떠있지 않음(침묵)")
                map_ok = False

        # 현재 진행 중인 wp_mode 점검
        cur = self._cur_wp_mode
        cur_owner_ok = True
        if cur is not None:
            if cur not in self.wp_map:
                lines.append(f"   ❌ 현재 wp_mode={cur} 인데 담당 노드가 매핑표에 없음")
                cur_owner_ok = False
            else:
                owner = self.wp_map[cur]
                if str(owner).lower() == "none":
                    # 순수 회피 구간(mode 5,8): ship_direction 이 GPS 로 감. 정상.
                    lines.append(f"   ✅ wp_mode={cur} 회피 구간(담당 노드 없음이 정상)")
                elif owner not in alive_nodes:
                    lines.append(f"   ❌ 현재 wp_mode={cur} 담당 {owner} 침묵")
                    cur_owner_ok = False
                elif self._sensor_state("candidate") == "DEAD":
                    lines.append(f"   ❌ wp_mode={cur} {owner} 살아있으나 /candidate_angle 끊김")
                    cur_owner_ok = False

        # --- geofence 배선 검사 ('잊으면 최악') ---
        # 발행자가 있는데 구독자가 0 이면 경계 이탈 방어가 통째로 없다 = 실격 위험.
        # 침묵하는 기능은 에러를 내지 않는다 → 여기서 소리내어 잡는다.
        geofence_ok = True
        if self.require_geofence_sub:
            n_pub = self.count_publishers(self.geofence_topic)
            n_sub = self.count_subscribers(self.geofence_topic)
            if n_pub == 0:
                lines.append(f"   ❌ {self.geofence_topic} 발행자 없음 (north_goal_angle 미기동?)")
                geofence_ok = False
            elif n_sub == 0:
                lines.append(f"   🚨 {self.geofence_topic} 구독자 0 — 경계 이탈 방어가 작동하지 않는다"
                             f" (실격 위험). ship_direction 이 구독해야 한다.")
                geofence_ok = False

        # --- 🚨 벤치 편의 스위치가 대회 설정에 남았나 (CLAUDE.md 7-2) ---
        # cog_require_reverse_gate=false 면 후진 표본이 섞여 헤딩이 정확히 180° 틀린 값에
        # 수렴할 수 있다(R=1.0 이라 신뢰도로 못 걸러진다). 벤치용 스위치가 실전에 남는 건
        # 이런 편의 스위치의 고전적 말로다 — 사람 기억이 아니라 여기서 기계적으로 잡는다.
        # ※ heading_source 가 cog_offset 일 때만 실패로 친다. imu_relative 로 도는 동안엔
        #   이 게이트가 무의미해서 꺼져 있어도 무해하다 → 거짓 정지를 만들지 않는다.
        gate_ok = True
        hs = self._heading_status
        if hs and hs.startswith("cog_offset:") and "gate=off" in hs:
            lines.append("   ❌ cog_require_reverse_gate=false 인데 heading_source=cog_offset "
                         "— 벤치 플래그가 대회 설정에 남았다. 후진 시 헤딩 180° 오수렴 위험")
            gate_ok = False

        # --- /health_ok 판정 ---
        # 센서는 WAIT(부팅 중)을 죽음으로 치지 않는다(오탐 방지). DEAD 만 실패.
        # 카메라도 sensors 에 포함된다 = DEAD 면 /health_ok=false.
        #   근거: 출발 전에 카메라가 죽어 있으면 게이트·도킹이 **조용히** 실패한다.
        #        이 프로젝트가 반복해 당한 침묵 실패라 출발 전 진단에서 잡는 게 맞다.
        #   ⚠️ 단 /health_ok 는 현재 **구독자 0개(진단 전용)** 이라 거짓 정지 위험이 없어서
        #      이렇게 둔 것이다. 나중에 이걸 실제 제어(정지)에 물린다면 재검토할 것 —
        #      LiDAR 상실은 치명적이지만 카메라 3초 끊김은 그 정도가 아니다.
        sensors_ok = all(st != "DEAD" for st in sensors.values())
        health_ok = sensors_ok and map_ok and cur_owner_ok and geofence_ok and gate_ok

        self.pub_health.publish(Bool(data=bool(health_ok)))

        lines.append(f"   → /health_ok = {health_ok}"
                     f"  (wp_mode={cur if cur is not None else '—'})")
        log = self.get_logger()
        (log.info if health_ok else log.warn)("\n".join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = HealthCheck()
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
