#!/usr/bin/env python3
"""
blackbox — 비행기 블랙박스처럼, 배가 도는 동안 모든 핵심 신호를 10Hz CSV 로 남긴다.

원칙 (CLAUDE.md 0단계):
  * 구독만 한다. 기존 토픽에 절대 발행하지 않는다. → 아무것도 못 깨뜨린다.
  * 이후 모든 단계의 개선을 '측정' 하는 눈이 된다. (규정: 종합임무 5회, 최고점 채택
    → 왜 실패했는지 알아야 다음 회차에서 고친다.)

컬럼:
  t_wall(ROS시계), t_mono(단조시계 경과s), wp_mode, gps_lat, gps_lon, gps_status,
  imu_yaw, yaw_error, candidate_angle, desired_angle, pwm_r, pwm_l,
  obstacle_min_dist, failsafe_level, gate_pass_count,
  gyro_z, accel_x, accel_y, gps_vel_x, gps_vel_y   ← 동역학 역산(system ID)용

★ 동역학 역산용 신호 (배 물성·추력을 CSV 에서 되찾기 위해):
  · gyro_z(각속도 z) + accel_x/y(선가속도)  ← /imu/data(sensor_msgs/Imu). 회전관성·추력을 '직접' 준다.
  · gps_vel_x/y(GPS 속도)                    ← ~/fix_velocity. 미분 없이 속도.
  이게 없으면 가속도를 GPS 위치의 2차 미분으로 뽑아야 해 노이즈가 심하다. 있으면 미분 단계가 사라진다.
  ⚠️ system-ID 주행 때는 rate_hz 를 20~50 으로 올려라(동역학이 더 잘 잡힌다). 기본은 10.
"""

import csv
import math
import os
import time
import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import Int32, Float32, Float64, Float32MultiArray
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import TwistWithCovarianceStamped


# 관찰자 QoS: BEST_EFFORT + VOLATILE.
# best_effort 구독자는 reliable/best_effort 발행자 모두와 호환된다(라이브 수신 보장).
# 발행자 QoS 를 모른 채로도 데이터가 조용히 사라지지 않게 하는 가장 안전한 선택.
OBSERVER_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

CSV_HEADER = [
    "t_wall", "t_mono", "wp_mode",
    "gps_lat", "gps_lon", "gps_status",
    "imu_yaw", "yaw_error", "candidate_angle", "desired_angle",
    "pwm_r", "pwm_l", "obstacle_min_dist",
    "failsafe_level", "gate_pass_count",
    # ---- 동역학 역산(system ID)용 ----
    "gyro_z", "accel_x", "accel_y",   # /imu/data: 각속도 z, 선가속도 x/y
    "gps_vel_x", "gps_vel_y",         # ~/fix_velocity: 속도 성분 (프레임은 드라이버 규약 — ⚠️ 확인)
]


class BlackBox(Node):
    def __init__(self):
        super().__init__("blackbox")

        # ---- 파라미터 ----
        self.rate_hz = float(self.declare_parameter("rate_hz", 10.0).value)
        log_dir = str(self.declare_parameter("log_dir", "").value)
        self.flush_every_n = int(self.declare_parameter("flush_every_n", 10).value)

        wp_mode_topic = self.declare_parameter("wp_mode_topic", "/wp_mode").value
        gps_topic = self.declare_parameter("gps_topic", "/ublox_gps_node/fix").value
        imu_yaw_topic = self.declare_parameter("imu_yaw_topic", "/imu/yaw").value
        yaw_error_topic = self.declare_parameter("yaw_error_topic", "/yaw_error").value
        candidate_topic = self.declare_parameter("candidate_topic", "/candidate_angle").value
        desired_angle_topic = self.declare_parameter("desired_angle_topic", "/desired_angle").value
        motor_topic = self.declare_parameter("motor_topic", "Motor_run").value
        obstacle_topic = self.declare_parameter("obstacle_topic", "/obstacle_distance_array").value
        failsafe_topic = self.declare_parameter("failsafe_topic", "/failsafe_level").value
        gate_count_topic = self.declare_parameter("gate_count_topic", "/gates_passed").value
        # 동역학 역산용 (system ID)
        imu_topic = self.declare_parameter("imu_topic", "/imu/data").value                       # sensor_msgs/Imu
        gps_vel_topic = self.declare_parameter(
            "gps_vel_topic", "/ublox_gps_node/fix_velocity").value                                # TwistWithCovarianceStamped

        # ---- 최신값 저장소 (콜백이 갱신, 타이머가 읽어 기록) ----
        self.latest = {k: None for k in CSV_HEADER}

        # ---- 로그 파일 준비 ----
        # 기본 경로를 ~/ssf_logs 로 (역산 워크플로우가 여기서 CSV 를 집는다). log_dir 로 바꿀 수 있음.
        if not log_dir:
            log_dir = os.path.join(os.path.expanduser("~"), "ssf_logs")
        os.makedirs(log_dir, exist_ok=True)
        # 파일명에만 벽시계 사용(가독성 목적, 제어 타이밍 아님 → CLAUDE.md 1-5 규칙 무관)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_path = os.path.join(log_dir, f"blackbox_{stamp}.csv")
        self._fh = open(self.log_path, "w", newline="")
        self._writer = csv.writer(self._fh)
        self._writer.writerow(CSV_HEADER)
        self._fh.flush()
        self._rows = 0
        self._t0 = time.monotonic()

        # ---- 구독 (전부 관찰 전용) ----
        self.create_subscription(Int32, wp_mode_topic, self._cb_wp_mode, OBSERVER_QOS)
        self.create_subscription(NavSatFix, gps_topic, self._cb_gps, OBSERVER_QOS)
        self.create_subscription(Float64, imu_yaw_topic, self._cb_imu_yaw, OBSERVER_QOS)
        self.create_subscription(Float32, yaw_error_topic, self._cb_yaw_error, OBSERVER_QOS)
        self.create_subscription(Float32, candidate_topic, self._cb_candidate, OBSERVER_QOS)
        self.create_subscription(Float32, desired_angle_topic, self._cb_desired, OBSERVER_QOS)
        self.create_subscription(Int32, motor_topic, self._cb_motor, OBSERVER_QOS)
        self.create_subscription(Float32MultiArray, obstacle_topic, self._cb_obstacle, OBSERVER_QOS)
        self.create_subscription(Int32, failsafe_topic, self._cb_failsafe, OBSERVER_QOS)
        self.create_subscription(Int32, gate_count_topic, self._cb_gate_count, OBSERVER_QOS)
        # 동역학 역산용
        self.create_subscription(Imu, imu_topic, self._cb_imu, OBSERVER_QOS)
        self.create_subscription(TwistWithCovarianceStamped, gps_vel_topic, self._cb_gps_vel, OBSERVER_QOS)

        # ---- 기록 타이머 ----
        period = 1.0 / self.rate_hz if self.rate_hz > 0 else 0.1
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"📼 blackbox 시작: {self.rate_hz:.0f}Hz → {self.log_path}\n"
            f"   구독만 함 (발행 없음). 미존재 토픽(failsafe/gate)은 빈칸으로 남습니다."
        )

    # ---- 콜백들: 최신값만 갱신 ----
    def _cb_wp_mode(self, msg): self.latest["wp_mode"] = msg.data

    def _cb_gps(self, msg):
        self.latest["gps_lat"] = msg.latitude
        self.latest["gps_lon"] = msg.longitude
        self.latest["gps_status"] = msg.status.status  # -1=NO_FIX, 0=FIX, 1=SBAS, 2=GBAS

    def _cb_imu_yaw(self, msg): self.latest["imu_yaw"] = msg.data
    def _cb_yaw_error(self, msg): self.latest["yaw_error"] = msg.data
    def _cb_candidate(self, msg): self.latest["candidate_angle"] = msg.data
    def _cb_desired(self, msg): self.latest["desired_angle"] = msg.data

    def _cb_motor(self, msg):
        # Motor_run = pwm_r*10000 + pwm_l (1500 = 중립)
        v = int(msg.data)
        self.latest["pwm_r"] = v // 10000
        self.latest["pwm_l"] = v % 10000

    def _cb_obstacle(self, msg):
        # ship_direction 계약: data = [closest_distance(m), closest_angle(deg)] 또는 [inf, nan].
        # data[1] 은 '각도'지 거리가 아니다 → min() 을 쓰면 각도를 거리로 오기록한다(버그).
        # 거리는 data[0] 하나뿐. inf/nan(장애물 없음)은 빈칸으로.
        if len(msg.data) >= 1 and math.isfinite(msg.data[0]):
            self.latest["obstacle_min_dist"] = msg.data[0]
        else:
            self.latest["obstacle_min_dist"] = None

    def _cb_failsafe(self, msg): self.latest["failsafe_level"] = msg.data
    def _cb_gate_count(self, msg): self.latest["gate_pass_count"] = msg.data

    def _cb_imu(self, msg):
        # sensor_msgs/Imu: 각속도 z(yaw rate) + 선가속도 x/y (body frame). 동역학 역산의 핵심.
        self.latest["gyro_z"] = msg.angular_velocity.z
        self.latest["accel_x"] = msg.linear_acceleration.x
        self.latest["accel_y"] = msg.linear_acceleration.y

    def _cb_gps_vel(self, msg):
        # TwistWithCovarianceStamped: 미분 없이 속도. 프레임은 ublox 드라이버 규약(보통 ENU) — ⚠️ 확인.
        self.latest["gps_vel_x"] = msg.twist.twist.linear.x
        self.latest["gps_vel_y"] = msg.twist.twist.linear.y

    # ---- 10Hz 기록 ----
    def _on_timer(self):
        now = self.get_clock().now()
        t_wall = now.nanoseconds * 1e-9              # ROS 시계 (기록용 절대시각)
        t_mono = time.monotonic() - self._t0         # 단조 경과초 (CLAUDE.md 1-5)
        self.latest["t_wall"] = f"{t_wall:.3f}"
        self.latest["t_mono"] = f"{t_mono:.3f}"

        row = []
        for k in CSV_HEADER:
            v = self.latest.get(k)
            row.append("" if v is None else v)
        self._writer.writerow(row)

        self._rows += 1
        if self._rows % max(1, self.flush_every_n) == 0:
            self._fh.flush()

    def destroy_node(self):
        try:
            self._fh.flush()
            self._fh.close()
            self.get_logger().info(f"📼 blackbox 종료: {self._rows}행 저장 → {self.log_path}")
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BlackBox()
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
