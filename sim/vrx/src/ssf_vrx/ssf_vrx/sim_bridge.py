"""sim_bridge — 시뮬에서 ssf_bridge + 아두이노 + IMU 드라이버 자리를 대신하는 노드.

하는 일 (실물 사슬과 대응):
  · Motor_run(Int32, ROS 규약 <1500 전진) → 각 추진기 cmd_thrust(Float64, N) 20 Hz.
    아두이노처럼 명령이 cmd_timeout_sec(0.5 s) 넘게 안 오면 추력 0 + /boat_cmd_watchdog true.
  · /boat_mode·/boat_estop·/boat_id·/boat_cmd_watchdog 를 10 Hz 로 낸다(ssf_bridge 와 같은 토픽·타입).
    boat_mode 는 파라미터(기본 2 = AUTO) — north_goal_angle 의 require_boat_mode_auto 게이트를 연다.
  · Gazebo IMU(sensor_msgs/Imu, ENU) → /imu/yaw_raw(Float64, 0=북 시계) — iahrs_driver 자리.
    보정은 실물과 똑같이 yaw_mux 가 한다(ssf_heading_A/B.yaml 의 장착값 0 이면 그대로).

안 하는 일: 거짓 상태(모드 전환 스위치·비상정지)는 흉내 내지 않는다. /boat_estop 은 항상 false.
"""
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float64, Int32

from ssf_vrx.convert import (command_is_stale, decode_motor_run, heading_from_quat,
                             pwm_to_thrust)

STATUS_QOS = 10   # ssf_bridge 와 같음

DEFAULT_JOINTS = {
    'A': (['left_engine_propeller_joint'], ['right_engine_propeller_joint']),
    'B': (['rear_left_engine_propeller_joint', 'front_left_engine_propeller_joint'],
          ['rear_right_engine_propeller_joint', 'front_right_engine_propeller_joint']),
}


def cmd_topics_for(model_name, joints):
    """gz Thruster 시스템의 기본 명령 토픽: /model/<모델>/joint/<조인트>/cmd_thrust."""
    return [f'/model/{model_name}/joint/{j}/cmd_thrust' for j in joints]


class SimBridge(Node):

    def __init__(self):
        super().__init__('sim_bridge')
        p = self.declare_parameter
        self.boat = str(p('boat', 'A').value).upper()
        if self.boat not in DEFAULT_JOINTS:
            raise RuntimeError(f"boat 파라미터는 A 또는 B 여야 한다: {self.boat!r}")
        self.model_name = str(p('model_name', 'ssf_boat').value)
        left_j, right_j = DEFAULT_JOINTS[self.boat]
        self.left_topics = list(p('left_cmd_topics',
                                  cmd_topics_for(self.model_name, left_j)).value)
        self.right_topics = list(p('right_cmd_topics',
                                   cmd_topics_for(self.model_name, right_j)).value)
        self.fwd_max_n = float(p('fwd_max_n', 47.0).value)
        self.rev_max_n = float(p('rev_max_n', 35.0).value)
        self.deadband = int(p('deadband_pwm', 25).value)
        self.cmd_timeout = float(p('cmd_timeout_sec', 0.5).value)
        self.boat_mode = int(p('boat_mode', 2).value)          # 0 WAIT 1 MANUAL 2 AUTO
        imu_topic = str(p('imu_topic', '/vrx/imu').value)
        yaw_raw_topic = str(p('yaw_raw_topic', '/imu/yaw_raw').value)
        thrust_hz = float(p('thrust_rate_hz', 20.0).value)
        status_hz = float(p('status_rate_hz', 10.0).value)

        self.last_cmd_t = None
        self.pwm_l = self.pwm_r = 1500
        self.watchdog = True

        self.pub_left = [self.create_publisher(Float64, t, 10) for t in self.left_topics]
        self.pub_right = [self.create_publisher(Float64, t, 10) for t in self.right_topics]
        self.pub_mode = self.create_publisher(Int32, '/boat_mode', STATUS_QOS)
        self.pub_watchdog = self.create_publisher(Bool, '/boat_cmd_watchdog', STATUS_QOS)
        self.pub_estop = self.create_publisher(Bool, '/boat_estop', STATUS_QOS)
        self.pub_boat_id = self.create_publisher(Int32, '/boat_id', STATUS_QOS)
        self.pub_yaw_raw = self.create_publisher(Float64, yaw_raw_topic, 10)

        self.create_subscription(Int32, 'Motor_run', self._on_motor_run, 10)
        self.create_subscription(Imu, imu_topic, self._on_imu, 10)
        self.create_timer(1.0 / thrust_hz, self._thrust_tick)
        self.create_timer(1.0 / status_hz, self._status_tick)

        self.get_logger().info(
            f"sim_bridge boat={self.boat} model={self.model_name} "
            f"L={self.left_topics} R={self.right_topics} "
            f"fwd_max={self.fwd_max_n} N rev_max={self.rev_max_n} N dead=±{self.deadband} "
            f"cmd_timeout={self.cmd_timeout}s boat_mode={self.boat_mode} "
            f"imu {imu_topic} → {yaw_raw_topic}")

    # ---- Motor_run → 추력 ----
    def _on_motor_run(self, msg):
        self.pwm_l, self.pwm_r = decode_motor_run(msg.data)
        self.last_cmd_t = time.monotonic()

    def _thrust_tick(self):
        stale = command_is_stale(time.monotonic(), self.last_cmd_t, self.cmd_timeout)
        if stale != self.watchdog:
            self.watchdog = stale
            if stale:
                self.get_logger().warn(
                    f"Motor_run {self.cmd_timeout}s 무수신 → 추력 0 (펌웨어 워치독과 같음)")
            else:
                self.get_logger().info("Motor_run 수신 → 추력 재개")
        if stale:
            t_l = t_r = 0.0
        else:
            t_l = pwm_to_thrust(self.pwm_l, self.fwd_max_n, self.rev_max_n, self.deadband)
            t_r = pwm_to_thrust(self.pwm_r, self.fwd_max_n, self.rev_max_n, self.deadband)
        for pub in self.pub_left:
            pub.publish(Float64(data=float(t_l)))
        for pub in self.pub_right:
            pub.publish(Float64(data=float(t_r)))

    # ---- 상태줄 대역 ----
    def _status_tick(self):
        self.pub_mode.publish(Int32(data=self.boat_mode))
        self.pub_watchdog.publish(Bool(data=bool(self.watchdog)))
        self.pub_estop.publish(Bool(data=False))
        self.pub_boat_id.publish(Int32(data=0 if self.boat == 'A' else 1))

    # ---- IMU → yaw_raw ----
    def _on_imu(self, msg):
        q = msg.orientation
        self.pub_yaw_raw.publish(Float64(data=heading_from_quat(q.x, q.y, q.z, q.w)))


def main(args=None):
    rclpy.init(args=args)
    node = SimBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 때 추력 0 — 실물 ssf_bridge 의 중립 송신과 같은 뜻
        try:
            for pub in node.pub_left + node.pub_right:
                pub.publish(Float64(data=0.0))
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
