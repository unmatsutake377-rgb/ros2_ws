#!/usr/bin/env python3
"""mission_monitor — '지금 어떤 미션이고 배가 무슨 모드인가' 한 화면.

    ros2 run ssf_tools mission_monitor

**구독 전용이다. 발행이 하나도 없다.**
그래서 두 대에서 동시에 띄워도 안전하다(healthcheck 는 /health_ok 를 발행하므로
두 대에서 띄우면 발행자가 2개가 된다 — 그건 하면 안 된다).

무선 너머로 볼 것을 전제로 만들었다:
  · **이미지·LaserScan 을 구독하지 않는다.** 스칼라만 본다.
    /scan 은 실측 183KB/s(1.5Mbps)라 링크를 갉아먹고, 그 대역폭은 NTRIP 이 써야 한다.
  · QoS 는 BEST_EFFORT — 늦은 값은 재전송하지 않고 버린다.
  · SSH 로 볼 땐 화면 갱신 글자만 오간다(수 KB/s).

🚨 **묵은 값은 안 보여준다.** 각 항목마다 도착 시각을 재서 stale_sec 을 넘으면 `—` 로 돌린다.
   마지막 값을 계속 띄우면 "3분 전에 자율이었다" 를 "지금 자율이다" 로 읽는다.
   이 프로젝트는 침묵 실패로 이미 여러 번 당했다 — 화면에서도 같은 규칙을 쓴다.
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float32, Int32

from ssf_tools import mission_view

OBSERVER_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# /health_ok 는 healthcheck 가 기본 QoS(RELIABLE)로 낸다 → 맞춰야 받는다.
# (구독자 BEST_EFFORT 는 RELIABLE 발행자와 호환되지만, 여기선 놓치면 안 되는 판정값이라
#  굳이 떨어뜨릴 이유가 없다.)
RELIABLE_QOS = 10


class _Fresh:
    """마지막 값 + 도착 시각. 묵으면 None 을 돌려준다."""

    def __init__(self, stale_sec):
        self._v = None
        self._t = 0.0
        self._stale = stale_sec

    def set(self, v):
        self._v = v
        self._t = time.monotonic()

    def get(self):
        if self._t == 0.0:
            return None                       # 한 번도 안 옴
        if time.monotonic() - self._t > self._stale:
            return None                       # 묵음
        return self._v

    def ever_seen(self):
        return self._t != 0.0


class MissionMonitor(Node):

    def __init__(self):
        super().__init__('mission_monitor')

        period = float(self.declare_parameter("refresh_sec", 0.5).value)
        stale = float(self.declare_parameter("stale_sec", 3.0).value)
        # GPS·미션 값은 1Hz 라 넉넉히 준다(1Hz 를 3초로 재면 정상인데 묵었다고 나온다)
        slow_stale = float(self.declare_parameter("slow_stale_sec", 5.0).value)
        self._clear = bool(self.declare_parameter("clear_screen", True).value)

        gps_topic = self.declare_parameter(
            "gps_topic", "/ublox_gps_node/fix").value

        self.f = {
            "boat_mode": _Fresh(stale),
            "boat_id": _Fresh(stale),
            "watchdog": _Fresh(stale),
            "estop": _Fresh(stale),
            "wp_mode": _Fresh(slow_stale),
            "goal_dist": _Fresh(slow_stale),
            "wp_remain": _Fresh(slow_stale),
            "rtk_sigma": _Fresh(slow_stale),
            "failsafe": _Fresh(stale),
            "gates": _Fresh(slow_stale),
            "health_ok": _Fresh(slow_stale),
        }

        # ── 펌웨어 상태 (ssf_bridge 가 발행) ──
        self._sub(Int32, "/boat_mode", "boat_mode", lambda m: int(m.data))
        self._sub(Int32, "/boat_id", "boat_id", lambda m: int(m.data))
        self._sub(Bool, "/boat_cmd_watchdog", "watchdog", lambda m: bool(m.data))
        self._sub(Bool, "/boat_estop", "estop", lambda m: bool(m.data))

        # ── 미션 진행 ──
        self._sub(Int32, "/wp_mode", "wp_mode", lambda m: int(m.data))
        # /goal_distance·/wp_remaining_time 은 CLAUDE.md 3-8 에 '죽은 토픽(아무도 안 받음)'
        # 으로 적혀 있던 것들이다. 발행은 계속 되고 있었다 — 이 모니터가 첫 소비자다.
        self._sub(Float32, "/goal_distance", "goal_dist", lambda m: float(m.data))
        self._sub(Float32, "/wp_remaining_time", "wp_remain", lambda m: float(m.data))
        self._sub(Int32, "/gates_passed", "gates", lambda m: int(m.data))

        # ── 안전 ──
        self._sub(Int32, "/failsafe_level", "failsafe", lambda m: int(m.data))
        self.create_subscription(NavSatFix, gps_topic, self._cb_gps, OBSERVER_QOS)
        self.create_subscription(Bool, "/health_ok", self._cb_health, RELIABLE_QOS)

        self.create_timer(period, self._draw)
        self.get_logger().info("mission_monitor 시작 — 구독 전용(발행 없음)")

    def _sub(self, msg_type, topic, key, extract):
        def cb(msg):
            self.f[key].set(extract(msg))
        self.create_subscription(msg_type, topic, cb, OBSERVER_QOS)

    def _cb_gps(self, msg):
        # position_covariance[0] = 동쪽 방향 분산(m²). 제곱근이 수평 σ.
        # status<0(fix 없음)이면 값이 의미 없다 → 침묵(gps_guard 와 같은 규칙).
        if msg.status.status < 0:
            return
        cov = msg.position_covariance[0] if len(msg.position_covariance) > 0 else None
        if cov is None or cov <= 0.0:
            return
        self.f["rtk_sigma"].set(cov ** 0.5)

    def _cb_health(self, msg):
        self.f["health_ok"].set(bool(msg.data))

    def _draw(self):
        state = {k: v.get() for k, v in self.f.items()}
        # 브릿지가 한 번이라도 상태를 준 적이 있는가 —
        # '아두이노가 없다' 와 '아두이노가 방금 끊겼다' 는 다른 사건이다.
        state["bridge_seen"] = self.f["boat_mode"].ever_seen()
        out = mission_view.render(state)
        if self._clear:
            # ANSI: 커서 홈 + 화면 지우기. tmux/SSH 에서 정상 동작한다.
            # 파일로 리다이렉트할 땐 clear_screen:=false 로 끈다.
            print("\033[H\033[J", end="")
        print(out, flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = MissionMonitor()
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
