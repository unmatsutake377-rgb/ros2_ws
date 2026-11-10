#!/usr/bin/env python3
"""GPS·RTK 상태 감시 — 배 노트북에서 /ublox_gps_node/navpvt 를 **구독만** 하고 상태를 파일로 남긴다.

  파라미터 앱(ROS 없이 돈다)이 이 파일을 읽어 위쪽 'RTK' 불을 켠다. tools/ssf_gps.py 가 tmux(ssf_gps_<배>)로 띄운다.
  python3 tools/ssf_gps_watch.py <A|B>      (ROS 환경 + tools/ssf_ros_net.sh 격리 뒤 — tools/ssf_gps_watch.sh)

- 발행 없음. 배 동작에 영향 없음.
- 출력: ~/ssf_logs/gps_status_<배>.json  {t, carr(0 없음·1 FLOAT·2 FIXED), fix_ok, fix_type, h_acc_m, num_sv, lat, lon}
  쓰기는 임시 파일 → 이름 바꾸기(읽는 쪽이 반쯤 쓴 파일을 보지 않게). 초당 최대 2번.
- 앱이 2 분 넘게 안 보면(~/ssf_logs/.gps_watch_<배>.hb 가 오래됨) 스스로 끝난다 — 남는 노드 없음.
- 판정 근거: ublox_gps 드라이버(ublox_firmware7plus.hpp)와 같은 NavPVT flags — CARRIER_PHASE 비트(192) 128=FIXED, 64=FLOAT.
"""
import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from ublox_msgs.msg import NavPVT

HB_MAX_SEC = float(os.environ.get("SSF_GPS_HB_MAX", "120"))   # 시험 때만 줄인다


class GpsWatch(Node):
    def __init__(self, boat):
        super().__init__("ssf_gps_watch")
        home = os.path.expanduser("~")
        d = os.path.join(home, "ssf_logs")
        os.makedirs(d, exist_ok=True)
        self.out = os.path.join(d, f"gps_status_{boat}.json")
        self.hb = os.path.join(d, f".gps_watch_{boat}.hb")
        self.last_write = 0.0
        self._write({"t": time.time(), "waiting": True})   # 지난 오류·옛 값을 지운다(첫 navpvt 전까지 '기다리는 중')
        topic = os.environ.get("SSF_GPS_TOPIC", "/ublox_gps_node/navpvt")
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(NavPVT, topic, self._cb, qos)
        self.create_timer(5.0, self._check_hb)
        self.get_logger().info(f"구독 전용: {topic} → {self.out}")

    def _cb(self, m):
        now = time.time()
        if now - self.last_write < 0.5:
            return
        carr_bits = m.flags & m.FLAGS_CARRIER_PHASE_MASK
        carr = 2 if carr_bits & m.CARRIER_PHASE_FIXED else (1 if carr_bits & m.CARRIER_PHASE_FLOAT else 0)
        data = {"t": now, "carr": carr, "fix_ok": bool(m.flags & m.FLAGS_GNSS_FIX_OK), "fix_type": int(m.fix_type),
                "h_acc_m": m.h_acc / 1000.0, "num_sv": int(m.num_sv), "lat": m.lat * 1e-7, "lon": m.lon * 1e-7}
        self._write(data)
        self.last_write = now

    def _write(self, data):
        tmp = self.out + f".tmp{os.getpid()}"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(data, f)
        os.replace(tmp, self.out)

    def _check_hb(self):
        try:
            age = time.time() - os.path.getmtime(self.hb)
        except OSError:
            age = HB_MAX_SEC + 1
        if age > HB_MAX_SEC:
            self.get_logger().info("앱이 2 분 넘게 안 봄 — 끝낸다")
            raise SystemExit(0)


def main():
    boat = (sys.argv[1] if len(sys.argv) > 1 else "").upper()
    if boat not in ("A", "B"):
        print("사용: ssf_gps_watch.py <A|B>")
        return 2
    rclpy.init()
    node = GpsWatch(boat)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
