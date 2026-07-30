#!/usr/bin/env python3
"""waypoint_recorder — 대회장에서 RC로 몰고 다니며 현재 GPS를 waypoints yaml 로 기록.

대회 좌표를 미리 못 받는 상황용. 흐름:
  1) '좌표 설정 단계' — GPS 드라이버 + 이 노드만 실행(미션 스택은 끔).
  2) RC 로 각 지점에 배를 대고 → 터미널에서 Enter → 현재 GPS 캡처 → mode/dwell 입력.
  3) 다 찍으면 q → 검증된 yaml 을 **새 파일**(config/waypoints_recorded.yaml)로 저장.
  4) 확인 후 waypoints.yaml 로 옮겨서 '미션 단계' 실행.

설계:
  · **미션 스택과 동시에 안 돈다** — 좌표 설정은 별도 단계라 waypoints 가 비어도 무방.
  · 한 샘플 안 낚는다 — 최근 몇 초 fix 를 평균(지터·순간값 방지), fix 없으면 거부(fail-loud).
  · 저장 전 waypoint_loader.parse_waypoints 로 **즉시 검증**(한국 범위·mode) — 형식도 우리 yaml 그대로.
  · **새 파일에만 쓴다** — 기존 waypoints.yaml 을 건드리지 않아 오입력해도 복구 가능.

순수 로직(average_fixes / format_waypoints_yaml)은 ROS 없이 테스트한다(test_waypoint_recorder.py).
실제 GPS 구독·키 입력은 Ubuntu(ROS2)에서만 동작한다.
"""

import os
import sys
import threading
import time

# ---------------- 순수 로직 (ROS 비의존, 테스트 대상) ----------------

def average_fixes(samples):
    """[(lat, lon), ...] 평균. 빈 리스트면 None. 지터를 줄이려 여러 표본을 평균한다."""
    if not samples:
        return None
    n = len(samples)
    lat = sum(s[0] for s in samples) / n
    lon = sum(s[1] for s in samples) / n
    return lat, lon


def format_waypoints_yaml(waypoints):
    """검증된 [[lat, lon, mode, dwell, label], ...] → waypoints.yaml 문자열.

    waypoint_loader 가 읽는 라벨 dict 형식으로 쓴다(비전공자가 나중에 손편집하기 쉬움).
    pyyaml 없이 직접 포맷한다(의존성 최소 + Mac 에서도 생성 가능).
    """
    lines = ["waypoints:"]
    for i, wp in enumerate(waypoints):
        lat, lon, mode, dwell = wp[0], wp[1], wp[2], wp[3]
        label = wp[4] if len(wp) > 4 and wp[4] else f"WP{i}"
        lines.append(
            f'  - {{구역: "{label}", lat: {lat:.7f}, lon: {lon:.7f}, '
            f'mode: {int(mode)}, dwell: {float(dwell)}}}'
        )
    return "\n".join(lines) + "\n"


# ---------------- ROS 노드 (Ubuntu 에서만 실행) ----------------

def _run_node(out_path, avg_window_sec, cov_warn):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import NavSatFix

    # 저장 직전 검증에 우리 로더를 그대로 쓴다(범위·mode 체크 일관).
    from north_goal_angle.waypoint_loader import parse_waypoints, WaypointError

    class WaypointRecorder(Node):
        def __init__(self):
            super().__init__('waypoint_recorder')
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(NavSatFix, '/ublox_gps_node/fix', self._cb, qos)
            self._buf = []            # 최근 fix (t, lat, lon, status, cov)
            self._lock = threading.Lock()
            self.recorded = []        # 확정된 [[lat, lon, mode, dwell, label], ...]
            self.get_logger().info(
                "waypoint_recorder 시작 — /ublox_gps_node/fix 대기. Enter=캡처, q=저장·종료")

        def _cb(self, msg):
            cov = msg.position_covariance[0] if len(msg.position_covariance) > 0 else 0.0
            with self._lock:
                self._buf.append((time.monotonic(), msg.latitude, msg.longitude,
                                  msg.status.status, cov))
                # 창 크기만 유지
                cut = time.monotonic() - max(avg_window_sec, 1.0) * 3
                self._buf = [b for b in self._buf if b[0] >= cut]

        def snapshot(self):
            """최근 avg_window_sec 내, fix 있는(status>=0) 표본만 반환."""
            now = time.monotonic()
            with self._lock:
                recent = [b for b in self._buf
                          if now - b[0] <= avg_window_sec and b[3] >= 0]
            return recent

    rclpy.init()
    node = WaypointRecorder()

    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    print("\n=== 좌표 기록 시작 ===")
    print("RC 로 지점에 배를 대고 Enter → 캡처. q + Enter → 저장·종료.\n")
    try:
        while True:
            cmd = input("[Enter=캡처 / q=종료] > ").strip().lower()
            if cmd == 'q':
                break
            recent = node.snapshot()
            if not recent:
                print("  ⚠️ 최근 GPS fix 없음(status<0 또는 무수신). 캡처 취소 — GPS 상태 확인.")
                continue
            avg = average_fixes([(b[1], b[2]) for b in recent])
            worst_cov = max(b[4] for b in recent)
            print(f"  표본 {len(recent)}개 평균 → lat {avg[0]:.7f}, lon {avg[1]:.7f}"
                  f" (최대 공분산 {worst_cov:.2f})")
            if worst_cov > cov_warn:
                print(f"  ⚠️ 공분산 높음(>{cov_warn}) — RTK 불안정일 수 있음. 그래도 쓰려면 진행.")
            mode = _ask_int("  mode(미션번호)? ")
            dwell = _ask_float("  dwell(머무는 초)? ")
            label = input("  구역 이름(엔터=자동)? ").strip()

            cand = [[avg[0], avg[1], mode, dwell, label]]
            # 즉시 검증 — 범위·mode 틀리면 저장 안 하고 알린다
            try:
                parse_waypoints({"waypoints": [
                    {"lat": avg[0], "lon": avg[1], "mode": mode, "dwell": dwell}]})
            except WaypointError as e:
                print(f"  🚨 검증 실패 — 기록 안 함: {e}")
                continue
            node.recorded.append(cand[0])
            print(f"  ✔ WP{len(node.recorded) - 1} 기록됨 (총 {len(node.recorded)}개)\n")
    except (EOFError, KeyboardInterrupt):
        pass

    # 저장
    if node.recorded:
        # 전체를 한 번 더 검증(순서·형식)
        try:
            parse_waypoints({"waypoints": [
                {"lat": w[0], "lon": w[1], "mode": w[2], "dwell": w[3]}
                for w in node.recorded]})
        except WaypointError as e:
            print(f"🚨 전체 검증 실패: {e}\n저장은 하되 미션 전 반드시 확인하세요.")
        text = format_waypoints_yaml(node.recorded)
        with open(out_path, "w", encoding="utf-8") as f:
            f.write(text)
        print(f"\n💾 {len(node.recorded)}개 저장 → {out_path}")
        print("   확인 후 config/waypoints.yaml 로 옮기면 미션에서 읽습니다.")
    else:
        print("\n(기록된 웨이포인트 없음 — 저장 안 함)")

    node.destroy_node()
    rclpy.shutdown()


def _ask_int(prompt):
    while True:
        try:
            return int(input(prompt).strip())
        except ValueError:
            print("    정수로 입력하세요.")


def _ask_float(prompt):
    while True:
        try:
            return float(input(prompt).strip())
        except ValueError:
            print("    숫자로 입력하세요.")


def main():
    # 기본 출력: 패키지 config 옆 waypoints_recorded.yaml (실전 파일 안 건드림)
    default_out = os.path.join(os.getcwd(), "waypoints_recorded.yaml")
    out_path = sys.argv[1] if len(sys.argv) > 1 else default_out
    _run_node(out_path, avg_window_sec=2.0, cov_warn=2.0)


if __name__ == "__main__":
    main()
