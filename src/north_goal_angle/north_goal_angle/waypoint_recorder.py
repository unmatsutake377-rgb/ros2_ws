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

입력 안내(2026-11-04, 계약서_waypoint_recorder안내):
  · 왜: 코드를 모르는 팀원이 대회장에서 **엔터만 눌러도** 미션 순서·mode·dwell 이 맞게 들어가게 한다.
  · 시작 화면에 mode 번호표 + 템플릿(기본 share 의 config/waypoints.yaml) 경로·개수를 보여 준다.
  · 캡처마다 "다음 예상: WP{i} 구역 mode dwell" 을 템플릿에서 뽑아 기본값으로 묻는다(엔터 = 기본).
  · 틀린 번호(4·6·9)·음수 dwell 은 캡처 취소가 아니라 **재질문**(위치 평균 유지). 최종 검증(is_valid)은 그대로.
  · q 저장 때 템플릿과 개수·순서가 다르면 경고만 하고 저장은 항상 한다.
  · 템플릿 경로: `--template=/경로.yaml`. 못 읽으면 "템플릿 없음(사유)" 를 드러내고 기본값 없이 진행한다.

순수 로직(average_fixes / format_waypoints_yaml / parse_cli)은 ROS 없이 테스트한다(test_waypoint_recorder.py).
안내·질문 로직은 recorder_prompts.py(test_recorder_prompts.py). 실제 GPS 구독·키 입력은 Ubuntu(ROS2)에서만 동작한다.
"""

import argparse
import os
import sys
import threading
import time

from north_goal_angle.recorder_prompts import (
    ask_dwell, ask_mode, compare_with_template, describe_template, dwell_default,
    load_template, mode_menu, next_expected_line, template_defaults,
)

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


def strip_ros_args(argv):
    """`--ros-args ... [--]` 구간을 떼어낸다(rclpy 의 remove_ros_args 와 같은 규칙, rclpy 없이).

    예전엔 sys.argv[1] 을 그대로 써서 `ros2 run ... waypoint_recorder --ros-args -p x:=1` 이면
    `--ros-args` 가 **출력 경로**가 됐다. argparse 만으로는 `-p` 뒤의 `x:=1` 이 위치 인자로 잡히므로
    `--ros-args` 부터 다음 `--` 까지를 먼저 통째로 버린다.
    """
    out = []
    skipping = False
    for a in argv:
        if skipping:
            if a == "--":
                skipping = False
            continue
        if a == "--ros-args":
            skipping = True
            continue
        out.append(a)
    return out


def parse_cli(argv):
    """명령줄(프로그램명 제외) → Namespace(out_path, template, unknown). 순수 함수(테스트 대상).

    · out_path: 위치 인자(기존 그대로). 없으면 None → main 이 ~/ssf_logs/waypoints_recorded.yaml.
    · --template=<경로>: 기본값 템플릿 yaml. 없으면 None → main 이 share 의 config/waypoints.yaml.
    · parse_known_args: 모르는 옵션(예: 미래의 ROS 인자)은 unknown 에 담아 경고만 하고 죽지 않는다.
    """
    p = argparse.ArgumentParser(
        prog="waypoint_recorder", add_help=True,
        description="RC 로 몰고 다니며 현재 GPS 를 waypoints yaml 로 기록한다.")
    p.add_argument("out_path", nargs="?", default=None,
                   help="출력 yaml 경로(기본 ~/ssf_logs/waypoints_recorded.yaml)")
    p.add_argument("--template", default=None,
                   help="기본값 템플릿 yaml(기본: 패키지 share 의 config/waypoints.yaml)")
    ns, unknown = p.parse_known_args(strip_ros_args(list(argv)))
    ns.unknown = unknown
    return ns


def resolve_template_path(explicit=None):
    """템플릿 경로 결정: --template > share config/waypoints.yaml > 소스 config/waypoints.yaml > None.

    share 를 먼저 보는 건 노드(north_goal_angle.py)가 읽는 파일과 같은 것을 기준으로 삼기 위해서다.
    재빌드를 안 해 share 가 옛 파일일 수 있으므로 시작 화면에 **절대경로·개수**를 찍어 드러낸다.
    """
    if explicit:
        return os.path.abspath(explicit)
    candidates = []
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(os.path.join(
            get_package_share_directory('north_goal_angle'), 'config', 'waypoints.yaml'))
    except Exception:  # ament 없음(Mac)·패키지 미설치 → 소스 경로로
        pass
    candidates.append(os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'waypoints.yaml'))
    for c in candidates:
        if os.path.isfile(c):
            return os.path.abspath(c)
    return None


# ---------------- ROS 노드 (Ubuntu 에서만 실행) ----------------

def _run_node(out_path, avg_window_sec, cov_warn, template_path=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import NavSatFix

    # 저장 직전 검증에 우리 로더를 그대로 쓴다(범위·mode 체크 일관).
    from north_goal_angle.waypoint_loader import parse_waypoints, WaypointError, VALID_MODES

    # 템플릿(기본값 출처). 못 읽으면 None + 사유 — 시작 화면에 그대로 드러낸다(그럴듯한 기본값을 안 만든다).
    template, template_reason = load_template(template_path)

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

    def capture_position():
        """현재 위치를 평균해서 (lat, lon) 반환. fix 없으면 None."""
        recent = node.snapshot()
        if not recent:
            print("  ⚠️ 최근 GPS fix 없음(status<0 또는 무수신). 취소 — GPS 상태 확인.")
            return None
        avg = average_fixes([(b[1], b[2]) for b in recent])
        worst_cov = max(b[4] for b in recent)
        print(f"  표본 {len(recent)}개 평균 → lat {avg[0]:.7f}, lon {avg[1]:.7f}"
              f" (최대 공분산 {worst_cov:.2f})")
        if worst_cov > cov_warn:
            print(f"  ⚠️ 공분산 높음(>{cov_warn}) — RTK 불안정일 수 있음. 그래도 쓰려면 진행.")
        return avg

    def is_valid(lat, lon, mode, dwell):
        """저장 전 범위·mode 검증(우리 loader 재사용)."""
        try:
            parse_waypoints({"waypoints": [
                {"lat": lat, "lon": lon, "mode": mode, "dwell": dwell}]})
            return True
        except WaypointError as e:
            print(f"  🚨 검증 실패: {e}")
            return False

    def list_wps():
        if not node.recorded:
            print("  (아직 기록 없음)")
            return
        for i, w in enumerate(node.recorded):
            print(f"  [{i}] {w[4] or f'WP{i}'}  lat {w[0]:.7f} lon {w[1]:.7f}"
                  f"  mode {int(w[2])}  dwell {w[3]}")

    def parse_idx(arg):
        """'d 2' / 'r0' 등에서 인덱스 파싱. 범위 밖이면 None."""
        try:
            idx = int(arg)
        except ValueError:
            print("  번호를 붙이세요 (예: d 2, r 0)")
            return None
        if not (0 <= idx < len(node.recorded)):
            print(f"  범위 밖 — 0~{len(node.recorded) - 1} 사이")
            return None
        return idx

    print("\n=== 좌표 기록 시작 ===")
    print("RC 로 지점에 배를 대고 조작하세요. 명령:")
    print("  Enter  = 현재 위치를 새 WP 로 추가")
    print("  l      = 기록 목록 보기")
    print("  d N    = N 번 WP 삭제")
    print("  r N    = N 번 WP 를 현재 위치로 재캡처(위치만 갱신, mode/dwell 유지)")
    print("  q      = 저장·종료\n")
    print(mode_menu(VALID_MODES))
    print(describe_template(template_path, template, template_reason))
    print("  (캡처마다 템플릿 순서대로 기본값을 보여 준다 — 맞으면 엔터, 다르면 번호를 직접 입력)")
    print(next_expected_line(template, len(node.recorded)) + "\n")
    try:
        while True:
            raw = input("[Enter=추가 / l=목록 / d N=삭제 / r N=재캡처 / q=종료] > ").strip()
            low = raw.lower()

            if low == 'q':
                break
            if low == 'l':
                list_wps()
                continue
            if low.startswith('d'):
                idx = parse_idx(low[1:].strip())
                if idx is not None:
                    removed = node.recorded.pop(idx)
                    print(f"  🗑 [{idx}] {removed[4] or f'WP{idx}'} 삭제 (남은 {len(node.recorded)}개)")
                continue
            if low.startswith('r'):
                idx = parse_idx(low[1:].strip())
                if idx is None:
                    continue
                avg = capture_position()
                if not avg:
                    continue
                w = node.recorded[idx]
                if not is_valid(avg[0], avg[1], w[2], w[3]):
                    continue
                node.recorded[idx] = [avg[0], avg[1], w[2], w[3], w[4]]
                print(f"  ↻ [{idx}] 위치 갱신됨 (mode/dwell 유지)")
                continue
            if raw == '':
                avg = capture_position()
                if not avg:
                    continue
                # 기본값은 매번 현재 개수(i = len(recorded)) 기준 — 'd N' 삭제로 인덱스가 당겨져도 맞는다.
                i = len(node.recorded)
                d = template_defaults(template, i)
                d_label, d_mode, d_dwell = d if d is not None else ("", None, None)
                print("  " + next_expected_line(template, i))
                # 틀린 번호·음수는 안에서 재질문한다(캡처 취소 아님 — 위치 평균 avg 는 그대로 쓴다).
                mode = ask_mode(input, VALID_MODES, d_mode)
                # 템플릿 dwell 은 템플릿 mode 를 그대로 받았을 때만 기본값. mode 를 바꿨으면 그 mode 의 폴백표.
                dwell = ask_dwell(input, dwell_default(mode, d_dwell if mode == d_mode else None))
                label_hint = d_label if d_label else f"자동 WP{i}"
                label = input(f"  구역 이름(엔터={label_hint})? ").strip() or d_label
                if not is_valid(avg[0], avg[1], mode, dwell):
                    continue
                node.recorded.append([avg[0], avg[1], mode, dwell, label])
                print(f"  ✔ WP{len(node.recorded) - 1} 추가됨 (총 {len(node.recorded)}개)")
                print("  " + next_expected_line(template, len(node.recorded),
                                                prefix="다음에 찍을 곳") + "\n")
                continue
            print("  알 수 없는 명령 — Enter/l/d N/r N/q")
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
        # 템플릿과 개수·순서(mode) 비교 — 경고만, 저장은 항상(계약서 §9-1). 'd N' 뒤 최종 목록 기준.
        for w in compare_with_template(node.recorded, template):
            print(f"⚠️ 템플릿과 다름: {w}")
        text = format_waypoints_yaml(node.recorded)
        with open(out_path, "w", encoding="utf-8") as f:
            f.write(text)
        print("\n" + "=" * 66)
        print(f"💾 {len(node.recorded)}개 저장 완료")
        print(f"   경로: {out_path}")
        print("=" * 66)
        print("   ▶ 복사하지 않고 바로 쓰려면 (기존 waypoints.yaml 을 안 건드린다):")
        print("       ros2 launch north_goal_angle north_goal_angle_launch.py \\")
        print(f"           waypoints_file:={out_path}")
        print("   ▶ 이걸 실전 파일로 확정하려면 config/waypoints.yaml 로 복사한 뒤 재빌드.")
    else:
        print("\n(기록된 웨이포인트 없음 — 저장 안 함)")

    node.destroy_node()
    rclpy.shutdown()


def main():
    # 🚨 기본 출력은 **고정 경로**다 (~/ssf_logs/waypoints_recorded.yaml).
    #    예전엔 os.getcwd() 였는데, 실행한 폴더에 따라 파일이 어디 생길지 모른다.
    #    배는 SSH 로 붙어 쓰므로 접속할 때마다 cwd 가 달라지고, 대회장에서 급할 때
    #    "찍긴 찍었는데 파일이 어디 있지" 가 된다 (2026-08-07 야외 시험에서 실제로 겪음 —
    #    세 군데를 뒤졌다). blackbox 로그(~/ssf_logs/)와 같은 자리에 모은다.
    #    다른 곳에 쓰려면 인자로 경로를 준다:  ros2 run ... waypoint_recorder /경로.yaml
    #    기본값 템플릿을 바꾸려면:            ros2 run ... waypoint_recorder --template=/경로.yaml
    #    (`--ros-args ...` 는 parse_cli 가 떼어내므로 출력 경로로 오인되지 않는다.)
    cli = parse_cli(sys.argv[1:])
    if cli.unknown:
        print(f"⚠️ 모르는 인자 무시: {cli.unknown}")
    default_dir = os.path.join(os.path.expanduser("~"), "ssf_logs")
    os.makedirs(default_dir, exist_ok=True)
    default_out = os.path.join(default_dir, "waypoints_recorded.yaml")
    out_path = os.path.abspath(cli.out_path if cli.out_path else default_out)
    template_path = resolve_template_path(cli.template)
    _run_node(out_path, avg_window_sec=2.0, cov_warn=2.0, template_path=template_path)


if __name__ == "__main__":
    main()
