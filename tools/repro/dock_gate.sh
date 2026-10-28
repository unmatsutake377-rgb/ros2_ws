#!/usr/bin/env bash
# 계약서_도킹게이트_20261028 재현 — 카메라·배 없이 2분. (2026-10-28 Ubuntu 에서 결과 확인)
#   bash tools/repro/dock_gate.sh
# 기대(현재 코드): 2번에서 10000 이 부표 방위와 번갈아 섞이고, 3번(정상 목표)에선 안 섞인다.
# 🚨 오타를 `-p target_shape:=Squre` 로 주면 안 먹는다 — vision.yaml 의 노드별 절이 명령줄 -p 보다 우선한다.
#    그래서 yaml 복사본을 만들어 넘긴다.
# 🚨 정리는 PID 로 한다. `pkill -f` 는 이 스크립트 자신까지 잡는다(10-28 실제로 겪음).
cd "$(dirname "$0")/../.."
source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID_REPRO:-78}     # 실제 배 스택과 섞이지 않게 별도 도메인
YAML=src/color_shape_detector/config/vision.yaml
TYPO=$(mktemp --suffix=.yaml)
sed 's/target_shape: "Square"/target_shape: "Squre"/' "$YAML" > "$TYPO"
grep -q 'target_shape: "Squre"' "$TYPO" || { echo "🚨 오타 yaml 생성 실패 — vision.yaml 의 target_shape 줄 확인"; exit 1; }

PIDS=()
bg(){ "$@" >/dev/null 2>&1 & PIDS+=($!); }
stop(){ for p in "${PIDS[@]}"; do pkill -INT -P "$p" 2>/dev/null; kill -INT "$p" 2>/dev/null; done; sleep 2; PIDS=(); }
trap 'stop; rm -f "$TYPO"' EXIT

case_(){  # 라벨  dock_yaml(빈값=dock 없음)
  bg python3 tools/repro/fake_cam.py --buoy
  bg ros2 run color_shape_detector basic_image_subscriberturn --ros-args --params-file "$YAML"
  [ -n "$2" ] && bg ros2 run color_shape_detector basic_image_subscriberdock --ros-args --params-file "$2"
  sleep 6
  bg ros2 topic pub /wp_mode std_msgs/msg/Int32 "{data: 3}" -r 2
  sleep 3
  python3 tools/repro/count_angle.py 6 "$1"
  stop
}
case_ "1. 모드 3 · turn 만 (기준)"
case_ "2. 모드 3 · turn + dock 오타" "$TYPO"
case_ "3. 모드 3 · turn + dock 정상" "$YAML"
