#!/bin/bash
# 센서 없는 벤치: 노드를 저장소 yaml 로 띄우고, 가짜 토픽을 넣어 출력을 기록한다(ssf_bridge 안 띄움 → 아두이노 안 씀).
# 사용: tools/bench_fake/run_bench.sh <시나리오.py> <로그이름> <노드[|추가 ros-args]> ...
#   예: tools/bench_fake/run_bench.sh s3_side_escape.py s3 "ship_direction|-p side_escape_deg:=60.0" ship_back
# 결과: ${BENCH_OUT:-/tmp/ssf_bench}/<로그이름>.txt (+ 노드별 로그). ROS_DOMAIN_ID 78(대회 스택과 분리).
# /yaw_error 토픽 부호: 노드 안에서 360−값 → 토픽 −20 = 목표 왼쪽 20°(테스트의 내부 yaw 20).
B=$(cd "$(dirname "$0")" && pwd)
OUT=${BENCH_OUT:-/tmp/ssf_bench}
SC=$1; NAME=$2; shift 2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=78
export PYTHONUNBUFFERED=1
mkdir -p "$OUT"
PIDS=()
for spec in "$@"; do
  node=${spec%%|*}; extra=""
  [[ "$spec" == *"|"* ]] && extra=${spec#*|}
  yaml=~/ros2_ws/install/$node/share/$node/config/$node.yaml
  # ros2 run 을 거치지 않고 실행파일을 직접 띄운다 → $! 가 노드 자체라 SIGINT 로 깔끔히 끝난다.
  ~/ros2_ws/install/$node/lib/$node/$node --ros-args --params-file $yaml $extra > "$OUT/${NAME}_$node.log" 2>&1 &
  PIDS+=($!)
done
sleep 3
python3 "$B/$SC" "$OUT/${NAME}.txt"
for p in "${PIDS[@]}"; do kill -INT $p 2>/dev/null; done
sleep 2
for p in "${PIDS[@]}"; do kill -0 $p 2>/dev/null && echo "아직 살아있음 $p" && kill -TERM $p; done
wait 2>/dev/null
echo "끝 ($OUT/${NAME}.txt)"
