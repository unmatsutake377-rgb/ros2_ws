#!/usr/bin/env bash
# 파라미터 앱 'launch' 탭이 tmux 세션(ssf_launch_<배>) 안에서 돌린다 — tools/ssf_launch.py.
#
#   bash tools/ssf_launch_run.sh <A|B> <main|tournament> [웨이포인트 파일 절대경로]
#
# 손으로 칠 때와 같은 명령이다:
#   ros2 launch launch_files launch_files.launch.py boat:=<배> course:=<코스> [waypoints_file:=<파일>]
# 다른 점 두 가지:
#   · tmux 는 비대화형이라 ~/.bashrc 의 ROS 격리(ROS_LOCALHOST_ONLY)가 안 읽힌다 → tools/ssf_ros_net.sh 로 같은 값을 넣는다.
#   · launch 가 끝나도(Ctrl+C·오류) 창을 닫지 않는다(sleep) — 마지막 로그를 앱에서 볼 수 있게.
# 🚨 끄기는 Ctrl+C(SIGINT) 한 번만 — 라이다가 정지 명령을 받는다. 강제 종료 금지(boat_launch_checklist §4).
# 이 파일 이름·인자에 'launch_files.launch.py' 를 넣지 않는다 — 'launch 실행 중' 판정(pgrep)이 이 셸을 launch 로 착각한다.
set -u
boat=${1:?배}
course=${2:?코스}
wp=${3:-}
cd "$(dirname "$0")/.." || exit 1

args=("boat:=$boat" "course:=$course")
[ -n "$wp" ] && args+=("waypoints_file:=$wp")

if [ -n "${SSF_LAUNCH_FAKE:-}" ]; then            # 테스트 전용 — ROS 없이 가짜 launch
  launch=(bash "$SSF_LAUNCH_FAKE")
else
  setup=$(ls /opt/ros/*/setup.bash 2>/dev/null | tail -1)
  if [ -z "$setup" ] || [ ! -f install/setup.bash ]; then
    echo "❌ ROS(/opt/ros/*/setup.bash) 또는 install/setup.bash 가 없다 — launch 못 함"
    exec sleep infinity
  fi
  # shellcheck disable=SC1090
  source "$setup"
  # shellcheck disable=SC1091
  source install/setup.bash
  launch=(ros2 launch launch_files "launch_files.launch.py")
fi
# shellcheck disable=SC1091
source tools/ssf_ros_net.sh && ssf_ros_net

echo "▶ $(date +%T) 앱에서 시작: ${launch[*]} ${args[*]}"
trap : INT                    # Ctrl+C 는 launch 가 받아 정리하고, 이 셸은 남아 '끝' 줄을 찍는다
"${launch[@]}" "${args[@]}"
rc=$?
trap - INT
echo "■ $(date +%T) launch 끝 (코드 $rc). 다시 켜기는 앱 'launch' 탭에서."
exec sleep infinity
