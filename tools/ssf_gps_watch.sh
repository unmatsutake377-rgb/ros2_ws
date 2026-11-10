#!/usr/bin/env bash
# GPS·RTK 감시 노드를 ROS 환경 + 배 ROS 격리(ROS_LOCALHOST_ONLY)로 띄운다 — tools/ssf_gps.py 가 tmux(ssf_gps_<배>)로 부른다.
#   tools/ssf_gps_watch.sh <A|B>
# 실패하면 이유를 ~/ssf_logs/gps_status_<배>.json 의 "error" 로 남긴다(앱 위쪽 불에 그대로 보인다).
set -u
boat=${1:?A 또는 B}
cd "$(dirname "$0")/.." || exit 1
mkdir -p "$HOME/ssf_logs"
out="$HOME/ssf_logs/gps_status_${boat}.json"
fail() { printf '{"t": %s, "error": "%s"}\n' "$(date +%s)" "$1" > "$out.tmp$$" && mv "$out.tmp$$" "$out"; echo "❌ $1"; exit 1; }

setup=$(ls /opt/ros/*/setup.bash 2>/dev/null | tail -1)
[ -n "$setup" ] && [ -f install/setup.bash ] || fail "ROS 환경이 없다(/opt/ros 또는 ~/ros2_ws/install)"
set +u   # ROS setup.bash 는 정의 안 된 변수를 쓴다
# shellcheck disable=SC1090
source "$setup"; source install/setup.bash
# shellcheck disable=SC1091
source tools/ssf_ros_net.sh && ssf_ros_net     # 다른 배 GPS 를 보지 않게 — ~/.bashrc 와 같은 격리
python3 -c "import ublox_msgs.msg" 2>/dev/null || fail "ublox_msgs 를 못 불러온다 — colcon build 확인"
exec python3 tools/ssf_gps_watch.py "$boat"
