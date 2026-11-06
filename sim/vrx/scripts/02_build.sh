#!/usr/bin/env bash
# VRX 시뮬 2단계 — VRX(humble) 와 ssf_vrx 빌드 (sudo 없음, SSH 로 돌려도 됨).
#   bash ~/ros2_ws/sim/vrx/scripts/02_build.sh
set -euo pipefail
source /opt/ros/humble/setup.bash

echo "== [1/3] ros2_ws 가 빌드돼 있어야 한다"
if [ ! -f ~/ros2_ws/install/setup.bash ]; then
  echo "~/ros2_ws/install 이 없다 → cd ~/ros2_ws && colcon build --symlink-install 먼저"; exit 1
fi
source ~/ros2_ws/install/setup.bash

echo "== [2/3] VRX humble 빌드 (~/vrx_ws, 처음은 10~30분)"
mkdir -p ~/vrx_ws/src
[ -d ~/vrx_ws/src/vrx ] || git clone -q -b humble https://github.com/osrf/vrx.git ~/vrx_ws/src/vrx
cd ~/vrx_ws
if [ ! -f install/setup.bash ]; then
  colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -5
else
  echo "이미 빌드됨 (다시 하려면 rm -rf ~/vrx_ws/build ~/vrx_ws/install)"
fi
source ~/vrx_ws/install/setup.bash

echo "== [3/3] ssf_vrx 빌드 (~/ros2_ws/sim/vrx)"
cd ~/ros2_ws/sim/vrx
colcon build --symlink-install 2>&1 | tail -3
source install/setup.bash

echo
echo "== 확인 (셋 다 경로가 나와야 한다)"
ros2 pkg prefix vrx_gz
ros2 pkg prefix ros_gz_bridge
ros2 pkg prefix ssf_vrx
ls "$(ros2 pkg prefix ssf_vrx)/share/ssf_vrx/worlds/"
echo "끝. 다음: bash ~/ros2_ws/sim/vrx/scripts/03_smoke.sh A"
