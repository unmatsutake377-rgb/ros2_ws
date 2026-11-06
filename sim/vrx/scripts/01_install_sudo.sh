#!/usr/bin/env bash
# VRX 시뮬 1단계 — 시스템 패키지 설치 (sudo 필요 → 노트북 터미널에서 직접 실행).
#   bash ~/ros2_ws/sim/vrx/scripts/01_install_sudo.sh
# 하는 일: Gazebo Garden + ROS 2 Humble↔Garden 브릿지 + VRX 빌드 의존 패키지.
# 멱등: 이미 깔려 있으면 그냥 지나간다.
set -euo pipefail

echo "== [1/4] osrfoundation 저장소"
sudo apt-get update -qq
sudo apt-get install -y -qq lsb-release gnupg curl git python3-colcon-common-extensions python3-rosdep
if [ ! -f /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg ]; then
  sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
    --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
fi
if [ ! -f /etc/apt/sources.list.d/gazebo-stable.list ]; then
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
  sudo apt-get update -qq
fi

echo "== [2/4] Gazebo Garden"
sudo apt-get install -y gz-garden

echo "== [3/4] ros-humble-ros-gzgarden (Humble ↔ Garden 브릿지·sim 런처)"
sudo apt-get install -y ros-humble-ros-gzgarden

echo "== [4/4] VRX 소스 의존 (rosdep)"
source /opt/ros/humble/setup.bash
mkdir -p ~/vrx_ws/src
if [ ! -d ~/vrx_ws/src/vrx ]; then
  git clone -q -b humble https://github.com/osrf/vrx.git ~/vrx_ws/src/vrx
fi
sudo rosdep init 2>/dev/null || true
rosdep update -q 2>/dev/null || rosdep update
rosdep install --from-paths ~/vrx_ws/src --ignore-src -r -y

echo
echo "== 확인"
gz sim --version | head -1
dpkg -l | grep -E "ros-humble-ros-gzgarden " | awk '{print $2, $3}'
echo "끝. 다음: bash ~/ros2_ws/sim/vrx/scripts/02_build.sh  (sudo 없음)"
