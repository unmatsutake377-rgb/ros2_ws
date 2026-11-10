# source 로 읽는다 — tmux·SSH 안의 비대화형 bash 에 배 노트북의 ROS 네트워크 격리 설정을 넣는다.
#
# 왜: 각 배 노트북 ~/.bashrc 의 'export ROS_LOCALHOST_ONLY=1'(두 배 토픽 섞임 방지, tools/boat_boot.sh [6])은
#     Ubuntu .bashrc 맨 앞의 'case $- in *i*) ;; *) return;; esac' 때문에 **대화형 터미널에서만** 읽힌다.
#     ssf 가 tmux 로 띄우는 녹화 도구·영상 송신은 비대화형이라 그 설정이 빠진다 → 같은 와이파이의 다른 배
#     /ublox_gps_node/fix 가 섞여 웨이포인트 좌표가 틀어질 수 있다(2026-11-10 검토에서 확인).
# 하는 일: ~/.bashrc 에서 ROS_LOCALHOST_ONLY · ROS_DOMAIN_ID 의 export 줄만 읽어 같은 값으로 export.
#          (.bashrc 를 통째로 실행하지 않는다 — 값은 영문·숫자만 받는다)
#          .bashrc 에 ROS_LOCALHOST_ONLY 가 없으면 1 로 둔다: 스택이 격리 안 됐으면 GPS 를 못 받아 '찍기 실패'로
#          드러난다(틀린 좌표를 조용히 쓰는 것보다 낫다).
ssf_ros_net() {
  local kv found=""
  while read -r kv; do
    [ -n "$kv" ] || continue
    export "$kv"; found="$found $kv"
  done <<EOF_SSF_NET
$(grep -oE '^[[:space:]]*export[[:space:]]+(ROS_LOCALHOST_ONLY|ROS_DOMAIN_ID)=[A-Za-z0-9_]+' "$HOME/.bashrc" 2>/dev/null \
   | sed -E 's/^[[:space:]]*export[[:space:]]+//')
EOF_SSF_NET
  if [ -z "${ROS_LOCALHOST_ONLY:-}" ]; then
    export ROS_LOCALHOST_ONLY=1
    echo "   ⚠️ ~/.bashrc 에 ROS_LOCALHOST_ONLY 가 없다 — 1 로 둔다(두 배 섞임 방지). 스택도 같은 설정인지 tools/boat_boot.sh [6]"
  fi
  echo "   ROS 격리: ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}${ROS_DOMAIN_ID:+ ROS_DOMAIN_ID=$ROS_DOMAIN_ID}  (~/.bashrc 와 같게${found:+ —$found})"
}
