#!/usr/bin/env bash
# 배 노트북: D455 영상 송신을 tmux 안에서 돌린다 — MacBook 의 'ssf b cam' 과 파라미터 앱 '영상' 탭이 부른다.
#
#   tools/ssf_cam_send.sh <A|B> <MacBook Tailscale IP>
#
# 하는 일: 배 확인(~/.ssf_boat) → 모드 고르기(카메라 노드가 D455 를 쥐고 있으면 ros, 아니면 v4l2)
#          → 배별 포트(A번배 5600 · B번배 5601 — 두 배를 같은 MacBook 으로 보내도 안 섞이게)
#          → tools/cam_stream.sh 를 그대로 실행, 출력은 ~/ssf_logs/cam_stream_<배>.log 에도 남긴다.
# 영상은 조종과 완전히 따로다 — 이 스크립트가 죽어도 RC 조종은 그대로. 설정 파일은 건드리지 않는다.
# 절차·문제 해결: docs/절차/실시간영상_빠른설명서.md
set -u
boat=${1:?A 또는 B}
host=${2:?MacBook Tailscale IP}
cd "$(dirname "$0")/.." || exit 1
mkdir -p "$HOME/ssf_logs"
log="$HOME/ssf_logs/cam_stream_${boat}.log"
fail() { echo "❌ $(date '+%F %T') $*" | tee "$log"; exit 1; }   # 창이 바로 닫혀도 앱·status 에서 보이게 로그에

mk=$(tr -d '[:space:]' < "$HOME/.ssf_boat" 2>/dev/null | tr ab AB)
[ "$mk" = "$boat" ] || fail "🚨 배가 다르다: 요청 ${boat}, 이 노트북 등록 '${mk:-없음}'(~/.ssf_boat) — SSH 별명을 확인"
case "$host" in *[!0-9.]*|"") fail "IP 형식이 아니다: $host";; esac

port=5600; [ "$boat" = B ] && port=5601
mode=v4l2
if pgrep -f realsense2_camera >/dev/null 2>&1; then
  mode=ros     # 카메라 노드가 D455 를 쥐고 있다 → 토픽 경유(Device busy 방지)
  setup=$(ls /opt/ros/*/setup.bash 2>/dev/null | tail -1)
  set +u   # ROS setup.bash 는 정의 안 된 변수를 쓴다
  # shellcheck disable=SC1090
  [ -n "$setup" ] && source "$setup" && [ -f install/setup.bash ] && source install/setup.bash
  set -u
fi
# ros 모드 구독이 다른 배 카메라를 받지 않게 — ~/.bashrc 와 같은 ROS 격리(이 셸에 export, 안내 문구는 로그로)
# shellcheck disable=SC1091
source tools/ssf_ros_net.sh
netmsg=$(ssf_ros_net)      # 문구만 받는다(서브셸)
ssf_ros_net >/dev/null     # 이 셸에 실제로 export
{
  echo "$netmsg"
  echo "━━ $(date '+%F %T') D455 영상 송신 — $( [ "$boat" = A ] && echo A번배 || echo B번배 ) → ${host}:${port} (${mode})"
  echo "   🚨 출항 전 흔들기 시험: dmesg -w 띄우고 카메라·케이블을 흔들어 disconnect 0 (케이블 불량이면 허브째 끊겨 IMU 도 끊긴다)"
  echo "   MacBook 수신: ssf $(echo "$boat" | tr AB ab) cam view   (또는 CAM_PORT=${port} ./tools/cam_view.sh)"
} | tee "$log"
CAM_PORT=$port ./tools/cam_stream.sh "$host" "$mode" 2>&1 | tee -a "$log"
