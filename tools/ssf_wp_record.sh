#!/usr/bin/env bash
# 배 노트북에서 웨이포인트 녹화 도구를 띄운다 — MacBook 의 'ssf b wp main' / 'ssf b wp tournament 1' 이 tmux 안에서 부른다.
#
#   tools/ssf_wp_record.sh <A|B> main
#   tools/ssf_wp_record.sh <A|B> tournament <레인>
#
# 하는 일: 배 확인(~/.ssf_boat) → 출력·템플릿 경로 결정 → 기존 녹화본 백업(~/ssf_logs/waypoint_backup/)
#          → ROS 환경 → ros2 run north_goal_angle waypoint_recorder → 끝나면 검사 결과.
# 절차·주의: docs/절차/웨이포인트_녹화절차.md
set -u
boat=${1:?A 또는 B}
course=${2:?main 또는 tournament}
lane=${3:-}
cd "$(dirname "$0")/.." || exit 1
PAUSED=0
pause() { PAUSED=1; echo; read -rp "엔터를 누르면 이 창을 닫는다 " _; }
# 어디서 끝나든 창이 말없이 닫히지 않게(tmux 창이 닫히면 SSH 도 바로 끊겨 오류를 못 본다)
trap 'rc=$?; if [ "$PAUSED" = 0 ]; then echo; echo "❌ 중간에 끝났다(코드 $rc) — 위 문구를 확인"; read -rp "엔터를 누르면 닫는다 " _; fi' EXIT

prep=$(python3 tools/ssf_param_core.py --boat "$boat" wp-prep "$course" $lane 2>&1)
rc=$?
echo "$prep" | grep -v '^WP_'
[ $rc -eq 0 ] || { pause; exit 1; }
OUT=$(echo "$prep" | sed -n 's/^WP_OUT=//p')
TEMPLATE=$(echo "$prep" | sed -n 's/^WP_TEMPLATE=//p')

setup=$(ls /opt/ros/*/setup.bash 2>/dev/null | tail -1)
if [ -z "$setup" ] || [ ! -f install/setup.bash ]; then
  echo "❌ ROS 환경이 없다(/opt/ros/*/setup.bash 또는 ~/ros2_ws/install/setup.bash) — 녹화 도구를 못 띄운다"
  pause; exit 1
fi
# ROS setup.bash 는 정의 안 된 변수(AMENT_TRACE_SETUP_FILES 등)를 쓴다 — set -u 인 채로 부르면 바로 죽는다
set +u
# shellcheck disable=SC1090
source "$setup"; source install/setup.bash
if ! command -v ros2 >/dev/null; then echo "❌ ROS 환경을 불러왔는데 ros2 명령이 없다"; pause; exit 1; fi

echo
echo "━━ 웨이포인트 녹화 — $( [ "$boat" = A ] && echo 1번배 || echo 2번배 ) · $( [ "$course" = tournament ] && echo "토너먼트 레인 $lane" || echo 종합임무 )"
echo "   저장 파일: $OUT"
[ -n "$TEMPLATE" ] && echo "   템플릿:   $TEMPLATE"
echo "   ① 먼저 RTK FIXED 확인(다른 창): ros2 topic echo /ublox_gps_node/navpvt --once  → carrSoln 2"
echo "   ② 조종기로 지점에 대고 몇 초 머문 뒤 Enter → mode → dwell → 이름. l=목록 d N=삭제 r N=재캡처 q=저장"
if [ "$course" = tournament ]; then
  echo "   ③ 🚨 전부 mode 8. 마지막 점 = 결승선 너머 약 3 m, dwell 0.5 — 경기장 경계 안"
else
  echo "   ③ 🚨 WP7 다음 점(토너먼트)은 엔터 금지 — mode 8 · dwell 3 · 이름 'WP10 토너먼트 회피' 직접 입력"
  echo "      도킹 네 줄은 찍지 않는다 — 저장 뒤 tools/dock_wp.py 출력을 WP7 과 WP10 사이에 끼운다"
fi
echo "   SSH 가 끊겨도 tmux 안이라 녹화는 계속된다 — Mac 에서 같은 ssf 명령을 다시 치면 이어 붙는다"
echo
if [ -n "$TEMPLATE" ]; then
  ros2 run north_goal_angle waypoint_recorder "$OUT" --template="$TEMPLATE"
else
  ros2 run north_goal_angle waypoint_recorder "$OUT"
fi

echo
echo "━━ 검사 (앱 '웨이포인트' 탭에서도 보인다)"
python3 tools/ssf_param_core.py --boat "$boat" wp-list | grep -v '^\['
echo
echo "다음: launch 할 때 waypoints_file:=$OUT  (저장소 파일에 복사하지 않는다 · 현장 조정표 §4 에 기록)"
pause
