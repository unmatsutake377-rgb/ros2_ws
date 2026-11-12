#!/usr/bin/env bash
# 배 노트북에서 웨이포인트 녹화 도구를 띄운다 — MacBook 의 'ssf b wp main' / 'ssf b wp tournament 1' 이 tmux 안에서 부른다.
#
#   tools/ssf_wp_record.sh <A|B> main
#   tools/ssf_wp_record.sh <A|B> tournament <레인>
#   tools/ssf_wp_record.sh <A|B> practice <gate|hold|search|avoid|dock>   (미션별 연습 — ~/ssf_logs/practice_<미션>.yaml)
#
# 하는 일: 배 확인(~/.ssf_boat) → 출력·템플릿 경로 결정 → 기존 녹화본 백업(~/ssf_logs/waypoint_backup/)
#          → ROS 환경 → ros2 run north_goal_angle waypoint_recorder → 끝나면 검사 결과.
# 절차·주의: docs/절차/웨이포인트_녹화절차.md
set -u
boat=${1:?A 또는 B}
course=${2:?main 또는 tournament}
lane=${3:-}
cd "$(dirname "$0")/.." || exit 1
# tmux 안이면 마우스 모드: 트랙패드 스크롤이 화살표 키로 입력 줄에 들어가지 않고 화면을 올려 본다(빠져나오기 q)
[ -n "${TMUX:-}" ] && tmux set-option mouse on >/dev/null 2>&1
# 이 세션이 어느 녹화인지 표시 — 다른 코스·레인 명령이 이 세션에 잘못 다시 붙지 않게(ssf 가 확인)
[ -n "${TMUX:-}" ] && tmux set-option @ssf_wp "${course}:${lane}" >/dev/null 2>&1
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
# shellcheck disable=SC1091
source tools/ssf_ros_net.sh && ssf_ros_net      # 두 배 GPS 섞임 방지 — ~/.bashrc 와 같은 ROS 격리

echo
case "$course" in
  tournament) what="토너먼트 레인 $lane" ;;
  practice) what="연습 $lane (그 미션만)" ;;
  *) what="종합임무" ;;
esac
echo "━━ 웨이포인트 녹화 — $( [ "$boat" = A ] && echo A번배 || echo B번배 ) · $what"
echo "   저장 파일: $OUT"
[ -n "$TEMPLATE" ] && echo "   템플릿:   $TEMPLATE"
echo "   ① 지금 GPS: $(python3 tools/ssf_param_core.py --boat "$boat" gps --wait 4 2>/dev/null | grep -v '^\[' | head -1 | sed 's/^ *//')"
echo "      🟢 RTK FIXED 일 때 찍는다(앱 위쪽 'RTK' 불 · ssf $(echo "$boat" | tr AB ab) gps)"
echo "   ② 조종기로 지점에 대고 몇 초 머문 뒤 Enter → mode → dwell → 이름. l=목록 d N=삭제 r N=재캡처 q=저장"
if [ "$course" = tournament ]; then
  echo "   ③ 🚨 전부 mode 8. 마지막 점 = 결승선 너머 약 3 m, dwell 0.5 — 경기장 경계 안"
elif [ "$course" = practice ]; then
  echo "   ③ 연습: 기본값(엔터)은 종합임무 템플릿의 그 미션 줄. 대회 녹화본(waypoints_recorded.yaml)과 따로 저장된다"
  echo "      launch 는 course:=main · 마지막 점 뒤엔 조종기로 받는다"
else
  echo "   ③ OT 순서: 게이트 시작·끝 → 위치유지 → 도킹(정렬 6m → 도크 정면 → 이탈) → 탐색 접근 → 탐색 3 → 회피 시작·끝 → WP10 종료 게이트 너머 — 엔터로 기본값 따라 13점"
  echo "      도킹: 정렬 6m = 도크 가운데 칸 정면 6 m(칸 축 위) · 도크 정면 = 가운데 칸 입구 앞 · 이탈 = 정렬 6m 과 같은 자리(다시 가서 찍기)"
  echo "      배는 정렬 6m 에서 제자리 선회로 목표 표식을 찾아 들어간다 — 칸마다 좌표 안 바꿈"
  echo "      위치유지: 부표 좌표 말고 부표 옆 대기점(부표에서 약 1.5 m, 1.0~2.0 · 조류 있으면 부표가 위쪽) — 배가 그 점에 6 s 선다(GPS)"
  echo "      탐색 부표가 바람에 밀리면 저장 전 l 로 번호 보고 r N 으로 그 점만 다시 찍는다"
fi
echo "   SSH 가 끊겨도 tmux 안이라 녹화는 계속된다 — Mac 에서 같은 ssf 명령을 다시 치면 이어 붙는다"
echo
sig() { cksum "$1" 2>/dev/null || echo "없음"; }
before=$(sig "$OUT")
if [ -n "$TEMPLATE" ]; then
  ros2 run north_goal_angle waypoint_recorder "$OUT" --template="$TEMPLATE"
else
  ros2 run north_goal_angle waypoint_recorder "$OUT"
fi
rec_rc=$?
saved=1
[ "$(sig "$OUT")" = "$before" ] && saved=0

echo
echo "━━ 검사 (앱 '웨이포인트' 탭에서도 보인다)"
python3 tools/ssf_param_core.py --boat "$boat" wp-list | grep -v '^\['
echo
[ $rec_rc -ne 0 ] && echo "❌ 녹화 도구가 오류로 끝났다(코드 $rec_rc) — 위 문구 확인"
if [ $saved = 1 ]; then
  echo "✅ 이번 녹화를 저장했다: $OUT"
  # $HOME 형식 — ~/ 는 ':=' 뒤라 bash 가 안 풀고 노드가 글자 그대로 연다(north_goal_angle_launch.py 주석)
  echo "다음: launch 할 때 waypoints_file:=${OUT/#$HOME/\$HOME}  (~/ 로 쓰지 않는다 · 저장소 파일에 복사하지 않는다 · 현장 조정표 §4 에 기록)"
else
  echo "⚠️ 이번에 저장된 것이 없다 — $OUT 은 이전 파일이거나 없다. 이 파일을 이번 녹화로 쓰지 말 것"
fi
pause
