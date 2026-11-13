#!/usr/bin/env bash
# 자율 토너먼트 실행 — 레인 녹화본으로 대회 launch 를 tournament 코스로 띄운다 (A·B 두 배 공용).
#
#   tools/ssf_tournament.sh <A|B> <레인>                     예: tools/ssf_tournament.sh A 1
#   tools/ssf_tournament.sh <A|B> <레인> [추가 launch 인자...]  예: tools/ssf_tournament.sh B 2 record:=false
#
# 손으로 칠 때와 같은 명령이다(대회 launch 그대로, 새 노드·파라미터 없음):
#   ros2 launch launch_files <대회 launch> boat:=<배> course:=tournament \
#       waypoints_file:=$HOME/ssf_logs/waypoints_tournament_<배>_<레인>.yaml d455_depth:=false perception_fusion:=false
#   · d455_depth·perception_fusion 끔: 토너먼트는 전부 mode 8 이라 안 쓴다 → USB 허브 부담만 줄인다.
#     켜고 싶으면 뒤에 d455_depth:=true perception_fusion:=true (뒤 인자가 이긴다).
#   · OAK·검출기는 그대로 뜨지만 mode 8 에선 쉰다(주행 영향 없음).
#
# 띄우기 전 확인(하나라도 틀리면 launch 안 함):
#   ① 배 = ~/.ssf_boat (있으면)   ② 대회 launch 가 이미 떠 있지 않음(노드 2벌 방지)
#   ③ 레인 녹화본 — ~/ssf_logs/waypoints_tournament_<배>_<레인>.yaml 우선, 없으면 공유
#      tools/tournament_lanes/waypoints_tournament_<레인>.yaml, 둘 다 없으면 녹화 명령 안내
#   ④ 녹화본 웨이포인트 2개 이상·전부 mode 8 (north 도 같은 검사로 출발 거부)
#   ⑤ 경기장 경계(north_goal_angle_tournament.yaml geofence_polygon) — 비면 경고 후 y 를 쳐야 띄운다
#
# 🚨 판이 끝나면 Ctrl+C 한 번(라이다 정지 명령) → 다음 판은 이 스크립트를 다시 실행.
#    MANUAL→AUTO 재진입은 웨이포인트 순서를 되돌리지 않는다 — 판마다 launch 재시작(계약서_토너먼트분리_20261109).
# 절차: docs/절차/boat_launch_checklist.md 토너먼트 체크, 녹화는 docs/절차/웨이포인트_녹화절차.md §3a.
# 이 파일 이름에 대회 launch 파일 이름을 넣지 않는다 — 앱의 'launch 실행 중' 판정(pgrep)이 이 셸을 launch 로 착각한다.
set -u

die() { echo "❌ $*" >&2; exit 1; }

boat=$(printf '%s' "${1:-}" | tr '[:lower:]' '[:upper:]')
lane=${2:-}
case "$boat" in
  A|B) ;;
  *) die "사용: tools/ssf_tournament.sh <A|B> <레인> [추가 launch 인자...]  예: tools/ssf_tournament.sh A 1" ;;
esac
[ -n "$lane" ] || die "레인이 필요하다 — 예: tools/ssf_tournament.sh $boat 1"
case "$lane" in
  [!A-Za-z0-9]*|*[!A-Za-z0-9_-]*) die "레인 이름은 영문·숫자로 시작, 영문·숫자·_·- 만(녹화 도구와 같은 규칙): '$lane'" ;;
esac
shift 2
cd "$(dirname "$0")/.." || exit 1
repo=$(pwd)

# ① 배 확인 — A번배 노트북에서 B 로 띄우는 사고 방지(파라미터 앱과 같은 ~/.ssf_boat)
if [ -f "$HOME/.ssf_boat" ]; then
  mine=$(tr -d '[:space:]' < "$HOME/.ssf_boat" | tr '[:lower:]' '[:upper:]')
  [ "$mine" = "$boat" ] || die "이 노트북은 $mine 번배(~/.ssf_boat)인데 $boat 로 띄우려 했다 — 배 인자를 확인"
fi

# ② 대회 launch 중복 확인
LAUNCH_PY="launch_files.launch.py"
if pgrep -f "ros2 launch launch_files $LAUNCH_PY" >/dev/null 2>&1; then
  die "대회 launch 가 이미 떠 있다 — 그 창에서 Ctrl+C 한 번으로 끈 뒤 다시 실행"
fi

# ③ 레인 녹화본 — 배 자기 녹화본 우선, 없으면 저장소 공유 레인(tools/tournament_lanes, 두 배 공용)
own="$HOME/ssf_logs/waypoints_tournament_${boat}_${lane}.yaml"
shared="$repo/tools/tournament_lanes/waypoints_tournament_${lane}.yaml"
if [ -f "$own" ]; then
  wp="$own"
  [ -f "$shared" ] && ! cmp -s "$own" "$shared" && \
    echo "⚠️  자기 녹화본을 쓴다(공유 레인 $shared 와 내용 다름) — 공유본을 쓰려면 자기 녹화본 이름을 바꿔 치울 것"
elif [ -f "$shared" ]; then
  wp="$shared"
  echo "ℹ️  자기 녹화본($own) 없음 → 공유 레인 사용: $shared"
else
  echo "❌ 레인 녹화본 없음: $own (공유 레인 $shared 도 없음)" >&2
  echo "   먼저 녹화: tools/ssf_wp_record.sh $boat tournament $lane   (절차: 웨이포인트_녹화절차.md §3a)" >&2
  exit 1
fi

# ④ 녹화본 내용 — 전부 mode 8, 2개 이상 (cd / 에서 실행: 저장소 안 파일이 모듈로 안 잡히게. 읽기만 한다)
(cd / && python3 - "$wp") <<'EOF_WP' || exit 1
import sys
import yaml
p = sys.argv[1]
try:
    d = yaml.safe_load(open(p, encoding="utf-8")) or {}
    wps = d.get("waypoints") or []
    modes = [int(w["mode"]) for w in wps]
except Exception as e:  # noqa: BLE001 — 어떤 형식 오류든 띄우지 않는다
    sys.exit(f"❌ 녹화본을 못 읽음({p}): {e}")
if len(modes) < 2:
    sys.exit(f"❌ 웨이포인트 {len(modes)}개 — 출발 근처·결승선 너머 3 m 최소 2개 필요 ({p})")
bad = [i for i, m in enumerate(modes) if m != 8]
if bad:
    sys.exit(f"❌ mode 8 아닌 웨이포인트 순번 {bad} — 토너먼트는 전부 mode 8 (north 가 출발 거부). 다시 녹화")
print(f"✓ 레인 녹화본 {p} — 웨이포인트 {len(modes)}개, 전부 mode 8")
EOF_WP

# ⑤ 경기장 경계 폴리곤 (설치본 우선, 없으면 소스)
poly_yaml="$repo/install/north_goal_angle/share/north_goal_angle/config/north_goal_angle_tournament.yaml"
[ -f "$poly_yaml" ] || poly_yaml="$repo/src/north_goal_angle/config/north_goal_angle_tournament.yaml"
npts=$(cd / && python3 - "$poly_yaml" <<'EOF_POLY'
import sys
import yaml
try:
    d = yaml.safe_load(open(sys.argv[1], encoding="utf-8")) or {}
    blk = d.get("/north_goal_angle") or d.get("north_goal_angle") or {}
    poly = (blk.get("ros__parameters") or {}).get("geofence_polygon") or []
    print(len(poly) // 2 if len(poly) >= 6 and len(poly) % 2 == 0 else 0)
except Exception:  # noqa: BLE001
    print(-1)
EOF_POLY
)
if [ "${npts:-0}" -ge 3 ] 2>/dev/null; then
  echo "✓ 경기장 경계 ${npts}점 ($poly_yaml) — 시작 로그 'geofence ON(${npts}점)' 확인"
else
  echo "⚠️  경기장 경계(geofence_polygon)가 비었거나 못 읽음 — 경계 이탈 방어 없이 달린다(실격 위험)."
  echo "    채울 곳: $repo/src/north_goal_angle/config/north_goal_angle_tournament.yaml (현장 경계 그대로, 결승선 너머 포함)"
  if [ -t 0 ]; then
    read -rp "    경계 없이 띄울까? [y/N] " ans
    [ "$ans" = "y" ] || [ "$ans" = "Y" ] || die "중단 — 경계를 채운 뒤 다시 실행"
  else
    die "대화형 터미널이 아니라 확인을 못 받는다 — 경계를 채우거나 터미널에서 직접 실행"
  fi
fi

# ROS 환경 — 대화형 터미널이면 이미 돼 있어도 다시 읽어 무해. 두 배 섞임 방지 격리도 같은 값으로.
if [ -z "${SSF_TOURNAMENT_DRY:-}" ]; then
  setup=$(ls /opt/ros/*/setup.bash 2>/dev/null | tail -1)
  [ -n "$setup" ] && [ -f install/setup.bash ] || die "ROS(/opt/ros/*/setup.bash) 또는 install/setup.bash 없음 — colcon build 확인"
  # shellcheck disable=SC1090
  source "$setup"
  # shellcheck disable=SC1091
  source install/setup.bash
fi
# shellcheck disable=SC1091
source tools/ssf_ros_net.sh && ssf_ros_net

args=("boat:=$boat" "course:=tournament" "waypoints_file:=$wp"
      "d455_depth:=false" "perception_fusion:=false" "$@")
echo "▶ $(date +%T) 토너먼트 ${boat}번배 레인 ${lane}: ros2 launch launch_files $LAUNCH_PY ${args[*]}"
echo "   판 끝 = Ctrl+C 한 번 → 다음 판은 이 스크립트 다시 실행"
if [ -n "${SSF_TOURNAMENT_DRY:-}" ]; then   # 시험 전용 — 확인만 하고 launch 안 함
  echo "(SSF_TOURNAMENT_DRY — launch 안 함)"
  exit 0
fi
exec ros2 launch launch_files "$LAUNCH_PY" "${args[@]}"
