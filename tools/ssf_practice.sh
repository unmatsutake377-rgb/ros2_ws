#!/usr/bin/env bash
# 연습용 가짜 배 두 척 — 배 없이 MacBook(또는 아무 컴퓨터)에서 ssf 앱·명령을 미리 써 본다.
#
#   tools/ssf_practice.sh [폴더]        (기본 ~/ssf_practice)
#
# 만드는 것
#   <폴더>/boat-a/ros2_ws, boat-b/ros2_ws   이 저장소의 git clone (커밋된 상태 그대로) — 배 노트북 흉내
#   <폴더>/boat-a/.ssf_boat = A, boat-b = B  배 등록 표시
#   install/<pkg>/share/<pkg>/config/*.yaml  src 를 가리키는 symlink (colcon --symlink-install 흉내 → '빌드 불필요')
#   <폴더>/fake_ssh                          ssh 대신: 'boat-a 명령' → 그 폴더를 집(SSF_HOME)으로 해서 이 컴퓨터에서 실행
#
# 쓰기
#   SSF_SSH=<폴더>/fake_ssh SSF_REMOTE_REPO=<폴더>/boat-a/ros2_ws … 가 귀찮으니 아래 한 줄로:
#   source <폴더>/env.sh        → 이 터미널에서 ssf 가 가짜 배로 간다 (ssf app both, ssf a get hold …)
#   다 하면 터미널을 닫으면 된다. 진짜 배엔 아무 영향 없음. 지우기: rm -rf <폴더>
# 🚨 가짜 배 clone 에서 바꾼 값은 진짜 저장소와 무관 — 연습 끝나면 버린다.
set -eu
REPO=$(cd "$(dirname "$0")/.." && pwd)
DIR=${1:-$HOME/ssf_practice}
mkdir -p "$DIR"
DIR=$(cd "$DIR" && pwd)
for b in a b; do
  B=$(echo $b | tr a-z A-Z)
  home="$DIR/boat-$b"
  if [ -d "$home/ros2_ws/.git" ]; then
    echo "boat-$b: 이미 있음 — 최신 커밋으로 맞춤 (가짜 배에서 바꾼 값은 지워짐)"
    git -C "$home/ros2_ws" fetch -q origin && git -C "$home/ros2_ws" reset -q --hard origin/HEAD 2>/dev/null \
      || git -C "$home/ros2_ws" reset -q --hard
  else
    mkdir -p "$home"
    git clone -q "$REPO" "$home/ros2_ws"
  fi
  # 아직 커밋 안 한 앱 파일도 연습에 쓰이게 복사(커밋 뒤에는 clone 에 이미 있다)
  for f in ssf_param_core.py ssf_param_app.py ssf_param_app.html ssf_param_catalog.yaml param_check.py ssf ssf_waypoints.py ssf_wp_record.sh; do
    cp "$REPO/tools/$f" "$home/ros2_ws/tools/$f"
  done
  echo "$B" > "$home/.ssf_boat"
  (cd "$home/ros2_ws" && for y in src/*/config/*.yaml; do
     pkg=$(echo "$y" | cut -d/ -f2); d="install/$pkg/share/$pkg/config"; mkdir -p "$d"
     ln -sfn "$PWD/$y" "$d/$(basename "$y")"; done)
  echo "boat-$b: $home (등록 $B)"
done

PYUSER=$(python3 -c 'import site; print(site.getusersitepackages())')
cat > "$DIR/fake_ssh" <<EOF
#!/usr/bin/env bash
# 가짜 ssh: [옵션] boat-a|boat-b 명령… → 그 가짜 배 폴더를 집으로 해서 이 컴퓨터에서 실행. -L 포트 전달은 필요 없다(같은 컴퓨터).
while [ \$# -gt 0 ]; do
  case "\$1" in
    -L|-o|-p|-i|-l|-F) shift 2;;
    -*) shift;;
    *) break;;
  esac
done
host=\$1; shift
case "\$host" in boat-a|boat-b) ;; *) echo "가짜 ssh: 모르는 배 \$host" >&2; exit 255;; esac
export HOME="$DIR/\$host" PYTHONPATH="$PYUSER\${PYTHONPATH:+:\$PYTHONPATH}"
cd "\$HOME" && exec bash -c "\$*"
EOF
chmod +x "$DIR/fake_ssh"
cat > "$DIR/env.sh" <<EOF
# source 로 읽는다: 이 터미널의 ssf 가 가짜 배로 간다
export SSF_SSH="$DIR/fake_ssh"
alias ssf="$REPO/tools/ssf"
echo "연습 모드: ssf → 가짜 배 ($DIR). 해제 = 터미널 닫기"
EOF
echo
echo "준비 끝. 이 터미널에서:"
echo "  source $DIR/env.sh"
echo "  ssf app both          # 앱 두 개 (1번배 http://localhost:8765, 2번배 :8766)"
echo "  ssf a get hold        # 터미널로"
