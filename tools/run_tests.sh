#!/usr/bin/env bash
# 로직 테스트 전체 실행 — 우리 패키지만.
#
# 🚨 `python3 -m pytest src/` 를 그냥 돌리면 **테스트를 하나도 못 돌리고 죽는다.**
#    두 가지가 겹쳐 있다:
#
#    ① 같은 파일명 충돌 — ament 보일러플레이트(test_copyright / test_flake8 / test_pep257)가
#       패키지 10곳에 **같은 이름**으로 있다. `launch_testing` 플러그인이 자체적으로
#       `pyimport()` 를 해서 `--import-mode=importlib` 로도 못 피한다.
#    ② 외부 패키지 — realsense / ublox / rplidar / ntrip / uros 는 우리가 고치지 않고,
#       빠진 의존성(`quaternion` 등)으로 수집부터 깨진다.
#
# 🚨 **이 설정을 `pytest.ini` 로 두면 안 된다.** `colcon test` 가 그 addopts 를 같이 주워서
#    lint 검사를 조용히 건너뛴다 — 실측: color_shape_detector 가
#    **74 tests / 2 failures → 71 tests / 0 failures** 로 바뀌어 기존 실패가 통과처럼 보였다.
#    그래서 설정 파일 대신 이 스크립트로 둔다. **colcon test 가 여전히 권위 있는 실행기다.**
#
# 쓰는 법
#   tools/run_tests.sh              전체
#   tools/run_tests.sh ssf_bridge   한 패키지
#
# 🚨 lint(flake8/pep257/copyright)는 여기서 안 돈다. 그건 `colcon test` 로 봐야 한다.

set -e
cd "$(dirname "$0")/.."

PKGS=(
  color_shape_detector launch_files motor_control north_goal_angle
  ship_back ship_direction ship_dock ship_gate ship_goal_angle ship_turn
  ssf_bridge ssf_heading ssf_tools
)

if [ $# -gt 0 ]; then
  PATHS=()
  for p in "$@"; do
    [ -d "src/$p" ] || { echo "❌ src/$p 가 없다"; exit 1; }
    PATHS+=("src/$p")
  done
else
  PATHS=()
  for p in "${PKGS[@]}"; do [ -d "src/$p" ] && PATHS+=("src/$p"); done
fi

exec python3 -m pytest "${PATHS[@]}" \
  --import-mode=importlib \
  --ignore-glob='*/test_copyright.py' \
  --ignore-glob='*/test_flake8.py' \
  --ignore-glob='*/test_pep257.py' \
  "${PYTEST_ARGS:--q}"
