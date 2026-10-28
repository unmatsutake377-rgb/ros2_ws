#!/usr/bin/env bash
# 보고서 분담용 "AI 업로드 묶음" 만들기 — 센서 하나당 파일 하나.
#
#   tools/report_pack.sh imu|gps|lidar|camera  [출력경로]
#   기본 출력: ~/보고서묶음_<센서>.md
#
# 묶음 안에 들어가는 것 (순서대로):
#   ① 공통 작성규칙  ② 현재 보고서 Ⅲ장 원고  ③ 담당 지시서
#   ④ 읽을 소스 파일 전부 — **행 번호 포함**(cat -n). AI 가 [근거: 파일:행] 을 달 수 있게.
#   ⑤ 관련 문서  ⑥ 작년 최종본(9bff661) 대비 변경 — 커밋 목록 + 새 파일 목록 + 바뀐 파일 diff
#      새 파일은 소스 전문과 중복이라 diff 를 넣지 않는다(카메라 묶음 11만 → 토큰 절감).
#
# 🚨 ntrip_client 는 diff 를 넣지 않는다. 작년 launch 에 NTRIP 계정이 평문으로 있었고
#    (f98a8f9 에서 제거), diff 의 '-' 줄에 그 값이 그대로 다시 나온다. 커밋 메시지만 넣는다.
# 🚨 rplidar_ros 는 9bff661 이 아니라 19cfde7(내재화) 기준. 그 전엔 gitlink 라 diff 가 9만 줄이 된다.
# ~/.ssf/ 는 어떤 경우에도 읽지 않는다.

set -e
cd "$(dirname "$0")/.."
BASE=9bff661
DIR="docs/전달용/보고서_분담"
S="${1:-}"

case "$S" in
  imu)
    NAME=IMU; GUIDE="$DIR/01_IMU.md"
    FILES=(
      src/ssf_heading/ssf_heading/yaw_mux.py
      src/ssf_heading/ssf_heading/heading_logic.py
      src/ssf_heading/config/ssf_heading.yaml
      src/ssf_heading/launch/ssf_heading.launch.py
      src/ship_goal_angle/ship_goal_angle/ship_goal_angle.py
      src/ship_goal_angle/config/ship_goal_angle.yaml
      src/iahrs_driver_ros2-main/iahrs_driver/src/iahrs_driver.cpp
      src/iahrs_driver_ros2-main/iahrs_driver/launch/iahrs_driver.py
      src/launch_files/launch/launch_files.launch.py
    )
    DOCS=(CLAUDE.md docs/전달용/IMU_노드_쉬운설명.md docs/전달용/회신_N4_mag_heading_검토.md docs/전달용/회신_상위3팀분석_IMU.md)
    TESTS=(src/ssf_heading/test)
    DIFF_PATHS=(src/iahrs_driver_ros2-main src/ship_goal_angle)
    LOG_PATHS=(src/ssf_heading src/iahrs_driver_ros2-main src/ship_goal_angle)
    ;;
  gps)
    NAME=GPS; GUIDE="$DIR/02_GPS.md"
    FILES=(
      src/north_goal_angle/north_goal_angle/north_goal_angle.py
      src/north_goal_angle/north_goal_angle/gps_guard.py
      src/north_goal_angle/north_goal_angle/gps_filter.py
      src/north_goal_angle/north_goal_angle/los_logic.py
      src/north_goal_angle/north_goal_angle/waypoint_loader.py
      src/north_goal_angle/north_goal_angle/waypoint_recorder.py
      src/north_goal_angle/config/north_goal_angle.yaml
      src/north_goal_angle/config/waypoints.yaml
      src/north_goal_angle/launch/north_goal_angle_launch.py
      src/ublox/ublox_gps/config/c94_m8p_rover.yaml
      src/ublox/ublox_gps/config/zed_f9p_rover.yaml
      src/ublox/ublox_gps/launch/ublox_gps_node-launch.py
      src/ntrip_client/launch/ntrip_client_launch.py
      src/ntrip_client/config/ntrip_credentials.example.yaml
      src/launch_files/launch/launch_files.launch.py
    )
    DOCS=(CLAUDE.md docs/전달용/팀원전달_GPS수정_병합안내.md docs/전달용/회신_GPS코드_기능추출_수정제안.md docs/전달용/회신_LOS강화_IMU추가_검토.md docs/절차/웨이포인트_녹화절차.md)
    TESTS=(src/north_goal_angle/test)
    DIFF_PATHS=(src/north_goal_angle src/ublox)
    LOG_PATHS=(src/north_goal_angle src/ublox src/ntrip_client)
    ;;
  lidar)
    NAME=LiDAR; GUIDE="$DIR/03_LiDAR.md"
    FILES=(
      src/ship_direction/ship_direction/ship_direction.py
      src/ship_direction/ship_direction/failsafe.py
      src/ship_direction/config/ship_direction.yaml
      src/ship_direction/launch/ship_direction_launch.py
      src/rplidar_ros/launch/rplidar_a3_launch.py
      src/launch_files/launch/launch_files.launch.py
    )
    DOCS=(CLAUDE.md docs/전달용/LiDAR_노드_쉬운설명.md docs/전달용/회신_rplidar_회피_수정제안.md docs/전달용/회신_ship_direction_수정제안.md docs/문제와작업/라이다_고장진단_20260818.md tools/sim/README.md)
    TESTS=(src/ship_direction/test)
    DIFF_PATHS=(src/ship_direction)
    LOG_PATHS=(src/ship_direction src/rplidar_ros)
    ;;
  camera)
    NAME=카메라; GUIDE="$DIR/04_카메라.md"
    FILES=(
      src/color_shape_detector/color_shape_detector/basic_image_subscribergate.py
      src/color_shape_detector/color_shape_detector/basic_image_subscriberturn.py
      src/color_shape_detector/color_shape_detector/basic_image_subscriberdock.py
      src/color_shape_detector/color_shape_detector/basic_image_subscriberhsv.py
      src/color_shape_detector/color_shape_detector/dock_logic.py
      src/color_shape_detector/color_shape_detector/vision_geom.py
      src/color_shape_detector/color_shape_detector/mode_gate.py
      src/color_shape_detector/color_shape_detector/hsv_ranges.py
      src/color_shape_detector/config/vision.yaml
      src/launch_files/launch/camera.launch.py
      src/launch_files/launch/launch_files.launch.py
    )
    DOCS=(CLAUDE.md docs/문제와작업/비전_개선후보_20261027.md docs/문제와작업/계약서_비전A_20261027.md docs/절차/oak_arrival_runbook.md docs/절차/카메라영상_설치와_벤치.md)
    TESTS=(src/color_shape_detector/test)
    DIFF_PATHS=(src/color_shape_detector)
    LOG_PATHS=(src/color_shape_detector)
    ;;
  *) echo "사용법: tools/report_pack.sh imu|gps|lidar|camera [출력경로]"; exit 1 ;;
esac

OUT="${2:-$HOME/보고서묶음_${NAME}.md}"

{
  echo "# 보고서 분담 묶음 — ${NAME}"
  echo "만든 시각: $(date '+%Y-%m-%d %H:%M') · 저장소 커밋: $(git rev-parse --short HEAD)"
  echo
  echo "이 파일 하나를 AI 에 올리고, 공통 작성규칙 5절의 프롬프트를 붙여 넣는다."
  echo "소스 파일에는 행 번호가 붙어 있다. 근거는 [근거: 파일경로:행] 으로 단다."
  echo
  for f in "$DIR/00_공통_작성규칙.md" "$DIR/00_현재원고_Ⅲ소프트웨어.md" "$GUIDE"; do
    echo; echo "=================================================================="
    echo "## 문서: $f"; echo "=================================================================="
    cat "$f"
  done
  echo; echo "=================================================================="
  echo "## 소스 파일 (행 번호 포함)"; echo "=================================================================="
  for f in "${FILES[@]}"; do
    [ -f "$f" ] || { echo "⚠️ 없음: $f"; continue; }
    echo; echo "----- FILE: $f -----"; echo '```'; cat -n "$f"; echo '```'
  done
  echo; echo "=================================================================="
  echo "## 테스트 목록 (함수 이름만)"; echo "=================================================================="
  for d in "${TESTS[@]}"; do
    for t in "$d"/test_*.py; do
      case "$t" in *copyright*|*flake8*|*pep257*) continue ;; esac
      echo "- $t : $(grep -c '^\s*def test_' "$t") 개"
      grep -n '^\s*def test_' "$t" | sed 's/^/    /'
    done
  done
  echo; echo "=================================================================="
  echo "## 관련 문서"; echo "=================================================================="
  for f in "${DOCS[@]}"; do
    [ -f "$f" ] || { echo "⚠️ 없음: $f"; continue; }
    echo; echo "----- DOC: $f -----"; cat "$f"
  done
  echo; echo "=================================================================="
  echo "## 작년 최종본(${BASE}) 이후 커밋 목록"; echo "=================================================================="
  git log --format='- %h %ad %s' --date=short ${BASE}..HEAD -- "${LOG_PATHS[@]}"
  echo; echo "=================================================================="
  echo "## 작년 최종본(${BASE}) 대비 변경 diff"; echo "=================================================================="
  echo "올해 새로 생긴 파일 (diff 생략 — 위 소스 파일 또는 저장소에서 직접 본다):"
  git diff --name-only --diff-filter=A ${BASE} HEAD -- "${DIFF_PATHS[@]}" | sed 's/^/- /'
  echo
  echo "작년 파일 중 바뀌거나 지워진 것의 diff (문맥 1줄):"
  echo '```diff'
  git diff -U1 --diff-filter=MD ${BASE} HEAD -- "${DIFF_PATHS[@]}"
  if [ "$S" = lidar ]; then git diff -U1 19cfde7 HEAD -- src/rplidar_ros; fi
  echo '```'
  if [ "$S" = gps ]; then
    echo
    echo "ntrip_client 의 diff 는 일부러 넣지 않았다(작년 평문 계정이 다시 나온다). 위 커밋 메시지로 쓴다."
  fi
} > "$OUT"

echo "만듦: $OUT ($(wc -l < "$OUT" | tr -d ' ') 줄, $(du -h "$OUT" | cut -f1))"
