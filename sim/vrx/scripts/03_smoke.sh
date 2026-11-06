#!/usr/bin/env bash
# VRX 시뮬 3단계 — 화면 없이(headless) 띄워 README §5 점검표 1~5·7 을 자동으로 잰다.
#   bash ~/ros2_ws/sim/vrx/scripts/03_smoke.sh A      (또는 B)
# 결과는 ~/ssf_logs/vrx_smoke_<배>_<시각>.log 에도 남는다. 스택(north·ship_*)은 안 띄운다.
set -uo pipefail
BOAT="${1:-A}"
b="$(echo "$BOAT" | tr 'A-Z' 'a-z')"
mkdir -p ~/ssf_logs
LOG=~/ssf_logs/vrx_smoke_${BOAT}_$(date +%Y%m%d_%H%M%S).log
exec > >(tee "$LOG") 2>&1

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/vrx_ws/install/setup.bash
source ~/ros2_ws/sim/vrx/install/setup.bash
SHARE="$(ros2 pkg prefix ssf_vrx)/share/ssf_vrx"
JSON="$SHARE/worlds/kaboat_${b}.json"
PSI0=$(python3 -c "import json;print(round(json.load(open('$JSON'))['psi0'],1))")
echo "== 배 $BOAT, 월드 kaboat_${b}, 시작 헤딩(psi0) ${PSI0}°, 로그 $LOG"

cleanup() { echo "== 정리"; pkill -f "ssf_vrx.launch.py" 2>/dev/null; pkill -f "gz sim" 2>/dev/null; pkill -f parameter_bridge 2>/dev/null; pkill -f sim_bridge 2>/dev/null; }
trap cleanup EXIT

echo "== launch (headless, 스택 없음) — 25 s 대기"
ros2 launch ssf_vrx ssf_vrx.launch.py boat:="$BOAT" headless:=true with_stack:=false > ~/ssf_logs/vrx_launch_${BOAT}.log 2>&1 &
sleep 25
if ! pgrep -f "gz sim" > /dev/null; then
  echo "❌ gz sim 이 안 떴다. ~/ssf_logs/vrx_launch_${BOAT}.log 끝 40줄:"; tail -40 ~/ssf_logs/vrx_launch_${BOAT}.log; exit 1
fi
grep -iE "error|exception|died|not found" ~/ssf_logs/vrx_launch_${BOAT}.log | grep -v "no matches" | head -10 || true

echo "== [1] RTF (gz stats 5 s)"
timeout 6 gz stats 2>/dev/null | tail -3 || echo "(gz stats 출력 없음)"

echo "== [2] 토픽 주기 (각 8 s). 기대: /scan≈11, /imu/yaw_raw≈100, /ublox_gps_node/fix≈1, /boat_mode 10, /vrx/ground_truth 10"
for t in /scan /imu/yaw_raw /ublox_gps_node/fix /boat_mode /vrx/ground_truth /oak/rgb/image_raw; do
  r=$(timeout 8 ros2 topic hz "$t" 2>/dev/null | grep -m1 "average rate" | awk '{print $3}')
  echo "   $t : ${r:-없음} Hz"
done
echo "   /boat_mode 값: $(timeout 3 ros2 topic echo --once /boat_mode 2>/dev/null | grep data | awk '{print $2}')  (기대 2)"

echo "== [3] 헤딩 부호: /imu/yaw_raw vs psi0 ${PSI0}° (±3° 안이어야)"
YAW=$(timeout 3 ros2 topic echo --once /imu/yaw_raw 2>/dev/null | grep data | awk '{print $2}')
echo "   /imu/yaw_raw = ${YAW:-없음}"
python3 - "$YAW" "$PSI0" <<'EOF'
import sys
try:
    y, p = float(sys.argv[1]), float(sys.argv[2]); d = (y - p + 180) % 360 - 180
    print(f"   차이 {d:+.1f}° → {'OK' if abs(d) <= 3 else '❌ 헤딩 변환/초기 yaw 확인'}")
except Exception: print("   (값 없음)")
EOF

gt() { timeout 4 ros2 topic echo --once /vrx/ground_truth 2>/dev/null | python3 -c "
import sys, yaml, math
d = yaml.safe_load(sys.stdin); p = d['pose']['pose']['position']; q = d['pose']['pose']['orientation']
yaw = math.degrees(math.atan2(2*(q['w']*q['z']+q['x']*q['y']), 1-2*(q['y']**2+q['z']**2)))
print(p['x'], p['y'], (90-yaw) % 360)"; }

echo "== [4a] 전진: Motor_run 13601360 을 12 s (기대: 헤딩 방향으로 전진, 속도 ~1 m/s 근처)"
read -r X0 Y0 H0 <<< "$(gt)"
timeout 12 ros2 topic pub -r 20 Motor_run std_msgs/msg/Int32 "{data: 13601360}" > /dev/null 2>&1
read -r X1 Y1 H1 <<< "$(gt)"
python3 - "$X0" "$Y0" "$H0" "$X1" "$Y1" "$H1" <<'EOF'
import sys, math
x0,y0,h0,x1,y1,h1 = map(float, sys.argv[1:])
dx,dy = x1-x0, y1-y0; dist = math.hypot(dx,dy)
brg = math.degrees(math.atan2(dx, dy)) % 360          # ENU 변위의 나침반 방위
err = (brg - h0 + 180) % 360 - 180
print(f"   이동 {dist:.2f} m (12 s → 평균 {dist/12:.2f} m/s, 가속 포함), 변위 방위 {brg:.0f}° vs 헤딩 {h0:.0f}° (차 {err:+.0f}°)")
print("   " + ("OK 전진" if dist > 2.0 and abs(err) < 25 else "❌ 안 움직이거나 방향이 다름 → 추진 부호/좌우/수력 확인"))
EOF

echo "== [4b] 우선회: Motor_run 14001320 (왼쪽 1320 강함) 6 s → 헤딩이 시계방향(증가)으로 돌아야"
read -r X0 Y0 H0 <<< "$(gt)"
timeout 6 ros2 topic pub -r 20 Motor_run std_msgs/msg/Int32 "{data: 14001320}" > /dev/null 2>&1
read -r X1 Y1 H1 <<< "$(gt)"
python3 - "$H0" "$H1" <<'EOF'
import sys
h0,h1 = map(float, sys.argv[1:]); d = (h1-h0+180) % 360 - 180
print(f"   헤딩 {h0:.0f}° → {h1:.0f}° (변화 {d:+.0f}°) → " + ("OK 우선회" if d > 5 else "❌ 좌우 뒤바뀜 또는 선회 안 함"))
EOF

echo "== [4c] 정지: 명령 끊고 3 s → 워치독으로 추력 0 (배가 서서히 멈춤)"
sleep 3
echo "   /boat_cmd_watchdog: $(timeout 3 ros2 topic echo --once /boat_cmd_watchdog 2>/dev/null | grep data | awk '{print $2}')  (기대 true)"

echo "== [5] LiDAR 정면 각: /obstacle_distance_array 는 ship_direction 이 내므로 스택 없이는 생략. 대신 /scan 길이·각 범위"
timeout 4 ros2 topic echo --once /scan 2>/dev/null | grep -E "angle_min|angle_max|angle_increment|range_max" | sed 's/^/   /'
echo "   기대: angle_min -3.14159, angle_increment 0.00436(0.25°), ranges 1440개"

echo "== 끝. 전체 코스는: ros2 launch ssf_vrx ssf_vrx.launch.py boat:=$BOAT  (스택 포함, 화면 있음)"
