#!/usr/bin/env bash
# 모드 전환 소요시간 / 각도 토픽 공백 실측 (3-4 검증)
#
# 왜 필요한가:
#   3-4 보고의 [후] 수치(<=33ms)는 코드에서 결정론적으로 나온다(1/30fps).
#   그러나 [전] 수치(1.5~3.0s)는 **추정**이다 — subprocess 기동 시간은 머신·디스크·
#   파이썬 import 시간에 달려 있어 실측해야 한다. 이 스크립트가 그 실측이다.
#   ⚠️ Ubuntu + ROS2 + 카메라가 있어야 돈다. Mac 에서는 못 돈다.
#
# 사용:
#   ./tools/measure_mode_switch.sh /image_angle 7 3
#     → /image_angle 을 보면서 wp_mode 를 7 ↔ 3 으로 번갈아 쏘고 공백을 잰다

set -euo pipefail

TOPIC="${1:-/image_angle}"
MODE_A="${2:-7}"
MODE_B="${3:-3}"
CYCLES="${4:-5}"
OUT="${TMPDIR:-/tmp}/mode_switch_$$.log"

echo "=== 모드 전환 실측 ==="
echo "  토픽    : $TOPIC"
echo "  모드    : $MODE_A <-> $MODE_B, ${CYCLES}회"
echo "  로그    : $OUT"
echo

if ! command -v ros2 >/dev/null 2>&1; then
  echo "❌ ros2 가 없다. Ubuntu 에서 실행할 것." >&2
  exit 1
fi

echo "[1] 사전 확인 — 발행자가 몇 개인가 (2개면 3-4 배선 오류)"
ros2 topic info "$TOPIC" || true
echo

echo "[2] 토픽 수신 시각 기록 시작"
# 각 메시지 수신 시각을 단조시계로 찍는다
( ros2 topic echo "$TOPIC" --no-arr 2>/dev/null \
    | grep --line-buffered -c '' \
    | while read -r _; do :; done ) &
ECHO_PID=$!

python3 - "$TOPIC" "$OUT" <<'PY' &
import subprocess, sys, time
topic, out = sys.argv[1], sys.argv[2]
p = subprocess.Popen(["ros2", "topic", "echo", topic, "--no-arr"],
                     stdout=subprocess.PIPE, text=True, bufsize=1)
with open(out, "w") as f:
    for line in p.stdout:
        if line.startswith("data:"):
            f.write(f"{time.monotonic():.6f}\n")
            f.flush()
PY
REC_PID=$!

sleep 2
echo "[3] 모드 전환 ${CYCLES}회"
for i in $(seq 1 "$CYCLES"); do
  for M in "$MODE_A" "$MODE_B"; do
    T=$(python3 -c 'import time; print(f"{time.monotonic():.6f}")')
    echo "  t=$T  wp_mode=$M"
    ros2 topic pub --once /wp_mode std_msgs/Int32 "{data: $M}" >/dev/null 2>&1
    sleep 3
  done
done

kill "$REC_PID" "$ECHO_PID" 2>/dev/null || true
wait 2>/dev/null || true

echo
echo "[4] 공백 분석"
python3 - "$OUT" <<'PY'
import sys
ts = [float(x) for x in open(sys.argv[1]) if x.strip()]
if len(ts) < 2:
    print("  ❌ 표본이 부족하다. 카메라·노드가 떠 있나?")
    sys.exit(1)
gaps = [b - a for a, b in zip(ts, ts[1:])]
gaps.sort()
n = len(gaps)
print(f"  메시지 {len(ts)}개, 간격 {n}개")
print(f"  중앙값 : {gaps[n//2]*1000:8.1f} ms   (정상 프레임 간격)")
print(f"  p95    : {gaps[int(n*0.95)]*1000:8.1f} ms")
print(f"  최대   : {gaps[-1]*1000:8.1f} ms   ← 전환 공백")
print()
big = [g for g in gaps if g > 0.2]
print(f"  200ms 초과 공백 : {len(big)}회")
if big:
    print(f"     최대 {max(big)*1000:.0f} ms")
    print("     ⚠️ 상주 구조에서 200ms 넘는 공백은 예상 밖이다.")
    print("        게이트가 색 콜백보다 늦게 갱신되는지, /wp_mode 주기가 느린지 확인할 것.")
else:
    print("     ✅ 200ms 초과 공백 없음 — 상주 전환이 의도대로 동작")
PY
