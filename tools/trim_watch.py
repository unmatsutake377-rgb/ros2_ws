"""조종기 트림 맞추기 — 스틱에서 손을 뗀 상태의 어긋남을 실시간으로 보여준다.

좌우 출력을 스로틀 성분과 조향 성분으로 분해한다:
    스로틀 = (좌 + 우) / 2 - 1500      전진(+) / 후진(-)
    조향   = (좌 - 우) / 2             우회전(+) / 좌회전(-)
트림 버튼은 두 축을 따로 움직이므로, 분해해서 봐야 어느 쪽을 눌러야 할지 안다.
"""
import serial, sys, time

DUR = int(sys.argv[1]) if len(sys.argv) > 1 else 90
DEAD = 25          # BESC30-R3 데드밴드 ±25us

s = serial.Serial('/dev/ttyACM0', 115200, timeout=0)
time.sleep(3.5)
s.reset_input_buffer()

print("스틱에서 손을 떼고, 트림 버튼을 눌러가며 0 에 맞추십시오.")
print(f"{DUR}초간 0.5초마다 찍습니다.  (데드밴드 ±{DEAD} 안이면 스러스터가 안 돕니다)")
print("🚨 반드시 **수동 모드**여야 합니다. 자율이면 워치독이 1500 을 물어 값이 안 보입니다.\n")
print(f"{'스로틀':>8} {'조향':>8}   {'ESC 출력':<24} 판정")
print("-" * 62)

buf = b''; t0 = time.time(); last = 0.0; cur = None
while time.time() - t0 < DUR:
    buf += s.read(512)
    while b'\n' in buf:
        line, buf = buf.split(b'\n', 1)
        line = line.decode('utf-8', 'replace').strip()
        if line.startswith('S,'):
            f = line.split(',')
            if len(f) >= 9:
                cur = f
    if cur and time.time() - last >= 0.5:
        last = time.time()
        L, R = int(cur[5]), int(cur[6])
        thr = (L + R) / 2 - 1500
        steer = (L - R) / 2
        ok_t = abs(thr) <= DEAD
        ok_s = abs(steer) <= DEAD
        if ok_t and ok_s:
            verdict = "✅ 맞음"
        else:
            parts = []
            if not ok_t: parts.append("스로틀 " + ("내리기" if thr > 0 else "올리기"))
            if not ok_s: parts.append("조향 " + ("왼쪽" if steer > 0 else "오른쪽"))
            verdict = "🚨 " + " · ".join(parts)
        print(f"{thr:+8.0f} {steer:+8.0f}   {','.join(cur[5:9]):<24} {verdict}", flush=True)
    time.sleep(0.01)
s.close()
if cur is None:
    print("🚨 수동 모드 상태줄이 하나도 없습니다 — 모드 스위치를 수동(위)으로 놓으십시오.")
print("\n관찰 종료")
