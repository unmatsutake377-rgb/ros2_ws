"""좌·우에 원하는 값을 일정 시간 고정 송출한다. 중립 탐색·수동 점검용.

사용: python3 tools/hold.py <좌us> <우us> [초] [시동대기초=25]
자율 모드여야 한다(워치독이 명령을 받아들이는 모드).
"""
import serial, sys, time

L = int(sys.argv[1]); R = int(sys.argv[2])
DUR = float(sys.argv[3]) if len(sys.argv) > 3 else 15.0

s = serial.Serial('/dev/ttyACM0', 115200, timeout=0)
time.sleep(3.5)
s.reset_input_buffer()

# 🚨 ESC 시동(arming) 대기 — 포트를 열면 보드가 재시작하므로 매번 필요하다.
#    이게 없어서 08-27 에 "1600 을 보냈는데 아무것도 안 돈다" 로 오진했다.
#    ESC 는 전원/신호가 살아난 뒤 중립을 15~25초 봐야 깨어난다(08-25 실측).
ARM = float(sys.argv[4]) if len(sys.argv) > 4 else 15.0   # 09-01 실측: 15초면 시동된다
print(f"▶ 중립 유지 {ARM:.0f}초 — ESC 시동 대기", flush=True)
_t = time.time(); _last = 0.0
while time.time() - _t < ARM:
    if time.time() - _last > 0.05:
        s.write(b"L1500,R1500\n"); _last = time.time()
    s.read(512)
    _e = int(time.time() - _t)
    if _e and _e % 5 == 0 and abs(time.time() - _t - _e) < 0.02:
        print(f"    {int(ARM) - _e}초 남음...", flush=True)
    time.sleep(0.01)
s.reset_input_buffer()

print(f"\n좌 {L} / 우 {R} 을 {DUR:.0f}초간 보냅니다.\n", flush=True)
t0 = time.time(); last = 0.0; buf = b''; seen = None; tick = 0
while time.time() - t0 < DUR:
    if time.time() - last > 0.05:
        s.write(f"L{L},R{R}\n".encode()); last = time.time()
    buf += s.read(512)
    while b'\n' in buf:
        line, buf = buf.split(b'\n', 1)
        line = line.decode('utf-8', 'replace').strip()
        if line.startswith('S,'):
            f = line.split(',')
            if len(f) >= 9: seen = f
    el = int(time.time() - t0)
    if el != tick:
        tick = el
        if seen and el % 3 == 0:
            print(f"  {el}초  실제 출력 {','.join(seen[5:9])}  (모드 {seen[1]}, 워치독 {seen[2]})", flush=True)
    time.sleep(0.01)
s.write(b"L1500,R1500\n"); s.flush()
s.close()
print("\n중립으로 복귀했습니다.")
